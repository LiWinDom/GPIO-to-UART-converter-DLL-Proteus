#include "pch.h"
#include "GPIO_UART_Converter.h"
#include <cstdio>
#include <algorithm>

GPIO_UART_Converter::GPIO_UART_Converter()
    : m_instance(nullptr), m_ckt(nullptr), m_CS(nullptr), m_WE(nullptr), m_RE(nullptr), m_CD(nullptr),
    m_RST(nullptr), m_INT(nullptr), m_UART_TXD(nullptr), m_UART_RXD(nullptr),
    m_txData(0), m_rxData(0), m_rxReady(FALSE), m_uartEnable(TRUE), m_intEnable(FALSE),
    m_baudCode(0), m_parityMode(0), m_doubleStop(FALSE), m_overwrite(FALSE),
    m_bitTime(1000000000000ULL / 9600), m_txState(TX_IDLE), m_txShiftReg(0), m_txBitCount(0), m_txBusy(FALSE),
    m_rxState(RX_IDLE), m_rxShiftReg(0), m_rxBitCount(0), m_rxBusy(FALSE),
    m_currentTime(0), m_txStopBitsLeft(1), m_clearOnRead(FALSE) {

    for (int i = 0; i < 8; i++) {
        m_DI[i] = nullptr;
        m_DO[i] = nullptr;
    }
}

GPIO_UART_Converter::~GPIO_UART_Converter() {}

INT GPIO_UART_Converter::isdigital(CHAR* pinname) {
    return TRUE;
}

VOID GPIO_UART_Converter::setup(IINSTANCE* instance, IDSIMCKT* dsim) {
    m_instance = instance;
    m_ckt = dsim;

    CHAR name[5];
    for (int i = 0; i < 8; i++) {
        sprintf_s(name, "DI%d", i);
        m_DI[i] = (IDSIMPIN2*)instance->getdsimpin(name, TRUE);
        sprintf_s(name, "DO%d", i);
        m_DO[i] = (IDSIMPIN2*)instance->getdsimpin(name, TRUE);
        if (m_DO[i]) {
            m_DO[i]->settiming(details::DELAY, details::DELAY, details::DELAY);
            m_DO[i]->setstates(SHI, SLO, FLT);
            m_DO[i]->setstate(FLT);
        }
    }

    char CS[] = "$CS$";
    char WE[] = "$WE$";
    char RE[] = "$RE$";
    char CD[] = "C/$D$"; // Pin name used in the Proteus symbol (MODE)
    char RST[] = "$RST$";
    char INT[] = "$INT$";
    m_CS = (IDSIMPIN2*)instance->getdsimpin(CS, TRUE);
    m_WE = (IDSIMPIN2*)instance->getdsimpin(WE, TRUE);
    m_RE = (IDSIMPIN2*)instance->getdsimpin(RE, TRUE);
    m_CD = (IDSIMPIN2*)instance->getdsimpin(CD, TRUE);
    m_RST = (IDSIMPIN2*)instance->getdsimpin(RST, TRUE);
    m_INT = (IDSIMPIN2*)instance->getdsimpin(INT, TRUE);

    CHAR uart_txd[] = "UART_TXD";
    CHAR uart_rxd[] = "UART_RXD";
    m_UART_TXD = (IDSIMPIN2*)instance->getdsimpin(uart_txd, TRUE);
    m_UART_RXD = (IDSIMPIN2*)instance->getdsimpin(uart_rxd, TRUE);

    if (m_UART_TXD) {
        m_UART_TXD->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_UART_TXD->setstates(SHI, SLO, FLT);
        m_UART_TXD->setstate(SHI); // Idle UART level = logical 1
    }
    if (m_INT) {
        m_INT->settiming(details::DELAY, details::DELAY, details::DELAY);
        m_INT->setstates(SHI, SLO, FLT);
        m_INT->setstate(SHI); // Interrupt idle (inactive) = logical 1
    }

    resetDevice();

    CHAR msg[] = "GPIO-to-UART BRIDGE (CS-ONLY CONTROL) READY";
    m_instance->log(msg);
}

VOID GPIO_UART_Converter::runctrl(RUNMODES mode) {}
VOID GPIO_UART_Converter::actuate(REALTIME time, ACTIVESTATE newstate) {}
BOOL GPIO_UART_Converter::indicate(REALTIME time, ACTIVEDATA* newstate) { return FALSE; }

VOID GPIO_UART_Converter::simulate(ABSTIME time, DSIMMODES mode) {
    m_currentTime = time;

    if (!m_CS || !m_WE || !m_RE || !m_CD || !m_RST) return;

    // 1) Hardware reset (active-low RST)
    if (islow(m_RST->istate())) {
        resetDevice();
        return;
    }

    // 2) Chip select handling: when CS is high (inactive) release DO bus
    STATE cs = m_CS->istate();
    if (ishigh(cs)) {
        for (int i = 0; i < 8; i++) {
            if (m_DO[i] && m_DO[i]->istate() != FLT) {
                m_DO[i]->setstate(time, details::DELAY, FLT);
            }
        }
        return;
    }
    else {
        // If CS is asserted (low) and RE is active, restore DO outputs
        if (islow(m_RE->istate()) && m_DO[0] && m_DO[0]->istate() == FLT) {
            writeDO(m_rxData);
        }
    }

    // 3) Start asynchronous RX sampling on UART RX falling edge (start bit)
    if (m_UART_RXD && m_UART_RXD->isnegedge() && !m_rxBusy && m_uartEnable) {
        m_rxBusy = TRUE;
        m_rxState = RX_START_CHECK;
        m_rxBitCount = 0;
        m_rxShiftReg = 0;
        m_ckt->setcallback(time + (m_bitTime / 2), this, 2); // Event 2 = RX handler
    }

    // 4) Parallel read handling (active-low RE)
    if (islow(m_RE->istate())) {
        if (ishigh(m_CD->istate())) {
            // Accessing configuration registers
            BYTE input = readDI();
            BYTE addr = input & 0x0F;

            if (addr == 0x0F) { // Address 0x0F = status register
                BYTE status = (m_rxReady ? 0x01 : 0) | (m_txBusy ? 0x02 : 0) |
                    (m_rxBusy ? 0x04 : 0) | (m_txComplete ? 0x08 : 0);
                writeDO(status);

                // Clear TX-complete flag on status read edge
                if (m_RE->isnegedge()) m_txComplete = FALSE;
            }
            else {
                BYTE cfgData = readConfig(addr);
                writeDO(cfgData);
            }
        }
        else {
            // CD = 0, RE = 0 -> MCU reading last received UART byte
            writeDO(m_rxData);

            // Clear RX-ready flag when MCU starts reading data
            if (m_RE->isnegedge()) m_rxReady = FALSE;
        }
    }
    else {
        // If RE is inactive, drive DO bus to high-impedance
        for (int i = 0; i < 8; i++) {
            if (m_DO[i] && m_DO[i]->istate() != FLT) {
                m_DO[i]->setstate(time, details::DELAY, FLT);
            }
        }
        // Optionally clear RX data after a read (if configured)
        if (m_RE->isposedge() && islow(m_CD->istate())) {
            if (m_clearOnRead) m_rxData = 0;
        }
    }

    // 5) Parallel write handling (WE latches on rising edge)
    if (m_WE->isposedge()) {
        BYTE input = readDI();
        if (ishigh(m_CD->istate())) {
            BYTE addr = input & 0x0F;
            BYTE data = (input >> 4) & 0x0F;
            processConfig(addr, data);
        }
        else {
            // Write to TX buffer and start UART transmit if enabled
            if (!m_txBusy && m_uartEnable) {
                m_txData = input;
                m_txComplete = FALSE; // Clear TX-complete on new TX
                startUART_TX();
            }
        }
    }

    // 6) Update hardware interrupt output (active-low)
    if (m_INT) {
        BOOL intActive = (m_rxReady && m_intEnable) || (m_txComplete && m_txIntEnable);
        m_INT->setstate(m_currentTime, details::DELAY, intActive ? SLO : SHI);
    }
}

VOID GPIO_UART_Converter::callback(ABSTIME time, EVENTID eventid) {
    if (eventid == 1) {
        handleUART_TX(time);
    }
    else if (eventid == 2) {
        handleUART_RX(time);
    }
}

VOID GPIO_UART_Converter::resetDevice() {
    CHAR msg[] = "RESET DEVICE (ALL REGISTERS = 0)";
    m_instance->log(msg);
    // Reset all configuration to defaults (UART and interrupts disabled)
    m_uartEnable = FALSE;
    m_intEnable = FALSE;
    m_txIntEnable = FALSE; // TX interrupt disabled by default
    m_txComplete = FALSE;  // No completed transmissions
    m_baudCode = 0;        // Default to lowest code -> 300 baud
    m_bitTime = 1000000000000ULL / 300;
    m_parityMode = 0;      // No parity
    m_doubleStop = FALSE;  // Single stop bit
    m_overwrite = FALSE;   // Overwrite on RX overrun
    m_clearOnRead = FALSE; // Do not clear RX data on read by default

    // Reset FSM states and buffers
    m_txState = TX_IDLE;
    m_txBusy = FALSE;
    m_txData = 0;

    m_rxState = RX_IDLE;
    m_rxBusy = FALSE;
    m_rxData = 0;
    m_rxReady = FALSE;

    if (m_UART_TXD) m_UART_TXD->setstate(m_currentTime, details::DELAY, SHI);
    if (m_INT) m_INT->setstate(m_currentTime, details::DELAY, SHI); // Interrupt idle = 1
    writeDO(0);
}

VOID GPIO_UART_Converter::processConfig(BYTE addr, BYTE data) {
    CHAR buf[64];
    sprintf_s(buf, "processConfig: addr=%d data=%d", addr, data);
    m_instance->log(buf);
    switch (addr) {
    case 0x00:
        // Control register: bit0 = UART enable, bit1 = RX interrupt enable,
        // bit2 = TX complete interrupt enable
        m_uartEnable = data & 0x01;
        m_intEnable = (data >> 1) & 0x01;
        m_txIntEnable = (data >> 2) & 0x01;
        break;
    case 0x01:
        m_baudCode = data & 0x0F;
        uint32_t baud;
        switch (m_baudCode) {
        case 0: baud = 300;     break;
        case 1: baud = 600;     break;
        case 2: baud = 1200;    break;
        case 3: baud = 2400;    break;
        case 4: baud = 4800;    break;
        case 5: baud = 9600;    break;
        case 6: baud = 19200;   break;
        case 7: baud = 38400;   break;
        case 8: baud = 57600;   break;
        case 9: baud = 115200;  break;
        case 10: baud = 230400; break;
        case 11: baud = 460800; break;
        case 12: baud = 921600; break;
        default: baud = 9600;
        }
        m_bitTime = 1000000000000ULL / baud;
        break;
    case 0x02:
        m_parityMode = data & 0x03; // 0=None, 1=Odd, 2=Even
        break;
    case 0x03:
        m_doubleStop = data & 0x01; // 0=1 stop bit, 1=2 stop bits
        break;
    case 0x04:
        m_overwrite = data & 0x01; // 0=overwrite, 1=discard
        break;
    case 0x05:
        // If set, clear the RX byte after it's read by the MCU
        m_clearOnRead = data & 0x01;
        break;
    }
}

BYTE GPIO_UART_Converter::readConfig(BYTE addr) {
    switch (addr) {
    case 0x00:
        // Return packed control bits: bit0=UART enable, bit1=RX int, bit2=TX int
        return (m_uartEnable ? 0x01 : 0) | (m_intEnable ? 0x02 : 0) | (m_txIntEnable ? 0x04 : 0);
    case 0x01:
        return m_baudCode;
    case 0x02:
        return m_parityMode;
    case 0x03:
        return m_doubleStop ? 0x01 : 0;
    case 0x04:
        return m_overwrite ? 0x01 : 0;
    case 0x05:
        return m_clearOnRead ? 0x01 : 0;
    default:
        return 0;
    }
}

BYTE GPIO_UART_Converter::readDI() {
    BYTE data = 0;
    for (int i = 0; i < 8; i++) {
        if (m_DI[i] && ishigh(m_DI[i]->istate())) data |= (1 << (7 - i));
    }
    return data;
}

VOID GPIO_UART_Converter::writeDO(BYTE data) {
    for (int i = 0; i < 8; i++) {
        if (m_DO[i]) m_DO[i]->setstate(m_currentTime, details::DELAY, (data & (1 << (7 - i))) ? SHI : SLO);
    }
}

// Transmitter (TX)
VOID GPIO_UART_Converter::startUART_TX() {
    m_txShiftReg = m_txData;
    m_txBusy = TRUE;
    m_txState = TX_START;
    m_txBitCount = 0;

    CHAR buf[64];
    sprintf_s(buf, "UART TX Start: Byte=0x%02X", m_txData);
    m_instance->log(buf);

    handleUART_TX(m_currentTime);
}

VOID GPIO_UART_Converter::handleUART_TX(ABSTIME time) {
    if (!m_UART_TXD || !m_txBusy) return;

    switch (m_txState) {
    case TX_START:
        m_UART_TXD->setstate(time, details::DELAY, SLO);
        m_txState = TX_DATA;
        m_txBitCount = 0;
        m_ckt->setcallback(time + m_bitTime, this, 1); // Event 1 = TX
        break;

    case TX_DATA: {
        BYTE bit = (m_txShiftReg >> m_txBitCount) & 0x01;
        m_UART_TXD->setstate(time, details::DELAY, bit ? SHI : SLO);
        m_txBitCount++;

        if (m_txBitCount >= 8) {
            m_txState = (m_parityMode != 0) ? TX_PARITY : TX_STOP;
            m_txStopBitsLeft = m_doubleStop ? 2 : 1;
        }
        m_ckt->setcallback(time + m_bitTime, this, 1);
        break;
    }

    case TX_PARITY: {
        BYTE ones = 0;
        for (int i = 0; i < 8; i++) {
            if ((m_txShiftReg >> i) & 1) ones++;
        }
        BYTE parityBit = 0;
        if (m_parityMode == 1)      parityBit = (ones % 2 == 0) ? 1 : 0;
        else if (m_parityMode == 2) parityBit = (ones % 2 != 0) ? 1 : 0;

        m_UART_TXD->setstate(time, details::DELAY, parityBit ? SHI : SLO);
        m_txState = TX_STOP;
        m_txStopBitsLeft = m_doubleStop ? 2 : 1;
        m_ckt->setcallback(time + m_bitTime, this, 1);
        break;
    }

    case TX_STOP:
        m_UART_TXD->setstate(time, details::DELAY, SHI);
        if (m_txStopBitsLeft > 0) {
            m_txStopBitsLeft--;
            m_ckt->setcallback(time + m_bitTime, this, 1);
        }
        else {
            m_txBusy = FALSE;
            m_txState = TX_IDLE;
            m_txComplete = TRUE; // Indicate successful transmission

            // Assert INT (active-low) immediately when TX-complete interrupt is enabled
            if (m_txIntEnable && m_INT) {
                m_INT->setstate(time, details::DELAY, SLO);
            }

            m_instance->log((char*)"UART TX Byte Sent Successfully.");
        }
        break;

    default:
        m_txBusy = FALSE;
        break;
    }
}

// Receiver (RX)
VOID GPIO_UART_Converter::handleUART_RX(ABSTIME time) {
    if (!m_UART_RXD || !m_rxBusy) return;

    switch (m_rxState) {
    case RX_START_CHECK:
        if (islow(m_UART_RXD->istate())) {
            m_rxState = RX_DATA;
            m_rxBitCount = 0;
            m_rxShiftReg = 0;
            m_ckt->setcallback(time + m_bitTime, this, 2); // Event 2 = RX
        }
        else {
            m_rxBusy = FALSE;
        }
        break;

    case RX_DATA: {
        BOOL bit = ishigh(m_UART_RXD->istate());
        if (bit) m_rxShiftReg |= (1 << m_rxBitCount);
        m_rxBitCount++;

        if (m_rxBitCount >= 8) {
            m_rxState = (m_parityMode != 0) ? RX_PARITY_CHECK : RX_STOP_CHECK;
        }
        m_ckt->setcallback(time + m_bitTime, this, 2);
        break;
    }

    case RX_PARITY_CHECK: {
        // Read parity bit from the line
        BOOL bit = ishigh(m_UART_RXD->istate());
        BYTE ones = 0;
        for (int i = 0; i < 8; i++) {
            if ((m_rxShiftReg >> i) & 1) ones++;
        }

        BOOL expectedParity = 0;
        // parityMode: 1 = Odd, 2 = Even
        if (m_parityMode == 1)      expectedParity = (ones % 2 == 0) ? 1 : 0; // Odd
        else if (m_parityMode == 2) expectedParity = (ones % 2 != 0) ? 1 : 0; // Even

        if (bit != expectedParity) {
            m_instance->log((char*)"UART RX Parity Error! Data corrupted.");
        }
        else {
            m_instance->log((char*)"UART RX Parity OK.");
        }

        m_rxState = RX_STOP_CHECK;
        m_ckt->setcallback(time + m_bitTime, this, 2);
        break;
    }

    case RX_STOP_CHECK:
        if (ishigh(m_UART_RXD->istate())) {
            if (m_rxReady) {
                if (m_overwrite) {
                    m_instance->log((char*)"UART RX Overrun! New byte ignored (Discard mode)");
                }
                else {
                    m_rxData = m_rxShiftReg;
                    m_instance->log((char*)"UART RX Overrun! Old byte overwritten");
                }
            }
            else {
                m_rxData = m_rxShiftReg;
                m_rxReady = TRUE;
            }
            m_instance->log((char*)"UART Byte Received Successfully.");
        }
        else {
            m_instance->log((char*)"UART Framing Error! (Invalid Stop Bit)");
        }
        m_rxBusy = FALSE;
        m_rxState = RX_IDLE;
        break;

    default:
        m_rxBusy = FALSE;
        break;
    }
}

extern "C" {
    __declspec(dllexport) IDSIMMODEL* createdsimmodel(CHAR* device, ILICENCESERVER* ils) {
        return new GPIO_UART_Converter();
    }
    __declspec(dllexport) VOID deletedsimmodel(IDSIMMODEL* model) {
        delete (GPIO_UART_Converter*)model;
    }
}