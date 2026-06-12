#pragma once
#include "sdk/vsm.hpp"

namespace details {
    const RELTIME DELAY = (RELTIME)1;
}

// GPIO <-> UART bridge model used by Proteus. The model exposes
// 8-bit parallel DI/DO buses and a single UART (TXD/RXD). Control
// signals follow Proteus conventions and are often active-low.
class GPIO_UART_Converter : public IDSIMMODEL {
private:
    // Simulation context
    IINSTANCE* m_instance;
    IDSIMCKT*  m_ckt;
    ABSTIME    m_currentTime;

    // Parallel buses: DI = device inputs, DO = device outputs
    IDSIMPIN2* m_DI[8];
    IDSIMPIN2* m_DO[8];

    // Control pins (note: many are active-low)
    IDSIMPIN2* m_CS;   // Chip Select (active low)
    IDSIMPIN2* m_WE;   // Write Enable (latch-on rising edge)
    IDSIMPIN2* m_RE;   // Read Enable (level sensitive)
    IDSIMPIN2* m_CD;   // Control/Data select (high = config)
    IDSIMPIN2* m_RST;  // Reset (active low)
    IDSIMPIN2* m_INT;  // Interrupt (active low)

    // UART pins
    IDSIMPIN2* m_UART_TXD;
    IDSIMPIN2* m_UART_RXD;

    // Buffers and flags
    BYTE m_txData;    // Byte pending transmission
    BYTE m_rxData;    // Last received byte
    BOOL m_rxReady;   // Indicates RX data available

    // UART configuration fields
    BOOL m_uartEnable;   // Global UART enable flag
    BOOL m_intEnable;    // RX interrupt enable
    BOOL m_txIntEnable;  // TX-complete interrupt enable
    BOOL m_txComplete;   // TX complete flag
    BYTE m_baudCode;     // Encoded baudrate selection
    BYTE m_parityMode;   // 0=None, 1=Odd, 2=Even
    BOOL m_doubleStop;   // FALSE = 1 stop bit, TRUE = 2 stop bits
    BOOL m_overwrite;    // RX overrun behavior: FALSE=overwrite, TRUE=discard
    BOOL m_clearOnRead;  // If TRUE, clear RX byte after MCU reads it
    ABSTIME m_bitTime;   // Duration of one UART bit (simulator time units)

    // Transmit FSM
    enum TX_STATE { TX_IDLE, TX_START, TX_DATA, TX_PARITY, TX_STOP };
    TX_STATE m_txState;
    BYTE m_txShiftReg;
    BYTE m_txBitCount;
    BOOL m_txBusy;
    BYTE m_txStopBitsLeft;

    // Receive FSM
    enum RX_STATE { RX_IDLE, RX_START_CHECK, RX_DATA, RX_PARITY_CHECK, RX_STOP_CHECK };
    RX_STATE m_rxState;
    BYTE m_rxShiftReg;
    BYTE m_rxBitCount;
    BOOL m_rxBusy;

    // Parallel bus helpers
    BYTE readDI();
    VOID writeDO(BYTE data);

    // UART helpers
    VOID startUART_TX();
    VOID handleUART_TX(ABSTIME time);
    VOID handleUART_RX(ABSTIME time);

    // Device utilities
    VOID resetDevice();
    VOID processConfig(BYTE addr, BYTE data);
    BYTE readConfig(BYTE addr);

public:
    static constexpr DWORD MODEL_KEY = 0x3A4B5C6D;

    GPIO_UART_Converter();
    virtual ~GPIO_UART_Converter();

    // IDSIMMODEL interface
    INT isdigital(CHAR* pinname) override;
    VOID setup(IINSTANCE* instance, IDSIMCKT* dsim) override;
    VOID runctrl(RUNMODES mode) override;
    VOID actuate(REALTIME time, ACTIVESTATE newstate) override;
    BOOL indicate(REALTIME time, ACTIVEDATA* newstate) override;
    VOID simulate(ABSTIME time, DSIMMODES mode) override;
    VOID callback(ABSTIME time, EVENTID eventid) override;
};