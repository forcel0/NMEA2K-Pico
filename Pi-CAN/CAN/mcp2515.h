#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "can.h"

#include "hardware/spi.h"
#include "hardware/sync.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "boards/pico.h"

#include "../../NMEA/src/NMEA2000.h"

#define MCPDEBUG        (0)
#define MCPDEBUG_TXBUF  (0)

class MCP2515 {
    public:
        // Return Values
        enum RETURN {
            RETURN_OK        = 0,
            RETURN_FAIL      = 1,
            RETURN_ALLTXBUSY = 2,
            RETURN_FAILINIT  = 3,
            RETURN_FAILTX    = 4,
            RETURN_MSGAVAIL  = 5,
            RETURN_NOMSG     = 6
        };

        enum STAT : uint8_t {
            // Transmit
            STAT_TX2IF       = (1 << 7),
            STAT_TX2_PENDING = (1 << 6),
            STAT_TX1IF       = (1 << 5),
            STAT_TX1_PENDING = (1 << 4),
            STAT_TX0IF       = (1 << 3),
            STAT_TX0_PENDING = (1 << 2),

            STAT_TXIF_MASK   = STAT_TX0IF | STAT_TX1IF | STAT_TX2IF,
            STAT_TX_PENDING_MASK = STAT_TX0_PENDING | STAT_TX1_PENDING | STAT_TX2_PENDING,
            // Reciveve
            STAT_RX1IF = (1 << 1),
            STAT_RX0IF = (1 << 0),
 
            STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF
        };

        static const uint32_t DEFAULT_SPI_CLOCK = 10000000; // 10MHz

        enum CAN_CLOCK { CAN_20MHZ, CAN_16MHZ, CAN_8MHZ };
        
        enum CAN_SPEED { CAN_1000KBPS, CAN_500KBPS, CAN_250KBPS, CAN_200KBPS,
                         CAN_125KBPS, CAN_100KBPS, CAN_95KBPS, CAN_83K3BPS,
                         CAN_80KBPS, CAN_50KBPS, CAN_40KBPS, CAN_33KBPS, 
                         CAN_31K25BPS, CAN_20KBPS, CAN_10KBPS, CAN_5KBPS };

        static const uint32_t  CAN_MAX_CHAR_IN_MESSAGE = 8;

        enum TXBn {
            TXB0 = 0,
            TXB1 = 1,
            TXB2 = 2
        };

        enum RXBn {
            RXB0 = 0,
            RXB1 = 1
        };

        enum MASK {
            MASK0,
            MASK1
        };

        enum RXF {
            RXF0 = 0,
            RXF1 = 1,
            RXF2 = 2,
            RXF3 = 3,
            RXF4 = 4,
            RXF5 = 5
        };


private:
////////////////////////////////////////////////////////////////////////
// Data structures are defined in the Microchip MCP2515 specification //
//          Stand-Alone CAN Controller with SPI Interface             //
////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////
//  SPI INSTRUCTION SET      -ref. MCP2515 Table 12-1:
        enum /*class*/ INSTRUCTION : uint8_t {
            MCP_WRITE       = 0x02,
            MCP_READ        = 0x03,
            MCP_BITMOD      = 0x05,
            MCP_LOAD_TX0    = 0x40,
            MCP_LOAD_TX1    = 0x42,
            MCP_LOAD_TX2    = 0x44,
            MCP_RTS_TX0     = 0x81,
            MCP_RTS_TX1     = 0x82,
            MCP_RTS_TX2     = 0x84,
            MCP_RTS_ALL     = 0x87,
            MCP_READ_RX0    = 0x90,
            MCP_READ_RX1    = 0x94,
            MCP_READ_STATUS = 0xA0,
            MCP_RX_STATUS   = 0xB0,
            MCP_RESET       = 0xC0
        };

/////////////////////////////////////////
//  Control Register Map      -ref. MCP2515 Table 11-1:
        enum REGISTER : uint8_t {
/*low byte       high byte  0x00                 0x10                 0x20                 0x30                 0x40                 0x50                 0x60                 0x70 */            
/* 0x00 */  MCP_RXF0SIDH  = 0x00, MCP_RXF3SIDH = 0x10, MCP_RXM0SIDH = 0x20, MCP_TXB0CTRL = 0x30, MCP_TXB1CTRL = 0x40, MCP_TXB2CTRL = 0x50, MCP_RXB0CTRL = 0x60, MCP_RXB1CTRL = 0x70,
/* 0x01 */  MCP_RXF0SIDL  = 0x01, MCP_RXF3SIDL = 0x11, MCP_RXM0SIDL = 0x21, MCP_TXB0SIDH = 0x31, MCP_TXB1SIDH = 0x41, MCP_TXB2SIDH = 0x51, MCP_RXB0SIDH = 0x61, MCP_RXB1SIDH = 0x71,       
/* 0x02 */  MCP_RXF0EID8  = 0x02, MCP_RXF3EID8 = 0x12, MCP_RXM0EID8 = 0x22, MCP_TXB0SIDL = 0x32, MCP_TXB1SIDL = 0x42, MCP_TXB2SIDL = 0x52, MCP_RXB0SIDL = 0x62, MCP_RXB1SIDL = 0x72,
/* 0x03 */  MCP_RXF0EID0  = 0x03, MCP_RXF3EID0 = 0x13, MCP_RXM0EID0 = 0x23, MCP_TXB0EID8 = 0x33, MCP_TXB1EID8 = 0x43, MCP_TXB2EID8 = 0x53, MCP_RXB0EID8 = 0x63, MCP_RXB1EID8 = 0x73,
/* 0x04 */  MCP_RXF1SIDH  = 0x04, MCP_RXF4SIDH = 0x14, MCP_RXM1SIDH = 0x24, MCP_TXB0EID0 = 0x34, MCP_TXB1EID0 = 0x44, MCP_TXB2EID0 = 0x54, MCP_RXB0EID0 = 0x64, MCP_RXB1EID0 = 0x74,
/* 0x05 */  MCP_RXF1SIDL  = 0x05, MCP_RXF4SIDL = 0x15, MCP_RXM1SIDL = 0x25, MCP_TXB0DLC  = 0x35, MCP_TXB1DLC  = 0x45, MCP_TXB2DLC  = 0x55, MCP_RXB0DLC  = 0x65, MCP_RXB1DLC  = 0x75,
/* 0x06 */  MCP_RXF1EID8  = 0x06, MCP_RXF4EID8 = 0x16, MCP_RXM1EID8 = 0x26, MCP_TXB0D0   = 0x36, MCP_TXB1D0   = 0x46, MCP_TXB2D0   = 0x56, MCP_RXB0D0   = 0x66, MCP_RXB1D0   = 0x76,
/* 0x07 */  MCP_RXF1EID0  = 0x07, MCP_RXF4EID0 = 0x17, MCP_RXM1EID0 = 0x27, MCP_TXB0D1   = 0x37, MCP_TXB1D1   = 0x47, MCP_TXB2D1   = 0x57, MCP_RXB0D1   = 0x67, MCP_RXB1D1   = 0x77,
/* 0x08 */  MCP_RXF2SIDH  = 0x08, MCP_RXF5SIDH = 0x18, MCP_CNF3     = 0x28, MCP_TXB0D2   = 0x38, MCP_TXB1D2   = 0x48, MCP_TXB2D2   = 0x58, MCP_RXB0D2   = 0x68, MCP_RXB1D2   = 0x78,
/* 0x09 */  MCP_RXF2SIDL  = 0x09, MCP_RXF5SIDL = 0x19, MCP_CNF2     = 0x29, MCP_TXB0D3   = 0x39, MCP_TXB1D3   = 0x49, MCP_TXB2D3   = 0x59, MCP_RXB0D3   = 0x69, MCP_RXB1D3   = 0x79,
/* 0x0A */  MCP_RXF2EID8  = 0x0A, MCP_RXF5EID8 = 0x1A, MCP_CNF1     = 0x2A, MCP_TXB0D4   = 0x3A, MCP_TXB1D4   = 0x4A, MCP_TXB2D4   = 0x5A, MCP_RXB0D4   = 0x6A, MCP_RXB1D4   = 0x7A,
/* 0x0B */  MCP_RXF2EID0  = 0x0B, MCP_RXF5EID0 = 0x1B, MCP_CANINTE  = 0x2B, MCP_TXB0D5   = 0x3B, MCP_TXB1D5   = 0x4B, MCP_TXB2D5   = 0x5B, MCP_RXB0D5   = 0x6B, MCP_RXB1D5   = 0x7B,
/* 0x0C */  MCP_BFPCTRL   = 0x0C, MCP_TEC      = 0x1C, MCP_CANINTF  = 0x2C, MCP_TXB0D6   = 0x3C, MCP_TXB1D6   = 0x4C, MCP_TXB2D6   = 0x5C, MCP_RXB0D6   = 0x6C, MCP_RXB1D6   = 0x7C,
/* 0x0D */  MCP_TXRTSCTRL = 0x0D, MCP_REC      = 0x1D, MCP_EFLG     = 0x2D, MCP_TXB0D7   = 0x3D, MCP_TXB1D7   = 0x4D, MCP_TXB2D7   = 0x5D, MCP_RXB0D7   = 0x6D, MCP_RXB1D7   = 0x7D,
/* 0x0E */  MCP_CANSTAT1  = 0x0E, MCP_CANSTAT2 = 0x1E, MCP_CANSTAT3 = 0x2E, MCP_CANSTAT4 = 0x3E, MCP_CANSTAT5 = 0x4E, MCP_CANSTAT6 = 0x5E, MCP_CANSTAT7 = 0x6E, MCP_CANSTAT8 = 0x7E,
/* 0x0F */  MCP_CANCTRL1  = 0x0F, MCP_CANCTRL2 = 0x1F, MCP_CANCTRL3 = 0x2F, MCP_CANCTRL4 = 0x3F, MCP_CANCTRL5 = 0x4F, MCP_CANCTRL6 = 0x5F, MCP_CANCTRL7 = 0x6F, MCP_CANCTRL8 = 0x7F,
        };

/////////////////////////////////////////
//  Control Register Summary/Data      -ref. MCP2515 Table 11-2:

        // BFPCTRL: RXnBF PIN CONTROL AND STATUS REGISTER DATA @ Reg:0x0C - MCP2515 REGISTER 4-3: 
        //  can be handled by the MicroController Unit (MCU) SPI config so not used

        // TXRTSCTRL: TXnRTS PIN CONTROL AND STATUS REGISTER DATA @ Reg:0x0D - MCP2515 REGISTER 3-2: 
        //  Not implemented in code defaults work with circiut design
        
        // CANSTAT: CAN STATUS REGISTER DATA @ Reg:0x0E - MCP2515 REGISTER 10-2:
        enum CANSTAT : uint8_t {
            CANSTAT_OPMOD2 = (1 << 7), // 0x80
            CANSTAT_OPMOD1 = (1 << 6), // 0x40
            CANSTAT_OPMOD0 = (1 << 5), // 0x20
            CANSTAT_ICOD2  = (1 << 3), // 0x08
            CANSTAT_ICOD1  = (1 << 2), // 0x04
            CANSTAT_ICOD0  = (1 << 1), // 0x02

            CANSTAT_OPMOD  = (CANSTAT_OPMOD0 | CANSTAT_OPMOD1 | CANSTAT_OPMOD2),
            CANSTAT_ICOD   = (CANSTAT_ICOD0 | CANSTAT_ICOD1 | CANSTAT_ICOD2)
        };

        // CANCTRL: CAN CONTROL REGISTER DATA @ Reg:0x0F - MCP2515 REGISTER 10-1:
        enum CANCTRL : uint8_t {
            CANCTRL_REQOP2  = (1 << 7), // 0x80
            CANCTRL_REQOP1  = (1 << 6), // 0x40
            CANCTRL_REQOP0  = (1 << 5), // 0x20
            CANCTRL_ABAT    = (1 << 4), // 0x10
            CANCTRL_OSM     = (1 << 3), // 0x08
            CANCTRL_CLKEN   = (1 << 2), // 0x04
            CANCTRL_CLKPRE1 = (1 << 1), // 0x02
            CANCTRL_CLKPRE0 = (1 << 0), // 0x01

            CANCTRL_REQOP   = (CANCTRL_REQOP0 | CANCTRL_REQOP1 | CANCTRL_REQOP2),
            CANCTRL_CLKPRE  = (CANCTRL_CLKPRE0 | CANCTRL_CLKPRE1)
        };

            // REQOP - Modes of Operation -ref. MCP2515 10.0
            enum CANCTRL_REQOP_MODE : uint8_t {
                CANCTRL_REQOP_POWERUP    = 0xE0,
                CANCTRL_REQOP_CONFIG     = 0x80,
                CANCTRL_REQOP_LISTENONLY = 0x60,
                CANCTRL_REQOP_LOOPBACK   = 0x40,
                CANCTRL_REQOP_SLEEP      = 0x20,
                CANCTRL_REQOP_NORMAL     = 0x00
            };

            // CLKEN & CLKPRE0/1 - Clock Out Pin Prescaler bits
            enum CAN_CLKOUT : uint8_t {
//                CLKOUT_DISABLE = 0xFF,  // CLKEN is a single bit but if set to -1 bit will be disable
                CLKOUT_DIV8 = (CANCTRL_CLKPRE1 | CANCTRL_CLKPRE0),
                CLKOUT_DIV4 = CANCTRL_CLKPRE1,
                CLKOUT_DIV2 = CANCTRL_CLKPRE0,
                CLKOUT_DIV1 = (0x00)
            };

        // TEC: TRANSMIT ERROR COUNTER REGISTER DATA @ Reg:0x1C - MCP2515 REGISTER 6-1: - will retrun uint8_t error count

        // REC: RECEIVE ERROR COUNTER REGISTER DATA @ Reg:0x1D - MCP2515 REGISTER 6-2: - will retrun uint8_t error count

        //////////////////////////
        // MCP2515 5.0 BIT TIMING

        // CNF3: CONFIGURATION REGISTER 3 DATA @ Reg:0x28 - MCP2515 REGISTER 5-3: 
        enum CNF3 : uint8_t {
                    CNF3_SOF    = (1 << 7), // Start-of-Frame (SOF) Signal
                    CNF3_WAKFIL = (1 << 6), // Wake-up Filter bit
    /* Baud */  /*       8 MHz                        16 MHz                        20 MHz     Oscillator*/
    /* 1000 Kbps */ CNF3_8MHz_1000kBPS = (0x80), CNF3_16MHz_1000kBPS = (0x82), CNF3_20MHz_1000kBPS = (0x82),
    /*  500 Kbp s*/ CNF3_8MHz_500kBPS  = (0x82), CNF3_16MHz_500kBPS  = (0x86), CNF3_20MHz_500kBPS  = (0x87),
    /*  250 Kbps */ CNF3_8MHz_250kBPS  = (0x85), CNF3_16MHz_250kBPS  = (0x85), CNF3_20MHz_250kBPS  = (0x86),
    /*  200 Kbps */ CNF3_8MHz_200kBPS  = (0x86), CNF3_16MHz_200kBPS  = (0x87), CNF3_20MHz_200kBPS  = (0x87),
    /*  125 Kbps */ CNF3_8MHz_125kBPS  = (0x85), CNF3_16MHz_125kBPS  = (0x86), CNF3_20MHz_125kBPS  = (0x87),
    /*  100 Kbps */ CNF3_8MHz_100kBPS  = (0x86), CNF3_16MHz_100kBPS  = (0x87), CNF3_20MHz_100kBPS  = (0x87),
    /* 83.33 Kbps*/                              CNF3_16MHz_83k3BPS  = (0x07), CNF3_20MHz_83k3BPS  = (0x87),
    /*   80 Kbps */ CNF3_8MHz_80kBPS   = (0x87), CNF3_16MHz_80kBPS   = (0x87), CNF3_20MHz_80kBPS  = (0x87),
    /*   50 Kbps */ CNF3_8MHz_50kBPS   = (0x86), CNF3_16MHz_50kBPS   = (0x87), CNF3_20MHz_50kBPS  = (0x87),
    /*   40 Kbps */ CNF3_8MHz_40kBPS   = (0x87), CNF3_16MHz_40kBPS   = (0x87), CNF3_20MHz_40kBPS  = (0x87),
    /*   33 Kbps */ CNF3_8MHz_33k3BPS  = (0x85), CNF3_16MHz_33k3BPS  = (0x85), CNF3_20MHz_33k3BPS = (0x87),
    /* 31.25 Kbps*/ CNF3_8MHz_31k25BPS = (0x84), 
    /*   20 Kbps */ CNF3_8MHz_20kBPS   = (0x87), CNF3_16MHz_20kBPS    = (0x87),
    /*   10 Kbps */ CNF3_8MHz_10kBPS   = (0x87),CNF3_16MHz_10kBPS    = (0x87),
    /*    5 Kbps */ CNF3_8MHz_5kBPS    = (0x87),CNF3_16MHz_5kBPS     = (0x87),
        };
  
        // CNF2: CONFIGURATION REGISTER 2 DATA @ Reg:0x29 - MCP2515 REGISTER 5-2:
        enum CNF2 : uint8_t {
    /* Baud */  /*       8 MHz                        16 MHz                        20 MHz     Oscillator*/
    /* 1000 Kbps */ CNF2_8MHz_1000kBPS = (0x80), CNF2_16MHz_1000kBPS = (0xD0), CNF2_20MHz_1000kBPS = (0xD9),
    /*  500 Kbp s*/ CNF2_8MHz_500kBPS  = (0x90), CNF2_16MHz_500kBPS  = (0xF0), CNF2_20MHz_500kBPS  = (0xFA),
    /*  250 Kbps */ CNF2_8MHz_250kBPS  = (0xB1), CNF2_16MHz_250kBPS  = (0xF1), CNF2_20MHz_250kBPS  = (0xFB),
    /*  200 Kbps */ CNF2_8MHz_200kBPS  = (0xB4), CNF2_16MHz_200kBPS  = (0xFA), CNF2_20MHz_200kBPS  = (0xFF),
    /*  125 Kbps */ CNF2_8MHz_125kBPS  = (0xB1), CNF2_16MHz_125kBPS  = (0xF0), CNF2_20MHz_125kBPS  = (0xFA),
    /*  100 Kbps */ CNF2_8MHz_100kBPS  = (0xB4), CNF2_16MHz_100kBPS  = (0xFA), CNF2_20MHz_100kBPS  = (0xFA),
    /* 83.33 Kbps*/                              CNF2_16MHz_83k3BPS  = (0xBE), CNF2_20MHz_83k3BPS  = (0xFE),
    /*   80 Kbps */ CNF2_8MHz_80kBPS   = (0xBF), CNF2_16MHz_80kBPS   = (0xFF), CNF2_20MHz_80kBPS   = (0xFF),
    /*   50 Kbps */ CNF2_8MHz_50kBPS   = (0xB4), CNF2_16MHz_50kBPS   = (0xFA), CNF2_20MHz_50kBPS   = (0xFA),
    /*   40 Kbps */ CNF2_8MHz_40kBPS   = (0xBF), CNF2_16MHz_40kBPS   = (0xFF), CNF2_20MHz_40kBPS   = (0xFF),
    /*   33 Kbps */ CNF2_8MHz_33k3BPS  = (0xE2), CNF2_16MHz_33k3BPS  = (0xF1), CNF2_20MHz_33k3BPS  = (0xFF),
    /* 31.25 Kbps*/ CNF2_8MHz_31k25BPS = (0xA4), 
    /*   20 Kbps */ CNF2_8MHz_20kBPS   = (0xBF), CNF2_16MHz_20kBPS   = (0xFF), 
    /*   10 Kbps */ CNF2_8MHz_10kBPS   = (0xBF), CNF2_16MHz_10kBPS   = (0xFF), 
    /*    5 Kbps */ CNF2_8MHz_5kBPS    = (0xBF), CNF2_16MHz_5kBPS    = (0xFF), 
        };

        // CNF1: CONFIGURATION REGISTER 1 DATA @ Reg:0x2A - MCP2515 REGISTER 5-1:
        enum CNF1 : uint8_t {
    /* Baud */  /*       8 MHz                        16 MHz                        20 MHz     Oscillator*/
    /* 1000 Kbps */ CNF1_8MHz_1000kBPS = (0x00), CNF1_16MHz_1000kBPS = (0x00), CNF1_20MHz_1000kBPS = (0x00),
    /*  500 Kbp s*/ CNF1_8MHz_500kBPS  = (0x00), CNF1_16MHz_500kBPS  = (0x00), CNF1_20MHz_500kBPS  = (0x00),
    /*  250 Kbps */ CNF1_8MHz_250kBPS  = (0x00), CNF1_16MHz_250kBPS  = (0x41), CNF1_20MHz_250kBPS  = (0x41),
    /*  200 Kbps */ CNF1_8MHz_200kBPS  = (0x00), CNF1_16MHz_200kBPS  = (0x01), CNF1_20MHz_200kBPS  = (0x01),
    /*  125 Kbps */ CNF1_8MHz_125kBPS  = (0x01), CNF1_16MHz_125kBPS  = (0x03), CNF1_20MHz_125kBPS  = (0x03),
    /*  100 Kbps */ CNF1_8MHz_100kBPS  = (0x01), CNF1_16MHz_100kBPS  = (0x03), CNF1_20MHz_100kBPS  = (0x04),
    /* 83.33 Kbps*/                              CNF1_16MHz_83k3BPS  = (0x03), CNF1_20MHz_83k3BPS  = (0x04),
    /*   80 Kbps */ CNF1_8MHz_80kBPS   = (0x01), CNF1_16MHz_80kBPS   = (0x03), CNF1_20MHz_80kBPS   = (0x04),
    /*   50 Kbps */ CNF1_8MHz_50kBPS   = (0x03), CNF1_16MHz_50kBPS   = (0x07), CNF1_20MHz_50kBPS   = (0x09),
    /*   40 Kbps */ CNF1_8MHz_40kBPS   = (0x03), CNF1_16MHz_40kBPS   = (0x07), CNF1_20MHz_40kBPS   = (0x09),
    /*   33 Kbps */ CNF1_8MHz_33k3BPS  = (0x47), CNF1_16MHz_33k3BPS  = (0x4E), CNF1_20MHz_33k3BPS  = (0x0B),
    /* 31.25 Kbps*/ CNF1_8MHz_31k25BPS = (0x07), 
    /*   20 Kbps */ CNF1_8MHz_20kBPS   = (0x07), CNF1_16MHz_20kBPS   = (0x0F),
    /*   10 Kbps */ CNF1_8MHz_10kBPS   = (0x0F), CNF1_16MHz_10kBPS   = (0x1F),
    /*    5 Kbps */ CNF1_8MHz_5kBPS    = (0x1F), CNF1_16MHz_5kBPS    = (0x3F)
        };

        // CANINTE: CAN INTERRUPT ENABLE REGISTER DATA @ Reg:0x2B - MCP2515 REGISTER 7-1:
        enum CANINTE : uint8_t {
            CANINTE_MERR_INT  = (1 << 7), // Message Error Interrupt
            CANINTE_WAK_INT   = (1 << 6), // Wake-up Interrupt
            CANINTE_ERR_INT   = (1 << 5), // Error Interrupt
            CANINTE_TX2_INT   = (1 << 4), // Enable TXB2 interrupts
            CANINTE_TX1_INT   = (1 << 3), // Enable TXB1 interrupt
            CANINTE_TX0_INT   = (1 << 2), // Enable TXB0 interrupt
            CANINTE_RX1IE_INT = (1 << 1), // Receive Buffer 1 Interrupt
            CANINTE_RX0IE_INT = (1 << 0), // Receive Buffer 0 Interrupt
            CANINTE_NO_INT    = (0x00),   // Disable all interrupts

            CANINTE_RX_INT    = (CANINTE_RX0IE_INT | CANINTE_RX1IE_INT),   // Enable receive interrupts
            CANINTE_TX_INT    = (CANINTE_TX0_INT | CANINTE_TX1_INT | CANINTE_TX2_INT)   // Enable all transmit interrupts
        };

        // CANINTF: CAN INTERRUPT FLAG REGISTER DATA@ Reg:0x2C - MCP2515 REGISTER 7-2: 
        enum CANINTF : uint8_t {
            CANINTF_MERRF = (1 << 7),   // Message Error Interrupt Flag
            CANINTF_WAKIF = (1 << 6),   // Wake-up Interrupt Flag
            CANINTF_ERRIF = (1 << 5),   // Error Interrupt Flag
            CANINTF_TX2IF = (1 << 4),   // Transmit Buffer 2 Interrupt Flag
            CANINTF_TX1IF = (1 << 3),   // Transmit Buffer 1 Interrupt Flag
            CANINTF_TX0IF = (1 << 2),   // Transmit Buffer 0 Interrupt Flag
            CANINTF_RX1IF = (1 << 1),   // Receive Buffer 1 Interrupt Flag
            CANINTF_RX0IF = (1 << 0),   // Receive Buffer 0 Interrupt Flag
        };

        // EFLG: ERROR FLAG REGISTER DATA @ Reg:0x2D - MCP2515 REGISTER 6-3: 
        enum EFLG : uint8_t {
            EFLG_RX1OVR = (1 << 7),     // RX buffer 1 overflowed
            EFLG_RX0OVR = (1 << 6),     // RX buffer 0 overflowed
            EFLG_TXBO   = (1 << 5),     // TX error counter > 255
            EFLG_TXEP   = (1 << 4),     // TX error counter > 127
            EFLG_RXEP   = (1 << 3),     // RX error counter > 127
            EFLG_TXWARN = (1 << 2),     // TX error counter > 95
            EFLG_RXWARN = (1 << 1),     // RX error counter > 95
            EFLG_EWARN  = (1 << 0),     // TX or RX error counter > 95

            EFLG_ERRORMASK = (EFLG_RX1OVR | EFLG_RX0OVR | EFLG_TXBO | EFLG_TXEP | EFLG_RXEP)
        };

        // TXB0CTRL: TXBnCTRL: TRANSMIT BUFFER n CONTROL REGISTER DATA @ Reg:0x30 - MCP2515 REGISTER 3-1 / 3-2 / 3-3: 
        // TXB1CTRL: @ Reg:0x40
        // TXB2CTRL: @ Reg:0x50

        static const int MCP_N_TXBUFFERS = 3;
        // Control Register (CTRL), Can identifier (SIDH), and Data (DATA)  
        struct TXBn_REGS {
            REGISTER CTRL;
            REGISTER SIDH;
            REGISTER DATA;
        };
        
        static const uint8_t RTR_MASK       = 0x40;
        static const uint8_t TXB_EXIDE_MASK = 0x08;
        static const uint8_t DLC_MASK       = 0x0F;

        // 3 Transmit Buffers starting with Control field(TXBnCTRL), Can identifier (SIDH), and Data fields (DLC)  
        static constexpr struct MCP2515::TXBn_REGS TXB[MCP2515::MCP_N_TXBUFFERS] = {
            {MCP_TXB0CTRL, MCP_TXB0SIDH, MCP_TXB0D0},
            {MCP_TXB1CTRL, MCP_TXB1SIDH, MCP_TXB1D0},
            {MCP_TXB2CTRL, MCP_TXB2SIDH, MCP_TXB2D0}
        };

        // MCP2515 TXBnCTRL: REGISTER 3-1: DATA
        enum TXBnCTRL : uint8_t {
            TXBnCTRL_ABTF   = (1 << 7),
            TXBnCTRL_MLOA   = (1 << 6),
            TXBnCTRL_TXERR  = (1 << 5),
            TXBnCTRL_TXREQ  = (1 << 4),
            TXBnCTRL_TXIE   = (1 << 3),
            TXBnCTRL_TXP1   = (1 << 1),
            TXBnCTRL_TXP0   = (1 << 0),
            TXBnCTRL_TXP    = (TXBnCTRL_TXP1 | TXBnCTRL_TXP0)
        };

        // RXB0CTRL: RECEIVE BUFFER 0 CONTROL REGISTER DATA @ Reg:0x60  - MCP2515 REGISTER 4-1: 
        // RXB1CTRL: RECEIVE BUFFER 1 CONTROL REGISTER DATA @ Reg:0x70  - MCP2515 REGISTER 4-2: 
        static const int N_RXBUFFERS = 2;
        // Control Register (CTRL), Can identifier (SIDH), and Data (DATA), Interrupt (CANINTF)
        struct RXBn_REGS {
            REGISTER CTRL;
            REGISTER SIDH;
            REGISTER DATA;
            CANINTF  CANINTF_RXnIF;
        };
        // 2 Receive Buffers starting with Control field(MCP_RXBnCTRL), Can identifier (SIDH), Data fields (DLC), and Interrupt
        static constexpr struct MCP2515::RXBn_REGS RXB[N_RXBUFFERS] = {
            {MCP_RXB0CTRL, MCP_RXB0SIDH, MCP_RXB0D0, CANINTF_RX0IF},
            {MCP_RXB1CTRL, MCP_RXB1SIDH, MCP_RXB1D0, CANINTF_RX1IF}
        };
        
        // MCP2515 RXB0CTRL: REGISTER 4-1 / 4-2: DATA
        enum RXBnCTRL : uint8_t {
            RXBnCTRL_RXM_EXT    = (0x40),
            RXBnCTRL_RXM_STD    = (0x20),
            RXBnCTRL_RXM_STDEXT = (0x00),

            RXBnCTRL_RXM_MASK   = (RXBnCTRL_RXM_EXT | RXBnCTRL_RXM_STD),

            RXBnCTRL_RTR        = (1 << 3),
            RXB0CTRL_BUKT       = (1 << 2),
            RXB1CTRL_FILHIT     = (1 << 0),
            RXB0CTRL_FILHIT     = (0x00),

            RXB1CTRL_FILHIT_MASK = 0x07,
            RXB0CTRL_FILHIT_MASK = 0x03,
        };

        // Frame Structure for Standard and Extended
        enum TXRX : uint8_t {
            TXRX_SIDH = 0,
            TXRX_SIDL = 1,
            TXRX_EID8 = 2,
            TXRX_EID0 = 3,
            TXRX_DLC  = 4,
            TXRX_DATA = 5
        };

private:
        spi_inst_t* SPI_CHANNEL;
        uint8_t SPI_CS_PIN;
        uint8_t SPI_TX_PIN;
        uint8_t SPI_RX_PIN;
        uint8_t SPI_SCK_PIN;
        uint32_t SPI_CLOCK;

        uint8_t nReservedTx; // Count of tx buffers for reserved send

        uint8_t CAN_INT_PIN;

        bool IsOpen;
        static bool CanInUse;

        inline void startSPI();
        inline void endSPI();

        uint8_t txIfFlag(uint8_t i);

        RETURN setMode(const CANCTRL_REQOP_MODE mode);

        uint8_t readRegister(const REGISTER reg);
        void readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n);

        void setRegister(const REGISTER reg, const uint8_t value);
        void setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n);

        void modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data);

        void prepareId(uint8_t *buffer, const bool ext, const uint32_t id);


    public:
        MCP2515(
            spi_inst_t* CHANNEL = spi0,
            uint8_t CS_PIN = PICO_DEFAULT_SPI_CSN_PIN,
            uint8_t TX_PIN = PICO_DEFAULT_SPI_TX_PIN,
            uint8_t RX_PIN = PICO_DEFAULT_SPI_RX_PIN,
            uint8_t SCK_PIN = PICO_DEFAULT_SPI_SCK_PIN,
            uint32_t _SPI_CLOCK = DEFAULT_SPI_CLOCK
        );

        RETURN Begin();
        RETURN Reset();
        void initCANBuffers(void);

        RETURN setConfigMode();
        RETURN setListenOnlyMode();
        RETURN setSleepMode();
        RETURN setLoopbackMode();
        RETURN setNormalMode();

//        RETURN setClkOut(const CAN_CLKOUT divisor);

        RETURN setBitrate(const CAN_SPEED canSpeed);
        RETURN setBitrate(const CAN_SPEED canSpeed, const CAN_CLOCK canClock);

        RETURN setFilterMask(const MASK num, const bool ext, const uint32_t ulData);
        RETURN setFilter(const RXF num, const bool ext, const uint32_t ulData);

        void reserveTxBuffers(uint8_t nTxBuf=0) { nReservedTx = (nTxBuf < MCP_N_TXBUFFERS ? nTxBuf : (MCP_N_TXBUFFERS - 1) ); }
        uint8_t getLastTxBuffer() { return MCP_N_TXBUFFERS - 1; }   // read index of last tx buffer

        RETURN sendMessage(const TXBn txbn, const struct can_frame *frame);
        RETURN sendMessage(const struct can_frame *frame);

        RETURN readMessage(struct can_frame *frame);
        RETURN readMessage(const RXBn rxbn, struct can_frame *frame);

        uint8_t readStatus(void);

        uint8_t readRxTxStatus(void);   // read if something to send or received
        uint8_t readRxStatus(void);
        uint8_t statusToBuffer(uint8_t flags);
        uint8_t checkClearRxStatus(uint8_t *status);    // read and clear and return first found rx status bit

//        bool checkError(void);
//        uint8_t getErrorFlags(void);
//        void clearRXnOVRFlags(void);
//        uint8_t getInterrupts(void);
//        uint8_t getInterruptMask(void);
//        void clearInterrupts(void);
//        void clearTXInterrupts(void);
//        void clearRXnOVR(void);
//        void clearMERR();
//        void clearERRIF();
//        uint8_t errorCountRX(void);
//        uint8_t errorCountTX(void);
   
        uint8_t checkClearTxStatus(uint8_t *status, uint8_t iTxBuf=0xff);   // read and clear and return first found or buffer specified tx status bit
        void clearBufferTransmitIfFlags(uint8_t flags);
};

#endif // _MCP2515_H_