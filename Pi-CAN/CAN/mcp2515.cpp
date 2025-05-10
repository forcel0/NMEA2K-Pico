#include <stdio.h>
#include <cstring>
#include "mcp2515.h"
#include "pico/stdlib.h"

bool MCP2515::CanInUse = false;
MCP2515 *pMCP2515 = 0;


MCP2515::MCP2515(spi_inst_t* CHANNEL, uint8_t CS_PIN, uint8_t TX_PIN, uint8_t RX_PIN, uint8_t SCK_PIN, uint32_t SPI_CLOCK) : nReservedTx(0)
{
    this->SPI_CHANNEL = CHANNEL;
    this->SPI_CS_PIN = CS_PIN;
    this->SPI_TX_PIN = TX_PIN;
    this->SPI_RX_PIN = RX_PIN;
    this->SPI_SCK_PIN = SCK_PIN;
    this->SPI_CLOCK = SPI_CLOCK;
}

MCP2515::RETURN MCP2515::Begin()
{
    spi_init(this->SPI_CHANNEL, this->SPI_CLOCK);

    gpio_set_function(this->SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(this->SPI_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(this->SPI_SCK_PIN, GPIO_FUNC_SPI);

    spi_set_format(this->SPI_CHANNEL, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_init(this->SPI_CS_PIN);
    gpio_set_dir(this->SPI_CS_PIN, GPIO_OUT);

    endSPI();

    //Initialize interface
    Reset();

    setConfigMode();

    // depends on oscillator & capacitors used
    sleep_ms(10);

    // set boadrate
    setBitrate(CAN_250KBPS, CAN_16MHZ); // jimg get this from configuration

    // init canbuffers
    initCANBuffers();
    
    // interrupt mode
    setRegister(MCP_CANINTE, CANINTE_RX_INT | CANINTE_TX_INT);

    // receives all valid messages using either Standard or Extended Identifiers that
    // meet filter criteria. RXF0 is applied for RXB0, RXF1 is applied for RXB1
    modifyRegister(MCP_RXB0CTRL,
                   RXBnCTRL_RXM_MASK | RXB0CTRL_BUKT | RXB0CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB0CTRL_BUKT | RXB0CTRL_FILHIT);
    modifyRegister(MCP_RXB1CTRL,
                   RXBnCTRL_RXM_MASK | RXB1CTRL_FILHIT_MASK,
                   RXBnCTRL_RXM_STDEXT | RXB1CTRL_FILHIT);

    // clear filters and masks
    // do not filter any standard frames for RXF0 used by RXB0
    // do not filter any extended frames for RXF1 used by RXB1
    RXF filters[] = {RXF0, RXF1, RXF2, RXF3, RXF4, RXF5};
    for (int i=0; i<6; i++) {
        bool ext = (i == 1);
        RETURN result = setFilter(filters[i], ext, 0);
        if (result != RETURN_OK) {
            return result;
        }
    }

    MASK masks[] = {MASK0, MASK1};
    for (int i=0; i<2; i++) {
        RETURN result = setFilterMask(masks[i], true, 0);
        if (result != RETURN_OK) {
            return result;
        }
    }

    //setListenOnlyMode();
    setNormalMode();

    return RETURN_OK; // jimg TO DO RETURN checking
}

inline void MCP2515::startSPI() {
    asm volatile("nop \n nop \n nop");
    gpio_put(this->SPI_CS_PIN, 0);
    asm volatile("nop \n nop \n nop");
}

inline void MCP2515::endSPI() {
    asm volatile("nop \n nop \n nop");
    gpio_put(this->SPI_CS_PIN, 1);
    asm volatile("nop \n nop \n nop");
}


/*********************************************************************************************************
** Function name:           txIfFlag
** Descriptions:            return tx interrupt flag
*********************************************************************************************************/
uint8_t MCP2515::txIfFlag(uint8_t i)
{
    switch (i)
    {
      case 0: return CANINTF_TX0IF;
      case 1: return CANINTF_TX1IF;
      case 2: return CANINTF_TX2IF;
    }
    return 0;
}

/*********************************************************************************************************
** Function name:           initCANBuffers
** Descriptions:            Initialize Tx and Rx buffers
*********************************************************************************************************/
void MCP2515::initCANBuffers(void)
{
    uint8_t zeros[14];
    memset(zeros, 0, sizeof(zeros));
    setRegisters(MCP_TXB0CTRL, zeros, 14);
    setRegisters(MCP_TXB1CTRL, zeros, 14);
    setRegisters(MCP_TXB2CTRL, zeros, 14);

    setRegister(MCP_RXB0CTRL, 0);
    setRegister(MCP_RXB1CTRL, 0);
}

// reinitialize the internal registers RESET Instruction ref 12.2
inline MCP2515::RETURN MCP2515::Reset(void) noexcept
{
    startSPI();

    uint8_t instruction = MCP_RESET;
    spi_write_blocking(this->SPI_CHANNEL, &instruction, 1);

    endSPI();

    return RETURN_OK;
}

uint8_t MCP2515::readRegister(const REGISTER reg) noexcept
{
    startSPI();

    uint8_t data[2] = {
        MCP_READ,
        reg
    };

    spi_write_blocking(this->SPI_CHANNEL, data, 2);

    uint8_t ret;
    spi_read_blocking(this->SPI_CHANNEL, 0x00, &ret, 1);

    endSPI();

    return ret;
}

inline void MCP2515::readRegisters(const REGISTER reg, uint8_t values[], const uint8_t n) noexcept
{
    startSPI();

    uint8_t data[2] = {
        MCP_READ,
        reg
    };
    spi_write_blocking(this->SPI_CHANNEL, data, 2);

    spi_read_blocking(this->SPI_CHANNEL, 0x00, values, n);

    endSPI();
}

void MCP2515::setRegister(const REGISTER reg, const uint8_t value) noexcept
{
    startSPI();

    uint8_t data[3] = {
        MCP_WRITE,
        reg,
        value
    };
    spi_write_blocking(this->SPI_CHANNEL, data, 3);

    endSPI();
}

inline void MCP2515::setRegisters(const REGISTER reg, const uint8_t values[], const uint8_t n) noexcept
{
    startSPI();

    uint8_t data[2] = {
        MCP_WRITE,
        reg
    };
    spi_write_blocking(this->SPI_CHANNEL, data, 2);

    spi_write_blocking(this->SPI_CHANNEL, values, n);

    endSPI();
}

inline void MCP2515::modifyRegister(const REGISTER reg, const uint8_t mask, const uint8_t data) noexcept
{
    startSPI();

    uint8_t d[4] = {
        MCP_BITMOD,
        reg,
        mask,
        data
    };

    spi_write_blocking(this->SPI_CHANNEL, d, 4);

    endSPI();
}

inline MCP2515::RETURN MCP2515::setConfigMode() noexcept
{
    return setMode(CANCTRL_REQOP_CONFIG);
}

inline MCP2515::RETURN MCP2515::setListenOnlyMode() noexcept
{
    return setMode(CANCTRL_REQOP_LISTENONLY);
}

inline MCP2515::RETURN MCP2515::setSleepMode() noexcept
{
    return setMode(CANCTRL_REQOP_SLEEP);
}

inline MCP2515::RETURN MCP2515::setLoopbackMode() noexcept
{
    return setMode(CANCTRL_REQOP_LOOPBACK);
}

inline MCP2515::RETURN MCP2515::setNormalMode() noexcept
{
    return setMode(CANCTRL_REQOP_NORMAL);
}

inline MCP2515::RETURN MCP2515::setMode(const CANCTRL_REQOP_MODE mode) noexcept
{
    modifyRegister(MCP_CANCTRL1, CANCTRL_REQOP, mode);

    unsigned long endTime = to_ms_since_boot(get_absolute_time()) + 10;
    bool modeMatch = false;
    while (to_ms_since_boot(get_absolute_time()) < endTime) {
        uint8_t newmode = readRegister(MCP_CANSTAT1);
        newmode &= CANSTAT_OPMOD;

        modeMatch = newmode == mode;

        if (modeMatch) {
            break;
        }
    }

    return modeMatch 
        ? RETURN_OK
        : RETURN_FAIL;
}

// inline MCP2515::RETURN MCP2515::setClkOut(const CAN_CLKOUT divisor) noexcept
// {
//     if (divisor == CLKOUT_DISABLE) {
// 	// Turn off CLKEN
// 	modifyRegister(MCP_CANCTRL1, CANCTRL_CLKEN, 0);

// 	// Turn on CLKOUT for SOF
// 	modifyRegister(MCP_CNF3, CNF3_SOF, CNF3_SOF);
//         return RETURN_OK;
//     }

//     // Set the prescaler (CLKPRE)
//     modifyRegister(MCP_CANCTRL1, CANCTRL_CLKPRE, divisor);

//     // Turn on CLKEN */
//     modifyRegister(MCP_CANCTRL1, CANCTRL_CLKEN, CANCTRL_CLKEN);

//     // Turn off CLKOUT for SOF
//     modifyRegister(MCP_CNF3, CNF3_SOF, 0x00);
//     return RETURN_OK;
// }

inline MCP2515::RETURN MCP2515::setBitrate(const CAN_SPEED canSpeed) noexcept
{
    return setBitrate(canSpeed, CAN_16MHZ);
}

inline MCP2515::RETURN MCP2515::setBitrate(const CAN_SPEED canSpeed, CAN_CLOCK canClock) noexcept
{
    RETURN RETURN = setConfigMode();
    if (RETURN != RETURN_OK) {
        return RETURN;
    }

    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canClock)
    {
        case (CAN_8MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5KBPS
            cfg1 = CNF1_8MHz_5kBPS;
            cfg2 = CNF2_8MHz_5kBPS;
            cfg3 = CNF3_8MHz_5kBPS;
            break;

            case (CAN_10KBPS):                                              //  10KBPS
            cfg1 = CNF1_8MHz_10kBPS;
            cfg2 = CNF2_8MHz_10kBPS;
            cfg3 = CNF3_8MHz_10kBPS;
            break;

            case (CAN_20KBPS):                                              //  20KBPS
            cfg1 = CNF1_8MHz_20kBPS;
            cfg2 = CNF2_8MHz_20kBPS;
            cfg3 = CNF3_8MHz_20kBPS;
            break;

            case (CAN_31K25BPS):                                            //  31.25KBPS
            cfg1 = CNF1_8MHz_31k25BPS;
            cfg2 = CNF2_8MHz_31k25BPS;
            cfg3 = CNF3_8MHz_31k25BPS;
            break;

            case (CAN_33KBPS):                                              //  33.333KBPS
            cfg1 = CNF1_8MHz_33k3BPS;
            cfg2 = CNF2_8MHz_33k3BPS;
            cfg3 = CNF3_8MHz_33k3BPS;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = CNF1_8MHz_40kBPS;
            cfg2 = CNF2_8MHz_40kBPS;
            cfg3 = CNF3_8MHz_40kBPS;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = CNF1_8MHz_50kBPS;
            cfg2 = CNF2_8MHz_50kBPS;
            cfg3 = CNF3_8MHz_50kBPS;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = CNF1_8MHz_80kBPS;
            cfg2 = CNF2_8MHz_80kBPS;
            cfg3 = CNF3_8MHz_80kBPS;
            break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = CNF1_8MHz_100kBPS;
            cfg2 = CNF2_8MHz_100kBPS;
            cfg3 = CNF3_8MHz_100kBPS;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = CNF1_8MHz_125kBPS;
            cfg2 = CNF2_8MHz_125kBPS;
            cfg3 = CNF3_8MHz_125kBPS;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = CNF1_8MHz_200kBPS;
            cfg2 = CNF2_8MHz_200kBPS;
            cfg3 = CNF3_8MHz_200kBPS;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = CNF1_8MHz_250kBPS;
            cfg2 = CNF2_8MHz_250kBPS;
            cfg3 = CNF3_8MHz_250kBPS;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = CNF1_8MHz_500kBPS;
            cfg2 = CNF2_8MHz_500kBPS;
            cfg3 = CNF3_8MHz_500kBPS;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = CNF1_8MHz_1000kBPS;
            cfg2 = CNF2_8MHz_1000kBPS;
            cfg3 = CNF3_8MHz_1000kBPS;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (CAN_16MHZ):
        switch (canSpeed)
        {
            case (CAN_5KBPS):                                               //   5Kbps
            cfg1 = CNF1_16MHz_5kBPS;
            cfg2 = CNF2_16MHz_5kBPS;
            cfg3 = CNF3_16MHz_5kBPS;
            break;

            case (CAN_10KBPS):                                              //  10Kbps
            cfg1 = CNF1_16MHz_10kBPS;
            cfg2 = CNF2_16MHz_10kBPS;
            cfg3 = CNF3_16MHz_10kBPS;
            break;

            case (CAN_20KBPS):                                              //  20Kbps
            cfg1 = CNF1_16MHz_20kBPS;
            cfg2 = CNF2_16MHz_20kBPS;
            cfg3 = CNF3_16MHz_20kBPS;
            break;

            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = CNF1_16MHz_33k3BPS;
            cfg2 = CNF2_16MHz_33k3BPS;
            cfg3 = CNF3_16MHz_33k3BPS;
            break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = CNF1_16MHz_40kBPS;
            cfg2 = CNF2_16MHz_40kBPS;
            cfg3 = CNF3_16MHz_40kBPS;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = CNF1_16MHz_50kBPS;
            cfg2 = CNF2_16MHz_50kBPS;
            cfg3 = CNF3_16MHz_50kBPS;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = CNF1_16MHz_80kBPS;
            cfg2 = CNF2_16MHz_80kBPS;
            cfg3 = CNF3_16MHz_80kBPS;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = CNF1_16MHz_83k3BPS;
            cfg2 = CNF2_16MHz_83k3BPS;
            cfg3 = CNF3_16MHz_83k3BPS;
            break; 

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = CNF1_16MHz_100kBPS;
            cfg2 = CNF2_16MHz_100kBPS;
            cfg3 = CNF3_16MHz_100kBPS;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = CNF1_16MHz_125kBPS;
            cfg2 = CNF2_16MHz_125kBPS;
            cfg3 = CNF3_16MHz_125kBPS;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = CNF1_16MHz_200kBPS;
            cfg2 = CNF2_16MHz_200kBPS;
            cfg3 = CNF3_16MHz_200kBPS;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = CNF1_16MHz_250kBPS;
            cfg2 = CNF2_16MHz_250kBPS;
            cfg3 = CNF3_16MHz_250kBPS;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = CNF1_16MHz_500kBPS;
            cfg2 = CNF2_16MHz_500kBPS;
            cfg3 = CNF3_16MHz_500kBPS;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = CNF1_16MHz_1000kBPS;
            cfg2 = CNF2_16MHz_1000kBPS;
            cfg3 = CNF3_16MHz_1000kBPS;
            break;

            default:
            set = 0;
            break;
        }
        break;

        case (CAN_20MHZ):
        switch (canSpeed)
        {
            case (CAN_33KBPS):                                              //  33.333Kbps
            cfg1 = CNF1_20MHz_33k3BPS;
            cfg2 = CNF2_20MHz_33k3BPS;
            cfg3 = CNF3_20MHz_33k3BPS;
	    break;

            case (CAN_40KBPS):                                              //  40Kbps
            cfg1 = CNF1_20MHz_40kBPS;
            cfg2 = CNF2_20MHz_40kBPS;
            cfg3 = CNF3_20MHz_40kBPS;
            break;

            case (CAN_50KBPS):                                              //  50Kbps
            cfg1 = CNF1_20MHz_50kBPS;
            cfg2 = CNF2_20MHz_50kBPS;
            cfg3 = CNF3_20MHz_50kBPS;
            break;

            case (CAN_80KBPS):                                              //  80Kbps
            cfg1 = CNF1_20MHz_80kBPS;
            cfg2 = CNF2_20MHz_80kBPS;
            cfg3 = CNF3_20MHz_80kBPS;
            break;

            case (CAN_83K3BPS):                                             //  83.333Kbps
            cfg1 = CNF1_20MHz_83k3BPS;
            cfg2 = CNF2_20MHz_83k3BPS;
            cfg3 = CNF3_20MHz_83k3BPS;
	    break;

            case (CAN_100KBPS):                                             // 100Kbps
            cfg1 = CNF1_20MHz_100kBPS;
            cfg2 = CNF2_20MHz_100kBPS;
            cfg3 = CNF3_20MHz_100kBPS;
            break;

            case (CAN_125KBPS):                                             // 125Kbps
            cfg1 = CNF1_20MHz_125kBPS;
            cfg2 = CNF2_20MHz_125kBPS;
            cfg3 = CNF3_20MHz_125kBPS;
            break;

            case (CAN_200KBPS):                                             // 200Kbps
            cfg1 = CNF1_20MHz_200kBPS;
            cfg2 = CNF2_20MHz_200kBPS;
            cfg3 = CNF3_20MHz_200kBPS;
            break;

            case (CAN_250KBPS):                                             // 250Kbps
            cfg1 = CNF1_20MHz_250kBPS;
            cfg2 = CNF2_20MHz_250kBPS;
            cfg3 = CNF3_20MHz_250kBPS;
            break;

            case (CAN_500KBPS):                                             // 500Kbps
            cfg1 = CNF1_20MHz_500kBPS;
            cfg2 = CNF2_20MHz_500kBPS;
            cfg3 = CNF3_20MHz_500kBPS;
            break;

            case (CAN_1000KBPS):                                            //   1Mbps
            cfg1 = CNF1_20MHz_1000kBPS;
            cfg2 = CNF2_20MHz_1000kBPS;
            cfg3 = CNF3_20MHz_1000kBPS;
            break;

            default:
            set = 0;
            break;
        }
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        setRegister(MCP_CNF1, cfg1);
        setRegister(MCP_CNF2, cfg2);
        setRegister(MCP_CNF3, cfg3);
        return RETURN_OK;
    }
    else {
        return RETURN_FAIL;
    }
}

void MCP2515::prepareId(uint8_t *buffer, const bool ext, const uint32_t id)
{
    uint16_t canid = (uint16_t)(id & 0x0FFFF);

    if (ext) {
        buffer[TXRX_EID0] = (uint8_t) (canid & 0xFF);
        buffer[TXRX_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        buffer[TXRX_SIDL] = (uint8_t) (canid & 0x03);
        buffer[TXRX_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        buffer[TXRX_SIDL] |= TXB_EXIDE_MASK;
        buffer[TXRX_SIDH] = (uint8_t) (canid >> 5);
    } else {
        buffer[TXRX_SIDH] = (uint8_t) (canid >> 3);
        buffer[TXRX_SIDL] = (uint8_t) ((canid & 0x07 ) << 5);
        buffer[TXRX_EID0] = 0;
        buffer[TXRX_EID8] = 0;
    }
}

MCP2515::RETURN MCP2515::setFilterMask(const MASK mask, const bool ext, const uint32_t ulData)
{
    RETURN res = setConfigMode();
    if (res != RETURN_OK) {
        return res;
    }
    
    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);

    REGISTER reg;
    switch (mask) {
        case MASK0: reg = MCP_RXM0SIDH; break;
        case MASK1: reg = MCP_RXM1SIDH; break;
        default:
            return RETURN_FAIL;
    }

    setRegisters(reg, tbufdata, 4);
    
    return RETURN_OK;
}

MCP2515::RETURN MCP2515::setFilter(const RXF num, const bool ext, const uint32_t ulData)
{
    RETURN res = setConfigMode();
    if (res != RETURN_OK) {
        return res;
    }

    REGISTER reg;

    switch (num) {
        case RXF0: reg = MCP_RXF0SIDH; break;
        case RXF1: reg = MCP_RXF1SIDH; break;
        case RXF2: reg = MCP_RXF2SIDH; break;
        case RXF3: reg = MCP_RXF3SIDH; break;
        case RXF4: reg = MCP_RXF4SIDH; break;
        case RXF5: reg = MCP_RXF5SIDH; break;
        default:
            return RETURN_FAIL;
    }

    uint8_t tbufdata[4];
    prepareId(tbufdata, ext, ulData);
    setRegisters(reg, tbufdata, 4);

    return RETURN_OK;
}

MCP2515::RETURN MCP2515::sendMessage(const TXBn txbn, const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return RETURN_FAILTX;
    }

    const struct TXBn_REGS *txbuf = &TXB[txbn];

    uint8_t data[13];

    bool ext = (frame->can_id & CAN_EFF_FLAG);
    bool rtr = (frame->can_id & CAN_RTR_FLAG);
    uint32_t id = (frame->can_id & (ext ? CAN_EFF_MASK : CAN_SFF_MASK));

    prepareId(data, ext, id);

    data[TXRX_DLC] = rtr ? (frame->can_dlc | RTR_MASK) : frame->can_dlc;

    memcpy(&data[TXRX_DATA], frame->data, frame->can_dlc);

    setRegisters(txbuf->SIDH, data, 5 + frame->can_dlc);

    modifyRegister(txbuf->CTRL, TXBnCTRL_TXREQ, TXBnCTRL_TXREQ);

    uint8_t ctrl = readRegister(txbuf->CTRL);
    if ((ctrl & (TXBnCTRL_ABTF | TXBnCTRL_MLOA | TXBnCTRL_TXERR)) != 0) {
        return RETURN_FAILTX;
    }
    return RETURN_OK;
}

MCP2515::RETURN MCP2515::sendMessage(const struct can_frame *frame)
{
    if (frame->can_dlc > CAN_MAX_DLEN) {
        return RETURN_FAILTX;
    }

    TXBn txBuffers[MCP_N_TXBUFFERS] = {TXB0, TXB1, TXB2};

    for(int i=0; i < MCP_N_TXBUFFERS; i++) {
        const struct TXBn_REGS *txbuf = &TXB[txBuffers[i]];
        uint8_t ctrlval = readRegister(txbuf->CTRL);
        if ((ctrlval & TXBnCTRL_TXREQ) == 0) {
            return sendMessage(txBuffers[i], frame);
        }
    }

    return RETURN_ALLTXBUSY;
}

MCP2515::RETURN MCP2515::readMessage(struct can_frame *frame)
{
    RETURN rc = RETURN_OK;
    uint8_t stat = readStatus();

    if ( stat & STAT_RX0IF ) {
        rc = readMessage(RXB0, frame);
    } else if ( stat & STAT_RX1IF ) {
        rc = readMessage(RXB1, frame);
    } else {
        rc = RETURN_NOMSG;
    }

    return rc;
}

MCP2515::RETURN MCP2515::readMessage(const RXBn rxbn, struct can_frame *frame)
{
    const struct RXBn_REGS *rxb = &RXB[rxbn];

    uint8_t tbufdata[5];

    readRegisters(rxb->SIDH, tbufdata, 5);

    uint32_t id = (tbufdata[TXRX_SIDH]<<3) + (tbufdata[TXRX_SIDL]>>5);

    if ( (tbufdata[TXRX_SIDL] & TXB_EXIDE_MASK) ==  TXB_EXIDE_MASK ) {
        id = (id<<2) + (tbufdata[TXRX_SIDL] & 0x03);
        id = (id<<8) + tbufdata[TXRX_EID8];
        id = (id<<8) + tbufdata[TXRX_EID0];
        id |= CAN_EFF_FLAG;
    }

    uint8_t dlc = (tbufdata[TXRX_DLC] & DLC_MASK);
    if (dlc > CAN_MAX_DLEN) {
        return RETURN_FAIL;
    }

    uint8_t ctrl = readRegister(rxb->CTRL);
    if (ctrl & RXBnCTRL_RTR) {
        id |= CAN_RTR_FLAG;
    }

    frame->can_id = id;
    frame->can_dlc = dlc;

    readRegisters(rxb->DATA, frame->data, dlc);

    modifyRegister(MCP_CANINTF, rxb->CANINTF_RXnIF, 0x00);

    return RETURN_OK;
}

/*********************************************************************************************************
** Function name:           getStatus
** Descriptions:            read status instruction ref 12.8
*********************************************************************************************************/
inline uint8_t MCP2515::readStatus(void) noexcept
{
    startSPI();

    uint8_t instruction = MCP_READ_STATUS;

    spi_write_blocking(this->SPI_CHANNEL, &instruction, 1);

    uint8_t ret;
    spi_read_blocking(this->SPI_CHANNEL, 0x00, &ret, 1);

    endSPI();

    return ret;
}

/*********************************************************************************************************
** Function name:           readRxTxStatus
** Descriptions:            Read RX and TX interrupt bits. Function uses status reading, but translates.
**                          result to MCP_CANINTF. With this you can check status e.g. on interrupt sr
**                          with one single call to save SPI calls. Then use checkClearRxStatus and
**                          checkClearTxStatus for testing. 
*********************************************************************************************************/
uint8_t MCP2515::readRxTxStatus(void) noexcept
{
    uint8_t ret=(readStatus() & (STAT_TXIF_MASK | STAT_RXIF_MASK));
    ret = (ret & STAT_TX0IF ? CANINTF_TX0IF : 0) | 
            (ret & STAT_TX1IF ? CANINTF_TX1IF : 0) | 
            (ret & STAT_TX2IF ? CANINTF_TX2IF : 0) | 
            (ret & STAT_RXIF_MASK); // Rx bits happend to be same on status and MCP_CANINTF

    return ret;                
}

/*********************************************************************************************************
** Function name:           readRxStatus
** Descriptions:            check if got something
*********************************************************************************************************/
uint8_t MCP2515::readRxStatus(void)
{
    uint8_t res = readStatus();

    return ((res & STAT_RXIF_MASK)?RETURN_MSGAVAIL:RETURN_NOMSG);
}

uint8_t MCP2515::statusToBuffer(uint8_t flags)
{
    switch(flags) {
        case CANINTF_RX0IF:
            return RXB0;
        case CANINTF_RX1IF:
            return RXB1;
        case CANINTF_TX0IF:
            return TXB0;
        case CANINTF_TX1IF:
            return TXB1;
        case CANINTF_TX2IF:
            return TXB2;
    }
    return RETURN_FAIL;
}

/*********************************************************************************************************
** Function name:           checkClearRxStatus
** Descriptions:            Return first found rx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this 
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
uint8_t MCP2515::checkClearRxStatus(uint8_t *status)
{
  uint8_t ret;
  
  ret = *status & CANINTF_RX0IF; *status &= ~CANINTF_RX0IF;
  
  if ( ret==0 ) { ret = *status & CANINTF_RX1IF; *status &= ~CANINTF_RX1IF; }

  return ret;                
}

/*********************************************************************************************************
** Function name:           checkClearTxStatus
** Descriptions:            Return specified buffer of first found tx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this 
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
uint8_t MCP2515::checkClearTxStatus(uint8_t *status, uint8_t iTxBuf)
{
    uint8_t ret;
  
  if ( iTxBuf < MCP_N_TXBUFFERS ) { // Clear specific buffer flag
    ret = *status & txIfFlag(iTxBuf); *status &= ~txIfFlag(iTxBuf);
  } else {
    ret=0;
    for (uint8_t i = 0; i < MCP_N_TXBUFFERS - nReservedTx; i++) {
      ret = *status & txIfFlag(i); 
      if ( ret!=0 ) {
        *status &= ~txIfFlag(i);
        return ret;
      }
    };
  }

  return ret;                
}

// bool MCP2515::checkRETURN(void)
// {
//     uint8_t eflg = getRETURNFlags();

//     if ( eflg & EFLG_RETURNMASK ) {
//         return true;
//     } else {
//         return false;
//     }
// }

// uint8_t MCP2515::getRETURNFlags(void)
// {
//     return readRegister(MCP_EFLG);
// }

// void MCP2515::clearRXnOVRFlags(void)
// {
// 	modifyRegister(MCP_EFLG, EFLG_RX0OVR | EFLG_RX1OVR, 0x00);
// }

// uint8_t MCP2515::getInterrupts(void)
// {
//     return readRegister(MCP_CANINTF);
// }

// void MCP2515::clearInterrupts(void)
// {
//     setRegister(MCP_CANINTF, 0);
// }

// uint8_t MCP2515::getInterruptMask(void)
// {
//     return readRegister(MCP_CANINTE);
// }

// void MCP2515::clearTXInterrupts(void)
// {
//     modifyRegister(MCP_CANINTF, (CANINTF_TX0IF | CANINTF_TX1IF | CANINTF_TX2IF), 0x00);
// }

// void MCP2515::clearRXnOVR(void)
// {
// 	uint8_t eflg = getRETURNFlags();
// 	if (eflg != 0) {
// 		clearRXnOVRFlags();
// 		clearInterrupts();
// 	}
// }

// void MCP2515::clearMERR()
// {
// 	modifyRegister(MCP_CANINTF, CANINTF_MERRF, 0x00);
// }

// void MCP2515::clearERRIF()
// {
//     modifyRegister(MCP_CANINTF, CANINTF_ERRIF, 0x00);
// }

// uint8_t MCP2515::RETURNCountRX(void)                             
// {
//     return readRegister(MCP_REC);
// }

// uint8_t MCP2515::RETURNCountTX(void)                             
// {
//     return readRegister(MCP_TEC);
// }

/*********************************************************************************************************
** Function name:           clearBufferTransmitIfFlags
** Descriptions:            Clear transmit interrupt flags for specific buffer or for all unreserved buffers.
**                          If interrupt will be used, it is important to clear all flags, when there is no
**                          more data to be sent. Otherwise IRQ will newer change state.
*********************************************************************************************************/
void MCP2515::clearBufferTransmitIfFlags(uint8_t flags)
{ 
  flags &= CANINTE_TX_INT;
  if ( flags==0 ) return;
  modifyRegister(MCP_CANINTF, flags, 0x00);
}