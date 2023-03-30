#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "stdbool.h"
#include "stdint.h"
#include "driver/spi_master.h"
#include "can.h"

#define TAG_MCP2515 "MCP2515"
/*
 *  Speed 8M
 */
#define MCP_8MHz_1000kBPS_CFG1 (0x00)
#define MCP_8MHz_1000kBPS_CFG2 (0x80)
#define MCP_8MHz_1000kBPS_CFG3 (0x80)

#define MCP_8MHz_500kBPS_CFG1 (0x00)
#define MCP_8MHz_500kBPS_CFG2 (0x90)
#define MCP_8MHz_500kBPS_CFG3 (0x82)

#define MCP_8MHz_250kBPS_CFG1 (0x00)
#define MCP_8MHz_250kBPS_CFG2 (0xB1)
#define MCP_8MHz_250kBPS_CFG3 (0x85)

#define MCP_8MHz_200kBPS_CFG1 (0x00)
#define MCP_8MHz_200kBPS_CFG2 (0xB4)
#define MCP_8MHz_200kBPS_CFG3 (0x86)

#define MCP_8MHz_125kBPS_CFG1 (0x01)
#define MCP_8MHz_125kBPS_CFG2 (0xB1)
#define MCP_8MHz_125kBPS_CFG3 (0x85)

#define MCP_8MHz_100kBPS_CFG1 (0x01)
#define MCP_8MHz_100kBPS_CFG2 (0xB4)
#define MCP_8MHz_100kBPS_CFG3 (0x86)

#define MCP_8MHz_80kBPS_CFG1 (0x01)
#define MCP_8MHz_80kBPS_CFG2 (0xBF)
#define MCP_8MHz_80kBPS_CFG3 (0x87)

#define MCP_8MHz_50kBPS_CFG1 (0x03)
#define MCP_8MHz_50kBPS_CFG2 (0xB4)
#define MCP_8MHz_50kBPS_CFG3 (0x86)

#define MCP_8MHz_40kBPS_CFG1 (0x03)
#define MCP_8MHz_40kBPS_CFG2 (0xBF)
#define MCP_8MHz_40kBPS_CFG3 (0x87)

#define MCP_8MHz_33k3BPS_CFG1 (0x47)
#define MCP_8MHz_33k3BPS_CFG2 (0xE2)
#define MCP_8MHz_33k3BPS_CFG3 (0x85)

#define MCP_8MHz_31k25BPS_CFG1 (0x07)
#define MCP_8MHz_31k25BPS_CFG2 (0xA4)
#define MCP_8MHz_31k25BPS_CFG3 (0x84)

#define MCP_8MHz_20kBPS_CFG1 (0x07)
#define MCP_8MHz_20kBPS_CFG2 (0xBF)
#define MCP_8MHz_20kBPS_CFG3 (0x87)

#define MCP_8MHz_10kBPS_CFG1 (0x0F)
#define MCP_8MHz_10kBPS_CFG2 (0xBF)
#define MCP_8MHz_10kBPS_CFG3 (0x87)

#define MCP_8MHz_5kBPS_CFG1 (0x1F)
#define MCP_8MHz_5kBPS_CFG2 (0xBF)
#define MCP_8MHz_5kBPS_CFG3 (0x87)

/*
 *  speed 16M
 */
#define MCP_16MHz_1000kBPS_CFG1 (0x00)
#define MCP_16MHz_1000kBPS_CFG2 (0xD0)
#define MCP_16MHz_1000kBPS_CFG3 (0x82)

#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xF0)
#define MCP_16MHz_500kBPS_CFG3 (0x86)

#define MCP_16MHz_250kBPS_CFG1 (0x41)
#define MCP_16MHz_250kBPS_CFG2 (0xF1)
#define MCP_16MHz_250kBPS_CFG3 (0x85)

#define MCP_16MHz_200kBPS_CFG1 (0x01)
#define MCP_16MHz_200kBPS_CFG2 (0xFA)
#define MCP_16MHz_200kBPS_CFG3 (0x87)

#define MCP_16MHz_125kBPS_CFG1 (0x03)
#define MCP_16MHz_125kBPS_CFG2 (0xF0)
#define MCP_16MHz_125kBPS_CFG3 (0x86)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xFA)
#define MCP_16MHz_100kBPS_CFG3 (0x87)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xFF)
#define MCP_16MHz_80kBPS_CFG3 (0x87)

#define MCP_16MHz_83k3BPS_CFG1 (0x03)
#define MCP_16MHz_83k3BPS_CFG2 (0xBE)
#define MCP_16MHz_83k3BPS_CFG3 (0x07)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xFA)
#define MCP_16MHz_50kBPS_CFG3 (0x87)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xFF)
#define MCP_16MHz_40kBPS_CFG3 (0x87)

#define MCP_16MHz_33k3BPS_CFG1 (0x4E)
#define MCP_16MHz_33k3BPS_CFG2 (0xF1)
#define MCP_16MHz_33k3BPS_CFG3 (0x85)

#define MCP_16MHz_20kBPS_CFG1 (0x0F)
#define MCP_16MHz_20kBPS_CFG2 (0xFF)
#define MCP_16MHz_20kBPS_CFG3 (0x87)

#define MCP_16MHz_10kBPS_CFG1 (0x1F)
#define MCP_16MHz_10kBPS_CFG2 (0xFF)
#define MCP_16MHz_10kBPS_CFG3 (0x87)

#define MCP_16MHz_5kBPS_CFG1 (0x3F)
#define MCP_16MHz_5kBPS_CFG2 (0xFF)
#define MCP_16MHz_5kBPS_CFG3 (0x87)

/*
 *  speed 20M
 */
#define MCP_20MHz_1000kBPS_CFG1 (0x00)
#define MCP_20MHz_1000kBPS_CFG2 (0xD9)
#define MCP_20MHz_1000kBPS_CFG3 (0x82)

#define MCP_20MHz_500kBPS_CFG1 (0x00)
#define MCP_20MHz_500kBPS_CFG2 (0xFA)
#define MCP_20MHz_500kBPS_CFG3 (0x87)

#define MCP_20MHz_250kBPS_CFG1 (0x41)
#define MCP_20MHz_250kBPS_CFG2 (0xFB)
#define MCP_20MHz_250kBPS_CFG3 (0x86)

#define MCP_20MHz_200kBPS_CFG1 (0x01)
#define MCP_20MHz_200kBPS_CFG2 (0xFF)
#define MCP_20MHz_200kBPS_CFG3 (0x87)

#define MCP_20MHz_125kBPS_CFG1 (0x03)
#define MCP_20MHz_125kBPS_CFG2 (0xFA)
#define MCP_20MHz_125kBPS_CFG3 (0x87)

#define MCP_20MHz_100kBPS_CFG1 (0x04)
#define MCP_20MHz_100kBPS_CFG2 (0xFA)
#define MCP_20MHz_100kBPS_CFG3 (0x87)

#define MCP_20MHz_83k3BPS_CFG1 (0x04)
#define MCP_20MHz_83k3BPS_CFG2 (0xFE)
#define MCP_20MHz_83k3BPS_CFG3 (0x87)

#define MCP_20MHz_80kBPS_CFG1 (0x04)
#define MCP_20MHz_80kBPS_CFG2 (0xFF)
#define MCP_20MHz_80kBPS_CFG3 (0x87)

#define MCP_20MHz_50kBPS_CFG1 (0x09)
#define MCP_20MHz_50kBPS_CFG2 (0xFA)
#define MCP_20MHz_50kBPS_CFG3 (0x87)

#define MCP_20MHz_40kBPS_CFG1 (0x09)
#define MCP_20MHz_40kBPS_CFG2 (0xFF)
#define MCP_20MHz_40kBPS_CFG3 (0x87)

#define MCP_20MHz_33k3BPS_CFG1 (0x0B)
#define MCP_20MHz_33k3BPS_CFG2 (0xFF)
#define MCP_20MHz_33k3BPS_CFG3 (0x87)


static const uint8_t CANSTAT_OPMOD = 0xE0;
static const uint8_t CANSTAT_ICOD = 0x0E;

static const uint8_t CNF3_SOF = 0x80;

static const uint8_t TXB_EXIDE_MASK = 0x08;
static const uint8_t DLC_MASK       = 0x0F;
static const uint8_t RTR_MASK       = 0x40;

static const uint8_t RXBnCTRL_RXM_STD    = 0x20;
static const uint8_t RXBnCTRL_RXM_EXT    = 0x40;
static const uint8_t RXBnCTRL_RXM_STDEXT = 0x00;
static const uint8_t RXBnCTRL_RXM_MASK   = 0x60;
static const uint8_t RXBnCTRL_RTR        = 0x08;
static const uint8_t RXB0CTRL_BUKT       = 0x04;
static const uint8_t RXB0CTRL_FILHIT_MASK = 0x03;
static const uint8_t RXB1CTRL_FILHIT_MASK = 0x07;
static const uint8_t RXB0CTRL_FILHIT = 0x00;
static const uint8_t RXB1CTRL_FILHIT = 0x01;

static const uint8_t MCP_SIDH = 0;
static const uint8_t MCP_SIDL = 1;
static const uint8_t MCP_EID8 = 2;
static const uint8_t MCP_EID0 = 3;
static const uint8_t MCP_DLC  = 4;
static const uint8_t MCP_DATA = 5;

static const uint8_t CANCTRL_REQOP = 0xE0;
static const uint8_t CANCTRL_ABAT = 0x10;
static const uint8_t CANCTRL_OSM = 0x08;
static const uint8_t CANCTRL_CLKEN = 0x04;
static const uint8_t CANCTRL_CLKPRE = 0x03;

#define N_TXBUFFERS 3
#define N_RXBUFFERS 2

typedef enum {
    MCP_20MHZ,
    MCP_16MHZ,
    MCP_8MHZ
}CAN_CLOCK_t;

typedef enum {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
}CAN_SPEED_t;

typedef enum {
    CLKOUT_DISABLE = -1,
    CLKOUT_DIV1 = 0x0,
    CLKOUT_DIV2 = 0x1,
    CLKOUT_DIV4 = 0x2,
    CLKOUT_DIV8 = 0x3,
}CAN_CLKOUT_t;

typedef enum {
	ERROR_OK        = 0,
	ERROR_FAIL      = 1,
	ERROR_ALLTXBUSY = 2,
	ERROR_FAILINIT  = 3,
	ERROR_FAILTX    = 4,
	ERROR_NOMSG     = 5
}ERROR_t;

typedef enum {
	MASK0,
	MASK1
}MASK_t;

typedef enum {
	RXF0 = 0,
	RXF1 = 1,
	RXF2 = 2,
	RXF3 = 3,
	RXF4 = 4,
	RXF5 = 5
}RXF_t;

typedef enum {
	RXB0 = 0,
	RXB1 = 1
}RXBn_t;

typedef enum {
	TXB0 = 0,
	TXB1 = 1,
	TXB2 = 2
}TXBn_t;

typedef enum {
	CANINTF_RX0IF = 0x01,
	CANINTF_RX1IF = 0x02,
	CANINTF_TX0IF = 0x04,
	CANINTF_TX1IF = 0x08,
	CANINTF_TX2IF = 0x10,
	CANINTF_ERRIF = 0x20,
	CANINTF_WAKIF = 0x40,
	CANINTF_MERRF = 0x80
}CANINTF_t;

typedef enum {
	EFLG_RX1OVR = (uint8_t)0b10000000,
	EFLG_RX0OVR = (uint8_t)0b01000000,
	EFLG_TXBO   = (uint8_t)0b00100000,
	EFLG_TXEP   = (uint8_t)0b00010000,
	EFLG_RXEP   = (uint8_t)0b00001000,
	EFLG_TXWAR  = (uint8_t)0b00000100,
	EFLG_RXWAR  = (uint8_t)0b00000010,
	EFLG_EWARN  = (uint8_t)0b00000001
}EFLG_t;





typedef enum {
	CANCTRL_REQOP_NORMAL     = (uint8_t)0x00,
	CANCTRL_REQOP_SLEEP      = (uint8_t)0x20,
	CANCTRL_REQOP_LOOPBACK   = (uint8_t)0x40,
	CANCTRL_REQOP_LISTENONLY = (uint8_t)0x60,
	CANCTRL_REQOP_CONFIG     = (uint8_t)0x80,
	CANCTRL_REQOP_POWERUP    = (uint8_t)0xE0
}CANCTRL_REQOP_MODE_t;



typedef enum {
	STAT_RX0IF = (uint8_t)(1<<0),
	STAT_RX1IF = (uint8_t)(1<<1)
}STAT_t;



typedef enum {
	TXB_ABTF   = (uint8_t)0x40,
	TXB_MLOA   = (uint8_t)0x20,
	TXB_TXERR  = (uint8_t)0x10,
	TXB_TXREQ  = (uint8_t)0x08,
	TXB_TXIE   = (uint8_t)0x04,
	TXB_TXP    = (uint8_t)0x03
}TXBnCTRL_t;


typedef enum {
	INSTRUCTION_WRITE       = (uint8_t)0x02,
	INSTRUCTION_READ        = (uint8_t)0x03,
	INSTRUCTION_BITMOD      = (uint8_t)0x05,
	INSTRUCTION_LOAD_TX0    = (uint8_t)0x40,
	INSTRUCTION_LOAD_TX1    = (uint8_t)0x42,
	INSTRUCTION_LOAD_TX2    = (uint8_t)0x44,
	INSTRUCTION_RTS_TX0     = (uint8_t)0x81,
	INSTRUCTION_RTS_TX1     = (uint8_t)0x82,
	INSTRUCTION_RTS_TX2     = (uint8_t)0x84,
	INSTRUCTION_RTS_ALL     = (uint8_t)0x87,
	INSTRUCTION_READ_RX0    = (uint8_t)0x90,
	INSTRUCTION_READ_RX1    = (uint8_t)0x94,
	INSTRUCTION_READ_STATUS = (uint8_t)0xA0,
	INSTRUCTION_RX_STATUS   = (uint8_t)0xB0,
	INSTRUCTION_RESET       = (uint8_t)0xC0
}INSTRUCTION_t;

typedef enum {
	MCP_RXF0SIDH = (uint8_t)0x00,
	MCP_RXF0SIDL = (uint8_t)0x01,
	MCP_RXF0EID8 = (uint8_t)0x02,
	MCP_RXF0EID0 = (uint8_t)0x03,
	MCP_RXF1SIDH = (uint8_t)0x04,
	MCP_RXF1SIDL = (uint8_t)0x05,
	MCP_RXF1EID8 = (uint8_t)0x06,
	MCP_RXF1EID0 = (uint8_t)0x07,
	MCP_RXF2SIDH = (uint8_t)0x08,
	MCP_RXF2SIDL = (uint8_t)0x09,
	MCP_RXF2EID8 = (uint8_t)0x0A,
	MCP_RXF2EID0 = (uint8_t)0x0B,
	MCP_CANSTAT  = (uint8_t)0x0E,
	MCP_CANCTRL  = (uint8_t)0x0F,
	MCP_RXF3SIDH = (uint8_t)0x10,
	MCP_RXF3SIDL = (uint8_t)0x11,
	MCP_RXF3EID8 = (uint8_t)0x12,
	MCP_RXF3EID0 = (uint8_t)0x13,
	MCP_RXF4SIDH = (uint8_t)0x14,
	MCP_RXF4SIDL = (uint8_t)0x15,
	MCP_RXF4EID8 = (uint8_t)0x16,
	MCP_RXF4EID0 = (uint8_t)0x17,
	MCP_RXF5SIDH = (uint8_t)0x18,
	MCP_RXF5SIDL = (uint8_t)0x19,
	MCP_RXF5EID8 = (uint8_t)0x1A,
	MCP_RXF5EID0 = (uint8_t)0x1B,
	MCP_TEC      = (uint8_t)0x1C,
	MCP_REC      = (uint8_t)0x1D,
	MCP_RXM0SIDH = (uint8_t)0x20,
	MCP_RXM0SIDL = (uint8_t)0x21,
	MCP_RXM0EID8 = (uint8_t)0x22,
	MCP_RXM0EID0 = (uint8_t)0x23,
	MCP_RXM1SIDH = (uint8_t)0x24,
	MCP_RXM1SIDL = (uint8_t)0x25,
	MCP_RXM1EID8 = (uint8_t)0x26,
	MCP_RXM1EID0 = (uint8_t)0x27,
	MCP_CNF3     = (uint8_t)0x28,
	MCP_CNF2     = (uint8_t)0x29,
	MCP_CNF1     = (uint8_t)0x2A,
	MCP_CANINTE  = (uint8_t)0x2B,
	MCP_CANINTF  = (uint8_t)0x2C,
	MCP_EFLG     = (uint8_t)0x2D,
	MCP_TXB0CTRL = (uint8_t)0x30,
	MCP_TXB0SIDH = (uint8_t)0x31,
	MCP_TXB0SIDL = (uint8_t)0x32,
	MCP_TXB0EID8 = (uint8_t)0x33,
	MCP_TXB0EID0 = (uint8_t)0x34,
	MCP_TXB0DLC  = (uint8_t)0x35,
	MCP_TXB0DATA = (uint8_t)0x36,
	MCP_TXB1CTRL = (uint8_t)0x40,
	MCP_TXB1SIDH = (uint8_t)0x41,
	MCP_TXB1SIDL = (uint8_t)0x42,
	MCP_TXB1EID8 = (uint8_t)0x43,
	MCP_TXB1EID0 = (uint8_t)0x44,
	MCP_TXB1DLC  = (uint8_t)0x45,
	MCP_TXB1DATA = (uint8_t)0x46,
	MCP_TXB2CTRL = (uint8_t)0x50,
	MCP_TXB2SIDH = (uint8_t)0x51,
	MCP_TXB2SIDL = (uint8_t)0x52,
	MCP_TXB2EID8 = (uint8_t)0x53,
	MCP_TXB2EID0 = (uint8_t)0x54,
	MCP_TXB2DLC  = (uint8_t)0x55,
	MCP_TXB2DATA = (uint8_t)0x56,
	MCP_RXB0CTRL = (uint8_t)0x60,
	MCP_RXB0SIDH = (uint8_t)0x61,
	MCP_RXB0SIDL = (uint8_t)0x62,
	MCP_RXB0EID8 = (uint8_t)0x63,
	MCP_RXB0EID0 = (uint8_t)0x64,
	MCP_RXB0DLC  = (uint8_t)0x65,
	MCP_RXB0DATA = (uint8_t)0x66,
	MCP_RXB1CTRL = (uint8_t)0x70,
	MCP_RXB1SIDH = (uint8_t)0x71,
	MCP_RXB1SIDL = (uint8_t)0x72,
	MCP_RXB1EID8 = (uint8_t)0x73,
	MCP_RXB1EID0 = (uint8_t)0x74,
	MCP_RXB1DLC  = (uint8_t)0x75,
	MCP_RXB1DATA = (uint8_t)0x76
}REGISTER_t;

static const uint32_t SPI_CLOCK = 10000000; // 10MHz
static const uint8_t STAT_RXIF_MASK = STAT_RX0IF | STAT_RX1IF;
static const uint8_t EFLG_ERRORMASK = EFLG_RX1OVR
									| EFLG_RX0OVR
									| EFLG_TXBO
									| EFLG_TXEP
									| EFLG_RXEP;

typedef struct TXBn_REGS_s {
	REGISTER_t CTRL;
	REGISTER_t SIDH;
	REGISTER_t DATA;
} TXBn_REGS_t[1], *TXBn_REGS;

typedef struct RXBn_REGS_s {
	REGISTER_t CTRL;
	REGISTER_t SIDH;
	REGISTER_t DATA;
	CANINTF_t  CANINTF_RXnIF;
} RXBn_REGS_t[1], *RXBn_REGS;


typedef struct MCP2515_s{
	ERROR_t ERROR;
	MASK_t MASK;
	RXF_t RXF;
	RXBn_t RXBn;
	TXBn_t TXBn;
	CANINTF_t CANINTF;
	EFLG_t EFLG_t;
	CANCTRL_REQOP_MODE_t CANCTRL_REQOP_MODE;
	STAT_t STAT;
	TXBnCTRL_t TXBnCTRL;
	INSTRUCTION_t INSTRUCTION;
	REGISTER_t REGISTER;

	TXBn_REGS TXB_ptr;
	RXBn_REGS RXB_ptr;

	spi_device_handle_t spi;
}MCP2515_t[1], *MCP2515;

ERROR_t MCP2515_setMode(const CANCTRL_REQOP_MODE_t mode);

uint8_t MCP2515_readRegister(const REGISTER_t reg);
void MCP2515_readRegisters(const REGISTER_t reg, uint8_t values[], const uint8_t n);
void MCP2515_setRegister(const REGISTER_t reg, const uint8_t value);
void MCP2515_setRegisters(const REGISTER_t reg, const uint8_t values[], const uint8_t n);
void MCP2515_modifyRegister(const REGISTER_t reg, const uint8_t mask, const uint8_t data);

void MCP2515_prepareId(uint8_t *buffer, const bool ext, const uint32_t id);

ERROR_t MCP2515_init();
ERROR_t MCP2515_reset(void);
ERROR_t MCP2515_setConfigMode();
ERROR_t MCP2515_setListenOnlyMode();
ERROR_t MCP2515_setSleepMode();
ERROR_t MCP2515_setLoopbackMode();
ERROR_t MCP2515_setNormalMode();
ERROR_t MCP2515_setOneShotMode(bool set);
ERROR_t MCP2515_setClkOut(const CAN_CLKOUT_t divisor);
ERROR_t MCP2515_setBitrate(const CAN_SPEED_t canSpeed, const CAN_CLOCK_t canClock);
ERROR_t MCP2515_setFilterMask(const MASK_t num, const bool ext, const uint32_t ulData);
ERROR_t MCP2515_setFilter(const RXF_t num, const bool ext, const uint32_t ulData);
ERROR_t MCP2515_sendMessage(const TXBn_t txbn, const CAN_FRAME frame);
ERROR_t MCP2515_sendMessageAfterCtrlCheck(const CAN_FRAME frame);
ERROR_t MCP2515_readMessage(const RXBn_t rxbn, const CAN_FRAME frame);
ERROR_t MCP2515_readMessageAfterStatCheck(const CAN_FRAME frame);
bool MCP2515_checkReceive(void);
bool MCP2515_checkError(void);
uint8_t MCP2515_getErrorFlags(void);
void MCP2515_clearRXnOVRFlags(void);
uint8_t MCP2515_getInterrupts(void);
uint8_t MCP2515_getInterruptMask(void);
void MCP2515_clearInterrupts(void);
void MCP2515_clearTXInterrupts(void);
uint8_t MCP2515_getStatus(void);
void MCP2515_clearRXnOVR(void);
void MCP2515_clearMERR();
void MCP2515_clearERRIF();

extern MCP2515 MCP2515_Object;


#endif
