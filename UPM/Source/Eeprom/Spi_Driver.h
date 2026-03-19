// ********************************************************************************************************
// *            Spi_Driver.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Spi_Driver.h
// *
// *    DESCRIPTION: Equates for the EEPROM Drivers routines
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *    DATE: 04/15/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************
#ifndef _SPI_DRIVER_H
#define _SPI_DRIVER_H

enum
{
    SPI_EEPROM,
    SPI_PLD,
    SPI_IOX
};

#define PLD_ADDR_STAT1 0
#define PLD_ADDR_VER1 1
#define PLD_ADDR_VER2 2
#define PLD_ADDR_TRAP 3
#define PLD_ADDR_FAN1 4
#define PLD_ADDR_STAT2 5
#define PLD_ADDR_FAN_STS 6
#define PLD_ADDR_READFANA 7
#define PLD_ADDR_READFANB 8
#define PLD_ADDR_READFANC 9
#define PLD_STAT2_BIT_EPOCLEAR BIT0

// io- expander register addresses, with configuration BANK=0
#define IOX_REG_IODIRA 0x0
#define IOX_REG_IODIRB 0x1
#define IOX_REG_IPOLA 0x2
#define IOX_REG_IPOLB 0x3
#define IOX_REG_GPINTENA 0x4
#define IOX_REG_GPINTENB 0x5
#define IOX_REG_DEFVALA 0x6
#define IOX_REG_DEFVALB 0x7
#define IOX_REG_INTCONA 0x8
#define IOX_REG_INTCONB 0x9
#define IOX_REG_IOCON 0x0A
//IOX_REG_IOCON 0x0B also
#define IOX_REG_GPPUA 0x0C
#define IOX_REG_GPPUB 0x0D
#define IOX_REG_INTFA 0x0E
#define IOX_REG_INTFB 0x0F
#define IOX_REG_INTCAPA 0x10
#define IOX_REG_INTCAPB 0x11
#define IOX_REG_GPIOA 0x12
#define IOX_REG_GPIOB 0x13
#define IOX_REG_OLATA 0x14
#define IOX_REG_OLATB 0x15
// bit masks for addressign registers
#define IOX_DEVICE_OP_CODE (BIT14+BIT10) //A1=1
#define IOX_READ BIT8
#define IOX_WRITE 0

    //SpiQueue[].Flags Bits 0-7
#define EE_OP_READ_EEPROM           0x0002          // EEPROM READ:  Use the passed data pointer for storing Read data
#define EE_OP_WRITE_EEPROM          0x0003          // EEPROM WRITE: Use the passed data pointer for write data
#define EE_OP_CLEAR_EEPROM          0x0004          // EEPROM WRITE: Clear the specifed addresses
#define EE_OP_ERASE_EEPROM          0x0005          // EEPROM WRITE: Erase the specifed addresses
#define EE_OP_READ_CHECKSUM         0x0006          // EEPROM READ: Add together all of the values for a checksum
#define EE_OP_UPDATE_CHECKSUM       0x0007          // EEPROM WRITE: Updates the section checksum

#define IOEXPANDER_WRITE            0x0010          // Write OP
#define IOEXPANDER_READ	            0x0011          // Read OP
#define CHECK_SPI_BUS               0xffff
//#define EE_OP_MAX_TYPE              RTC_WRITE_NVRAM // VALID TYPE ARE 0 -- WRITE EEPROM, 1 READ EEPROM
#define EE_OP_MAX_TYPE				CHECK_SPI_BUS
#define PLD_WRITE                   0x000E          // Write
#define PLD_READ                    0x000F
#define PLD_READ_VERSION            0x0012          //

// *********************************************************************************************************
// *    EEPROM DEVICE ACCESS CODES (CODES SENT TO THE DEVICE)
// *********************************************************************************************************
#define READ_EEPROM_COMMAND     0x0300         //Command sent to EEPROM Device
#define WRITE_EEPROM_COMMAND    0x0200         //Command sent to EEPROM Device
#define WRITE_ENABLE_COMMAND    0x0600         //Command sent to EEPROM Device
#define WRITE_DISABLE_COMMAND   0x0400         //Command sent to EEPROM Device
#define READ_STATUS_COMMAND     0x0500         //Command sent to EEPROM Device

#define BOGUS_EEP_DATA          0xaa00         //for read command clocking

// *********************************************************************************************************
// *    EEPROM ACCESS DEFINITIONS - EEPROM is the LC25C640
// *********************************************************************************************************
#define MAX_EE_ADDRESS_B        8191        // size for 8192 x 8bit, valid byte addresses 0..8191
#define MAX_EE_ADDRESS_W        4095        // -"- as 4096 x 16bit, valid word addresses 0..4095

#define EE_PAGESIZE_B           32          // eeprom page size, in bytes
#define EE_PAGESIZE_W           16          // words that fit in a page
#define MAX_EE_LENGTH_B         8192        // max Block operation, 8192 bytes
#define MAX_EE_LENGTH_W         4096        // max Block operation, 4096 words


// ********************************************************************************************************
// * FUNCTION PROTOTYPES, External Data
// ********************************************************************************************************
void read_eeprom_array(uint16_t start_add, uint16_t numwds, uint16_t* dataptr, uint16_t optype);
void write_eeprom_array(uint16_t start_address, uint16_t Num_Words, uint16_t* data_ptr, uint16_t op_type);
uint16_t write_eeprom_sequence( uint16_t start_address, uint16_t num_words, const uint16_t* data_ptr );
uint16_t GetEEPROMStatusRegistor(void);
uint16_t WriteEEROM(uint16_t start_address, uint16_t *data_ptr);
uint16_t ReadEEROM(uint16_t start_address, uint16_t *data_ptr);

void read_IOX_sequence( uint16_t addr, uint16_t *data_ptr);
void write_IOX_sequence( uint16_t addr, const uint16_t *data_ptr);
void Init_IOX(void);
void read_PLD_sequence(uint16_t start_address, uint16_t *data_ptr); // reads i8 from address
void read_PLD_sequence_i16(uint16_t start_address, uint16_t *data_ptr); //cobines data from address a and a+1 to i16
void write_PLD_sequence(uint16_t start_address, uint16_t *data_ptr);


// ********************************************************************************************************
// *            END OF Eeprom_Drv.h
// ********************************************************************************************************
#endif

