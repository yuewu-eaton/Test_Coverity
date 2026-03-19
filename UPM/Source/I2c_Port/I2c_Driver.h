
// ******************************************************************************************************
// *            I2c_Deriver.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: I2c_Driver.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR:
// *
// *    DATE: 3/11/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#ifndef _I2c_Driver_H
#define _I2c_Driver_H

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************   

// I2c Device ID
#define cDevice_Null          0x00
#define cDevice_PCF8575       0x01
#define cDevice_PLD           0x02
#define cDevice_PLD_TRAP      0x03
#define cDevice_SHA204        0x04
#define cDevice_TPT29555      0x05

// I2c Device Address
#define cAddress_PCF8575      0x22
#define cAddress_TPT29555     (0x4C>>1)
#define cAddress_PLD          0x25
#define cAddress_PLD_TRAP     0x21
#define cAddress_SHA204       (0xC8>>1) // In 7 bit address mode use 0-6bit of I2caRegs.I2CSAR

// I2c Operation Type
#define cOperation_Null       0x00 
#define cOperation_Write      0x01
#define cOperation_Read       0x02

// I2c Interrupt Source
#define cI2c_Isr_SCD          0x06
#define cI2c_Isr_ARDY         0x03

#define I2c_FClk             400000    // I2C module speed 400K
#define I2c_Bit_Cycle        CPUFrequency / I2c_FClk
#define cI2c_Byte_Timeout    (  8 * I2c_Bit_Cycle * 2 ) // Multiply 2 to ensure time is out.
#define cI2c_Word_Timeout    ( 16 * I2c_Bit_Cycle * 2 )

enum I2CError {
	I2C_ERROR_TIMEOUT = 1, // Module timeout
	I2C_ERROR_ARBITRATION = 2, // Arbitration lost
	I2C_ERROR_NACK_PLD = 3, // PLD NACKed us, or IOX value
	I2C_ERROR_RSFULL = 4, // Recieve FIFO overrun
	I2C_ERROR_NACK_IOX = 5, // IOExpander NACKed us
	I2C_ERROR_BUSY = 6, // Start condition set, but not by us
	I2C_ERROR_ESCD = 7,  // Stop condition set, but not by us
	I2C_ERROR_NACK_SHA204 = 8 // SHA204 NACKed us
};

extern uint16_t I2C_error_counters[8];
extern uint16_t I2c_Operation;
extern bool I2c_Enable;

// function prototypes
void Init_I2c_Gpio(void);
void Init_I2c_Register(void);
void I2c_StateCheck(void);
bool I2c_Write_Read(uint16_t device,uint16_t operation,uint16_t* data, uint16_t count = 1);

void I2c_Init_for_TRAP(void);
void I2c_Write_Read_for_TRAP(uint16_t device,uint16_t operation,uint16_t data,uint16_t retry);

uint16_t I2c_State_Check_for_TRAP(void);

bool I2c_Get_Idle_Status(void);

#endif
// ******************************************************************************************************
// *            End of I2c_Driver.h
// ******************************************************************************************************

