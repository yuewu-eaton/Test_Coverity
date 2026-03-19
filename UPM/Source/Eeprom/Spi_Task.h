#ifndef _SPI_PORT_H
#define _SPI_PORT_H
// ********************************************************************************************************
// *            SPI_PORT.h
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
// *    FILE NAME: SPI_PORT.h
// *
// *    DESCRIPTION: Equates for the SPI PORT DRIVER routines
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *    DATE: 05/15/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

#include "Spi_Driver.h"
#include "Parameters.h"
#include "Queue.h"

// ********************************************************************************************************
//      Global data
// ********************************************************************************************************
extern uint16_t EepResetNoSyncTimer;

struct EepAddrValue
{
	uint16_t addr;
	uint16_t value;
};
extern bool disableSPIQueue;

// ********************************************************************************************************
// * FUNCTION PROTOTYPES
// ********************************************************************************************************

uint16_t GetEepData( uint16_t startAddress, uint16_t numWords, uint16_t* dataPtr, uint16_t ticksToWait );
bool PendEepRead(uint16_t addr, QueueHandle receiver);
uint16_t PutEepData( uint16_t startAddress, uint16_t numWords, uint16_t* dataPtr, uint16_t ticksToWait );
uint16_t EraseEepData( uint16_t startAddress, uint16_t numWords, uint16_t ticksToWait );

uint16_t WriteParameter( parameter_t param, void* data, uint16_t numWords );
uint16_t ReadParameter( parameter_t param, void* data, uint16_t numWords );

void check_valid_eeprom( uint16_t start_address, uint16_t num_words, uint16_t* data_ptr );
void ReBootEepromSection( uint16_t section );
void QueueSpiNoFeedB( uint16_t start_address, uint16_t num_words, uint16_t* data_ptr, uint16_t op_type);

// ********************************************************************************************************
// *            END OF SPI_PORT.h
// ********************************************************************************************************
#endif

