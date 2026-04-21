
// ******************************************************************************************************
// *            SPI_TASK.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: SPI_TASK.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *    DATE: 5/20/2003
// *
// *    HISTORY: See Visual Source Safe history.
// ******************************************************************************************************


// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "Spi_Task.h"
#include "Spi_Driver.h"
#include "Eeprom_Map.h"
#include "Queue.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "Rtc.h"
#include "DebuggerBlocks.h"
#include "IOexpansion.h"
#include "ParallelCan.h"
#include "tsk.h"
#include <string.h>

#define EEPROM_BUFFER_SIZE          128

// ********************************************************************************************************
// * Global data
// ********************************************************************************************************
uint16_t EE_Dummy;                                    // garbage variable
uint16_t EepromBuffer[EEPROM_BUFFER_SIZE];
bool disableSPIQueue = false;

uint16_t EepResetNoSyncTimer = 0;

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES AND TYPE DEFINITIONS
// ********************************************************************************************************
extern "C"
{
    void Spi_Task( void );
    void InitEepromSystemType( void );
}

namespace {

enum spi_feedback_tag
{
    FEEDBACK_NONE,
    FEEDBACK_SEMAPHORE,
    FEEDBACK_CALLBACK,
    FEEDBACK_QUEUE
};

union spi_feedback_t
{
    void*        None;
    SEM_Handle   Semaphore;
    void         (*Callback)(void *cbdata);
    QueueHandle  Queue;
    uint16_t     Ticks;
    
    spi_feedback_t(): None(NULL) {}
    spi_feedback_t(void (*callback)(void *cbdata)): Callback(callback) {}
    spi_feedback_t(QueueHandle queue) : Queue(queue) {}
    spi_feedback_t(uint16_t ticks) : Ticks(ticks) {}
};

struct SPI_QUEUE_ENTRY
{
    uint16_t    Flag;            // Pointer to a results/Status Flag
    uint16_t    StartAddress;    // beginning address of the read or write operation
    uint16_t    Length;          //
    uint16_t    *PtrToData;      // Points to Data to write or where to put read data
    // The reporting type tag uniquely identifies the means of reporting that the
    // operation is complete.
    spi_feedback_tag ReportingType;
    spi_feedback_t Reporting;    // How to report that the work is complete.
};

bool QueueSpi( uint16_t start_address, 
    uint16_t num_words, 
    uint16_t* data_ptr, 
    uint16_t op_type,
    spi_feedback_tag type,
    spi_feedback_t feedback);

// local func to handle task requests and block until their completion
uint16_t HandleEepDataRequest( uint16_t startAddress, 
    uint16_t numWords, 
    uint16_t* dataPtr, 
    uint16_t opType, 
    spi_feedback_tag fbType,
    spi_feedback_t fb );

void EE_execute( uint16_t start_address, uint16_t* dataPtr );
void EE_versionUpdate( void );
void ClearEepromBuffer( void );
void UpdateSectionCheckSum( uint16_t section );

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
const uint16_t SPI_QUEUE_SIZE = 40;                     // 40 entries max for the EEPROM Queue

QueueHandle SpiQueue;

} // !namespace (anon)

// ****************************************************************************
// *
// *  Function      :  InitEepromOutputkVARating()
// *
// *  Purpose       :  Init Eeprom OutputkVARating to distinguish different system
// *
// *  Parms Passed  :  none
// *
// *  Returns       :   None
// *
// *  Description:   This module distinguishes different system and initializes parameters.
// *
// *
// *
// *
// ****************************************************************************
void InitEepromSystemType( void )
{
    // index to the EE table OutputKVARating
    uint16_t eeIdx = FindEepromTableIndex( PARAM_SystemType );
    // check for valid index
    if ( eeIdx < GetEETableSize() )
    {
        const EE_ID* ee = &eeprom[ eeIdx ];  //Jacob/20130814/Merge 120k// Keming/20120803I, change Eeprom map name
        // read eeprom data into EEPROM buffer
        ReadEEROM(ee->eep_addr, EepromBuffer);
        // call the function
        uint16_t eepFunctionIndex = ee->eep_functionIndex & ~EE_PARALLEL;
        EEP_PTR const eepFunc = Eep_Functions[ eepFunctionIndex ];

        ( eepFunc )( (EE_ID*)ee, EepromBuffer );
    }
}

// ****************************************************************************
// *
// *  Function        : UpdateRam
// *
// *  Description     : Re-run the update function for a particular RAM variable
// *
// *  Parms Passed    : Parameter number
// *
// *  Returns         : Nothing
// *
// ****************************************************************************
void UpdateRam( parameter_t param )
{
    EE_ID* ee = (EE_ID *)NULL;
    uint16_t data = 0;

    ee = GetParameterEE( param );
    if ( NULL != ee )
    {
        // don't use GetEepData or the spi task will block itself
        read_eeprom_array( ee->eep_addr, ee->eep_length, &data, EE_OP_READ_EEPROM );
        EE_execute( ee->eep_addr, &data); //Update RAM, no EEP sync feedback
    }
}

// ****************************************************************************
// *
// *  Function      :  Initialize EEPROM Data
// *
// *  Purpose       :  Looks at Ee_Map and initializes the system with data from EEPROM
// *
// *  Parms Passed  :  none
// *
// *  Returns       :   None
// *
// *  Description:   This module initializes data from the EEPROM. The following occurrs.
// *
// *
// *
// *
// ****************************************************************************
void InitEepromData()
{
    uint16_t section = EE_System_ID;
    
    while ( section < EE_Last_Section )
    {
        if ( CheckChecksumProtected( section ) )
        {
            uint16_t start = GetSectionAddress( section );
            uint16_t length = GetSectionLength( section );
            uint16_t ee_chksum = 0;
            
            // verify checksum of this section
            read_eeprom_array( start, length, &ee_chksum, EE_OP_READ_CHECKSUM );
            
            if ( 0 == ee_chksum )
            {
                // good section, load data
                ++start;            // index past MY_EE_ID
                length -= 2;        // length - MY_EE_ID - checksum
                
                while ( length )
                {
                    // index to the EE table
                    uint16_t eeIdx = FindEepromTableIndex( start );
                    // check for valid index
                    if ( eeIdx < GetEETableSize() )
                    {
                        const EE_ID* ee = &eeprom[ eeIdx ];
                        // read eeprom data into EEPROM buffer
                        read_eeprom_array( ee->eep_addr, ee->eep_length, EepromBuffer, EE_OP_READ_EEPROM );
                        // call the function
                        uint16_t eepFunctionIndex = ee->eep_functionIndex & ~EE_PARALLEL;
                        EEP_PTR const eepFunc = Eep_Functions[ eepFunctionIndex ];
                                                
                        ( eepFunc )( (EE_ID*)ee, EepromBuffer );
                        
                        // increment address, decrement length
                        start += ee->eep_length;
                        if ( length > ee->eep_length )
                        {
                            length -= ee->eep_length;
                        }
                        else
                        {
                            length = 0;
                        }        
                    }
                    else
                    {
                        // skip unused location
                        ++start;
                        --length;
                    }    
                }    
            }
            else
            {
                EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] |= ( 1 << section );
                
                // load defaults to RAM
                ++start;            // index past MY_EE_ID
                length -= 2;        // length - MY_EE_ID - checksum
                
                while ( length )
                {
                    // index to the EE table
                    uint16_t eeIdx = FindEepromTableIndex( start );
                    // check for valid index
                    if ( eeIdx < GetEETableSize() )
                    {
                        const EE_ID* ee = &eeprom[ eeIdx ];

                        // read eeprom data into EEPROM buffer
                        if ( 1 == ee->eep_length )
                        {
                            EepromBuffer[0] = ee->eep_DefaultValue[CoefficientsIndex];
                        }
                        // todo, handle multi-word data
                        
                        // call the function
                        EEP_PTR const eepFunc = Eep_Functions[ ee->eep_functionIndex & ~EE_PARALLEL ];
                        
                        if(ee->paramNum != PARAM_SystemType)  // Do not execute system type function to avoid system update incorrectly.
                        {
                            ( eepFunc )( (EE_ID*)ee, EepromBuffer );
                        }
                        
                        // increment address, decrement length
                        start += ee->eep_length;
                        if ( length > ee->eep_length )
                        {
                            length -= ee->eep_length;
                        }
                        else
                        {
                            length = 0;
                        }        
                    }
                    else
                    {
                        // skip unused location
                        ++start;
                        --length;
                    }    
                }    
            }
        }
        
        ++section;
    }
    
    if ( ( 0 == EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] ) &&
         ( UPMSystem != SystemInvalid ) )
    {
        // don't update if there's a problem
        EE_versionUpdate();
    }    
        
    EEStatusBits.bit.EEDataInitialized = 1;
}




// ****************************************************************************
// *
// *  Function: SpiTask
// *
// *  Purpose :    To control operation of the SPI device.
// *
// *  Parms Passed   :  Nothing
// *
// *
// *  Returns        :  Nothing
// *
// *  Description:    This module checks the SPI Queue to determine if an SPI
// *                Operation is requested. If so, it executes the request.
// *                A 5-10 Msec suspend occurs after each queue entry is serviced.
// *
// ****************************************************************************
void Spi_Task(void)
{
    SPI_QUEUE_ENTRY ee_op;

    // initialize queue
    SpiQueue = QueueCreate( SPI_QUEUE_SIZE, sizeof( SPI_QUEUE_ENTRY ) );
    // clear status words
    EEStatusBits.w[0] = 0;
    EEStatusBits.w[1] = 0;
    EEStatusBits.w[2] = 0;
    
    //initial IO expander
    Init_IOX();

    // Wait Expansion I/O ready, some EEP functions need it
    if( !ExpansionInputReg.Ready )
    {
        TSK_sleep( TSK_5_ms );
    }

    // initialize eeprom data    
    InitEepromData();
    
    // initialize ups id
    InitUpsId();
    
    const uint16_t maxSyncRead = 64;
    uint16_t syncReadBuffer[64];
    
    for ( ; ; )
    {
        // Block until somebody posts SPI operation or timeout
        if ( QueueReceive( SpiQueue, &ee_op, SYS_FOREVER ) )
        {
            switch ( ee_op.Flag )
            {
                case EE_OP_READ_EEPROM:
                case EE_OP_READ_CHECKSUM: 
                {
                    if (!ee_op.PtrToData                            && 
                        ee_op.Length < maxSyncRead                  &&
                        ( ee_op.ReportingType == FEEDBACK_CALLBACK) ||
                          ee_op.ReportingType == FEEDBACK_QUEUE)
                    {
                        memset(syncReadBuffer, 0, sizeof(syncReadBuffer));
                        ee_op.PtrToData = syncReadBuffer;
                    }
                    read_eeprom_array( ee_op.StartAddress, ee_op.Length, ee_op.PtrToData, ee_op.Flag );
                    break;
                }
                case EE_OP_WRITE_EEPROM:
                    check_valid_eeprom( ee_op.StartAddress, ee_op.Length, ee_op.PtrToData );
                    //lint -fallthrough
                case EE_OP_CLEAR_EEPROM:
                case EE_OP_ERASE_EEPROM:
                    write_eeprom_array( ee_op.StartAddress, ee_op.Length, ee_op.PtrToData, ee_op.Flag );
                    // execute function if this is a write
                    if ( EE_OP_WRITE_EEPROM == ee_op.Flag )
                    {
                        for( uint16_t cnt = 0; cnt < ee_op.Length; cnt++ )
                        {
                            EE_execute( ee_op.StartAddress++, ee_op.PtrToData++ );
                        }
                    }    
                    break;
                    
                case EE_OP_UPDATE_CHECKSUM:
                    // start address is section number for this one
                    if ( ( ee_op.StartAddress < EE_Last_Section ) && CheckChecksumProtected( ee_op.StartAddress ) )
                    {
                        UpdateSectionCheckSum( ee_op.StartAddress );
                    }
                    break;          
                    
                case IOEXPANDER_WRITE:
                     write_IOX_sequence( IOX_REG_GPIOA, ee_op.PtrToData);
                     break;
                case IOEXPANDER_READ:
                     read_IOX_sequence( IOX_REG_GPIOA, ee_op.PtrToData);
                     break;
                     
                case PLD_READ:
                    read_PLD_sequence(ee_op.StartAddress, ee_op.PtrToData);
                    break;
                case PLD_READ_VERSION:      //reads 2 bytes so this is different
                    read_PLD_sequence_i16(ee_op.StartAddress, ee_op.PtrToData);
                    break;
                case PLD_WRITE:
                    write_PLD_sequence(ee_op.StartAddress, ee_op.PtrToData);
                    break;

                default:
                    break;
                       
            }  // end switch ( op_type )
            
            switch (ee_op.ReportingType)
            {
                case FEEDBACK_SEMAPHORE:
                    if (ee_op.Reporting.Semaphore != NULL)
                    {
                        SEM_post( ee_op.Reporting.Semaphore);
                    }
                    break;
                
                case FEEDBACK_CALLBACK:
                    ee_op.Reporting.Callback(ee_op.PtrToData);
                    break;
                    
                case FEEDBACK_QUEUE:
                    if (ee_op.Reporting.Queue != NULL)
                    {
                        bool spaceRemaining = true;
                        for (uint16_t i = 0; i < ee_op.Length && spaceRemaining; ++i)
                        {
                            EepAddrValue result;
                            result.addr = ee_op.StartAddress + i;
                            result.value = ee_op.PtrToData[i];
                            spaceRemaining = QueueSend(ee_op.Reporting.Queue, &result, 0);
                        }
                    }
                    break;
                    
                case FEEDBACK_NONE:
                    break;
                    
                default:
                    break;
            } 
        }
    }
}

// ****************************************************************************
// *
// *  Function: check_valid_eeprom
// *
// *  Purpose :    Check and protect important eeprom when write eeprom
// *
// *  Parms Passed   : start_address -- The beginning address in the EEPROM to Read
// *
// *                   num_words     -- The number of 16 bit words to Read
// *
// *                   *data_ptr     -- A pointer to a data array of size num_words or larger.
// *                                   This is where the data is written to from the EEPROM.
// *                                   data_ptr is only required if op_type is USE_DATA_PTR.
// *
// *  Returns        :    (result) Results Flag formatted defined in EEPROM_DRV.H
// *
// *  Description:    In order to pretect eeprom from being changed to invalid value, check important
// *                eeprom when execute EE_OP_WRITE_EEPROM. These eeps include Output voltage and Output
// *                frequency.
// *
// ****************************************************************************
void check_valid_eeprom( uint16_t start_address, uint16_t num_words, uint16_t* data_ptr )
{
    for( uint16_t cnt = 0; cnt < num_words; cnt++ )
    {
       switch ( start_address )
       {
           case PARAM_OutNomVolts:  // Check ouput voltage
                if ( ( *data_ptr >= 1190 ) && ( *data_ptr <= 1210 ) )      // 120V
                {
                    *data_ptr = 1200;
                }
                else if ( ( *data_ptr >= 1260 ) && ( *data_ptr <= 1280 ) ) // 127V
                {
                    *data_ptr = 1270;
                }
                else if ( ( *data_ptr >= 2190 ) && ( *data_ptr <= 2210 ) ) // 220V
                {
                    *data_ptr = 2200;
                }
                else if ( ( *data_ptr >= 2290 ) && ( *data_ptr <= 2310 ) ) // 230V
                {
                    *data_ptr = 2300;
                }
                else if ( ( *data_ptr >= 2390 ) && ( *data_ptr <= 2410 ) ) //240V
                {
                    *data_ptr = 2400;
                }
                else
                {
                    *data_ptr = OutNomVolts;
                }
                break;
            case PARAM_OutFrequency:  // Check output frequency
                if ( ( *data_ptr != 50 ) &&
                     ( *data_ptr != 60 ) )
                {
                    *data_ptr = OutNomFreq;
                }                   
                break;
            case PARAM_OutKVA:       //if OutputKVARating is invalid ,it shouldn't be written to EEP
                if( CheckValidOutputKVARating ( *data_ptr ) == false )
                {
                    *data_ptr = OutputkVARating;
                }                
                break;
            default:
                break;
        }
        
        *data_ptr++;
        start_address++;
    }
}

// ***********************************************************************************************************
// *
// *  Function: INT16U PutEepData(INT16U startAddress, INT16U numWords, INT16U *dataPtr, INT16U maxRetry)
// *
// *  Purpose : Function to write data to Eeprom - DOES NOT RETURN UNTIL DATA IS WRITTEN.
// *
// *  Parms Passed   :  startAddress:   address of first Eeprom location to write
// *                    numWords2Get:   how many words to write
// *                    *dataPtr:       pointer to buffer containing data to write
// *                    maxRetry:       max number of retries in case Eeprom cannot be written to on first try
// *
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    0 = no error,
// *                                    5 = Operation failed
// *
// *  Description:      This module only returns after  the data has been written, or when failed.
// *
// *                    MUST BE CALLED ONLY FROM TASKS, SINCE IT INVOKES THE SLEEP FUNCTION
// *
// **********************************************************************************************************
uint16_t PutEepData( uint16_t startAddress, uint16_t numWords, uint16_t* dataPtr, uint16_t ticksToWait )
{
	if(disableSPIQueue)//Wombat238 follow 9P-1788 P9P100-452
	{
		return false;
	}	
    return HandleEepDataRequest( startAddress, numWords, dataPtr, EE_OP_WRITE_EEPROM, FEEDBACK_SEMAPHORE, ticksToWait );
}

// ***********************************************************************************************************
// *
// *  Function: uint16_t GetEepData(uint16_t startAddress, uint16_t numWords, uint16_t *dataPtr, uint16_t maxRetry)
// *
// *  Purpose : Function to rewad data from Eeprom - DOES NOT RETURN UNTIL DATA IS Read.
// *
// *  Parms Passed   :  startAddress:   address of first Eeprom location to write
// *                    numWords2Get:   how many words to write
// *                    *dataPtr:       pointer to buffer containing data to write
// *                    maxRetry:       max number of retries in case Eeprom cannot be written to on first try
// *
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    0 = no error,
// *                                    5 = Operation failed
// *
// *  Description:      This module only returns after  the data has been written, or when failed.
// *
// *                    MUST BE CALLED ONLY FROM TASKS, SINCE IT INVOKES THE SLEEP FUNCTION
// *
// **********************************************************************************************************
uint16_t GetEepData( uint16_t startAddress, uint16_t numWords, uint16_t* dataPtr, uint16_t ticksToWait )
{
	if(disableSPIQueue)//Wombat238 follow 9P-1788 P9P100-452
	{
		return false;
	}		
    return HandleEepDataRequest( startAddress, numWords, dataPtr, 
        EE_OP_READ_EEPROM, FEEDBACK_SEMAPHORE, ticksToWait );
}

// ***********************************************************************************************************
// *
// *  Function: bool PendEepRead( uint16_t addr, QueueHandle receiver)
// *
// *  Purpose : Function to read data from Eeprom - This version only reads a single
// *            value from EEPROM.  The result is enqueued to the handle provided by
// *            the caller at some later time, or not at all.
// *
// *  Parms Passed   :  startAddress:   address of the Eeprom location to read
// *                    receiver:       The queue that will receive the value read.
// *                                    A single value of type EepAddrValue will be
// *                                    enqueued to this handle.
// *
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    false: request was enqueued.,
// *                                    true:  request was not enqueued
// *
// *
// **********************************************************************************************************
bool PendEepRead( uint16_t addr, QueueHandle receiver)
{
	if(disableSPIQueue)//Wombat238 follow 9P-1788 P9P100-452
	{
		return false;
	}		
    return HandleEepDataRequest( addr, 1, NULL,
        EE_OP_READ_EEPROM, FEEDBACK_QUEUE, receiver);
}

// ***********************************************************************************************************
// *
// *  Function: uint16_t EraseEepData(uint16_t startAddress, uint16_t numWords, uint16_t *dataPtr, uint16_t maxRetry)
// *
// *  Purpose : Function to Erase data in Eeprom - DOES NOT RETURN UNTIL DATA IS WRITTEN.
// *
// *  Parms Passed   :  startAddress:   address of first Eeprom location to write
// *                    numWords2Get:   how many words to write
// *                    *dataPtr:       pointer to buffer containing data to write
// *                    maxRetry:       max number of retries in case Eeprom cannot be written to on first try
// *
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    0 = no error,
// *                                    5 = Operation failed
// *
// *  Description:      This module only returns after  the data (0xffff) has been written, or when failed.
// *
// *                    MUST BE CALLED ONLY FROM TASKS, SINCE IT INVOKES THE SLEEP FUNCTION
// *
// **********************************************************************************************************
uint16_t EraseEepData( uint16_t startAddress, uint16_t numWords, uint16_t ticksToWait )
{
    return HandleEepDataRequest( startAddress, numWords, NULL, 
        EE_OP_ERASE_EEPROM, FEEDBACK_SEMAPHORE, ticksToWait );
}

// ***********************************************************************************************************
// *
// *  Function: uint16_t ReBootEepromSection(uint16_t section, uint16_t maxRetry)
// *
// *  Purpose : Function to reboot eeprom section - DOES NOT RETURN UNTIL DATA IS WRITTEN.
// *
// *  Parms Passed   :  section:        section to reboot
// *                    maxRetry:       max number of retries in case Eeprom cannot be written to on first try
// *
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    0 = no error,
// *                                    5 = Operation failed
// *
// *  Description:      This module only returns after  the data (0xffff) has been written, or when failed.
// *                    Recalculates and writes new checksum for a section.
// *                    MUST BE CALLED ONLY FROM TASKS, SINCE IT INVOKES THE SLEEP FUNCTION
// *
// **********************************************************************************************************
void ReBootEepromSection( uint16_t section )
{
    // only reboot checksum sections
    if ( CheckChecksumProtected( section ) )
    {
		EepResetNoSyncTimer += 5; // disable EEP sync 5s for every section when reset
        uint16_t start = GetSectionAddress( section );
        uint16_t length = GetSectionLength( section ) - 2;        // size - EE_ID - checksum
        uint16_t checksum = MY_EE_ID;
        
        // write EE ID
        EE_Dummy = MY_EE_ID;
        PutEepData( start, 1, &EE_Dummy, TSK_1000_ms );
        ++start;
        
        // default the rest of the data
        while ( length )
        {
            uint16_t eeIdx = FindEepromTableIndex( start );
            
            if ( eeIdx < GetEETableSize() )
            {
                const EE_ID* ee = &eeprom[ eeIdx ];

                if ( 1 == ee->eep_length )
                {
                    EE_Dummy = ee->eep_DefaultValue[CoefficientsIndex];
                    checksum += EE_Dummy;     
                    PutEepData( start, 1, &EE_Dummy, TSK_1000_ms );
                }    
        
                // decrement length
                if ( length > ee->eep_length )
                {
                    length -= ee->eep_length;
                }
                else
                {
                    length = 0;
                }
                // increment address
                start += ee->eep_length; 
            }
            else
            {
                // hole in the section, fill with 0
                EE_Dummy = 0;
                PutEepData( start, 1, &EE_Dummy, TSK_1000_ms );
                ++start;
                --length;
            }            
        }
        
        // now write the checksum
        checksum = 65536 - checksum;
        PutEepData( start, 1, &checksum, TSK_1000_ms );
        
        // clear section failed
        EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] &= ~( 1 << section );
    }
}

// ***********************************************************************
// *
// *    FUNCTION:  QueueSpiNoFeedB
// *
// *    Purpose       :
// *
// *
// *    ARGUMENTS:   data_ptr must point to global data because of delayed read/write.
// *
// *    RETURNS:
// *
// ***********************************************************************
void QueueSpiNoFeedB( uint16_t start_address, uint16_t num_words, uint16_t* data_ptr, uint16_t op_type)
{
    SPI_QUEUE_ENTRY     spi_ptr;         //Pointer to data structure
    spi_ptr.Flag = op_type;
    spi_ptr.Length = num_words;
    spi_ptr.PtrToData = data_ptr;
    spi_ptr.StartAddress = start_address;
    spi_ptr.Reporting = spi_feedback_t() ;
    spi_ptr.ReportingType = FEEDBACK_NONE;
    QueueSend( SpiQueue, &spi_ptr, 0 );
}

// ***********************************************************************
// *
// *    FUNCTION: WriteParameter 
// *
// *    DESCRIPTION: writes ee data by parameter number
// *
// *    ARGUMENTS: parameter number, data to write, number of words to write
// *
// *    RETURNS: 
// *
// ***********************************************************************
uint16_t WriteParameter( parameter_t param, void* data, uint16_t numWords )
{
    uint16_t result = false;
    EE_ID*   ee = GetParameterEE( param );
    
    if ( NULL != ee )
    {
        // parameter is valid, restrict number of words to ee length
        if ( numWords > ee->eep_length )
        {
            numWords = ee->eep_length;
        }
        // clear buffer to pad with zeros if numWords < eelength
        ClearEepromBuffer();
        // copy data
        uint16_t* paramData = (uint16_t*)data;
        for ( uint16_t idx = 0; idx < numWords; idx++ )
        {
            EepromBuffer[ idx ] = *paramData;
            ++paramData;
        }    
          
        result = PutEepData( ee->eep_addr, ee->eep_length, EepromBuffer, TSK_1000_ms );    
    }
    
    return result;
}

// ***********************************************************************
// *
// *    FUNCTION: ReadParameter 
// *
// *    DESCRIPTION: reads ee data by parameter number
// *
// *    ARGUMENTS: parameter number, data buffer to put data, number of words read
// *
// *    RETURNS: 
// *
// ***********************************************************************
uint16_t ReadParameter( parameter_t param, void* data, uint16_t numWords )
{
    uint16_t result = false;
    EE_ID*   ee = GetParameterEE( param );
    
    if ( NULL != ee )
    {
        ClearEepromBuffer();
        // read data to buffer
        result = GetEepData( ee->eep_addr, ee->eep_length, EepromBuffer, TSK_1000_ms );
        
        if ( result )
        {
            if ( numWords > EEPROM_BUFFER_SIZE )
            {
                numWords = EEPROM_BUFFER_SIZE;
            }    
        
            // copy data to pointer
            uint16_t* paramData = (uint16_t*)data;
            for ( uint16_t idx = 0; idx < numWords; idx++ )
            {
                *paramData = EepromBuffer[ idx ];
                ++paramData;
            }
        }        
    }
    
    return result;

}

// ***********************************************************************
// *
// *    FUNCTION:  
// *
// *    Purpose       :  Checks, if ROM version is newer (or older) than EEPROM version
// *                   and updates the EEPROM, and sets status bits
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************       
int32_t TaintedSections = 0;
void EE_versionUpdateWriteDefaultValue( parameter_t param, uint16_t OperateBits = 0xffff )
{
    EE_ID* ee = NULL;

    ee = GetParameterEE( param );
    if ( NULL != ee )
    {
        if ( 0xffff == OperateBits )
        {
            write_eeprom_array( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), EE_OP_WRITE_EEPROM );
            EE_execute( ee->eep_addr, &(ee->eep_DefaultValue[CoefficientsIndex] ) );
        }
        else
        {
            uint16_t value = 0;
            read_eeprom_array( ee->eep_addr, ee->eep_length, &value, EE_OP_READ_EEPROM );
            value &= ~OperateBits;
            value |= ee->eep_DefaultValue[CoefficientsIndex] & OperateBits;
	    //test third

            write_eeprom_array( ee->eep_addr, ee->eep_length, &value, EE_OP_WRITE_EEPROM );//test again
            EE_execute( ee->eep_addr, &value );
        }
        TaintedSections |= ( ((int32_t)1) << GetSectionNumber( ee->eep_addr ) );
    }
}

// Private function definitions
namespace {
// ****************************************************************************
// *
// *  Function        : UpdateBatteryStringEepValue
// *
// *  Purpose         : Update scale for EEP303: internal battery strings
// *
// *  Parms Passed    : None
// *
// *  Returns         : Nothing
// *
// *  Description:    :
// *
// ****************************************************************************
void UpdateBatteryStringEepValue( void )
{
    const EE_ID* ee = NULL;

    // index to the EE table
    uint16_t eeIdx = FindEepromTableIndex( PARAM_NumStrings );
    // check for valid index
    if (  eeIdx < GetEETableSize() )
    {
        ee = GetParameterEE( PARAM_NumStrings );
        // read eeprom data into EEPROM buffer
        read_eeprom_array( ee->eep_addr, ee->eep_length, EepromBuffer, EE_OP_READ_EEPROM );

        EepromBuffer[0] *= uint16_t(300);

        write_eeprom_array( ee->eep_addr, ee->eep_length, EepromBuffer, EE_OP_WRITE_EEPROM );

        // call the function
        EEP_PTR const eepFunc = Eep_Functions[ ee->eep_functionIndex & ~EE_PARALLEL ];

        ( eepFunc )( (EE_ID*)ee, EepromBuffer );
    }

    TaintedSections |= ( ((int32_t)1) << GetSectionNumber( ee->eep_addr ) );
}

void EE_versionUpdate_HV( void )
{
    int16_t i = 0;

    if ( CURRENT_EEMAP_VERSION_HV != EEVersion )
    {
        switch ( EEVersion )
        {
            case 0:
				EE_versionUpdateWriteDefaultValue ( PARAM_Rectifer_DCLink_Set );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierCurrentMax );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierGenCurrentMax );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectiferCurrentMargin );
				EE_versionUpdateWriteDefaultValue ( PARAM_Inverter_Current_Limit_Set );
				EE_versionUpdateWriteDefaultValue ( PARAM_Rectifier_Current_Limit_Set );
				EE_versionUpdateWriteDefaultValue ( PARAM_Battery_Current_Limit_Set );
				
			case 1:
				EE_versionUpdateWriteDefaultValue ( PARAM_MAXFanSpeed );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailLimt );
					
			case 2:				
                EE_versionUpdateWriteDefaultValue ( PARAM_WalkinStartPercentLoad );
						
			case 3: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_ATBFixedDelay );
			
			case 4: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_AmbientTempLimit );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanInputAmpsRMSLevel_3 );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_HeatsinkTempFanFull );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_RecFanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatFanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_InvHeatsinkTempFanFull );

            case 5:
                EE_versionUpdateWriteDefaultValue ( PARAM_DisableSiteWiringFault );
                EE_versionUpdateWriteDefaultValue ( PARAM_SiteWiringFaultHighLimit );
                EE_versionUpdateWriteDefaultValue ( PARAM_PhaseCHeatsinkOTWarning );
                EE_versionUpdateWriteDefaultValue ( PARAM_KeepAlive_Delay );
                EE_versionUpdateWriteDefaultValue ( PARAM_InvCapHeatsinkOTWarning );
                EE_versionUpdateWriteDefaultValue ( PARAM_InvCapHeatsinkOTTrip );

			case 6:				
				EE_versionUpdateWriteDefaultValue ( PARAM_PrechargeTime );
				EE_versionUpdateWriteDefaultValue ( PARAM_BypassPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_UtilityPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_OutputPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_BasePhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_AbsDCUVSet );
				
			case 7: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_RelDCUVSet );
				EE_versionUpdateWriteDefaultValue ( PARAM_InputSyncEnabled);
				EE_versionUpdateWriteDefaultValue ( PARAM_MinReqkVA);
				EE_versionUpdateWriteDefaultValue ( PARAM_OvrldXtrmTime);
				EE_versionUpdateWriteDefaultValue ( PARAM_Inverter_Current_Limit_Set);
				EE_versionUpdateWriteDefaultValue ( PARAM_Rectifier_Current_Limit_Set);
				EE_versionUpdateWriteDefaultValue ( PARAM_Battery_Current_Limit_Set);				
				EE_versionUpdateWriteDefaultValue ( PARAM_EnableOpenLoop);
				EE_versionUpdateWriteDefaultValue ( PARAM_BypassPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_UtilityPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_OutputPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_BasePhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxInv );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxRect );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxBatt );

			case 8: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_EnableOpenLoop );
				EE_versionUpdateWriteDefaultValue ( PARAM_BypassPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_UtilityPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_OutputPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_BasePhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_InverterVoltLoopGain );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatChargeCurrLoopGain );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxInv );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxRect );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxBatt );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailLimt );

			case 9: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_EnableOpenLoop );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxInv );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxRect );
				EE_versionUpdateWriteDefaultValue ( PARAM_CurrLimOffMaxBatt );
				EE_versionUpdateWriteDefaultValue ( PARAM_BypassPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_UtilityPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_OutputPhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_BasePhaseOffset );
				EE_versionUpdateWriteDefaultValue ( PARAM_InvEctCurrLoopGain );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectiferCurrentMargin );

			case 10: 			
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_AmbientTempLimit );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_HeatsinkTempFanFull );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_RecFanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatFanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_InvHeatsinkTempFanFull );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseCHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_InvCapHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_InvCapHeatsinkOTTrip );

			case 11:							
				EE_versionUpdateWriteDefaultValue ( PARAM_ECTSineRefForwardEnabled );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierCurrentMax );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierGenCurrentMax );				
				EE_versionUpdateWriteDefaultValue ( PARAM_RectiferCurrentMargin );

			case 12:							
				EE_versionUpdateWriteDefaultValue ( PARAM_Battery_Current_Limit_Set );
				EE_versionUpdateWriteDefaultValue ( PARAM_WalkinStartPercentLoad );

			case 13:							
				EE_versionUpdateWriteDefaultValue ( PARAM_MAXFanSpeed);
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailLimt );
				
			case 14:							
				EE_versionUpdateWriteDefaultValue ( PARAM_UPSPowerShareBuffer );

            case 15:
                EE_versionUpdateWriteDefaultValue ( PARAM_EnableParalPwmSync );

            case 16:
                EE_versionUpdateWriteDefaultValue ( PARAM_EnableParalPwmSync );
                EE_versionUpdateWriteDefaultValue ( PARAM_ESSEnabled );

            case 17:
                EE_versionUpdateWriteDefaultValue ( PARAM_OvrldLowPercent );

            case 18:
                EE_versionUpdateWriteDefaultValue ( PARAM_FanFailLimt );

            case 19:
                EE_versionUpdateWriteDefaultValue ( PARAM_FrequencyConverterMode );

            case 20:
                EE_versionUpdateWriteDefaultValue ( PARAM_L1_Relay_Time );
                EE_versionUpdateWriteDefaultValue ( PARAM_DCPreChargeRailLimit );
                EE_versionUpdateWriteDefaultValue ( PARAM_ACPreChargeRailLimit );

			case 21:
				EE_versionUpdateWriteDefaultValue ( PARAM_Rectifier_Sine_Ref_Gain );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierCurrentMax );
				EE_versionUpdateWriteDefaultValue ( PARAM_RectifierGenCurrentMax );
				
			case 22:
				EE_versionUpdateWriteDefaultValue ( PARAM_SingleUPSStartUpEnabled );

			case 23:
				EE_versionUpdateWriteDefaultValue ( PARAM_NumOfModule );

			case 24:
				EE_versionUpdateWriteDefaultValue ( PARAM_OutPF );

			case 25:
			    EE_versionUpdateWriteDefaultValue ( PARAM_NewCtrlBoard );

			case 26:
				EE_versionUpdateWriteDefaultValue ( PARAM_SCRHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_SCRHeatsinkOTTrip );

			case 27:
				EE_versionUpdateWriteDefaultValue ( PARAM_InverterCurrLoopGain );

			case 28:
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseAHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_PhaseBHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTWarning );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatHeatsinkOTTrip );
				EE_versionUpdateWriteDefaultValue ( PARAM_FanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_RecFanFailTempPrtDec );
				EE_versionUpdateWriteDefaultValue ( PARAM_BatFanFailTempPrtDec );
			case 29:
				EE_versionUpdateWriteDefaultValue ( PARAM_RecThdiImprove );
			case 30:
				EE_versionUpdateWriteDefaultValue ( PARAM_InvDutyCompensation );
			case 31:
				EE_versionUpdateWriteDefaultValue ( PARAM_FeedForwardGain );
			case 32:
				EE_versionUpdateWriteDefaultValue ( PARAM_FrequencyConverterMode );
			case 33:
				EE_versionUpdateWriteDefaultValue ( PARAM_InvDutyCompensation );
			case 34:
				EE_versionUpdateWriteDefaultValue ( PARAM_DelayCount );



                // **************************************************************
                //
                //   When modifying EEPROM map and increasing VersionEep
                //   (#define CURRENT_EEMAP_VERSION in eeprom_map.h), add your
                //   case here for the previous version, to do the 
                //   needed modifications to EEPROM.
                //
                // **************************************************************

                // update checksums of changed sections
                for ( i = 0; i < EE_Last_Section; ++i )
                {
                    if ( TaintedSections & ((int32_t)1 << i ) )
                    {
                        UpdateSectionCheckSum( i );
                    }
                }

                // attach always the last step of EEPROM update
                // right BEFORE the case CURRENT_EEMAP_VERSION
                EEVersion = CURRENT_EEMAP_VERSION_HV;
                write_eeprom_array( 1, 1, &EEVersion, EE_OP_WRITE_EEPROM );

                // unfortunately this needs to be done again
                UpdateSectionCheckSum( EE_System_ID );          
                // FALL THROUGH!!!
                //lint -fallthrough
                
            case CURRENT_EEMAP_VERSION_HV: // ready, EEPROM is up to date
                break;
                
            // If EEPROM's VersionEep > our Firmware's CURRENT_EEMAP_VERSION
            default: // i.e. a FW downgrade
                // service expert shall fix the EEPROM:
                EEStatusBits.bit.EepromVersionDowngrade = 1;
                break;
        }
    }
}

void EE_versionUpdate( void )
{
	EE_versionUpdate_HV();
}


// ***********************************************************************************************************
// *
// *  Function: UpdateSectionCheckSum(uint16_t section)
// *
// *  Purpose : Function to update a section's checksum
// *
// *  Parms Passed   :  section:        section to recalc the checksum
// *
// *  Returns        :  nothing
// *
// **********************************************************************************************************
void UpdateSectionCheckSum( uint16_t section )
{
    uint16_t start;
    uint16_t length;
    uint16_t checksum;
    
    start = GetSectionAddress( section );
    length = GetSectionLength( section ) - 1;        // exclude the checksum itself
    checksum = 0;
    read_eeprom_array( start, length, &checksum, EE_OP_READ_CHECKSUM );
    checksum = 65536 - checksum;
    start += length;
    write_eeprom_array( start, 1, &checksum, EE_OP_WRITE_EEPROM );
}

// ****************************************************************************
// *
// *  Function: QueueSPI
// *
// *
// *  Purpose :     To Queue an SPI Operation (EEPROM Read, EEPROM Write, etc..)
// *
// *
// *  Parms Passed   :  op_type: (EQUATES are located in EEPROM_DRV.H) Type of operation
// *                            EEPROM WRITE,  EEPROM READ. ... ETC..
// *
// *
// *                    start-address -- The starting Address for the operation
// *
// *
// *                    num_words -- The number of words to read or write from the EEprom Device
// *
// *
// *                    *data_ptr -- For Writes points to the data to write to the SPI (EEPROM)
// *                                 For Reads  points to the location to store the data as it is read
// *                                 For Clear and Erase, Not Used
// *
// *
// *  Returns        :   entry_number. Location in the SPI Queue. This number is used by the application
// *                     to retrieve the status of the queue entry. (The Entry Position is always less
// *                     than 255.
// *
// *
// *  Description:    This module makes an Entry into the circular SPI Queue. The position in the queue is
// *                the entry_number. When a valid entry into the Queue is made, the entry number is returned.
// *                The entry-number will therefore be .... 0 .. (SPI_QUEUE_SIZE-1).
// *                If entry_number is 0xff, an ERROR Occured and the Queue entry was not made.
// *
// *
// *                NOTE: This module must disable and re-enable interrupts globally. If Interrupts are
// *                      Disabled, they will be re-enable by this module.
// *
// ****************************************************************************
bool QueueSpi( uint16_t start_address, 
    uint16_t num_words, 
    uint16_t* data_ptr, 
    uint16_t op_type,
    spi_feedback_tag type,
    spi_feedback_t feedback)
{
    SPI_QUEUE_ENTRY     spi_ptr;         //Pointer to data structure

    
    if ( !EepromAddressValid( start_address, num_words ) &&
         ( EE_OP_UPDATE_CHECKSUM != op_type ) )
    {
        return false;
    }
    else
    {
        spi_ptr.Flag = op_type;
        spi_ptr.Length = num_words;
        spi_ptr.PtrToData = data_ptr;
        spi_ptr.StartAddress = start_address;
        spi_ptr.Reporting = feedback;
        spi_ptr.ReportingType = type;
        if ( QueueSend( SpiQueue, &spi_ptr, 0 ) )
        {
            return true;
        }
        else
        {
            return false;
        }        
    }
}

// ***********************************************************************************************************
// *
// *  Function: INT16U HandleEepDataRequest(INT16U startAddress, INT16U numWords, INT16U *dataPtr, 
// *                     INT16U maxRetry, INT16U opType)
// *
// *  Purpose : Handle task requests to the eeprom. The function blocks until the task has
// *            been succesfully completed, or when it fails. 
// *            General func used by PutEepData(), GetEepData(), EraseEepData().
// *
// *  Parms Passed   :  startAddress:   address of first Eeprom location to read/write/erase
// *                    numWords:       how many words to read/write/erase
// *                    *dataPtr:       pointer to buffer containing data
// *                    opType:         type of operation (supported: EE_OP_WRITE_EEPROM, 
// *                                        EE_OP_READ_EEPROM, or EE_OP_ERASE_EEPROM)
// *                    ticksToWait     number of OS ticks to wait, can be 0
// *                    type:           feedback type
// *                    feedback:       feedback
// *  Returns        :  errorCode:      Result of operation, as follows:
// *                                    0 = no error,
// *                                    5 = Operation failed
// *
// *  Description:      This module only returns after the requested operation has completed or failed.
// *
// *                    MUST BE CALLED ONLY FROM TASKS, SINCE IT INVOKES THE SLEEP FUNCTION
// *
// **********************************************************************************************************
uint16_t HandleEepDataRequest( uint16_t startAddress, 
    uint16_t numWords, 
    uint16_t* dataPtr, 
    uint16_t opType, 
    spi_feedback_tag fbType,
    spi_feedback_t fb )
{
    uint16_t eepQueEntry = 0;
    uint16_t section;
    SEM_Handle sem = NULL;
    uint16_t semaphoreTicks = 0;

    if ( ( fbType == FEEDBACK_SEMAPHORE ) && ( fb.Ticks != 0 ) )
    {
//	        sem = SEM_create( 0, NULL );
//	        semaphoreTicks = fb.Ticks;
//	        fb.Semaphore = sem;
		//For jira wombat 238 follow 9P-1788 P9P100-452
		/* Create new semaphore only if the wait time is not zero. There is
		 * no use in making a semaphore if we aren't going to wait for it.
		 */
		if ( fb.Ticks != 0 )
		{
			sem = SEM_create( 0, NULL );
			if ( sem == NULL )
			{
				/* Failed to allocate memory? Then fall back to no feedback. */
				fbType = FEEDBACK_NONE;
			}
			else
			{
				semaphoreTicks = fb.Ticks;
				fb.Semaphore = sem;
			}
		}
		else
		{
			/* Wait time was zero, which equals to no feedback. */
			fbType = FEEDBACK_NONE;
		}

    }
    
    // handle according to request type
    switch ( opType )
    {
            
        case EE_OP_WRITE_EEPROM:
            if( !NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT ) )
            {
                // queue a write request
                // check if checksum section or not
                section = GetSectionNumber( startAddress );
                if ( CheckChecksumProtected( section ) )
                {
                    eepQueEntry = QueueSpi( startAddress, numWords, dataPtr, EE_OP_WRITE_EEPROM, FEEDBACK_NONE, spi_feedback_t() );
                    if ( eepQueEntry )
                    {
                        eepQueEntry = QueueSpi( section, 0, NULL, EE_OP_UPDATE_CHECKSUM, fbType, fb );
                    }
                }
                else
                {
                    eepQueEntry = QueueSpi( startAddress, numWords, dataPtr, EE_OP_WRITE_EEPROM, fbType, fb );
                }
            }
            break;
           
        case EE_OP_READ_EEPROM:
            // queue a read request
            eepQueEntry = QueueSpi( startAddress, numWords, dataPtr, EE_OP_READ_EEPROM, fbType, fb );
            break;
           
        case EE_OP_ERASE_EEPROM:
            
            if( !NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT ) )
            {
                // queue an erase request
                eepQueEntry = QueueSpi( startAddress, numWords, &EE_Dummy, EE_OP_ERASE_EEPROM, fbType, fb );
            }
            break;

        case EE_OP_READ_CHECKSUM:
            eepQueEntry = QueueSpi( startAddress, numWords, &EE_Dummy, EE_OP_READ_CHECKSUM, fbType, fb );
            break;
        
        default:
            break;
    }        
    
//	    if ( NULL != sem )
//	    {
//	        // don't bother waiting if it didn't queue
//	        if ( eepQueEntry )
//	        {
//	            eepQueEntry = SEM_pend( sem, semaphoreTicks );
//	        }    
//	        SEM_delete( sem );    
//	    }

	//For jira wombat 238 follow 9P-1788 P9P100-452
	/* Check if a feedback semaphore was used. */
	if ( NULL != sem )
	{
		/* Don't wait for semaphore if request didn't queue. */
		if ( eepQueEntry )
		{
			/* Here we wait for the semaphore to get posted. The semaphore SHOULD be
			 * posted sometime because the request queued. In case this doesn't happen
			 * for some reason, there is a maximum amount of time to wait.
			 */
			uint16_t retries = 50;

			/* First wait the time passed to this function. */
			eepQueEntry = SEM_pend( sem, semaphoreTicks );

			/* If semaphore didn't get posted in that time, wait a bit longer. */
			while( !eepQueEntry && retries-- )
			{
				eepQueEntry = SEM_pend( sem, TSK_100_ms );
			}

			/* Now the semaphore should shave been posted. If it hasn't, we still remove it
			 * since otherwise there would be a memory leak. There is a risk that in case
			 * the semaphore does get posted sometime in the future, DSP/BIOS might crash
			 *	(because the semaphore doesn't exist anymore).
			 */
#if 0	/* For debugging. */
			if ( !eepQueEntry )
			{
				/* A semaphore timed out, might cause crash if the semaphore gets posted.
				 * Execution should never reach here. */
			}
#endif
		}
		SEM_delete( sem );
	}

        
    return eepQueEntry;
}

// ***********************************************************************
// *
// *    FUNCTION: ClearEepromBuffer 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ClearEepromBuffer( void )
{
    memset(EepromBuffer, 0, EEPROM_BUFFER_SIZE);     
}


// ****************************************************************************
// *
// *  Function      :  EE_execute
// *
// *  Purpose       :  Process data written to the EEPROM
// *
// *  Parms Passed  :  ee address, dataPtr
// *
// *  Returns       :  
// *
// *  Description:   This module executes the EEP function. Non-checksum
// *                 or ee address not the the EE table will not do anything
// *
// ****************************************************************************
void EE_execute( uint16_t start_address, uint16_t* data_ptr )
{
    // index to the EE table
    uint16_t eeIdx = FindEepromTableIndex( start_address );
    
    // check for valid index
    if ( eeIdx < GetEETableSize() )
    {
        const EE_ID* ee = &eeprom[ eeIdx ];    //Jacob/20130814/Merge 120k
        
        // call the function
        uint16_t eepFunctionIndex = ee->eep_functionIndex & ~EE_PARALLEL;
        bool parallelEep = (ee->eep_functionIndex & EE_PARALLEL) == EE_PARALLEL;
                        
        EEP_PTR const eepFunc = Eep_Functions[ eepFunctionIndex ];
        
        ( eepFunc )( (EE_ID*)ee, data_ptr );
        
        if ( parallelEep &&
            ( 0 == EepResetNoSyncTimer ) )  // not sync when reset, 1.eep change -> 1.1 to all
        {
            // Route the change also through PCAN
            ParallelCan.PCan_OnParameterChanged(ee->eep_addr, *data_ptr);
        }
    }
}

} // !namespace (anon)


// ************************************************************************************
//  No more
// ************************************************************************************
