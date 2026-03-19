// ********************************************************************************************************
// *            PortHandler.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: PortHandler.c
// *
// *    DESCRIPTION: This module is a state machine for data ports. It routes incoming data to various modules
// *                    such as XCP, Debugger, etc.
// *
// *    ORIGINATORS: Jonathan Rodriguez, Costin Radoias
// *
// *    DATE: 2/20/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "HQ_Funcs.h"
#include "Debugger.h"
#include "History.h"
#include "Constants.h"

extern "C" 
{
    void PortHandler( void  );
}    

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************
void CharHandler( SerialPort* port );

// ********************************************************************************************************
// *
// * Function: PortXHandler();    [X=1..3]
// *
// * Purpose: This function is called from ServicePorts() to handle data that has come in on the port.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function calls a data processing module such as the debugger, XCP handler, etc. to
// *                parse the incoming data. This is used in a task because because the data processing
// *                module is called within this context and may block. This task is constantly running,
// *                but pends until the ServicePorts posts the semaphore.
// *
// ********************************************************************************************************
void PortHandler()
{
    SerialPort port( &SciaRegs );
    InitSciaGpio();
    
    while(1)
    {
        // pend until data has come in on the port
        while ( 0 == port.SizeRxBuf() )
        {
            TSK_sleep( TSK_10_ms );
        }
        
        CharHandler( &port );
    }   // end while
}

// ********************************************************************************************************
// *
// * Function: void CharHandler(PORT_S* myPort, SEM_Obj port_sem, HandlerAttrib* handlerAttrib)
// *
// * Purpose:
// *
// * Parms Passed   :   myPort          :   This is the port data has come in on.
// *                :   port_sem        :   This is the semaphore to be used if the port needs to block while
// *                                        waiting for data.
// *                :   handlerAttrib   :   This is the attributes of the current data stream (state and mode).
// * Returns        :   handlerAttrib   :   This is altered by the function to reflect updated state and mode.
// *
// * Description:
// *
// *
// *
// *
// ********************************************************************************************************
void CharHandler( SerialPort* port )
{
    uint16_t timeOut = ( 30000 / TSK_10_ms );         // 30s timeout before giving up
    Debugger* debug = NULL;
    History*  log = NULL;
    
    while ( timeOut )
    {
        --timeOut;
        TSK_sleep ( TSK_10_ms );
        
        if ( port->SizeRxBuf() )
        {
            switch ( port->GetRxChar() )
            {
                case 'D':
                case 'd':   // start debugger
                    debug = new Debugger( &SciaRegs );
                    debug->DebuggerStart();
                    delete debug;
                    timeOut = 0;
                    break;
                
                case 'B':
                case 'b':
                    debug = new Debugger( &SciaRegs );
                    debug->DebuggerStartBode();
                    delete debug;
                    timeOut = 0;
                    break;
                    
                case 'l':
                case 'L':   // start history log
                    log = new History( &SciaRegs );
                    log->LogStart();
                    delete log;
                    timeOut = 0;
                    break;
        
                case ESC:
                    timeOut = ( 30000 / TSK_10_ms );        // restart timeout if another esc recieved
                    break;

                default:    // some other command...
                    timeOut = 0;
                    break;  // do nothing...
            }   // end switch
        }    
    }
}

// ********************************************************************************************************
// *            END OF PortHandler.c
// ********************************************************************************************************


