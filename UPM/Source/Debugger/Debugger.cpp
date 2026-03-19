// ********************************************************************************************************
// *            Debugger.c
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
// *    FILE NAME: Debugger.c
// *
// *    DESCRIPTION: Engine for the debugger. Definitions for the Debugger API are here.
// *
// *    ORIGINATOR: Jonathan Rodriguez
// *
// *    DATE: 12/11/2002
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "BypassInterface.h"
#include "BootloaderAPI.h"
#include "Debugger.h"
#include "DebuggerBlocks.h"
#include "Constants.h"
#include "F28335Port.h"
#include "InverterControl.h"
#include "RectifierStateControl.h"
#include "Spi_Driver.h"
#include "Spi_Task.h"
#include "Eeprom_Map.h"
#include "adc.h"
#include "algos.h"
#include "InitPWM.h"
#include "PeriodicTSKs.h"
#include "HQ_Funcs.h"
#include "History.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include "Fan.h"
#include "Rtc.h" 
#include "NB_Config.h"
#include "NB_Funcs.h" 
#include "XCPDefs.h"
#include "BootloaderAPI.h"
#include "BTR.h"
#include "IOexpansion.h"
#include "BypassState.h"
#include "BackfeedState.h"
#include "AutoCal.h"
#include "ExtSignalPLL.h"
#include "VirtualNeutralMeter.h"
#include "BodeScan.h"
#include "InternalCan.h"
#include "FCTState.h"
#include "ProcessAdc.h"
#include "tsk.h"

#include <limits>
#include <cstdio>
#include <cstring>

using namespace std;

const char* const UnRecComStr         = {"Unrecognized Command"};
const char* const EepValueIsStr       = {"EEP "};
const char* const EnterCommStr        = {"Enter Command>> "};
const char* const HeaderStr           = {"Santak 28335 Debugger"};
const char* const FooterStr           = {"Enter Command When Ready..."};
const char* const McuStr              = {"MCU"};
const char* const InverterStr         = {"INVERTER"};
const char* const RectifierStr        = {"RECTIFIER"};
const char* const SystemStr           = {"SYSTEM"};
const char* const ChargerStr          = {"CHARGER"};
const char* const ExitStr             = {"Debugger Off"};
const char* const UnRecNBStr          = {"Invalid/Unimplemented nodebit number"};

bool dcff_test = 0;

const uint16_t promptLine             = 22;           // the line the prompt is written on
const uint16_t debuggerSleeptime      = TSK_10_ms;
const uint16_t DebuggerUpdateRatems   = 100;          
const uint16_t DebuggerUpdateRate     = ( DebuggerUpdateRatems / debuggerSleeptime );
const uint16_t CommandTimeoutTime     = ( 15000 / debuggerSleeptime );                                  // 15s
const uint32_t DebuggerTimeoutTime    = ( ( 10L * 60L * 1000L ) / (uint32_t)debuggerSleeptime );        // 10 minutes
// ********************************************************************************************************
// *
// * Function: DebuggerMain(PORT_S *port);
// *
// * Purpose: To display prompt on screen and store characters into buffer for parsing.
// *
// * Parms Passed   :   port        : Pointer to Port to get characters from.
// * Returns        :   Nothing
// *
// * Description: This is the main engine of the debugger. It stores commands and calls the command parser.
// *
// ********************************************************************************************************
void Debugger::DebuggerMain( void )
{
    uint16_t tempSize = 0;                // temp vars used for putting incoming characters into commandBuf
    bool   escCharRx = false;           // signal if 'esc' char is received
    
    cmdBufIndex = 0;

    while ( !escCharRx &&                   // exit when escape character is recieved
            ( debuggerTimeoutCount > 0 ) )  // or timeout
    {
        // store and echo character(s)
        tempSize = SizeRxBuf();
        
        if ( tempSize )
        {
            // char received, reset timeout
            debuggerTimeoutCount = DebuggerTimeoutTime;
            cmdTimeout = CommandTimeoutTime;
            
            // if we were in RefreshMode and we receive a character...
            if ( ReDrawON )
            {
                ReDrawON = false;
                updateCount = 0;

                // Print Debug Prompt
                SetCursor( promptLine, 0 );
                EraseRestOfLine();
                PutTxString( (char*)EnterCommStr );
            }
        
            if (tempSize + cmdBufIndex <= CommandBufferSize)
            {
                GetChars( &commandBuf[ cmdBufIndex ], tempSize );
                PutTxBuf( (char*)&commandBuf[ cmdBufIndex ], tempSize );
                cmdBufIndex += tempSize;
            }
            else
            {
                PurgeRxChars( 64 );
                cmdBufIndex = 0;
            }
            // Check to see if latest character is a <esc>
            if ( ( ESC == commandBuf[ cmdBufIndex-1 ] ) || 
                 ( XCP_SFD == commandBuf[ cmdBufIndex-1 ] )  )       // even though we don't have XCP, exit on SFD anyway
            {
                escCharRx = true;
            }
            else
            {        
                // Check to see if latest character is a CR...
                // if character at current pointer is a CR a complete request has been received
                if( ( commandBuf[ cmdBufIndex-1 ] == CR ) || (  commandBuf[ cmdBufIndex-1 ] == 0x0A  ) )
                {
                    // calls command to clean out backspaces
                    CheckCommandForBackSpace();
                    // Call DEBUG command parsing function...
                    ParseDebugCommand();
                    cmdBufIndex = 0;
                }
            }    
        }    
        else
        {
            // don't refresh when somebody is typing
            if ( 0 == cmdBufIndex )
            {
                DebuggerUpdate();
            }
            else
            {
                if ( cmdTimeout )
                {
                    --cmdTimeout;
                }
                else
                {
                    cmdBufIndex = 0; 
                }    
            }    
        }
        
        TSK_sleep( debuggerSleeptime );
        --debuggerTimeoutCount;
    }    
    
    ClrScrn();
    SetCursor( promptLine, 0 );
    EraseRestOfLine(); 
    PutTxString( (char*)ExitStr );
}

void Debugger::DebuggerBode( void )
{
    uint16_t tempSize = 0;              // temp vars used for putting incoming characters into commandBuf
    bool   escCharRx = false;           // signal if 'esc' char is received
    
    cmdBufIndex = 0;
    
    ClrScrn();
    SetCursor( 2, 0 );
    EraseRestOfLine(); 
    PutTxString("Now we are in Bode Scan Mode,Press 'ESC' back.\n");
	
    while ( !escCharRx &&                   // exit when escape character is recieved
            ( debuggerTimeoutCount > 0 ) )  // or timeout
    {
        // store and echo character(s)
        tempSize = SizeRxBuf();
        
        if ( tempSize )
        {
            // char received, reset timeout
            debuggerTimeoutCount = DebuggerTimeoutTime;
            cmdTimeout = CommandTimeoutTime;
            
            /*
            // if we were in RefreshMode and we receive a character...
            if ( ReDrawON )
            {
                ReDrawON = false;
                updateCount = 0;

                // Print Debug Prompt
                SetCursor( promptLine, 0 );
                EraseRestOfLine();
                PutTxString( (char*)EnterCommStr );
            }
            */
            
            if (tempSize + cmdBufIndex <= CommandBufferSize)
            {
                GetChars( &commandBuf[ cmdBufIndex ], tempSize );
              //PutTxBuf( (char*)&commandBuf[ cmdBufIndex ], tempSize );   // don't echo in bode-scan
                cmdBufIndex += tempSize;
            }
            else
            {
              //PurgeRxChars( 64 );
                cmdBufIndex = 0;
            }
            // Check to see if latest character is a <esc>
            if ( ( ESC == commandBuf[ cmdBufIndex-1 ] ) || 
                 ( XCP_SFD == commandBuf[ cmdBufIndex-1 ] )  )       // even though we don't have XCP, exit on SFD anyway
            {
                escCharRx = true;
            }
            else
            {        
                // Check to see if latest character is a CR...
                // if character at current pointer is a CR a complete request has been received
                if( ( commandBuf[ cmdBufIndex-1 ] == CR ) || (  commandBuf[ cmdBufIndex-1 ] == 0x0A  ) )
                {
                    // calls command to clean out backspaces
                    CheckCommandForBackSpace();
                    // Call DEBUG command parsing function...
                    ParseBodeCommand();
                    cmdBufIndex = 0;
                }
            }    
        }    
        else
        {
            // don't refresh when somebody is typing
            if ( 0 == cmdBufIndex )
            {
                DebuggerUpdate();
            }
            else
            {
                if ( cmdTimeout )
                {
                    --cmdTimeout;
                }
                else
                {
                    cmdBufIndex = 0; 
                }    
            }    
        }
        
        TSK_sleep( debuggerSleeptime );
        --debuggerTimeoutCount;
    }    
    
    ClrScrn();
    SetCursor( promptLine, 0 );
    EraseRestOfLine(); 
    PutTxString( "Bode Scan Mode OFF!\n" );
}

// ********************************************************************************************************
// *
// * Function: DebuggerUpdate();
// *
// * Purpose: To periodically update the display.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// *
// ********************************************************************************************************
void Debugger::DebuggerUpdate( void )
{
    // if there is a screen to display...
    if ( eScreenNone != ScreenFunction ) 
    {
        if ( ( updateCount > DebuggerUpdateRate ) ||
             ReDrawON )
        {
            updateCount = 0;
            switch ( ScreenFunction )
            {
                case eBlockScreen:
                    BlocksScreen();
                    break;
                case eDumpScreen:
                    DumpScreen();
                    break;
                case eDumpSamplesScreen:
                    DumpSamples();
                    break;
                case eEventScreen:
                    ActiveEventsScreen();
                    break;    
                case eLogScreen:
                	LoggerScreen();
                	break;
                default:
                    ScreenFunction = eScreenNone;
                    break;                    
            }
            
            // Print Debug Prompt
            SetCursor( promptLine, 0 );
            EraseRestOfLine();
            
        }
        else
        {
            ++updateCount;
        }        
    }
}

// ********************************************************************************************************
// *
// * Function: DebugStart();
// *
// * Purpose: To do any initialization required and then start the debugger.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function should be called to start the debugger.
// *
// ********************************************************************************************************
void Debugger::DebuggerStart( void )
{
    TextBuffer = new char[ TextBufferSize ];
    commandBuf = new char[ CommandBufferSize ];
    
    if ( ( NULL != TextBuffer ) && ( NULL != commandBuf ) )
    {
        debuggerTimeoutCount = DebuggerTimeoutTime;
        Gm();           // set startup screen (must use one of the DebuggerFunctions)
        DebuggerMain();
    }
    
    delete [] TextBuffer;
    delete [] commandBuf; 
    TextBuffer = NULL;
    commandBuf = NULL;  
}

void Debugger::DebuggerStartBode( void )
{
    TextBuffer = new char[ TextBufferSize ];
    commandBuf = new char[ CommandBufferSize ];
    
    if ( ( NULL != TextBuffer ) && ( NULL != commandBuf ) )
    {
        debuggerTimeoutCount = DebuggerTimeoutTime;
//      Gm();           // set startup screen (must use one of the DebuggerFunctions)
        DebuggerBode();
    }
    
    delete [] TextBuffer;
    delete [] commandBuf; 
    TextBuffer = NULL;
    commandBuf = NULL;  
}

// ********************************************************************************************************
// *
// * Function: CheckCommandForBackSpace(INT8S commandBuf[]);
// *
// * Parms Passed   :   commandBuf[]      : buffer containing the command to be parsed.
// * Returns        :   Nothing
// *
// * Description: Parses command string for backspaces and reformats cleaning up deleted
// *                characters so command is correct for ParseDebugCommand().
// ********************************************************************************************************
void Debugger::CheckCommandForBackSpace( void )
{
    uint16_t  index = 0;
    uint16_t  moveIndex;
    uint16_t  destIndex;
    uint16_t  tempChar;

        // loops looking for the CR end of string
    while ( (tempChar = commandBuf[index++]) != CR && index < CommandBufferSize)
    {
            // if char is a backspace, copy from next char to end of string and put
            // starting at character before backspace
        if (tempChar == BS && 2 <= index)
        {
            destIndex = index - 2;
            moveIndex = index;
            while (commandBuf[moveIndex] != CR && moveIndex < CommandBufferSize-1)
            {
                commandBuf[destIndex++] = commandBuf[moveIndex++];
            }
            commandBuf[destIndex] = commandBuf[moveIndex];
            index -= 2;
        }
    }
}

// ********************************************************************************************************
// *
// * Function: GetCommandArgument(int16_t argumentNum);
// *
// * Purpose: To parse an argument from a debugger command.
// *
// * Parms Passed   :   argumentNum : The number of the argument to parse. 1 is the first argument.
// * Returns        :   int16_t      : The parsed and converted argument.
// *
// * Description: This function parses an ASCII representation of a debugger argument and returns the
// *                unsigned int version of it.
// *
// ********************************************************************************************************
int32_t Debugger::GetCommandArgument( int16_t argumentNum )
{
    int32_t arg = -1;                       // value of argument
    int16_t currentArg = 0;                 // current argument number
    int16_t index = 0;                      // command buffer index
    bool  argIsNegative = false;            // argument is negative

        // skip first characters, they are the command,
        // skip the arguments you are not interested in
    while ( ( currentArg != argumentNum ) && ( index < CommandBufferSize ) )
    {
            // skip the argument
        while ( ( ' ' != commandBuf[index] ) && 
                ( CR != commandBuf[index] )  &&
                ( index < CommandBufferSize ) )
        {
            index++;
        }
            // skip the spaces
        while ( (' ' == commandBuf[index]) && ( index < CommandBufferSize ) )
        {
            index++;
        }
        currentArg++;
    }

        // if it's a hex number...
    if ('h' == commandBuf[index])
    {
        arg = 0;    // if we made it into here, we must have a valid number
        index++;    // skip the 'h'

            // parse the argument until space or CR is reached
        while ( (' ' != commandBuf[index]) && 
                (CR != commandBuf[index]) &&
                ( index < CommandBufferSize ))
        {
            arg = arg << 4;             // shift over one digit (base 16)
            if (commandBuf[index] > '9')
            {
                arg += (commandBuf[index] - 0x57);
            }
            else
            {
                arg += (commandBuf[index] - 0x30);
            }
            index++;
        }
    }
    else        // it's a decimal (base 10) number
    {
            // check for a negative sign
        if ('-' == commandBuf[index])
        {
            argIsNegative = true;
            index++;
        }

            // parse the argument until space or CR is reached
        while ( (' ' != commandBuf[index]) && 
                (CR != commandBuf[index])  &&
                ( index < CommandBufferSize ))
        {
                // if we made it into this loop, then we must have a valid argument
            if (-1 == arg)
            {
                arg = 0;    // reset to 0
            }
            arg = arg * 10;             // shift over one digit (base 10)
            arg += (commandBuf[index] - 0x30);
            index++;
        }
    }

    if (argIsNegative)
    {
        arg = arg * (-1);
    }

    return arg;
}

// ********************************************************************************************************
// *
// * Function: ParseDebugCommand(PORT_S* port, INT8S commandBuf[]);
// *
// * Purpose: To parse the command and call the appropriate function.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// *                    commandBuf[]      : buffer containing the command to be parsed.
// * Returns        :   Nothing
// *
// * Description: This is the function that parses the command and call the appropriate function. If a new
// *                command is added to the debugger, it must be added to the switch statement in this function.
// *                This function also sets the current port for use by any of the command functions.
// *
// ********************************************************************************************************
void Debugger::ParseDebugCommand( void )
{
    char  tmp;

    // make first 3 chars uppercase
    tmp = commandBuf[0];
    if(tmp>='a' && tmp<='z') { commandBuf[0] = tmp - ('a'-'A'); }
    tmp = commandBuf[1];
    if(tmp>='a' && tmp<='z') { commandBuf[1] = tmp - ('a'-'A'); }

    switch (commandBuf[0])
    {
        case 'A':
            SetScreenFunction( eEventScreen );
            break;
            
            
            // 'g' (GET) commands...
        case 'G':
            switch (commandBuf[1])
            {
                case 'M':
                    Gm();
                    break;

                case 'I':
                    Gi();
                    break;

                case 'R':
                    Gr();
                    break;

                case 'S':
                    Gs();
                    break;

                case 'C':
                    Gc();
                    break;

                case 'T':
                    Gt();
                    break;
                    
                case 'E':
                    Ge();
                    break;                    
                                    
                // Get nodebit status
                case 'N':
                    Gn();
                    break;
                    
                case 'P':
                    Gp();
                    break;

                case '1':
                    G1();
                    break;

                case '2':
                    G2();
                    break;
					
				case '3':
					G3();
					break;
					
				case '4':
					G4();
					break;

                default:
                    PutTxString( (char*)UnRecComStr );
                    EraseRestOfLine();
                    ReDrawON = true;
                    break;
            }
            break;

        // 'Write' commands...
        case 'W':
            switch (commandBuf[1])
            {
                case 'B':   // write block...
                    Wb();   // write block
                    break;

                case 'E':
                    We();   // write eep (1 word)
                    break;
                
                case 'N': // Force nodebit set and enter NB test mode if needed.
                    Wn();
                    break;
                    
                case 'S': // Set the nodebit without entering test mode.
                    Ws();
                    break;
                    
                default:
                    PutTxString( (char*)UnRecComStr );
                    EraseRestOfLine();
                    ReDrawON = true;
                    break;
            }
            break;

        // 'Read' commands...
        case 'R':
            switch (commandBuf[1])
            {
                // 'Eep' command
                case 'E':
                    Re();
                    break;

                // 'Eep reset' command
                case 'S':
                    Rs();
                    break;
                
                case 'N':
                    // Reset nodebits
                    Rn();
                    break;
                
                default:
                    PutTxString( (char*)UnRecComStr );
                    EraseRestOfLine();
                    ReDrawON = true;
                    break;
            }
            break;

        // 'Dump' commands...
        case 'D':
            switch(commandBuf[1])
            {
                case 'M':
                    Dm();
                    break;
                case 'E':
                    De();
                    break;
                case 'D':
                	Dd();
                	break;
                default:
                    PutTxString( (char*)UnRecComStr );
                    EraseRestOfLine();
                    ReDrawON = true;
                    break;
            }
            break;

        // Commands
        case 'C':
            switch(commandBuf[1])
            {
                case 'I':   // Inverter Commands
                    Ci();
                    break;

                case 'R':   // Rectifier Commands
                    Cr();
                    break;

                case 'M':   // MCU commands
                    Cm();
                    break;

                case 'C':  // Charger command
                    Cc();
                    break;

                case 'B':  // Boost command
                    Cb();
                    break;

                case 'T':  // Control Coefficients Test command
                    Ct();
                    break;

                default:
                    PutTxString( (char*)UnRecComStr );
                    EraseRestOfLine();
                    ReDrawON = true;
                    break;
            }

            break;

        case 'S':
            S();
            break;
            
        case 'T':
            T();
            break;

        case 'F':
            switch (commandBuf[1])
            {
                case 'L': // fan test with Load
                    FanTestWithLoad(GetCommandArgument(1), GetCommandArgument(2));
                    break;
                case 'S': // fan test with Speed
                    FanTestWithSpeed(GetCommandArgument(1), GetCommandArgument(2));
                    break;
                default:
                    break;
            }
            break;
            
        case 'U':
            U();
            break; 
            
        case 'P':   // 'P'arameter commands
            P();
            break;                       
                            
        default:
            PutTxString( (char*)UnRecComStr );
            EraseRestOfLine();
            ReDrawON = true;
            break;


    }       // end of switch statement
}

void Debugger::ParseBodeCommand( void )
{
    char  tmp;

    // make first 3 chars uppercase
    tmp = commandBuf[0];
    if(tmp>='a' && tmp<='z') { commandBuf[0] = tmp - ('a'-'A'); }
    tmp = commandBuf[1];
    if(tmp>='a' && tmp<='z') { commandBuf[1] = tmp - ('a'-'A'); }

    if( commandBuf[0] == 'B' )
    {
        if( commandBuf[1] == 'S' )
        {
            BSCommand();
            BSCommandCnt++;
        }
        else if( commandBuf[1] == 'T' )
        {
            BTCommand();
            BTCommandCnt++;
        }
        else if( commandBuf[1] == 'Q' )
        {
            BQCommand();
            BQCommandCnt++;
        }
        else    
        {
          //PutTxString("B? - Invalid Command!\n");
        }            
    }
    else
    {
       //PutTxString("?? - Invalid Command!\n");
       InValidCommandCnt++;
    }    
}

// ***********************************************************************
// *
// *    FUNCTION: SetScreenFunction 
// *
// *    DESCRIPTION: De-stupified version from 2812 project 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void Debugger::SetScreenFunction( eScreenFunc screen )
{
    ScreenFunction = screen;
    updateCount = 0;
    lastNumberOfEvents = 0xffff;
    ReDrawON = true;
}

// ********************************************************************************************************
// * COMMAND FUNCTIONS
// *   Add the code for your new commands here. Each command has its own function.
// ********************************************************************************************************

// ********************************************************************************************************
// *
// * Function: G_()
// *
// * Purpose: GET BLOCKS
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function sets which blocks to display, then causes the Blocks screen to be displayed.
// *
// ********************************************************************************************************
void Debugger::Gm()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_MCU_START;
    BlocksDS.endBlock = BLOCKS_MCU_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gi()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_INVERTER_START;
    BlocksDS.endBlock = BLOCKS_INVERTER_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gr()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_RECTIFIER_START;
    BlocksDS.endBlock = BLOCKS_RECTIFIER_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gs()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_SYSTEM_START;
    BlocksDS.endBlock = BLOCKS_SYSTEM_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gc()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_CHARGER_START;
    BlocksDS.endBlock = BLOCKS_CHARGER_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gt()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_METERS1_START;
    BlocksDS.endBlock = BLOCKS_METERS1_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Ge()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_METERS2_START;
    BlocksDS.endBlock = BLOCKS_METERS2_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::Gn()
{
    SetCursor(20, 1);
    // Get nodebit, does not affect test mode
    int16_t nbNumber = GetCommandArgument(1); // internal nodebit ID number
    
    if (nbNumber < 0 || nbNumber > UPM_NB_NUMBER_OF_SUPPORTED_EVENTS) {
        PutTxString(UnRecNBStr);
        return;
    }
    uint16_t state = NB_GetNodebit(nbNumber);
    SYS_sprintf(TextBuffer, "node %d: %u", nbNumber, state);
    PutTxString(TextBuffer); 
    EraseRestOfLine();
}

void Debugger::Gp()
{
        // set starting block #
    BlocksDS.startBlock = BLOCKS_PARALLEL_START;
    BlocksDS.endBlock = BLOCKS_PARALLEL_END;

        // set which screen to draw
    SetScreenFunction( eBlockScreen );
}

void Debugger::G1()
{
    BlocksDS.startBlock = BLOCKS_1_START;
    BlocksDS.endBlock = BLOCKS_1_END;

    SetScreenFunction( eBlockScreen );
}
void Debugger::G2()
{
    BlocksDS.startBlock = BLOCKS_2_START;
    BlocksDS.endBlock = BLOCKS_2_END;

    SetScreenFunction( eBlockScreen );
}
void Debugger::G3()
{
    BlocksDS.startBlock = BLOCKS_3_START;
    BlocksDS.endBlock = BLOCKS_3_END;

    SetScreenFunction( eBlockScreen );
}
void Debugger::G4()
{
    BlocksDS.startBlock = BLOCKS_4_START;
    BlocksDS.endBlock = BLOCKS_4_END;

    SetScreenFunction( eBlockScreen );
}

void Debugger::Wn()
{
    SetCursor(20, 1);
    // Write nodebit, enter test mode
    int32_t nbNumber = GetCommandArgument(1); // Internal nodebit number
    int32_t nbState = GetCommandArgument(2); // Set or clear the nodebit
    int32_t auxData = GetCommandArgument(3); // Optional auxiliary data to set with the nodebit.
    // TODO: Make auxData parsing optional.
    
    if (nbNumber < 0 || nbNumber > UPM_NB_NUMBER_OF_SUPPORTED_EVENTS) {
        PutTxString("Invalid or unimplemented nodebit number");
        return;
    }
    if (nbState != 0 && nbState != 1) {
        PutTxString("Invalid nodebit state");
        return;
    }
    if (auxData < 0 || auxData > std::numeric_limits<uint16_t>::max()) {
        PutTxString("Invalid log data");
        return;
    }
    
    uint16_t state = NB_Force(nbNumber, nbState, auxData);
    
    SYS_sprintf(TextBuffer, "node %d: %d", (int16_t)nbNumber, (int16_t)state);
    PutTxString(TextBuffer);
    EraseRestOfLine();
}

void Debugger::Ws()
{
    SetCursor(20, 1);
    // Write nodebit
    int32_t nbNumber = GetCommandArgument(1); // Internal nodebit number
    int32_t nbState = GetCommandArgument(2); // Set or clear the nodebit
    int32_t auxData = GetCommandArgument(3); // Optional auxiliary data to set with the nodebit.
    // TODO: Make auxData parsing optional.
    
    if (nbNumber < 0 || nbNumber > UPM_NB_NUMBER_OF_SUPPORTED_EVENTS) {
        PutTxString("Invalid or unimplemented nodebit number");
        return;
    }
    if (nbState != 0 && nbState != 1) {
        PutTxString("Invalid nodebit state");
        return;
    }
    if (auxData < 0 || auxData > std::numeric_limits<uint16_t>::max()) {
        PutTxString("Invalid log data");
        return;
    }
    
    uint16_t state = NB_SetNodebit(nbNumber, nbState, auxData, false);
    
    SYS_sprintf(TextBuffer, "node %d: %d", (int16_t)nbNumber, (int16_t)state);
    PutTxString(TextBuffer);
    EraseRestOfLine();
}

void Debugger::Rn()
{
    // Reset Nodebits, exit test mode
    NB_Init();
    SetCursor(20, 1);
    PutTxString("Resuming normal nodebit function");
    EraseRestOfLine();
}

// ********************************************************************************************************
// *
// * Function: Wb();
// *
// * Purpose: WRITE BLOCK.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function writes a value to a block.
// *
// ********************************************************************************************************
void Debugger::Wb()
{
    uint16_t block;       // index into the BlockArray
    int32_t value;        // this is the new value to store into a block
    const stBlock* thisBlock;
    
    // parse block number
    block = GetCommandArgument(1);

    // parse new value
    value = GetCommandArgument(2);

    // set data to new value
    if ( block < BlockArraySize )
    {
        thisBlock = &BlockArray[block]; 
        
        switch ( thisBlock->memtype )
        {
            case MemTypeINT:
                *(int16_t*)thisBlock->data = (int16_t)value;
                break;
                
            case MemTypeUINT:
                *(uint16_t*)thisBlock->data = (uint16_t)value;
                break;
                
            case MemTypeLONG:
                *(int32_t*)thisBlock->data = value;
                break;
                
            case MemTypeULONG:
                *(uint32_t*)thisBlock->data = (uint32_t)value;
                break;
                
            case MemTypeFLOAT: {
                float scale = 1.0;
                switch (thisBlock->type) {
                    case DEC1:
                        scale = 0.1;
                        break;
                    case DEC2:
                        scale = 0.01;
                        break;
                    case DEC3:
                        scale = 0.001;
                        break;
                    case DEC4:
                        scale = 0.0001;
                        break;
                    
                    // The following FALLTHROUGH deliberately.
                    case DEC0:
                    case HEX:
                    case NONE:
                    default:
                        scale = 1.0;
                }
                *(float*)thisBlock->data = (float)value * scale;
                break;
            }
                
            default:
                break;    
                                    
        }   
    }
}

// ********************************************************************************************************
// *
// * Function: We();
// *
// * Purpose: WRITE EEP.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function sets an eep to a specified value.
// *
// ********************************************************************************************************
void Debugger::We()
{
     int32_t eepAddress;
     int32_t newValue;
     uint16_t value;
 
     eepAddress = GetCommandArgument(1);
     newValue = GetCommandArgument(2);

     if (eepAddress < 0 || eepAddress > MAX_EE_LENGTH) {
        PutTxString("Invalid EEPROM address");
        return;
     }

     
     if (newValue < 0 || newValue > std::numeric_limits<uint16_t>::max()) {
        PutTxString("Invalid EEPROM value");
        return;
     }
    
     SetCursor(20, 1);
 
     if ( EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] )
     {
        PutTxChar('\\');    
     }  
     else
     {
        PutTxChar('*');
     }
     
     value = uint16_t(newValue);
     PutEepData(eepAddress, 1, &value, TSK_1000_ms);    // Write newValue to eepAddress
     PutTxString(EepValueIsStr);
     PutTxChar('W');
     PutTxChar('#');
     SYS_sprintf( TextBuffer, "%d", uint16_t(eepAddress));
     PutTxString( TextBuffer );
     PutTxChar('=');
     PutTxChar(' ');
     SYS_sprintf( TextBuffer, "%u %x", value, value );
     PutTxString( TextBuffer );
     EraseRestOfLine();
     
     GetEepData(eepAddress, 1, &value, TSK_1000_ms);   // read Eeprom     
     PutTxChar(',');
     PutTxChar('R');
     PutTxChar('=');
     SYS_sprintf( TextBuffer, "%u %x", value, value );           
     PutTxString( TextBuffer );
     EraseRestOfLine();
}

// ********************************************************************************************************
// *
// * Function: Re();
// *
// * Purpose: READ EEP.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function prints the value of an eep.
// *
// ********************************************************************************************************
void Debugger::Re()
{
     uint16_t value = 0;
     uint16_t eepAddress = 0;
 
     eepAddress = GetCommandArgument(1);
        
     GetEepData(eepAddress, 1, &value, TSK_1000_ms);   // read Eeprom
 
     SetCursor(20, 1);

     if ( EEStatusBits.w [ EE_STATUS_CHECKSUM_WORD ] )
     {
        PutTxChar('\\');    
     }  
     else
     {
        PutTxChar('*');
     }       
     
     PutTxString(EepValueIsStr);              // else, display read value
     PutTxChar('R');
     PutTxChar('#');
     SYS_sprintf( TextBuffer, "%d", eepAddress );
     PutTxString( TextBuffer );
     PutTxChar('=');
     SYS_sprintf( TextBuffer, "%u %x", value, value );           
     PutTxString( TextBuffer );
     EraseRestOfLine();
}


// ********************************************************************************************************
// *
// * Function: Rs();
// *
// * Purpose: RESET EEPROM.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function reloads ROM defaults to EEPROM
// *
// ********************************************************************************************************
void Debugger::Rs()
{
    uint16_t section = GetCommandArgument( 1 );
    
    SetCursor(20, 1);
    
    if ( 255 == section )
    {
        // reset entire eeprom
        uint16_t thisSection = EE_System_ID;
        
        while ( thisSection < EE_Last_Section )
        {
            ReBootEepromSection( thisSection );
            ++thisSection;    
        }
        PutTxString( "EEPROM Reset" );
    }
    else if ( section >= EE_System_ID && section < EE_Last_Section )
    {
        // reset single section
        ReBootEepromSection( section );
        SYS_sprintf( TextBuffer, "%s %d %s", "Section", section, "Reset" );
        PutTxString( TextBuffer );
    }
    else
    {
    	PutTxString( "Invalid section" );
    }
    EraseRestOfLine();

}

// ********************************************************************************************************
// *
// * Function: Dr();
// *
// * Purpose: DUMP data at a range of addresses.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function dumps the data from a range of specified addresses using the Dump screen.
// *
// ********************************************************************************************************
void Debugger::Dm()
{
    eBlockType format;          // this is the format of the data to be dumped

    DumpDS.memoryType = MemTypeINT;
    // parse start address
    DumpDS.startAddress = (uint16_t*)GetCommandArgument(1);

    // parse number of words to read
    DumpDS.length = (uint16_t)GetCommandArgument(2);

    // parse format
    format = (eBlockType)GetCommandArgument(3);

    if ( NONE == format )    // if a format wasn't specified, default to unsigned int
    {
        format = DEC0;
    }
    DumpDS.format = format;
    
    if ( ( DEC0 == DumpDS.format ) || ( HEX == DumpDS.format ) )
    {
        DumpDS.memoryType = MemTypeUINT;
    }
    else
    {
        DumpDS.memoryType = MemTypeFLOAT;
    }        

    // set which screen to draw
    SetScreenFunction( eDumpScreen );
}

// ********************************************************************************************************
// *
// * Function: De();
// *
// * Purpose: DUMP eeps at a range of addresses.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function dumps the data from a range of specified addresses using the Dump screen.
// *
// ********************************************************************************************************
void Debugger::De()
{
    int32_t format;                     // this is the format of the data to be dumped


    DumpDS.memoryType = MemTypeEEPROM;  //Get memory from EEPROM

    // get start address
    DumpDS.startAddress = (void*)GetCommandArgument(1);

    // get length
    DumpDS.length = (uint16_t)GetCommandArgument(2);

    // get format
    format = GetCommandArgument(3);

    if ( -1 == format )                    // if a format wasn't specified, default to hex data
    {
        format = DEC0;
    }
    DumpDS.format = (eBlockType)format;

    // set which screen to draw
    SetScreenFunction( eDumpScreen );

    ReDrawON = true;
}

// ********************************************************************************************************
// *
// * Purpose: list Eeps that differ from their default value
// *
// * Description:
// *
// ********************************************************************************************************
void Debugger::Dd()
{
	uint16_t index, addr, val;
	uint16_t length = GetEETableSize();
	SetScreenFunction(eLogScreen);
	ClrScrn();
	SetCursor(3, 0);
	PutTxString("addr\tdef\tval\r\n");

	index = 0;
	const EE_ID* ee = &eeprom[ index ];  //Jacob/20130814/Merge 120k //Keming/20120803I, change Eeprom map name

	for (index = 0; index < length; index++)
	{
		addr = ee[index].eep_addr;
		if (addr < MAX_EE_LENGTH && addr > 0) //unused addresses give larger values
		{
			if (GetEepData(addr, 1, &val, TSK_1000_ms))
			{
				if (ee[index].eep_DefaultValue[CoefficientsIndex] != val)
				{ // addr value
					SYS_sprintf(TextBuffer, "%u\t%u\t%u\r\n", addr,
							ee[index].eep_DefaultValue[CoefficientsIndex], val);
					PutTxString(TextBuffer);
				}
			}
		}
	}
}

// ********************************************************************************************************
// *
// * Function: Cc();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a charger command function
// *
// ********************************************************************************************************
void Debugger::Cc()
{
    uint16_t command;     // Command to execute
    float DutyCmd = 50.0;
    float tempArgument = 0;
    const float PWMPeriod = float( float( CPUFrequency / ChargeFrequency) / 2.0f );

        // Get Command
    command = GetCommandArgument(1);
    tempArgument = (float) GetCommandArgument(2);  //duty cycle
    DutyCmd = tempArgument/100.0;
    
    switch(command)
    {
        case 0:
            BatteryConverter.ChargerOff();
            break;
        case 1:
            MasterPWMOn();
            ChargerPWMOn(); 
			EPwm2Regs.CMPA.half.CMPA = (uint16_t)( DutyCmd * PWMPeriod );	//5A:Q1_a
			EPwm2Regs.CMPB			 = (uint16_t)( DutyCmd * PWMPeriod );	//5B:Q1_b/Q3_b:  wombat notuse
		//	EPwm6Regs.CMPA.half.CMPA = (uint16_t)( DutyCmd * PWMPeriod );	//6A:Q3_a
            break;
        case 2:
            BatteryConverter.ChargerOnOpenLoop();
            break;
        case 3:
            BatteryConverter.ChargerOn();
            break;
        case 4:
            BatteryConverter.SetChargeVoltage(tempArgument);
            break;
        case 5:
            BatteryConverter.SetChargeCurrentLimit(tempArgument);
            break;            
        case 6:
            BatteryConverter.ChrgRelayCmdOn();
            break;
        case 7:
            BatteryConverter.ChrgRelayCmdOff();
            break;
        case 8:
            BatteryConverter.ChargerCmdOn();
            break;
        case 9:
            BatteryConverter.ChargerCmdOff();
            break;
        case 10:
            tempArgument = tempArgument / 100000.0;
            BatteryConverter.ChargeVGainsPos.B0 = tempArgument;
            break;   
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: Cb();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a boost command function
// *
// ********************************************************************************************************
void Debugger::Cb()
{
    uint16_t command;     // Command to execute
    float DutyCmd = 50.0;
    float Current = 0;
    float tempArgument = 0;
    const float PWMPeriod = float( float( CPUFrequency / BoostFrequency) / 2.0f );
    
    
        // Get Command
    command = GetCommandArgument(1);
    tempArgument = (float) GetCommandArgument(2);  //duty cycle
    Current = tempArgument;
    DutyCmd = tempArgument/100.0;
    
    switch(command)
    {
        case 0:
            BatteryConverter.BoostOff();
            BoostLegBPWMOff();
            Rectifier.DisableLegBTest();
        	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through
        	EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
			EPwm3Regs.TBPHS.half.TBPHS = 0;
            break;
        case 1:
        	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
        	EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
        	EPwm3Regs.TBPHS.half.TBPHS = 4165; //4165
            MasterPWMOn();
            BatteryConverter.BoostPWMOn();
            BoostLegBPWMOn();
			EPwm3Regs.CMPA.half.CMPA = (uint16_t)( DutyCmd * PWMPeriod );	//5A:Q2_a
			EPwm3Regs.CMPB			 = (uint16_t)( DutyCmd * PWMPeriod );	//5B:Q2_b:  wombat notuse
			EPwm2Regs.CMPA.half.CMPA = (uint16_t)( DutyCmd * PWMPeriod );
			EPwm2Regs.CMPB = (uint16_t)( DutyCmd * PWMPeriod );
            break;
        case 2:
            BatteryConverter.BoostOnOpenLoop();            
            break;  
        case 3:
            BatteryConverter.BoostOn();
            break;    
        case 4:
            BatteryConverter.SetHWCurrentLimit(Current); 
            break; 
        case 5:
            tempArgument = tempArgument / 1000.0;
            BatteryConverter.BoostVGainsPos.B0 = tempArgument;
            break;
        case 6:
            BatteryConverter.BatteryTestCmdOn();
            break;
        case 7:
        	ConfigBoostLegBPWMOn();
			Rectifier.EnableLegBTest();
			BoostLegBPWMOn();
			break;
        case 9:
			break;
        case 11:
        	ConfigBoostLegBPWMOn();
			Rectifier.EnableLegBTest();
			BoostLegBPWMOn();
			BatteryConverter.BoostOn();
			break;
        case 12:
			MasterPWMOn();
			BatteryConverter.BoostPWMOn();
			EPwm2Regs.CMPA.half.CMPA = (uint16_t)( DutyCmd * PWMPeriod );	//5A:Q2_a
			EPwm2Regs.CMPB			 = (uint16_t)( DutyCmd * PWMPeriod );	//5B:Q2_b:  wombat notuse
			break;
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: Ci();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a inverter command function
// *
// ********************************************************************************************************
void Debugger::Ci()
{
    uint16_t command, InvPhase;     // Command to execute
    float temp;
    int32_t arg2 = 0;

        // Get Command
    command = GetCommandArgument(1);
    temp = (float) GetCommandArgument(2);  //duty cycle
    InvPhase = GetCommandArgument(2);   // case 14: 1->phA, 2->phB, 3->phC
    arg2 = GetCommandArgument(2); //duty cycle

    switch(command)
    {
        case 0:
            Inverter.Off();
            break;

        case 1:
        	Inverter.StartFixedDuty( (int16_t)arg2 );
            InverterPWMOn();
            break;
            
        case 2:
            Inverter.OnOpenLoop();
            break;
            
        case 3:
            Inverter.Off();
            break;
                    
        case 4:
            Inverter.SetHWCurrentLimit(temp);
            break;
            
        case 5:
            Inverter.On();  //closed loop
            break;
            
        case 8:
            Inverter.EnableRMS();
            break;
            
        case 9:
            Inverter.DisableRMS();
            break;
            
        case 12:
            if ( !Inverter.MatchBypassForced )
            {
            	Inverter.MatchBypassForced = true;
            	Inverter.StoredBypassMatchState = Inverter.InverterStatus.bit.MatchBypassRMS;
            }
            Inverter.InverterStatus.bit.MatchBypassRMS = 1;
            break;

        case 13:
            if ( !Inverter.MatchBypassForced )
            {
	            Inverter.MatchBypassForced = true;
	            Inverter.StoredBypassMatchState = Inverter.InverterStatus.bit.MatchBypassRMS;
            }
            Inverter.InverterStatus.bit.MatchBypassRMS = 0;
            break;
            
        case 14:
        	   switch(InvPhase)
             {
               case 1:
                 Inverter.InverterVmag.phA = 0;
                 break;

               case 2:
                 Inverter.InverterVmag.phB = 0;
                 break;
            
               case 3:
                 Inverter.InverterVmag.phC = 0;
                 break;
                 
               default:
               	 break;  
             }
             break;
        
        case 15:  // stop forcing bypass matching
            if ( Inverter.MatchBypassForced )
            {
                Inverter.InverterStatus.bit.MatchBypassRMS = Inverter.StoredBypassMatchState;
                Inverter.MatchBypassForced = false;
            }
            break;
        
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: Cr();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a rectifier command function
// *
// ********************************************************************************************************
void Debugger::Cr()
{
    uint16_t command;     // Command to execute
    int32_t  arg2 = 0;
        
        // Get Command
    command = GetCommandArgument(1);
    arg2 = GetCommandArgument(2);

    switch(command)
    {
            // Turn Rectifier Off
        case 0:
            Rectifier.RectifierOff();
            break;

            // Turn Rectifier On fixed duty cycle, no control
        case 1:
        	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through
        	EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
        	EPwm3Regs.TBPHS.half.TBPHS = 0;
            Rectifier.StartFixedDuty( (int16_t)arg2 );
            break;
        
        case 2:
            Rectifier.OnNormal();
            break;    
        
        case 4:
            Rectifier.SetHWCurrentLimit( arg2 );
            break;

        case 5:
            Rectifier.StartupRectifier();
            break;

        case 6:
            Rectifier.StartBatteryCycleTest();
            break;

        case 7:
            Rectifier.StopBatteryCycleTest();
            break;

        case 8:
            Rectifier.SetMixedModeGain( arg2 );
            break;

        case 10:	//cr 10 xx;   //1:on RecA; 2:on RecB; 3:on RecC 			
            Rectifier.RecOnOnePhase(arg2);
            Rectifier.OnNormal();
            break;    
            
        default:
            break;
            
    }
}


// ********************************************************************************************************
// *
// * Function: Cm();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a mcu command function
// *
// ********************************************************************************************************
void Debugger::Cm()
{
    uint16_t command;     // Command to execute

        // Get Command
    command = GetCommandArgument(1);

    switch(command)
    {
        case 0:  // Set DSP port A
            DSPOutRegister.GpoA.all = 0xffffffff;
            break;

        case 1:  // Clear DSP port A
            DSPOutRegister.GpoA.all = 0;
            break;
            
        case 2: // Set DSP port B
            DSPOutRegister.GpoB.all = 0xffffffff;
            break;
            
        case 3: // Clear DSP port B
            DSPOutRegister.GpoB.all = 0;
            break;
            
        case 4: // Set DSP port C
            DSPOutRegister.GpoC.all = 0xffffffff;
            break;
            
        case 5: // Clear DSP port C
            DSPOutRegister.GpoC.all = 0;
            break;
                
        case 6: // trip battery breaker
            DSPOutRegister.GpoB.bit.TripBatteryBreaker = 1;
            break;
            
        case 7: // reset battery breaker IO
            DSPOutRegister.GpoB.bit.TripBatteryBreaker = 0;
            break;
            
        case 8: // close battery relay 1
            DSPOutRegister.GpoB.bit.BatteryRelay1 = 1;
            break;
            
        case 9: // open battery relay 1
            DSPOutRegister.GpoB.bit.BatteryRelay1 = 0;
            break;        
            
        case 10:    // close battery relay 2
            DSPOutRegister.GpoB.bit.BattLegB_Relay = 1;
            break;
            
        case 11:    // open battery relay 2
            DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
            break;
            
        case 12:    // close input relay L1
		  	float tempApk = ScreenMeters.InputVoltageRMS.phA * SQRT_2 - 40;
			if((RawAdcDataPtr->st.RailVoltagePositive > tempApk) && (RawAdcDataPtr->st.RailVoltageNegative < (-tempApk)))
			{
	            DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
        	}
            break;
            
        case 13:    // open input relay L1
            DSPOutRegister.GpoB.bit.InputRelay_L1 = 0;
            break;
            
        case 14:    // close input relay L2-3
		  	float tempBpk = ScreenMeters.InputVoltageRMS.phB * SQRT_2 - 40;
		  	float tempCpk = ScreenMeters.InputVoltageRMS.phC * SQRT_2 - 40;
			if(((RawAdcDataPtr->st.RailVoltagePositive > tempBpk) && (RawAdcDataPtr->st.RailVoltageNegative < (-tempBpk)))
				&& ((RawAdcDataPtr->st.RailVoltagePositive > tempCpk) && (RawAdcDataPtr->st.RailVoltageNegative < (-tempCpk))))
			{
				DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
        	}        
            break;
            
        case 15:    // open input relay L2-3
            DSPOutRegister.GpoB.bit.InputRelay_L23 = 0;
            break;    
            
        case 16:    // close inverter relay
            MCUStateMachine.SetInverterRelay( RELAY_CLOSED );
            break;    
            
        case 17:    // open inverter relay
            MCUStateMachine.SetInverterRelay( RELAY_OPEN );
            break;
            
        case 18:    // close backfeed relay
            DSPOutRegister.GpoB.bit.BackFeedRelay = 1;
            break;
            
        case 19:    // open backfeed relay
            DSPOutRegister.GpoB.bit.BackFeedRelay = 0;
            break;
            
        case 20:    // close balancer relay
            DSPOutRegister.GpoB.bit.BalancerRelay = 1;
            break;
            
        case 21:    // open balancer relay
            DSPOutRegister.GpoB.bit.BalancerRelay = 0;
            break;           
            
        case 26:
            PreChargeOn();
            break;
            
        case 27:
            PreChargeOff();
            break;
            
        case 28:
            MCUStateMachine.NormalCommand();
            break;
            
        case 29:
            MCUStateMachine.Turn_UPS_Off();
            break;
            
        case 30:
            MCUStateMachine.NormalCommand();
            //MCUStateMachine.AutoModeCommand();
            break;
            
        case 31:
            //MCUStateMachine.AutoModeOffCommand();
            break;
            
        case 32:
            MCUStateMachine.StandbyCommand();
            break;
            
        case 33:
            MasterPWMOn();
            break;
            
        case 34:
            MasterPWMOff();
            break;             

        case 37:
            SetIOGood( true );
            break;

        case 38:
            SetIOGood( false );
            break;
            
        case 39:
            MCUStateMachine.EnableTestMode();
            break;

        case 40:
            MCUStateMachine.DisableTestMode();
            break;

        case 41:
            BypassState().RequestBypassState( BYPASS_OFF_STATE );
            break;

        case 42:
            BypassState().RequestBypassState( BYPASS_READY_STATE );
            break;

        case 43:
            BypassState().RequestBypassState( BYPASS_PRIMED_STATE );
            break;

        case 44:
            BypassState().RequestBypassState( BYPASS_FIRE_STATE );
            break;

        case 45:
            DSPOutRegister.GpoC.bit.Inv_Online = 0;
            break;

        case 46:
            DSPOutRegister.GpoC.bit.Inv_Online = 1;
            break;

        case 47:
            DSPOutRegister.GpoC.bit.Byp_Avail = 0;
            break;

        case 48:
            DSPOutRegister.GpoC.bit.Byp_Avail = 1;
            break;
            
        case 49:
            // Warning: Undefined behavior unless this really is UPM0
            static_cast<BypassStateMachine&>(BypassState())
                .Backfeed->ForceClosed( true );
            break;
            
        case 50:
            // Warning: Undefined behavior unless this really is UPM0
            static_cast<BypassStateMachine&>(BypassState())
                .Backfeed->ForceClosed( false );
            break;    
            
        case 51:
        	DSPOutRegister.GpoC.bit.Supply_24VOff = 1; //turns off supply
            break; 
            
        case 52:
            ClearLatchedEPO();
            break;

        case 53:
            AutoCal.StartAutoCal();
            break;

        case 54:
            static_cast<BypassStateMachine&>(BypassState())
                .Backfeed->ForceOpen( true );
            break;

        case 55:
            static_cast<BypassStateMachine&>(BypassState())
                .Backfeed->ForceOpen( false );
            break;
            
        case 56:    // fail close inverter relay
            if ( !MCUStateMachine.MCUStatus.bit.InverterContactorTest )
            {
	            MCUStateMachine.StoredInverterRelayState = DSPOutRegister.GpoB.bit.InverterRelay;
	            MCUStateMachine.MCUStatus.bit.InverterContactorTest = 1;
            }
            DSPOutRegister.GpoB.bit.InverterRelay = 1;
            break;    
            
        case 57:    // fail open inverter relay
            if ( !MCUStateMachine.MCUStatus.bit.InverterContactorTest )
            {
	            MCUStateMachine.StoredInverterRelayState = DSPOutRegister.GpoB.bit.InverterRelay;
	            MCUStateMachine.MCUStatus.bit.InverterContactorTest = 1;
            }
            DSPOutRegister.GpoB.bit.InverterRelay = 0;
            break;
        
        case 58: // clear inverter relay failure test fault
            if ( MCUStateMachine.MCUStatus.bit.InverterContactorTest )
            {
	            DSPOutRegister.GpoB.bit.InverterRelay = MCUStateMachine.StoredInverterRelayState;
	        	MCUStateMachine.MCUStatus.bit.InverterContactorTest = 0;
            }
        	break;
        	
        case 59:
            // test the shutdown ETB branch
            MCUStateMachine.CheckOutputACUV = ONE_SECOND;
            break;

        case 60: // test ADC timer
        	if(GetCommandArgument(2) == 0)
        	{
        		EssTimeTest = false;
        		//ADCTimeTest = false;
        	}
        	else if(GetCommandArgument(2) == 1)
        	{
        		EssTimeTest = true;
        	}
            break;
           
        case 61:
            // Simulate corner-grounded input fault
//	            DirectInputNeutral = DirectNeutralMeter(
//	                const_cast<const float&>(RawAdcDataPtr->st.InputVoltageNeutral));
            break;
        
        case 62:
        	// Force a transfer out of HE mode through PCAN
        	MCUStateMachine.MCUStatus.bit.ForwardTransfer = 1;
        	break;

        case 63:
        	// Write system type to EEPROM 11
        	SetSystemType(GetCommandArgument(2),GetCommandArgument(3));
        	break;

        case 64:
        	// Adjust voltage forward gains
        	VoltageForwardGain = float(GetCommandArgument(2))/100.0;
        	break;

        case 65:
        	// it can used to clear "configure error" when no CSB connected
        	InternalCan.SetSystemTypeCheckResult( true );
        	break;

        case 70:
        	// SET ECT power: 7000 -> 70%
			MCUStateMachine.SetECTPower( GetCommandArgument(2) );
        	break;

        case 71:
        	// set ECT PF   80~120
        	Inverter.SetECTPowerfactor( GetCommandArgument(2) );
        	break;

        case 72:
        	//bat  ECT time: minute
        	MCUStateMachine.SetBatECTDuration ( (uint32_t)GetCommandArgument(2) * 60L );
        	break;

        case 73:
	        //line ECT time: minute
			MCUStateMachine.SetLineECTDuration( (uint32_t)GetCommandArgument(2) * 60L );
        	break;

        case 74:
        	// ECT command
            if( MyUPMNumber == 0 )
            {
                ParallelCan.C9Command.bit.c9_ect_on_command = 1;
            }
        	break;

		case 75:
			Inverter.SetECTPowerAndPf( GetCommandArgument(2), GetCommandArgument(3) );
			break;

 		case 76:
			break;

 		case 77:
            NB_SetNodebit( UPM_NB_STATIC_SWITCH_SHORT, 1, 99);
            break;

        case 95:
            if( MCUStateMachine.UPMTestMode )
            {
                ActivePullChain();
            }
            break;

        case 96:
            if( MCUStateMachine.UPMTestMode )
            {
                DeActivePullChain();
            }
            break;

        case 98:
            ResetStickyAlarms();
            break;

        case 99:
            HistoryQueue.EraseAllEvents();
            break;  

        case 101:
            dcff_test = 1; //enable dcff
            break;

        case 102:
            dcff_test = 0; //disable dcff
            break;

        case 110:
            break;

        case 9999:
            PreventWatchdogKick = 1; //should cause watch dog interrupt and reset
            break;

        default:
            PutTxString( UnRecComStr );
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: Ct();
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Executes a mcu command function
// *
// ********************************************************************************************************
void Debugger::Ct()
{
    uint16_t command;     // Command to execute
    uint16_t coefficient_type;        //B0,B1,B2,A1,A2
    uint16_t data;
    uint16_t scale;

        // Get Command
    command = GetCommandArgument(1);
    coefficient_type = GetCommandArgument(2);
    data = GetCommandArgument(3);

    scale = GetCommandArgument(2);

    switch(command)
    {
        case 0:  // Set DSP port A
        	CoefficientsScale = scale;
            break;

		// Inverter
		case 10:
			DebugCoefficientsFirst = Inverter.GetInverterVRMSGains();
			UpdateControllerCoefficentsFirst(coefficient_type, data, InverterRMSLoopCoefficients_ptr);
			Inverter.SetInverterVRMSGains(&DebugCoefficientsFirst);
			break;

		case 11:
			DebugCoefficients = Inverter.GetInverterVGains1();
			UpdateControllerCoefficents(coefficient_type, data, InverterVoltageLoopCoefficients_ptr);
			Inverter.SetInverterVGains1(&DebugCoefficients);
			break;

		case 12:
			DebugCoefficients = Inverter.GetInverterVGains2();
			UpdateControllerCoefficents(coefficient_type, data, InverterVoltageLoopCoefficients2_ptr);
			Inverter.SetInverterVGains2(&DebugCoefficients);
			break;

		case 13:
			DebugCoefficients = Inverter.GetInverterIGains();
			UpdateControllerCoefficents(coefficient_type, data, InverterCurrentLoopCoefficients_ptr);
			Inverter.SetInverterIGains(&DebugCoefficients);
			break;

		case 14:
			DebugCoefficients = Inverter.GetInverterDCCompensator();
			UpdateControllerCoefficents(coefficient_type, data, InverterDCCompensationCoefficients_ptr);
			Inverter.SetInverterDCCompensator(&DebugCoefficients);
			break;

		case 15:	//ct 15 0 xx:  Id_fil
			DebugCoefficientsFirst = Inverter.GetLoadShareDroopTable();
			UpdateControllerCoefficentsFirst(coefficient_type, data, LoadShareDroopCoefficients_ptr);
//			UpdateControllerCoefficents(coefficient_type, data, LoadShareDCFeedForwardCoefficients_ptr);
			Inverter.SetLoadShareDroopTable(&DebugCoefficientsFirst);
			break;

		case 16:
			DebugCoefficientsFirst = Inverter.GetECTPowerGains();
			UpdateControllerCoefficentsFirst(coefficient_type, data, ECTPowerLoopCoefficients_ptr);
			Inverter.SetECTPowerGains(&DebugCoefficientsFirst);
			break;

		case 17:
			DebugCoefficientsFirst = Inverter.GetECTInverterIGains();
			UpdateControllerCoefficentsFirst(coefficient_type, data, ECTCurrentLoopCoefficients_ptr);
			Inverter.SetECTInverterIGains(&DebugCoefficientsFirst);
			break;

        // Battery
        case 20:
        	DebugCoefficientsFirst = BatteryConverter.GetBoostVGains();
        	UpdateControllerCoefficentsFirst(coefficient_type, data, BoostVGains_ptr);
        	BatteryConverter.SetBoostVGains(&DebugCoefficientsFirst);
        	break;

        case 21:
        	DebugCoefficientsFirst = BatteryConverter.GetBoostPowerModeGains();
        	UpdateControllerCoefficentsFirst(coefficient_type, data, BoostPowerModeGains_ptr);
        	BatteryConverter.SetBoostPowerModeGains(&DebugCoefficientsFirst);
        	break;

        case 22:
        	DebugCoefficientsFirst = BatteryConverter.GetChargeVGains();
        	UpdateControllerCoefficentsFirst(coefficient_type, data, ChargeVGains_ptr);
        	BatteryConverter.SetChargeVGains(&DebugCoefficientsFirst);
        	break;

        case 23:
        	DebugCoefficientsFirst = BatteryConverter.GetOuterChargeIGains();
        	UpdateControllerCoefficentsFirst(coefficient_type, data, OuterChargeIGains_ptr);
        	BatteryConverter.SetOuterChargeIGains(&DebugCoefficientsFirst);
        	break;

		// Rectifier
		case 30:
			DebugCoefficientsFirst = Rectifier.GetLinkVoltageTable();
			UpdateControllerCoefficentsFirst(coefficient_type, data, RectifierVoltageLoopCoefficients_ptr);
			Rectifier.SetLinkVoltageTable(&DebugCoefficientsFirst);
			break;

//		case 31:
//			DebugCoefficientsFirst = Rectifier.GetLinkOffsetTable();
//			UpdateControllerCoefficentsFirst(coefficient_type, data, RectifierOffsetCoefficients_ptr);
//			Rectifier.SetLinkOffsetTable(&DebugCoefficientsFirst);
//			break;

//		case 32:
//			DebugCoefficientsFirst = Rectifier.GetFastLinkOffsetTable();
//			UpdateControllerCoefficents(coefficient_type, data, RectifierCurrentLoopCoefficients1_ptr);
//			Rectifier.SetFastLinkOffsetTable(&DebugCoefficientsFirst);
//			break;

		case 33:
			DebugCoefficients = Rectifier.GetPFCCurrentLoop();
			UpdateControllerCoefficents(coefficient_type, data, RectifierCurrentLoopCoeffiNewRecTopo_ptr);
			Rectifier.SetPFCCurrentLoop(&DebugCoefficients);
			break;

		//4. Loadshare: CT 40 0 15, Iq_fil
		//4.0 ReactiveLoadShareDroopTable
		case 40:
			DebugCoefficientsFirst = Inverter.GetReactiveLoadShareDroopTable();
			UpdateControllerCoefficentsFirst(coefficient_type, data, ReactiveLoadShareDroopCoefficients_ptr);
			Inverter.SetReactiveLoadShareDroopTable(&DebugCoefficientsFirst);
			break;		
	
		//4.1 ReactivePowerDroopTable, Iq_nofil
		case 41:
			DebugCoefficientsFirst = Inverter.GetReactivePowerDroopTable();
			UpdateControllerCoefficentsFirst(coefficient_type, data, ReactivePowerDroopCoefficients_ptr);
			Inverter.SetReactivePowerDroopTable(&DebugCoefficientsFirst);
			break;		

		//4.2 LoadShareDCFeedForwardTable
		case 42:
			DebugCoefficientsFirst = Inverter.GetLoadShareDCFeedForwardTable();
			UpdateControllerCoefficentsFirst(coefficient_type, data, LoadShareDCFeedForwardCoefficients_ptr);
			Inverter.SetLoadShareDCFeedForwardTable(&DebugCoefficientsFirst);
			break;		

		//4.3 HighDCVPhaseDroopTable
		case 43:
			DebugCoefficients = Inverter.GetHighDCVPhaseDroopTable();
			UpdateControllerCoefficents(coefficient_type, data, HighDCVPhaseDroopCoefficients_ptr);
			Inverter.SetHighDCVPhaseDroopTable(&DebugCoefficients);
			break;		

        default:
        	break;
    }
}

bool Debugger::Abort(void)
{
	bool result = false;
	uint16_t tempSize;
	char tempBuf[32] = {0};


	tempSize = SizeRxBuf();
	if (tempSize != 0)
	{
		GetChars( &tempBuf[0], tempSize );
		if (tempBuf[tempSize - 1 ] == ESC)
		{
			result = true;
			PutTxString("Command aborted.\n");
		}
	}

	return result;
}

// ********************************************************************************************************
// *
// * Function: s();
// *
// * Purpose: Sample Data at a given address at a given multiple of the PWM frequency
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function samples a specified memory location and dumps it to the screen.
// *                Command Buffer Argument 1 = Memory Address
// *                Command Buffer Argument 2 = Frequency
// *
// ********************************************************************************************************
void Debugger::S(void)
{
    uint32_t i;
    int32_t data;

    // arg1 = address
    data = GetCommandArgument( 1 );
    
    // check for valid address
    if ( data <= 0xffff )
    {
        // type doesn't matter here
        if ( data < RAW_ADC_SIZE )
        {
            // read A/D channel
            CaptureControl.Address.ptrInt = (int16_t*)&RawAdcDataPtr->w[ data ];
        }
        else
        {
            // read RAM address
            CaptureControl.Address.ptrInt = (int16_t*)data;
        }        
    }
    else
    {
        PutTxString( "Invalid Address\r\f" );
        return;    
    }    
 
    // arg2 = frequency
    data = GetCommandArgument( 2 );
    if ( data == -1 )
    {
        // Capture frequency of -1 means no argument was given.  Default to 0.
        data = 0;
    }
    CaptureControl.Frequency = (uint16_t)data;
    
    // arg3 = memory type
    data = GetCommandArgument( 3 );
    // set to float if none given
    if ( -1 == data )
    {
        data = 0;
    }
    
    if ( 0 == data )
    {
        CaptureControl.LongData = true;
    }
    else
    {        
        CaptureControl.LongData = false;
    }

    // arg4 = timeout interval
	data = GetCommandArgument( 4 );
	if (data <= 0)
	{
		data = 10000; // 10 seconds by default
	}
	else if (data > 1000L * 3600L * 24L)
	{
		data = 1000L * 3600L * 24L; // maximum 24 hours
	}
	else
	{
		// do nothing
	}
	CaptureControl.Timeout = (uint32_t)(data / 5); // this task sleeps for 5 ms

#if CAPTURE_DUAL_CHANNEL == 0x55AA
	CaptureControl.Count2ndChan = 0U;
	for( i = 0; i < CAPTUR_2ND_BUFFER_SIZE; i++ )
	{
		Capture2ndBuffer[i] = 0U;
	}
	if ((CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_INT16)
		|| (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_UINT16)
		|| (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_BITSET))
	{
		CaptureControl.LongData2ndChan = false;
	}
	else
	{
		CaptureControl.LongData2ndChan = true;
	}
#endif

     // First zero capture array
    for( i = 0; i < CAPTURE_SIZE; i++ )
    {
        CaptureData[i].l = 0;
    }
    
    CaptureControl.DataPtr.ptrLong = (int32_t*)&CaptureData[0].l;
 
    CaptureControl.Count = 0;
    CaptureControl.FrequencyCount = 0;
    CaptureControl.Complete = false;
    CaptureControl.ArmTrigger = false; /* avoid trigger flag be override by result of trigger condition validation */
    CaptureControl.Start = true;  
    i = 0;
 	/* count in the sampling time, minimum 100 milliseconds */
    while( !CaptureControl.Complete
		   && (i < (CaptureControl.Timeout + 100U / 5U * (CaptureControl.Frequency + 1U)))
		   && !Abort())
    {
        i++;// Do Nothing.  Wait for capture complete.  Timeout if necessary.
        TSK_sleep(TSK_5_ms);
    }
 
    // Dump data to the screen.
    if ( CaptureControl.LongData )
    {
        DumpDS.memoryType = MemTypeFLOAT;
        DumpDS.format = DEC4;
    }
    else
    {
        DumpDS.memoryType = MemTypeINT;
        DumpDS.format = DEC0;
    }    

    // set which screen to draw
    SetScreenFunction( eDumpSamplesScreen );
 
    ReDrawON = true;
}


// ********************************************************************************************************
// *
// * Function: t();
// *
// * Purpose: same as sample but has a trigger source and trigger level
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function samples a specified memory location and dumps it to the screen.
// *                Command Buffer Argument 1 = Memory Address
// *                Command Buffer Argument 2 = Frequency
// *
// ********************************************************************************************************
void Debugger::T(void)
{
    uint16_t command;
    uint32_t i; 
    int32_t data;
    
    // arg1 = address
    command = GetCommandArgument( 1 );
    data = GetCommandArgument( 2 );
    
    switch(command)
    {
        case 0:  
             // First zero capture array
            for( i = 0; i < CAPTURE_SIZE; i++ )
            {
                CaptureData[i].l = 0;
            }
            CaptureControl.DataPtr.ptrLong = (int32_t*)&CaptureData[0].l;
            if(data != -1)
            {
                CaptureControl.Frequency = data;
            }
            else
            {
                CaptureControl.Frequency = 2;
            } 
            data = GetCommandArgument( 3 );
            if (data <= 0)
            {
            	data = 10000L; // 10 seconds by default
            }
            else if (data > 1000L * 3600L * 24L)
            {
            	data = 1000L * 3600L * 24L; // maximum 24 hours
            }
            else
            {
            	// do nothing
            }
            CaptureControl.Timeout = (uint32_t)(data / 5); // this task sleeps for 5 ms

		#if CAPTURE_DUAL_CHANNEL == 0x55AA
			CaptureControl.Count2ndChan = 0U;
			for( i = 0; i < CAPTUR_2ND_BUFFER_SIZE; i++ )
			{
				Capture2ndBuffer[i] = 0U;
			}
			if ((CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_INT16)
				|| (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_UINT16)
				|| (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_BITSET))
			{
				CaptureControl.LongData2ndChan = false;
			}
			else
			{
				CaptureControl.LongData2ndChan = true;
			}
		#endif

            //CaptureControl.LongData = true;
            if ((CaptureControl.SourceSignalType == WAVECAPTURE_SIG_INT16)
				|| (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT16)
				|| (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_BITSET))
			{
				CaptureControl.LongData = false;
			}
			else
			{
				CaptureControl.LongData = true;
			}
            //CaptureControl.ArmTrigger = true;         
            CaptureControl.Count = 0;
            CaptureControl.FrequencyCount = 0;
            CaptureControl.Complete = false;
            CaptureControl.Start = false;  
			CaptureControl.ArmTrigger = true;   // start detecting trigger condition only after all variables are initialized
            i = 0;
         
            while( !CaptureControl.Complete && !Abort())
            {
                i++;// Do Nothing.  Wait for capture complete.  Timeout if necessary.
                if (CaptureControl.Trigger)
                {
                	/* count in the sampling time, minimum 100 milliseconds */
	                if (i >= (CaptureControl.Timeout + 100UL / 5U * (CaptureControl.Frequency + 1U)))
	                {
	                	break;
	                }
                }
                else
                {
                	/* in case the trigger condition cannot be met */
					if (i >= CaptureControl.Timeout)
	                {
						CaptureControl.ArmTrigger = false; // single trigger mode
						break;
	                }
                }
                TSK_sleep(TSK_5_ms);
            }

            // Dump data to the screen.
            if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT16)
			{
				DumpDS.memoryType = MemTypeUINT;
				DumpDS.format = DEC0;
			}
			else if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT32)
			{
				DumpDS.memoryType = MemTypeULONG;
				DumpDS.format = DEC0;
			}
			else if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_BITSET)
			{
				DumpDS.memoryType = MemTypeUINT;
				DumpDS.format = HEX;
			}
			else
			{
				DumpDS.memoryType = MemTypeFLOAT;
				DumpDS.format = DEC4;
			}

            // set which screen to draw
            SetScreenFunction( eDumpSamplesScreen );
         
            ReDrawON = true;
            break;        
        case 1:   
            // trigger source
            if (data != -1)
            {
	            // check for valid address
	            if ( data <= RAW_ADC_SIZE )
	            {
	                // read A/D channel
	                CaptureControl.PtrTrig.ptrInt = (int16_t*)&RawAdcDataPtr->w[ data ];
	            }
	            else
	            {
					// read RAM address
	            	CaptureControl.PtrTrig.ptrInt = (int16_t*)data;
	            }
			#if defined(CAPTURE_TRIGGER_ENHANCE)
	            // to simplify use, trigger level is constant by default
	            CaptureControl.PtrTrigLevel.ptrInt = 0;
	        #endif
            }
		#if defined(CAPTURE_TRIGGER_ENHANCE)
            // specify trigger signal type, optional
            data = GetCommandArgument( 3 );
            if ((data >= WAVECAPTURE_SIG_FLOAT) && (data <= WAVECAPTURE_SIG_BITSET))
            {
            	CaptureControl.TriggerSignalType = (uint16_t)data;
            }
            else
            {
            	CaptureControl.TriggerSignalType = WAVECAPTURE_SIG_FLOAT;
            }

            // if trigger signal type is BITSET, specify bit number to detect flag or status change, optional
            if (CaptureControl.TriggerSignalType == WAVECAPTURE_SIG_BITSET)
            {
            	data = GetCommandArgument( 4 );
				if ((data >= 0) && (data < 16))
				{
					CaptureControl.TriggerBitNo = (uint16_t)data;
				}
				else
				{
					CaptureControl.TriggerBitNo = 0U;
				}
            }
         #endif
            break;
            
       case 2:
            // trigger level with two decimal digits
            CaptureControl.TriggerLevel = (float)data / 100;
		#if defined(CAPTURE_TRIGGER_ENHANCE)
            // specify a object or variable and its value will be used as trigger level, optional
            data = GetCommandArgument( 3 );
			if (data != -1)
			{
				CaptureControl.PtrTrigLevel.ptrInt = (int16_t*)data;
			}
			else
			{
				CaptureControl.PtrTrigLevel.ptrInt = 0;
			}
		#endif
            break;
            
       case 3:
       		if (data != -1)
       		{
	            // check for valid address
	            if ( data <= RAW_ADC_SIZE)
	            {
	                // read A/D channel
	                CaptureControl.Address.ptrInt = (int16_t*)&RawAdcDataPtr->w[ data ];
	            }
	            else
	            {
					// read RAM address
            		CaptureControl.Address.ptrInt = (int16_t*)data;
	            }
            }

       		// specify source signal type, optional
       		data = GetCommandArgument( 3 );
       		if ((data >= WAVECAPTURE_SIG_FLOAT) && (data <= WAVECAPTURE_SIG_BITSET))
       		{
       			CaptureControl.SourceSignalType = (uint16_t)data;
       		}
       		else
       		{
       			CaptureControl.SourceSignalType = WAVECAPTURE_SIG_FLOAT;
			}

            break;

       case 4:
    	   // select trigger edge or level
    	   if (data < 0)
    	   {
    		   data = 0;
    	   }
    	   CaptureControl.TriggerSel = (uint16_t)data;
    	   if (CaptureControl.TriggerSel > WAVECAPTURE_TRIG_LEVEL_LOW)
    	   {
    		   CaptureControl.TriggerSel = WAVECAPTURE_TRIG_EDGE_RISING;
    	   }
    	   break;
		#if CAPTURE_DUAL_CHANNEL == 0x55AA
       case 5:
    	    if (data != -1)
			{
				// check for valid address
				if ( data <= RAW_ADC_SIZE)
				{
					// read A/D channel
					CaptureControl.Address2ndChan.ptrInt = (int16_t*)&RawAdcDataPtr->w[ data ];
				}
				else
				{
					// read RAM address
					CaptureControl.Address2ndChan.ptrInt = (int16_t*)data;
				}
		    }

    	    // specify source signal type, optional
			data = GetCommandArgument( 3 );
    	    if ((data >= WAVECAPTURE_SIG_FLOAT) && (data <= WAVECAPTURE_SIG_BITSET))
			{
				CaptureControl.SourceSignalType2ndChan = (uint16_t)data;
			}
			else
			{
				CaptureControl.SourceSignalType2ndChan = WAVECAPTURE_SIG_FLOAT;
			}
    	    break;
		#endif
       case 6:
	#if defined(CAPTURE_TRIGGER_ENHANCE)
        #if CAPTURE_DUAL_CHANNEL == 0x55AA
    	   if (data == 1)
    	   {
    		   // print samples of channel 2
			   for (i = 0; i < CAPTUR_2ND_BUFFER_SIZE/2U && i < CAPTURE_SIZE; i++)
			   {
				   CaptureData[i].l = *(uint32_t *)(&Capture2ndBuffer[i * 2U]);
			   }
			   while (i < CAPTURE_SIZE)
			   {
				   CaptureData[i++].l = 0;
			   }
			   CaptureControl.LongData = CaptureControl.LongData2ndChan;

			   // Dump data to the screen.
				if (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_UINT16)
				{
					DumpDS.memoryType = MemTypeUINT;
					DumpDS.format = DEC0;
				}
				else if (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_UINT32)
				{
					DumpDS.memoryType = MemTypeULONG;
					DumpDS.format = DEC0;
				}
				else if (CaptureControl.SourceSignalType2ndChan == WAVECAPTURE_SIG_BITSET)
				{
					DumpDS.memoryType = MemTypeUINT;
					DumpDS.format = HEX;
				}
				else
				{
					DumpDS.memoryType = MemTypeFLOAT;
					DumpDS.format = DEC4;
				}
    	   }
    	   else
		#endif
    	   {
    		   // print samples of channel 1
    		   if ((CaptureControl.SourceSignalType == WAVECAPTURE_SIG_INT16)
					|| (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT16)
					|| (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_BITSET))
				{
					CaptureControl.LongData = false;
				}
				else
				{
					CaptureControl.LongData = true;
				}

    		   // Dump data to the screen.
				if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT16)
				{
					DumpDS.memoryType = MemTypeUINT;
					DumpDS.format = DEC0;
				}
				else if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_UINT32)
				{
					DumpDS.memoryType = MemTypeULONG;
					DumpDS.format = DEC0;
				}
				else if (CaptureControl.SourceSignalType == WAVECAPTURE_SIG_BITSET)
				{
					DumpDS.memoryType = MemTypeUINT;
					DumpDS.format = HEX;
				}
				else
				{
					DumpDS.memoryType = MemTypeFLOAT;
					DumpDS.format = DEC4;
				}
    	   }

    	   if (DumpDS.memoryType == MemTypeFLOAT)
			{
				data = GetCommandArgument( 3 );
				if ((data > 0) && (data < 10000))
				{
					floatDataGain = (uint16_t)data;
				}
				else
				{
					floatDataGain = 1U;
				}
			}

			// set which screen to draw
    	   SetScreenFunction( eDumpSamplesScreen );

    	   ReDrawON = true;
    	   break;
	#endif
       case 9:
            asm(" TRAP  #19");
            break;
            
       default:
            break;
    }  
            
                            
        
}
// ********************************************************************************************************
// *
// * Function: U();
// *
// * Purpose: Sets and clears UPS On/Off command
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: This function replaces start button on display when debugging
// *
// ********************************************************************************************************
void Debugger::U(void)
{
    uint16_t command;     // Command to execute
    
        // Get Command
    command = GetCommandArgument( 1 );

    switch(command)
    {
        case 0:                                             // turn UPS off like it would be done through XCP
            MCUStateMachine.Turn_UPS_Off();
			MCUStateMachine.ParalSystemOn = false;
            break;

        case 1:                                             // turn UPS "on" like it would be done through XCP
            MCUStateMachine.NormalCommand();
            ResetAlarms();			
			MCUStateMachine.ParalSystemOn = false;
            break;

        case 2:                                             // Turn UPS off like it would be turned off of front panel
            MCUStateMachine.StandbyCommand();
            break;
            
        case 3:
            MCUStateMachine.BypassCommand();
            break;
            
        case 4:
            MCUStateMachine.NormalCommand();
            break;        

        case 5:                                             // Start up rectifier, go to standby.
            MCUStateMachine.AutoStandbyCommand();
            break;
            
        case 6:
            MCUStateMachine.EcoOnCommand();
            break;

        case 7:
            MCUStateMachine.EcoOffCommand();
            break;
        
        case 8:
            {
	            if( MyUPMNumber == 0 )
                {
                    ParallelCan.C9Command.bit.c9_ect_on_command = 1;
                }  
            }
            break;
            
        case 9:
            {
                if( MyUPMNumber == 0 )
                {
                    ParallelCan.C9Command.bit.c9_ect_off_command = 1;
                }
            }
            break; 
            
        case 10:	//U 10,  inner paral shutdown
			if( MyUPMNumber == 0 )
			{
				ParallelCan.InternalCommand.bit.csb_load_off_command = 1;
			}	
            break;

        case 11:	//U 11,  inner paral online
			if( MyUPMNumber == 0 )
			{
				ParallelCan.InternalCommand.bit.csb_on_normal_command = 1;
				MCUStateMachine.ParalSystemOn = true;				
			}	 
            break;

		case 13:	//U 13,  inner paral bypass
			if( MyUPMNumber == 0 )
			{
				ParallelCan.InternalCommand.bit.csb_bypass_on_command = 1;
			}
			break;
            
        default:
            break;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: P() 
// *
// *    DESCRIPTION: Parameter commands
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void Debugger::P( void )
{
    char function       = commandBuf[1];
    parameter_t param   = (parameter_t)GetCommandArgument( 1 );
    uint16_t data       = (uint16_t)GetCommandArgument( 2 );
    
    SetCursor(20, 1);
    
    switch ( function )
    {
        case 'W':   // write
            WriteParameter( param, &data, 1 );
            SYS_sprintf( TextBuffer, "%s %u %u\r\f", "Parameter write:", (uint16_t)param, data );
            PutTxString( TextBuffer );
            //lint -fallthrough
        case 'R':
            ReadParameter( param, &data, 1 );
            SYS_sprintf( TextBuffer, "%s %u %u\r\f", "Parameter read:", (uint16_t)param, data );
            PutTxString( TextBuffer );
            EraseRestOfLine();
            break;
        default:
            PutTxString( UnRecComStr );
            break;    
                
    }
}

/******************************************************************************
function name : sBTCommand
Description : Transmit Bode Scan Data 
parameters : none
Used external var : none
Modify external var : none
returns : none
*******************************************************************************/
void Debugger::BTCommand( void )
{
    char TextBuffer[10] = {0};
    uint16_t cnt;
    
//	PutTxString("BT Command - Debugger is in Bode Mode.\n");     // just for debug
	
	if( FINISH_BODESCAN_MARK == 1 )
	{
        SYS_sprintf( TextBuffer, "%04u", uwpreF_INPUT );
        PutTxString( TextBuffer );
        PutTxChar( ' ' );

        SYS_sprintf( TextBuffer, "%04u", uwA_INPUT );
        PutTxString( TextBuffer );
        PutTxChar( ' ' );

        SYS_sprintf( TextBuffer, "%04u", uwCircle );
        PutTxString( TextBuffer );
        PutTxChar( ' ' );

        SYS_sprintf( TextBuffer, "%04u", dftn );
        PutTxString( TextBuffer );
        
        for( cnt = 0; cnt < dftn; cnt++ )
        {
        	PutTxChar( ' ' );
        	SYS_sprintf( TextBuffer, "%04u", CaptureData[cnt].s[0] );
            PutTxString( TextBuffer );
        }
        
        FINISH_BODESCAN_MARK = 0;
	}
	else
	{		
		SYS_sprintf( TextBuffer, "%04u", 0 );
	    PutTxString( TextBuffer );
	}
}

/******************************************************************************
function    : sBSCommand
Description : get parameters and set a mark to start Bode Scan.  
			  For example, enter Char 'B''S''1''4''0''0''5', means doing Bode Scan(freqence=140,amplify=5).
parameters  : none
returns     : none
*******************************************************************************/
void Debugger::BSCommand( void )
{
	char TextBuffer[10] = {0};
	uint16_t uwParaDetectFreq;
	uint16_t uwParaDetectAmpl;
	
	// accept command only when bode_scan is idle
	if( START_BODESCAN_MARK )
	{
	    return;
	}
	    
	BodeInit();
	
//	PutTxString("BS Command - Debugger is in Bode Mode.\n");     // just for debug
	
	//For example: Enter Char 'B''S''0''1''4''0''0''5',means do Bode Scan(freqence=140,amplify=5).
	
	//aaa(0~9999)
	uwParaDetectFreq = ( commandBuf[2] - '0' ) * 1000 + ( commandBuf[3] - '0' ) * 100 + 
	                   ( commandBuf[4] - '0' ) * 10   + ( commandBuf[5] - '0' );
	//bb(0~99)
	uwParaDetectAmpl = ( commandBuf[6] - '0' ) * 10 + ( commandBuf[7] - '0' );
    
	if( uwParaDetectAmpl > 10 ) //limit
	{
	    uwParaDetectAmpl = 0;
	    PutTxString("Amplification over limit!!!\n");
	    return;
	}
	
	if( uwParaDetectFreq > (uint16_t)( (uint32_t)uwF_SAMP * 24 >> 7 ) )//limit
	{
		uwParaDetectFreq = 0;
		PutTxString("Frequency over limit!!!\n");
		return;
	}
	
//	OS_ENTER_CRITICAL();
	uwF_INPUT = uwParaDetectFreq;
	uwA_INPUT = uwParaDetectAmpl;
	dftn = 128;
	uwCircle = 6;

	if( uwF_INPUT > ( (uint32_t)uwF_SAMP * 12 >> 7 ) )
	{
	    uwCircle = 24;
	    dftn = (uint16_t)( (uint32_t)uwF_SAMP * uwCircle / uwF_INPUT );
	}
	else if( uwF_INPUT > ( (uint32_t)uwF_SAMP * 6 >> 7 ) )
	{
	    uwCircle = 12;
	    dftn = (uint16_t)( (uint32_t)uwF_SAMP * uwCircle / uwF_INPUT );
	}
	else if( uwF_INPUT > ( (uint32_t)uwF_SAMP * 3 >> 7 ) )
	{
	    uwCircle = 6;
	    dftn = (uint16_t)( (uint32_t)uwF_SAMP * uwCircle / uwF_INPUT );
	}
    
    if( dftn > 256 )
    {
        PutTxString("dftn over limit!!!\n");
        return;
    }
        
	uwpreF_INPUT = uwF_INPUT;
	udwTIME_OFFSET = (uint32_t)uwF_SAMP * uwCircle * 2 / uwF_INPUT;   //这里取2倍点数是为了在一半时开始取数据(等待stable之后)
	fUnitTheta = 512 * (float)uwF_INPUT / (float)uwF_SAMP;
	START_BODESCAN_MARK = 1;
//	OS_EXIT_CRITICAL();
	
	SYS_sprintf( TextBuffer, "%04d", 0 );
	PutTxString( TextBuffer );
}

void Debugger::BQCommand( void )
{
    char TextBuffer[20] = {0};
    
//	PutTxString("BQ Command - Debugger is in Bode Mode.\n");     // just for debug
	
    SYS_sprintf( TextBuffer, "%04u", uwpreF_INPUT );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );

    SYS_sprintf( TextBuffer, "%04u", uwA_INPUT );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );

    SYS_sprintf( TextBuffer, "%04u", uwCircle );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );

    SYS_sprintf( TextBuffer, "%04u", dftn );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", START_BODESCAN_MARK );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", FINISH_BODESCAN_MARK );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%03d", uwcntN );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    PutTxString( "0x" );
    SYS_sprintf( TextBuffer, "%04x", (uint16_t)(udwTIMECNT >> 16) );
    PutTxString( TextBuffer );
    SYS_sprintf( TextBuffer, "%04x", (uint16_t)(udwTIMECNT & 0xFFFF) );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    PutTxString( "0x" );
    SYS_sprintf( TextBuffer, "%04x", (uint16_t)(udwTIME_OFFSET >> 16) );
    PutTxString( TextBuffer );
    SYS_sprintf( TextBuffer, "%04x", (uint16_t)(udwTIME_OFFSET & 0xFFFF) );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", BSCommandCnt );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", BTCommandCnt );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", BQCommandCnt );
    PutTxString( TextBuffer );
    PutTxChar( ' ' );
    
    SYS_sprintf( TextBuffer, "%d", InValidCommandCnt );
    PutTxString( TextBuffer );
    PutTxChar( '\n' );
}

// ********************************************************************************************************
// *
// * Function: PrintHeader(PORT_S* port);
// *
// * Purpose: To print a header on the top line of the screen.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// * Returns        :   Nothing
// *
// * Description: This function prints a header on the top line of the screen.
// *
// ********************************************************************************************************
void Debugger::PrintHeader( void )
{
    SetCursor( 0, 0 );
    PutTxString( HeaderStr );
    
    SetCursor( 2, 36 );
        // checks start block number and prints screen header for
        // which blocks screen is being displayed
    switch ( BlocksDS.startBlock )
    {
        case BLOCKS_MCU_START:
            PutTxString( McuStr );
            break;
        case BLOCKS_INVERTER_START:
            PutTxString( InverterStr );
            break;
        case BLOCKS_RECTIFIER_START:
            PutTxString( RectifierStr );
            break;
        case BLOCKS_SYSTEM_START:
            PutTxString( SystemStr );
            break;
        case BLOCKS_CHARGER_START:
            PutTxString( ChargerStr );
            break;
        case BLOCKS_METERS1_START:
            PutTxString( "METERS 1" );
            break;
        case BLOCKS_METERS2_START:
            PutTxString( "METERS 2" );
            break;            
        case BLOCKS_PARALLEL_START:
            PutTxString( "PARALLEL" );
            break;
        case BLOCKS_1_START:
            PutTxString( "Gains 1" );
            break;
        case BLOCKS_2_START:
            PutTxString( "Gains 2" );
            break;
		case BLOCKS_3_START:
			PutTxString( "Gains 3" );
			break;
		case BLOCKS_4_START:
			PutTxString( "Gains 4" );
			break;
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: PrintFooter(PORT_S* port);
// *
// * Purpose: To print a footer on the last line of the screen.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// * Returns        :   Nothing
// *
// * Description: This function prints a footer on the bottom line of the screen.
// *
// ********************************************************************************************************
void Debugger::PrintFooter( void )    
{
    SetCursor(24, 0);
    PutTxString((char*)FooterStr);
}

// ********************************************************************************************************
// * SCREEN FUNCTIONS (add new screens here)
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * Function: BlocksScreen(PORT_S* port, void* ds);
// *
// * Purpose: To redraw or update the blocks screen.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// *                    ds                : a data structure containing the necessary arguments for this function.
// * Returns        :   Nothing
// *
// * Description: This function is called by DebuggerUpdate() and redraws or updates part of the screen with each call.
// *
// ********************************************************************************************************
void Debugger::BlocksScreen( void )
{
        // Screen Parameters
    const uint16_t lineOffset    = 3;     // offset from top of screen
    const uint16_t columnOffset  = 2;     // offset from left side of screen
    const uint16_t dataOffset    = 16;    // offset of data from beginning of label
    const uint16_t labelSpacing  = 26;    // offset between beginning of labels
    const uint16_t blocksPerLine = 3;     // blocks per line

        // local variables
    uint16_t refreshLine = 1;             // the current line to redraw/refresh
    uint16_t nextBlock = 0;               // the next block to print
    uint16_t reDrawing = 0;               // whether or not we are in the process of redrawing the screen
    uint16_t block;                       // looping variable

    // if we are supposed to redraw the screen, then start from the beginning
    if ( ReDrawON )
    {
        reDrawing = 1;
        ReDrawON = false;                      // if this gets set again, then we will start over redrawing the screen
        ClrScrn();
        PrintHeader();                         // print banner at top of screen
        PrintFooter();                         // print help summary at bottom of screen
    }
    
    DisplayCurrentTime();
    
    nextBlock = BlocksDS.startBlock;      // set appropriate starting block

    while (refreshLine <= ((BlocksDS.endBlock - BlocksDS.startBlock) / blocksPerLine) + 1)
    {
        for (block = 0; block < blocksPerLine; block++)
        {
            // Redraw labels and data
            if (reDrawing)
            {
                SetCursor( refreshLine+lineOffset, (block*labelSpacing)+columnOffset );
 
                    // print block number
                SYS_sprintf( TextBuffer, "%d", nextBlock );
                if (nextBlock < 10 )
                {
                    strcat( TextBuffer, ".   ");
                }
                else
                {    
                    if (nextBlock < 100)
                    {
                        strcat( TextBuffer, ".  " );
                    }
                    else
                    {
                        strcat( TextBuffer, ". " );
                    }
                }    

                 // print label
               strcat( TextBuffer, BlocksStrings[nextBlock] );
               PutTxString( TextBuffer );
            }

            SetCursor( refreshLine+lineOffset, (block*labelSpacing)+dataOffset+columnOffset);
           
            // update data
            // skip undefined blocks
            if ( ( NONE != BlockArray[nextBlock].type ) &&
                 ( MemTypeNONE != BlockArray[nextBlock].memtype ) )
            {     
                FormatData( TextBuffer, BlockArray[nextBlock].type, BlockArray[nextBlock].memtype, BlockArray[nextBlock].data );
                PutTxString( TextBuffer );
            }    

            nextBlock++;
        }   // finished drawing/updating line
        
        EraseRestOfLine();

        refreshLine++;
    }

    reDrawing = 0;
}

// ********************************************************************************************************
// *
// * Function: DumpScreen(PORT_S* port, void* ds);
// *
// * Purpose: To redraw or update the dump screen.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// *                    ds                : a data structure containing the necessary arguments for this function.
// * Returns        :   Nothing
// *
// * Description: This function is called by DebuggerUpdate() and redraws or updates part of the screen with each call.
// *
// ********************************************************************************************************
void Debugger::DumpScreen( void )
{
    // screen variables
    const uint16_t lineOffset     = 3;     // distance between top of screen and first line
    const uint16_t maxlength      = 100;
    
    // local variables
    uint16_t* currentAddress;   // the current address to print
    uint16_t  ee_address;

    if ( ReDrawON )
    {
        ClrScrn();
        SetCursor(lineOffset, 0);
        // limit number of lines
        if ( DumpDS.length > maxlength )
        {
            DumpDS.length = maxlength;
        }    
    
        currentAddress = (uint16_t*)DumpDS.startAddress;
        ee_address     = *(uint16_t*)&DumpDS.startAddress;

        for ( uint16_t idx = 0; idx < DumpDS.length; idx++ )
        {

                // print data
            if ( MemTypeEEPROM == DumpDS.memoryType )
            {
                uint16_t ee_data = 0;
                uint16_t address = (uint32_t)currentAddress;
                            // print data number
                SYS_sprintf( TextBuffer, "Addr: %d", address );
                strcat( TextBuffer, ":  " );
                PutTxString( TextBuffer );
                
                (void)GetEepData( ee_address, 1, &ee_data, TSK_1000_ms );
                ++ee_address;
                FormatData( TextBuffer, DumpDS.format, MemTypeUINT, &ee_data );
            }
            else
            {
                            // print data number
                SYS_sprintf( TextBuffer, "Addr: 0x%04x%04x", (int32_t)currentAddress );
                strcat( TextBuffer, ":  " );
                PutTxString( TextBuffer );
                FormatData( TextBuffer, DumpDS.format, DumpDS.memoryType, currentAddress ); 
            }
            
            PutTxString( TextBuffer );
            PutTxChar( CR );
            PutTxChar( LF );
            if ( ( MemTypeUINT == DumpDS.memoryType ) || ( MemTypeEEPROM == DumpDS.memoryType ) )
            {
                currentAddress++;
            }
            else
            {
                currentAddress += 2;
            }        
        }
    
        PutTxString( "\r\f" );
        PutTxString( "\r\f" );
        PutTxString( "\r\f" );
    }
    
    ReDrawON = false;    
}

// ********************************************************************************************************
// *
// * Function: DumpSamples(PORT_S* port, void* ds);
// *
// * Purpose: To redraw or update the dump screen.
// *
// * Parms Passed   :   port              : pointer to the currectly active debugger port.
// *                    ds                : a data structure containing the necessary arguments for this function.
// * Returns        :   Nothing
// *
// * Description: This function is called by DebuggerUpdate() and redraws or updates part of the screen with each call.
// *
// ********************************************************************************************************
void Debugger::DumpSamples( void )
{
    // screen variables
    const uint16_t lineOffset    = 3;     // distance between top of screen and first line
 
    // local variables
    stCapturePointer currentAddress;    
         
    uint16_t thisLength;;                 // the last address to be printed

    if ( ReDrawON )
    {
        ClrScrn();
        currentAddress.ptrInt = (int16_t*)&CaptureData[0].s[0];
        thisLength = ( CAPTURE_SIZE * 2 );

        // move cursor
        SetCursor(lineOffset, 0);
             
        while ( thisLength )
        {
            // print data
            FormatData( TextBuffer, DumpDS.format, DumpDS.memoryType, currentAddress.ptrInt );   
            PutTxString( TextBuffer );
            PutTxChar( CR );
            PutTxChar( LF );

            if ( CaptureControl.LongData )
            {     
                ++currentAddress.ptrLong;
                thisLength -= 2;
            }
            else
            {
                ++currentAddress.ptrInt;
                --thisLength;
            }        
        }
         
        PutTxString( "\r\f" );
        PutTxString( "\r\f" );
        PutTxString( "\r\f" );
    }
    
    ReDrawON = false;         
     
}

// ***********************************************************************
// *
// *    FUNCTION: FormatData 
// *
// *    DESCRIPTION: formats numeric data to ASCII
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void Debugger::FormatData( char* buffer, eBlockType type, eMemType memType, void* data )
{
    const uint16_t MAXFIELDLENGTH = 7;     // max number of ascii characters in a data field to be displayed
   
    float fval = 0;
    int32_t ival = 0;
    
    switch ( memType )
    {
        case MemTypeINT:
            ival = *(int16_t*)data;
            break;
            
        case MemTypeUINT:
            ival = *(uint16_t*)data;
            break;    
            
        case MemTypeLONG:
            ival = *(int32_t*)data;
            break;
            
        case MemTypeULONG:
            ival = *(uint32_t*)data;
            break;
            
        case MemTypeFLOAT:
            fval = *(float*)data * (float)floatDataGain;
            break;
            
        case FuncTypeINT:
            if ( ((uint32_t)data >= (uint32_t)MAINPROG_ENTRY) && 
                 ((uint32_t)data < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
            {
                ival = ((FUNC_TYPE_INT_PTR)((uint32_t)data))();
            }
            break;
            
        case FuncTypeUINT:
            if ( ((uint32_t)data >= (uint32_t)MAINPROG_ENTRY) && 
                 ((uint32_t)data < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
            {
                ival = ((FUNC_TYPE_UINT_PTR)((uint32_t)data))();
            }
            break;

        case FuncTypeLONG:
            if ( ((uint32_t)data >= (uint32_t)MAINPROG_ENTRY) && 
                 ((uint32_t)data < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
            {
                ival = ((FUNC_TYPE_LONG_PTR)((uint32_t)data))();
            }
            break;

        case FuncTypeULONG:
            if ( ((uint32_t)data >= (uint32_t)MAINPROG_ENTRY) && 
                 ((uint32_t)data < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
            {
                ival = ((FUNC_TYPE_ULONG_PTR)((uint32_t)data))();
            }
            break;

        case FuncTypeFLOAT:
            if ( ((uint32_t)data >= (uint32_t)MAINPROG_ENTRY) && 
                 ((uint32_t)data < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
            {
                fval = ((FUNC_TYPE_FLOAT_PTR)((uint32_t)data))();
            }
            break;

        default:
            ival = 0;
            break;    
    }
    
    char* charPtr;
    
    if ( MemTypeFLOAT  != memType &&
         FuncTypeFLOAT != memType)
    {
        fval = ival;
    }    
     
    switch ( type )
    {
        // note: SYS_printf always return 4 decimal places, ref spru625k
        case DEC0:
            SYS_sprintf( buffer, "%f", fval );
            // replace '.' with NULL terminator
            charPtr = strchr( buffer, '.' );
            if ( (char*)NULL != charPtr )
            {
                *charPtr = '\0';
            }    
            break;
            
        case DEC1:
            SYS_sprintf( buffer, "%f", fval );
            // terminate after '.0'
            charPtr = strchr( buffer, '.' );
            if ( (char*)NULL != charPtr )
            {
                charPtr += 2;
                *charPtr = '\0';
            }    
            break;
            
        case DEC2:
            SYS_sprintf( buffer, "%f", fval );
            // terminate after '.0'
            charPtr = strchr( buffer, '.' );
            if ( (char*)NULL != charPtr )
            {
                charPtr += 3;
                *charPtr = '\0';
            }    
            break;
            
        case DEC3:
            SYS_sprintf( buffer, "%f", fval );
            // terminate after '.0'
            charPtr = strchr( buffer, '.' );
            if ( (char*)NULL != charPtr )
            {
                charPtr += 4;
                *charPtr = '\0';
            }    
            break;
            
        case DEC4:
            SYS_sprintf( buffer, "%f", fval );
            break;                
            
        case HEX:
            SYS_sprintf( buffer, "0x%04x", (int16_t)ival );
            break; 
            
        default:
            strcpy( buffer, "" );
            break;        
    }
    
    // check length
    if ( strlen( buffer ) > MAXFIELDLENGTH )
    {
        buffer[ MAXFIELDLENGTH ] = '\0';
    }
    else
    {
        while ( strlen( buffer ) < MAXFIELDLENGTH )
        {
            strcat( buffer, " " );  // append a space
        }    
    }    
}

void Debugger::DisplayCurrentTime(void)
{
    char FormattedTime[24]; //2010-05-12 [3] 16:24:30
    SysTimeToBcdTime(&RTC_SysTime, &RTC_BcdTime);
    
    FormattedTime[0] = '2';
    FormattedTime[1] = '0';
    FormattedTime[2] = (RTC_BcdTime.s.year_bcd >> 4) + '0';
    FormattedTime[3] = (RTC_BcdTime.s.year_bcd & 0x0F) + '0';
    FormattedTime[4] = '-';
    FormattedTime[5] = (RTC_BcdTime.s.month_bcd >> 4) + '0';
    FormattedTime[6] = (RTC_BcdTime.s.month_bcd & 0x0F) + '0';
    FormattedTime[7] = '-';
    FormattedTime[8] = (RTC_BcdTime.s.date_bcd >> 4) + '0';
    FormattedTime[9] = (RTC_BcdTime.s.date_bcd & 0x0F) + '0';
    FormattedTime[10] = ' ';
    FormattedTime[11] = '[';
    FormattedTime[12] = (RTC_BcdTime.s.day_bcd & 0x0F) + '0';
    FormattedTime[13] = ']';
    FormattedTime[14] = ' ';
    FormattedTime[15] = (RTC_BcdTime.s.hours_bcd >> 4) + '0';
    FormattedTime[16] = (RTC_BcdTime.s.hours_bcd & 0x0F) + '0';
    FormattedTime[17] = ':';
    FormattedTime[18] = (RTC_BcdTime.s.min_bcd >> 4) + '0';
    FormattedTime[19] = (RTC_BcdTime.s.min_bcd & 0x0F) + '0';
    FormattedTime[20] = ':';
    FormattedTime[21] = (RTC_BcdTime.s.secs_bcd >> 4) + '0';
    FormattedTime[22] = (RTC_BcdTime.s.secs_bcd & 0x0F) + '0';
    FormattedTime[23] = '\0';
    
    SetCursor( 0, 40 );
    
    PutTxString("  Current Time : ");
    
    PutTxString( FormattedTime );
}

// ***********************************************************************
// *
// *    FUNCTION: ActiveEventsScreen  
// *
// *    DESCRIPTION: displays active events/notices
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
const char* GetEventText( uint16_t strNum );
void Debugger::ActiveEventsScreen( void )
{
    uint16_t numEvents = NB_GetNumAlarms() + NB_GetNumNotices(); 
    
    if ( lastNumberOfEvents != numEvents )
    {
        lastNumberOfEvents = numEvents;
        
        ClrScrn();
    
        SetCursor( 3, 0 );
    
        SYS_sprintf( TextBuffer, "%u %s\r\f", numEvents, "active events" );
        PutTxString( TextBuffer );
    
        uint16_t NB_idx = 0;
    
        while ( numEvents && ( NB_idx < GetNumDefinedNB() )  )
        {
            if ( NB_GetNodebit( NB_idx ) )
            {
                const st_NB_Cfg* nb = &NB_Cfg_Flash[ NB_idx ];
                
                // ignore status
                if ( nb->bit.XCPAlarmLevel >= EVENT_LEVEL_NOTICE )
                {
                    const char* level = NULL;
            
                    if ( nb->bit.XCPAlarmLevel < EVENT_LEVEL_ALARM )
                    {
                        level = (const char*)noticeStr;    
                    }
                    else
                    {
                        level = (const char*)alarmStr;
                    }
            
                    // alarm/notice/status + number:
                    SYS_sprintf( TextBuffer, "%s%u\t", level, nb->bit.XCPAlarmNumber );
                    PutTxString( TextBuffer );
                    PutTxString( (char*)GetEventText( nb->bit.XCPAlarmNumber ) );
            
                    PutTxString( "\r\f" );
                    --numEvents;
                }    
            }
            
            ++NB_idx;
        }
    }    
}

void Debugger::LoggerScreen(void)
{

}

// ***********************************************************************
// *
// *    FUNCTION: ActiveEventsScreen
// *
// *    DESCRIPTION: displays active events/notices
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void Debugger::UpdateControllerCoefficents(uint16_t type, uint16_t data, const stSecondOrderIIRFP * init_gains )
{
    switch(type)
    {
        case 0:
            DebugCoefficients.B0 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 1:
            DebugCoefficients.B1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 2:
            DebugCoefficients.B2 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 3:
            DebugCoefficients.A1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 4:
            DebugCoefficients.A2 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 10:
            DebugCoefficients.B0 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 11:
            DebugCoefficients.B1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 12:
            DebugCoefficients.B2 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 13:
            DebugCoefficients.A1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 14:
            DebugCoefficients.A2 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 20:
            DebugCoefficients.X1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

		case 21:
			DebugCoefficients.X2 = float(data)/pow(float(10.0),float(CoefficientsScale));
			break;
			
		case 30:
			DebugCoefficients.X1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
			break;
		
		case 31:
			DebugCoefficients.X2 = -float(data)/pow(float(10.0),float(CoefficientsScale));
			break;

        case 99:
            DebugCoefficients = *init_gains;
            break;

        default:
            break;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: UpdateControllerCoefficentsFirst
// *
// *    DESCRIPTION: update first order control param by ct cmd
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void Debugger::UpdateControllerCoefficentsFirst(uint16_t type, uint16_t data, const stFirstOrderIIRFP * init_gains )
{
    switch(type)
    {
        case 0:
            DebugCoefficientsFirst.B0 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 1:
            DebugCoefficientsFirst.B1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 2:
            DebugCoefficientsFirst.A1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 10:
            DebugCoefficientsFirst.B0 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 11:
            DebugCoefficientsFirst.B1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 12:
            DebugCoefficientsFirst.A1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
            break;

        case 20:
            DebugCoefficients.X1 = float(data)/pow(float(10.0),float(CoefficientsScale));
            break;
		
		case 30:
			DebugCoefficients.X1 = -float(data)/pow(float(10.0),float(CoefficientsScale));
			break;

        case 99:
            DebugCoefficientsFirst = *init_gains;
            break;

        default:
            break;
    }
}

// ********************************************************************************************************
// *            END OF Debugger.c
// ********************************************************************************************************


