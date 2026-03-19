#ifndef _DEBUGGER_H
#define _DEBUGGER_H
// ********************************************************************************************************
// *            Debugger.h
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
// *    FILE NAME: Debugger.h
// *
// *    DESCRIPTION: Engine for the debugger: Debugger API. See comments for using these functions in an
// *                    application.
// *
// *    ORIGINATOR: Jonathan Rodriguez
// *
// *    DATE: 12/11/2002
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

#include "SerialPort.h"
#include "DebuggerBlocks.h"
#include "algos.h"

const uint16_t TextBufferSize = 64;
const uint16_t CommandBufferSize = 32;

typedef struct 
{
    void*       startAddress;       // address of first data to print
    uint16_t    length;             // number of data elements to print
    eBlockType  format;             // the format to print the data in
    eMemType    memoryType;         // what type of memory
} stDumpDataStruct;

typedef struct 
{
    uint16_t startBlock;              // number of the first block to print
    uint16_t endBlock;                // number of the last block to print
} stBlocksDataStruct;

typedef enum
{
    eScreenNone,
    eBlockScreen,
    eDumpScreen,
    eDumpSamplesScreen,
    eEventScreen,
    eLogScreen,
    eKevinsScreen
} eScreenFunc;

typedef enum
{
	eAdcIsr,
	eReverse1,
	eReverse2,
	eReverse3,
	eReverse4,
	eMax	//max is 5
}eRunTime;

class Debugger : public SerialPort
{
    public:
        // constructor
        Debugger( PortHandle port ) : SerialPort( port )
        {
            ReDrawON = false;                                         
            ScreenFunction = eScreenNone;
            cmdBufIndex = 0;
            updateCount = 0;
            TextBuffer = NULL;
            commandBuf = NULL;  
            CoefficientsScale = 4;
            floatDataGain = 1;
        }       
        // destructor
        ~Debugger()
        {
        }
    public:
        void DebuggerStart( void );
        void DebuggerStartBode( void );
        
    protected:
        void DebuggerMain( void );
        void DebuggerBode( void );
        void DebuggerUpdate( void );
        void CheckCommandForBackSpace( void ); 
        int32_t GetCommandArgument( int16_t argumentNum );
        void ParseDebugCommand( void );
        void ParseBodeCommand( void );
        void SetScreenFunction( eScreenFunc screen );
        void Gm( void );          // GET MCU BLOCKS
        void Gi( void );          // GET INVERTER BLOCKS
        void Gr( void );          // GET RECTIFIER BLOCKS
        void Gs( void );          // GET SYSTEM BLOCKS
        void Gc( void );          // GET CHARGER BLOCKS
        void Gt( void );          // Get meters block 1
        void Ge( void );          // Get meters block 2      
        void Gp( void );          // Get parallel blocks
        void G1( void );          // Get blocks 1
        void G2( void );          // Get blocks 2
        void G3( void );          // Get blocks 3
        void G4( void );          // Get blocks 4
        void Gn( void );          // GET nodebit state
        void Wb( void );          // WRITE BLOCK
        void We( void );          // WRITE EEP
        void Wn( void );          // WRITE nodebit with test mode
        void Ws( void );          // WRITE nodebit without test mode
        void Re( void );          // READ EEP
        void Dm( void );          // DUMP data at addresses...
        void De( void );          // DUMP eeps at addresses...
        void Dd( void );          // List eeprom data that differs from default value.
        void Cc( void );          // Control Charger
        void Cb( void );          // Control Boost
        void Ci( void );          // Control Inverter
        void Cr( void );          // Control Rectifier
        void Cm( void );          // Control MCU
        void Ct( void );          // Control Test
        void S ( void );          // Sample memory location at specified frequency
        void T ( void );          // Sample triggered data
        void Rs( void );          // Reset EEPROM
        void Rn( void );          // Reset nodebits and exit NB test mode
        void U(  void );          // UPS commands   
        void P ( void );          // Paramter commands
        void K ( void );          //       
        void BTCommand( void );   // Bode Command
        void BSCommand( void );   // Bode Command
        void BQCommand( void );   // Bode Command
        bool Abort(void);		// abort waiting for events when escape key is pressed
        
        void PrintHeader( void );
        void PrintFooter( void );
        void BlocksScreen( void );
        void DumpScreen( void );
        void DumpSamples( void );
        void KevinsSamples( void );
        void FormatData( char* buffer, eBlockType type, eMemType memType, void* data );
        void DisplayCurrentTime(void);
        void ActiveEventsScreen( void );
        void LoggerScreen(void);
        void UpdateControllerCoefficents(uint16_t type,uint16_t data, const stSecondOrderIIRFP * init_gains );
		void Debugger::UpdateControllerCoefficentsFirst(uint16_t type, uint16_t data, const stFirstOrderIIRFP * init_gains );
        
    protected:
        // protected data
        bool                ReDrawON;                                 // set when screen is to be redrawn
        uint16_t            cmdBufIndex;                              // the index to which the next new character will be placed in the commandBuf
        uint16_t            updateCount;                              // sets update rate of blocks screens
        uint16_t            lastNumberOfEvents;
        uint16_t            cmdTimeout;
        uint16_t            floatDataGain;
        uint32_t            debuggerTimeoutCount;
        eScreenFunc         ScreenFunction;
        stDumpDataStruct    DumpDS;
        stBlocksDataStruct  BlocksDS;
        char*               TextBuffer;
        char*               commandBuf;
        uint16_t            CoefficientsScale;
        stSecondOrderIIRFP  DebugCoefficients;
        stFirstOrderIIRFP   DebugCoefficientsFirst;
};

// ********************************************************************************************************
// *            END OF Debugger.h
// ********************************************************************************************************
#endif
