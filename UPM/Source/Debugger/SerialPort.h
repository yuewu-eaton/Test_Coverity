// ******************************************************************************************************
// *            SerialPort.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: SerialPort.h
// *
// *    DESCRIPTION: Header for SerialPort.cpp
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 5/8/2008
// *
// *    HISTORY: See Wildcat SubVersion history.
// ******************************************************************************************************
#ifndef _SERIALPORT_H
#define _SERIALPORT_H

// ******************************************************************************************************
// *            Constants
// ******************************************************************************************************

// bauds we can support
// no need to eep, is debugger only
const uint32_t BaudRateDefault = 57600;

// ******************************************************************************************************
//  Modified from Wildcat/Kiger, removed queues. BIOS queues don't seem to be as useful as
//  FreeRTOS queues, and since this is just for debugger and history log, just use the hardware
//  FIFOs
// ******************************************************************************************************


// ******************************************************************************************************
// *            Definitions
// ******************************************************************************************************

typedef volatile SCI_REGS* PortHandle;

// Serial port class definition
class SerialPort
{
    public:
        SerialPort( PortHandle port );
        virtual ~SerialPort()
        {
        }

    public:     // keep public, can be useful for debugging
        void PutTxChar( char data );
    
    protected:
        void SetPortBaudRate( uint32_t baud );
        void SetPortDataBits( uint16_t databits );
        void SetPortStopBits( uint16_t stopbits );
        void PutTxString( const char* buffer );
        void PutTxBuf( const char* Buffer, const uint16_t Size );
        void PurgeRxChars( uint16_t numChars );
        uint16_t GetChars( char* data, uint16_t num );
        // these are for terminal mode, put here because they are common to
        // both debugger and log 
        void ClrScrn( void );
        void SetCursor( int line, int column );
        void EraseRestOfLine( void );
                
    public:
        uint16_t SizeRxBuf( void )
        {
            return Port->SCIFFRX.bit.RXFFST;
        }
        char GetRxChar( void )
        {
            return Port->SCIRXBUF.bit.RXDT;
        }

    protected:
        // private data
        PortHandle  Port;

};

// *********************************************************************************************************
// *        Useful Character Defines
// *********************************************************************************************************
#define ESC             27      // escape
#define CR              13      // carriage return
#define LF              10      // line feed
#define BS              8       // backspace
#define XCP_SFD         0xAB    // XCP start of frame

#define SPACES_FOR_ZEROS    0x8000


// ********************************************************************************************************
// *            END OF Port.h
// ********************************************************************************************************
#endif
