// ******************************************************************************************************
// *            SerialPort.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton  
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: SerialPort.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 5/8/2008
// *
// *    HISTORY: See Wildcat SubVersion history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "SerialPort.h"
#include "Constants.h"

// ***********************************************************************
// *
// *    FUNCTION: SerialPort  
// *
// *    DESCRIPTION: Constructor: Initializes port hardware to known state
// *
// *    ARGUMENTS: PortHandle
// *
// *    RETURNS: 
// *
// ***********************************************************************
SerialPort::SerialPort( PortHandle port )
{
    Port = port;
    
    // FIFO init
    Port->SCIFFTX.bit.SCIFFENA = 1;         // enable Tx FIFO
    Port->SCIFFTX.bit.TXFIFOXRESET = 0;     // reset FIFO
    Port->SCIFFRX.bit.RXFIFORESET = 0;      // Reset Rx FIFO
    Port->SCIFFCT.all = 0x0;                // none
    
    
    Port->SCICCR.all = 0x0007;              // 1 stop bit,  No loopback
                                            // No parity,8 char bits,
                                            // async mode, idle-line protocol
    Port->SCICTL1.all = 0x0003;             // enable TX, RX, internal SCICLK,
                                            // Disable RX ERR, SLEEP, TXWAKE
    Port->SCICTL2.all = 0;                  // no interrupts                             
    
    SetPortBaudRate( BaudRateDefault );     // set BRregs
    
    Port->SCIFFTX.bit.TXFFINTCLR = 1;       // clear any FIFO interrupts
    Port->SCIFFTX.bit.TXFIFOXRESET = 1;     // release Tx FIFOR from reset
    Port->SCIFFRX.bit.RXFFINTCLR = 1;       // clear any interrupts
    Port->SCIFFRX.bit.RXFIFORESET = 1;      // release Rx FIFO from reset
    Port->SCIFFTX.bit.SCIRST = 1;           // enable Tx/Rx
    
    Port->SCICTL1.all =0x0023;              // Relinquish SCI from Reset
    
    Port->SCIFFTX.bit.SCIRST = 1;           // enable Tx/Rx
}

// ***********************************************************************
// *
// *    FUNCTION: SetPortDataBits 
// *
// *    DESCRIPTION: Sets databits, no limits checking, debug only
// *
// *    ARGUMENTS: wanted databits
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SerialPort::SetPortDataBits( uint16_t databits )
{
    Port->SCICCR.bit.SCICHAR = databits - 1;
}

// ***********************************************************************
// *
// *    FUNCTION: SetPortStopBits 
// *
// *    DESCRIPTION: Sets port stopbits, no limits checking, debug only
// *
// *    ARGUMENTS: wanted stopbits
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SerialPort::SetPortStopBits( uint16_t stopbits )
{
    Port->SCICCR.bit.STOPBITS = stopbits - 1;
}

// ***********************************************************************
// *
// *    FUNCTION: SetBaudRate 
// *
// *    DESCRIPTION: Sets port baud rate, no limits checking, debug only
// *
// *    ARGUMENTS: wanted baud
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SerialPort::SetPortBaudRate( uint32_t baud )
{
    uint32_t BRR;
    uint32_t div;
    
    //********************************************************
    //* set registers:
    //********************************************************

    div = (SysCtrlRegs.LOSPCP.all & 0x0007) * 2;
    if (div == 0)
    {
        div = 1;
    }
    
    BRR = ((CPUFrequency/div)/(baud*8)) - 1;
    // Set Baud Rate
    // Note that it is dependent on LSPCLK and SCIAENCLK
    // At present time, LSPCLK = 0x02, which means SYSCLKOUT/4 = 150MHz/4 = 37.5MHz
    // BRR = [LSPCLK/(BaudRate*8)] - 1 = 37500000/57600*8 - 1 = 80  (0x50)  for 57.6 KBaud
    // BRR = [LSPCLK/(BaudRate*8)] - 1 = 37500000/19200*8 - 1 = 243 (0xF3)  for 19.2 KBaud
    // BRR = [LSPCLK/(BaudRate*8)] - 1 = 37500000/9600*8  - 1 = 487 (0x1E7) for 9.6 KBaud
    Port->SCIHBAUD = (BRR >> 8) & 0x00FF;
    Port->SCILBAUD = BRR & 0x00FF;
} 

// ***********************************************************************
// *
// *    FUNCTION: PutTxChar 
// *
// *    DESCRIPTION: writes character to TXFIFO
// *
// *    ARGUMENTS: char to write
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void SerialPort::PutTxChar( char data )
{
    while ( Port->SCIFFTX.bit.TXFFST == 16 ) 
    {
        // 57600 bps ~= 57600/(1 start bit + 8 data bits + 1 stop bit ) = 174us/byte * 16 = ~2.7ms to clear buffer
        TSK_sleep( TSK_5_ms );
    }
    
    Port->SCITXBUF = data;    
}

// ********************************************************************************************************
// *
// * Function: PutTxString( PORT_S *Port, const INT8S* String, INT8S PackedString )
// *
// * Purpose: to copy contents of string in TxFIFO (up to a max of 64 chars)
// *                String must be Null_terminated, but can be either packed or unpacked.
// *
// * Parms Passed   :   Port        : Pointer to Port to receive this string
// *                :   String      : Pointer to String to be Put.
// *                :   PackedString: If PackedString, it puts both High Nibble and Low Nibbles to TxFIFO
// *                                    Otherwise, puts only the Low Nibble to TxFIFO, until NULL encountered.
// *
// * Returns        :   Returns # of characters it put in the TxFIFO
// *
// ********************************************************************************************************
void SerialPort::PutTxString( const char* buffer )
{
    uint16_t sanitycheck = 64;
    
    while ( ( *buffer != '\0' ) && sanitycheck )
    {
        PutTxChar( *buffer );
        ++buffer;
        --sanitycheck;
    }    
}

        
// ********************************************************************************************************
// *
// * Function: PutTxBuf(PORT_S *Port, INT8S *Buffer, uint16_t Size)
// *
// * Purpose:       :   Adds contents of buffer to TxFIFO, no ASCII conversion performed.
// *
// * Parms Passed   :   Port        : Pointer to Port to receive copy of buffer
// *                :   Buffer      : Pointer to Buffer to be Put.
// *                :   Size        : Number of chars to copy from Buffer to TxFIFO
// *
// * Returns        :   Returns # of characters it put in the TxFIFO
// *
// ********************************************************************************************************
void SerialPort::PutTxBuf( const char* Buffer, uint16_t Size )
{
    while ( Size )
    {
        PutTxChar( *Buffer );
        ++Buffer;
        --Size;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: GetChars 
// *
// *    DESCRIPTION: reads chars from RXFIFO and writes to buffer
// *
// *    ARGUMENTS: number of chars to read, pointer to data
// *
// *    RETURNS: number of chars read
// *
// ***********************************************************************
uint16_t SerialPort::GetChars( char* data, uint16_t num )
{
    uint16_t chars = 0;
    
    while ( ( num > 0 ) && ( Port->SCIFFRX.bit.RXFFST > 0 ) )
    {
        *data = (char)Port->SCIRXBUF.bit.RXDT;
        ++data;
        ++chars;
        --num;
    }
    
    return chars;
} 

// ***********************************************************************
// *
// *    FUNCTION: PurgeRxChars 
// *
// *    DESCRIPTION: clears RxFIFO
// *
// *    ARGUMENTS: number of chars to clear
// *
// *    RETURNS:
// *
// ***********************************************************************
void SerialPort::PurgeRxChars( uint16_t numChars )
{
    while ( numChars && ( Port->SCIFFRX.bit.RXFFST > 0 ) )
    {
        char dummy = (char)Port->SCIRXBUF.bit.RXDT;
        --numChars;    
    }
}

// ****************** The following are specific to terminal mode, put here because they are common to 
// ****************** both the debugger and log terminal

// ********************************************************************************************************
// *
// * Function: ClrScrn(PORT_S *Port);
// *
// * Purpose: To send out the clear screen command to HyperTerminal.
// *
// * Parms Passed   :   Port    : Pointer to Port to which the clear command will be sent.
// * Returns        :   Nothing
// *
// * Description: This function uses PORT_S functions to send the clear screen command.
// *
// ********************************************************************************************************
void SerialPort::ClrScrn( void )
{
    PutTxChar( ESC );
    PutTxChar( '[' );
    PutTxChar( '2' );
    PutTxChar( 'J' );
}

// ********************************************************************************************************
// *
// * Function: SetCursor(int16_t line, int16_t column, PORT_S *Port);
// *
// * Purpose: To place the cursor at a certain location on HyperTerminal.
// *
// * Parms Passed   :   line    : The line number for the cursor (starts at line 1)
// *                :   column  : The column number for the cursor (starts at column 1)
// *                :   Port    : Pointer to Port to which the move cursor command will be sent.
// * Returns        :   Nothing
// *
// * Description: This function uses PORT_S functions to send the move cursor command.
// *
// ********************************************************************************************************
void SerialPort::SetCursor( int16_t line, int16_t column )
{
    char buffer[10] = {0};
    
    // VT100 command to move cursor:
    // ESC [ <line#> ; <column#> H
    PutTxChar( ESC );
    PutTxChar( '[' );
    SYS_sprintf( buffer, "%d", line );
    PutTxString( buffer );
    PutTxChar( ';' );
    SYS_sprintf( buffer, "%d", column );
    PutTxString( buffer );
    PutTxChar( 'H' );
}

// ********************************************************************************************************
// *
// * Function: EraseRestOfLine(PORT_S *Port);
// *
// * Purpose: To send out the 'erase rest of line' command to HyperTerminal.
// *
// * Parms Passed   :   Port    : Pointer to Port to which the command will be sent.
// * Returns        :   Nothing
// *
// * Description: This function uses PORT_S functions to send the 'erase rest of line' command.
// *
// ********************************************************************************************************
void SerialPort::EraseRestOfLine( void )
{
    PutTxChar( ESC );
    PutTxChar( '[' );
    PutTxChar( 'K' );
}
       

// ********************************************************************************************************
// *            END OF Port.c
// ********************************************************************************************************
