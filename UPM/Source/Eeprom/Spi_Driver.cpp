// ******************************************************************************************************
// *            Spi_Driver.c
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
// *    FILE NAME: Spi_Driver.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *                NOTE: This file must be Optimized for Speed
// *                JFW:  some attempts at optimizing...
// *
// *    DATE: 4/15/2003
// *
// *    HISTORY: See Visual Source Safe history.
// ******************************************************************************************************


// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "IOexpansion.h"
#include "Spi_Driver.h"
#include "Eeprom_Map.h"
#include "Constants.h"
#include "NB_Funcs.h"

// ********************************************************************************************************
// * Global DECLARATIONS
// ********************************************************************************************************
extern volatile uExpansionReg ExpansionInputReg;

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************
extern "C"
{
    void InitSPI(void);
}

uint16_t write_eeprom_sequence( uint16_t start_address, uint16_t num_words, const uint16_t* data_ptr );
uint16_t sendSPIdata( uint16_t txdata );

void reset_spi_fifos( void );
void reset_and_conf_spi(bool transfer_16bits=false,bool polarity_pos=false,uint16_t phase=0);
void enable_SPI_CS( uint16_t device );
void disable_SPI_CS( void );
void WaitEEPROMBusy( void );

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
const uint16_t eeprom_clear_data[]={0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                                  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

const uint16_t eeprom_erase_data[]={0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff,
                                  0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff};


// ****************************************************************************
// *
// *  Function: InitSpi
// *
// *  Purpose :    To initialize/ Open the SPI device
// *
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description:    This module initializaes the SPI registers and SPI GPIO
// *
// ****************************************************************************
void InitSPI(void)
{
    uint32_t BRR;
    uint32_t div;
    
    // initialize GPI to SPIA
    InitSpiaGpio();

    // Initialize SPI FIFO registers
    // SPIRST = 1;
    // SPIFFENA = 1;
    // TXFIFO Reset = 1;
    // TXFFST4-0 = 00000;
    // TXFINT = 0;
    // TXFINT CLR = 1;
    // TXFFIENA = 0;
    // TXFFIL4-0 = 00000;
    SpiaRegs.SPIFFTX.all = 0xE040;
    // RXFFOVF = 0;
    // RXFFOVF CLR = 1;
    // RXFIFO Reset = 1;
    // RXFFST4-0 = 00000;
    // RXFINT = 0;
    // RXFINT CLR = 1;
    // RXFFIENA = 0;
    // RXFFIL4-0 = 00000;
    SpiaRegs.SPIFFRX.all = 0x6040;
    // FFTXDLY7-0 = 0;
    SpiaRegs.SPIFFCT.all = 0x0;

    SpiaRegs.SPISTS.all = 0x00A0;                //BUFFULL_FLAG = 1, OVERRUN_FLAG = 1   
    SpiaRegs.SPICCR.all = 0x0007;                // Reset on, rising edge, 8-bit char bits  
    SpiaRegs.SPICTL.all = 0x0006;                // Enable master mode, normal phase,
                                                 // enable talk, and SPI int disabled.

    SpiaRegs.SPICTL.bit.CLK_PHASE     = 1;       // Zhangjun : Configure for 25LC640 and should be 
                                                 // re-Configured for DS1305
        
    //********************************************************
    // set baud rate, 1Mbps
    //********************************************************

    div = ( SysCtrlRegs.LOSPCP.all & 0x0007 ) * 2;
    if ( div == 0 )
    {
        div = 1;
    }
    
    BRR = ( ( CPUFrequency / div ) / ( 1000000 ) ) - 1;
    
    SpiaRegs.SPIBRR = BRR;                                    
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;          // Relinquish SPI from Reset   
    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission

    // setup chip selects
    EALLOW;

    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO58 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;

    EDIS;

    // disable both SPI devices
    disable_SPI_CS();
}

// ***********************************************************************
// *
// *    FUNCTION: enable_SPI_CS  
// *
// *    DESCRIPTION: drives requested line low, delay for 750ns( min )
// *
// *    ARGUMENTS: SPI device to enable 
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void enable_SPI_CS( uint16_t device )
{
    // 750 ns: TI's formula = ((( 750 / 6.667 ) - 9) / 5 ) = 20.8;
    // since interrupts are not disabled and doesn't execute from 0 wait state memonry is
    // actually longer than that.
    uint32_t cs_delay = 0;
    // for control board with encrypt
    //
    //       DSP Output pins                   SPI device (output of PLD)
    //     GPIO 57                GPIO 60
    //PIN_DSP_SPI_CS1         PIN_DSP_SPI_CS2
    //      0                       0                    Null
    //      1                       0                    EEPROM
    //      0                       1                    IO expander
    //      1                       1                    SPI_PLD
    switch (device)
    {
        case SPI_EEPROM:
            DSPOutRegister.GpoB.bit.SpiCs1Dsp = 1;
            DSPOutRegister.GpoB.bit.SpiCs2Dsp = 0;
            cs_delay = 22UL;
            break;
        case SPI_IOX:
            DSPOutRegister.GpoB.bit.SpiCs1Dsp = 0;
            DSPOutRegister.GpoB.bit.SpiCs2Dsp = 1;
            cs_delay = 1UL;
            break;
        case SPI_PLD:
            DSPOutRegister.GpoB.bit.SpiCs1Dsp = 1;
            DSPOutRegister.GpoB.bit.SpiCs2Dsp = 1;
            cs_delay = 1UL;
            break;
        default:
            break;
    }
    WriteDSPOutputs_TSK();
    DSP28x_usDelay (cs_delay);
}


// ***********************************************************************
// *
// *    FUNCTION: disable_SPI_CS  
// *
// *    DESCRIPTION: drives both SPI CE enable signals high
// *
// *    ARGUMENTS: none 
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void disable_SPI_CS( void )
{
    DSPOutRegister.GpoB.bit.SpiCs1Dsp = 0;
    DSPOutRegister.GpoB.bit.SpiCs2Dsp = 0;
    WriteDSPOutputs_TSK();
}

// ****************************************************************************
// *
// *  Function: sendSPIbyte
// *
// *  Purpose : writes 1 byte to SPI tx buffer, reads the byte that's clocked
//              into the rx buffer
// *
// *  Parms Passed   :   data to send
// *
// *  Returns        :   data receieved
// *
// ****************************************************************************
uint16_t sendSPIdata( uint16_t txdata )
{
    uint16_t rxdata;
    uint16_t timeout = 5000;

    // transmit the passed data - receives simultaneously...
    SpiaRegs.SPITXBUF = txdata;

    // wait for data received
    while ( 0 == SpiaRegs.SPIFFRX.bit.RXFFST )
    { 
        // this is mostly cosmetic, as long as the GPIOs are actually configured as SPI
        // a byte will always be clocked into the RX buffer, regardless.
        // just don't like loops without a means to get out
        if ( 0 == --timeout )
        {
            break;
        }    
    };

    // read
    rxdata = SpiaRegs.SPIRXBUF;

    return rxdata;
}

// ****************************************************************************
// *
// *  Function: reset_spi_fifos
// *
// *  Purpose : To reset the Rx, TX fifo's 
// *
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description:  This module resets/ clears the Spi fifo's
// *
// ****************************************************************************
void reset_spi_fifos(void)
{
        // Reset the TX and RX Fifo's
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;            // useful name change... thanks TI!
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
}

// ****************************************************************************
// *
// *  Function: reset_and_conf_spi
// *
// *  Purpose : To reset the Rx, TX fifo's
// *
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description:  This module resets/ clears the Spi fifo's
// *
// ****************************************************************************
void reset_and_conf_spi(bool transfer_16bits,bool polarity_pos, uint16_t phase)
{
        // Reset the TX and RX Fifo's
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = transfer_16bits ? 15 : 7; // true=16 or f=8 bits
    SpiaRegs.SPICCR.bit.CLKPOLARITY = polarity_pos ? 1 : 0; // ?t:f. 1=ouput on falling edge.
    SpiaRegs.SPICTL.bit.CLK_PHASE = phase;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;

    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;            // useful name change... thanks TI!
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
}

// ****************************************************************************
// *
// *  Function: read_eeprom_array
// *
// *  Purpose :    To read a block of words from the  EEPROM device.
// *
// *  Parms Passed   : StartAddress -- The beginning address in the EEPROM to Read
// *
// *                   numWords     -- The number of 16 bit words to Read
// *
// *                   *data_ptr     -- A pointer to where the data read from the EEPROM is loaded
// *
// *                   op_type      -- Operation Type, op_type(s) are defined in EEPROM_DRV.H
// *
// *  Returns        :  if checksum operation, returns pass/fail
// *
// *  Description:    This module reads a block of EEPROM data from the EEPROM Device.
// *                 *data_ptr is only loaded with the data that is actually
// *                 read from the EEPROM Device for OP_TYPE EE_OP_READ_EEPROM.
// *
// *                In Other op_types, the data is either verfied or used to compute a
// *                checksum.
// ****************************************************************************
void read_eeprom_array(uint16_t start_address, uint16_t num_words, uint16_t *data_ptr, uint16_t op_type)
{
    uint16_t readData;   // Data read from eeprom

    // wait for previous write complete
    reset_and_conf_spi(false,true,0);
    WaitEEPROMBusy();
    
    if ( op_type == EE_OP_READ_CHECKSUM )
    {
        *data_ptr = 0; //reset the checksum
    }

    // reset fifos, enable the eeprom
    reset_spi_fifos();
    enable_SPI_CS( SPI_EEPROM );

    // send the read eeprom command, return value is garbage
    (void)sendSPIdata( READ_EEPROM_COMMAND );
    // now set to 16 bits
    // send the address, right shifted 1 words to bytes
    (void)sendSPIdata( start_address << 1 );
    (void)sendSPIdata( start_address << 9 );

    while ( 0 != num_words )
    {
        reset_spi_fifos();
        // send garbage data to receive the data
        readData = sendSPIdata( BOGUS_EEP_DATA );
        readData |= ( sendSPIdata( BOGUS_EEP_DATA ) ) << 8; 
              
        switch (op_type)  //this_length contains the read word
        {
             case EE_OP_READ_EEPROM:
                *data_ptr++ = readData;  //Save Read Data
                break;

             case EE_OP_READ_CHECKSUM:    //Compute the checksum of the data
                  //Save Checksum during the last pass
                (*data_ptr) += readData;
                break;
                
             default:
                break;
        }//switch(op)

        --num_words;
    }

    // end operation
    disable_SPI_CS();
}

// ****************************************************************************
// *
// *  Function: write_eeprom_sequence
// *
// *  Purpose :    A single write sequence that writes up to 16 WORDS to the EEPROM DEVICE
// *
// *  Parms Passed   : start_address -- The beginning address in the EEPROM to Read
// *
// *                   num_words     -- The number of 16 bit words to write
// *
// *                   *data_ptr     -- A pointer to a data array of size num_words or larger.
// *                                    If this pointer points to ROM cast as (INT16U *)
// *
// *  Returns        :     Returns the number of words written. The Number will vary depending
// *                     on Fifo Size and page boundaries.
// *
// *  Description:    This module completes one Write Sequence. The data pointed to
// *                by the RAM array *data_ptr length num_words is written
// *                to the EEPROM ADDRESS specified by start_address. The module
// *                returns after the write is complete (5-10 Msec).
// *
// *                The number of words written may be limited due to FIFO sizes and
// *                page boundaries. The actual number written is pass back as the return value.
// *
// ****************************************************************************
uint16_t write_eeprom_sequence( uint16_t start_address, uint16_t num_words, const uint16_t* data_ptr )
{

    uint16_t  results;            // Good =0, always good for now
    uint16_t  this_length;        // number of words written this time
    uint16_t  txdata;
    
    // wait for previous write complete
    reset_and_conf_spi(false,true,0);
    WaitEEPROMBusy();

    //compute the length based on a 8 word (16 byte) page
    //Fifo Size is 16 bytes or eight words
    this_length = 0x0008 - (start_address & 0x0007);
    if (this_length > num_words)
    {
        this_length = num_words;
    }

    results = this_length;         //return the number written

    // Load Write Enable Command, Stall while data goes out.
    reset_spi_fifos();
    enable_SPI_CS( SPI_EEPROM );
    (void)sendSPIdata( WRITE_ENABLE_COMMAND );

    // Latch Write Enable, Reset the TX and RX Fifo's
    disable_SPI_CS();
    reset_spi_fifos();
 
    //Load Read Command, wait for complete
    enable_SPI_CS( SPI_EEPROM );
    (void)sendSPIdata( WRITE_EEPROM_COMMAND );
    (void)sendSPIdata( start_address << 1 );
    (void)sendSPIdata( start_address << 9 );

    reset_spi_fifos();
           
    // transmit the data,
    while ( this_length-- != 0 )
    {
       txdata = *data_ptr;
       (void)sendSPIdata( txdata << 8 );     // LO
       (void)sendSPIdata( txdata );          // HI
       *data_ptr++;
    }

    //Wrote the last bit of the last word, now 
    //bring CS high to start EEPROM Write Cycle.
    disable_SPI_CS();

    return results;
}


// ****************************************************************************
// *
// *  Function: write_eeprom_array
// *
// *  Purpose :    To Write a block of words to the  EEPROM device
// *
// *  Parms Passed   : startAddress -- The beginning address in the EEPROM to Read
// *
// *                   numWords     -- The number of 16 bit words to Read
// *
// *                   *data_ptr     -- A pointer to a data array of size num_words or larger.
// *                                   This is where the data is written to from the EEPROM.
// *                                   data_ptr is only required if op_type is USE_DATA_PTR.
// *
// *                   op_type      -- Operation Type, op_type(s) are defined in EEPROM_DRV.H
// *
// *  Returns        :    (result) Results Flag formatted defined in EEPROM_DRV.H
// *
// *  Description:    This module writes a block of data to the EEPROM Device. The data is pointed
// *                to by the passed pointer *DataPtr if the op_type is USE_DATA_PTR. If Not the
// *                area is either cleared or erased.
// *
// *                A write over section limits is split into multiple writes which lie within section
// *                limits.
// *
// *                A write into checksum address destroys the section checksum.
// *                If you do this, you have to rewrite the whole section at once to
// *                get a valid checksum (or do it like EE_RebootCheckSumSection() does).
// *
// *                The Results of the Operation are passed back to the calling module.
// *
// ****************************************************************************
void write_eeprom_array( uint16_t startAddress, uint16_t numWords, uint16_t* dataPtr, uint16_t opType )
{
    uint16_t section_number;              // Index into EeFAT[] with EEPROM MAP INFO
    uint16_t this_length;                 // temp space

    // Get section
    section_number = GetSectionNumber( startAddress ); 

    // Find the last address of this section
    this_length = GetSectionAddress( section_number ) + GetSectionLength( section_number );
     
    // verify the write does not cross section boundary and valid section
    if ( ( ( startAddress + numWords ) <= this_length ) && ( section_number < EE_Last_Section ) )
    {
        while ( 0 != numWords )
        {
            // check what type of data to write 
            // (Note: writing is performed in packs of EE_PAGESIZE_W words, or less
            //  Thus when using eeprom_temp_data as a data source, for clear and erase,
            //  the buffer pointer should be reset back to the start after each
            //  sequence/pack write to prevent buffer access overflow)

            switch (opType)
            {
                case EE_OP_WRITE_EEPROM:
                      // user-specified data buffer
                    break;

                case EE_OP_CLEAR_EEPROM:
                    dataPtr = (uint16_t *) &eeprom_clear_data[0];
                    break;

                case EE_OP_ERASE_EEPROM:
                    dataPtr = (uint16_t *) &eeprom_erase_data[0]; // (16 word buffer)
                    break;
                    
                default:
                    break;
            }

            // Write the data (taking into account EEPROM page boundaries)
            this_length = write_eeprom_sequence( startAddress, numWords, dataPtr );

            // From the returned num of bytes written, calculate how
            // many are still left to write, and update the start address
            numWords -= this_length;
            startAddress += this_length;
            dataPtr += this_length;
        } // end while ( numWords )
    }
}



// ****************************************************************************
// *
// *  Function: GetEEPROMStatusRegistor
// *
// *  Purpose :    To read a block of words from the  EEPROM device.
// *
// *  Parms Passed   : 
// *
// *  Returns        :  if checksum operation, returns pass/fail
// *
// *  Description:    
// *
// ****************************************************************************
uint16_t GetEEPROMStatusRegistor()
{
    uint16_t rxdata = 0xffff;
    uint16_t tmp_st = 0;
    
    enable_SPI_CS( SPI_EEPROM );    
    
    SpiaRegs.SPITXBUF = READ_STATUS_COMMAND;
    DSP28x_usDelay( 50*37 );
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;    
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;
    SpiaRegs.SPITXBUF = BOGUS_EEP_DATA;
    DSP28x_usDelay( 50*37 );

    if (SpiaRegs.SPIFFRX.bit.RXFFOVF)
    {
        rxdata = 0xffff;
    }
    else
    {
        tmp_st = SpiaRegs.SPIFFRX.bit.RXFFST;
        if (tmp_st == 1)
        {   
            rxdata = SpiaRegs.SPIRXBUF;
        }
    }
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    
    disable_SPI_CS();
    
    return (rxdata);    
}

// ***********************************************************************
// *
// *    FUNCTION: WaitEEPROMBusy 
// *
// *    DESCRIPTION: Reads the EE status register, delays if busy
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void WaitEEPROMBusy( void )
{
    uint16_t status;
    uint16_t idx;

    // Wait for the write cycle to finish: poll the internal 
    // Status Register of the EEPROM chip for the Write-In-Progress bit
    for ( idx = 2000; idx > 0 ; --idx )
    {
        // get the status register
        status = GetEEPROMStatusRegistor();

         // check the if WIP bit (bit 0) is already cleared (= write finished)
        if ( 0 == (status & 1) )
        {
           status = 0; // (just to set a debugging breakpoint)
           break;
        }

        TSK_sleep( TSK_5_ms );
    }
}

// // ****************************************************************************
// // *
// // *  Function: WriteEEROM
// // *
// // *  Purpose :    To read a block of words from the  EEPROM device.
// // *
// // *  Parms Passed   : StartAddress -- The beginning address in the EEPROM to Read
// // *
// // *                   *data_ptr     -- A pointer to where the data read from the EEPROM is loaded
// // *
// // *  Returns        :  if checksum operation, returns pass/fail
// // *
// // *  Description:    This module reads a block of EEPROM data from the EEPROM Device.
// // *
// // ****************************************************************************
// uint16_t WriteEEROM( uint16_t start_address, uint16_t *data_ptr )
// {
//     uint16_t num_words = 1;
//     
//     enable_SPI_CS( SPI_EEPROM );          
//     SpiaRegs.SPITXBUF = WRITE_ENABLE_COMMAND;       
//     DSP28x_usDelay( 16*37 );    
//     disable_SPI_CS();   
//     
//     reset_spi_fifos();      
//     enable_SPI_CS(SPI_EEPROM);
//     SpiaRegs.SPITXBUF = WRITE_EEPROM_COMMAND;
//     
//     DSP28x_usDelay( 16*37 );
//     reset_spi_fifos();
//     SpiaRegs.SPITXBUF = start_address << 1;
//     SpiaRegs.SPITXBUF = start_address << 9;
//                     
//     SpiaRegs.SPITXBUF = *data_ptr << 8;
//     SpiaRegs.SPITXBUF = *data_ptr;
//     
//     DSP28x_usDelay( 250*37 );
//     disable_SPI_CS();
//     
//     //wait ~5ms
// //     DSP28x_usDelay( 1000*32 );
// //     DSP28x_usDelay( 1000*32 );
// //     DSP28x_usDelay( 1000*32 );
// //     DSP28x_usDelay( 1000*32 );
// //     DSP28x_usDelay( 1000*32 );
// //     DSP28x_usDelay( 1000*32 );
//     return (num_words);    
// }
// 
// ****************************************************************************
// *
// *  Function: ReadEEROM
// *
// *  Purpose :    To read a block of words from the  EEPROM device.
// *
// *  Parms Passed   : StartAddress -- The beginning address in the EEPROM to Read
// *
// *                   *data_ptr     -- A pointer to where the data read from the EEPROM is loaded
// *
// *  Returns        :  num of word read
// *
// *  Description:    
// *
// ****************************************************************************
uint16_t ReadEEROM(uint16_t start_address, uint16_t *data_ptr)
{
    uint16_t rxdata = 0;
    uint16_t tmp_st = TRUE;
    
    enable_SPI_CS( SPI_EEPROM );
    reset_spi_fifos();    
    SpiaRegs.SPITXBUF = READ_EEPROM_COMMAND;
        
    SpiaRegs.SPITXBUF = start_address << 1;
    SpiaRegs.SPITXBUF = start_address << 9;
    DSP28x_usDelay( 50*37 );
                
    reset_spi_fifos();
    SpiaRegs.SPITXBUF = BOGUS_EEP_DATA;
    SpiaRegs.SPITXBUF = BOGUS_EEP_DATA;
    DSP28x_usDelay( 50*37 );
    
    if (SpiaRegs.SPIFFRX.bit.RXFFOVF)
    {
        tmp_st = 0;
    }
    else
    {
        tmp_st = SpiaRegs.SPIFFRX.bit.RXFFST;
        
        rxdata = SpiaRegs.SPIRXBUF;
        if ( tmp_st > 1) 
        {
            rxdata |= SpiaRegs.SPIRXBUF << 8;
        }       
    }
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    
    disable_SPI_CS();
    
    *data_ptr = rxdata;
    
    return (tmp_st);    
}

void read_IOX_sequence(uint16_t addr, uint16_t *data_ptr)
{
    uint16_t x;
    reset_and_conf_spi(true,true,0);
    enable_SPI_CS( SPI_IOX );
    x= IOX_DEVICE_OP_CODE + IOX_READ + addr;
    (void)sendSPIdata( x ); // send the read command, return value is garbage
    *data_ptr=sendSPIdata( 0xf0f0 ); //dummy send
    disable_SPI_CS(); // end operation
}

void write_IOX_sequence(uint16_t addr, const uint16_t *data_ptr)
{
    uint16_t x;
    reset_and_conf_spi(true,true,0);
    enable_SPI_CS( SPI_IOX );
    x= IOX_DEVICE_OP_CODE + IOX_WRITE + addr;
    (void)sendSPIdata( x );
    sendSPIdata( *data_ptr ); //write data, GPIOA and GPIOB, read data discarded
    disable_SPI_CS(); // end operation

}

void Init_IOX(void)
{
    uint16_t d,e;
    ExpansionInputReg.Ready = false;
    ExpansionInputReg.Init= false;
    PLDVersionReg.Ready = false;
    PLDTrapReg.Ready = false;
    read_PLD_sequence(PLD_ADDR_TRAP, (uint16_t *)&PLDTrapBoot); //store PLD trap register value for later use.
    PLDTrapReg.Trap = PLDTrapBoot; // setting PLDTrapReg value also means that PLD read is completed.
    read_PLD_sequence_i16(PLD_ADDR_VER1 , (uint16_t *)&PLDVersionReg);
    read_PLD_sequence(PLD_ADDR_STAT1, (uint16_t *)&PLDStatusReg);
    // 3 RevID bits are split to ioexpander and PLD pins.

    d = ~BIT12;	//EpoDisable is output singal
    read_IOX_sequence( IOX_REG_IODIRA, &e); //check that ioexpander can store configuration.
    write_IOX_sequence( IOX_REG_IODIRA, &d);
    read_IOX_sequence( IOX_REG_IODIRA, &e); //check that ioexpander can store configuration.
    if( d!=e )
    {
        NB_DebounceAndQue(UPM_NB_SPI_BUS_FAILURE, true, 1);
    }
    read_IOX_sequence( IOX_REG_GPIOA, (uint16_t*)&ExpansionInputReg.all);

    ExpansionInputReg.Init = true;
    ExpansionInputReg.Ready = true;
    PLDVersionReg.Ready = true;
    PLDTrapReg.Ready = true;

}

void read_PLD_sequence(uint16_t start_address, uint16_t *data_ptr)
{
    reset_and_conf_spi(false,false,1);
    enable_SPI_CS( SPI_PLD );
    (void)sendSPIdata( (BIT6 | start_address)<<8 ); // send the read command, return value is garbage
    *data_ptr=0xff & sendSPIdata( 0xf0f0 ); //dummy send
    disable_SPI_CS(); // end operation
}
/* read 2 i8 and stuff them to i16, msb read from addr, lsb from addr+1 */
void read_PLD_sequence_i16(uint16_t start_address, uint16_t *data_ptr)
{
    uint16_t tmp,tmp2;
    reset_and_conf_spi(false,false,1);
    enable_SPI_CS( SPI_PLD );
    (void)sendSPIdata( (BIT6 | start_address)<<8 ); // send the command, return value is garbage
    tmp=sendSPIdata( 0xf0f0 ); //dummy send
    tmp = (tmp & 0xff)<<8;
    start_address++;
    (void)sendSPIdata( (BIT6 | start_address)<<8 ); // send the command, return value is garbage
    tmp2=sendSPIdata( 0xf0f0 ); //dummy send
    tmp= tmp+ (tmp2&0xff);
    *data_ptr=tmp;
    disable_SPI_CS(); // end operation
}

void write_PLD_sequence(uint16_t start_address, uint16_t *data_ptr)
{
    reset_and_conf_spi(false,false,1);
    enable_SPI_CS( SPI_PLD );
    (void)sendSPIdata( (BIT7 | start_address)<<8 ); // send the  command, return value is garbage
    sendSPIdata( (*data_ptr)<<8 ); //write data, read data discarded
    disable_SPI_CS(); // end operation
}

// ************************************************************************************
//  No more
// ************************************************************************************

