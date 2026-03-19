// ******************************************************************************************************
// *            I2c_Driver.c
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
// *    FILE NAME: IOexpansion.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR:
// *
// *    DATE: 3/11/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"   // Device Headerfile and Examples Include File
#include "I2c_Driver.h"
#include "IOexpansion.h"
#include "Alarms.h"
#include "DebuggerBlocks.h"
#include "NB_Funcs.h"
#include "F28335Port.h"
#include "EncryptionVry.h"
#include "Eeprom_Map.h"
extern "C"
{
    void InitI2c( void);
    void I2c_Interrupt(void);
}

uint16_t I2c_Operation = cOperation_Null;  // Current I2c Operation 
uint16_t I2c_Device = cDevice_Null;        // Current I2c Device
bool I2c_Enable = true;                    // Current I2c Status
uint16_t I2C_error_counters[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
bool PLD_DT_Init = false;

// ***********************************************************************
// *
// *    FUNCTION: InitI2c 
// *
// *    DESCRIPTION: Initialize I2c IO registers and I2c control registers
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none 
// *
// ***********************************************************************
void InitI2c(void)
{
    Init_I2c_Gpio();
    Init_I2c_Register();
    I2c_Operation = cOperation_Null;
    I2c_Device = cDevice_Null;
    I2c_Enable = true;

    if( ControlBoardRevID == ControlBoardTPT29555 )
    {
        ClearExpansionIOReg();
        ExpansionInputReg.all = 0xFFFF;
        ExpansionInputReg.Init = false;
        ExpansionInputReg.Ready = false;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: Init_I2c_Gpio 
// *
// *    DESCRIPTION: Initialize I2c IO registers.
// *                 
// *    ARGUMENTS: none
// *
// *    RETURNS: none 
// *
// ***********************************************************************
void Init_I2c_Gpio(void)
{
    EALLOW;
    
    GpioCtrlRegs.GPBPUD.bit.GPIO32   = 1;   // Disable pull-up on GPIO32
    GpioCtrlRegs.GPBMUX1.bit.GPIO32  = 1;   // SDAA
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;   // Asynch input GPIO32 (SDAA)
    
    GpioCtrlRegs.GPBPUD.bit.GPIO33   = 1;   // Disable pull-up on GPIO33
    GpioCtrlRegs.GPBMUX1.bit.GPIO33  = 1;   // SCLA
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;   // Asynch input GPIO33 (SCLA)
    
    EDIS;
}
 
void nop(void)
{
    asm("nop");
}

// ***********************************************************************
// *
// *    FUNCTION: Init_I2c_Register
// *
// *    DESCRIPTION: Initialize I2c IO control registers,I2c speed is 400kHz.
// *                 It is inappropriate to call this function from
// *                 interrupt context due to some extra delays in the function.
// *                 
// *                 
// *    ARGUMENTS: none
// *
// *    RETURNS: none 
// *
// ***********************************************************************
void Init_I2c_Register(void)
{
    I2caRegs.I2CMDR.all = 0x0000;   // Reset I2c Module
    if (DSPInRegister.GpiB.bit.I2CSCL && !DSPInRegister.GpiB.bit.I2CSDA)
    {
        // If clock is high and data is low, then something is holding onto the
        // bus.  Attempt to clear it by sending an explicit start/stop pair.
        
        // Configure as GPIO
        {
            CriticalSection enter;
            EALLOW;
            GPB1_REG gpbmux1;
            gpbmux1.all = GpioCtrlRegs.GPBMUX1.all;
            GPBDAT_REG gpbdir;
            gpbdir.all = GpioCtrlRegs.GPBDIR.all;
        
            // Configure as GPIO
            gpbmux1.bit.GPIO33 = 0;
            // Configure as output
            gpbdir.bit.GPIO33 = 1;
            
            GpioCtrlRegs.GPBDIR.all = gpbdir.all;
            GpioCtrlRegs.GPBMUX1.all = gpbmux1.all;
            EDIS;
        }
        // Bit-bang out 9 clocks on SCL to send Bus Clear.  See section 3.16 of I2C spec
        for (unsigned bit = 0; bit < 9; ++bit) {
            // Force low
            GPBDAT_REG set;;
            set.all = 0ul;
            set.bit.GPIO33 = 1;
            
            GpioDataRegs.GPBCLEAR.all = set.all;
            // wait for 1.3 uS.  This loop gets compiled into a single-clock
            // decrement-and-branch-if-zero per iteration.
            for (int i = 0; i < 195; ++i)
            {
                nop();
            }
            // Allow high
            GpioDataRegs.GPBSET.all = set.all;
            // wait for 0.6 us.
            for (int i = 0; i < 100; ++i)
            {
                nop();
            }
        }
        
        // Restore GPIO to SCL functionality
        {
            CriticalSection enter;
            EALLOW;
            GPBDAT_REG gpbpud;
            gpbpud.all = GpioCtrlRegs.GPBPUD.all;
            GPB1_REG gpbmux1;
            gpbmux1.all = GpioCtrlRegs.GPBMUX1.all;
            GPB1_REG gpbqsel1;
            gpbqsel1.all = GpioCtrlRegs.GPBQSEL1.all;
            
            
            gpbpud.bit.GPIO33   = 1;
            gpbmux1.bit.GPIO33  = 1;
            gpbqsel1.bit.GPIO33 = 3;
            
            GpioCtrlRegs.GPBPUD.all = gpbpud.all;
            GpioCtrlRegs.GPBMUX1.all = gpbmux1.all;
            GpioCtrlRegs.GPBQSEL1.all = gpbqsel1.all;
            EDIS;
        }

        // Treat the error as Bus Busy. The hardware does not signal this error.
        I2C_error_counters[I2C_ERROR_BUSY - 1]++;
    }
    
    I2caRegs.I2CPSC.all = 14;       // Prescaler to 10MHz(within 7-12MHz)
    I2caRegs.I2CCLKL = 10;          // 10MHz/(10+5+5+5)=400KHz
    I2caRegs.I2CCLKH = 5;           // 
    I2caRegs.I2CIER.all = 0;        // Disable SCD & ARDY interrupts
    I2caRegs.I2CMDR.all = 0x0020;   // Take I2c out of reset,Stop when suspended
    I2caRegs.I2CFFTX.all = 0x6000;  // Enable FIFO mode and TXFIFO
    I2caRegs.I2CFFRX.all = 0x2040;  // Enable RXFIFO, clear RXFFINT,                                           
    I2caRegs.I2CIER.bit.SCD = 1;    // Enable SCD Interrupt
    if (I2caRegs.I2CSTR.bit.BB) {
        NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_BUSY);
        I2C_error_counters[I2C_ERROR_BUSY - 1]++;
    }
}

bool I2c_Get_Idle_Status(void)
{
    // Bus will remain in the non-idle state until either the pending operation
    // completes, or the module is reset on timeout
    if( ( I2c_Enable == false ) && 
        ( I2c_Device == cDevice_Null ) &&
        ( I2c_Operation == cOperation_Null ) )
    {
        return(true);
    }
    
    return(false);    
}

// ***********************************************************************
// *
// *    FUNCTION: I2c_Write_Read
// *
// *    DESCRIPTION: Write to/Read from IO expander.
// *                 
// *    ARGUMENTS: data : data write or read to(from) IO expander.
// *               type : value cI2c_Write - Write operation;
// *                      value cI2c_Read  - Read operation;  
// *               bcount: Bytes of input data
// *    RETURNS: none 
// *
// ***********************************************************************
bool I2c_Write_Read(uint16_t device,uint16_t operation,uint16_t *data, uint16_t bcount)
{
    bool resourceAvailable = false;
    
    if( I2c_Enable == false)
    {
        return false;
    }
    if((Encrypt_Verify_204.I2C_Block == false) || (device == cDevice_SHA204))
    {
    
    {
        CriticalSection enter;
        
        if ( ( ( operation == cOperation_Write ) ||
               ( operation == cOperation_Read  ) ) &&
             ( I2c_Operation == cOperation_Null ) )
        {
            I2c_Operation = operation;
            resourceAvailable = true;
        }

    }
            
    if((operation == cOperation_Write) && resourceAvailable)
    {
        if( device == cDevice_SHA204 )
        {
            CriticalSection enter(IER_DMA_PROC_ADC_ONLY);//

            I2c_Device = cDevice_SHA204;
            I2caRegs.I2CSAR = cAddress_SHA204;
            I2caRegs.I2CCNT = bcount;
            I2caRegs.I2CMDR.all = 0x6E20; //Write
            for(uint16_t n = 0; n < bcount; n++ )
            {
                uint32_t timeout = cI2c_Byte_Timeout;
                I2caRegs.I2CDXR = ReadByteinWord(data, n);
                if (I2caRegs.I2CSTR.bit.NACK )
                {
                    I2C_error_counters[I2C_ERROR_NACK_SHA204 - 1]++;
                }
                while( I2caRegs.I2CFFTX.bit.TXFFST != 0 ) // TX FIFO contains xxxx bytes when 0 it means datas have been sent out.
                {
                    if( timeout-- == 0 )
                    {
                        I2caRegs.I2CFFTX.bit.TXFFRST = 0; //Reset the transmit FIFO pointer to 0000 and hold the transmit FIFO in the reset state.
                        I2c_Device = cDevice_Null;
                        I2c_Operation = cOperation_Null;
                        resourceAvailable = false;
                        EncryptionTest = 0x8000+n;//for encryptiontest
                        I2caRegs.I2CMDR.all = 0x0E20;     //Send out a STOP condition to free the I2C bus
                        I2caRegs.I2CFFTX.bit.TXFFRST = 1; //Enable the transmit FIFO operation.
                        return resourceAvailable;
                    }
                }

            }
            I2c_Device = cDevice_Null;
            I2c_Operation = cOperation_Null;
        }
        else
        {
            CriticalSection enter;

            switch ( device )
            {
                case cDevice_PCF8575:
                    I2c_Device = cDevice_PCF8575;
                    I2caRegs.I2CSAR = cAddress_PCF8575;
                    I2caRegs.I2CCNT = 2;
                    I2caRegs.I2CMDR.all = 0x6E20;
                    I2caRegs.I2CDXR = (*data) & 0x00FF;
                    I2caRegs.I2CDXR = (*data) >> 8;
                    break;
                case cDevice_TPT29555:
                    I2c_Device = cDevice_TPT29555;
                    I2caRegs.I2CSAR = cAddress_TPT29555;
                    I2caRegs.I2CCNT = 2;
                    I2caRegs.I2CMDR.all = 0x6E20;
                    I2caRegs.I2CDXR = (*data) & 0x00FF;
                    I2caRegs.I2CDXR = (*data) >> 8;
                    break;
                case cDevice_PLD:
                    I2c_Device = cDevice_PLD;
                    I2caRegs.I2CSAR = cAddress_PLD;
                    I2caRegs.I2CCNT = 1;
                    I2caRegs.I2CMDR.all = 0x6E20;
                    I2caRegs.I2CDXR = (*data) & 0x00FF;
                    break;
                case cDevice_PLD_TRAP:
                    I2c_Device = cDevice_PLD_TRAP;
                    I2caRegs.I2CSAR = cAddress_PLD_TRAP;
                    I2caRegs.I2CCNT = 1;
                    I2caRegs.I2CMDR.all = 0x6E20;
                    I2caRegs.I2CDXR = (*data) & 0x00FF;
                    break;
                default:  // This wasn't a valid device, release I2c_Operation
                    I2c_Operation = cOperation_Null;
                    break;
            }
        }

    }
    else if((operation == cOperation_Read) && resourceAvailable)
    {
        if( device == cDevice_SHA204 )
        {
            CriticalSection enter(IER_DMA_PROC_ADC_ONLY); //

            I2c_Device = cDevice_SHA204;
            I2caRegs.I2CSAR = cAddress_SHA204;
            I2caRegs.I2CCNT = bcount;
            I2caRegs.I2CMDR.all = 0x6C20; // Read .
            for(uint16_t n = 0; n < bcount; n++)
            {
                if (I2caRegs.I2CSTR.bit.NACK)
                {
                    I2C_error_counters[I2C_ERROR_NACK_SHA204 - 1]++;
                }
                uint32_t timeout = cI2c_Byte_Timeout;
                while(!I2caRegs.I2CFFRX.bit.RXFFST) //RX FIFO contains xxxx bytes when 0 it means datas have been Received.
                {
                    if(timeout-- == 0)
                    {
                        I2caRegs.I2CFFRX.bit.RXFFRST = 0; //Reset the transmit FIFO pointer to 0000 and hold the transmit FIFO in the reset state.
                        I2c_Device = cDevice_Null;
                        I2c_Operation = cOperation_Null;
                        resourceAvailable = false;
                        EncryptionTest = 0x4000+n;//for encryptiontest
                        I2caRegs.I2CMDR.all = 0x0C20;     //Send out a STOP condition to free the I2C bus
                        I2caRegs.I2CFFRX.bit.RXFFRST = 1; //Enable the transmit FIFO operation.
                        return resourceAvailable;
                    }
                }
                WriteBytetoWord(Encrypt_Verify_204.Response, n, I2caRegs.I2CDRR);
            }
            I2c_Device = cDevice_Null;
            I2c_Operation = cOperation_Null;
        }
        else
        {
            CriticalSection enter;
            switch ( device )
            {
                case cDevice_PCF8575:
                    I2c_Device = cDevice_PCF8575;
                    I2caRegs.I2CSAR = cAddress_PCF8575;
                    I2caRegs.I2CCNT = 2;
                    I2caRegs.I2CMDR.all = 0x6C20;
                    break;
                case cDevice_TPT29555:
                    I2c_Device = cDevice_TPT29555;
                    I2caRegs.I2CSAR = cAddress_TPT29555;
                    I2caRegs.I2CCNT = 2;
                    I2caRegs.I2CMDR.all = 0x6C20;
                    break;
                case cDevice_PLD:
                    I2c_Device = cDevice_PLD;
                    I2caRegs.I2CSAR = cAddress_PLD;
                    I2caRegs.I2CCNT = 2;
                    I2caRegs.I2CMDR.all = 0x6C20;
                    break;
                case cDevice_PLD_TRAP:
                    I2c_Device = cDevice_PLD_TRAP;
                    I2caRegs.I2CSAR = cAddress_PLD_TRAP;
                    I2caRegs.I2CCNT = 1;
                    I2caRegs.I2CMDR.all = 0x6C20;
                    break;
                default:  // This wasn't a valid device, release I2c_Operation
                    I2c_Operation = cOperation_Null;
                    break;
            }
        }
    }
    
    }//End of I2CBlock
    return resourceAvailable;
}

// ***********************************************************************
// *
// *    FUNCTION: I2c_Interrupt
// *
// *    DESCRIPTION: Write to/Read from IO expander.
// *                 
// *    ARGUMENTS: data : data write to IO expander(only for Write operation).
// *               type : value cI2c_Write - Write operation;
// *                      value cI2c_Read  - Read operation;  
// *
// *    RETURNS: none 
// *
// ***********************************************************************

#pragma CODE_SECTION("ramfuncs");
void I2c_Interrupt(void)
{

    uint16_t uwI2cIntCode = 0;
    uint16_t uwDataTemp1 = 0;
    uint16_t uwDataTemp2 = 0;
    
    //Step1 : Read Interrupt Code
    {
        I2CISRC_REG i2cisrc_reg_tmp;
        
        i2cisrc_reg_tmp.all = uint16_t(I2caRegs.I2CISRC.all);
        uwI2cIntCode = i2cisrc_reg_tmp.bit.INTCODE;
    }
    
    //Step2 : Interrupt Service
    if(I2c_Operation == cOperation_Write)
    {       
        if(uwI2cIntCode == cI2c_Isr_SCD)
        {
            if (I2caRegs.I2CFFTX.bit.TXFFST == 0)
            {
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, false);
                if(I2c_Device == cDevice_PCF8575)
                {
                   if( ControlBoardRevID == ControlBoardOld )
                   {
                     ExpansionInputReg.Init = true;
                    }
                }
                else if(I2c_Device == cDevice_TPT29555)
                {
                    ExpansionInputReg.Init = true;
                }

                I2c_Device = cDevice_Null;
                I2c_Operation = cOperation_Null;
                I2caRegs.I2CSTR.all = 0x0020; // Do not use bit writes to this register, it is Write1-Clear
            }
            else
            {
                // STOP condtion detected, but not at the end-of-transfer
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_ESCD);
                I2caRegs.I2CSTR.all = 0x0020;
                I2C_error_counters[I2C_ERROR_ESCD - 1]++;
            }
        }
    }
    else if(I2c_Operation == cOperation_Read)
    {   
        if(uwI2cIntCode == cI2c_Isr_SCD)
        {
            I2CFFRX_REG ffrx; ffrx.all = I2caRegs.I2CFFRX.all;
            if (ffrx.bit.RXFFST != I2caRegs.I2CCNT && I2c_Device != cDevice_SHA204) // Don't use when device is SHA204
            {
                // STOP condtion detected, but not at the end-of-transfer
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_ESCD);
                I2caRegs.I2CSTR.all = 0x0020;
                I2C_error_counters[I2C_ERROR_ESCD - 1]++;
            } 
            else
            {
                I2c_Operation = cOperation_Null;
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, false);
                I2caRegs.I2CSTR.all = 0x0020; // Do not use bit writes to this register, it is Write1-Clear

                if(I2c_Device == cDevice_TPT29555)
                {
                    uwDataTemp1 = I2caRegs.I2CDRR;
                    uwDataTemp2 = I2caRegs.I2CDRR;
                    ExpansionInputReg.all = ((uwDataTemp2 << 8) | (uwDataTemp1 & 0x00FF));
                    ExpansionInputReg.Ready = true;
                    I2C_error_counters[2] = ExpansionInputReg.all;
                 }
                else if(I2c_Device == cDevice_PCF8575)
                {
                    uwDataTemp1 = I2caRegs.I2CDRR;
                    uwDataTemp2 = I2caRegs.I2CDRR;
                    ExpansionInputReg.all = ((uwDataTemp2 << 8) | (uwDataTemp1 & 0x00FF));
                    ExpansionInputReg.Ready = true;
                }
                else if(I2c_Device == cDevice_PLD)
                {
                    uwDataTemp1 = I2caRegs.I2CDRR;
                    uwDataTemp2 = I2caRegs.I2CDRR;
                    PLDVersionReg.Version = uwDataTemp1 * 100 + uwDataTemp2;
                    PLDVersionReg.Ready = true;
                }
                else if(I2c_Device == cDevice_PLD_TRAP)
                {
                	 /* PLDTrapReg.all = I2caRegs.I2CDRR;
                    
                    if ( !PLDTrapReg.Ready )
                    {
                        PLDTrapBoot = PLDTrapReg.bit.Trap;
                    }*/
                    PLDTrapReg.Ready = true;
                }
                I2c_Device = cDevice_Null;
            }
        }
    }
    //Step3 : Clear Group Interrupt Flag 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

}


// ***********************************************************************
// *
// *    FUNCTION: I2c_StateCheck 
// *
// *    DESCRIPTION: This function checks I2c communication state.If errors
// *                 are found then I2c module will be reset and Warning flag
// *                 is set.This functions runs every 80kHz/16/4=0.8ms and 
// *                 Warning flag set every 4ms.
// *                 
// *    ARGUMENTS: None  
// *
// *    RETURNS: None 
// *
// ***********************************************************************
#pragma CODE_SECTION("ramfuncs");
void I2c_StateCheck(void)
{
    static uint16_t cnt = 0;
    
    if(I2c_Operation != cOperation_Null)
    {
        I2CSTR_REG status;
        status.all = I2caRegs.I2CSTR.all;
        uint16_t addr = I2caRegs.I2CSAR;
        
        bool error = false;
        if (status.bit.ARBL)
        {
            NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_ARBITRATION);
            I2C_error_counters[I2C_ERROR_ARBITRATION - 1]++;
            error = true;
        }
        if (status.bit.NACK)
        {
            if ((addr == cAddress_PCF8575)||(addr == cAddress_TPT29555))
            {
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_NACK_IOX);
                I2C_error_counters[I2C_ERROR_NACK_IOX - 1]++;       
            }
            else if (addr == cAddress_PLD || cAddress_PLD_TRAP)
            {
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_NACK_PLD);
                I2C_error_counters[I2C_ERROR_NACK_PLD - 1]++;
            }
            else if(addr == cAddress_SHA204)
            {
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_NACK_SHA204);
                I2C_error_counters[I2C_ERROR_NACK_SHA204 - 1]++;
            }
            error = true;
        }
        if (status.bit.RSFULL)
        {
            NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_RSFULL);
            I2C_error_counters[I2C_ERROR_RSFULL - 1]++;
            error = true;
        }
        
        if(++cnt >= 5 || error)
        {
            // Read why...
            if (!error)
            {
                NB_DebounceAndQue(UPM_NB_I2C_FAIL, true, I2C_ERROR_TIMEOUT);
                I2C_error_counters[I2C_ERROR_TIMEOUT - 1]++;
            }
            // Force a module reset
            cnt = 0;
            Init_I2c_Register();
            I2c_Operation = cOperation_Null; 
            I2c_Device = cDevice_Null;
        }
        //Block timeout detect when device is ATsha204
        if(Encrypt_Verify_204.I2C_Block == true)
        {
             cnt = 0;
        }
    }
    else
    {
        cnt = 0;
    }
}


// ***********************************************************************
// *
// *    FUNCTION    : I2c_Init_for_TRAP 
// *
// *    DESCRIPTION : This function Initializes I2c module and can only be
// *                  called when all interrupts are disabled

// *    ARGUMENTS   : None  
// *
// *    RETURNS     : None    
// *
// ***********************************************************************
void I2c_Init_for_TRAP(void)
{
    Init_I2c_Gpio();
    Init_I2c_Register();
}

// ***********************************************************************
// *
// *    FUNCTION    : Check_I2c_State_for_TRAP 
// *
// *    DESCRIPTION : This function checks I2c communication state by checking
// *                  SCD bit, which is used to record a STOP condition.
// *                  NOTE : THIS FUNCTION IS FOR TRAP FUNCTION ONLY !!!

// *    ARGUMENTS   : None  
// *
// *    RETURNS     : false - a STOP condtion not detected; true - a STOP 
// *                  conditon is detected    
// *
// ***********************************************************************
uint16_t I2c_State_Check_for_TRAP(void)
{
    uint16_t timeout = 0;
    
    while(I2caRegs.I2CSTR.bit.SCD == 0)
    {
        if(++timeout >= 1000)
        {
            timeout = 0;
            return(false);
        }    
    }
    
    return(true);
}

// ***********************************************************************
// *
// *    FUNCTION    : I2c_Write_Read_for_TRAP 
// *
// *    DESCRIPTION : This function perform I2c communication by polling
// *                  register status instead of interrupt.
// *                  NOTE : THIS FUNCTION IS FOR TRAP FUNCTION ONLY !!!

// *    ARGUMENTS   : device    - ExpansionIO or PLD
// *                  operation - Read or Write operation
// *                  data      - only used for write operation,16 bit
// *                  retry     - retry times in case of errors detected   
// *
// *    RETURNS     : None    
// *
// ***********************************************************************
/*
void I2c_Write_Read_for_TRAP(uint16_t device,uint16_t operation,uint16_t data,uint16_t retry)
{
    volatile uint16_t temp1 = 0;
    volatile uint16_t temp2 = 0;
    
    while(retry--)
    {
        // write register
        if(operation == cOperation_Write)
        {
            uint16_t tmp_I2C_operation = I2c_Operation;
            I2c_Operation = cOperation_Write;
            
            if(device == cDevice_PCF8575)
            {
                I2caRegs.I2CSAR = cAddress_PCF8575;
                I2caRegs.I2CCNT = 2;
                I2caRegs.I2CDXR = data & 0x00FF;
                I2caRegs.I2CDXR = data >> 8;    
                I2caRegs.I2CMDR.all = 0x2E20;
            }
            else if(device == cDevice_PLD)
            {
                I2caRegs.I2CSAR = cAddress_PLD;
                I2caRegs.I2CCNT = 1;
                I2caRegs.I2CDXR = data & 0x00FF;
                I2caRegs.I2CMDR.all = 0x2E20;
            }
            else if(device == cDevice_PLD_TRAP)
            {
                data |= UpmModel;
                I2caRegs.I2CSAR = cAddress_PLD_TRAP;
                I2caRegs.I2CCNT = 1;
                I2caRegs.I2CDXR = data & 0x00FF;
                I2caRegs.I2CMDR.all = 0x2E20;
            }
            else
            {
                I2c_Operation = tmp_I2C_operation;
            }
        }
        else if(operation == cOperation_Read)
        {
            uint16_t tmp_I2C_operation = I2c_Operation;
            I2c_Operation = cOperation_Read;
            
            if(device == cDevice_PCF8575)
            {
                I2caRegs.I2CSAR = cAddress_PCF8575;
                I2caRegs.I2CCNT = 2;
                I2caRegs.I2CMDR.all = 0x2C20;
            }
            else if(device == cDevice_PLD)
            {
                I2caRegs.I2CSAR = cAddress_PLD;
                I2caRegs.I2CCNT = 2;
                I2caRegs.I2CMDR.all = 0x2C20;
            }
            else if(device == cDevice_PLD_TRAP)
            {
                I2caRegs.I2CSAR = cAddress_PLD_TRAP;
                I2caRegs.I2CCNT = 1;
                I2caRegs.I2CMDR.all = 0x2C20;
            }
            else
            {
                I2c_Operation = tmp_I2C_operation;
            }
        }
        
        // check state
        if(I2c_State_Check_for_TRAP() == false)
        {
            Init_I2c_Register();
        }
        else
        {
            if(operation == cOperation_Read)
            {
                switch ( device )
                {
                    case cDevice_PCF8575:
                        temp1 = I2caRegs.I2CDRR;
                        temp2 = I2caRegs.I2CDRR;
                        ExpansionInputReg.all = ((temp2 << 8) | (temp1 & 0x00FF));
                        break;
                    case cDevice_PLD:
                        temp1 = I2caRegs.I2CDRR;
                        temp2 = I2caRegs.I2CDRR;
                        PLDVersionReg.Version = temp1 * 100 + temp2;
                        break;
                    case cDevice_PLD_TRAP:
                        PLDTrapReg.all = I2caRegs.I2CDRR;
                        break;
                    default:
                        break;
                }
            }
            
            I2c_Operation = cOperation_Null;
            I2caRegs.I2CSTR.all = 0x0020; // Do not use bit writes to this register, it is Write1-Clear
            break;
        }
    }
}

*/

// ******************************************************************************************************
// *            End of I2c_Driver.c
// ******************************************************************************************************
