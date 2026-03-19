// ********************************************************************
// *            Pld.c
// ********************************************************************
// ********************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************
// *
// *  Copyright (c) 2007 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************
// *    FILE NAME: Pld.c
// *
// *    DESCRIPTION: Handles GPIO between DSP and PLD
// *
// *    ORIGINATOR: ASa, PP
// *
// *    DATE: 4/29/2003
// *
// *    HISTORY: See CVS history
// ********************************************************************

// ********************************************************************
// * INCLUDE FILES
// ********************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "Constants.h"
#include "Coefficients.h"


// ********************************************************************
// * FILE VARIABLES (global)
// ********************************************************************

volatile stDSPOutRegister DSPOutRegister;
volatile stDSPInRegister  DSPInRegister;
uint16_t ControlBoardRevID = 0;	// common CNTL add
// * Local variables
extern uint16_t BoostLegBOn;


void ConfigChargePWM( void );
void ConfigBoostPWM( void );

void InitDSP_IORegs( void )
{
    // InitGpio in DSP2833c_Gpio.c has already initialized everything to inputs
    // just change the IOs that we need to outputs and we're done

    DSPOutRegister.GpoA.all = ( IOGOODSET_BIT | TRIPBYPASSBREAKERNOT_BIT);   // IO GOOD SET high = IO GOOD low
    DSPOutRegister.GpoB.all = PULLCHAINOUT_BIT; // Pull-Chain out; high = inactive
    DSPOutRegister.GpoC.all = 0; //SUPPLY_24VON_BIT;
    
    WriteDSPOutputs_ISR();

    EALLOW;

    GpioCtrlRegs.GPADIR.all = OUTREGA_OUTPUT_MASK;
    GpioCtrlRegs.GPBDIR.all = OUTREGB_OUTPUT_MASK;
    GpioCtrlRegs.GPCDIR.all = OUTREGC_OUTPUT_MASK;

    EDIS;

    DSPInRegister.GpiA.all = 0;
    DSPInRegister.GpiB.all = 0;
    DSPInRegister.GpiC.all = 0;
    
    InverterPWMOff();
    RectifierPWMOff();
    FanGpioConfigure();
}
        
// ********************************************************************
// *
// * Function:  WriteDSPOutputs_ISR( void ) ;
// *
// * Purpose:   Write RAM copy of GPIO output registers to actual ports
// *
// * Parms Passed   : None
// * Returns    : Nothing
// *
// *
// * Description: Executed once per ADC ISR
// *
// ********************************************************************
#pragma CODE_SECTION( WriteDSPOutputs_ISR, "ramfuncs" );
void WriteDSPOutputs_ISR( void )
{
    //
    // * This function runs from DMA ISR, no BIOS calls allowed
    //
    
    uint32_t regValue;

    regValue = DSPOutRegister.GpoA.all;
    GpioDataRegs.GPASET.all = regValue;
    regValue = ~DSPOutRegister.GpoA.all;
    GpioDataRegs.GPACLEAR.all = regValue;

    regValue = DSPOutRegister.GpoB.all;
    GpioDataRegs.GPBSET.all = regValue;
    regValue = ~DSPOutRegister.GpoB.all;
    GpioDataRegs.GPBCLEAR.all = regValue;

    regValue = DSPOutRegister.GpoC.all;
    regValue &= OUTREGC_UPDATE_MASK;//can't rewrite PWM polarity
    GpioDataRegs.GPCSET.all = regValue;
    regValue = ~DSPOutRegister.GpoC.all;
    regValue &= OUTREGC_UPDATE_MASK;//can't rewrite PWM polarity
    GpioDataRegs.GPCCLEAR.all = regValue;

}


// ********************************************************************
// *
// * Function:  ReadDSPInputs( void ) ;
// *
// * Purpose:   Update RAM copy of GPIO input registers
// *
// * Parms Passed   : None
// * Returns    : Nothing
// *
// *
// * Description:
// *
// ********************************************************************
#pragma CODE_SECTION( ReadDSPInputs, "ramfuncs" );
void ReadDSPInputs( void )
{
    uint32_t regValue;

    regValue = GpioDataRegs.GPADAT.all;
    // regValue &= INREGA_INPUT_MASK;
    DSPInRegister.GpiA.all = regValue;

    regValue = GpioDataRegs.GPBDAT.all;
    // regValue &= INREGB_INPUT_MASK;
    DSPInRegister.GpiB.all = regValue;

    regValue = GpioDataRegs.GPCDAT.all;
    // regValue &= INREGC_INPUT_MASK;
    DSPInRegister.GpiC.all = regValue;
}


// ******************************************************************************************************
// *
// * Function:      InverterOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void InverterPWMOn(void)
{
    DSPOutRegister.GpoA.bit.InvPWMdisable = 0;
    MasterPWMOn();
    EALLOW;     
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;     // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;     // Configure GPIO9 as ePWM5B
    EDIS;

}

// ******************************************************************************************************
// *
// * Function:      InverterPWMOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void InverterPWMOff(void)
{
    DSPOutRegister.GpoA.bit.InvPWMdisable = 1;
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // Configure GPIO0 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // Configure GPIO8 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;   // Configure GPIO9 as input
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      RectifierPWMOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void RectifierPWMOn(void)
{
    // configure for rectifier mode. a pha(reset with bat unbalance), b/c not need
    EPwm1Regs.AQCTLB.all = 0;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    DSPOutRegister.GpoA.bit.RectPWMdisableA = 0;
    DSPOutRegister.GpoA.bit.RectPWMdisableBC = 0;   
    MasterPWMOn();
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      RectifierPWMOnDischargeLink(void);
// *
// * Description:   Enables the gating of the rectifier IGBT's for dischage link voltage when on battery mode.
// *
// ******************************************************************************************************

void RectifierPWMOnDischargeLink(void)
{
    // configure for rectifier mode
    EPwm1Regs.AQCTLB.all = 0;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;
    
    DSPOutRegister.GpoA.bit.RectPWMdisableA = 0;
    DSPOutRegister.GpoA.bit.RectPWMdisableBC = 0;
    MasterPWMOn();
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM2B
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      RectifierL1PWMOn(void);
// *
// * Description:   Enables the gating of the L1 rectifier IGBT's
// *
// ******************************************************************************************************
void RectifierL1PWMOn( void )
{
    // configure for balancer mode
    EPwm1Regs.AQCTLB.all = 0;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.PRD = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;
    
    DSPOutRegister.GpoA.bit.RectPWMdisableA = 0;
    MasterPWMOn();
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      RectifierPWMOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void RectifierPWMOff(void)
{
    // configure for rectifier mode
    EPwm1Regs.AQCTLB.all = 0;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;
    EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;

    DSPOutRegister.GpoA.bit.RectPWMdisableA = 1;
    DSPOutRegister.GpoA.bit.RectPWMdisableBC = 1; 
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;   // Configure GPIO1 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as input
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // Configure GPIO5 as input
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      BoostPWMOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void BoostPWMTurnOn( void )
{
    ConfigBoostPWM();
    DSPOutRegister.GpoC.bit.PLDBoostMode = FALSE;
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;  // Configure GPIO2 as ePWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;  // Configure GPIO3 as ePWM2B
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      BoostPWMOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void BoostPWMOff(void)
{
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // Configure GPIO2 as IO
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;   // Configure GPIO3 as IO
    EDIS;
}

// ******************************************************************************************************
// *
// * Function:      BoostLegBPWMOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void BoostLegBPWMOn(void)
{
	DSPOutRegister.GpoA.bit.RectPWMdisableBC = 0;
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	EDIS;
	PIN_RECT_S_POL_to1(); //S.T2,pos T2 on/off,T3 off
	PIN_RECT_T_POL_to0(); //T.T3,neg T3 on/off,T2 off
	BoostLegBOn = 1;
}

// ******************************************************************************************************
// *
// * Function:      BoostLegBPWMOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void BoostLegBPWMOff(void)
{
	DSPOutRegister.GpoA.bit.RectPWMdisableBC = 1;
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as IO
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;   // Configure GPIO5 as IO
	EDIS;
	PIN_RECT_S_POL_to0();
	PIN_RECT_T_POL_to0();
	BoostLegBOn = 0;	
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMPosOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMPosOn(void)
{
     ConfigChargePWM();
     DSPOutRegister.GpoC.bit.PLDBoostMode = TRUE;
     
     EALLOW;
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;            // Configure GPIO2 as ePWM2A
//     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;            // Configure GPIO3 as ePWM2B
     EDIS;
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMNegOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMNegOn(void)
{
     ConfigChargePWM();
     DSPOutRegister.GpoC.bit.PLDBoostMode = TRUE;
     
     EALLOW;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;            // Configure GPIO2 as ePWM2A
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;            // Configure GPIO3 as ePWM2B
     EDIS;
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMPosOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMPosOff(void)
{
     EALLOW;
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;    // Configure GPIO2 as input
//     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;    // Configure GPIO3 as input
     EDIS;
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMNegOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMNegOff(void)
{
     EALLOW;
//     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;    // Configure GPIO2 as input
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;    // Configure GPIO3 as input
     EDIS;
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMOn(void);
// *
// * Description:   Enables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMOn(void)
{
     ConfigChargePWM();
     DSPOutRegister.GpoC.bit.PLDBoostMode = TRUE;
     
     EALLOW;
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;            // Configure GPIO2 as ePWM2A
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;            // Configure GPIO3 as ePWM2B
     EDIS;
}

// ******************************************************************************************************
// *
// * Function:      ChargerPWMOff(void);
// *
// * Description:   Disables the gating of the IGBT's
// *
// ******************************************************************************************************
void ChargerPWMOff(void)
{
     EALLOW;
     GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;    // Configure GPIO2 as input
     GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;    // Configure GPIO3 as input
     EDIS;
}


// ***********************************************************************
// *
// *    FUNCTION: ConfigChargePWM 
// *
// *    DESCRIPTION: Sets up PWM for charge freq. non interleaved, 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ConfigChargePWM( void )
{
    EPwm2Regs.TBPRD = ( CPUFrequency / ChargeFrequency ) / 2L;

    // ePWM6 already set to charge frequency
}

// ***********************************************************************
// *
// *    FUNCTION: ConfigBoostPWM 
// *
// *    DESCRIPTION: Sets up PWM for boost freq. non interleaved, 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ConfigBoostPWM( void )
{
    EPwm2Regs.TBPRD = ( CPUFrequency / BoostFrequency ) / 2L;

}

// ***********************************************************************
// *
// *    FUNCTION: ConfigBoostLegBPWM
// *
// *    DESCRIPTION: Sets up PWM for boost interleaved
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void ConfigBoostLegBPWMOn( void )
{
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;

	EPwm3Regs.TBCTL.bit.PHSDIR = TB_DOWN;
	EPwm3Regs.TBPHS.half.TBPHS = 4165;
	EPwm3Regs.TBPRD = ( CPUFrequency / BoostFrequency ) / 2L; //18K

	EPwm3Regs.CMPA.half.CMPA = 0;
	EPwm3Regs.CMPB = 0;	
}

// ***********************************************************************
// *
// *    FUNCTION: ConfigBoostLegBPWMOff
// *
// *    DESCRIPTION: Sets up PWM for rectifier S/T
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void ConfigBoostLegBPWMOff( void )
{
	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;       // Pass through

	EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;
	EPwm3Regs.TBPHS.half.TBPHS = 0;
	EPwm3Regs.TBPRD = ( CPUFrequency / PWMFrequency ) / 2L; //26K
}

// ***********************************************************************
// *
// *    FUNCTION: SetIOGood 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SetIOGood( uint16_t good )
{
    if ( good )
    {
        DSPOutRegister.GpoA.bit.IO_Good_Set = 0;
    }
    else
    {
        DSPOutRegister.GpoA.bit.IO_Good_Set = 1;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: InitHWSync 
// *
// *    DESCRIPTION: Only 1 pin on parallel board, only master can drive
// *                 everyone else configures as input
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ConfigureParallelSyncOut( uint16_t enable )
{
    if ( enable )
    {
        EALLOW;
        GpioCtrlRegs.GPCDIR.all |= PARALLELSYNCOUT_BIT;
        EDIS;
    }
    else
    {
        EALLOW;
        GpioCtrlRegs.GPCDIR.all |= PARALLELSYNCOUT_BIT;
        DSPOutRegister.GpoC.bit.ParallelSyncOut = 1;
        EDIS;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: Read_Controlboard_RevID
// *
// *    DESCRIPTION: Read controlboard revision ID. Return original GPIO settings after ID is read
// *
// *    ARGUMENTS:
// *
// *    RETURNS: Revision ID
// *
// ***********************************************************************
int16_t Read_Controlboard_RevID( void )
{
    int16_t CtrlBoardID = 0;
    int16_t tempMuxGPIO49 = 0;
    int16_t tempMuxGPIO74 = 0;
    int16_t tempMuxGPIO77 = 0;
    int16_t tempDirGPIO49 = 0;
    int16_t tempDirGPIO74 = 0;
    int16_t tempDirGPIO77 = 0;
    int16_t tempPudGPIO49 = 0;
    int16_t tempPudGPIO74 = 0;
    int16_t tempPudGPIO77 = 0;

    EALLOW;
    // Preserve original settings
    tempMuxGPIO49 = GpioCtrlRegs.GPBMUX2.bit.GPIO49;
    tempMuxGPIO74 = GpioCtrlRegs.GPCMUX1.bit.GPIO74;
    tempMuxGPIO77 = GpioCtrlRegs.GPCMUX1.bit.GPIO77;

    tempDirGPIO49 = GpioCtrlRegs.GPBDIR.bit.GPIO49;
    tempDirGPIO74 = GpioCtrlRegs.GPCDIR.bit.GPIO74;
    tempDirGPIO77 = GpioCtrlRegs.GPCDIR.bit.GPIO77;

    tempPudGPIO49 = GpioCtrlRegs.GPBPUD.bit.GPIO49;
    tempPudGPIO74 = GpioCtrlRegs.GPCPUD.bit.GPIO74;
    tempPudGPIO77 = GpioCtrlRegs.GPCPUD.bit.GPIO77;

    // Set ID pins to GPIO input mode with PUs disabled
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 0;  // Configure GPIO49 as IO
    GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0;  // Configure GPIO74 as IO
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0;  // Configure GPIO77 as IO

    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 1;// output
    GpioCtrlRegs.GPCDIR.bit.GPIO74 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 1;//Pullup's disabled
	GpioCtrlRegs.GPCPUD.bit.GPIO74 = 1;
	GpioCtrlRegs.GPCPUD.bit.GPIO77 = 1;

	GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1;//set output 0 to clear IO pull up status
	GpioDataRegs.GPCCLEAR.bit.GPIO74 = 1;
	GpioDataRegs.GPCCLEAR.bit.GPIO77 = 1;

    GpioCtrlRegs.GPBDIR.bit.GPIO49 = 0;// input
    GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0;
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;

    EDIS;

    DSP28x_usDelay(10);	//add delay, at least 3us delay

    CtrlBoardID = GpioDataRegs.GPBDAT.bit.GPIO49 |
                  (GpioDataRegs.GPCDAT.bit.GPIO74 << 1) |
                  (GpioDataRegs.GPCDAT.bit.GPIO77 << 2) ;
    //pulling up line sets "bit".
    CtrlBoardID =  CtrlBoardID & 7 ;  //3 lowest bits are used.

    EALLOW;
    // Return original settings
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = tempPudGPIO49;	//pullup
    GpioCtrlRegs.GPCPUD.bit.GPIO74 = tempPudGPIO74;
    GpioCtrlRegs.GPCPUD.bit.GPIO77 = tempPudGPIO77;

    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = tempMuxGPIO49; //IO
    GpioCtrlRegs.GPCMUX1.bit.GPIO74 = tempMuxGPIO74;
    GpioCtrlRegs.GPCMUX1.bit.GPIO77 = tempMuxGPIO77;

    GpioCtrlRegs.GPBDIR.bit.GPIO49 = tempDirGPIO49; //DIR
    GpioCtrlRegs.GPCDIR.bit.GPIO74 = tempDirGPIO74;
    GpioCtrlRegs.GPCDIR.bit.GPIO77 = tempDirGPIO77;
    EDIS;

    return CtrlBoardID;
}

// ***********************************************************************
// *
// *    FUNCTION: FanGpioConfigure
// *
// *    DESCRIPTION: Fan fault detection Gpio initialization
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void FanGpioConfigure(void)
{
	GpioCtrlRegs.GPCPUD.bit.GPIO73 = 0;    //pullup
	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0;   //Configure GPIO73 as IO
	GpioCtrlRegs.GPCDIR.bit.GPIO73 = 0;    //input
}

// ********************************************************************
// *        End of Pld.c
// ********************************************************************
