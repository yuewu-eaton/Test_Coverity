//###########################################################################
// $TI Release: DSP281x C/C++ Header Files V1.20 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "Version.h"
#include "SystemTrap.h"
#include "Coefficients.h"

extern void InitCAN(void);
extern void InitI2c(void);
extern void InitInv(void);
extern void InitBattConv(void);
extern void InitWatchdog(void);
extern void NB_Init( void );
extern void RTC_InitSysTime(void);
extern void InitEepromOutputkVARating( void );
extern void InitEepromSystemType( void );
extern void InitSPI(void);
extern void InitRec(void);
extern void IdentifyModuleByHWID(void);

void TRCdata_Init(void);


#if (__TI_COMPILER_VERSION__ != 5002010)
#error "Code not compiled with the correct Code Generation Tools (5.2.10)"
#endif

int main(void)
{
    // These are defined by the linker
    extern uint16_t hwi_vec_loadstart;
    extern uint16_t hwi_vec_loadend;
    extern uint16_t hwi_vec_runstart;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
    InitSysCtrl();
   
// Step 2. Subsumed by assembly startup code.

// Step 3. Initalize GPIO: 
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
    InitGpio();
    // initialize project IO
    InitDSP_IORegs();

// Step 4. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
    DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2833x_PieCtrl.c file.
    InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP281x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
    EALLOW;
    memcpy(&hwi_vec_runstart, &hwi_vec_loadstart, &hwi_vec_loadend - &hwi_vec_loadstart);
    EDIS;

// Step 5. Initialize all the Device Peripherals:
// This function is found in DSP281x_InitPeripherals.c
//   InitPeripherals();
    InitCpuTimers();
    StartCpuTimer0();
    RTC_InitSysTime();
     
// Step 6. User specific code, enable interrupts:

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
    InitFlash();

// Project initialization
    //Get control board revision ID
    ControlBoardRevID = Read_Controlboard_RevID();

    InitSPI();                          //add to enable spi before read eep
    InitEepromSystemType();             //add to distinguish system
    InitParameters();                   //add to initialize global varibles and control coefficients
    InitAdc();
    InitECap();    
    InitEPwm();
    InitI2c();
    IdentifyModuleByHWID();
    
    InitRec();
    InitInv();
    InitBattConv();
    MasterPWMOff();
    InitVersionNumber();
    NB_Init();

// Set wanted HWI IER bits
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx8 = 1;
    
    IER |= ( M_INT1 | M_INT2 | M_INT7 | M_INT8 );       // Enable CPU Interrupts
    
    InitWatchdog();
        
    // start timers
    StartePWMTimers();

// Step 6. Exit to DSP/BIOS
    return 0;
}

void TRCdata_Init(void)
{
    // These are defined by the linker
    extern uint16_t trcdata_loadstart;
    extern uint16_t trcdata_loadend;
    extern uint16_t trcdata_runstart;
    // Initialize .trcdata section before main() (See TI::spra958)
    memcpy(&trcdata_runstart, &trcdata_loadstart, &trcdata_loadend - &trcdata_loadstart);

}


//===========================================================================
// No more.
//===========================================================================
