// ********************************************************************
// *            F28335Port.h
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
// ********************************************************************
// *    FILE NAME: F28335Port.h
// *
// *    DESCRIPTION: Header file for DSP GPIO 
// *
// *    ORIGINATOR: ASa, PP
// *
// *    DATE: 10/3/2003
// *
// *    HISTORY: See CVS history 
// ********************************************************************
#ifndef _F28335Port_H
#define _F28335Port_H

#include "Constants.h"
#include "Version.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint16_t  LoWord;     
    uint16_t  HiWord;
} GPIO_WORD;

// Output port shadow registers
typedef struct
{
    uint16_t  reservedA0           : 1;        // ePWM1A
    uint16_t  reservedA1           : 1;        // ePWM1B
    uint16_t  reservedA2           : 1;        // ePWM2A
    uint16_t  reservedA3           : 1;        // ePWM2B
    
    uint16_t  reservedA4           : 1;        // ePWM3A
    uint16_t  reservedA5           : 1;        // ePWM3B   
    uint16_t  reservedA6           : 1;        // ePWM4A
    uint16_t  reservedA7           : 1;        // ePWM4B
    
    uint16_t  reservedA8           : 1;        // ePWM5A
    uint16_t  reservedA9           : 1;        // ePWM5B
    uint16_t  reservedA10          : 1;        // ePWM6A
//	    uint16_t  reservedA11          : 1;        // ePWM6B
	uint16_t  sync_out_rsd_Low  : 1; 	   // UPM0_master:ePWM6B;  UPM1_slave:sync_out_rsd_Low
    
    uint16_t  reservedA12          : 1;        // TZ1
    uint16_t  reservedA13          : 1;        // TZ2
    uint16_t  reservedA14          : 1;        // TZ3
    uint16_t  reservedA15          : 1;        // TZ4

    uint16_t  reservedA16          : 1;        // TZ5
    uint16_t  reservedA17          : 1;        // TZ6
    uint16_t  RectPWMdisableA      : 1;        // GPO18
    uint16_t  InvPWMdisable        : 1;        // GP019            
    
    uint16_t  reservedA20          : 1;        // CanBTx
    uint16_t  reservedA21          : 1;        // CanARx
    uint16_t  NeutralRelay         : 1;        // GPO22
    uint16_t  TripBypassBreakerNot : 1;        // GPO23
    
    uint16_t  IO_Good_Set          : 1;        // GPO24
    uint16_t  reservedA25          : 1;        // PLD_EPO_Latch
    uint16_t  RectPWMdisableBC     : 1;        // GPO26
    uint16_t  reservedA27          : 1;        // aux PWM
    
    uint16_t  reservedA28          : 1;        // SciARx
    uint16_t  Feel_Good_LED        : 1;        // GPO29
    uint16_t  reservedA30          : 1;        // CanARx
    uint16_t  reservedA31          : 1;        // CanATx
} st_OutRegA;

#define RECTPWMDISABLEA_BIT             BIT_18
#define INVPWMDISABLE_BIT               BIT_19 
#define NEUTRAL_RELAY_BIT               BIT_22
#define TRIPBYPASSBREAKERNOT_BIT        BIT_23
#define IOGOODSET_BIT                   BIT_24
#define RECTPWMDISABLEBC_BIT            BIT_26
#define FEELGOODLED_BIT                 BIT_29   

// GPIOA outputs
#define OUTREGA_OUTPUT_MASK             ( RECTPWMDISABLEA_BIT | INVPWMDISABLE_BIT | NEUTRAL_RELAY_BIT | TRIPBYPASSBREAKERNOT_BIT | IOGOODSET_BIT | RECTPWMDISABLEBC_BIT | FEELGOODLED_BIT )

typedef struct
{
    uint16_t  reservedB32         : 1;        // SDAA
    uint16_t  reservedB33         : 1;        // SCLA
    uint16_t  reservedB34         : 1;        // ExtSync ECAP1
    uint16_t  reservedB35         : 1;        // SciATx
    
    uint16_t  TripBatteryBreaker  : 1;        // GPO36
    uint16_t  reservedB37         : 1;        // eCap2
    uint16_t  BattCLimReset       : 1;        // GPO38
    uint16_t  reservedB39         : 1;        // GPO39 BYP_READY
    
    uint16_t  DSP_WD              : 1;        // GPO40
    uint16_t  AD_Mux_1_2          : 2;        // AD_Mux 1, 2
    uint16_t  reservedB43         : 1;        // GPI43
    
    uint16_t  BatteryRelay1       : 1;        // GPO44
    uint16_t  BattLegB_Relay       : 1;        // GPO45
    uint16_t  reservedB46         : 1;        // PLD TDI
    uint16_t  reservedB47         : 1;        // PLD TMS
    
    uint16_t  reservedB48         : 1;        // ecap
    uint16_t  reservedB49         : 1;        // ecap, one of CNTL board revision bits(GPIO49, GPIO74 and GPIO77)
    uint16_t  InputRelay_L1       : 1;        // GPO50
    uint16_t  InputRelay_L23      : 1;        // GPO51
    
    uint16_t  InverterRelay       : 1;        // GPO52  NOTE: Use SetInverterRelay to set/clear. please.
    uint16_t  BackFeedRelay       : 1;        // GPO53
    uint16_t  reservedB54         : 1;        // SPIA SIMO
    uint16_t  reservedB55         : 1;        // SPIA SOMI
    
    uint16_t  reservedB56         : 1;        // SPIA CLK
    uint16_t  SpiCs1Dsp           : 1;        // EEPROM chip select not
    uint16_t  RtcSpiCS            : 1;        // RTC chip select
    uint16_t  BalancerRelay       : 1;        // GPO59
    
    uint16_t  SpiCs2Dsp           : 1;        // GPO60
    uint16_t  Driver_Detect       : 1;        // GPO61 Driver_Detect
    uint16_t  PullChainOut        : 1;        // GPO62
    uint16_t  reservedB63         : 1;        // GPI63
} st_OutRegB;

#define TRIPBATTERYBREAKER_BIT          BIT_4
#define BATTCLIMRESET_BIT               BIT_6
#define DSPWATCHDOG_BIT                 BIT_8
#define ADMUX12_BIT                     ( BIT_9 | BIT_10 )
#define BATTERYRELAY1_BIT               BIT_12
#define BATTLINERELAY_BIT               BIT_13
#define INPUTRELAYL1_BIT                BIT_18
#define INPUTRELAYL23_BIT               BIT_19
#define INVERTERRELAY_BIT               BIT_20
#define BACKFEEDRELAY_BIT               BIT_21
#define SPI_CS1_DSP                     BIT_25
#define REC_R_BALANCER                  BIT_26
#define BALANCERRELAY_BIT               BIT_27
#define FANCONTROL1_BIT                 BIT_28
#define Driver_Detect_BIT               BIT_29
#define PULLCHAINOUT_BIT                BIT_30

// GPIOB outputs
#define OUTREGB_OUTPUT_MASK             ( TRIPBATTERYBREAKER_BIT    |   \
                                          BATTCLIMRESET_BIT         |   \
                                          DSPWATCHDOG_BIT           |   \
                                          ADMUX12_BIT               |   \
                                          BATTERYRELAY1_BIT         |   \
                                          BATTLINERELAY_BIT         |   \
                                          INPUTRELAYL1_BIT          |   \
                                          INPUTRELAYL23_BIT         |   \
                                          INVERTERRELAY_BIT         |   \
                                          BACKFEEDRELAY_BIT         |   \
                                          SPI_CS1_DSP               |   \
                                          REC_R_BALANCER            |   \
                                          BALANCERRELAY_BIT         |   \
                                          FANCONTROL1_BIT           |   \
                                          Driver_Detect_BIT         |   \
                                          PULLCHAINOUT_BIT          )   

typedef struct
{
    uint16_t  Supply_24VOff       : 1;        // GPO64
    uint16_t  PrechargeOn         : 1;        // GPO65
    uint16_t  InvSPolarity        : 1;        // GPO66
    uint16_t  InvTPolarity        : 1;        // GPO67
    
    uint16_t  PLDBoostMode        : 1;        // GPO68
    uint16_t  GateEnable          : 1;        // GPO69
    uint16_t  ReservedC70         : 1;        // GPI70
    uint16_t  ParallelSyncOut     : 1;        // GPO71
    
    uint16_t  STSPowerFault       : 1;        // GPO72, bypass STS 24V power fault, add new NB, behavior TBD
    uint16_t  ShowTime0           : 1;        // GPO73
    uint16_t  BypassGate3         : 1;        // GPO74, one of CNTL board revision bits(GPIO49, GPIO74 and GPIO77)
    uint16_t  reservedC75         : 1;        // GPO75
    
    uint16_t  ShowTime1           : 1;        // GPO76
    uint16_t  BypassGate6         : 1;        // GPO77, one of CNTL board revision bits(GPIO49, GPIO74 and GPIO77)
    uint16_t  InvRPolarity        : 1;        // GPO78
    uint16_t  EPO_Enable          : 1;        // GPO79       
      
    uint16_t  reservedC80         : 1;        // PLD TDO
    uint16_t  reservedC81         : 1;        // PLDTCK
    uint16_t  RecRPolarity        : 1;        // GPO82
    uint16_t  reservedC83         : 1;        // GPI83 BatteryCurrentLimit
    
    uint16_t  RecSPolarity        : 1;        // GPO84
    uint16_t  RecTPolarity        : 1;        // GPO85
    uint16_t  Inv_Online          : 1;        // GPO86
    uint16_t  Byp_Avail           : 1;        // GPO87
    
    uint16_t  rsvd1               : 8;        // 88-95  reserved
} st_OutRegC;

#define SUPPLY_24VON_BIT            BIT_0
#define PRECHARGEON_BIT             BIT_1
#define PIN_INV_S_POL_BIT           BIT_2
#define PIN_INV_T_POL_BIT           BIT_3
#define PLDBOOSTMODE_BIT            BIT_4
#define GATEENABLE_BIT              BIT_5
#define PARALLELSYNCOUT_BIT         BIT_7
#define STSPOWRFAULT_BIT            BIT_8
#define BYPASSGATE3_BIT             BIT_10 
#define BYPASSGATE6_BIT             BIT_13
#define PIN_INV_R_POL_BIT           BIT_14
#define EPOENABLE_BIT               BIT_15
#define PIN_REC_R_POL_BIT           BIT_18
#define PIN_REC_S_POL_BIT           BIT_20
#define PIN_REC_T_POL_BIT           BIT_21
#define INVONLINE_BIT               BIT_22
#define BYPAVAIL_BIT                BIT_23   

// GPIOC outputs
#define OUTREGC_UPDATE_MASK         ( SUPPLY_24VON_BIT      |   \
                                      PRECHARGEON_BIT       |   \
                                      PLDBOOSTMODE_BIT      |   \
                                      GATEENABLE_BIT        |   \
                                      STSPOWRFAULT_BIT      |   \
                                      BYPASSGATE3_BIT       |   \
                                      BYPASSGATE6_BIT       |   \
                                      EPOENABLE_BIT         |   \
                                      INVONLINE_BIT         |   \
                                      BYPAVAIL_BIT)

// GPIOC outputs
#define OUTREGC_OUTPUT_MASK         ( OUTREGC_UPDATE_MASK   |   \
                                      PIN_INV_R_POL_BIT     |   \
                                      PIN_INV_S_POL_BIT     |   \
                                      PIN_INV_T_POL_BIT     |   \
                                      PIN_REC_R_POL_BIT     |   \
                                      PIN_REC_S_POL_BIT     |   \
                                      PIN_REC_T_POL_BIT )
            

typedef struct
{
    union {
        st_OutRegA   bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpoA;
    union {
        st_OutRegB   bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpoB;
    union {
        st_OutRegC   bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpoC;
} stDSPOutRegister; 

// Input port shadow registers
typedef struct
{
    uint16_t  reservedA0          : 1;        // ePWM1A
    uint16_t  reservedA1          : 1;        // ePWM1B
    uint16_t  reservedA2          : 1;        // ePWM2A
    uint16_t  reservedA3          : 1;        // ePWM2B
    
    uint16_t  reservedA4          : 1;        // ePWM3A
    uint16_t  reservedA5          : 1;        // ePWM3B   
    uint16_t  reservedA6          : 1;        // ePWM4A
    uint16_t  reservedA7          : 1;        // ePWM4B
    
    uint16_t  reservedA8          : 1;        // ePWM5A
    uint16_t  reservedA9          : 1;        // ePWM5B
    uint16_t  reservedA10         : 1;        // ePWM6A
    uint16_t  reservedA11         : 1;        // ePWM6B
    
    uint16_t  reservedA12         : 1;        // TZ1
    uint16_t  reservedA13         : 1;        // TZ2
    uint16_t  reservedA14         : 1;        // TZ3
    uint16_t  reservedA15         : 1;        // TZ4

    uint16_t  reservedA16         : 1;        // TZ5
    uint16_t  reservedA17         : 1;        // TZ6
    uint16_t  reservedA18         : 1;        // 
    uint16_t  reservedA19         : 1;        // PWM Sync Out             
    
    uint16_t  reservedA20         : 1;        // CanBTx
    uint16_t  reservedA21         : 1;        // CanARx
    uint16_t  reservedA22         : 1;        // SciBTx
    uint16_t  reservedA23         : 1;        // SciBRx
    
    uint16_t  reservedA24         : 1;        // GPO24
    uint16_t  PLD_EPO_Latch       : 1;        // GPO25
    uint16_t  reservedA26         : 1;        // aux PWM
    uint16_t  reservedA27         : 1;        // aux PWM
    
    uint16_t  reservedA28         : 1;        // SciARx
    uint16_t  reservedA29         : 1;        // GPO29
    uint16_t  reservedA30         : 1;        // CanARx
    uint16_t  reservedA31         : 1;        // CanATx
} st_InRegA;

#define PLD_EPO_LATCH_BIT       BIT_25

// GPIOA inputs
#define INREGA_INPUT_MASK    ( PLD_EPO_LATCH_BIT )

typedef struct
{
    uint16_t  I2CSDA              : 1;        // SDAA
    uint16_t  I2CSCL              : 1;        // SCLA
    uint16_t  reservedB34         : 1;        // Ecap1
    uint16_t  reservedB35         : 1;        // SciATx
    
    uint16_t  reservedB36         : 1;        // GPO36
    uint16_t  reservedB37         : 1;        // Ecap2
    uint16_t  reservedB38         : 1;        // GPO38
    uint16_t  Byp_Ready           : 1;        // BYP_READY
    
    uint16_t  reservedB40         : 1;        // GPO40
    uint16_t  reservedB41         : 1;        // GPO41
    uint16_t  reservedB42         : 1;        // GPO42
    uint16_t  reservedB43         : 1;        // GPI43
    
    uint16_t  reservedB44         : 1;        // GPO44
    uint16_t  reservedB45         : 1;        // GPO45
    uint16_t  reservedB46         : 1;        // PLD TDI
    uint16_t  reservedB47         : 1;        // PLD TMS
    
    uint16_t  reservedB48         : 1;        // ecap
    uint16_t  reservedB49         : 1;        // ecap
    uint16_t  reservedB50         : 1;        // GPO50
    uint16_t  reservedB51         : 1;        // GPO51
    
    uint16_t  reservedB52         : 1;        // GPO52
    uint16_t  reservedB53         : 1;        // GPO53
    uint16_t  reservedB54         : 1;        // SPIA SIMO
    uint16_t  reservedB55         : 1;        // SPIA SOMI
    
    uint16_t  reservedB56         : 1;        // SPIA CLK
    uint16_t  reservedB57         : 1;        // SPIA STEA
    uint16_t  reservedB58         : 1;        // GPO58
    uint16_t  reservedB59         : 1;        // GPO59
    
    uint16_t  reservedB60         : 1;        // GPO60 SpiCs2Dsp
    uint16_t  reservedB61         : 1;        // Driver_Detect
    uint16_t  reservedB62         : 1;        // GPO62
    uint16_t  PullChain           : 1;        // GPI63
} st_InRegB;

#define BYPREADY_BIT            BIT_7
//#define DRIVERDETECT_BIT        BIT_29
#define PULLCHAININ_BIT         BIT_31
// GPIOB inputs
//#define INREGB_INPUT_MASK       ( BYPREADY_BIT | PULLCHAININ_BIT )  
#define INREGB_INPUT_MASK       ( BYPREADY_BIT | PULLCHAININ_BIT )

typedef struct
{
    uint16_t  reservedC64         : 1;        // GPO64
    uint16_t  reservedC65         : 1;        // GPO65
    uint16_t  reservedC66         : 1;        // GPO66
    uint16_t  reservedC67         : 1;        // GPO67
    
    uint16_t  reservedC68         : 1;        // GPO68
    uint16_t  reservedC69         : 1;        // GPO69
    uint16_t  reservedC70         : 1;        // GPI70
    uint16_t  reservedC71         : 1;        // GPO71
    
    uint16_t  reservedC72         : 1;        // GPO72
    uint16_t  FanCheck_RevID_P3   : 1;        // GPI73, CONST_InterfaceBrdRev_ID_P3 == InterfaceBoardRevID
    uint16_t  reservedC74         : 1;        // GPO74
    uint16_t  RecFault            : 1;        // GPI75
    
    uint16_t  InvFault            : 1;        // GPI76
    uint16_t  reservedC77         : 1;        // GPO77, one of CNTL board revision bits(GPIO49, GPIO74 and GPIO77)
    uint16_t  reservedC78         : 1;        // GPO78 InvRPolarity
    uint16_t  reservedC79         : 1;        // GPO79
     
    uint16_t  reservedC80         : 1;        // PLD TDO
    uint16_t  reservedC81         : 1;        // PLDTCK
    uint16_t  reservedC82         : 1;        // GPO82 RecRPolarity
    uint16_t  BatteryCurrentLimit : 1;        // GPI83
     
    uint16_t  reservedC84         : 1;        // GPO84 RecSPolarity
    uint16_t  reservedC85         : 1;        // GPO85 RecTPolarity
    uint16_t  reservedC86         : 1;        // GPO86 Inv_Online
    uint16_t  reservedC87         : 1;        // GPO87 Byp_Avail
    
    uint16_t  rsvd1               : 8;        // 88-95  reserved
} st_InRegC;

#define EXTBATBREAKERFB_BIT      BIT_9
#define RECFAULT_BIT             BIT_11
#define INVFAULT_BIT             BIT_12
#define CURRENTLIMIT7_BIT        BIT_14
#define PLD_BATTERY_CURRENT_LIMIT   BIT_19

#define INREGC_INPUT_MASK     ( EXTBATBREAKERFB_BIT         |   \
                                RECFAULT_BIT                |   \
                                INVFAULT_BIT                |   \
                                CURRENTLIMIT7_BIT           |   \
                                PLD_BATTERY_CURRENT_LIMIT )  
    
typedef struct
{
    union {
        st_InRegA    bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpiA;
    union {
        st_InRegB    bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpiB;
    union {
        st_InRegC    bit;
        GPIO_WORD    word;
        uint32_t       all;
    }GpiC;
} stDSPInRegister; 

//old control board revision ID will be 0
#define ControlBoardOld	( (uint16_t)( 0 ) )	//Bishlant/20140304 common CNTL add
#define ControlBoardTPT29555	( (uint16_t)( 2 ) ) //update to 2 according new HW board

extern volatile stDSPOutRegister DSPOutRegister;
extern volatile stDSPInRegister  DSPInRegister;
extern uint16_t ControlBoardRevID;
void InitDSP_IORegs( void );
void WriteDSPOutputs_ISR( void );
void ReadDSPInputs( void );
void InverterPWMOn( void );
void InverterPWMOff( void );
void RectifierPWMOn( void );
void RectifierPWMOnDischargeLink( void );
void RectifierL1PWMOn( void );
void RectifierPWMOff( void );
void FanGpioConfigure(void);
//void BoostPWMOn( void );
void BoostPWMTurnOn( void );
void BoostPWMOff( void );
void ChargerPWMPosOn(void);
void ChargerPWMNegOn(void);
void ChargerPWMPosOff(void);
void ChargerPWMNegOff(void);
void ChargerPWMOn( void );
void ChargerPWMOff( void );
void SetIOGood( uint16_t good );
void ConfigureParallelSyncOut( uint16_t enable );
int16_t Read_Controlboard_RevID( void );
void BoostLegBPWMOn( void );
void BoostLegBPWMOff( void );
void ConfigBoostLegBPWMOn( void );
void ConfigBoostLegBPWMOff( void );

inline void ToggleLED( void )
{   DSPOutRegister.GpoA.bit.Feel_Good_LED = ~DSPOutRegister.GpoA.bit.Feel_Good_LED;     }
inline void ToggleDSPWatchDog( void )
{   DSPOutRegister.GpoB.bit.DSP_WD = ~DSPOutRegister.GpoB.bit.DSP_WD;                   }
inline void PreChargeOn( void )
{
    if (CONST_InterfaceBrdRev_ID_P3 == InterfaceBoardRevID)
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 0;
    }
    else if(CONST_InterfaceBrdRev_ID_P6 == InterfaceBoardRevID)
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 1;
    }
    else
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 1;
    }
}
inline void PreChargeOff( void )
{
    if (CONST_InterfaceBrdRev_ID_P3 == InterfaceBoardRevID)
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 1;
    }
    else if(CONST_InterfaceBrdRev_ID_P6 == InterfaceBoardRevID)
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 0;
    }
    else
    {
    	DSPOutRegister.GpoC.bit.PrechargeOn = 0;
    }
}
inline void MasterPWMOn( void )
{   DSPOutRegister.GpoC.bit.GateEnable = 0;                                             }
inline void MasterPWMOff( void )
{   DSPOutRegister.GpoC.bit.GateEnable = 1;                                             }
inline void PIN_INV_R_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO78 = 1;                                                 }
inline void PIN_INV_R_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO78 = 1;                                               }
inline void PIN_INV_S_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO66 = 1;                                                 }
inline void PIN_INV_S_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;                                               }
inline void PIN_INV_T_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO67 = 1;                                                 }
inline void PIN_INV_T_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;                                               }
inline void PIN_RECT_R_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO82 = 1;                                                 }
inline void PIN_RECT_R_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO82 = 1;                                               }
inline void PIN_RECT_S_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO84 = 1;                                                 }
inline void PIN_RECT_S_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;                                               }
inline void PIN_RECT_T_POL_to1( void )
{   GpioDataRegs.GPCSET.bit.GPIO85 = 1;                                                 }
inline void PIN_RECT_T_POL_to0( void )
{   GpioDataRegs.GPCCLEAR.bit.GPIO85 = 1;                                               }
inline void SetExtSyncHigh( void )
{   DSPOutRegister.GpoC.bit.ParallelSyncOut = 1;                                        }
inline void SetExtSyncLow( void )
{   DSPOutRegister.GpoC.bit.ParallelSyncOut = 0;                                        }
inline void OpenInputRelays( void )
{
    DSPOutRegister.GpoB.bit.InputRelay_L1 = 0;
    DSPOutRegister.GpoB.bit.InputRelay_L23 = 0;
}
inline void CloseInputRelays( void )
{
    DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
    DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
}

inline void ActivePullChain( void )
{
    GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;
    DSPOutRegister.GpoB.bit.PullChainOut = 0;
}
inline void DeActivePullChain( void )
{
    GpioDataRegs.GPBSET.bit.GPIO62 = 1;
    DSPOutRegister.GpoB.bit.PullChainOut = 1;
}
 
#ifdef __cplusplus
// Note: Defined in PeriodicTSKs.cpp
void WriteDSPOutputs_TSK( void);
}
#endif

// ********************************************************************
// *        END OF PLD.H
// ********************************************************************
#endif

