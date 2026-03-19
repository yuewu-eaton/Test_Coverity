// ********************************************************************************************************
// *    BatteryStateControl.h
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// * Copyright (c) 2003 Eaton
// * ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *   FILE NAME:   BatteryStateControl.h
// *
// *   DESCRIPTION: State machine for charger and boost.
// *
// *   ORIGINATORS: Pasi Pulkkinen
// *
// *   DATE:        23.12.2003
// *
// *   HISTORY:     See CVS history.
// *********************************************************************************************************
#ifndef BATTERYSTATE_H_
#define BATTERYSTATE_H_

// *********************************************************************************************************
// *        INCLUDE FILES  (included files must have #ifndef protection)
// *********************************************************************************************************
#include "DSP28x_Project.h" 
#include "Adc.h"
#include "F28335Port.h"
#include "MCUState.h"
#include "BatteryConverterControl.h"
#include "StateTimer.h"
#include "RectifierStateControl.h"
#include "BTR.h"

// *********************************************************************************************************
// *        Function Prototypes
// *********************************************************************************************************


// *********************************************************************************************************
// *        Public Data
// *********************************************************************************************************


#define BATTERY_BALANCE_MINIMUMPWM          3675 // Fixed PWM for relay balancer 
#define BATTERY_BALANCE_CHRGPWM             3500 // Fixed PWM for relay balancer 
#define BATTERY_BALANCE_BOOSTPWM            3500 // Fixed PWM for relay balancer 

#define LOW_BATT_SHUTDOWN_DELAY             60   // 60 secs.: for XCP, and shutdown iminent alarms

// Substate for battery start
const uint16_t SUBSTATE_TO_BAT_START_PRECHARGE_DC  = 10;
/*** DEFINE THE MACHINE STATES (used in BoChMachineState) ****/
enum battery_states_t
{
    BATTERY_INIT_STATE         =           0,
    BATTERY_SHUTDOWN_STATE     =           1,
    BATTERY_CHARGE_STATE       =           2,
    BATTERY_ON_BOOST_STATE     =           3,
    BATTERY_TEST_STATE         =           4,
    BATTERY_TRIP_STATE         =           5
};

#define BATT_VOLT_MIN                       0.0
#define BATT_RELAY_CLOSE_MARGIN             50.0
#define BATT_CB_OPEN_MARGIN                 20.0
#define BATTVOLTBACKUP_MARGIN               15.0   // 30.0
#define CHRGLIMIT                           10.00  // current limit 10 amps
#define BATT_HIGH_STOP_LIMIT                500.0  // 500 volts to stop charger
#define CHARGE_ENABLE_LIMIT                 450.0  // enable charger if stopped and voltage lower than 450 volts
#define LOW_BATT_SHUTDOWN_COUNTER           (LOW_BATT_SHUTDOWN_DELAY*2)  // (sec/2) Shutdown timer after low battery shutdown alarm
#define BATT_NEG_FAIL_LIMIT                 -100.0 // 100V Negative battery voltage limit where we give alarm
#define MINIMUM_BOOST_TIME                  3     // s Minimum time to stay on boost state --> Should be greater than minimum on battery time before bypass and minimum time that triggers alarm

#define BATT_DISCONNECT_VPC  1.574  //1.574 = 170V/108cells


// Time constants, based on 231us task.
#define BATT_ONE_SECOND                     4329L
#define BATT_100MSECOND                     433L
#define BATT_40MSECOND                      174L
#define BATT_20MSECOND                      87L
#define BATT_10MSECOND                      44L
#define BATT_5MSECOND                       22L

#define BATT_TEST_ONE_SECOND    			433//510 change to 450 -4.333kHz

#define CHARGER_MARGIN                      20      // 2A cushion 


extern float BatUVWrnLimVPC;            // EEP
extern float BatUVShtLimVPC;            // EEP
extern float BatUVMinLimVPC;            // EEP
extern float BatteryOVLevelVPC;         // EEP
extern float KeepAlive_BatUVPC;         // EEP
extern uint16_t OnBatteryAlarmDelay;    // EEP        
extern uint16_t BatOVTime;              // EEP
extern uint16_t ChargerOVCounter;     
extern uint16_t BatteryBreakerCounter;  // Pan/20120914,add
extern uint16_t BatteryBreakerFeedbackCounter;  // Pan/20120919,add

// Pan/20121011 add begin
enum eBatteryBreakerState
{
    BAT_BRK_OPENED_STATE    =   0,
    BAT_BRK_CLOSED_STATE    =   1,
    BAT_BRK_TRIP_STATE      =   2,
    BAT_BRK_FAIL_STATE      =   3    
};
// Pan/20121011 end

typedef struct
{
	// Word 0
    uint16_t BatteryLow             : 1;
    uint16_t ShutdownTimerActive    : 1;
    uint16_t BatteryTestCmd         : 1; 
    uint16_t BattUVShutDown         : 1;
    
    uint16_t UVShutdownSync         : 1;
    uint16_t BattDisconnected       : 1;
    uint16_t OnBatteryStatus        : 1; 
    uint16_t BattLowAlarmEnabled    : 1;
    
    uint16_t ChrgRelCmdOn           : 1;
    uint16_t BattNegFail            : 1;
    uint16_t BoostCmdOn             : 1; 
    uint16_t ChargerFailed          : 1;    

    uint16_t EnableBattery          : 1;
    uint16_t ChrgCmdOn              : 1;
    uint16_t BattCmsgOn             : 1;
    uint16_t BattTestOn             : 1;    
    
    // Word 1
    uint16_t BattOVTripDone         : 1;
    uint16_t BattRelayFailToClose   : 1;
    uint16_t BattRelayFailToOpen    : 1;
    //uint16_t /*unallocated*/        : 1;
    uint16_t BattBreakerTrip        : 1;     // Pan/20120914,add status bit for battery breaker
    
    uint16_t BattPrechargeVHigh     : 1;
    uint16_t BattPrechargeVLow      : 1;   
    uint16_t BattFaultPos           : 1;
    uint16_t BattFaultNeg           : 1;
    
    uint16_t AutoZeroBatteryCurrentSensors  : 1;
    //uint16_t  /* unallocated */             : 1;    
    uint16_t BattBreakerFeedbackOpen        : 1;	// Pan/20120919,add flag for battery breaker feedback
    uint16_t BatteryTestCancel              : 1;
    uint16_t ChargerFailOV                  : 1;    
       
//	    uint16_t unused28                       : 1;
	uint16_t UpmsToBatTogether				: 1;
    uint16_t StartupCommand  				: 1;
    uint16_t BatteryStartPrecharging 		: 1;
    uint16_t CancelBatteryStart             : 1;
    
    uint16_t word2placeholder;
    uint16_t /*unallocated word 3 */: 12;
    uint16_t SequenceNumber : 4;
            
} stBatteryStateStatus;

typedef union
{
    stBatteryStateStatus bit;
    uint32_t           all;
    uint16_t		words[sizeof(stBatteryStateStatus)];
} uBatteryStateStatus;   


class BatteryStateControl : public BatteryConverterControl
{
    public:
        BatteryStateControl( void ) : BatteryConverterControl()
        {
            BoChMachineState = BATTERY_INIT_STATE;
            ChargerPhState = 0;
            BatteryStatus.all = 0;
            MinBoostTimeBatt = (50 * BATT_100MSECOND);
            MaxRelayBreakCurrent = 10.0;  //current should be below 10 amps before opening batt relay.
            
            //RailBoostLim = 189.0;       // calculated based on rail setpoint and EEP margin
            RailBoostLim = 190.0 - 17.0;
                        
            BatteryLowCounter = 1;
            BatteryOVDelayCntr = 120;
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = true;
            
            OnBatteryAlarmDelayCntr = 0;
            BoostFailTimer = 0;
            LineEventCounter = 0;
            BatteryTimer1.ClearTimer();
            BatteryTimer2.ClearTimer();
            BatteryTimer3.ClearTimer();
        }
        ~BatteryStateControl( void )
        {
        }
        
    public:
        void RunBatteryState( void );
        void DebounceCharger( void );
        void SetBatteryRelay( uint16_t state );
        void ClearBatteryAlarms( void );
        bool CheckBatteryUVShutDown( void );
        void CheckBatteryBreaker( void );   // Pan/20120914 ,add

        battery_states_t GetBatteryState( void )
        {
            return BoChMachineState;
        }
        
        const uBatteryStateStatus& GetBatteryStateStatus( void )
        {
            return BatteryStatus;
        }
        
        volatile const uBatteryStateStatus * GetBatteryStateStatusPtr( void ) const
        {
            return &BatteryStatus;
        }
        
        void BoostCmdOn( void )
        {
           BatteryStatus.bit.BoostCmdOn = 1;   
        }
        
        void BoostCmdOff( void )
        {
           BatteryStatus.bit.BoostCmdOn = 0;
        }
        
        void ChrgRelayCmdOn( void )
        {
            BatteryStatus.bit.ChrgRelCmdOn = 1;
            BatteryStatus.bit.EnableBattery = 1;  //temporary
        }
        
        void ChrgRelayCmdOff( void )
        {
            BatteryStatus.bit.ChrgRelCmdOn = 0;
        }
        
        void ChargerCmdOn( void )
        {
            BatteryStatus.bit.ChrgCmdOn = 1;
        }
        
        void ChargerCmdOff( void )
        {
            BatteryStatus.bit.ChrgCmdOn = 0;            
        }
        
        void StartupBatteryConverter( void )
        {
            BatteryStatus.bit.StartupCommand = 1;
            BatteryStatus.bit.CancelBatteryStart = 0;
        }
        
        void CancelBatteryStart( void )
        {
            BatteryStatus.bit.StartupCommand = 0;
            BatteryStatus.bit.CancelBatteryStart = 1;
        }
        
        void BatteryTestCmdOn( void )
        {
            if ( ( BoChMachineState == BATTERY_SHUTDOWN_STATE ) || 
                ( BoChMachineState == BATTERY_CHARGE_STATE )
               )
            {
                BatteryStatus.bit.BatteryTestCmd = 1;
            }  
        }
        
        void CancelBatteryTest( void )
        {
            BatteryStatus.bit.BatteryTestCancel = 1;  
        }
        
        DualStageBatteryTestInfo GetBatteryTestResults( void )
        {
            return BatteryTestResults;
        }
        void SetChargerFailed( void )
        {
            BatteryStatus.bit.ChargerFailed = 1;
        }
        
        bool BatteryAvailable( void )
        {
            return ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
                     NB_GetNodebit(UPM_NB_BATTERY_INSTALLED)         &&
                     ( BoChMachineState != BATTERY_INIT_STATE ) );
        }

		void SetUpmsToBatTogether( bool bit)
		{
			BatteryStatus.bit.UpmsToBatTogether = bit;
		}

        bool GetUpmsToBatTogether( void )
        {
			return BatteryStatus.bit.UpmsToBatTogether;
        }

    protected:
        void BatteryInitState( void );
        inline void BatteryBlockState(void);
		void BatteryShutdownState( void );
        inline void BatteryOnBoostState( void );
        inline void BatteryTestState( void );
        inline void BatteryOnChargeState( void );
		void BatteryOnCBTripState(void);
        void BatteryTransferState( battery_states_t new_state );
 

    public:        
        uBatteryStateStatus BatteryStatus;
        battery_states_t BoChMachineState;
        uint16_t ChargerPhState; 
		uint16_t PreChargePhState;
        uint16_t BatTestAbandonReason;
        float RailBoostLim;    
        float MaxRelayBreakCurrent;
        eBatteryBreakerState BatBreakerState;       // Pan/20121011 add
        bool CheckBatteryShutdown( void );
        
        uint32_t BatteryTest1Time;
        uint32_t BatteryTest2Time;

     protected:       
        uint16_t BatteryLowCounter;
        uint32_t MinBoostTimeBatt;
        uint16_t BattTestDownSampleCount;
        DualStageBatteryTestInfo    BatteryTestResults;

        StateTimer BatteryTimer1;
        StateTimer BatteryTimer2;
        StateTimer BatteryTimer3;

        uint16_t OnBatteryAlarmDelayCntr;

        
        uint16_t BatteryOVDelayCntr;
        uint16_t BoostFailTimer;
        
        uint16_t LineEventCounter;
        uRectifierStatus    rectStatus;
        rectifier_states_t  rectState;
        uint16_t K2OpenDelayTimer;
};

extern BatteryStateControl BatteryConverter;

// ********************************************************************************************************
// *            END OF BatteryStateControl.h
// ********************************************************************************************************
#endif /*BATTERYSTATE_H_*/
