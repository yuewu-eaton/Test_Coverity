#ifndef _MCUSTATE_H
#define _MCUSTATE_H
// ********************************************************************************************************
// *                 MCUState.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME:   MCUState.h
// *
// *    DESCRIPTION:                   
// *
// *    ORIGINATOR:  Pasi Pulkkinen, Tuomo Kaikkonen
// *
// *    DATE:        2010/06/22
// *
// *    HISTORY:     See SVN history
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "CriticalSection.h"
#include "DebuggerBlocks.h"
#include "F28335Port.h"
#include "Eeprom_Map.h"
#include "StateTimer.h"
#include "ArmedTrigger.h"
#include "FilteredBit.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

/*** DEFINE THE MACHINE STATES ****/
enum eMCUState
{
    INITIALIZATION_STATE         =           0,
    SHUTDOWN_STATE               =           1,
    STANDBY_STATE                =           2,
    ONLINE_STATE                 =           3,
    BYPASS_STATE                 =           4,
    BYPASS_STANDBY_STATE         =           5,
    ESS_MODE_STATE               =           6,
    EASY_CAPACITY_TEST_STATE     =           7
};

// Time constants, based on 231us(4.333k) task.
#define WAIT_400_uS                         2L
#define WAIT_600_uS                         3L
#define WAIT_1_MS                           4L
#define WAIT_2_MS                           9L
#define WAIT_3_MS                           13L
#define WAIT_5_MS                           22L
#define WAIT_8_MS                           35L
#define WAIT_10_MS                          44L
#define WAIT_15_MS                          65L
#define WAIT_20_MS                          87L
#define WAIT_25_MS                          108L
#define WAIT_30_MS                          130L
#define WAIT_40_MS                          174L
#define WAIT_50_MS                          216L
#define WAIT_60_MS                          258L
#define WAIT_100_MS                         433L
#define WAIT_150_MS                         650L
#define WAIT_200_MS                         866L
#define WAIT_300_MS                         1299L
#define WAIT_500_MS                         2166L
#define ONE_SECOND                          4329L
#define FIVE_SECONDS                        21645L
#define TEN_SECONDS                         43290L
#define TWENTY_SECONDS                      86580L
#define FORTY_SECONDS                       173160L
#define ONE_MINUTE                          259740L
#define TWO_MINUTES                         519480L
#define THREE_MINUTES                       779220L
#define WAIT_1_HOUR                         15584400L
// Convert an EEP in ms to a wait in counts.
inline uint32_t MS_TO_WAIT(uint32_t ms) { return (ms*4433 + 512) / 1024; } //{ return (ms*1000)/231 ; } //{ return (ms*5224 + 512) / 1024; }

#define RELAY_CLOSED                        1
#define RELAY_OPEN                          0

#define BYPASS_CMD_TIMEOUT                  80       //  80 * 100ms = 8 seconds
#define USER_CMD_TIMEOUT                    30       //  30 * 100ms = 3 seconds
#define BYPASS_LOAD_LOSS_ON_BYPASS_TIMEOUT  20       //  20 * 100ms = 2 seconds
#define OUTPUT_OVERLOAD_ETB_TIMEOUT         21       //  21 * 100ms = 2.1 seconds
#define INVERTER_INHIBIT_RESET_TIMEOUT      6000     //  6000 * 100ms = 10 minutes
#define INVERTER_INHIBIT_TRANSFER_LIMIT     3        //  number of transfers to lock on bypass

#define INVERTER_INHIBIT_TIME               (ONE_MINUTE * 60)
#define INVERTER_BYPASS_MATCHING_TIMEOUT    TWENTY_SECONDS

// Return values for StartupLogic()
#define STARTUP_ONLINE         1  // Conditionds are good for online state
#define STARTUP_BYPASS_ON      2  // Conditionds are good for bypass while other units are already ON bypass
#define STARTUP_BYPASS         3  // Conditionds are good for bypass (other units are also OFF)
#define STARTUP_OFF            4  // Startup not possible, normal cmd cleared
#define STARTUP_WAIT           5  // Do nothing.
// Alarm values for Abnormal output voltage
#define ABNORMALOUTV_OK                   0
#define ABNORMALOUTV_BYP_NOUOUT_CANON     1   // No voltage on output, other unit says it's on bypass
#define ABNORMALOUTV_INV_NOUOUT_CANON     2   // No voltage on output, other unit says it's online
#define ABNORMALOUTV_BYP_UOUTON_CANOFF    3   // Voltage on output, other units are not on
#define ABNORMALOUTV_INV_RLYOPEN          4   // Voltage on inverter, inverter relay open, inverter off
//Dead Time EEP Configure UnitConfig.bit.DeadTimeConfig
#define CfgDeadTime1_4us					  0
#define CfgDeadTime2_0us					  1
#define CfgDeadTimeDynamic					  2
//Dead Time Configure for UpmModel
#define DeadTime1_4us					  0x0000
#define DeadTime2_0us					  0x0040

#define APAC_VERSION

    // ****************************************************************
    // MCU Status Bit definitions
    // ****************************************************************

struct sMCUStatusBits
{
    // word 0
    uint16_t     ADCReady:1;                       // 0 A
    uint16_t     MetersReady:1;                    // 1 A
    uint16_t     AutoZeroHallSensors:1;            // 2 A    
    uint16_t     NormalCmd:1;                      // 3 A
   
    uint16_t     ShutdownCmd:1;                    // 4 B  ShutdownCmd. Maintain command active until commanded otherwise 
                                                   //      (i.e. don't try to go back online etc until commanded).
    uint16_t     StandbyCmd:1;                     // 5 B  To standby/Shutdown. Maintain active until commanded otherwise (like ShutdownCmd)
    uint16_t     AutoStandbyCmd:1;                 // 6 B  From Shutdown to Standby, i.e. starts up rectifier
    
    uint16_t     BldInpRemoteUPSOn:1;              // 7 B
   
    uint16_t     ToBypassCmd:1;                    // 8 C  Leave active until commanded otherwise. If we have "go normal" building input, 
                                                   //      only clear ToBypassCmd on positive edge. If ToBypassCmd gets set again (due to 
                                                   //      failed transfers), it should lock us on bypass until new command happens.
    uint16_t     ESSCmd:1;                         // 9 C
    uint16_t     AutoZeroBypassSensors:1;          // 10 C
    uint16_t     UnsynchronizedTransfer:1;         // 11 C
   
    uint16_t     OutputOverloadETB:1;              // 12 D
    uint16_t     BypassAvailableOnBypass:1;        // 13 D
    uint16_t     NormalXferInProgress:1;           // 14 D
    uint16_t     BypassXferInProgress:1;           // 15 D

    // word 1
    uint16_t     CheckedOut:1;                     // 16 A   //Jacob/20130916/unused16->CheckedOut// Tsk_5ms; If true, this UPS can drop off the CAN bus without causing CAN failed
    uint16_t     InverterInhibit:1;                // 17 A
    uint16_t     OutACUVST:1;                      // 18 A
    uint16_t     PostESSMode:1;                    // 19 A

    uint16_t     ShutdownCmdHistory:1;             // 20 B
    uint16_t     StandbyCmdHistory:1;              // 21 B
    uint16_t     ToBypassCmdHistory:1;             // 22 B
    uint16_t     NormalCmdHistory:1;               // 23 B

    uint16_t     BypReturnFailed:1;                // 24 C
    uint16_t     EmergencyTransferToBypass:1;      // 25 C
    uint16_t     ESSCmdHistory:1;                  // 26 C
    uint16_t     BldInpRemoteGotoBypass:1;         // 27 C

    uint16_t     BypassLoadOffHistory:1;           // 28 D
    uint16_t     BldInpBatteryDisconnected:1;      // 29 D
    uint16_t     BldInpMaintenanceBypass:1;        // 30 D
    uint16_t     BldInpMOBClosed:1;                // 31 D

    // word 2
    uint16_t     MaintenanceBypassHistory:1;       // 32 A
    uint16_t     StoreAutoRestart:1;               // 33 A
    uint16_t     AutoRestartReady:1;               // 34 A
    uint16_t     ESSChargeCurrentOverLimit:1;      // 35 A

    uint16_t     ESSLockout:1;                     // 36 B
    uint16_t     StandbyChargerOffCmd:1;           // 37 B
    uint16_t     InternalMaintenanceBypass:1;      // 38 B
    uint16_t     TransferReady:1;                  // 39 B

    uint16_t     ShutdownXferInProgress:1;         // 40 C
    uint16_t     ESSXferInProgress:1;              // 41 C
    uint16_t     ToECTCmdHistory:1;		           // 42 C
    uint16_t     ECTXferInProgress:1;		       // 43 C

    uint16_t     InhibitToECT:1;                   // 44 D
    uint16_t     ToECTCmd:1;				       // 45 D
    uint16_t     ForwardTransfer:1;				   // 46 D
    uint16_t     InverterSuspend:1;                // 47 D

	// word 3
    uint16_t     EcoBatteryTest:1;                 // 48 A
    uint16_t     BypassAvailable:1;                // 49 A
    uint16_t     InverterAvailable:1;              // 50 A
    uint16_t     InverterContactorTest:1;          // 51 A
    
    uint16_t     OnInverter:1;                     // 52 B
    uint16_t     UPSInverterAvailable:1;           // 53 B
    uint16_t     StaticSwitchShort:1;              // 54 B
    uint16_t     ForceSyncBase   :1;               // 55 B

    uint16_t     SyncStartReady:1;                 // 56 C
    uint16_t     OutputSyncDisable:1;              // 57 C
    uint16_t     BldInpRemoteUPSOff:1;             // 58 C
    uint16_t     KeepAliveTrip:1;                  // 59 C

    uint16_t     SequenceNumber:4;                 // 60 D
    // To add any more bits, another word
    // must be added to the word structure below.  PCAN must be
    // updated to transmit the data as a new status packet.
};

union uMCUStatus
{
   uint16_t			words[sizeof(sMCUStatusBits)];
   sMCUStatusBits  bit;
};

struct sBuildingInputs
{
    // word 0
    uint16_t BldInpOnGenerator      		:1;
    uint16_t BldInpRemoteCommand_Online		:1;
    uint16_t BldInpRemoteCommand_Bypass		:1;
    uint16_t BldInpRemoteCommand_Loadoff	:1;
	
    uint16_t BldInpChargerOff       	:1;
    uint16_t BldInpBatteryDisconnected  :1;
    uint16_t BldInpMaintenanceBypass 	:1;
    uint16_t BldInpMOBOpen          	:1;
	
    uint16_t rsd   						:8;		//16-8

};

union uBuildingInputs
{
   uint16_t     words[sizeof(sBuildingInputs)/sizeof(uint16_t)];
   sBuildingInputs  bit;
};



// ********************************************************************************************************
// * STATE MACHINE CLASS DEFINITION
// ********************************************************************************************************
class MCUStateControl
{
private:
    void InitializingState    (void);
    void ShutdownState        (void);
    void StandbyState         (void);
    void OnlineState          (void);
    void OnBypassState        (void);
    void OnBypassStandbyState (void);
    void ESSModeState         (void);
    void EasyCapacityTestState(void);

    void TransferState(eMCUState new_state);

    void     CheckOutACUVST             ( void );
    void     CheckForStuckRelay         ( void );
    void     CheckForFailedRelayFast    ( void );
    uint16_t StartupLogic               ( void );
    bool     StartupRectifier           ( void );
    bool     ShutdownInverter           ( void );
	bool     ShutdownInverterForATB		( void );
    void     ResetTransferState         ( void );
    bool     PrepareForBypassTransfer   ( void );
    bool     PrepareForInverterTransfer ( void );
    bool     BypToNormalOk              ( void );
    bool     BypToNormalPersistentOk    ( void );
    bool     LoadOffToNormalPersistentOk( void );
    bool     BypToECTPersistentOk       ( void );
    bool     AnyRectifierOnCommand      ( void );
    void     StoreAutoRestartEEP        ( void );
    void     ResetAutoRestart           ( void );
    void     CheckPullChain             ( void );
    bool     Check_L3L4_OverLoad        ( void );
    void     CheckEnoughActiveUPM       ( void );
    bool     KeepAliveShutdown          ( bool onBattery );
    bool     AnyBatteryConverterOnCommand( void );
    void     TurnOffLogicPowerSupply    ( void ) { DSPOutRegister.GpoC.bit.Supply_24VOff = 1;}  //SD change to low active to fix HW till new IO board
    bool     StartupBatteryConverter    ( void );
	bool	 IsMatchedOutput			(void);	 
    inline uint16_t EEP_BypassWithNormalCmd      ( void ) { return BypassWithNormalCommand;       }
    inline uint16_t EEP_EPOTransferToBypass      ( void ) { return false;                         }
    inline uint16_t EEP_BattNotRequiredToGoOnline( void ) { return BatteryNotRequiredForOnline;   }
    inline uint16_t EEP_EnableAutoStandby        ( void ) { return EnableAutoStandby;             }
       
protected:
    eMCUState  McuMachineState;

    uint16_t PhaseState;
    uint16_t TransferPhaseState;
    uint16_t RectifierPhaseState;
	uint16_t BatteryPhaseState;

    uint16_t FailedInverterTransfers;
    uint16_t InverterInhibitNumBypassTransfers;
    uint16_t RectifierStartupAttempts;
	uint16_t UtilityTransientDetDelay;
    uint16_t BatteryStartupAttempts;

    uint16_t IsRectifierStart;
    uint16_t IsBatteryStart;

    uint32_t OutputMonitorDelay;
    uint16_t AbnormalOutVoltage;
    uint16_t OldStartResult;
//	    uint32_t ForceStayOnline;
    uint32_t StayOnBypass;
    uint16_t ESSNumOutages;
    uint16_t ESSAbandonReason;
    uint16_t ESSFaultCondition;
    uint16_t StoredInverterRelayState;
    uint32_t InverterAvailableDelay;
    uint32_t StayLoadOff;
    uint32_t CheckOutputACUV;
    bool     ESSOutputSdLow;
    bool     ParallelForwardTransfer;

    StateTimer ESSOutageResetTimer;
    StateTimer ESSLockoutTimer;
    StateTimer PostESSModeTimer;
    StateTimer BypassCommandTimer100ms;
    StateTimer NormalCommandTimer100ms;
    StateTimer InverterInhibitTimer;
    StateTimer RectifierTimer;
    StateTimer InvRelayTimer;
    StateTimer TransferTimer;
    StateTimer ETBDelayTimer;
    StateTimer MCUTimer1;
    StateTimer MCUTimer2;
    StateTimer MCUTimer3;
    StateTimer MCUTimer4;
    StateTimer MCUTimer5;
    StateTimer MCUTimer6;
    StateTimer MCUTimer7;
    StateTimer MCUTimer8;
    StateTimer BypassNotAvailableOnBypassTimer;
    StateTimer KeepAlive;
    StateTimer MCUStateTimer;    // How long you have been in your current MCU State, up to 1 hour
    StateTimer BatteryStartTimer;
    StateTimer ATBTimer;
    FilteredBit BypassParallelingInverter;
    FilteredBit EnoughActiveUPM;

public:
    MCUStateControl();
    ~MCUStateControl(){};

    void Run( void );
    eMCUState GetState( void ) 
    {
        return McuMachineState; 
    }

    const uMCUStatus& GetStatus( void )
    {
        return MCUStatus;
    }

	bool GetUtiLossDetEnable(void)
	{
		return(!UtilityTransientDetDelay);
	}

    bool GetMetersReadyFlag( void ) { return MCUStatus.bit.MetersReady; }
    bool GetADCReadyFlag   ( void ) { return MCUStatus.bit.ADCReady;    }
    bool GetSelectiveTripReadyFlag (void) {return (MCUStateTimer.TimerValue() > ONE_SECOND);}

        // In MCUStateFuncs.h:
    void ClearAllCommands  ( void );
    void Turn_UPS_Off      ( void );
    void NormalCommand     ( void );
    void StandbyCommand    ( void );
    void AutoStandbyCommand( void );
    void BypassCommand     ( void );
    void EcoOnCommand      ( void );
    void EcoOffCommand     ( void );
    void StandbyChargerOffCommand ( void );
    void LogCommands       ( void );
    bool EnableTestMode    ( void );
    void DisableTestMode   ( void );
    bool CheckOKToFlash    ( void );
	
    void SetInverterRelay  ( uint16_t state );
    inline uint16_t GetInverterRelayState( void ) 
    { 
    	if (MCUStatus.bit.InverterContactorTest)
    	{
    		return StoredInverterRelayState;
    	}
    	else
    	{
    		return DSPOutRegister.GpoB.bit.InverterRelay;
    	} 
    }

    void Run1Sec             ( void );
    void Run100Msec          ( void );
    void Run5Msec            ( void );
    void ResetInverterInhibit( void );
    void ResetESSLockout     ( void );
    void ResetSticky         ( void );
    
    inline void BldInpBatteryDisconnected( bool active )
    {   
        MCUStatus.bit.BldInpBatteryDisconnected = active;
    }
    
    inline bool Get_BldInpBatteryDisconnected( void )
    {
        return MCUStatus.bit.BldInpBatteryDisconnected;
    }
    
    inline void BldInpMaintenanceBypass( bool active )
    {   
        MCUStatus.bit.BldInpMaintenanceBypass = active;
    }
    
    void BldInpMOBOpen( bool active );
    
    inline void InternalMaintenanceBypass( bool active )
    {   
        MCUStatus.bit.InternalMaintenanceBypass = active;    
    }
    
    void LeaveEcoForBatteryTest( void )
    {   
        MCUStatus.bit.EcoBatteryTest = 1;   
    }

    void ParaForwardTransfer ( void )
    {
        // External parallel UPS has forward transferred from ESS, need to forward transfer
        if ( McuMachineState == ESS_MODE_STATE )
        {
        	ParallelForwardTransfer = true;
        }
    }

    void BldInpRemoteCommand( uint16_t remoteCommand );
    bool SupportingLoad(void) const;
    bool InStandby(void) const;
    bool InvSupportingLoad(void) const;
    bool BypassSupportingLoad(void) const;
    
    ArmedTrigger ZeroCross;
    uint32_t atb_gap_cycles;
	uint32_t ECTLineTimerOut;
    uint32_t ECTBatTimerOut;
    bool ECTTimeOver;
    uint16_t ECTPhase;
    uint16_t ECTAbandonReason;
    int16_t ECTRestartNumber;
    bool ECTunbalancePower;
	uint16_t AbnormalExitECT;
	uint16_t ForceSyncBaseToByp;
    bool ECTCommand			( void );
    bool ECTLineCommand     ( void );
    bool ECTBatteryCommand	( void );
    void ECTClearCommand    ( void );
	bool UPMTestMode;

    void SetLineECTDuration( uint32_t counter )
    {
    	ECTLineTimerOut = counter;
    }
    
    void SetBatECTDuration( uint32_t counter )
    {
    	ECTBatTimerOut = counter;
    }
    void SetECTPower( uint16_t power )
	{
		ECTPowerSet = ((float)power)/10000;	
	}
	
    void ResetECTAlarm( void )
    {
        MCUStatus.bit.InhibitToECT = 0;
        ECTAbandonReason = 0;
    }
    
    void SetStaticSwitchShort( bool set )
    {
    	MCUStatus.bit.StaticSwitchShort = set;
    }
    
    void SetVFModeReady( bool value )
    {

    }
    uint16_t BatStartupAttempts( void )
    {
        return BatteryStartupAttempts;
    }
    bool IsBatteryStartAtPresent( void )
    {
        return IsBatteryStart;//Be set when battery start begin, be cleared if battery start success to standby state or cleared together with sticky alarms..
    }

    float   ECTPowerSet;
    
    uMCUStatus MCUStatus;    // move to public for fan control
    StateTimer MCUTimerBatECT;
    StateTimer MCUTimerLineECT;
    friend class Debugger;
    uint16_t ForwardTransferCount;
    bool ForwardTransferTransient;
    bool PullChainSyncStart;

    bool AutoBypassAllowed;
	uint32_t ForceStayOnline;	
    bool EtbToOffRelayForBatOcp;	
    bool ParalSystemOn;	
    uBuildingInputs BuildingInputs;
};


extern MCUStateControl MCUStateMachine;

// ********************************************************************************************************
// *                 END OF MCUState.h
// ********************************************************************************************************
#endif

