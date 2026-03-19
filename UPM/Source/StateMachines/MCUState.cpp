// ******************************************************************************************************
// *                 MCUState.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME:   State.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR:  Pasi Pulkkinen, Tuomo Kaikkonen
// *
// *
// *    DATE:        2010/06/22
// *
// *    HISTORY:     See SVN history.
// ******************************************************************************************************


// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Abm.h"
#include "Constants.h"
#include "Debugger.h"
#include "MCUState.h"
#include "Spi_Task.h"
#include "RectifierStateControl.h"
#include "BypassInterface.h"
#include "Eeprom_Map.h"
#include "InvSync.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "Meters.h"
#include "IOexpansion.h"
#include "InverterControl.h"
#include "F28335Port.h"
#include "StateTimer.h"
#include "BootloaderAPI.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
#include "ExtSync.h"
#include "Btr.h"
#include "ACMeter.h"
#include "ACPowerMeter.h"
#include "FCTState.h"
#include <algorithm>
#include "Alarms_AC.h"
#include "RectifierControl.h"
#include "Version.h"

// ********************************************************************************************************
// * EXTERNAL DECLARATIONS
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
MCUStateControl MCUStateMachine;

extern uint16_t NumOfModule;

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************


// ********************************************************************************************************
// *
// * Function:     Class Constructor
// *
// * Purpose:      Initialize things
// *
// ********************************************************************************************************
MCUStateControl::MCUStateControl()
    : BypassParallelingInverter(TEN_SECONDS, 1),
      EnoughActiveUPM(PASS_8,PASS_4)
{
    MCUStatus.words[0] = 0;
    MCUStatus.words[1] = 0;
    MCUStatus.words[2] = 0;
    MCUStatus.words[3] = 0;
    McuMachineState = INITIALIZATION_STATE;

    PhaseState = 0;
    TransferPhaseState = 0;
    RectifierPhaseState = 0;

    BatteryPhaseState = 0;
    BatteryStartupAttempts = 0;
    IsRectifierStart = false;
    IsBatteryStart = false;
    FailedInverterTransfers = 0;
    InverterInhibitNumBypassTransfers = 0; 
    RectifierStartupAttempts = 0;
    AbnormalOutVoltage = ABNORMALOUTV_OK;
    OutputMonitorDelay = 0;
    OldStartResult = STARTUP_OFF;
    StayOnBypass = 0;
    ESSNumOutages = 0;
    ForceStayOnline = 0; 
    ESSAbandonReason = 0;
    InverterAvailableDelay = 0;
    StayLoadOff = 0;
    CheckOutputACUV = 0;
    ParallelForwardTransfer = false;
	UPMTestMode = false;
	MCUStatus.bit.PostESSMode = false;

    // Initialize timers
    ESSOutageResetTimer.ClearTimer();
    ESSLockoutTimer.ClearTimer();
    PostESSModeTimer.ClearTimer();
    BypassCommandTimer100ms.ClearTimer();
    NormalCommandTimer100ms.ClearTimer();
    RectifierTimer.ClearTimer();
    InvRelayTimer.ClearTimer();
    TransferTimer.ClearTimer();
    MCUTimer1.ClearTimer();
    MCUTimer2.ClearTimer();
    MCUTimer3.ClearTimer();
    MCUTimer4.ClearTimer();
    MCUTimer5.ClearTimer();
    MCUTimer6.ClearTimer();
    MCUTimer7.ClearTimer();
    InverterInhibitTimer.ClearTimer();
    ATBTimer.ClearTimer();
	BatteryStartTimer.ClearTimer();
    
    ECTBatTimerOut = 600;
    ECTLineTimerOut = 86400;
    ECTAbandonReason = 0;
    ECTPowerSet = 0.4;
    ECTTimeOver = false;
    ECTPhase = 0;
    ForwardTransferCount = 0;
    ForwardTransferTransient = false;
    PullChainSyncStart = false;
    DeActivePullChain();
    MCUStateTimer.ClearTimer();
    AutoBypassAllowed = true;
    ESSOutputSdLow = false;

	EtbToOffRelayForBatOcp = false;	
	ParalSystemOn = false;	
	BuildingInputs.words[0]= 0;	
}


// ********************************************************************************************************
// *
// * Function: State(void);
// *
// * Purpose:
// *
// * Description:
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void MCUStateControl::Run(void) //4.33K 231us
{
    if (UPMTestMode)
    {
        // For debugging purposes only.
        return;
    }                                         
    
    if (FCTStateMachine.FCT_Status.bit.FCTModeOn)
    {
        // For FCT purposes only.
        FCTStateMachine.RunStateMachine();
        return;
    }                                         
    
    LogCommands();

    // debounce inverter contactor closed, takes ~10ms to open/close
    bool closed = DSPOutRegister.GpoB.bit.InverterRelay;
    if (MCUStatus.bit.InverterContactorTest)
    {
    	closed = !closed;
    }
    NB_DebounceAndQue( UPM_NB_INVERTER_CONTACTOR_CLOSED, closed);
    
    // General purpose timer to track time in a state after transfer.
    MCUStateTimer.CheckTimeout( WAIT_1_HOUR );
    
//	    // disable on rev 5 hardware
//	    if( ( ( ExpansionInputReg.Ready == true )            &&
//	          ( ExpansionInputReg.bit.RevID != Rev5 ) )      ||
//	          DisableRevisionCheck )
//	    {
//	    	// Run a the full state machine rate to minimize timing differences in the sync state
//	    	// between UPMs.
//	    	CheckPullChain();
//	    }
//	    else
//	    {
//	    	NB_DebounceAndQue(UPM_NB_PULL_CHAIN, false);
//	    	NB_DebounceAndQue(UPM_NB_CHECK_PULL_CHAIN, false);
//	    }
	// Run a the full state machine rate to minimize timing differences in the sync state
	// between UPMs.
	CheckPullChain();

	//for JIRA WOMBAT-184, Bat OCP when bus low
	if(!EtbToOffRelayForBatOcp)
	{
		if((McuMachineState != ONLINE_STATE) && (McuMachineState != BYPASS_STANDBY_STATE))
		{
			EtbToOffRelayForBatOcp = false;
		}
	}
	
    switch ( McuMachineState )
    {
        case (SHUTDOWN_STATE):
            ShutdownState();
            break;
        case (STANDBY_STATE):
            StandbyState();
            break;
        case (ONLINE_STATE):
            OnlineState();
            break;
        case (BYPASS_STATE):
            OnBypassState();
            break;
        case (BYPASS_STANDBY_STATE):
            OnBypassStandbyState();
            break;
        case (ESS_MODE_STATE):
            ESSModeState();
            break;
        case (EASY_CAPACITY_TEST_STATE):
            EasyCapacityTestState();
            break;

        case INITIALIZATION_STATE:
        default:
            InitializingState();
            break;
    }

	if(McuMachineState != INITIALIZATION_STATE)
	{
		InitHWSync();
	}
}

// ********************************************************************************************************
// *
// * Function: TransferState(INT8U new_state);
// *
// * Purpose:
// *
// * Parms Passed   :   new_state
// * Returns        :   Nothing
// *
// * Description: Handles the tranfers between states
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void MCUStateControl::TransferState(eMCUState new_state)
{
	float nBypass = std::max(ParallelCan.TotalNumberOfBypass, 1u);
	
    switch (new_state)
    {
        case SHUTDOWN_STATE:
            MCUStatus.bit.InverterSuspend = 0;
            StayOnBypass = 0;
            InverterAvailableDelay = 0;
            ResetInverterInhibit();
            BypassState().RequestBypassState( BYPASS_OFF_STATE );
            Inverter.Off();
            Inverter.MatchBypassVoltage( false );
            BatteryConverter.ChrgRelayCmdOff();
			IsRectifierStart = false;
            //fix JIRA STKEV-28, always turn on charger after UPM off 
            Abm().ClearChargerOffCmd();
            //power conditioner mode, online to shut down
            if( !NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )  &&
                ( ONLINE_STATE == McuMachineState )         &&
                !ParallelCan.ParallelStatus.bit.UpsOffCmd   &&
                !MCUStatus.bit.StandbyCmd                   &&
                !MCUStatus.bit.ShutdownCmd                  &&
                !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ))
            {
                MCUStatus.bit.StoreAutoRestart = 1;
            }
            // Clear ready when trans to shutdown/standby
            MCUStatus.bit.SyncStartReady = 0;
            Rectifier.ClearACPreChargeFail();
            break;

        case STANDBY_STATE:
            MCUStatus.bit.AutoStandbyCmd = 0;
            MCUStatus.bit.InverterSuspend = 0;
            ResetInverterInhibit();
            Inverter.Off();
            Inverter.MatchBypassVoltage( false );
            BatteryConverter.ChrgRelayCmdOn();
            AutoBypassAllowed = false;

            if ( ( McuMachineState == ONLINE_STATE     ||
                   McuMachineState == ESS_MODE_STATE ) &&
                 BatteryConverter.CheckBatteryUVShutDown() )
            {
                MCUStatus.bit.StoreAutoRestart = 1;
            }
            
            //power conditioner mode, online to standby
            if( !NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )  &&
                ( ONLINE_STATE == McuMachineState )         &&
                !ParallelCan.ParallelStatus.bit.UpsOffCmd   &&
                !MCUStatus.bit.StandbyCmd                   &&
                !MCUStatus.bit.ShutdownCmd                   )
            {
                MCUStatus.bit.StoreAutoRestart = 1;
            }
            
            if ( McuMachineState != SHUTDOWN_STATE )
            {
            	InverterAvailableDelay = 0;
            }
            else
            {
            	// give the battery converter time to start up
             	InverterAvailableDelay = ONE_SECOND * 3;
            }
            // Clear ready when trans to shutdown/standby
            MCUStatus.bit.SyncStartReady = 0;
            break;

        case ONLINE_STATE:
        	if ( (McuMachineState == ESS_MODE_STATE) ||
        		 (McuMachineState == BYPASS_STANDBY_STATE))
    		{
    			// preload the load power meter
        		// JIRA EOSD-149: if not correct power divided, bypass to online transfer may cause overload
        		if (NumOfModule > 0)
	        	{
        			LoadPower.DividePower(NumOfModule/*ParallelCan.TotalNumberOfUPMs / nBypass*/);
	        	}
			}
			if( McuMachineState == BYPASS_STANDBY_STATE )
			{
				UtilityTransientDetDelay = WAIT_20_MS;
			}
            StayOnBypass = 0;
            InverterAvailableDelay = 0;
            SetInverterRelay( RELAY_CLOSED );
            MCUStatus.bit.InverterSuspend = 0;
            MCUStatus.bit.UnsynchronizedTransfer = 0;
            BatteryConverter.ChrgRelayCmdOn();

            #if defined(APAC_VERSION)
            if( DeadTimeConfig == CfgDeadTimeDynamic )
            {
                if(UpmModel == DeadTime2_0us) //fix CPSDVAVE-40 for 20-80K
                {
                    UpmModel = DeadTime1_4us; //2.0us->1.4us
                    DeadtimeConfEnd = false;
                }
            }
            #endif
            Rectifier.ClearACPreChargeFail();
            break;

        case BYPASS_STANDBY_STATE:
            MCUStatus.bit.InverterSuspend = 0;
            MCUStatus.bit.AutoStandbyCmd = 0;
            MCUStatus.bit.StandbyCmd = 0;
            MCUStatus.bit.ShutdownCmd = 0;
            BatteryConverter.ChrgRelayCmdOn();
            if ( McuMachineState == BYPASS_STATE )
            {
                InverterAvailableDelay = ONE_SECOND * 3; //give battery converter time to start up
            }
            else
            {
                InverterAvailableDelay = 0;
            }

            //lint -fallthrough
        case BYPASS_STATE:
        	//fix JIRA STKEV-28, always turn on charger after UPM off
        	Abm().ClearChargerOffCmd();
        	AutoBypassAllowed = false;
            // preload the load power meter
            if (McuMachineState == ONLINE_STATE)
            {
                // Assume balanced load across all UPMs prior to the transfer.
                // Set nBypass to be no less than one, to defensively avoid divide-by-zero.
                LoadPower.DividePower(nBypass / ParallelCan.TotalNumberOfUPMs);
            }

            MCUStatus.bit.InverterSuspend = 0;
            if ( new_state == BYPASS_STATE )
            {
                BatteryConverter.ChrgRelayCmdOff();
            }

            if ( !MCUStatus.bit.UnsynchronizedTransfer )
            {
//	                BypassState().RequestBypassState( BYPASS_FIRE_STATE );
				if(!EtbToOffRelayForBatOcp) 	//for JIRA WOMBAT-184
				{				
					BypassState().RequestBypassState( BYPASS_FIRE_STATE ); 		
				}
            }

            // Stay minimum of one second before allowing OnLine transfer,
            // If coming from offline states, allow time for the system load to be detected,
            // Minimum of 3 seconds to detect inverter contactor failure
            if ( McuMachineState != BYPASS_STATE  &&
                 McuMachineState != BYPASS_STANDBY_STATE )
            {
                StayOnBypass = ONE_SECOND * 4;
            }

            if ( ( McuMachineState == ONLINE_STATE                 ||
                   McuMachineState == ESS_MODE_STATE )             &&
                 ( MCUStatus.bit.EmergencyTransferToBypass         ||
                   NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )    ||
                   NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) ||
                   BatteryConverter.CheckBatteryUVShutDown() ||
                   NB_GetNodebit( UPM_NB_SHUTDOWN_IMMINENT ) )
                )
            {
                InverterInhibitNumBypassTransfers++;
                
                if ( BatteryConverter.CheckBatteryUVShutDown() )
                {
                    MCUStatus.bit.StoreAutoRestart = 1;
                }
            }

            //power conditioner mode, online to bypass
            if( !NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )            &&
                ( ONLINE_STATE == McuMachineState )                   &&
                !MCUStatus.bit.ToBypassCmd                            &&
                !MCUStatus.bit.ToECTCmd                               &&
                !NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )          &&
                !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )   &&
                !( NB_GetNodebit( UPM_NB_MBS_CLOSED ) &&
                   !NB_GetNodebit( UPM_NB_MOB_OPEN ) )     )
            {
                MCUStatus.bit.StoreAutoRestart = 1;
            }

            if ( MCUStatus.bit.ToBypassCmd || MCUStatus.bit.ToECTCmd  )
            {
                MCUStatus.bit.NormalCmd = 0;
            }
            
            if ( McuMachineState == SHUTDOWN_STATE )
            {
                InverterAvailableDelay = WAIT_300_MS; // give output ACUV time to clear
            }
            else if ( new_state != BYPASS_STANDBY_STATE )
            {
            	InverterAvailableDelay = 0;
            }
            break;

        case ESS_MODE_STATE:
            StayOnBypass = 0;
            InverterAvailableDelay = 0;
            BypassState().RequestBypassState( BYPASS_FIRE_STATE );
            MCUStatus.bit.InverterSuspend = 1;
            break;

        case EASY_CAPACITY_TEST_STATE:
            StayOnBypass = 0;
            InverterAvailableDelay = 0;
            ECTAbandonReason = 0;
            
            if( Rectifier.GetStatus().bit.BatteryECT )
            {
                BatteryConverter.ChrgRelayCmdOn();
            }
//				#if defined(APAC_VERSION)
//	            if( UnitConfig.bit.ECTDTProtectEnable == true )
//	            {
//	                if(UpmModel == DeadTime1_4us)//fix CPSDVAVE-40 for 20-80K
//	            	{
//	            	    UpmModel = DeadTime2_0us;//1.4us->2.0us
//	            	    DeadtimeConfEnd = false;
//	            	}
//	            }
//				#endif
            break;
            
        default:
            StayOnBypass = 0;
            InverterAvailableDelay = 0;
            break;
    }

    NB_SetNodebit( UPM_NB_UPS_ON_NORMAL, (new_state == ONLINE_STATE     || 
                                          new_state == ESS_MODE_STATE   ||
                                          (new_state == EASY_CAPACITY_TEST_STATE && 
                                          !Rectifier.GetStatus().bit.BatteryECT) ) );
                                          
    NB_SetNodebit( UPM_NB_UPS_ON_BYPASS, (new_state == BYPASS_STATE ||
                                          new_state == BYPASS_STANDBY_STATE ||
                                          new_state == EASY_CAPACITY_TEST_STATE) );

    NB_SetNodebit( UPM_NB_LOAD_POWER_OFF, (new_state == SHUTDOWN_STATE ||
                                           new_state == STANDBY_STATE) );

    NB_SetNodebit( UPM_NB_IN_HIGH_EFFICIENCY_MODE, (new_state == ESS_MODE_STATE ), ESSAbandonReason );
    
    NB_SetNodebit( UPM_NB_IN_EASY_CAPACITY_TEST_MODE, (new_state == EASY_CAPACITY_TEST_STATE), ECTAbandonReason );
    

    if ( new_state != ONLINE_STATE )
    {
        OutputMonitorDelay = 0;
        ForceStayOnline = 0;
    }
    if ( new_state != BYPASS_STANDBY_STATE  &&
         new_state != BYPASS_STATE )
    {
    	MCUStatus.bit.EmergencyTransferToBypass = 0;
    }
    if ( new_state != STANDBY_STATE   &&
         new_state != SHUTDOWN_STATE )
    {
    	StayLoadOff = 0;
    	CheckOutputACUV = 0;
    }
    
    ESSAbandonReason = 0;
    RectifierStartupAttempts = 0;//APACTS-35: clear the counter once startup success
    
    McuMachineState = new_state;
    NB_LogStateChange(UPM_NB_MCU_STATE_CHANGED, McuMachineState);
    PhaseState = 0;
    RectifierPhaseState = 0;
    BatteryPhaseState = 0;
    PullChainSyncStart = false;
    DeActivePullChain();
    ResetTransferState();

    MCUStatus.bit.BypassXferInProgress = 0;
    MCUStatus.bit.ShutdownXferInProgress = 0;
    MCUStatus.bit.NormalXferInProgress = 0;
    MCUStatus.bit.ECTXferInProgress = 0;
    MCUStatus.bit.ESSXferInProgress = 0;
    FailedInverterTransfers = 0;
    MCUStatus.bit.StandbyChargerOffCmd = 0;
    MCUStatus.bit.ForwardTransfer = 0;
    MCUStatus.bit.OutputSyncDisable = 0;
    ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;
    ParallelCan.ParallelCommand.bit.para_redundant_off = 0;
    ParallelCan.ParallelCommand.bit.para_redundant_battery_off = 0;
    ParallelCan.ParallelStatus.bit.UpsOffCmd = 0;
    if ( NB_GetNodebit( UPM_NB_UPS_ON_BYPASS ) )
    {
    	MCUStatus.bit.BypassAvailable = true;
    }
    if ( NB_GetNodebit( UPM_NB_UPS_ON_NORMAL ) )
    {
    	MCUStatus.bit.InverterAvailable = true;
    }
    MCUStatus.bit.OnInverter = NB_GetNodebit( UPM_NB_UPS_ON_NORMAL );
    if ( MCUStatus.bit.OnInverter )
    {
    	// UPSInverterAvailable stays set in "on normal" states.
    	// It does not need to be cleared when transfering out of "on normal",
    	// it will be cleared in those states if necessary
    	MCUStatus.bit.UPSInverterAvailable = 1;
    }
    BypassParallelingInverter.SetState(false);
    
    MCUStatus.bit.AutoZeroBypassSensors = false;
    ForceSyncBaseToByp = 0;
    ESSOutputSdLow = false;
	MCUStatus.bit.SyncStartReady = 0;
    ParallelForwardTransfer = false;

    MCUTimer1.ClearTimer();
    MCUTimer2.ClearTimer();
    MCUTimer3.ClearTimer();
    MCUTimer4.ClearTimer();
    MCUTimer5.ClearTimer();
    MCUTimer6.ClearTimer();
    MCUTimer7.ClearTimer();
	MCUTimer8.ClearTimer();
    InvRelayTimer.ClearTimer();    
    RectifierTimer.ClearTimer();
	BatteryStartTimer.ClearTimer();
    KeepAlive.ClearTimer();
    MCUStateTimer.ClearTimer();
    ATBTimer.ClearTimer();
}

// ********************************************************************************************************
// *
// * Function: InitializingState(void);
// *
// * Purpose:
// *
// * Description:
// *
// ********************************************************************************************************
void MCUStateControl::InitializingState(void)
{
    MCUStatus.bit.NormalCmd = 0;
    MCUStatus.bit.ShutdownCmd = 0;
    MCUStatus.bit.StandbyCmd = 0;
    MCUStatus.bit.AutoStandbyCmd = 0;
   
    switch ( PhaseState )
    {
        default:
        case 0:
                // Check hardware ID pins. Hardware board ID has to be supported, control board version
                // and PLD version have to be known and compatible. If not then a software incompatibility 
                // detected alarm is set and the state machine will remain in the initialization state.
            if ( MCUTimer1.CheckTimeout( ONE_SECOND + WAIT_500_MS) )
            {
                // This timeout should be large enough for the error to be logged
                // If it is also a hardware ID error, then xfer to BLD.
                if (NB_GetNodebit(UPM_NB_SOFTWARE_INCOMPATIBILITY_DETECTED))
                {
                    if ((MyHardwareNumber != MACHINE_ID_PANDA_ESSENTIAL) && (MyHardwareNumber != MACHINE_ID_HOBBIT_93PR) && (MyHardwareNumber != MACHINE_ID_ORIONDO))
                    {
                        // Transfer to bootloader with an invalid addr space specified.
                        // This should not cause flash to be erased.
                        Transfer2Bootloader(0xffff);
                    }
                    // Else wait.
                }
                else
                {
                    MCUTimer1.ClearTimer();
                    PhaseState++;
                }
            }
            break;

        case 1:
                // Wait until auxiliary power has stabilized
            if ( !NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )  &&
                 !NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT ) &&
                 !NB_GetNodebit( UPM_NB_DRIVER_FAULT ) )
            {
                    // Wait a few seconds to see the alarm is not set
                if ( MCUTimer1.CheckTimeout( ONE_SECOND * 3 ) )
                {
                    PhaseState++;
                    MCUTimer1.ClearTimer();
                }
            }
            else
            {
                MCUTimer1.ClearTimer();
            }
            break;

        case 2:
                // Wait for correct bypass status information
                // Initialize bypass signals
            if (BypassState().IsBypassInitialized())
            {
                if ( BypassState().GetMonitoredBypassState() == BYPASS_FIRE_STATE )
                {
                    BypassState().RequestBypassState( BYPASS_FIRE_STATE );
                }
                else
                {
                    BypassState().RequestBypassState( BYPASS_OFF_STATE );
                }

                MCUTimer1.ClearTimer();
                PhaseState++;
            }
            else
            {
                    // Bypass not initialized.
                    // do we want to block here, or continue?
                    //
                    // For now, continue after a timeout and hope there is an alarm (STATIC_SWITCH_FAILURE ?)
                if ( MCUTimer1.CheckTimeout( TEN_SECONDS ) )
                {
                    BypassState().RequestBypassState( BYPASS_OFF_STATE );
                    MCUTimer1.ClearTimer();
                    PhaseState++;
                }
            }          
            break;

        case 3:
            if ( EEStatusBits.bit.EEDataInitialized )
            {
                    // 14) Auto zero Hall sensor offsets. Inverter and rectifier sensors are zeroed.
                MCUStatus.bit.AutoZeroHallSensors = 1;
                InitHWSync();

                #if defined(APAC_VERSION)
                if( DeadTimeConfig == CfgDeadTime1_4us )
                {
                	if(UpmModel == DeadTime2_0us)
                	{
                		UpmModel = DeadTime1_4us;//2.0us->1.4us
                		DeadtimeConfEnd = false;
                	}
                }
                #endif
                MCUTimer1.ClearTimer();
                PhaseState++;
            }
            break;
            
        case 4:
            if ( MCUTimer1.CheckTimeout( ONE_SECOND ) )
            {
                MCUStatus.bit.AutoZeroHallSensors = 0;
                MCUStatus.bit.ADCReady = 1;
                // some bit of nonsense here
                MCUTimer1.ClearTimer();
                PhaseState++;
            }
            break;
            
        case 5:
            if ( MCUTimer1.CheckTimeout( ONE_SECOND ) )
            {
                MCUStatus.bit.MetersReady = 1;
                MCUTimer1.ClearTimer();
                PhaseState++;
            }
            break;
            
        case 6:
            // TBD, need to do some of this ??:
            //
            //  7)  Initialize slow utility measurements to fast measurement values.
            //  8)  Enable aux voltage alarm monitoring
            //  9)  Clear auxiliary power startup failure, back-feed failure, abnormal output voltage alarm, 
            //      bypass SCR failure, DC charger failure, balancer relay failure, rectifier failure, 
            //      inverter startup failure, fuse failure, DCOV and DCUV alarms
            //  10) Activate IO_GOOD (Control board logic good)
            //  11) Wait 500ms
            //  12) Initialize rectifier control values
            //  13) Start monitoring temperature alarms, building alarms, voltages etc.

            //

            SetIOGood( true );

            PhaseState++;
            break;

        case 7:            
            if ( !NB_GetNodebit( UPM_NB_NON_VOLATILE_RAM_FAILURE ) )
            {
                if ( MCUStatus.bit.ShutdownCmd )
                {
                    TransferState( SHUTDOWN_STATE );
                }
                else if ( BypassState().GetBypassState() == BYPASS_FIRE_STATE )
                {
                    TransferState( BYPASS_STATE );
                }
                else
                {
                    TransferState( SHUTDOWN_STATE );

                    if ( EEP_EnableAutoStandby() )
                    {                        
                        AutoStandbyCommand();
                    }
                }
            }

            break;                
    }
} // InitializingState


// ********************************************************************************************************
// *
// * Function: ShutdownState(void);
// *
// * Purpose:  
// *
// * Description: 
// *
// ********************************************************************************************************
void MCUStateControl::ShutdownState(void)
{
    uRectifierStatus rectStatus = Rectifier.GetStatus();
    uint16_t StartupMode = StartupLogic();
    
    MCUStatus.bit.BypassAvailable = !FrequencyConverterMode                       &&
                                    NB_GetNodebit( UPM_NB_BYPASS_INSTALLED )      &&
                                    !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE ) &&
                                    !NB_GetNodebit( UPM_NB_PARALLEL_SETUP_FAIL );

    MCUStatus.bit.InverterAvailable = MCUStatus.bit.NormalCmd                              &&
                                      LoadOffToNormalPersistentOk()                        &&
                                      !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )      &&
                                      !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )  &&
                                      !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )        &&
                                      !NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )       &&
                                      !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE )          &&
                                      !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )     &&
                                      ( ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
                                          NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) ) ||
                                        EEP_BattNotRequiredToGoOnline() );
    
    if ( CheckOutputACUV )
    {
        CheckOutputACUV--;
    }
    
    //set normalCmd for ECT restart
    if( AbnormalExitECT && ( ECTRestartNumber > 0 ) )
    {
    	if ( MCUTimer4.CheckTimeout( TEN_SECONDS ) )
		{
			MCUTimer4.ClearTimer();
			MCUStatus.bit.NormalCmd = 1;
		}
	}
	else
	{
		MCUTimer4.ClearTimer();
	}

        // 
        // SHUTDOWN conditions:
        // 
    if ( NB_GetNodebit(UPM_NB_REMOTE_EMERGENCY_POWER_OFF) ||
         MCUStatus.bit.ShutdownCmd ||
         NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) )
    {
            // EPO - remain in shutdown state
        MCUStatus.bit.BypassXferInProgress = 0;
        ClearAllCommands();
        ResetAutoRestart();

            // Keep bypass in IDLE (not READY), if not starting up.
        BypassState().RequestBypassState( BYPASS_OFF_STATE );
            
        if ( rectStatus.bit.RectifierPrecharging || NB_GetNodebit( UPM_NB_RECTIFIER_ON ) )
        {
            Rectifier.RectifierOff();
        }    

        MCUTimer1.ClearTimer();
        MCUTimer2.ClearTimer();
        MCUTimer3.ClearTimer();

            // remain in SHUTDOWN_STATE:
        StartupMode = STARTUP_OFF;
    }
    
        //
        // -->Parallel State Sync
        // 
    if (ParallelCan.ParallelCommand.bit.para_inverter_off_sync &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)
    {
        ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;    
        
        if( MCUStatus.bit.BypassAvailable )
        {
        	MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
            MCUStatus.bit.EmergencyTransferToBypass = 1;   
            TransferState( BYPASS_STATE );
        }
        else
        {
        	MCUStatus.bit.NormalCmd = 0;
        }
        
        return;
    }
    
        // 
        // -->ETB (Emergency Transfer on Bypass) conditions:
        //
    if ( ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass &&
         MCUStatus.bit.BypassAvailable )
    {
        MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
        TransferState( BYPASS_STATE );
    }
    else if ( CheckOutputACUV    &&
              MCUStatus.bit.BypassAvailable &&
              NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) )
    {
        MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
        MCUStatus.bit.EmergencyTransferToBypass = 1;
        TransferState( BYPASS_STATE );
    }
    
        // 
        // -->BYPASS transfer conditions:
        // 
    if( Unit_Setup1.bit.AutoBypassEnable  &&
	    !AutoRestart                      &&
	    AutoBypassAllowed                 &&
    	MCUStatus.bit.BypassAvailable )
    {
        if( MCUTimer5.CheckTimeout( FIVE_SECONDS ) && !ParallelCan.UPSIsParallel() ) // auto-bypass only for single ups
        {
        	AutoBypassAllowed = false;
    		ParallelCan.InternalCommand.bit.csb_bypass_on_command = 1;
		}
    }
    else
    {
    	MCUTimer5.ClearTimer();
    }

    if ( StartupMode == STARTUP_BYPASS )
    {
        MCUStatus.bit.BypassXferInProgress = 1;

        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( BYPASS_STATE );
        }
    }
    else if ( StartupMode == STARTUP_BYPASS_ON )
    {
    	TransferState( BYPASS_STATE );
    }
    else
    {
        MCUStatus.bit.BypassXferInProgress = 0;
        BypassState().RequestBypassState( BYPASS_OFF_STATE );
    }        


        // 
        // -->STANDBY(->NORMAL) transfer conditions:
        // 
    bool UtilityOkayForRectifierStart =
         !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) &&
		 !NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE ) &&
		 !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
		 !NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY );
		
    if( UtilityOkayForRectifierStart &&
        !Rectifier.GetStatus().bit.ACPreChargeFail &&
        !IsBatteryStart)
    {
	    if ( !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE ) &&
	         ( StartupMode == STARTUP_ONLINE || AnyRectifierOnCommand() ) )
	    {
			switch ( PhaseState )
			{
				case 0:
					if ( Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE )
					{
						// Make sure the rectifier is fully shut down from a canceled battery start
						PhaseState++;
					}
					break;
				case 1:
					IsRectifierStart = true;
					break;
				default:
					break; 
	        }
			
            if( IsRectifierStart )  //IsRectifierStart may be set by shutdown state or bypass state
            {			
		        if ( !StartupRectifier() )
		        {
				    PhaseState = 0;
		            ClearAllCommands();
	            
		            // command other UPM's in my UPS to clear all commands
		            ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
		        }
            }
	    }
	    else
	    {
	        RectifierStartupAttempts = 0;

	        if ( ShutdownInverter() )
	        {
	            Rectifier.RectifierOff();
	        }
		}
    }
	else if( !UtilityOkayForRectifierStart && IsRectifierStart )
	{
		//If normal start is in processing and the utility interrupts, normal start is aborted. And no more battery start transfer.
			
		// Reset the startup attempts; problems on the utility side should not cause rectifier startup failed to log
		RectifierPhaseState = 0;
		RectifierStartupAttempts = 0;
        IsRectifierStart = false;
		
//			if( AnyRectifierOnCommand() )
//			{
//				NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, true, 2 );
//				ClearAllCommands();
//				// command other UPM's in my UPS to clear all commands
//				ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
//			}
			
		if ( ShutdownInverter() )
		{
			Rectifier.RectifierOff();
		}
		
		if ( KeepAliveShutdown( NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
								!NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) )  && //If battery is disconnected, KeepAliveShutdown should not work.
			 MCUStateTimer.TimerValue() > ONE_SECOND ) // wait an additional second for when coming from standby (for relays)
		{
			MCUStatus.bit.KeepAliveTrip = 1;
            DSPOutRegister.GpoC.bit.Supply_24VOff = 1;			
		}
//			if( MCUStatus.bit.KeepAliveTrip &&
//				NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
//			{
//				TurnOffLogicPowerSupply();
//			}
    }
	else
	{
		if ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )  &&
			 !NB_GetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE ) &&
			 !NB_GetNodebit( UPM_NB_BATTERY_LOW )	  &&
			 !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )	  &&
			 ( ( AnyBatteryConverterOnCommand() )	 ||
			   ( STARTUP_ONLINE == StartupMode ) )	 &&
				 !MCUStatus.bit.KeepAliveTrip ) // Prevents state loops after standby -> shutdown because of keep alive trip
            {
				IsBatteryStart = true;
				//does precharge procedure,UPM_NB_BATTERY_DISCHARGING is set and DC reaches target voltage when battery converter is on 		  
				if( !StartupBatteryConverter() )
				{
					ClearAllCommands(); 		   
					// command other UPM's in my UPS to clear all commands
					ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
				}
			}
			else
			{
				BatteryPhaseState = 0;
				BatteryStartTimer.ClearTimer();
				BatteryStartupAttempts = 0;
		
//					if( AnyBatteryConverterOnCommand())
//					{
//						if(!NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE ))
//						{
//							NB_SetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE, true, 2 );
//							ClearAllCommands();
//							// command other UPM's in my UPS to clear all commands
//							ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
//						}
//					}
		
				if ( ShutdownInverter() )
				{
					Rectifier.RectifierOff();
					BatteryConverter.CancelBatteryStart();
					BatteryConverter.BoostOff();
					IsBatteryStart = false; // allow rectifier start to process if input is okay
				}
				
				if ( KeepAliveShutdown( NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
										!NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) )  && //If battery is disconnected, KeepAliveShutdown should not work.
					 MCUStateTimer.TimerValue() > ONE_SECOND ) // wait an additional second for when coming from standby (for relays)
				{
					MCUStatus.bit.KeepAliveTrip = 1;
					DSPOutRegister.GpoC.bit.Supply_24VOff = 1;
					
				}
//					if(MCUStatus.bit.KeepAliveTrip && \
//						NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ))
//					{
//						TurnOffLogicPowerSupply();
//					}
            }		 
    }

	// 
	// -->STANDBY if rectifier is on
	// 
	if ( NB_GetNodebit( UPM_NB_RECTIFIER_ON )&& (!IsBatteryStart )	)
	{
		if ( MCUTimer3.CheckTimeout( ONE_SECOND ) )
		{
			MCUTimer3.ClearTimer();
			TransferState( STANDBY_STATE );
		}
	}
	else if(IsBatteryStart) //if battery start
	{
		float DCLinkRampupCompleteVoltage = 5.0;
//			float DCLinkVoltage = DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered;
//			float dclinkerror = std::fabs( ( Rectifier.GetDCLinkVoltageRef() - 10.0f ) - DCLinkVoltage );
		float dclinkerrorP = ((Rectifier.GetDCLinkVoltageRef() - 10.0f)*0.5f) - DCLinkVoltagePositive.FastFiltered;
		float dclinkerrorN = ((Rectifier.GetDCLinkVoltageRef() - 10.0f)*0.5f) + DCLinkVoltageNegative.FastFiltered;

		if ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
			 ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE ) &&
			 ( dclinkerrorP < DCLinkRampupCompleteVoltage ) &&
			 ( dclinkerrorN < DCLinkRampupCompleteVoltage ) )
		{
			if ( MCUTimer2.CheckTimeout( ONE_SECOND ) )
			{
				MCUTimer2.ClearTimer();
				TransferState( STANDBY_STATE );
				IsBatteryStart = false;
			}
		}
		else
		{
			MCUTimer2.ClearTimer();
		}
		MCUTimer3.ClearTimer();
	}
	else
	{
		MCUTimer3.ClearTimer();
	}
    
    if ( !MCUStatus.bit.BypassXferInProgress  &&
         !MCUStatus.bit.NormalXferInProgress  &&
         StartupMode != STARTUP_OFF )
    {
    	ResetTransferState();
    }
    
    if ( StartupMode == STARTUP_OFF )
    {
    	// allow other UPS's to transfer when I am staying off
    	// only set on zero cross in case I'm the last one to set this (common with rectifier failure)
    	PrepareForInverterTransfer();
    }
} // ShutdownState



// ********************************************************************************************************
// *
// * Function:  StandbyState()
// *
// * Purpose:   Standby state.  Rectifier and charger are on.  Load is unsupported.
// *
// * Description: 
// *
// ********************************************************************************************************
void MCUStateControl::StandbyState( void )
{
    uint16_t StartupMode = StartupLogic();
    uInverterStatus iStat = Inverter.GetStatus();
    uRectifierStatus rStat = Rectifier.GetStatus();
    
    MCUStatus.bit.BypassAvailable = !FrequencyConverterMode                       &&
                                    NB_GetNodebit( UPM_NB_BYPASS_INSTALLED )      &&
                                    !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE ) &&
                                    !NB_GetNodebit( UPM_NB_PARALLEL_SETUP_FAIL );

    if ( !InverterAvailableDelay ) //wait for battery converter to start before disqualifying my inverter
    {
        MCUStatus.bit.InverterAvailable = MCUStatus.bit.NormalCmd                              &&
                                          LoadOffToNormalPersistentOk()                        &&
                                          !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )      &&
                                          !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )  &&
                                          !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )        &&
                                          !NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )       &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE )          &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )     &&
                                          ( BatteryConverter.BatteryAvailable()                ||
                                            EEP_BattNotRequiredToGoOnline() );
    }
    else
    {
    	InverterAvailableDelay--;
    }
    
    if ( CheckOutputACUV )
    {
        CheckOutputACUV--;
    }
                                        
        //
        // -->SHUTDOWN conditions
        //
    if ( NB_GetNodebit(UPM_NB_REMOTE_EMERGENCY_POWER_OFF) ||
         MCUStatus.bit.ShutdownCmd ||
         NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) )
    {
        TransferState(SHUTDOWN_STATE);
        return;
    }
    
        //
        // -->Parallel State Sync
        // 
    if (ParallelCan.ParallelCommand.bit.para_inverter_off_sync &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)
    {
        ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;    
        
        if( MCUStatus.bit.BypassAvailable )
        {
        	MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
            MCUStatus.bit.EmergencyTransferToBypass = 1;   
            TransferState( BYPASS_STANDBY_STATE );
        }
        else
        {
        	MCUStatus.bit.NormalCmd = 0;
        }
        
        return;
    }
    
        // 
        // -->ETB (Emergency Transfer on Bypass) conditions:
        //
    if ( ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass &&
         MCUStatus.bit.BypassAvailable)
    	 //GPE-1464/Remove this condition/ NB_GetNodebit(UPM_NB_PULL_CHAIN) )
    {
    	MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
    	TransferState( BYPASS_STANDBY_STATE );
    }
    else if ( CheckOutputACUV    &&
              MCUStatus.bit.BypassAvailable &&
              NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) )
    {
    	MCUStatus.bit.NormalCmd = 1; // allow parallel UPS's to attempt to go back online
    	MCUStatus.bit.EmergencyTransferToBypass = 1;
    	TransferState( BYPASS_STATE );
    }
    
        // 
        // -->BYPASS transfer conditions:
        // 
    if ( StartupMode == STARTUP_BYPASS )
    {
        MCUStatus.bit.BypassXferInProgress = 1;

        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( BYPASS_STANDBY_STATE );
        }
    }
    else if ( StartupMode == STARTUP_BYPASS_ON )
    {
    	TransferState( BYPASS_STANDBY_STATE );
    }
    else
    {
        MCUStatus.bit.BypassXferInProgress = 0;
        // Get bypass ready
        BypassState().RequestBypassState( BYPASS_READY_STATE );
    }        
    
    bool bypassAvailable = !FrequencyConverterMode                        &&
                           NB_GetNodebit( UPM_NB_BYPASS_INSTALLED)        &&
                           !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE )  &&
                           !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE ) &&
                           ParallelCan.PCan_CheckSystemBypassAvailable();

        // 
        // -->ONLINE transfer conditions:
        // 
    if ( StartupMode == STARTUP_ONLINE )
    {
        // 1.Bat normal ||
        // 2.Rec normal&&(eep not need bat)
        if ( BatteryConverter.BatteryAvailable() ||
           ( EEP_BattNotRequiredToGoOnline() && Rectifier.GetState() == RECTIFIER_NORMAL_STATE ) )
        {
            // 1. Sys Outp ACUV
            if ( MCUStatus.bit.OutACUVST )
            {
                if ( RELAY_OPEN == GetInverterRelayState() )
                {
                    SetInverterRelay( RELAY_CLOSED );
                    MCUStatus.bit.SyncStartReady = 0;
                    MCUTimer1.ClearTimer();
                }
                else if ( !iStat.bit.InverterOn )                           
                {
                    if ( MCUTimer1.CheckTimeout( WAIT_500_MS ) )
                    {
                        // to tell somebody don't reset transfer
                        MCUStatus.bit.NormalXferInProgress = 1;  // to tell somebody don't reset transfer
						MCUStatus.bit.OutputSyncDisable = true;

                        // 1.1. Paral(no byp/line): use pullchain
                        if( PullChainSyncStart )
                        {
                            switch( PhaseState )
                            {
                                case 0:
                                    // step 1: set myself ready if others are not starting
                                    // as when unit in MOB is allow on single
                                    MCUStatus.bit.SyncStartReady = 1;
                                    MCUTimer2.ClearTimer();
                                    MCUTimer3.ClearTimer();
                                    PhaseState++;
//										MCUStatus.bit.OutputSyncDisable = true;
                                    break;
                                    
                                case 1:
                                    // step 2: there are enough UPS support the load, active pull chain
                                    // paral cmd on
//	                                    if(MCUStateMachine.ParalSystemOn)
                                    {
//											if ( ParallelCan.ParGlobalAndData.McuStatus.bit.SyncStartReady ||
//											   ( NB_GetNodebit( UPM_NB_MOB_OPEN ) && !EEP_IsInternalParallel() ) ) // if all ready
										if ( ParallelCan.ParGlobalAndData.McuStatus.bit.SyncStartReady ||
										     ( NB_GetNodebit( UPM_NB_MOB_OPEN ) && (NumOfUPMs == 1)) ) // if all ready
	                                    {
											if( (ParallelCan.ParallelStatus.bit.Master) ||
												( NB_GetNodebit( UPM_NB_MOB_OPEN ) && (NumOfUPMs == 1)) ||
												( NB_GetNodebit( UPM_NB_MOB_OPEN ) && (NumOfUPMs > 1) && (MyUPMNumber == 0)) ) // if all ready
											{
		                                        ActivePullChain();
											}
	                                    }
                                        // for jira559, Parallel Autorestart, will cause, U1(then autorestart to Standby, first unit) will show inv over time >5s; 'UPM_NB_INVERTER_STARTUP_FAILURE = 1'
                                        //   1) U1=Standby, Vbus=360; Send cmd: U2~4 on; wait paral inv on over 5s, then inv ot fail
                                        //   2) Because U2~4 Receive cmd,  Vbus 0->360V, about20~40s; -> Standby -> wait 5s, inv fail
                                        // Solve Method1: 1)Paral inv on, wait time from 5s to 1min;  to let U2~4 Standby ready
	                                    else if ( MCUTimer2.CheckTimeout(ONE_MINUTE) )
	                                    {
	                                        // there is not enough UPS support the load
	                                        NB_SetNodebit( UPM_NB_INVERTER_STARTUP_FAILURE, true, 1 );
	                                        DeActivePullChain();
	                                        PhaseState = 0;
	                                    }
                                    }
                                    // step 3: after myself/globl ready, wait for pull chain sync start                                    
                                    if ( !DSPInRegister.GpiB.bit.PullChain )
                                    {                                    
										Inverter.On();
                                        MCUTimer1.ClearTimer();
                                    }
									else if ( MCUTimer3.CheckTimeout(ONE_SECOND + ONE_MINUTE) )
                                    {
                                        // if didn't receive pull chain within 1min 1s
                                        NB_SetNodebit( UPM_NB_INVERTER_STARTUP_FAILURE, true, 2 );
                                        PhaseState = 0;
                                    }
                                    break;

                                default:
                                    break;
                            }
                        }
                        // 1.2. Single or Paral(with byp/line): only use pll byp/line (not use pullchain)
                        else
                        {
							if(Rectifier.GetStatus().bit.ACPreChargeFail == false)
							{
	                            if ( PrepareForInverterTransfer() || UPMCalEnabled )
	                            {
	                                Inverter.On();
									Rectifier.ClearACPreChargeFail();
	                                MCUTimer1.ClearTimer();
								}
							}
							// when ac prechg fail, wait unit bat boost -> rec normal -> then online
							else
							{
								if(Rectifier.InputAvailable() == false)
								{
									if ( PrepareForInverterTransfer() || UPMCalEnabled )
									{
										Inverter.On();										
										Rectifier.ClearACPreChargeFail();
										MCUTimer1.ClearTimer();
									}
								}
								else
								{
									if( ( ( Rectifier.GetState() == RECTIFIER_NORMAL_STATE ) && ( BatteryConverter.GetBatteryState() != BATTERY_ON_BOOST_STATE) ) ||
										MCUTimer4.CheckTimeout(FIVE_SECONDS) )	
									{
										if ( PrepareForInverterTransfer() || UPMCalEnabled )
										{
											Inverter.On();											
											Rectifier.ClearACPreChargeFail();
											MCUTimer1.ClearTimer();
											MCUTimer4.ClearTimer();
										}
									}
								}
                            }
                        }
                    }
                }
                else
                {
                    NB_DebounceAndQue( UPM_NB_INVERTER_STARTUP_FAILURE, false );
                    
//	                    if ( !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) )
					if ( !NB_GetNodebit( UPM_NB_INVERTER_AC_UNDER_VOLTAGE ) )	//Inv/Vo nobbit can not use, as always false
					{
						if(MCUTimer1.CheckTimeout( WAIT_200_MS ))	//add wait inv nobbit stable
						{
	                        TransferState( ONLINE_STATE );
						}
                    }
                    else if ( ( MCUTimer1.CheckTimeout( WAIT_500_MS )  &&
                                (OutputCurrentRMS.RawRMS.phA < 80)     &&
                                (OutputCurrentRMS.RawRMS.phB < 80)     &&
                                (OutputCurrentRMS.RawRMS.phC < 80) ) ||
                              NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT ) )
                    {
                        NB_DebounceAndQue( UPM_NB_INVERTER_STARTUP_FAILURE, true );
                        Inverter.Off();
                        ClearAllCommands();
                        MCUTimer1.ClearTimer();
                        
                        // command other UPM's in my UPS to clear all commands
                        ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
                        
                        if( bypassAvailable )
                        {
                            MCUStatus.bit.EmergencyTransferToBypass = 1;
                            TransferState( BYPASS_STANDBY_STATE );
                        }    
                        // clr start step
                        PhaseState = 0;
                    }
                }
            }
            // 2. Sys no Outp ACUV
            else
            {
                DeActivePullChain();
                PhaseState = 0; //clr it,as may value run under 1.sys no op.
				MCUStatus.bit.OutputSyncDisable = false;	//recover outputSync if others are startup already

                // joining parallel. future.
                if ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() )
                {
                    // load is energized, need to start inverter first, then close relays
                    if ( !iStat.bit.InverterOn )
                    {
						//2.1.to open rly first(may on under 1.sys no op)
						if (GetInverterRelayState() == RELAY_CLOSED)
						{			
							SetInverterRelay( RELAY_OPEN);			//1.open inv relay first
							MCUTimer1.ClearTimer();
							MCUTimer4.ClearTimer();
						}
						else
						{  		
							if(MCUTimer4.CheckTimeout(WAIT_100_MS))	//1.2 wait 100ms relay open
							{
			                    // load is energized, need to start inverter first, then close relays
			                    if ( !iStat.bit.InverterOn )
			                    {
			                        Inverter.On();
			                        MCUTimer1.ClearTimer();
			                    }
							}
						}						
                    }
                    else
                    {
                        if ( RELAY_OPEN == GetInverterRelayState() )
                        {
//	                            if ( ((InverterSyncedToOutput()&& IsMatchedOutput())                     || 
						   	if ( ((InverterSyncedToOutput()) 					|| 
                                   ( ParallelCan.ParGlobalAndData.SyncStatus.bit.BypassSyncAvail    &&
                                     InverterSyncedToBypass() ) )                                 &&
                                 MCUTimer1.CheckTimeout( FIVE_SECONDS ) )
                            {
                            	MCUStatus.bit.NormalXferInProgress = 1;
                            	if ( PrepareForInverterTransfer()   ||
                            	     !( ParallelCan.ParGlobalOrData.BypassStatus.bit.BypassState & BYPASS_FIRE_STATE ) )
                            	{
	                                SetInverterRelay( RELAY_CLOSED );
	                                MCUTimer1.ClearTimer();
									
//										InitDropPower = float(OutputkVARating) * 34.0f;  //InitDropPower is the phase VA. 34~= 100/3, 5/8 full kVA ~=21.25									
									InitDropPower = ParallelCan.SystemLoad * OutputkVARating / 3.0f;	
                            	}
                            }     
                        }
                        else
                        {
                            //In parallel, if only inverter ralay is closed, there should be some loop current,
                            //disable sync to output to avoid loop current increased
                             if( (std::fabs( Inverter.InverterCurrentDQ0Data.Sd) > 0.01f ) &&
                                (std::fabs( Inverter.InverterCurrentDQ0Data.Sq) > 0.01f )) //fix NEW3C3-172
                            {
                                MCUStatus.bit.OutputSyncDisable = true;
                            }
                            if ( MCUTimer1.CheckTimeout( WAIT_20_MS ) )
                            {							
                                TransferState( ONLINE_STATE );
                                return;
                            }
                        }
                    }    
                }
                else
                {
                    // should not be here yet...
                    ClearAllCommands();
                } 
            }
        }
        else
        {
            // if inverter is on, turn it off
            ShutdownInverter();
            MCUTimer1.ClearTimer();
            MCUStatus.bit.NormalXferInProgress = 0;
        }
    }
    else
    {
        // if inverter is on, turn it off
        ShutdownInverter();
        MCUTimer1.ClearTimer();
        MCUStatus.bit.NormalXferInProgress = 0;
    }

        // 
        // -->SHUTDOWN after logic power keep alive
        // 
    //fix APACTS-348,battery deep discharge when powershare.
	if( KeepAliveShutdown( ( rStat.bit.RectifierOnBattery || BatteryConverter.GetStatus().bit.BoostOn ) && 
									!NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )))
    {
	    MCUStatus.bit.KeepAliveTrip = 1;
        Rectifier.RectifierOff();
        TransferState ( SHUTDOWN_STATE );	
		DSPOutRegister.GpoC.bit.Supply_24VOff = 1;					
	}
        // 
        // -->SHUTDOWN if rectifier is off
        // 
    if ( (!NB_GetNodebit( UPM_NB_RECTIFIER_ON ) && MCUTimer6.CheckTimeout(WAIT_1_MS)) ||
        MCUStatus.bit.StandbyChargerOffCmd )
    {
        Rectifier.RectifierOff();
        TransferState ( SHUTDOWN_STATE );
    }
    
    if ( !MCUStatus.bit.BypassXferInProgress  &&
         !MCUStatus.bit.NormalXferInProgress  &&
         StartupMode != STARTUP_OFF )
    {
    	ResetTransferState();
    }
    
    if ( StartupMode == STARTUP_OFF )
    {
    	// allow other UPS's to transfer when I am staying off
    	// only set on zero cross in case I'm the last one to set this (common with rectifier failure)
    	PrepareForInverterTransfer();
    }
} // StandbyState


// ********************************************************************************************************
// *
// * Function: OnBypassState(void);
// *
// * Purpose:  On bypass.  The inverter is off.  Rectifier and charger are off. Bypass is supporting the load.
// *
// * Description:
// *
// ********************************************************************************************************
void MCUStateControl::OnBypassState(void)
{
    CheckForStuckRelay();
    
    bool systemInverterAvailable = ParallelCan.PCan_CheckSystemInverterAvailable();
    MCUStatus.bit.UPSInverterAvailable = ParallelCan.PCan_CheckUPSInverterAvailable();
    
    if ( !InverterAvailableDelay ) //wait for ouput ACUV to clear before disqualifying my inverter
    {
        // Let parallel system know if my inverter is available
        MCUStatus.bit.InverterAvailable = MCUStatus.bit.NormalCmd                                 &&
                                          BypToNormalPersistentOk()                               &&
                                          !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )         &&
                                          !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )    &&
                                          !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )        &&
                                          !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )         &&
                                          !NB_GetNodebit( UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY )    &&
                                          !NB_GetNodebit( UPM_NB_BATTERY_LOW )                    &&
                                          ( ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
                                              NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) ) ||
                                            EEP_BattNotRequiredToGoOnline() )                     &&
                                          !NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD )                &&
                                          !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )           &&
                                          !NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )          &&
                                          !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )     &&
                                          !NB_GetNodebit( UPM_NB_RECTIFIER_FAILED )               &&
                                          !NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )  &&
                                          !NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP ) &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE )             &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP );
    }
    else
    {
    	InverterAvailableDelay--;
    }
    
    BypassState().RequestBypassState( BYPASS_FIRE_STATE );
    
    if ( StayOnBypass )
    {
    	if (StayOnBypass < WAIT_20_MS)
    	{
	    	// Clear the ETB status bit some time after actually going to bypass, but
	    	// before allowing return to online, in order
    		// to ensure it gets transmitted over PCAN in both directions.
    		MCUStatus.bit.EmergencyTransferToBypass = 0;
    	}
        StayOnBypass--;
    }
    else
    {
	    MCUStatus.bit.EmergencyTransferToBypass = 0;
    }
    
        // If inverter is on, turn it off.
        // Wait 3ms for the STSW to turn on.
    if ( MCUStatus.bit.UnsynchronizedTransfer )
    {
    	ShutdownInverterForATB();
    }
    else
    {
	     if ( MCUTimer1.CheckTimeout( WAIT_3_MS ) )
	    {
	        ShutdownInverter();
	    }
    }


        // 
        // -->SHUTDOWN conditions:
        // 
    if ( ( !EEP_EPOTransferToBypass()                          &&
           NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) ) ||
         ( !MCUStatus.bit.BypassAvailableOnBypass && !Unit_Setup1.bit.ATBUnconditional ) )
    {
        MCUStatus.bit.BypassLoadOffHistory = 1;
        ClearAllCommands();
        TransferState( SHUTDOWN_STATE );
        return;
    }
    else if ( MCUStatus.bit.ShutdownCmd                       ||
              MCUStatus.bit.StandbyCmd                         )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		ResetTransferState();
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    	}
        
        // Syncronize user commands
        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            MCUStatus.bit.BypassLoadOffHistory = 1;
            TransferState( SHUTDOWN_STATE );
            return;
        }
    }
    else if ( NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) )
    {
            // Stay on bypass
        MCUStatus.bit.ShutdownXferInProgress = 0;
        return;
    }
    else if ( MCUStatus.bit.NormalCmd            &&
              !MCUStatus.bit.ToECTCmd            &&
              !MCUStatus.bit.ToBypassCmd         &&
              systemInverterAvailable            &&
              !MCUStatus.bit.UPSInverterAvailable )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		ResetTransferState();
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    	}
        if ( PrepareForBypassTransfer() )
        {
        	// don't need to check for UPMCalEnabled, this branch will never be taken in that case
        	// prevent immediate transfers back to bypass when BypassWithNormalCmd is set
        	StayLoadOff = ONE_SECOND;
            TransferState( SHUTDOWN_STATE );
            return;
        }
    }
    else
    {
    	MCUStatus.bit.ShutdownXferInProgress = 0;
    }
    
        // 
        // -->STANDBY(-->NORMAL) transfer conditions:
        // 
	bool UtilityOkayForRectifierStart =
	!NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) &&
	!NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE ) &&
	!NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
	!NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY );

	if( UtilityOkayForRectifierStart &&
        !Rectifier.GetStatus().bit.ACPreChargeFail &&
        !IsBatteryStart )
    {
	    if ( AnyRectifierOnCommand() )
	    {
			switch ( PhaseState )
			{
				default: // fall-through
				case 0:
				if ( Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE )
				{
				    // Make sure the rectifier is fully shut down from a canceled battery start
				    PhaseState++;
				}
				break;
				case 1:
				IsRectifierStart = true;    //startup on bypass
				break;
			}

			if( IsRectifierStart )	//IsRectifierStart may be set by shutdown state or bypass state
			{
				if ( !StartupRectifier() )
				{
					PhaseState = 0;
					//if ( !systemInverterAvailable )
					{
					   ClearAllCommands();
				   
					   // command other UPM to clear all commands
					   ParallelCan.TransmitCommandPacket( pcan::fw_clear_commands);
					}
				}
			}
	    }
	    else//abort startup procedure
	    {
			RectifierPhaseState = 0;
			RectifierStartupAttempts = 0;

			if ( ShutdownInverter() ) // true when inverter is shut down and relays opened
			{
			    Rectifier.RectifierOff();
			}
			PhaseState = 0;
			KeepAliveShutdown( false ); // always false in this branch because UtilityOkayForRectifierStart
	    }
    }
	else if( !UtilityOkayForRectifierStart && IsRectifierStart )
	{
		//If normal start is in processing and the utility interrupts, normal start is aborted. And no more battery start transfer.
		
		// Reset the startup attempts; problems on the utility side should not cause rectifier startup failed to log
		RectifierPhaseState = 0;
		RectifierStartupAttempts = 0;
		
//			if( AnyRectifierOnCommand() )
//			{
//				NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, true, 2 );
//				ClearAllCommands();
//				// command other UPM's in my UPS to clear all commands
//				ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
//			}

		if ( ShutdownInverter() )
		{
			Rectifier.RectifierOff();
		}

		if ( KeepAliveShutdown( NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
								!NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) )  && //If battery is disconnected, KeepAliveShutdown should not work.
			 MCUStateTimer.TimerValue() > ONE_SECOND ) // wait an additional second for when coming from standby (for relays)
		{
			MCUStatus.bit.KeepAliveTrip = 1;
            DSPOutRegister.GpoC.bit.Supply_24VOff = 1;
		}
	}
	else
	{
	    if ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )  &&
	         !NB_GetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE ) &&
	         !NB_GetNodebit( UPM_NB_BATTERY_LOW )     && // Don't do battery start with low battery
			 !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )	  &&
			( AnyBatteryConverterOnCommand() )               &&
	         !MCUStatus.bit.KeepAliveTrip ) // Prevents state loops after standby -> shutdown because of keep alive trip
	    {
	        IsBatteryStart = true;
	        //does precharge procedure,UPM_NB_BATTERY_DISCHARGING is set and DC reaches target voltage when battery converter is on           
	        if( !StartupBatteryConverter() )
	        {
	            ClearAllCommands();            
	            // command other UPM's in my UPS to clear all commands
	            ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
	        }
	    }
	    else
	    {
	        BatteryPhaseState = 0;
	        BatteryStartTimer.ClearTimer();
	        BatteryStartupAttempts = 0;

//		        if( AnyBatteryConverterOnCommand() )
//		        {
//		        	if(!NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE ))
//	        		{
//		 	            NB_SetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE, true, 2 );
//			            ClearAllCommands();
//			            // command other UPM's in my UPS to clear all commands
//			            ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
//	        		}
//		        }

	        if ( ShutdownInverter() )
	        {
	            Rectifier.RectifierOff();
	            BatteryConverter.CancelBatteryStart();
	            BatteryConverter.BoostOff();
	            IsBatteryStart = false; // allow rectifier start to process if input is okay
	        }
	        
	        if ( KeepAliveShutdown( NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
	        	                    !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ) && //If battery is disconnected, KeepAliveShutdown should not work.
	        	 MCUStateTimer.TimerValue() > ONE_SECOND ) // wait an additional second for when coming from standby (for relays)
	        {
	            MCUStatus.bit.KeepAliveTrip = 1;
                DSPOutRegister.GpoC.bit.Supply_24VOff = 1;
	        }
	    }
	}


		//
		// -->BYPASS_STANDBY if rectifier is ON 
		//
	if ( NB_GetNodebit( UPM_NB_RECTIFIER_ON ) && (!IsBatteryStart ))
	{
		if ( MCUTimer3.CheckTimeout( ONE_SECOND ) )
		{
			TransferState( BYPASS_STANDBY_STATE );
			return;
		}
	}
	else if(IsBatteryStart) //if battery start
	{
		float DCLinkRampupCompleteVoltage = 5.0;
//			float DCLinkVoltage = DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered;
//			float dclinkerror =std::fabs( ( Rectifier.GetDCLinkVoltageRef() - 10.0f ) - DCLinkVoltage );
		float dclinkerrorP = ((Rectifier.GetDCLinkVoltageRef() - 10.0f)*0.5f) - DCLinkVoltagePositive.FastFiltered;
		float dclinkerrorN = ((Rectifier.GetDCLinkVoltageRef() - 10.0f)*0.5f) + DCLinkVoltageNegative.FastFiltered;
	
		if ( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
			 ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE ) &&
			 ( dclinkerrorP < DCLinkRampupCompleteVoltage ) &&
			 ( dclinkerrorN < DCLinkRampupCompleteVoltage ))
		{
			if ( MCUTimer2.CheckTimeout( ONE_SECOND ) )
			{
				MCUTimer2.ClearTimer();
				TransferState( BYPASS_STANDBY_STATE );
				IsBatteryStart = false;
			}
		}
		else
		{
			MCUTimer2.ClearTimer();
		}
		MCUTimer3.ClearTimer();
	}
	else
	{
		MCUTimer2.ClearTimer();
		MCUTimer3.ClearTimer();
	}

    
    if ( !MCUStatus.bit.ShutdownXferInProgress )
    {
        // Clear out transition timers
        ResetTransferState();
    }

} // OnBypassState


// ********************************************************************************************************
// *
// * Function: OnBypassStandbyState(void);
// *
// * Purpose: On bypass. Rectifier and charger are on.  Bypass is supporting the load.
// *
// * Description:
// *
// ********************************************************************************************************
void MCUStateControl::OnBypassStandbyState(void)
{
    uInverterStatus iStat = Inverter.GetStatus();
    uRectifierStatus rStat = Rectifier.GetStatus();
    
    bool systemInverterAvailable = ParallelCan.PCan_CheckSystemInverterAvailable();
	MCUStatus.bit.UPSInverterAvailable = ParallelCan.PCan_CheckUPSInverterAvailable();	//1.wait all upms inv avail
    
    CheckForStuckRelay();

//	    BypassState().RequestBypassState( BYPASS_FIRE_STATE );
	//for Bat Ocp
	if(!EtbToOffRelayForBatOcp)
	{		
		BypassState().RequestBypassState( BYPASS_FIRE_STATE ); 	//1. about 2.25k->then send to PIC			

		MCUTimer8.ClearTimer();
	}			
	else
	{
		if(RELAY_OPEN == GetInverterRelayState())		
		{
			if( MCUTimer8.CheckTimeout(WAIT_10_MS) )		//45=10ms/0.222ms*
			{
				MCUTimer8.ClearTimer();
				BypassState().RequestBypassState( BYPASS_FIRE_STATE );

				EtbToOffRelayForBatOcp = false;					
			}
		}
		else
		{
			MCUTimer8.ClearTimer();
		}
	}		
	//end
	
    if ( StayOnBypass )
    {
    	if (StayOnBypass < WAIT_20_MS)
    	{
	    	// Clear the ETB status bit some time after actually going to bypass, but
	    	// before allowing return to online, in order
    		// to ensure it gets transmitted over PCAN in both directions.
    		MCUStatus.bit.EmergencyTransferToBypass = 0;
    	}
        StayOnBypass--;
    }
    else
    {
	    MCUStatus.bit.EmergencyTransferToBypass = 0;
    }
    
    if ( !InverterAvailableDelay ) //wait for battery converter to start or output ACUV to clear before disqualifying my inverter
    {
        // Let parallel system know if my inverter is available
        MCUStatus.bit.InverterAvailable = MCUStatus.bit.NormalCmd                             &&
                                          BypToNormalOk()                                     &&
                                          !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE )         &&
                                          !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )    &&
                                          ( FailedInverterTransfers < INVERTER_INHIBIT_TRANSFER_LIMIT );
    }
    else
    {
    	InverterAvailableDelay--;
    }

        // 
        // -->SHUTDOWN conditions:
        // 
    if ( (!EEP_EPOTransferToBypass() &&
          NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )) )
    {
        TransferState( SHUTDOWN_STATE );
        return;
    }
    else if ( MCUStatus.bit.ShutdownCmd )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
	        MCUStatus.bit.ShutdownXferInProgress = 1;
	        ResetTransferState();
    	}
        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( SHUTDOWN_STATE );
            return;
        }
    }
    else if ( !MCUStatus.bit.BypassAvailableOnBypass && !Unit_Setup1.bit.ATBUnconditional )
    {
    	ClearAllCommands();
        TransferState( STANDBY_STATE );
        return;
    }
    else if ( MCUStatus.bit.StandbyCmd )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    		ResetTransferState();
    	}
        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( STANDBY_STATE );
            return;
        }
    }
    else if ( MCUStatus.bit.NormalCmd            &&
              !MCUStatus.bit.ToECTCmd            &&
              !MCUStatus.bit.ToBypassCmd         &&
              systemInverterAvailable            &&
              !MCUStatus.bit.UPSInverterAvailable )
    {
    	//other UPS's in a redundant system are transferring to normal, but my UPS needs to shut off
		if ( MCUStatus.bit.NormalXferInProgress == 1 )
		{
			//a upm in this ups failed the normal transfer, cancel my normal transfer
			Inverter.Off();
			MCUStatus.bit.NormalXferInProgress = 0;
			Inverter.MatchBypassVoltage( false );
		}
		if ( !MCUStatus.bit.ShutdownXferInProgress )
		{
			MCUStatus.bit.ShutdownXferInProgress = 1;
			ResetTransferState();
		}
        if ( PrepareForBypassTransfer() )
        {
        	// don't need to check for UPMCalEnabled, this branch will never be taken in that case
        	// prevent immediate transfers back to bypass when BypassWithNormalCmd is set
        	StayLoadOff = ONE_SECOND;
            TransferState( STANDBY_STATE );
            return;
        }
    }
    else
    {
        MCUStatus.bit.ShutdownXferInProgress = 0;
    }

        // 
        // -->NORMAL conditions:
        // 
    if ( !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) && 
         MCUStatus.bit.NormalCmd            &&
         !MCUStatus.bit.ToECTCmd            &&
         !MCUStatus.bit.ToBypassCmd         &&
         !StayOnBypass                      &&
         BypToNormalOk()                    &&
         ( FailedInverterTransfers < INVERTER_INHIBIT_TRANSFER_LIMIT ) &&
         systemInverterAvailable            &&
         MCUStatus.bit.UPSInverterAvailable &&
         InverterSyncedToBypass())
    {
        if ( !MCUStatus.bit.NormalXferInProgress )
        {
            if(Rectifier.GetStatus().bit.ACPreChargeFail == false)
            {   // 1.line normal on
                Inverter.On();
                Rectifier.ClearACPreChargeFail();
                ResetTransferState();
                MCUStatus.bit.NormalXferInProgress = 1;
            }
            else
            {   // 2.line prechg fail, to bat mode on
                if(Rectifier.InputAvailable() == false)
                {
                    Inverter.On();
                    Rectifier.ClearACPreChargeFail();
                    ResetTransferState();
                    MCUStatus.bit.NormalXferInProgress = 1;
                }
                else
                {
                    // for jira412, bat connect, wait walkin end, then turn on
                    if( (( Rectifier.GetState() == RECTIFIER_NORMAL_STATE ) && ( BatteryConverter.GetBatteryState() != BATTERY_ON_BOOST_STATE )) ||
                        MCUTimer7.CheckTimeout(FIVE_SECONDS ) )	
                    {
                        Inverter.On();
                        Rectifier.ClearACPreChargeFail();
                        ResetTransferState();
                        MCUStatus.bit.NormalXferInProgress = 1;	
                        MCUTimer7.ClearTimer();
                    }
                }
            }
        }

        MCUTimer1.ClearTimer();

        Inverter.MatchBypassVoltage( true );
        
        if ( MCUTimer3.CheckTimeout( INVERTER_BYPASS_MATCHING_TIMEOUT ) ||
             Inverter.IsMatchedBypass() )
        {
            if ( iStat.bit.InverterOn                               &&
                 !NB_GetNodebit( UPM_NB_INVERTER_AC_UNDER_VOLTAGE ))
            {
                NB_DebounceAndQue( UPM_NB_INVERTER_STARTUP_FAILURE, false );
                if ( PrepareForBypassTransfer() || UPMCalEnabled )
                {
                    TransferState( ONLINE_STATE );
                }
            }
            else
            {
                Inverter.Off();
                MCUStatus.bit.NormalXferInProgress = 0;
                Inverter.MatchBypassVoltage( false );
                NB_DebounceAndQue( UPM_NB_INVERTER_STARTUP_FAILURE, true );
                
                if ( !systemInverterAvailable )
                {
	                ClearAllCommands();
	                // command other UPM to clear all commands
	                ParallelCan.TransmitCommandPacket( pcan::fw_clear_commands);
                }
            }
        }
    }
    else
    {
        if ( MCUStatus.bit.NormalXferInProgress && !BypToNormalOk() )
        {
            if( FailedInverterTransfers < INVERTER_INHIBIT_TRANSFER_LIMIT )
            {
            	FailedInverterTransfers++;
            	InverterInhibitNumBypassTransfers++;
            	MCUStatus.bit.NormalXferInProgress = 0;
            }
            if ( ( FailedInverterTransfers >= INVERTER_INHIBIT_TRANSFER_LIMIT ) && !systemInverterAvailable )
            {
                MCUStatus.bit.ToBypassCmd = 1;        // Lock on bypass using ToBypassCmd
                MCUStatus.bit.ToBypassCmdHistory = 1; // Don't log, no user command
                MCUStatus.bit.NormalCmd = 0;
            }
        }
        else
        {
        	MCUStatus.bit.NormalXferInProgress = 0;
        }
        
        // This clears the normal command for persistent conditions preventing transfer
        if ( !BypToNormalPersistentOk() && !systemInverterAvailable )
        {
            MCUStatus.bit.NormalCmd = 0;
        }

        MCUTimer3.ClearTimer();   
    }

    //
    //--> ECT  conditions:  
    //
    if ( MCUStatus.bit.ToECTCmd                                 &&
         !MCUStatus.bit.ToBypassCmd                             &&
         !MCUStatus.bit.NormalCmd                               &&
         !MCUStatus.bit.InhibitToECT                            &&
         BypToECTPersistentOk()    
        )
    {
        if ( !MCUStatus.bit.ECTXferInProgress )
        {
            MCUStatus.bit.ECTXferInProgress = 1;
        }

        if ( MCUTimer4.CheckTimeout( ONE_SECOND * 2 ) )
        {
            if ( (RELAY_CLOSED   == GetInverterRelayState())            &&
                 (BypassState().GetBypassState() == BYPASS_FIRE_STATE ) &&
                 InverterSyncedToBypass() )
            {
                if ( PrepareForBypassTransfer() || UPMCalEnabled )
                {
                    Inverter.OnECTCurrentLoop();
                    TransferState( EASY_CAPACITY_TEST_STATE );
                }
            }
            else
            {
                ShutdownInverter();
                MCUStatus.bit.ECTXferInProgress = 0;
                MCUStatus.bit.InhibitToECT = 1;
                MCUStatus.bit.ToECTCmd = 0;
//                MCUStatus.bit.NLFLEnable = 0;
                ClearAllCommands();

                ParallelCan.TransmitCommandPacket( pcan::fw_clear_commands);
            }
        }
    }
    else
    {
        if ( MCUStatus.bit.ECTXferInProgress && !BypToECTPersistentOk() )
        {
            MCUStatus.bit.InhibitToECT = 1;
            MCUStatus.bit.ToECTCmd = 0;
//          MCUStatus.bit.NLFLEnable = 0;
        }

        MCUStatus.bit.ECTXferInProgress = 0;
        MCUTimer4.ClearTimer();
    }

    if ( !MCUStatus.bit.NormalCmd  &&
         !MCUStatus.bit.ToECTCmd  )
    {
        MCUStatus.bit.ToBypassCmd = 1;        // Lock on bypass using ToBypassCmd
        MCUStatus.bit.ToBypassCmdHistory = 1; // Don't log, no user command
    }
 
		//for JIRA WOMBAT-184 begin, Bat Ocp  
		if(!EtbToOffRelayForBatOcp) 	
		{
		    if ( !MCUStatus.bit.NormalXferInProgress   &&
		         !MCUStatus.bit.ECTXferInProgress
		       )
		    {
		    	if ( MCUStatus.bit.UnsynchronizedTransfer )
		    	{
		    		ShutdownInverterForATB();
		    	}
		    	else
		    	{
			      // Wait 3ms for the STSW to turn on, then inverter off
			        if ( MCUTimer1.CheckTimeout( WAIT_3_MS ) )
			        {
		                // if inverter is on, turn it off
			            ShutdownInverter();
			        }
		    	}
		    }
		}
		else
		{
			if ( !MCUStatus.bit.NormalXferInProgress   &&
				 !MCUStatus.bit.ECTXferInProgress
			   )
			{
				ShutdownInverterForATB(); //imeda off relay						
			}
		}

    //
    // -->BYPASS if rectifier is OFF
    ////fix APACTS-348,battery deep discharge when powershare.
	if( KeepAliveShutdown( ( rStat.bit.RectifierOnBattery || BatteryConverter.GetStatus().bit.BoostOn ) && 
									!NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )))
    {
    	MCUStatus.bit.KeepAliveTrip = 1;
        Rectifier.RectifierOff();
        DSPOutRegister.GpoC.bit.Supply_24VOff = 1;
        TransferState ( BYPASS_STATE );
    }
    else if ( !NB_GetNodebit( UPM_NB_RECTIFIER_ON ) )
    {
        if ( MCUTimer5.CheckTimeout( ONE_SECOND ) )
        {
            Rectifier.RectifierOff();
            TransferState ( BYPASS_STATE );
        }
    }
    else if ( MCUStatus.bit.StandbyChargerOffCmd )
    {
        MCUTimer5.ClearTimer();
        Rectifier.RectifierOff();
        TransferState ( BYPASS_STATE );
    }
    else
    {
        MCUTimer5.ClearTimer();
    }
    
    if ( !MCUStatus.bit.NormalXferInProgress   &&
         !MCUStatus.bit.ShutdownXferInProgress &&
         !MCUStatus.bit.ECTXferInProgress
       )
    {
        ResetTransferState();
    }
}


// ********************************************************************************************************
// *
// * Function: OnlineState(void);
// *
// * Purpose:  Inverter is supporting the load.
// *
// * Description: 
// *
// ********************************************************************************************************
void MCUStateControl::OnlineState(void)
{
    bool loadPowerOff = false;
    bool outputOverloadETBimmediate = false;

    SetInverterRelay( RELAY_CLOSED );
    MCUStatus.bit.PostESSMode = false;

	// UL safety: Do not parallel bypass with inverter for > 10 seconds.  Also acts as a final failsafe
	// to ensure that when one node performs an ETB, that eventually everyone else does too.
	BypassParallelingInverter.Debounce(ParallelCan.ParGlobalOrData.BypassStatus.bit.BypassState & BYPASS_FIRE_STATE);

    if ( OutputMonitorDelay )
    {
        if ( MCUTimer2.CheckTimeout( OutputMonitorDelay ) )
        {
            OutputMonitorDelay = 0;
        }
    }

    if ( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP ) &&
		 ( NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_A ) ||
		   NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_B ) ||
		   NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_C ) ||
		   NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_A ) ||
		   NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_B ) ||
		   NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_C ) ) )
	{
		outputOverloadETBimmediate = true;
		MCUStatus.bit.OutputOverloadETB = 1;
	}

    if ( ForceStayOnline )
    {
        if ( MCUTimer5.CheckTimeout( ForceStayOnline )            ||
			 NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )         ||
			 NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )        ||
			 NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )    ||
			 NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )   ||
			 NB_GetNodebit( UPM_NB_DRIVER_FAULT )                 ||
			 outputOverloadETBimmediate                           ||
			 ( NB_GetNodebit( UPM_NB_MBS_CLOSED ) &&
			   !NB_GetNodebit( UPM_NB_MOB_OPEN )    )             ||
			 NB_GetNodebit( UPM_NB_PULL_CHAIN ) )
        {
            ForceStayOnline = 0;
        }
    }
    
    if( UtilityTransientDetDelay )
    {
    	UtilityTransientDetDelay --;
    }
    
        // We can open bypass SCRs and stop bypass matching when there are enough 
        // UPMs ready or after timeout = TBD (parallel).
        // Emergency transfer back to bypass if not enough ready after timeout.
    if ( MCUStateTimer.TimerValue() > RelayTime )     //get relay time from eep
    {
        if ( NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE ) ||
             ( ParallelCan.TotalNumberOfBypass > 1 )      ||
             FrequencyConverterMode )   //frequency converter mode, reset DSP should transfer to load off state
                                        //so bypass pic should be ready state instead of primed state
        {
            BypassState().RequestBypassState( BYPASS_READY_STATE );
        }
        else
        {
            BypassState().RequestBypassState( BYPASS_PRIMED_STATE );
        }
    }

    if ( (MCUStateTimer.TimerValue() > WAIT_500_MS) &&
         ( (BypassState().GetBypassState() != BYPASS_PRIMED_STATE) &&
           (BypassState().GetBypassState() != BYPASS_READY_STATE ) ) )
    {
        if ( MCUStateTimer.TimerValue() > (WAIT_100_MS * 6) )
        {
            MCUStatus.bit.BypReturnFailed = false;
        }
        else
        {
            MCUStatus.bit.BypReturnFailed = true;
        }
    }
    else
    {
        MCUStatus.bit.BypReturnFailed = false;
    }

        // 
        // Hard conditions to allow bypass transfer
        // 
    bool bypassAvailable = ( !ForceStayOnline                              &&
                             !FrequencyConverterMode                       &&
                             NB_GetNodebit( UPM_NB_BYPASS_INSTALLED)       &&
                             !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE ) &&
                             !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE ));
    MCUStatus.bit.BypassAvailable = bypassAvailable;

        //
        // Soft conditions which also must be true to allow manual bypass transfers
        // 
    bool bypassManuallyAvailable = bypassAvailable &&
                                   ParallelCan.PCan_CheckSystemBypassAvailable();

        //
        // -->Parallel State Sync
        // 
    if (ParallelCan.ParallelCommand.bit.para_inverter_off_sync &&
        !MCUStatus.bit.StandbyCmd && 
        !MCUStatus.bit.ToBypassCmd &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)
    {        
        if( !InverterSyncedToBypass() &&
            ATBEnabled                &&
            bypassAvailable             )
        {
			//move protect condict inside, to avoid mistake run next else.
			//for jira Wombat475,paral some byp avail,some not avail.ATB enable active,need Vbus>(Vbyp_max-50.0f),to avoid byp pour bus,damage inv
			if(ATBEnabled && (((DCLinkVoltagePositive.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f) &&		//Vbus-Vbyp_max>-50V 
							((-DCLinkVoltageNegative.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f)))		  
			{
	            Inverter.Off();
	            ShutdownInverterForATB();//InverterRelay OPEN
	            if (OutputPLL.SourceNominalDQO.Sd < 0.85f)//Wait for inverter to turn off
	            {
	                if ( ATBTimer.CheckTimeout(atb_gap_cycles) )
	                {
	                	ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;
	                	MCUStatus.bit.UnsynchronizedTransfer = 1;
	                    MCUStatus.bit.EmergencyTransferToBypass = 1;
	                    TransferState( BYPASS_STANDBY_STATE );
	                }
	            }

			}
			else
			{
				//local to stanby mode, and not set ETB flag, to avoid: stanby to byp 
				ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;    
				
				MCUStatus.bit.NormalCmd = 0;
				TransferState( STANDBY_STATE );
			}			
        }
        else if( bypassAvailable )
        {
        	ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;
            MCUStatus.bit.EmergencyTransferToBypass = 1;
            //MCUStatus.bit.UnsynchronizedTransfer = !InverterSyncedToBypass();
            TransferState( BYPASS_STANDBY_STATE );
        }
        else
        {
        	ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 0;
        	MCUStatus.bit.NormalCmd = 0;
            TransferState( STANDBY_STATE );
        }
        
        return;
    }
  
        // 
        // -->SHUTDOWN transfer conditions (system redundant, no load dump):
        // 
    if ( NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT )               &&
         ( NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE )          ||
           NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )     ||
	       NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP )    ||
	       NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )           ||
           NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )         ||
           NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )        ||
           NB_GetNodebit( UPM_NB_DRIVER_FAULT )                      ||
           NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )              ||
           NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )             ||
		   //this line,left from ETB condi 										
		   NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )	 	 ||           
           (Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE) ) )
    {
        ParallelCan.TransmitToUPMsInUPS( pcan::fw_redundant_off_command, 0, 0, 0, 0 );
    	// check for Output ACUV for one second after this transfer in case all
    	// UPS's trip off at the same time; if so an ETB will occur from shutdown
    	CheckOutputACUV = ONE_SECOND;
        TransferState( SHUTDOWN_STATE );
        return;
    }
    else if ( NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT )      &&
              BatteryConverter.CheckBatteryUVShutDown() )
    {
    	ParallelCan.TransmitToUPMsInUPS( pcan::fw_redundant_battery_off_command, 0, 0, 0, 0 );
    	MCUStatus.bit.NormalCmd = 0;
        TransferState( STANDBY_STATE );
        return;
    }
    else if ( ParallelCan.ParallelCommand.bit.para_redundant_off )
    {
    	ParallelCan.ParallelCommand.bit.para_redundant_off = 0;
    	// check for Output ACUV for one second after this transfer in case all
    	// UPS's trip off at the same time; if so an ETB will occur from shutdown
    	CheckOutputACUV = ONE_SECOND;
    	TransferState( STANDBY_STATE );
    	return;
    }
    else if ( ParallelCan.ParallelCommand.bit.para_redundant_battery_off )
    {
    	ParallelCan.ParallelCommand.bit.para_redundant_battery_off = 0;
    	MCUStatus.bit.NormalCmd = 0;
    	MCUStatus.bit.StoreAutoRestart = 1;
    	TransferState( STANDBY_STATE );
    	return;
    }

        // 
        // -->ETB (Emergency Transfer on Bypass) conditions:
        //

		//1)Bat Overload level4 or 2)Bat Ocp
		if( NB_GetNodebit( UPM_NB_BATTERY_CURRENT_LIMIT)		|| 
			( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP ) && 		   
			    (NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_A ) ||
		   		NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_B ) ||
		   		NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_C ))
			&& ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE )))	
		{
			if(bypassAvailable)
			{
				if(BypassPLL.GetSourceNormFactor() > 0.0f)
				{
					if(((DCLinkVoltagePositive.FastFiltered-BypassPLL.SourceDQODataSlowSd) < -50.0f) || 	//Vbus-Vbyp_max<-50V, en  
						((-DCLinkVoltageNegative.FastFiltered-BypassPLL.SourceDQODataSlowSd) < -50.0f)) 		
					{
						EtbToOffRelayForBatOcp = true;	
					}
				}						
			}
		}
		//end

    if ( ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass                                   ||
         BypassParallelingInverter.GetState()                                                                  ||
         ( ( NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE ) ||
             ( NB_GetNodebit( UPM_NB_MBS_CLOSED )  &&
               !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) && 
           MCUStatus.bit.BypassAvailableOnBypass && bypassAvailable )                                          ||
         ( ( bypassAvailable || ( Unit_Setup1.bit.ATBUnconditional && !ForceStayOnline ) ) &&
           ( !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE ) )     &&
           ( ( NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) && !OutputMonitorDelay &&
               ( !DisableImmBypassTransfer || ETBDelayTimer.CheckTimeout(MS_TO_WAIT(ImmBypassTransferDelay)))) ||
             NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )                                                    ||
             MCUStatus.bit.OutputOverloadETB                                                                   ||
             NB_GetNodebit( UPM_NB_PULL_CHAIN )                                                                ||
             ( NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) && EEP_EPOTransferToBypass() )               ||
             NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )                                                 ||
             NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )                                                ||
             NB_GetNodebit( UPM_NB_DRIVER_FAULT )                                                              ||                    
             NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )                                                      ||
             NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )                                                     ||
			 !( Rectifier.RectifierOnReal || BatteryConverter.BatteryAvailable() )						   	   ||  //JIRA:WOMBAT-164 , Nothing is keeping the DC link up(add follow 9P)             
             NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )                                             ||
             NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP )                                            ||
             NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )                                                   ||
			 NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )													   ||
			 ParallelCan.ParGlobalOrData.McuStatus.bit.StaticSwitchShort									   ||
             NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE )                                                  ||
             NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )                                              ||
             MCUStatus.bit.BypReturnFailed                                                                     ||
			 ( NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )  && 
                ( MCUStatus.bit.ToBypassCmd                      ||  
                  NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )   ||  
                  BatteryConverter.CheckBatteryUVShutDown() ) )                                                ||
             ( !NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT ) &&
               Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE &&
               ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                 !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) ) )
           )
         )
       )
    {
        if(bypassAvailable && InverterSyncedToBypass() )   // normal transfer with ETB
        {
            MCUStatus.bit.EmergencyTransferToBypass = 1;
            TransferState( BYPASS_STANDBY_STATE );
            return;
        }
        //ATB with ETB
        else if( ATBEnabled  &&
                ( bypassAvailable || Unit_Setup1.bit.ATBUnconditional ) &&
                ( !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE ) ) )
        {
//	            Inverter.Off();
//	            ShutdownInverterForATB();//InverterRelay OPEN
//	            if (OutputPLL.SourceNominalDQO.Sd < 0.85f ||	//Wait for inverter to turn off
//	                ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)//if other ups is onbypass, don't wait,fix APACTS-276
//	            {
//	                if ( ATBTimer.CheckTimeout(atb_gap_cycles) )
//	                {
//	                	MCUStatus.bit.UnsynchronizedTransfer = 1;
//	                    MCUStatus.bit.EmergencyTransferToBypass = 1;
//	                    TransferState( BYPASS_STANDBY_STATE );
//	                }
//	            }
//	            else
//	            {
//	            	ATBTimer.CheckTimeout(atb_gap_cycles);
//	            }
//	            return;

			//move protect condict inside, to avoid mistake run next else.
			//for jira Wombat475,paral some byp avail,some not avail.ATB enable active,need Vbus>(Vbyp_max-50.0f),to avoid byp pour bus,damage inv
			if(ATBEnabled && (((DCLinkVoltagePositive.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f) &&		//Vbus-Vbyp_max>-50V 
							((-DCLinkVoltageNegative.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f)))		  
			{
				Inverter.Off();
                ParallelCan.TransmitCommandPacket( pcan::fw_inverter_off_sync_command);
				ShutdownInverterForATB();//InverterRelay OPEN				
				if (OutputPLL.SourceNominalDQO.Sd < 0.85f ||	//Wait for inverter to turn off
					ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)//if other ups is onbypass, don't wait,fix APACTS-276
				{
					if ( ATBTimer.CheckTimeout(atb_gap_cycles) )
					{
						MCUStatus.bit.UnsynchronizedTransfer = 1;
						MCUStatus.bit.EmergencyTransferToBypass = 1;
						TransferState( BYPASS_STANDBY_STATE );
					}
				}
	        }
	        else
	        {
	        	MCUStatus.bit.NormalCmd = 0;
	            TransferState( STANDBY_STATE );
	        }
			return;
        }
    }
    else if (!NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ))
    {
        ETBDelayTimer.ClearTimer();
    }

    //set toECTCmd for ECT restart
	if( AbnormalExitECT             &&
		( ECTRestartNumber > 0 )    &&
		BypToECTPersistentOk()      &&
		InverterSyncedToBypass() )  //need because in bypass standby state if enter condition fail, toECTcmd will be clear
	{
		MCUStatus.bit.ToECTCmd = 1;
		MCUStatus.bit.NormalCmd = 0;
	}

        //
        // -->BYPASS transfer conditions:
        // 
    if ( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )                  &&   //GPE-1341
         ( bypassManuallyAvailable            || 
           ( Unit_Setup1.bit.ATBUnconditional && !ForceStayOnline ) ) &&
         ( !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE ) )         &&
         ( MCUStatus.bit.ToBypassCmd                                ||
           NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )             ||
           BatteryConverter.CheckBatteryUVShutDown()                ||
           ( MCUStatus.bit.ToECTCmd && InverterSyncedToBypass() ) )
        )
    {
        if( bypassManuallyAvailable && InverterSyncedToBypass() )
        {
            // Perform synchronized transfer if possible
            if ( !MCUStatus.bit.BypassXferInProgress )
            {
               MCUStatus.bit.BypassXferInProgress = 1;
               ResetTransferState();
            }
            
            Inverter.MatchBypassVoltage( true );
            
            if ( MCUTimer1.CheckTimeout( INVERTER_BYPASS_MATCHING_TIMEOUT ) ||
                 Inverter.IsMatchedBypass() )
            {
                if ( PrepareForBypassTransfer()  || UPMCalEnabled )
                {
                    if( MCUStatus.bit.ToECTCmd )
                    {
                        Inverter.Off();
                        Inverter.MatchBypassVoltage( false );
                    }
                    TransferState( BYPASS_STANDBY_STATE );                
                }
            }
            if( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP ) ||
                BatteryConverter.CheckBatteryUVShutDown() )
            {
                // However, only run in output overload or battery UV for a maximum of 2 seconds before
                // performing unsynchronized transfer to bypass
                if( MCUTimer6.CheckTimeout( ONE_SECOND * 2 ) )
                {
                    TransferState( BYPASS_STANDBY_STATE );
                    // command other UPMs to bypass
                    MCUStatus.bit.EmergencyTransferToBypass = 1;
                }
            }
            // return placed outside of the TransferState to prevent transfers to ESS 
            return;
        }
		else if( ATBEnabled     &&
                ( ATBBypassCMDEnable                           ||
                  NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP ) ||
                  BatteryConverter.CheckBatteryUVShutDown() ) )
        {
            if(!Unit_Setup1.bit.ATBUnconditional)
        	{
			    ForceSyncBaseToByp = 1;
			}

            if( Unit_Setup1.bit.ATBUnconditional || MCUTimer8.CheckTimeout( FIVE_SECONDS ) )
        	{
				//move protect condict inside, to avoid mistake run next else.
				//for jira wombat 475,paral some byp avail,some not avail.ATB enable active,need Vbus>(Vbyp_max-50.0f),to avoid byp pour bus,damage inv
				if(ATBEnabled && (((DCLinkVoltagePositive.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f) &&		//Vbus-Vbyp_max>-50V 
								((-DCLinkVoltageNegative.FastFiltered-BypassPLL.SourceDQODataSlowSd)>-50.0f)))		  
				{				
	        		Inverter.Off();
	                ParallelCan.TransmitCommandPacket( pcan::fw_inverter_off_sync_command);
					ShutdownInverterForATB();//InverterRelay OPEN
					if (OutputPLL.SourceNominalDQO.Sd < 0.85f ||	//Wait for inverter to turn off
			            ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass)//if other ups is onbypass, don't wait,fix APACTS-276
					{
						if ( ATBTimer.CheckTimeout(atb_gap_cycles) )
						{
							MCUStatus.bit.UnsynchronizedTransfer = 1;
							MCUStatus.bit.EmergencyTransferToBypass = 1;
							TransferState( BYPASS_STANDBY_STATE );
						}
					}

				}
				else
				{	
					//local to stanby mode, and not set ETB, to avoid: stanby to byp 
					TransferState( STANDBY_STATE );
				}
				return;					
        	}
        }
        
    }
    else
    {
    	ForceSyncBaseToByp = 0;
        MCUStatus.bit.BypassXferInProgress = 0;
        MCUTimer1.ClearTimer();
        MCUTimer6.ClearTimer();
        MCUTimer8.ClearTimer();
    }
    
    
        // 
        // -->STANDBY transfer conditions (load dump):
        // Each condition is also covered by ETB or Bypass transfer conditions.
        // Dump the load only after the other blocks have failed.
        // 
    loadPowerOff = ( NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )        ||
                     NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )         ||
                     NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )  ||
                     NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )        ||
                     MCUStatus.bit.OutputOverloadETB                        ||
                     BatteryConverter.CheckBatteryUVShutDown() );

    if ( NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )         ||
         NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )        ||
		 !( Rectifier.RectifierOnReal || BatteryConverter.BatteryAvailable() )						  ||  //JIRA:WOMBAT-164 , Nothing is keeping the DC link up(add follow 9P)         
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )    ||
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )   ||
         NB_GetNodebit( UPM_NB_DRIVER_FAULT )                 ||
         outputOverloadETBimmediate                           ||         
         ( loadPowerOff && MCUTimer3.CheckTimeout( ONE_SECOND * 3 ) ) ||
         ( NB_GetNodebit( UPM_NB_MBS_CLOSED ) &&
           !NB_GetNodebit( UPM_NB_MOB_OPEN )    )             ||
         NB_GetNodebit( UPM_NB_PULL_CHAIN ) 				  ||
		 NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )   //for jira wombat469, valtest2.8,out short(line,no byp),unit mistake keep in online
         )
    {
    	// Immediate load dump, unless other UPMs have bypass available
    	MCUStatus.bit.NormalCmd = 0;
    	ParallelCan.TransmitCommandPacket( pcan::fw_inverter_off_sync_command );
        TransferState( STANDBY_STATE );
        return;
    }
    else if ( ParallelCan.ParallelStatus.bit.UpsOffCmd )
    {
    	MCUStatus.bit.NormalCmd = 0;
    	MCUStatus.bit.StandbyCmd = 1; //if the remote UPS on is active, load off the UPS would be fail. Thus add this
    	TransferState( STANDBY_STATE );
    }
    else if ( MCUStatus.bit.StandbyCmd )
    {
    	// Synchronized load dump
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    		ResetTransferState();
    	}
        
        if ( PrepareForInverterTransfer() || UPMCalEnabled )
        {
            TransferState( STANDBY_STATE );
            return;
        }
    }
    else if ( !loadPowerOff )
    {
        MCUStatus.bit.ShutdownXferInProgress = 0;
        MCUTimer3.ClearTimer();
    }
    
        // 
        // -->SHUTDOWN transfer conditions (load dump):
        // 
    if ( MCUStatus.bit.ShutdownCmd                            ||
         NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )   ||
         (Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE) )
    {
        TransferState( SHUTDOWN_STATE );
        return;
    }

        // 
        // -->ESS/ECO transfer conditions:
        // 
    ESSFaultCondition = MCUStatus.bit.ESSLockout                 * 0x0001 +
                    !InverterSyncedToBypass()                    * 0x0002 +
                    ( 0 != ForceStayOnline )                     * 0x0004 +
                    ( Abm().GetState() == ABM_CHARGE ||
                      ( Abm().GetState() == ABM_RESET && !ABMDisabled ) ||
                      Abm().GetState() == ABM_DISCHARGING )      * 0x0008 +
                    !Abm().GetStatus().bit.BelowHEChargeLimit    * 0x0010 +
                    ( !MCUStatus.bit.BypassAvailableOnBypass ||
                      !ParallelCan.PCan_CheckSystemBypassAvailable() ) * 0x0020 +
                    ( !NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )          ||
                      NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )      ||
                      NB_GetNodebit( UPM_NB_BATTERY_TEST_IN_PROGRESS )    ||
                      NB_GetNodebit( UPM_NB_BATTERY_NEEDS_SERVICE )       ||
                      !NB_GetNodebit( UPM_NB_BYPASS_INSTALLED )           ||
                      NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE )        ||
                      NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )     ||
                      NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )      ||
                      NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY ) ||
                      NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )       ||
                      NB_GetNodebit( UPM_NB_CHECK_PULL_CHAIN )            ||  // Add Pull chain failure protection
                      NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD )             ||
                      NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )     ||
                      NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )      ||
                      NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )  ||
                      NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )   ||
                      NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )		  ||
                      NB_GetNodebit( UPM_NB_FAN_FAILURE ) 			      ||
                      NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR ) )       * 0x0040 +
                      !( MCUStateTimer.TimerValue() > TWENTY_SECONDS ) * 0x0080 +
                      ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE ) * 0x0100;

    if ( !MCUStatus.bit.BypassXferInProgress                &&
         MCUStatus.bit.ESSCmd                               &&
         ( ESSFaultCondition == 0 )                         &&
         ParallelCan.ParGlobalAndData.McuStatus.bit.BypassAvailableOnBypass  &&
         ParallelCan.ParGlobalAndData.McuStatus.bit.BypassAvailable ) // all external UPSs are bypass available
         //Note: update the HE not available error code
         //if you add some ESS entry condition that there is not a nodebit indecated
    {
        if ( !MCUStatus.bit.ESSXferInProgress )
        {
            MCUStatus.bit.ESSXferInProgress = 1;
            ResetTransferState();
        }

        if( MCUTimer7.CheckTimeout( ONE_SECOND * 20 ) )
        {
	        Inverter.MatchBypassVoltage( true );

            //for jira wombat541, 250vac line to ess,match time up to 1min (ori 20s,will exhaust)
            if ( MCUTimer4.CheckTimeout( ONE_MINUTE ) ||		
	            Inverter.IsMatchedBypass() )
            {
	            if ( PrepareForBypassTransfer() || UPMCalEnabled )
	            {
                    TransferState( ESS_MODE_STATE );
	            }
            }
        }
    }
    else
    {
        MCUStatus.bit.ESSXferInProgress = 0;

        MCUTimer4.ClearTimer();
        MCUTimer7.ClearTimer();

        if ( !MCUStatus.bit.BypassXferInProgress &&
             BYPASS_FIRE_STATE != BypassState().GetMonitoredBypassState() )
        {
            // Stop matching bypass once PIC is OFF fire state.
            Inverter.MatchBypassVoltage( false );
        }
    }

    if ( !MCUStatus.bit.BypassXferInProgress &&
         !MCUStatus.bit.ShutdownXferInProgress &&
         !MCUStatus.bit.ESSXferInProgress )
    {
        ResetTransferState();
    }

    //if forward transfer, after 1 or 2 cycle, transfer to close loop
    if(ForwardTransferTransient)
    {
        ForwardTransferCount++;
        if(ForwardTransferCount >= WAIT_20_MS)
        {
            ForwardTransferTransient = false;
            //follow 9p not need, standard suggest delete
            //Inverter.OnWithIntialize();  //Need on():9E,Ess openloop-> close
            ForwardTransferCount = 0;
        }
    }
}


// ********************************************************************************************************
// *
// * Function: ESSModeState(void);
// *
// * Purpose:  In Energy Saver System or ECO mode.  Bypass is supporting the load.  Inverter is suspended.
// *
// * Description: 
// *
// ********************************************************************************************************
void MCUStateControl::ESSModeState(void)
{
    uint16_t bypassNotAvailableDelayed = false;
    static uint16_t OutputSdLowCnt = 0;

    // Wait 3ms for the STSW to turn on, then inverter off
    if ( MCUTimer1.CheckTimeout( WAIT_3_MS ) )
    {
        ShutdownInverter();
        if (Rectifier.GetState() != RECTIFIER_SUSPEND_STATE)
        {
        	Rectifier.RectifierSuspend();
        }
        CheckForFailedRelayFast();
        // For the parallel system, bypass is available once the bypass has
        // the load, and we are not actively transferring forward.
        MCUStatus.bit.BypassAvailable = !MCUStatus.bit.ForwardTransfer;
    }
    else
	{
		MCUStatus.bit.BypassAvailable = 0;
	}

    // delayed "bypass available on bypass"
    if ( !MCUStatus.bit.BypassAvailableOnBypass )
    {
        bypassNotAvailableDelayed = MCUTimer2.CheckTimeout( WAIT_500_MS );
    }
    else
    {
        MCUTimer2.ClearTimer();
    }	

    //
    // -->SHUTDOWN conditions:
    // Identical to the bypass standby state
    //
    if ( (!EEP_EPOTransferToBypass() &&
          NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )) )
    {
        TransferState( SHUTDOWN_STATE );
        return;
    }
    else if ( MCUStatus.bit.ShutdownCmd )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    		ResetTransferState();
    	}
        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( SHUTDOWN_STATE );
            return;
        }
    }
    else if ( MCUStatus.bit.StandbyCmd )
    {
    	if ( !MCUStatus.bit.ShutdownXferInProgress )
    	{
    		MCUStatus.bit.ShutdownXferInProgress = 1;
    		ResetTransferState();
    	}
        if ( PrepareForBypassTransfer() || UPMCalEnabled )
        {
            TransferState( STANDBY_STATE );
            return;
        }
    }
    else if( ParallelCan.ParallelStatus.bit.UpsOffCmd )
    {
        MCUStatus.bit.NormalCmd = 0;
        MCUStatus.bit.StandbyCmd = 1;//if the remote UPS on is active, load off the UPS would be fail. Thus add this
        TransferState( STANDBY_STATE );
    }
    else
    {
        MCUStatus.bit.ShutdownXferInProgress = 0;
    }

    // 
    // -->BYPASS transfer conditions:
    //
    //except conditions with nodebit already
    ESSAbandonReason = ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass * 0x0800 +
                       MCUStatus.bit.ToBypassCmd                 * 0x1000 +
                       BatteryConverter.CheckBatteryUVShutDown() * 0x2000 +
                       Abm().GetStatus().bit.BatteryOV * 0x4000;

    if ( ESSAbandonReason                                      ||
         NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )           ||
         ( NB_GetNodebit( UPM_NB_MBS_CLOSED ) &&
           !NB_GetNodebit( UPM_NB_MOB_OPEN ) )                 ||
         ( NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) && EEP_EPOTransferToBypass() ) ||
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )     ||
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )    ||
         NB_GetNodebit( UPM_NB_DRIVER_FAULT )                  ||
         NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )           ||
         NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )            ||
         NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )          ||
         ( Check_L3L4_OverLoad() && 
           !EnoughActiveUPM.GetState() )                       ||
         NB_GetNodebit(UPM_NB_BATTERIES_DISCONNECTED)          ||
         !NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )            ||
         NB_GetNodebit( UPM_NB_BATTERY_NEEDS_SERVICE )         ||
         NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE )    ||
         NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE )            ||
         NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )       ||
         NB_GetNodebit(UPM_NB_DC_LINK_UNDER_VOLTAGE )          ||
         NB_GetNodebit(UPM_NB_DC_LINK_OVER_VOLTAGE )
       )
    {
        MCUStatus.bit.EmergencyTransferToBypass = 1;
        if( MCUTimer4.CheckTimeout(WAIT_2_MS)  )
        {
            TransferState( BYPASS_STANDBY_STATE );
        }    
        return;
    }

    float OutputVoltageLowLimit  = BypassInterface::GetBypassDQLowLimit_ESS();

    if( OutputPLL.SourceNominalDQO.Sd < OutputVoltageLowLimit )
	{
		if( ++OutputSdLowCnt > ESSOutputSdLowTime )
		{
			ESSOutputSdLow = true;
			OutputSdLowCnt = 0;
		}
	}
	else
	{
		OutputSdLowCnt = 0;
	}

    // 
    // -->ONLINE transfer conditions:
    // 
    // except conditions with nodebit already
    ESSAbandonReason = !MCUStatus.bit.ESSCmd                    * 0x0001 +
				  	   !InverterSyncedToBypass()                * 0x0002 +
					   bypassNotAvailableDelayed                * 0x0004 +
					   ESSOutputSdLow                           * 0x0008 +
                       ( Abm().GetState() == ABM_RESET && !ABMDisabled )             * 0x0010 +
                       ( !Abm().GetStatus().bit.BelowHEChargeLimit && !Abm().GetStatus().bit.BatteryOV ) * 0x0020 +
                       ( Abm().GetState() == ABM_CHARGE )                            * 0x0040 +
					   ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE ) * 0x0080 +
					   ( BTR.BTRStatus.bit.BatteryTest                      ||
					     BTR.BTRStatus.bit.BatteryQuickTest                 ||
					     MCUStatus.bit.EcoBatteryTest )                           * 0x0100 +
					   ParallelCan.ParGlobalOrData.McuStatus.bit.ForwardTransfer  * 0x0200 + // Some other node is going online
					   // Some other node failed to go to ESS.  30ms is less than the 40ms L4 overload time
					   ( MCUStateTimer.TimerValue() > WAIT_30_MS && !ParallelCan.ParGlobalAndData.McuStatus.bit.InverterSuspend ) * 0x0400;

    if ( !MCUStatus.bit.ForwardTransfer                        && // already transferring
         ( ESSAbandonReason                                    ||
           NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )     ||
           NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )      ||
		   NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE ) 	   ||
		   NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )	   ||
           NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY ) ||
           NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR )            ||
           ( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD ) && !Check_L3L4_OverLoad() ) ||
           ( Check_L3L4_OverLoad() && EnoughActiveUPM.GetState() ) ||
           ParallelForwardTransfer )
       )
    {
        // turn off bypass scrs and let the inverter turn on before going to normal
        MCUStatus.bit.ForwardTransfer = 1;
        MCUStatus.bit.InverterSuspend = 0;

        // let bypass state machine know that scrs have been turned off.
        // Turn off command occurs in no more than 400 us, followed by up to 210us
        // before the bypass PIC gets the command, followed by up to one half-cycle
        // to shut down the diodes.
        BypassState().RequestBypassState( BYPASS_PRIMED_STATE );

		if (!ParallelForwardTransfer && !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ))
		{
            // tell external parallel UPSs to go online as well
            ParallelCan.TransmitCommandPacket( pcan::sys_ups_forward_transfer_command );
		}

        MCUTimer6.ClearTimer();//3c3 jira 203 fix ess to battery interrupt time 8ms bug       
        // What all reasons should force staying OnLine (no ETB) for a while ?
        // After the 9395 ESS K3 debacle: Everything.
        ForceStayOnline = WAIT_150_MS;

        if ( MCUStatus.bit.ESSCmd ) // not leaving ESS due to command, but other reason
        {
            ESSNumOutages++;
        }
        if ( ESSNumOutages >= ESSMaxNumOutages )
        {
            MCUStatus.bit.ESSLockout = 1;
        }
    }

    if( ( MCUStatus.bit.ForwardTransfer ) &&
        MCUTimer6.CheckTimeout(WAIT_1_MS) )//3c3 jira 203 fix ess to battery interrupt time 8ms bug
    {
        Inverter.OnFromESS();
        MCUStatus.bit.PostESSMode = true;
        PostESSModeTimer.ClearTimer();
//	        Inverter.OnOpenLoop();
        ForwardTransferTransient = true;  //after 1 cycle, transfer to close loop control: InverterStatus.bit.InverterOpenLoop = 0;
        MCUStatus.bit.ForwardTransfer = 0;
        ParallelForwardTransfer = false;
//			InitDropPower = float(OutputkVARating) * 34.0f;  //InitDropPower is the phase VA. 34~= 100/3, 5/8 full kVA ~=21.25		
		InitDropPower = ParallelCan.SystemLoad * OutputkVARating / 3.0f;	
        TransferState( ONLINE_STATE );
    }
}

// ********************************************************************************************************
// *
// * Function: EasyCapacityTestState(void);
// *
// * Purpose: In Easy Capacity Test.  Bypass is supporting the load.  Inverter is in current mode. 
// *
// * Description:
// *
// ********************************************************************************************************
void MCUStateControl::EasyCapacityTestState(void)
{
	uint16_t EnableRestartAbnormal;
	uint16_t DisableRestartAbnormal;

	if( AbnormalExitECT )
	{
		ECTRestartNumber--;
		AbnormalExitECT = 0;
	}

	//abnormal exit ECT canbe restart when clear
	EnableRestartAbnormal = ECTunbalancePower                                      ||
	                        ( ( !NB_GetNodebit( UPM_NB_RECTIFIER_ON )              ||
                                NB_GetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN  ) )  &&
                              ( Rectifier.GetState() == RECTIFIER_NORMAL_STATE) )  ||
							NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )         ||
							NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )        ||
							NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )    ||
							NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE )           ||
							NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )        ||
							NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )         ||
							NB_GetNodebit( UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY )    ||
							NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )           ||
							NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )           ||
							NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )   ||
							NB_GetNodebit( UPM_NB_INVERTER_AC_UNDER_VOLTAGE )      ||
							NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT )            ||
							NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE )          ||
							NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE)          ||
							NB_GetNodebit( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT )   ||
							NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY )     ||
							NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE );

    DisableRestartAbnormal = ( NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )     ||
							   NB_GetNodebit( UPM_NB_PULL_CHAIN )                     ||
							   NB_GetNodebit( UPM_NB_CONFIGURATION_ERROR )            ||
							  ( NB_GetNodebit( UPM_NB_MBS_CLOSED )                    &&
							    !NB_GetNodebit( UPM_NB_MOB_OPEN ) )                   ||
							   NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )            ||
							   NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )             ||
							   NB_GetNodebit( UPM_NB_FAN_FAILURE )                    ||
							   NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION )          ||
							   NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )          ||
							   NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )            ||
							   NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )          ||
							   NB_GetNodebit( UPM_NB_POWER_SUPPLY_FAILURE )           ||
							   NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE )     ||
							   NB_GetNodebit( UPM_NB_FUSE_FAILURE )                   ||
							   NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )  ||
							   NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP ) ||
							   NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )        ||
							   NB_GetNodebit( UPM_NB_BATTERY_TEST_IN_PROGRESS )       ||
							   NB_GetNodebit( UPM_NB_BATTERY_CURRENT_LIMIT )          ||
							   NB_GetNodebit( UPM_NB_LOW_BATTERY_SHUTDOWN )           ||
							   NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR )               ||
							   NB_GetNodebit( UPM_NB_RECTIFIER_PHASE_ROTATION ) );

	//except conditions with nodebit already
	ECTAbandonReason = ECTunbalancePower                   * 0x0002 +
	                   DisableRestartAbnormal              * 0x0004 +
					   EnableRestartAbnormal               * 0x0008 +
	                   !MCUStatus.bit.ToECTCmd             * 0x0010 +
			           MCUStatus.bit.NormalCmd             * 0x0020 +
			           MCUStatus.bit.ToBypassCmd           * 0x0040 +
			           MCUStatus.bit.ShutdownCmd           * 0x0080 +
			           MCUStatus.bit.StandbyCmd            * 0x0100 +
					   MCUStatus.bit.BldInpRemoteGotoBypass     * 0x0200 +
			           ( ( NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN )  ||
			               NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_FAIL )  ||
			               !BatteryConverter.GetStatus().bit.BoostOn  )    &&
                         Rectifier.GetStatus().bit.BatteryECT ) * 0x0400 +
                       NB_GetNodebit( UPM_NB_LOSS_OF_PWM_SYNC ) * 0x0800;

	if ( ECTAbandonReason )
    {
		if( !EnableRestartAbnormal || ( ECTRestartNumber <= 0 ) ||
			DisableRestartAbnormal )
		{
		    ParallelCan.TransmitCommandPacket( pcan::sys_ups_shut_down_command);
			MCUStatus.bit.InhibitToECT = 1;
            ECTRestartNumber = 0;
			TransferState( SHUTDOWN_STATE );
			Rectifier.ClearBatteryECT();
			MCUTimerBatECT.ClearTimer();
			MCUTimerLineECT.ClearTimer();
			ECTTimeOver = false;
			ECTBatTimerOut = 600;
			ECTLineTimerOut = 86400;
			ECTPowerSet = 0.4;
			NB_SetNodebit( UPM_NB_ABNORMAL_EXIT_ECT_MODE, true );
			return;
		}
		else
		{
			 TransferState( SHUTDOWN_STATE );
			 MCUStatus.bit.ToECTCmd = 0;
			 AbnormalExitECT = 1;
			 return;
		}
    }

    if( ECTTimeOver )
    {
        Inverter.Off();
        if( MCUTimer1.CheckTimeout(ONE_SECOND) )
        {
            ShutdownInverter();
            if( MCUTimer2.CheckTimeout(ONE_SECOND) )
            {
                //ECTExitCommand();
                ECTClearCommand();
                ECTRestartNumber = 0;
				ECTAbandonReason |= 0x0001;
                TransferState( BYPASS_STANDBY_STATE );
                MCUTimerBatECT.ClearTimer();
                MCUTimerLineECT.ClearTimer();
                ECTTimeOver = false;
            }
        }
    }
    else
    {
        MCUTimer1.ClearTimer();
        MCUTimer2.ClearTimer();
    }
}


