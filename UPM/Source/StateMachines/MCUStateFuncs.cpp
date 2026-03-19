// ******************************************************************************************************
// *                 MCUStateFuncs.c
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
// *    FILE NAME:   MCUStateFuncs.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR:  Tuomo Kaikkonen
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
#include <cmath>
#include "Constants.h"
#include "DebuggerBlocks.h"
#include "Abm.h"
#include "BypassInterface.h"
#include "Eeprom_Map.h"
#include "F28335Port.h"
#include "InverterControl.h"
#include "RectifierControl.h"
#include "IOexpansion.h"
#include "Meters.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "MCUState.h"
#include "RectifierStateControl.h"
#include "Spi_Task.h"
#include "Spi_Driver.h"
#include "StateTimer.h"
#include "FilteredBit.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
#include "ACMeter.h"
#include "Alarms_AC.h"
#include "InternalCan.h"
#include "ACPowerMeter.h"

using namespace std;

// ********************************************************************************************************
// * EXTERNAL DECLARATIONS
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************


// ********************************************************************************************************
// *
// * Function:    void ClearAllCommands( void )
// *
// * Purpose:     Resets all pending normal/bypass commands.
// *
// ********************************************************************************************************
void MCUStateControl::ClearAllCommands( void )
{
    CriticalSection enter;
    // clear all, excluding shutdown
    MCUStatus.bit.NormalCmd   = 0;
    MCUStatus.bit.ToBypassCmd = 0;
    MCUStatus.bit.AutoStandbyCmd = 0;
    MCUStatus.bit.ToECTCmd = 0;
    ParallelForwardTransfer = false;
}


// ********************************************************************************************************
// *
// * Function:    void Turn_UPS_Off( void )
// *
// * Purpose:     
// *
// ********************************************************************************************************
void MCUStateControl::Turn_UPS_Off( void )
{   
    {
        CriticalSection enter;
        MCUStatus.bit.ShutdownCmd = 1;
        MCUStatus.bit.NormalCmd   = 0;
        MCUStatus.bit.StandbyCmd  = 0;
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.AutoStandbyCmd = 0;
        MCUStatus.bit.ToECTCmd = 0;
        MCUStateMachine.AbnormalExitECT = 0;
        MCUStateMachine.ECTRestartNumber = 0;
    }
    ResetSticky();
}


// ********************************************************************************************************
// *
// * Function:    void NormalCommand( void )
// *
// * Purpose:     Commands the UPS to normal operation mode. The unit will advance to OnLine state if possible.
// *
// ********************************************************************************************************
void MCUStateControl::NormalCommand( void )
{
    RectifierStartupAttempts = 0;
    NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, false );
    {
        CriticalSection enter;
        MCUStatus.bit.NormalCmd   = 1;
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.StandbyCmd  = 0;
        MCUStatus.bit.ToECTCmd = 0;
        MCUStateMachine.AbnormalExitECT = 0;
        MCUStateMachine.ECTRestartNumber = 0;
    }
    ResetSticky();
    IsRectifierStart = false;
    IsBatteryStart = false;
}

// ********************************************************************************************************
// *
// * Function:    void StandbyCommand( void )
// *
// * Purpose:     Command UPS to Standby (or Shutdown if rectifier is not on).
// *
// ********************************************************************************************************
void MCUStateControl::StandbyCommand( void )
{   
    {
        CriticalSection enter;
        MCUStatus.bit.StandbyCmd  = 1;
        MCUStatus.bit.NormalCmd   = 0;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.ToECTCmd = 0;
        MCUStateMachine.AbnormalExitECT = 0;
        MCUStateMachine.ECTRestartNumber = 0;
    }
    ResetSticky();
    IsRectifierStart = false;
    IsBatteryStart = false;
}


// ********************************************************************************************************
// *
// * Function:    void AutoStandbyCommand( void )
// *
// * Purpose:     From shutdown to standby. Turns on rectifier.
// *
// ********************************************************************************************************
void MCUStateControl::AutoStandbyCommand( void )
{   
    RectifierStartupAttempts = 0;
    NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, false );
    CriticalSection enter;
    MCUStatus.bit.AutoStandbyCmd = 1;
    MCUStatus.bit.ShutdownCmd = 0;
    MCUStatus.bit.StandbyCmd = 0;
    IsRectifierStart = false;
    IsBatteryStart = false;
}


// ********************************************************************************************************
// *
// * Function:    void BypassCommand( void )
// *
// * Purpose:     Comamnd the UPS on bypass. It will not resume OnLine operation by itself.
// *
// ********************************************************************************************************
void MCUStateControl::BypassCommand( void )
{   
    {
        CriticalSection enter;
        MCUStatus.bit.ToBypassCmd = 1;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.StandbyCmd  = 0;
        MCUStatus.bit.ToECTCmd = 0;
        if ( ( McuMachineState == SHUTDOWN_STATE || 
               McuMachineState == BYPASS_STATE ) && 
             MCUStatus.bit.NormalCmd
            )
        {
                // Leave AutoStandbyCmd on (if it was), and activate it if NormalCmd was active.
                // This way rectifier startup is not cancelled if it was ramping.
            MCUStatus.bit.AutoStandbyCmd = 1;
        }
        if ( McuMachineState != ONLINE_STATE   &&
             McuMachineState != ESS_MODE_STATE )
        {
            MCUStatus.bit.NormalCmd   = 0; 
                // Don't automatically clear normal command in online state. 
                // If bypass transfer does not happen, leaves NormalCmd active. 
                // TransferState to bypass clears NormalCmd if transfer is possible.
        }
    }
	MCUStateMachine.AbnormalExitECT = 0;
    MCUStateMachine.ECTRestartNumber = 0;
    ResetSticky();
}



// ********************************************************************************************************
// *
// * Function:    void EcoOnCommand( void )
// *
// * Purpose:     ECO mode on
// *
// ********************************************************************************************************
void MCUStateControl::EcoOnCommand( void )
{   
    if ( NB_GetNodebit( UPM_NB_ECO_INSTALLED ) &&
         !NB_GetNodebit( UPM_NB_IN_EASY_CAPACITY_TEST_MODE )
       )
    {
        MCUStatus.bit.ESSCmd = 1;
    }
    //ResetESSLockout();
}


// ********************************************************************************************************
// *
// * Function:    void EcoOffCommand( void )
// *
// * Purpose:     ECO mode off
// *
// ********************************************************************************************************
void MCUStateControl::EcoOffCommand( void )
{   
    MCUStatus.bit.ESSCmd = 0;
}


// ********************************************************************************************************
// *
// * Function:    void EcoOffCommand( void )
// *
// * Purpose:     ECO mode off
// *
// ********************************************************************************************************
void MCUStateControl::StandbyChargerOffCommand( void )
{
    MCUStatus.bit.StandbyChargerOffCmd = 1;
}

// ********************************************************************************************************
// *
// * Function:    bool EnableTestMode( void )
// *
// * Purpose:     skips state machine. testing only. Always give shutdown command before enabling test mode.
// *
// ********************************************************************************************************
bool MCUStateControl::EnableTestMode( void )
{
    if ( (INITIALIZATION_STATE == McuMachineState   ||
          SHUTDOWN_STATE       == McuMachineState ) &&
         MCUStatus.bit.ShutdownCmd )
    {
        UPMTestMode = true;
    }
    return UPMTestMode;
}


// ********************************************************************************************************
// *
// * Function:    void DisableTestMode( void )
// *
// * Purpose:     
// *
// ********************************************************************************************************
void MCUStateControl::DisableTestMode( void )
{
    UPMTestMode = false;
}


// ********************************************************************************************************
// *
// * Function:    bool AnyRectifierOnCommand( void )
// *
// * Purpose:     Returns if any command is active that should cause turning on rectifier.
// *
// ********************************************************************************************************
bool MCUStateControl::AnyRectifierOnCommand( void )
{
    return ( MCUStatus.bit.AutoStandbyCmd    ||
             MCUStatus.bit.NormalCmd         ||
             ( AutoRestart                &&
               !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) )
        );
}


// ********************************************************************************************************
// *
// * Function:    bool AnyBatteryConverterOnCommand( void )
// *
// * Purpose:     Returns if any command is active that should cause turning on Battery Converter.
// *
// ********************************************************************************************************
bool MCUStateControl::AnyBatteryConverterOnCommand( void )
{
    return ( MCUStatus.bit.NormalCmd );
}

// ********************************************************************************************************
// *
// * Function:    void StoreAutoRestartEEP( void )
// *
// * Purpose:     
// *
// ********************************************************************************************************
void MCUStateControl::StoreAutoRestartEEP( void )
{
    if ( EnableAutoRestart )
    {
        if ( !AutoRestart )
        {
            AutoRestart = 1;
            PutEepData( Find_Eep_Num(&AutoRestart), 1, &AutoRestart, 0 );
        }
    }
}


// ********************************************************************************************************
// *
// * Function:    void ResetAutoRestart( void )
// *
// * Purpose:     
// *
// ********************************************************************************************************
void MCUStateControl::ResetAutoRestart( void )
{
    if ( AutoRestart )
    {
        AutoRestart = 0;
        PutEepData( Find_Eep_Num(&AutoRestart), 1, &AutoRestart, 0 );
    }
    MCUStatus.bit.AutoRestartReady = 0;
}

// ********************************************************************************************************
// *
// * Function:    void CheckPullChain( void )
// *
// * Purpose:     Debounce pull_chain and check_pull_chain nodebits
// *
// ********************************************************************************************************
void MCUStateControl::CheckPullChain(void)
{
    bool pcanFailed = NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR );
    bool checkPullChain;
    bool pullChainActive = ( DSPInRegister.GpiB.bit.PullChain == 0 ) && ( !PullChainSyncStart );

    if ( pcanFailed )
    {
    	//only annunciate check pullchain if we cannot pull the signal down.
    	//It's okay if other UPM's are pulling the signal down when this UPM
    	//is not on bypass
    	checkPullChain = BypassState().GetMonitoredBypassState() == BYPASS_FIRE_STATE  &&
    	                 !pullChainActive;
    }
    else
    {
    	//annunciate check pullchain if it's not as expected
    	checkPullChain = ( (bool) ( ParallelCan.ParGlobalOrData.BypassStatus.bit.MonitoredBypassState & BYPASS_FIRE_STATE ) ) !=
    	                 pullChainActive;
//	        checkPullChain &= !NB_GetNodebit(UPM_NB_MOB_OPEN);    //GOLDILOCKS-127
		if(NumOfUPMs == 1)	//only 20/40k, 80k still show check pullchain
		{
			checkPullChain &= !NB_GetNodebit(UPM_NB_MOB_OPEN);
		}
    }
    
    if( ( ParallelCan.ParallelStatus.bit.AutoID || DisableAutoID ) && 
        ( MCUStateMachine.GetState() > INITIALIZATION_STATE ) 	   &&
        EnablePullChainChk)
    {    
        // Zhangjun : UPM_NB_PULL_CHAIN - means the PullChain is active
        //            8 active and 4 inactive @ 5 kHz
        NB_DebounceAndQue(UPM_NB_PULL_CHAIN, pcanFailed && pullChainActive);
        
        // UPM_NB_CHECK_PULL_CHAIN - means the PullChain is not as expected
        //            2551 active and 1 inactive @ 5 kHz
        NB_DebounceAndQue(UPM_NB_CHECK_PULL_CHAIN, checkPullChain );
    }
    else
    {
    	NB_DebounceAndQue(UPM_NB_PULL_CHAIN, false);
    	NB_DebounceAndQue(UPM_NB_CHECK_PULL_CHAIN, false );
    }
}

// ********************************************************************************************************
// *
// * Function:    void LogCommands( void )
// *
// * Purpose:     Log commands to history
// *
// ********************************************************************************************************
#pragma CODE_SECTION("ramfuncs");
void MCUStateControl::LogCommands( void )
{
    if ( MCUStatus.bit.NormalCmdHistory != MCUStatus.bit.NormalCmd )
    {
        MCUStatus.bit.NormalCmdHistory = MCUStatus.bit.NormalCmd;
        NB_SetNodebit( UPM_NB_NORMAL_MODE_COMMAND, MCUStatus.bit.NormalCmd );
    }

    if ( MCUStatus.bit.ToBypassCmdHistory != MCUStatus.bit.ToBypassCmd )
    {
        MCUStatus.bit.ToBypassCmdHistory = MCUStatus.bit.ToBypassCmd;
        NB_SetNodebit( UPM_NB_TO_BYPASS_COMMAND, MCUStatus.bit.ToBypassCmd );
    }

    if ( MCUStatus.bit.ShutdownCmdHistory != MCUStatus.bit.ShutdownCmd ||
         MCUStatus.bit.StandbyCmdHistory  != MCUStatus.bit.StandbyCmd )
    {
        NB_SetNodebit( UPM_NB_UPS_OFF_COMMAND, 
                       ( MCUStatus.bit.ShutdownCmd || 
                         MCUStatus.bit.StandbyCmd ),
                       MCUStatus.bit.ShutdownCmd * 0x01 );

        MCUStatus.bit.StandbyCmdHistory  = MCUStatus.bit.StandbyCmd;
        MCUStatus.bit.ShutdownCmdHistory = MCUStatus.bit.ShutdownCmd;
    }
    
    // Internal MBS Active
    if ( NB_GetNodebit( UPM_NB_INTERNAL_MBS_INSTALLED ) )
    {
        if ( ExpansionInputReg.Ready )
        {
            // Current HW no InternalMaintenanceBypass feedback signal
            MCUStatus.bit.InternalMaintenanceBypass = 0;// !ExpansionInputReg.bit.MaintBypass;
            
            if( NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE ) )
            {
                if( !MCUStatus.bit.InternalMaintenanceBypass )
                {
                    NB_SetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE, false );
                }
            }
            else
            {
                if( MCUStatus.bit.InternalMaintenanceBypass )
                {
                    NB_SetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE, true );
                }
            }        
        }
    }
    else
    {
        NB_SetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE, false );
    }
    
    // Build_input MBS Closed/Open
    if( NB_GetNodebit( UPM_NB_MBS_CLOSED ) )
    {
        if ( !ParallelCan.PCan_CheckMBSClosed() )
        {
            NB_SetNodebit( UPM_NB_MBS_CLOSED, false ); 
        }    
    }
    else
    {
        if( ParallelCan.PCan_CheckMBSClosed() )
        {
            NB_SetNodebit( UPM_NB_MBS_CLOSED, true );
        }    
    }
    
    // MBS history        
    if ( MCUStatus.bit.MaintenanceBypassHistory != ( MCUStatus.bit.BldInpMaintenanceBypass || MCUStatus.bit.InternalMaintenanceBypass ) )
    {
        MCUStatus.bit.MaintenanceBypassHistory = MCUStatus.bit.BldInpMaintenanceBypass || MCUStatus.bit.InternalMaintenanceBypass;
    }
    
    if ( !NB_GetNodebit( UPM_NB_ECO_INSTALLED ) )
    {
            // ESSCmd = always zero if not installed.
        MCUStatus.bit.ESSCmd = 0;
    }
    else
    {
        if( ESSEnabled )
        {
            // always go to eco mode if it is installed.
            EcoOnCommand();
        }
        else
        {
            EcoOffCommand();
        }
    }
    
    if ( MCUStatus.bit.ESSCmdHistory != MCUStatus.bit.ESSCmd )
    {
        MCUStatus.bit.ESSCmdHistory = MCUStatus.bit.ESSCmd;
        NB_SetNodebit( UPM_NB_ECO_ENABLE, MCUStatus.bit.ESSCmd );
    }
    
    if ( MCUStatus.bit.ToECTCmdHistory != MCUStatus.bit.ToECTCmd )
    {
        MCUStatus.bit.ToECTCmdHistory = MCUStatus.bit.ToECTCmd;
        NB_SetNodebit( UPM_NB_ECT_MODE_COMMAND, MCUStatus.bit.ToECTCmd );
    }
}

// ****************************************************************************
// *
// * Function:     void BldInpMOBOpen( uint16_t active )
// *
// * Purpose:      Report the building alarm status of the MOB
// *
// * Parms Passed: State    true == MOB Open, false == MOB Close
// * Returns:      Nothing
// *
// *
// ****************************************************************************
void MCUStateControl::BldInpMOBOpen( bool active )
{   
    if (!MOBEnabled)
    {
        // If the UPS does not have an MOB, behave identically to one that does when
        // its MOB is closed.
        MCUStatus.bit.BldInpMOBClosed = 1;
    }
    else
    {
        MCUStatus.bit.BldInpMOBClosed = !active;
    }
        
    NB_SetNodebit( UPM_NB_MOB_OPEN, !MCUStatus.bit.BldInpMOBClosed );
}

// ****************************************************************************
// *
// * Function:     void SetInverterRelay( uint16_t state )
// *
// * Purpose:      Open/Close inverter output relay
// *
// * Parms Passed: State    RELAY_OPEN / RELAY_CLOSED
// * Returns:      Nothing
// *
// * Description:  Opens/Closes the relay and logs the event.
// *
// ****************************************************************************
void MCUStateControl::SetInverterRelay( uint16_t state )
{
	if ( !MCUStatus.bit.InverterContactorTest )
	{
	    if ( RELAY_CLOSED == state )
	    {
	        if ( DSPOutRegister.GpoB.bit.InverterRelay == 0 )
	        {
	            DSPOutRegister.GpoB.bit.InverterRelay = 1;
	            // relay takes ~10ms to close, don't set NB here
	            //NB_SetNodebit( UPM_NB_INVERTER_CONTACTOR_CLOSED, DSPOutRegister.GpoB.bit.InverterRelay );
	        }
	    }
	    else
	    {
	        if ( DSPOutRegister.GpoB.bit.InverterRelay == 1 )
	        {
	            DSPOutRegister.GpoB.bit.InverterRelay = 0;
	            // relay takes ~10ms to open, don't set NB here
	            //NB_SetNodebit( UPM_NB_INVERTER_CONTACTOR_CLOSED, DSPOutRegister.GpoB.bit.InverterRelay );
	        }        
	    }
	}
	else
	{
		// inverter contactor test; leave relay alone, but store its proper state
		StoredInverterRelayState = state;
	}
}


// ****************************************************************************
// *
// * Function:     void CheckOutACUVST( void )
// *
// * Purpose:      Filter Output ACUV status a few state machine cycles for startup logic.
// *
// ****************************************************************************
void MCUStateControl::CheckOutACUVST( void )
{
    static uint16_t OutACUVST_delay = 0;
    
    if ( !MCUStatus.bit.OutACUVST )
    {
        if( OutVolRMS_NotPresent.GetState() )
        {
            if ( OutACUVST_delay >= 2 )
            {
                MCUStatus.bit.OutACUVST = 1;
            }
            else
            {
                OutACUVST_delay++;
            }
        }
        else
        {
            OutACUVST_delay = 0;
        }
    }
    else
    {
        if( !OutVolRMS_NotPresent.GetState() )
        {
            if ( OutACUVST_delay >= 2 )
            {
                MCUStatus.bit.OutACUVST = 0;
            }
            else
            {
                OutACUVST_delay++;
            }
        }
        else
        {
            OutACUVST_delay = 0;
        }
    }
}

// ****************************************************************************
// *
// * Function:     uint16_t StartupLogic(void)
// *
// * Purpose:      Check startup conditions in shutdown/standby states
// *
// * Parms Passed: Nothing
// * Returns:      One of following:
// *                 STARTUP_ONLINE     = Conditionds are good for online state
// *                 STARTUP_BYPASS_ON  = Conditionds are good for bypass while other units are already ON bypass
// *                 STARTUP_BYPASS     = Conditionds are good for bypass (other units are also OFF)
// *                 STARTUP_OFF        = Startup not possible, normal cmd cleared under persistant conditions
// *                 STARTUP_WAIT       = Do nothing
// *
// * Description: Called in shutdown and syncing states in main state machine.
// *
// ****************************************************************************
uint16_t MCUStateControl::StartupLogic( void )
{
    uint16_t Result = 0;
    bool UnitsOnBypass = ParallelCan.ParGlobalOrData.BypassStatus.bit.BypassState & BYPASS_FIRE_STATE;
	MCUStatus.bit.UPSInverterAvailable = ParallelCan.PCan_CheckUPSInverterAvailable();

    CheckOutACUVST();
    
    if ( MCUStatus.bit.NormalCmd  &&
         !LoadOffToNormalPersistentOk() )
    {
    	// clear normal commands on persistant conditions only
    	ClearAllCommands();
    	// command other UPM's in my UPS to clear all commands
        ParallelCan.TransmitToUPMsInUPS( pcan::fw_clear_commands, 0, 0, 0, 0 );
    }

        // Reasons to shut down:
    if ( !Result && 
         (MCUStatus.bit.ShutdownCmd ||
          MCUStatus.bit.StandbyCmd ||
          NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) ||
          NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) )
        )
    {
        Result = STARTUP_OFF;
    }
    
    if ( StayLoadOff )
    {
    	// wait for statuses to update before attempting to transfer
    	StayLoadOff--;  
    	Result = STARTUP_WAIT;
    }
    
    #if 0
    // TODO: Disabled for now.
    // This check is intended to capture the case of a phase leg with one shorted
    // of inverter diodes on startup to avoid escalating the failure.
    // It creates false positives on Load Off that themselves trigger an ETB.
    
    // Check inverter voltage.  It should be < 25% of rated on all three phases
    float threshold = 0.75 * OutNomVolts * 0.1;
    
    if( ( InverterVoltageRMS.FilteredRMS.phA > threshold ||
    	  InverterVoltageRMS.FilteredRMS.phB > threshold ||
    	  InverterVoltageRMS.FilteredRMS.phC > threshold ) &&
    	!NB_GetNodebit(UPM_NB_INVERTER_CONTACTOR_CLOSED)   &&
    	!Inverter.GetStatus().bit.InverterOn) )
	{
		Result = STARTUP_OFF;
		AbnormalOutVoltage = ABNORMALOUTV_INV_RLYOPEN;
	}
	#endif

        // Check bypass command
    if ( !Result                                       &&
         ( MCUStatus.bit.ToBypassCmd                     ||
           ( MCUStatus.bit.NormalCmd                       &&
             EEP_BypassWithNormalCmd()                     &&
             !ParallelCan.ParGlobalOrData.McuStatus.bit.OnInverter &&  // don't use bypass with units already online
             !UPMCalEnabled ) )                        && // ignore BypassWithNormal when calibrating
         !FrequencyConverterMode                       &&
         NB_GetNodebit( UPM_NB_BYPASS_INSTALLED )      &&
         !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE ) &&
         !NB_GetNodebit( UPM_NB_PARALLEL_SETUP_FAIL ) )
    {
            // Check output voltage
        if ( MCUStatus.bit.OutACUVST               ||
             ( NB_GetNodebit( UPM_NB_MBS_CLOSED )    &&
               !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ||
             NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE ) )
        {
            // Bypass startup on dead bus
            Result = STARTUP_BYPASS;

            // Ignore bypass cmd and check normal command if none of the 
            // conditions above are true bypass not available or not enough 
            // bypass capacity
        }
        else
        {       // Output voltage detected
            if ( UnitsOnBypass )
            {
            	// Join units already on bypass
            	Result = STARTUP_BYPASS_ON;
            }
            else if ( ParallelCan.ParGlobalOrData.McuStatus.bit.OnInverter )
            {
            	// Synchronous transfer to bypass with units that are online
            	Result = STARTUP_BYPASS;
            }
            else
            {
	            AbnormalOutVoltage = ABNORMALOUTV_BYP_UOUTON_CANOFF;
	            if ( NB_GetNodebit( UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP ) )
	            {
	                Result = STARTUP_OFF;
	            }
	            else
	            {
	                Result = STARTUP_WAIT;
	            }
            }
        }
    }
    // End of check bypass command


        // Check normal command. Skip if EPO active
    if ( !Result                                                 &&
         !MCUStatus.bit.StandbyCmd                               &&
         MCUStatus.bit.NormalCmd                                 &&
         !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )         &&
         ParallelCan.PCan_CheckSystemInverterAvailable()         &&
         MCUStatus.bit.UPSInverterAvailable )
    {
        uint16_t TempResult = STARTUP_OFF;

            // Check output voltage
        if ( MCUStatus.bit.OutACUVST )
        {
            // Inverter assumes dead bus
            // Parallel system, system can't sync with bypass, startup though pull-chain
//	            if ( ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() ) && 
//	                 ( ParallelCan.ParGlobalAndData.SyncStatus.bit.SyncState != SYNC_STATE_BYPASS ||
//	                   !InverterSyncedToBypass() ||
//	                   ParallelCan.ParGlobalAndData.SyncStatus.bit.SyncState != SYNC_STATE_INPUT) )
			if ( ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() ) && 
				 ( (ParallelCan.ParGlobalAndData.SyncStatus.bit.SyncState != SYNC_STATE_BYPASS
				     || !InverterSyncedToBypass()) &&
				   (ParallelCan.ParGlobalAndData.SyncStatus.bit.SyncState != SYNC_STATE_INPUT)) )
            {
                PullChainSyncStart = true;
            }
            // Single UPS or system is sync with bypass, startup normally(do not go though pull-chain).
            else
            {
                PullChainSyncStart = false;
            }

	    	// External parallel systems cannot go online unless all are synced to bypass
	    	TempResult = STARTUP_ONLINE;
        }
        else
        {
            // Output is present, do not go though pull-chain startup
            PullChainSyncStart = false;
            // Normal command and voltage detected on output
            if ( ParallelCan.ParGlobalOrData.McuStatus.bit.OnInverter )
            {
            	// Join other units that are already online
            	TempResult = STARTUP_ONLINE;
            }
            else if ( UnitsOnBypass )
            {
            	// Other units on bypass
            	// Join them synchronously online
            	TempResult = STARTUP_ONLINE;
            }
            else
            {
	            AbnormalOutVoltage = ABNORMALOUTV_BYP_UOUTON_CANOFF;
	            if ( NB_GetNodebit( UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP ) )
	            {
	                TempResult = STARTUP_OFF;
	            }
	            else
	            {
	                TempResult = STARTUP_WAIT;
	            }
            }
        }

        Result = TempResult;
    }
    
    if ( !Result && MCUStatus.bit.AutoStandbyCmd )
    {
        Result = STARTUP_WAIT;
    }

    if ( !Result )
    {
        // EPO is on, RemotePowerModuleOff, there was no active normal or bypass
        // command or bypass startup was requested and it wasn't possible.
        Result = STARTUP_OFF;
    }

        // Clear pending abnormal output voltage if unit started
    if ( Result == STARTUP_ONLINE ||
         Result == STARTUP_BYPASS_ON ||
         Result == STARTUP_BYPASS )
    {
        AbnormalOutVoltage = 0;
    }
    
    if ( OldStartResult != Result )
    {
    	// Make sure the transfer phase is starting at zero
    	ResetTransferState();
    }

    OldStartResult = Result;

    return (Result);
}


// ********************************************************************************************************
// *
// * Function:    void Run1Sec( void )
// *
// * Purpose:     Update any timers needed by MCU state machine, that are not needed to run at ADC rate.
// *
// * Description:
// *              Call from 1s periodic
// ********************************************************************************************************
void MCUStateControl::Run1Sec( void )
{
    static StateTimer AutoRestartTimer;
    static bool ECTTimeOverTemp = 0;
    static uint16_t OnlineDelayCounter = 0;

    if ( AutoRestart                                     &&
         McuMachineState != SHUTDOWN_STATE               &&
         McuMachineState != INITIALIZATION_STATE         &&
         NB_GetNodebit( UPM_NB_RECTIFIER_ON )            &&
         !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
         !BatteryConverter.CheckBatteryUVShutDown()      &&
         ( !EEP_IsInternalParallel()                       &&
           !ParallelCan.UPSIsParallel()                    ||
           ParallelCan.ParallelStatus.bit.ParamsInit ) )
    {
        if ( AutoRestartTimer.CheckTimeout( 10 ) )    // some kind of "sleeve delay", just in case.
        {
            MCUStatus.bit.AutoRestartReady = 1;       // to parallel units (future)        
        }
    }
    else
    {
        AutoRestartTimer.ClearTimer();
        MCUStatus.bit.AutoRestartReady = 0;
    }

        // 
        // ESS lockout timers
        // 
    if ( ESSNumOutages && 
         MCUStatus.bit.ESSCmd )
    {
        if ( ESSOutageResetTimer.CheckTimeout( ESSOutageResetTime * 60 ) &&
             !MCUStatus.bit.ESSLockout )
        {
            ESSNumOutages = 0;
            ESSOutageResetTimer.ClearTimer();
        }
    }
    else
    {
        ESSNumOutages = 0;
        ESSOutageResetTimer.ClearTimer();
    }

    if ( MCUStatus.bit.ESSLockout &&
         MCUStatus.bit.ESSCmd )
    {
        if ( ESSLockoutTimer.CheckTimeout( ESSLockoutTime * 60 ) )
        {
            MCUStatus.bit.ESSLockout = 0;
            ESSNumOutages = 0;
            ESSOutageResetTimer.ClearTimer();
            ESSLockoutTimer.ClearTimer();
        }
    }
    else
    {
        MCUStatus.bit.ESSLockout = 0;
        ESSLockoutTimer.ClearTimer();
    }
    
    if(McuMachineState == EASY_CAPACITY_TEST_STATE)
    {
        switch( ECTPhase )
        {
            case 0:
                ECTTimeOverTemp = MCUTimerBatECT.CheckTimeout( ECTBatTimerOut );
                if( ECTTimeOverTemp )
                {                    
                    ECTTimeOverTemp = MCUTimerLineECT.CheckTimeout( ECTLineTimerOut );
                    if( !ECTTimeOverTemp )
                    {                        
                        ECTPhase++;
                        Inverter.Off();
                    }
                }
                break;
                
            case 1:
                Rectifier.ClearBatteryECT();
                ECTPhase++;
                break;
                
            case 2:                                
                ECTPhase++;
                break;
                
            case 3:
                Inverter.OnECTCurrentLoop();
                NB_SetNodebit( UPM_NB_UPS_ON_NORMAL, true );
                ECTPhase++;
                break;
                
            case 4:
                ECTTimeOverTemp = MCUTimerLineECT.CheckTimeout( ECTLineTimerOut );
                break;
                
            case 5:
                ECTTimeOverTemp = MCUTimerBatECT.CheckTimeout( ECTBatTimerOut );
                break;
                
            default:
                ECTTimeOverTemp = true;
                break;
        }                
        ECTTimeOver = ECTTimeOverTemp;            
    }
    
    
    if( MCUStatus.bit.EcoBatteryTest )
    {
        // dont' start battery test until unit is out of eco mode
        if ( NB_GetNodebit( UPM_NB_RECTIFIER_ON )  &&
        	 Rectifier.GetState() == RECTIFIER_NORMAL_STATE
            )
		{
		    MCUStatus.bit.EcoBatteryTest = 0; 
		    BTR.StartDualStageBatteryTest();
		}
		else if (Rectifier.GetState() == RECTIFIER_SHUTDOWN_STATE)
		{
			// Cancel the command if the rectifier shuts down prior to
			// meeting the prereq's to start the test.
			MCUStatus.bit.EcoBatteryTest = 0;
		}
    }

    //add to enlarge input range for HV system ECT.
    static bool ups_on_ect_mode = false;

//	    if ( McuMachineState == EASY_CAPACITY_TEST_STATE )
//	    {
//	        if ( ups_on_ect_mode == false )
//	        {
//	            ups_on_ect_mode = true;
//	            //move all SPI operation to SPI Tsk to avoid EEPROM R/W error
//	            BypVoltMaxLimit += 10;
//	            PutEepData(PARAM_BypACOVLevel, 1, &BypVoltMaxLimit, 0);
//	                
//	            BypVoltMinLimit += 10;
//	            PutEepData(PARAM_BypACUVLevel, 1, &BypVoltMinLimit, 0);
//	                
//	            BypVoltMaxLimitFast += 10;
//	            PutEepData(PARAM_BypACOVFastLevel, 1, &BypVoltMaxLimitFast, 0);
//	
//	            BypVoltMinLimitFast += 10;
//	            PutEepData(PARAM_BypACUVFastLevel, 1, &BypVoltMinLimitFast, 0);
//	                
//	            EE_PLLMaxPhaseError += 5;
//	            PutEepData(PARAM_BypassPLLMaxPhaseError, 1, &EE_PLLMaxPhaseError, 0);
//	            PutEepData(Param_OutputPLLMaxPhaseError, 1, &EE_PLLMaxPhaseError, 0);
//	        }
//	    }
//	    else
//	    {
//	        if ( ups_on_ect_mode == true )
//	        {
//	            ups_on_ect_mode = false;
//	            //move all SPI operation to SPI Tsk to avoid EEPROM R/W error
//	            BypVoltMaxLimit -= 10;
//	            PutEepData(PARAM_BypACOVLevel, 1, &BypVoltMaxLimit, 0);
//	                
//	            BypVoltMinLimit -= 10;
//	            PutEepData(PARAM_BypACUVLevel, 1, &BypVoltMinLimit, 0);
//	                
//	            BypVoltMaxLimitFast -= 10;
//	            PutEepData(PARAM_BypACOVFastLevel, 1, &BypVoltMaxLimitFast, 0);
//	
//	            BypVoltMinLimitFast -= 10;
//	            PutEepData(PARAM_BypACUVFastLevel, 1, &BypVoltMinLimitFast, 0);
//	                
//	            EE_PLLMaxPhaseError  -= 5;
//	            PutEepData(PARAM_BypassPLLMaxPhaseError, 1, &EE_PLLMaxPhaseError, 0);
//	            PutEepData(Param_OutputPLLMaxPhaseError, 1, &EE_PLLMaxPhaseError, 0);
//	        }
//	    }

    // stay in online mode for 10s, force sync to base
    if( !MCUStatus.bit.ESSCmd                  &&
        !MCUStatus.bit.ToECTCmd                &&
        !MCUStateMachine.ForceSyncBaseToByp    &&
        SyncConfig.bit.ForceSyncBaseInLineMode &&
        ( ONLINE_STATE == McuMachineState ) )
    {
        if( ++OnlineDelayCounter >= 10 )    // delay 10s
        {
            OnlineDelayCounter = 10;
            MCUStatus.bit.ForceSyncBase = 1;
        }
    }
    else
    {
        OnlineDelayCounter = 0;
        MCUStatus.bit.ForceSyncBase = 0;
    }
}


// ********************************************************************************************************
// *
// * Function:    void Run5Msec( void )
// *
// * Purpose:     Update any timers needed by MCU state machine, that are not needed to run at ADC rate.
// *
// * Description:
// *              Call from 5ms periodic
// ********************************************************************************************************
void MCUStateControl::Run5Msec( void )
{
	static float lastMatched = 0.0f;
    static uint16_t DebugNum = 0;
    static uint16_t SampleCount = 0;
	float thisMatched = 0.0f;

    if ( MCUStatus.bit.StoreAutoRestart )
    {
        MCUStatus.bit.StoreAutoRestart = 0;
        StoreAutoRestartEEP();

        if ( !MCUStatus.bit.NormalCmd )
        {
            NB_SetNodebit( UPM_NB_AUTOMATIC_STARTUP_PENDING, true );
        }
    }

    NB_DebounceAndQue( UPM_NB_AUTOMATIC_STARTUP_PENDING, AutoRestart && 
                                                         !MCUStatus.bit.NormalCmd ) ;

    if ( MCUStatus.bit.AutoRestartReady &&       // check also parallel (future)
         !MCUStatus.bit.NormalCmd       &&
         !ParallelCan.PCan_CheckSystemOnNormalFail() )
    {
        //NormalCommand();
        
        if( ParallelCan.ParallelStatus.bit.Master )
        {
            // if myself is master,the on normal command will be sent in parallel task
            ParallelCan.ParallelCommand.bit.para_on_normal_command = 1;
        }
        else
        {
            // if myself is slave,broadcast to master to further handle this command
            ParallelCan.TransmitCommandPacket( pcan::para_ups_normal_commmand);
        }    
    }

    MCUStatus.bit.CheckedOut = ( (McuMachineState == SHUTDOWN_STATE) ||
                                 (McuMachineState == INITIALIZATION_STATE) ||
                                 NB_GetNodebit( UPM_NB_MOB_OPEN ) ) &&
                                ParallelCan.UPSIsParallel();

    CheckEnoughActiveUPM();

    //Set HWCL before online
    static uint16_t temp_InvHWCurrentLimit = InvHWCurrentLimit;
    static uint16_t InvHWCLCount = 0;
    static bool HWCLSetNormReady = true;
    static bool RecCurrentLoopGainSetNormReady = true;
    static uint16_t temp_RectCurrentLoopGain = RectCurrentLoopGain;
    static uint16_t RectCurrentLoopGainCount = 0;



    //NTOPR-29/48 Decrease the inverter HWCL level in the first several cycles during battery/online startup.

    if ( SHUTDOWN_STATE        == McuMachineState ||
    	 STANDBY_STATE         == McuMachineState ||
         BYPASS_STATE          == McuMachineState ||
    	 BYPASS_STANDBY_STATE  == McuMachineState 	)
    {
        temp_InvHWCurrentLimit = InvHWCurrentLimit * 0.7f;
        Inverter.SetHWCurrentLimit( temp_InvHWCurrentLimit );
        InvHWCLCount = 0;

        if(RectCurrentLoopGain <= 100)
        {
            temp_RectCurrentLoopGain = RectCurrentLoopGain * 1.2f;
            Rectifier.SetRectCurrentloopGain( temp_RectCurrentLoopGain );
            RectCurrentLoopGainCount = 0;
        }
    }
    else
    {
         //Nothing need to do
    }

   if(( ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE ) || // ON_BATTERY nodebit is too slow, check state directly
	    ( Rectifier.GetState() == RECTIFIER_NORMAL_STATE ))               &&
		(Inverter.GetStatus().bit.InverterOn)                            &&
		(GetInverterRelayState() == RELAY_CLOSED)                           &&
       (!MCUStatus.bit.InverterSuspend))
    {
       InvHWCLCount++;
       if(RectCurrentLoopGain <= 100)
       {
           RectCurrentLoopGainCount++;

           if( RectCurrentLoopGainCount < 20 ) //100ms
    	   {
    	       Rectifier.SetRectCurrentloopGain( temp_RectCurrentLoopGain );
    	   }
    	   else if (20 == RectCurrentLoopGainCount)
    	   {
    	       Rectifier.SetRectCurrentloopGain( RectCurrentLoopGain );
    		   RectCurrentLoopGainCount = 99;
    	   }
    	   else
    	   {
    	       RectCurrentLoopGainCount = 99;
    	   }
       }

       if( InvHWCLCount < 20 ) //100ms
	   {
		   Inverter.SetHWCurrentLimit( temp_InvHWCurrentLimit );
	   }
	   else if (20 == InvHWCLCount)
	   {
		   Inverter.SetHWCurrentLimit( InvHWCurrentLimit );
		   InvHWCLCount = 99;
	   }
	   else
	   {
	       InvHWCLCount = 99; //To avoid InvHWCLCount overflow to zero.
	   }
    }
    else if(MCUStateMachine.GetState() == STANDBY_STATE )
    {
		   Inverter.SetHWCurrentLimit( temp_InvHWCurrentLimit );
	       if(RectCurrentLoopGain <= 100)
	       {
		       Rectifier.SetRectCurrentloopGain( temp_RectCurrentLoopGain );
	       }
    }
    else    //postEssMode
    {
        if(PostESSModeTimer.TimerValue() < WAIT_20_MS && MCUStatus.bit.PostESSMode )
        {
		   Inverter.SetHWCurrentLimit( temp_InvHWCurrentLimit );
           HWCLSetNormReady = false;
        }
    	else if( !HWCLSetNormReady )
    	{
    	    Inverter.SetHWCurrentLimit( InvHWCurrentLimit );
            HWCLSetNormReady = true;
    	}

        if(RectCurrentLoopGain <= 100)
        {
            if(PostESSModeTimer.TimerValue() < WAIT_20_MS && MCUStatus.bit.PostESSMode )
            {
     	       Rectifier.SetRectCurrentloopGain( temp_RectCurrentLoopGain );
    		   RecCurrentLoopGainSetNormReady = false;
            }
        	else if( !RecCurrentLoopGainSetNormReady )
        	{
     	       Rectifier.SetRectCurrentloopGain( RectCurrentLoopGain );
        	    RecCurrentLoopGainSetNormReady = true;
        	}

        }
    }



   thisMatched = ScreenMeters.PercentLoad.sum;
   if((thisMatched > 20 && lastMatched < 20) || (thisMatched > 20 && lastMatched > 20))
   {
	   if(DebugNum < 50)
	   {
		   if(SampleCount == 3)
		   {
			   DebugPara[DebugNum][0] = MCUStateMachine.GetInverterRelayState() 					   * 100 +
					   	   	   	   	   	//ParallelCan.ParGlobalOrData.InverterStatus.bit.MatchBypassRMS * 0x0010 +
					   	   	   	   	   	//Inverter.IsMatchedBypass() 								   * 0x0001 +
					   	   	   	   	   	DebugFloat[0];//DebugFloat[0] = vdroopA
			   DebugPara[DebugNum][1] = InverterVoltageRMS.RawRMS.phA;
			   DebugPara[DebugNum][2] = DebugFloat[1];//DebugFloat[1] = Ref*100 +temp;
			   DebugPara[DebugNum][3] = RawAdcData.st.RailVoltagePositive-RawAdcData.st.RailVoltageNegative;
			   DebugNum++;
			   SampleCount = 0;
		   }
		   else
		   {
			   SampleCount++;
		   }
	   }
   }
   else
   {
	   SampleCount = 0;
	   DebugNum = 0;
   }
   lastMatched = thisMatched;
}

// ********************************************************************************************************
// *
// * Function:    void Run100Msec( void )
// *
// * Purpose:     Update any timers needed by MCU state machine, that are not needed to run at ADC rate.
// *
// * Description:
// *              Call from 100ms periodic
// ********************************************************************************************************
void MCUStateControl::Run100Msec( void )
{
    static uint16_t outputOverloadETBTimer = 0;


    
    uRectifierStatus rectStatus = Rectifier.GetStatus();


    //update rec rms max input current, for different load percent in val
    Rectifier.SetMaxRMSInputCurrent( MaxRMSInputCurrentEEValue );

        //
        // Clear toBypassCmd in 3 seconds if there is no transfer in sight.
        //
    if ( MCUStatus.bit.ToBypassCmd              &&
         !NB_GetNodebit( UPM_NB_UPS_ON_BYPASS ) &&
         !MCUStatus.bit.BypassXferInProgress )
    {
            // Always clear bypass command after a timeout
        if ( BypassCommandTimer100ms.CheckTimeout ( BYPASS_CMD_TIMEOUT ) )
        {
            MCUStatus.bit.ToBypassCmd = 0;
            BypassCommandTimer100ms.ClearTimer();
        }
    }
    else
    {
        BypassCommandTimer100ms.ClearTimer();
    }


        //
        // Clear NormalCmd in 3 seconds if there is no transfer in sight.
        //
    if ( ((McuMachineState == SHUTDOWN_STATE || 
           McuMachineState == BYPASS_STATE)  && 
          AnyRectifierOnCommand()            && 
          NB_GetNodebit( UPM_NB_RECTIFIER_FAILED ) )
         ||
         ((McuMachineState == STANDBY_STATE  ||
           McuMachineState == BYPASS_STANDBY_STATE) &&
          ( NB_GetNodebit( UPM_NB_CHARGER_FAILURE ) ||
            ( ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) ) &&
              !EEP_BattNotRequiredToGoOnline() )) &&
            ( !UPMCalEnabled && !ParallelCan.ParGlobalAndData.McuStatus.bit.NormalCmd ) && 
          MCUStatus.bit.NormalCmd)
        )
    {
        if ( NormalCommandTimer100ms.CheckTimeout ( USER_CMD_TIMEOUT ) && !ParallelCan.PCan_CheckSystemInverterAvailable() )
        {
            MCUStatus.bit.NormalCmd = 0;
            NormalCommandTimer100ms.ClearTimer();
        }
    }
    else
    {
        NormalCommandTimer100ms.ClearTimer();
    }


    if ( ( OutputPLL.SourceNominalDQO.Sd < .75f || Output25RMSUV.GetState() )   &&  // -25% limits
         ( ( BypassPLL.SourceNominalDQO.Sd < .75f  || Bypass25RMSUV.GetState() ) || // -25% limits
           NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )   ||
           ( NB_GetNodebit( UPM_NB_UPS_ON_BYPASS )          &&
            BYPASS_FIRE_STATE != BypassState().GetMonitoredBypassState()) )
         // TODO:?? 3. Protected Bypass eep is enabled and bypass goes out of limits.
        )
    {
            // The output voltage is less than 25% of nominal for 2 seconds, and the bypass switchgear has failed
            // Bypass and output voltage go below 25% of nominal for 2 seconds
    }
    else
    {
        BypassNotAvailableOnBypassTimer.ClearTimer();
    }

    if ( BypassNotAvailableOnBypassTimer.CheckTimeout( BYPASS_LOAD_LOSS_ON_BYPASS_TIMEOUT ) ||
         NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) )
    {
        MCUStatus.bit.BypassAvailableOnBypass = 0;
    }
    else
    {
        MCUStatus.bit.BypassAvailableOnBypass = 1;
    }
    
        // 
        // If output overload is active for 2 seconds and there is no regula bypass transfer,
        // do emergency transfer to bypass
        // 
    if ( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP ) )
    {
        if ( outputOverloadETBTimer < OUTPUT_OVERLOAD_ETB_TIMEOUT )
        {
            outputOverloadETBTimer++;
        }
        else
        {
            MCUStatus.bit.OutputOverloadETB = 1;
        }
    }
    else
    {
        MCUStatus.bit.OutputOverloadETB = 0;
        outputOverloadETBTimer = 0;
    }


        //
        // Too many bypass transfers within 10 minutes lock the load on bypass for 60 minutes
        //
    if ( !MCUStatus.bit.InverterInhibit &&
         InverterInhibitNumBypassTransfers > 0 )
    {
        if ( InverterInhibitNumBypassTransfers >= INVERTER_INHIBIT_TRANSFER_LIMIT ||
             InverterInhibitTimer.CheckTimeout( INVERTER_INHIBIT_RESET_TIMEOUT ) )
        {
            if ( InverterInhibitNumBypassTransfers >= INVERTER_INHIBIT_TRANSFER_LIMIT &&
                 NB_GetNodebit( UPM_NB_UPS_ON_BYPASS ) )
            {
                StayOnBypass = INVERTER_INHIBIT_TIME;
            }
            else
            {
                ResetInverterInhibit();
            }
        }
    }

    if ( InverterInhibitNumBypassTransfers >= INVERTER_INHIBIT_TRANSFER_LIMIT && 
         StayOnBypass )
    {
        MCUStatus.bit.InverterInhibit = 1;
        NB_SetNodebit( UPM_NB_TOO_MANY_INVERTER_TRANSFERS, true );
    }
    else 
    {
        if ( MCUStatus.bit.ESSLockout && ( ESSNumOutages >= ESSMaxNumOutages ) )
        {
                // For now, use the same nodebit for ESS to indicate lockout. Create another NB if not good.
            NB_SetNodebit( UPM_NB_TOO_MANY_INVERTER_TRANSFERS, true, 355 );
        }
        else
        {
            NB_SetNodebit( UPM_NB_TOO_MANY_INVERTER_TRANSFERS, false );
        }        

        if ( MCUStatus.bit.InverterInhibit )
        {
            ResetInverterInhibit();
        }
    }

    NB_DebounceAndQue( UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP, AbnormalOutVoltage, AbnormalOutVoltage );
    
    //Check HE not available
    if( !ESSInstalled                        ||
        !ESSEnabled                          ||
        !( ONLINE_STATE == McuMachineState ) ||
        ( 0 == ESSFaultCondition )             )
    {
        NB_DebounceAndQue( UPM_NB_HE_NOT_AVAILABLE, false );
    }
    else
    {
        NB_DebounceAndQue( UPM_NB_HE_NOT_AVAILABLE, true, ESSFaultCondition );
    }

}



// ********************************************************************************************************
// *
// * Function:    void CheckForStuckRelay( void )
// *
// * Purpose:     Check for welded/stuck/failed inverter output relay
// *
// * Description: Compares inverter voltage samples to output voltage samples. 
// *              If different from zezo (there is a window) but very close to each other,
// *              declare the relay as failed. Only compare on bypass, after opening
// *              inverter output relays.
// *
// *              Call from MCU State, 5.1kHz, only while on bypass
// ********************************************************************************************************
void MCUStateControl::CheckForStuckRelay( void )
{
    const float relayFailLim = 0.05f;       // 5% to allow for calibration error

    uint16_t invContactorFailure = 0;
    stThreePhase limit;
    stThreePhase difflimit;

    limit.phA = OutputVoltageRMS.RawRMS.phA * ( 1.0f - relayFailLim );
    limit.phB = OutputVoltageRMS.RawRMS.phB * ( 1.0f - relayFailLim );
    limit.phC = OutputVoltageRMS.RawRMS.phC * ( 1.0f - relayFailLim );

    difflimit.phA = OutputVoltageRMS.RawRMS.phA * relayFailLim;
    difflimit.phB = OutputVoltageRMS.RawRMS.phB * relayFailLim;
    difflimit.phC = OutputVoltageRMS.RawRMS.phC * relayFailLim;

    if ( NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE ) )
    {
        // Stick on bypass at least ten seconds after resetting alarm. If the alarm is real,
        // it should have plenty of time to re-appear.
        StayOnBypass = TEN_SECONDS;
    }
    else
    {
        if ( !Inverter.GetStatus().bit.InverterOn               &&
             !NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_CLOSED ) &&
             !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) )
        {
            if ( ( InverterVoltageRMS.RawRMS.phA > limit.phA ) &&
                 ( fabs( RawAdcDataPtr->st.InverterVoltage.phA - RawAdcDataPtr->st.OutputVoltageRaw.phA ) < difflimit.phA ) )
        	{
                invContactorFailure = 1;
            }
            if ( ( InverterVoltageRMS.RawRMS.phB > limit.phB ) &&
                 ( fabs( RawAdcDataPtr->st.InverterVoltage.phB - RawAdcDataPtr->st.OutputVoltageRaw.phB ) < difflimit.phB ) )
            {
                invContactorFailure *= 10;
                invContactorFailure += 2;
            }
            if ( ( InverterVoltageRMS.RawRMS.phC > limit.phC ) &&
                 ( fabs( RawAdcDataPtr->st.InverterVoltage.phC - RawAdcDataPtr->st.OutputVoltageRaw.phC ) < difflimit.phC ) )
            {
                invContactorFailure *= 10;
                invContactorFailure += 3;
            }

            NB_DebounceAndQue( UPM_NB_INVERTER_CONTACTOR_FAILURE, invContactorFailure, invContactorFailure );
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_INVERTER_CONTACTOR_FAILURE, 0, 0 );
        }
    }
}

// ********************************************************************************************************
// *
// * Function:    void CheckForFailedRelayFast( void )
// *
// * Purpose:     Check for failed-open inverter output relay
// *
// * Description: Compares inverter voltage samples to output voltage samples. 
// *              If different from not close to each other,
// *              declare the relay as failed. Only compare in ESS.
// *
// *              Call from MCU State, 5.1kHz, only while in ESS
// ********************************************************************************************************
void MCUStateControl::CheckForFailedRelayFast( void )
{
    // 5% to allow for calibration error and raw sample variation
    const float relayFailLim = 0.05f;

    stThreePhase difflimit = {
    	OutputVoltageRMS.RawRMS.phA * relayFailLim,
    	OutputVoltageRMS.RawRMS.phB * relayFailLim,
    	OutputVoltageRMS.RawRMS.phC * relayFailLim
    };
    
    if ( NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE ) )
    {
        // Stick on bypass at least ten seconds after resetting alarm. If the alarm is real,
        // it should have plenty of time to re-appear.
        StayOnBypass = TEN_SECONDS;
    }
    else
    {
    	if (!Inverter.GetStatus().bit.InverterOn &&
    		NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_CLOSED ))
		{
		    uint16_t invContactorFailure = 0;
            if ( ( fabs( RawAdcDataPtr->st.InverterVoltage.phA - RawAdcDataPtr->st.OutputVoltageRaw.phA ) > difflimit.phA ) )
            {
                invContactorFailure = 1;
            }
            if ( ( fabs( RawAdcDataPtr->st.InverterVoltage.phB - RawAdcDataPtr->st.OutputVoltageRaw.phB ) > difflimit.phB ) )
            {
                invContactorFailure *= 10;
                invContactorFailure += 2;
            }
            if ( ( fabs( RawAdcDataPtr->st.InverterVoltage.phC - RawAdcDataPtr->st.OutputVoltageRaw.phC ) > difflimit.phC ) )
            {
                invContactorFailure *= 10;
                invContactorFailure += 3;
            }

            NB_DebounceAndQue( UPM_NB_INVERTER_CONTACTOR_FAILURE, invContactorFailure, invContactorFailure );
		}
		else
		{
			NB_DebounceAndQue( UPM_NB_INVERTER_CONTACTOR_FAILURE, 0, 0);
		}
    }
}

// ********************************************************************************************************
// *
// * Function:    void ResetSticky( void )
// *
// * Purpose:     Reset sticky type alarms and bits in state machine.
// *
// ********************************************************************************************************
void MCUStateControl::ResetSticky( void )
{
    MCUStatus.bit.BypassLoadOffHistory = 0;
    ResetInverterInhibit();
    Rectifier.ResetRectifierAlarms();
    ResetESSLockout();
    ResetAutoRestart();
    AbnormalOutVoltage = ABNORMALOUTV_OK;
}



// ********************************************************************************************************
// *
// * Function:    void ResetInverterInhibit( void )
// *
// * Purpose:     Reset "inverter inhibit" counters and timers.
// *
// ********************************************************************************************************
void MCUStateControl::ResetInverterInhibit( void )
{
    if ( MCUStatus.bit.InverterInhibit )
    {
        StayOnBypass = 0;
    }
    MCUStatus.bit.InverterInhibit = 0;
    FailedInverterTransfers = 0;
    InverterInhibitNumBypassTransfers = 0;
    InverterInhibitTimer.ClearTimer();
}


// ********************************************************************************************************
// *
// * Function:    void ResetESSLockout( void )
// *
// * Purpose:     Reset ESS lockout counters and timers.
// *
// ********************************************************************************************************
void MCUStateControl::ResetESSLockout( void )
{
    ESSNumOutages = 0;
    MCUStatus.bit.ESSLockout = 0;

        // Initialize timers
    ESSOutageResetTimer.ClearTimer();
    ESSLockoutTimer.ClearTimer();    
}


// ********************************************************************************************************
// *
// * Function:     bool ResetTransferState( void )
// *
// * Parms Passed: N/A
// * Returns:      N/A
// *
// * Purpose:      Cancels any state machine controlled transfer.
// *
// ********************************************************************************************************
void MCUStateControl::ResetTransferState( void )
{
    TransferTimer.ClearTimer();
    TransferPhaseState = 0;
    MCUStatus.bit.TransferReady = false;
    ZeroCross.Disarm();
}


// ********************************************************************************************************
// *
// * Function:     bool PrepareForBypassTransfer( void )
// *
// * Parms Passed: N/A
// * Returns:      true if immediate transfer possible
// *
// * Purpose:      Prepare for transfers involving the bypass.
// *
// ********************************************************************************************************
bool MCUStateControl::PrepareForBypassTransfer( void )
{
    bool transferReady = false;
    
    switch ( TransferPhaseState )
    {
    	case 0:
    	    MCUStatus.bit.TransferReady = false;
    	    // Wait for every UPM to receive the command before starting
    	    // This timer prevents UPM's that receive commands less than 10ms early
    	    // from going to online/bypass/bypass standby without the other UPM's
    	    if( TransferTimer.CheckTimeout( WAIT_10_MS ) )
    	    {
    	    	TransferPhaseState++;
    	    }
    	    break;
    	    
        case 1:
            // This will cause you to ready the bypass if you are transferring from
            // idle, but keep it on if you are transferring from fire
            if ( BypassState().GetMonitoredBypassState() < BYPASS_READY_STATE )
            {
                BypassState().RequestBypassState( BYPASS_READY_STATE );
            }
            TransferTimer.ClearTimer();
            TransferPhaseState++;
            break;

        case 2:
            if ( BypassState().GetMonitoredBypassState() >= BYPASS_READY_STATE &&
                 !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE ) )
            {
                ZeroCross.Arm( BYPASS_ZERO_CROSSING );
                TransferPhaseState++;
            }
            break;
            
        case 3:
            if ( ZeroCross.Check() )
            {
                TransferTimer.ClearTimer();
                TransferPhaseState++;
            }
            break;
            
        case 4:
            if ( TransferTimer.CheckTimeout( WAIT_3_MS ) )
            {
                MCUStatus.bit.TransferReady = true;
                TransferPhaseState++;
            }
            break;
            
        case 5:
            if ( ParallelCan.ParGlobalAndData.McuStatus.bit.TransferReady )
            {
                ZeroCross.Arm( BYPASS_ZERO_CROSSING );
                TransferPhaseState++;
            }
            break;
            
        case 6:
            if ( ZeroCross.Check() )
            {
                transferReady = true;
            }
            break;
            
        default:
            break;
    }
    
    return transferReady;
}

// ********************************************************************************************************
// *
// * Function:     bool PrepareForInverterTransfer( void )
// *
// * Parms Passed: N/A
// * Returns:      true if immediate transfer possible
// *
// * Purpose:      Prepare for transfers involving the inverter.
// *
// ********************************************************************************************************
bool MCUStateControl::PrepareForInverterTransfer( void )
{
    bool transferReady = false;
    
    switch ( TransferPhaseState )
    {
    	case 0:
    	    MCUStatus.bit.TransferReady = false;
    	    // Wait for every UPM to receive the command before starting
    	    // This timer prevents UPM's that receive commands less than 10ms early
    	    // from going to online/bypass/bypass standby without the other UPM's
    	    if( TransferTimer.CheckTimeout( WAIT_10_MS ) )
    	    {
    	    	TransferPhaseState++;
    	    }
    	    break;
    	
        case 1:
            ZeroCross.Arm( INVERTER_ZERO_CROSSING );
            TransferPhaseState++;
            break;
            
        case 2:
            if ( ZeroCross.Check() )
            {
                TransferTimer.ClearTimer();
                TransferPhaseState++;
            }
            break;
            
        case 3:
            if ( TransferTimer.CheckTimeout( WAIT_3_MS ) )
            {
                MCUStatus.bit.TransferReady = true;
                TransferPhaseState++;
            }
            break;
            
        case 4:
            if ( ParallelCan.ParGlobalAndData.McuStatus.bit.TransferReady )
            {
                ZeroCross.Arm( INVERTER_ZERO_CROSSING );
                TransferPhaseState++;
            }
            break;
            
        case 5:
            if ( ZeroCross.Check() )
            {
                transferReady = true;
            }
            break;
        
        default:
            break;
    }

    return transferReady;
}

// ********************************************************************************************************
// *
// * Function:     bool StartupRectifier( void )
// *
// * Parms Passed: N/A
// * Returns:      false if all 3 startup attempts failed.
// *
// * Purpose:      Handles starting up of rectifier, including 3 retries
// *
// ********************************************************************************************************
bool MCUStateControl::StartupRectifier( void )
{
    uRectifierStatus rectStatus = Rectifier.GetStatus();

    if ( Rectifier.CheckRectifierFailure() ||
         NB_GetNodebit( UPM_NB_RECTIFIER_FAILED ) ||
		Rectifier.GetStatus().bit.ACPreChargeFail )
    {
        if ( RectifierPhaseState > 0 )
        {
            if ( RectifierStartupAttempts >= 3 )
            {
                if( !Rectifier.GetStatus().bit.ACPreChargeFail )
                {
                    NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, true );
                }
                RectifierStartupAttempts = 0;
            }
        }
        RectifierPhaseState = 0;
        RectifierTimer.ClearTimer();
    }
    else
    {
        switch ( RectifierPhaseState )
        {
            case 0:
                if ( !rectStatus.bit.RectifierPrecharging &&
                     !NB_GetNodebit( UPM_NB_RECTIFIER_ON ) )
                {
                    Rectifier.StartupRectifier();
                    RectifierStartupAttempts++;

                    RectifierTimer.ClearTimer();
                    RectifierPhaseState++;
                }
                break;
            case 1:
                if ( !rectStatus.bit.RectifierPrecharging && 
                     !NB_GetNodebit( UPM_NB_RECTIFIER_ON ) )
                {
                    if ( RectifierTimer.CheckTimeout(ONE_SECOND) )
                    {
                        RectifierTimer.ClearTimer();
                        RectifierPhaseState++;
                    }
                }
                break;
            case 2:
                    // Rectifier Startup Failed
                if ( RectifierStartupAttempts < 3 )
                {
                    // Allow precharger to cool for 10 seconds before another
                    // attempt.  Safety also checked in the rectifier state
                    // machine.
                    if (RectifierTimer.CheckTimeout(ONE_SECOND * 10))
                    {
                        RectifierPhaseState = 0;
                    }
                }
                else
                {
                    NB_SetNodebit( UPM_NB_RECTIFIER_FAILED, true );
                    RectifierStartupAttempts = 0;
                    RectifierTimer.ClearTimer();
                }
                break;
                
            default:
                break;
        }
    }

    return !NB_GetNodebit( UPM_NB_RECTIFIER_FAILED );
}

// ********************************************************************************************************
// *
// * Function:     bool StartupBatteryConverter( void )
// *
// * Parms Passed: None
// * Returns:      false if all 3 startup attempts failed.
// *
// * Purpose:      Handles starting up of Battery Converter, including 3 retries
// *
// ********************************************************************************************************

bool MCUStateControl::StartupBatteryConverter( void )
{
    //Remove PreventBatStart to fix PANDA JIRA 1544.
    if ( BatteryConverter.CheckBatteryShutdown() ||
         NB_GetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE ) )
    {
        if (BatteryPhaseState > 0)
        {
            if ( BatteryStartupAttempts >= 3 )
            {
                BatteryStartupAttempts = 0;
                NB_SetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE, true, 1 );
            }
        }
        BatteryPhaseState = 0;
        BatteryStartTimer.ClearTimer();
    }
    else
    {
        switch ( BatteryPhaseState )
        {
            case 0:
                if ( !BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging &&
                     NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
                {
                    BatteryConverter.StartupBatteryConverter();//transfer to battery state

                    BatteryStartupAttempts++;
                    BatteryStartTimer.ClearTimer();
                    BatteryPhaseState++;
                }
                break;
            case 1:
                if (BatteryStartTimer.CheckTimeout(ONE_SECOND))
                {
                    if ( !BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging &&
                         NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
                    {
                        BatteryPhaseState++;
                        BatteryStartTimer.ClearTimer();
                    }
                }
                break;
            case 2:
                if (BatteryStartupAttempts < 3)
                {
                    if (BatteryStartTimer.CheckTimeout(ONE_SECOND * 10))
                    {
                        BatteryPhaseState = 0;
                        BatteryStartTimer.ClearTimer();
                    }
                }
                else
                {
                    NB_SetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE, true, 1 );
                    BatteryStartupAttempts = 0;
                }

                break;
            default:
                break;
        }
    }

    return !NB_GetNodebit( UPM_NB_BATTERY_STARTUP_FAILURE );
}

// ********************************************************************************************************
// *
// * Function:     bool IsMatchedOutput( void )
// *
// * Parms Passed: N/A
// * Returns:      true when InverterVoltageRMS and OutputVoltageRMS is little different
// *
// * Purpose:      
// *
// ********************************************************************************************************
bool MCUStateControl::IsMatchedOutput(void)		
{
	bool state = false;
	
	if ( ( std::fabs( OutputVoltageRMS.RawRMS.phA - InverterVoltageRMS.RawRMS.phA ) < 0.1f * (float)DiffVoltInvOutRms_X10) &&
		 ( std::fabs( OutputVoltageRMS.RawRMS.phB - InverterVoltageRMS.RawRMS.phB ) < 0.1f * (float)DiffVoltInvOutRms_X10  ) &&
		 ( std::fabs( OutputVoltageRMS.RawRMS.phC - InverterVoltageRMS.RawRMS.phC ) < 0.1f * (float)DiffVoltInvOutRms_X10  ) )
	{
		 state = true;
	}
	 return( state );
}  


// ********************************************************************************************************
// *
// * Function:     bool ShutdownInverter( void )
// *
// * Parms Passed: N/A
// * Returns:      true when inverter is shut down and relays opened
// *
// * Purpose:      
// *
// ********************************************************************************************************
bool MCUStateControl::ShutdownInverter( void )
{
    uInverterStatus iStat = Inverter.GetStatus();
    bool done = false;

    if ( iStat.bit.InverterOn )
    {
        Inverter.Off();
        Inverter.MatchBypassVoltage( false );
        InvRelayTimer.ClearTimer();
    }
    else if ( RELAY_CLOSED   == GetInverterRelayState() &&
              ESS_MODE_STATE != McuMachineState)            
    {
    	// If we are shutting down the inverter immediately following an ETB, give the output
    	// extra time to stabilize to minimize the current transient on the inverter relays.
        if ( // ( ParallelCan.ParGlobalOrData.McuStatus.bit.EmergencyTransferToBypass &&
             //   InvRelayTimer.CheckTimeout( WAIT_40_MS ) )  ||
             InvRelayTimer.CheckTimeout( WAIT_10_MS ) )
        {
            SetInverterRelay( RELAY_OPEN );
            InvRelayTimer.ClearTimer();
        }
        #if defined(APAC_VERSION)
        if( DeadTimeConfig == CfgDeadTimeDynamic )
        {
            if(UpmModel == DeadTime1_4us)//fix CPSDVAVE-40 for 20-80K
        	{
        		UpmModel = DeadTime2_0us;//1.4us->2.0us
        		DeadtimeConfEnd = false;
        	}
        }
        #endif
    }
    else
    {
        done = true;
        InvRelayTimer.ClearTimer();        
    }

    return done;
}
bool MCUStateControl::ShutdownInverterForATB( void )
{
    uInverterStatus iStat = Inverter.GetStatus();
    bool done = false;

    if ( iStat.bit.InverterOn )
    {
        Inverter.Off();
        Inverter.MatchBypassVoltage( false );
        InvRelayTimer.ClearTimer();
    }
    else if ( RELAY_CLOSED   == GetInverterRelayState() &&
              ESS_MODE_STATE != McuMachineState)            
    {
        SetInverterRelay( RELAY_OPEN );
        InvRelayTimer.ClearTimer();
	   #if defined(APAC_VERSION)
        if( DeadTimeConfig == CfgDeadTimeDynamic )
        {
        	if(UpmModel == DeadTime1_4us)//fix CPSDVAVE-40 for 20-80K
        	{
        		UpmModel = DeadTime2_0us;//1.4us->2.0us
        		DeadtimeConfEnd = false;
        	}
        }
       #endif
    }
    else
    {
        done = true;
        InvRelayTimer.ClearTimer();        
    }

    return done;
}

// ********************************************************************************************************
// *
// * Function:    uint16_t LoadOffToNormalPersistentOk( void )
// *
// * Purpose:     Hard conditions to allow standby/shutdown to online transfers, failing this clears the
// *              normal command.
// *
// ********************************************************************************************************
bool MCUStateControl::LoadOffToNormalPersistentOk( void )
{
    return ( 
        !NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )            &&
        ( !NB_GetNodebit( UPM_NB_MBS_CLOSED )                     ||
          NB_GetNodebit( UPM_NB_MOB_OPEN ) )                    &&
        !NB_GetNodebit( UPM_NB_CONFIGURATION_ERROR )            &&
        !NB_GetNodebit( UPM_NB_PARALLEL_SETUP_FAIL )            &&
        !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )             &&
        !NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE )     &&
        !NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE )       &&
        !NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )  &&
        !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )        &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP ) &&
        !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )            &&
        !NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )      &&
        !NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )     &&
        !NB_GetNodebit( UPM_NB_DRIVER_FAULT )                   &&
        !NB_GetNodebit( UPM_NB_INVERTER_STARTUP_FAILURE )       &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_FAILED )               &&
        !NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT )              &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.StaticSwitchShort &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.InternalMaintenanceBypass
        );
}

// ********************************************************************************************************
// *
// * Function:    uint16_t BypToNormalPersistentOk( void )
// *
// * Purpose:     Hard conditions to allow bypass to online transfers, failing this clears commands
// *
// ********************************************************************************************************
bool MCUStateControl::BypToNormalPersistentOk( void )
{
    return ( 
        !NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )           &&
        ( !NB_GetNodebit( UPM_NB_MBS_CLOSED )                    ||
          NB_GetNodebit( UPM_NB_MOB_OPEN ) )                   &&
        !NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION )         &&
        !NB_GetNodebit( UPM_NB_OUTPUT_PHASE_ROTATION )         &&
        !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )           &&
        !NB_GetNodebit( UPM_NB_PULL_CHAIN )                    &&
        !NB_GetNodebit( UPM_NB_CONFIGURATION_ERROR )           &&
        !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )         &&
        !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )            &&
        !NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE )    &&  // for now, lock on bypass if relay failed.
        !ParallelCan.ParGlobalOrData.McuStatus.bit.BldInpRemoteGotoBypass  &&
        !NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE )      &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.StaticSwitchShort    &&
        !ParallelCan.ParGlobalOrData.McuStatus.bit.InternalMaintenanceBypass
        );
}

// ********************************************************************************************************
// *
// * Function:    uint16_t BypToNormalOk( void )
// *
// * Purpose:     Hard conditions to allow bypass to online transfers
// *
// ********************************************************************************************************
bool MCUStateControl::BypToNormalOk( void )
{
    return ( 
         BypToNormalPersistentOk()                             &&
        !NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )       &&
        !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )        &&
        !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )   &&
        !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )       &&
        !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )        &&
        !NB_GetNodebit( UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY )   &&
        !NB_GetNodebit( UPM_NB_BATTERY_LOW )                   &&
        ( BatteryConverter.BatteryAvailable()                ||
//	          EEP_BattNotRequiredToGoOnline() )                    &&
		(EEP_BattNotRequiredToGoOnline() && Rectifier.RectifierOnReal))   &&	  //for jira Wombat318
        !NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD )               &&
        !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )          &&
        !NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )         &&
        !NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )  &&
        !NB_GetNodebit( UPM_NB_INVERTER_AC_UNDER_VOLTAGE )     &&
        !NB_GetNodebit( UPM_NB_INVERTER_STARTUP_FAILURE )      &&
        !NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP ) &&
	    !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )       
        );
}

// ********************************************************************************************************
// *
// * Function:    bool CheckOKToFlash( void )
// *
// * Purpose:     For now, unit must be off to flash
// *
// ********************************************************************************************************
bool MCUStateControl::CheckOKToFlash( void )
{
    bool OK = false;
    
    if ( ( ( INITIALIZATION_STATE == McuMachineState ) || 
           ( SHUTDOWN_STATE == McuMachineState )       ||
           ( BYPASS_STATE == McuMachineState) ) &&
         !Rectifier.GetStatus().bit.RectifierOnNormal &&
         !Rectifier.GetStatus().bit.RectifierOnBattery &&
         !Inverter.GetStatus().bit.InverterOn )
    {
        OK = true;
    }
    
    return OK;
}

// *****************************************************************************
// *
// * Function:    bool SupportingLoad( void )
// *
// * Purpose:     Global predicate that is true when the UPS is "supporting the
// *              load" and false otherwise.
// *
// *****************************************************************************
bool MCUStateControl::SupportingLoad(void) const
{
    return McuMachineState == ONLINE_STATE                ||
            McuMachineState == BYPASS_STATE               ||
            McuMachineState == BYPASS_STANDBY_STATE       ||
            McuMachineState == ESS_MODE_STATE             ||
            McuMachineState == EASY_CAPACITY_TEST_STATE;
}

// *****************************************************************************
// *
// * Function:    bool InStandby( void )
// *
// * Purpose:     Global predicate that is true when the UPS is "standby" 
// *               and false otherwise.
// *
// *****************************************************************************
bool MCUStateControl::InStandby(void) const
{
    return McuMachineState == STANDBY_STATE                ||
            McuMachineState == BYPASS_STANDBY_STATE;
}

// *****************************************************************************
// *
// * Function:    bool InvSupportingLoad( void )
// *
// * Purpose:     Global predicate that is true when the UPS inverter 
// *               is somehow "supporting the load" and false otherwise.
// *
// *****************************************************************************
bool MCUStateControl::InvSupportingLoad(void) const
{
    return McuMachineState == ONLINE_STATE                ||
            McuMachineState == ESS_MODE_STATE             ||
            McuMachineState == EASY_CAPACITY_TEST_STATE;
}

bool MCUStateControl::BypassSupportingLoad(void) const
{
    return McuMachineState == BYPASS_STATE         ||
           McuMachineState == BYPASS_STANDBY_STATE ||
           McuMachineState == ESS_MODE_STATE;         
}    
    
// *****************************************************************************
// *
// * Function:    ECTOnCommand( void )
// *
// * Purpose:     To transfer to Line Easy Capacity Test mode 
// *       
// *
// *****************************************************************************
bool MCUStateControl::ECTLineCommand(void)
{
    bool done = false;
    
    if( //MCUStatus.bit.NLFLEnable                          &&
        McuMachineState == ONLINE_STATE                 &&
        !ForceStayOnline                                    &&
        Rectifier.GetStatus().bit.RectifierOnNormal     &&
        !MCUStatus.bit.ToECTCmd                         &&
        !MCUStatus.bit.InhibitToECT
       )
    {
        CriticalSection enter;
        MCUStatus.bit.NormalCmd   = 0;
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.ToECTCmd = 1;
        MCUStatus.bit.ESSCmd = 0;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.StandbyCmd  = 0;
        ECTPhase = 4;
        done = true;
    }
    else
    {
//      MCUStatus.bit.NLFLEnable = 0;
        done = false;
    }
    
    return done;
}

// *****************************************************************************
// *
// * Function:    ECTBatteryCommand( void )
// *
// * Purpose:     To transfer to Battery Easy Capacity Test mode 
// *       
// *
// *****************************************************************************
bool MCUStateControl::ECTBatteryCommand( void )
{
    bool done = false;
                        
    if( //MCUStatus.bit.NLFLEnable                          &&
        NB_GetNodebit( UPM_NB_BATTERY_INSTALLED )           &&
        !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )     &&
        !NB_GetNodebit( UPM_NB_BATTERY_TEST_IN_PROGRESS )   &&
        !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN )     &&
        !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_FAIL )     &&
        !MCUStatus.bit.ToECTCmd                             &&
        McuMachineState == ONLINE_STATE                     &&
        !ForceStayOnline                                    &&
        Rectifier.GetStatus().bit.RectifierOnNormal         &&
        !MCUStatus.bit.InhibitToECT
       )
    {
        CriticalSection enter;
        MCUStatus.bit.NormalCmd   = 0;
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.ToECTCmd= 1;
        Rectifier.ToBatteryECT();
        MCUStatus.bit.ESSCmd = 0;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.StandbyCmd  = 0;
        ECTPhase = 5;
        done = true;
    }
    else
    {
//      MCUStatus.bit.NLFLEnable = 0;
        done = false;
    }
    
    return done;
}

// *****************************************************************************
// *
// * Function:    ECTCommand( void )
// *
// * Purpose:     To transfer to Easy Capacity Test mode 
// *       
// *
// *****************************************************************************
bool MCUStateControl::ECTCommand( void )
{
    bool done = false;
    
    if( MCUTimerBatECT.CheckTimeout( ECTBatTimerOut ) )
    {
        if( MCUTimerLineECT.CheckTimeout( ECTLineTimerOut ) )
        {
            done = false;
        }
        else
        {
            MCUTimerLineECT.ClearTimer();                       
            done = ECTLineCommand();
        }
    }
    else
    {
        MCUTimerBatECT.ClearTimer();
        done = ECTBatteryCommand();
        ECTPhase = 0;
    }
    return done;

}

// *****************************************************************************
// *
// * Function:    ECTOffCommand(void)
// *
// * Purpose:     To transfer to Normal mode from ECT mode 
// *               
// *
// *****************************************************************************
void MCUStateControl::ECTClearCommand( void )//ECTExitCommand(void)
{
    if( MCUStatus.bit.ToECTCmd )
    {
        CriticalSection enter;
        MCUStatus.bit.ToECTCmd = 0;
        Rectifier.ClearBatteryECT();
//        MCUStatus.bit.NLFLEnable = 0;
        MCUStatus.bit.AutoStandbyCmd = 0;
        MCUStatus.bit.ESSCmd = 0;
        MCUStatus.bit.NormalCmd   = 1; 
        MCUStatus.bit.ToBypassCmd = 0;
        MCUStatus.bit.ShutdownCmd = 0;
        MCUStatus.bit.StandbyCmd  = 0;
    }
//    else
//    {
//        MCUStatus.bit.NLFLEnable = 0;
//    }
}


// ********************************************************************************************************
// *
// * Function:    uint16_t BypToECTPersistentOk( void )
// *
// * Purpose:     Hard conditions to allow bypass to ECT transfers, failing this clears commands
// *
// ********************************************************************************************************
bool MCUStateControl::BypToECTPersistentOk( void )
{
    return (
        !FrequencyConverterMode                                 &&
        !NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )     &&
        !MCUStatus.bit.BldInpRemoteGotoBypass                   &&
        !NB_GetNodebit( UPM_NB_PULL_CHAIN )                     &&
        !NB_GetNodebit( UPM_NB_CONFIGURATION_ERROR )            &&
        ( !NB_GetNodebit( UPM_NB_MBS_CLOSED )                     ||
          NB_GetNodebit( UPM_NB_MOB_OPEN ) )                    &&
        !NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE )            &&
        !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR )             &&        
        !NB_GetNodebit( UPM_NB_FAN_FAILURE )                    &&
        !NB_GetNodebit( UPM_NB_BYPASS_AC_OVER_VOLTAGE )         &&
        !NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE )        &&
        !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )    &&
        !NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION )          &&
        !NB_GetNodebit( UPM_NB_BYPASS_NOT_AVAILABLE )           &&
        !NB_GetNodebit( UPM_NB_STATIC_SWITCH_FAILURE )          &&
        !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )            &&
        !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )        &&
        !NB_GetNodebit( UPM_NB_OUTPUT_AC_OVER_VOLTAGE )         &&
        !NB_GetNodebit( UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY )    &&
        !NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )           &&   //Jacob/20130909/UPM_NB_OUTPUT_OVERLOAD->UPM_NB_OUTPUT_OVERLOAD_TRIP
        !NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )           &&
        !NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )          &&
        !NB_GetNodebit( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT )   &&        
        !NB_GetNodebit( UPM_NB_INVERTER_AC_UNDER_VOLTAGE )      &&
        !NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )      &&
        !NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE )     &&
        !NB_GetNodebit( UPM_NB_FUSE_FAILURE )                   &&
        !NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE_TRIP )  &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP ) &&
        !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP )        &&
        !NB_GetNodebit( UPM_NB_BATTERY_TEST_IN_PROGRESS )       &&
        !NB_GetNodebit( UPM_NB_BATTERY_CURRENT_LIMIT )          && 
        !NB_GetNodebit( UPM_NB_LOW_BATTERY_SHUTDOWN )           &&
        !NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR )               &&
        !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT )            &&
        !NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE )          &&
        !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE)          &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT )   &&
        !NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY )     &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_PHASE_ROTATION )       &&
        !NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE )        &&
        !NB_GetNodebit( UPM_NB_LOSS_OF_PWM_SYNC )
        );
}

// ********************************************************************************************************
// *
// * Function:    bool KeepAliveShutdown( void )
// *
// * Purpose:     Returns true if you should shut down the rectifier due to keep alive
// *
// ********************************************************************************************************
bool MCUStateControl::KeepAliveShutdown( bool onBattery )
{
    // Twenty seconds of time needed in case of user command
    if( onBattery )
    {
		//keep alive same as 9P, for jira hobbit-72
        if ( KeepAlive.CheckTimeout( KeepAlive_Delay * ONE_SECOND )                  ||
             ( ( KeepAlive.TimerValue() > TWENTY_SECONDS ) &&
               (( BattVoltsperCell < KeepAlive_BatUVPC) ||
			  ( BattVoltsperCell1 < KeepAlive_BatUVPC) ||
			  ( BattVoltsperCell2 < KeepAlive_BatUVPC)) ) )
        {
            return true;
        }		
    }
    else
    {
        KeepAlive.ClearTimer();
    }
    return false;
}
// ********************************************************************************************************
// *
// * Function:    bool Check_L3L4_OverLoad( bool )
// *
// * Purpose:     Check Level3,4 overload
// *
// ********************************************************************************************************
bool MCUStateControl::Check_L3L4_OverLoad ( void )
{
    return (
            NB_GetNodebit(UPM_NB_LEVEL_3_OVERLOAD_PHASE_A) ||
            NB_GetNodebit(UPM_NB_LEVEL_3_OVERLOAD_PHASE_B) ||
            NB_GetNodebit(UPM_NB_LEVEL_3_OVERLOAD_PHASE_C) ||
            NB_GetNodebit(UPM_NB_LEVEL_4_OVERLOAD_PHASE_A) ||
            NB_GetNodebit(UPM_NB_LEVEL_4_OVERLOAD_PHASE_B) ||
            NB_GetNodebit(UPM_NB_LEVEL_4_OVERLOAD_PHASE_C)
            );
}
// ********************************************************************************************************
// *
// * Function:    void BldInpRemoteCommand( uint16_t )
// *
// * Purpose:     Sets the status bit for remote command and transmits the command over the CAN 
// *              if necessary.
// *              Remote go to bypass has a higher priority than remote online and load off
// *
// ********************************************************************************************************
void MCUStateControl::BldInpRemoteCommand( uint16_t remoteCommand )
{
    CriticalSection enter;
    bool remoteOnline = (remoteCommand & (1<<BI_REMOTE_UPS_ON))>>BI_REMOTE_UPS_ON;
    bool remoteBypass = (remoteCommand & (1<<BI_REMOTE_GO_TO_BYPASS))>>BI_REMOTE_GO_TO_BYPASS;
    bool remoteLoadoff = (remoteCommand & (1<<BI_REMOTE_UPS_OFF))>>BI_REMOTE_UPS_OFF;

	/* UPM trigger alarm "parallel can failure", last for 10 seconds or worse */
	if ((ParallelCan.parallelSynchFlags & PARALLEL_SYNCH_FLAG_IDENTIFY_MASTER) != 0U)
	{
		return;
	}
    //Handle the remote operation. regard the remote operation as command from LCD of CSB
    if (!MCUStatus.bit.BldInpRemoteGotoBypass && remoteBypass)
    {
        ParallelCan.InternalCommand.bit.csb_bypass_on_command = 1;
    }
    else if(!MCUStatus.bit.BldInpRemoteUPSOn && !remoteBypass && remoteOnline)
    {
        ParallelCan.InternalCommand.bit.csb_on_normal_command = 1;
    }
    else if(!MCUStatus.bit.BldInpRemoteUPSOff && !remoteBypass && !remoteOnline && remoteLoadoff)
    {
        ParallelCan.InternalCommand.bit.csb_load_off_command = 1;
    }
    //following bit just used to judge the rising edge
    //fix remote building bypass active,remote bypass will be ignored because bypass available detect delay
	if ((ParallelCan.parallelSynchFlags & PARALLEL_SYNCH_FLAG_BYPASS_NOT_AVAILABLE) == 0U)
	{
		MCUStatus.bit.BldInpRemoteGotoBypass = remoteBypass;
	}
    MCUStatus.bit.BldInpRemoteUPSOn = !remoteBypass && remoteOnline;
    MCUStatus.bit.BldInpRemoteUPSOff = !remoteBypass && !remoteOnline && remoteLoadoff;

    if (!remoteBypass)
    {
        MCUStatus.bit.BypassLoadOffHistory = 0;
    }
}


// ********************************************************************************************************
// *
// * Function:    CheckEnoughActiveUPM
// *
// * Purpose:     Check if there are enough active UPM to support the current load
// *
// ********************************************************************************************************
void MCUStateControl::CheckEnoughActiveUPM( void )
{
    float activeUPM_VA_L = float(OutputkVARating) * ParallelCan.PCan_SumOnInverter() / 3.0f;
    float activeUPM_W_L = float(OutputPowerFactorRating) * activeUPM_VA_L;
    float hysteresis = 0.05f * activeUPM_VA_L;
    
    EnoughActiveUPM.Debounce_Hysterisis(activeUPM_VA_L > LoadPower.TotalPower.phA &&
                                        activeUPM_VA_L > LoadPower.TotalPower.phB &&
                                        activeUPM_VA_L > LoadPower.TotalPower.phC &&
                                        activeUPM_W_L > LoadPower.ActivePower.phA &&
                                        activeUPM_W_L > LoadPower.ActivePower.phB &&
                                        activeUPM_W_L > LoadPower.ActivePower.phC,
                                        
                                        activeUPM_VA_L < (LoadPower.TotalPower.phA - hysteresis) ||
                                        activeUPM_VA_L < (LoadPower.TotalPower.phB - hysteresis) ||
                                        activeUPM_VA_L < (LoadPower.TotalPower.phC - hysteresis) ||
                                        activeUPM_W_L < (LoadPower.ActivePower.phA - hysteresis) ||
                                        activeUPM_W_L < (LoadPower.ActivePower.phB - hysteresis) ||
                                        activeUPM_W_L < (LoadPower.ActivePower.phC - hysteresis) );
}
//************************************************************************
