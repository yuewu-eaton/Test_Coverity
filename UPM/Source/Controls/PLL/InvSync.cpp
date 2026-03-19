// ******************************************************************************************************
// *            InvSync.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2006...2008 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: InvSync.cpp
// *
// *    DESCRIPTION: InvSync determines inverter sync source, controls inverter PLL
// *
// *    ORIGINATOR: Jason Anderson
// *
// *
// *    DATE: 7/9/2003
// *
// *    HISTORY: See cvs history
// ******************************************************************************************************


// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "adc.h"
#include "F28335Port.h"
#include "InverterControl.h"
#include "Algos.h"
#include "InvSync.h"
#include "MCUState.h"
#include "Eeprom_Map.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "ParallelCan.h"
#include "SineReference.h"
#include "ExtSignalPLL.h"
#include "StateTimer.h"

using namespace std;

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************
static void TransferSyncState( sync_states_t newState );

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
namespace {
    // based on the 1.25 kHz call rate for SyncState()
    const uint16_t SYNC_TIMEOUT_4MS = 5u;
    uSyncStatus         LocalSyncStatus = { 0uL };
}
sync_states_t       Sync_State = SYNC_STATE_BASE_FREE_RUN;
ext_sync_states_t   ExtSyncState = SYNC_SOURCE_BYPASS;

// ********************************************************************************************************
// *
// * Function: OuputSync(void);
// *
// * Purpose: Determines what the inverter syncs to (bypass, output, parallel packet, or base). Calculates
// *          the target frequency, phase error and determines phase lock.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Determines what the inverter syncs to (bypass, output, parallel packet, or base). Calculates
// *              the target frequency, phase error and determines phase lock.
// *
// ********************************************************************************************************
void InitSync(void)
{
    Inverter.SetSyncSource( SYNC_STATE_BASE_FREE_RUN );
    LocalSyncStatus.all = 0;
}

// ***********************************************************************
// *
// *    FUNCTION: TransferSyncState 
// *
// *    DESCRIPTION: Sets inverter sync source
// *
// *    ARGUMENTS: new state
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
static void TransferSyncState( sync_states_t newState )
{
    if ( ( newState < SYNC_STATE_MAX ) && ( newState != Sync_State ) )
    {
        {
            CriticalSection enter(IER_DMA_ONLY);
            if (newState == SYNC_STATE_BASE_FREE_RUN &&
                ( NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR) ||
                  ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed) &&
                OutputPLL.IsSourcePresent() )
            {
//	                // Keep the inverter output frequency stable during the transient.
//	                // That means the reference frequency must be equal to the output frequency
//	                // plus the compensation for the load share droop.  Only consider the DC
//	                // portion of the droop gain for now.
//	                // Set the base frequency to the current frequency + load share droop
                float currentFrequency = OutputPLL.SineRef.Frequency + 0.2f * Inverter.ActivePowerFiltered;
//	                BaseSineRef.UpdateFrequencyAndAngleStep( currentFrequency );
//	                // Step the inverter's frequency up to this value, too.
//	                Inverter.SineRef.UpdateFrequencyAndAngleStep( currentFrequency );

				// Add an offset to counter the load share droop
				BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.GetBaseFrequency());// + 0.1f );
            }
            else
            {
                // RT systems use the nominal frequency
                BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.GetBaseFrequency() );
            }
            Inverter.SetSyncSource( newState );
        }
        Sync_State = newState;
        LocalSyncStatus.bit.SyncState = Sync_State;
        NB_LogStateChange( UPM_NB_SYNC_STATE_CHANGED, Sync_State );
    }
    
}    

// ********************************************************************************************************
// *
// * Function: SyncState(void);
// *
// * Purpose: Determines what the inverter syncs to (bypass, output, parallel packet, or base). Calculates
// *          the target frequency, phase error and determines phase lock.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Determines what the inverter syncs to (bypass, output, input, or base).
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void SyncState(void)
{
    static StateTimer SyncMatchedTimer;
    bool bypassSyncOK = false;
    bool inputSyncOK = false;
    bool outputSyncOK = false;
    bool baseSyncExtOK = false;
    const uint16_t SyncQualCount = 5000;
	//Follow wombat:to avoid byp not avail(dis ATB) Vd_byp disturbed,then sync base then byp not avail( under MX source step 230->220)	
    const uint16_t SyncDisQualCount = 2;
    static FilteredBit bypassSyncGood( SyncQualCount, SyncDisQualCount, NOT_STICKY );
    static uint16_t bypassSyncDelay = 0;
    static uint16_t bypassSyncAvailable = 0;
    static FilteredBit inputSyncGood( SyncQualCount, SyncDisQualCount, NOT_STICKY );
    //
    // sync rules
    // 
    // priority
    //
    // ext sync-  internal parallel only
    //
    // output sync - only for parallel units, if load is energized AND this UPS (that's UPS not UPM) is not connected to the load 
    //
    // bypass sync - sync to bypass if bypass is within limits and parellel enabled
    //
    // utility sync - sync to input if utility is within limits and parellel enabled
    //
    // base - when none of the above apply
    //
    
    // internal parallel ext sync
    if ( EEP_ExtSyncEnabled()                                       &&
         !ParallelCan.UPSIsParallel()                               &&
         !NB_GetNodebit( UPM_NB_LOSS_OF_SYNC_BUS )                  && 
         !ParallelCan.ParGlobalOrData.SyncStatus.bit.AutoCalBypass  &&
         !ParallelCan.ParGlobalOrData.SyncStatus.bit.AutocalInput )
    {
        baseSyncExtOK = true;    
    }

    // sync to output OK?
    if ( ( ( MCUStateMachine.GetState() == STANDBY_STATE ) || 
           ( MCUStateMachine.GetState() == EASY_CAPACITY_TEST_STATE ) ) &&
           OutputPLL.IsSourcePresent() &&
          !MCUStateMachine.GetStatus().bit.OutputSyncDisable )
    {
        outputSyncOK = true;
    }

    if ( !BypassPLL.IsDQGoodForSync()                           ||
         NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )    ||
         FrequencyConverterMode )
    {
        bypassSyncGood.Debounce( false );
    }
    else
    {
        bypassSyncGood.Debounce( true );
    }

    //fix APACTS-565, if sync to base, UPS goto ATB, force sync to bypass first
    if( MCUStateMachine.MCUStatus.bit.ForceSyncBase )
    {
    	bypassSyncDelay = 0;
    	bypassSyncAvailable = 0;
    }
    else
    {
    	if( MCUStateMachine.ForceSyncBaseToByp )
		{
            bypassSyncAvailable = 1;
		}
    	else
    	{
    		if( ++bypassSyncDelay > 5000 )  //0.8ms * 5000 = 4s
			{
				bypassSyncDelay = 5000;
				bypassSyncAvailable = 1;
			}
    	}
    }

    LocalSyncStatus.bit.BypassSyncAvail = ( bypassSyncGood.GetState() && bypassSyncAvailable )? 1 : 0;
//	    LocalSyncStatus.bit.CanBusFailed = NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR);
    LocalSyncStatus.bit.CanBusFailed = NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR);
	//begin
	if(LocalSyncStatus.bit.CanBusFailed == true)
	{
		ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed = true;
		for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
		{
			for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
			{
			   ParallelCan.UpmData[ ups ][ upm ].SyncStatus.bit.CanBusFailed = true;
			}
		}
	}	

    // parallel consensus determines bypass available for sync
    if ( ((( ParallelCan.ParGlobalAndData.SyncStatus.bit.BypassSyncAvail &&
           LocalSyncStatus.bit.BypassSyncAvail                         &&
           !ParallelCan.ParGlobalOrData.SyncStatus.bit.AutocalInput    &&	
           !NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR))    ||
         ( NB_GetNodebit(UPM_NB_PULL_CHAIN))) 						&& 
         LocalSyncStatus.bit.BypassSyncAvail )						&&
         (!ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed) ) 						//&&
    {
        bypassSyncOK = true;
    }

    // parallel consensus determines input available for sync
    // TODO: Figure out why this check doesn't pass
//	    if ( false )
//	    // if ( ParallelCan.ParGlobalAndData.SyncStatus.bit.InputSyncAvail )
//	    {
//	        // daniel this gets set when CAN returns for some reason.  
//	        // jonathan - not observed yet
//	        inputSyncOK = true;
//	    }
	//20180504 begin: add for line&bat+nobyp paral
	if(EEP_InputSyncEnabled() 
		&& Rectifier.UtilityPLL.IsSourcePresent()
		&& !NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE ))
	{
        inputSyncGood.Debounce( true );
	}
	else
	{
        inputSyncGood.Debounce( false );
	}
    LocalSyncStatus.bit.InputSyncAvail = inputSyncGood.GetState();	

	if( ParallelCan.ParGlobalAndData.SyncStatus.bit.InputSyncAvail 	&&
		LocalSyncStatus.bit.InputSyncAvail 							&&
		!NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR)					&&
		!ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed )		
	{
		inputSyncOK = true;
	}
	//20180504 end
	
    // check who to sync to
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // stay at base until meters ready
        switch ( Sync_State )
        {
            case SYNC_STATE_BYPASS:
                if ( baseSyncExtOK )
                {
                    TransferSyncState( SYNC_STATE_BASE_EXT_SYNC );
                }
                else
                {
                        if ( !bypassSyncOK )
                        {
                            if ( outputSyncOK )
                            {
                                TransferSyncState( SYNC_STATE_OUTPUT );
                            }
                            else if ( inputSyncOK )
                            {
                                TransferSyncState( SYNC_STATE_INPUT );
                            }
                            else
                            {
                                TransferSyncState( SYNC_STATE_BASE_FREE_RUN );
                            }
                        }        
                }            
                break;
            
            case SYNC_STATE_INPUT:
                if ( baseSyncExtOK )
                {
                    TransferSyncState( SYNC_STATE_BASE_EXT_SYNC );
                }
                else
                {
                    if ( outputSyncOK )
                    {
                        TransferSyncState( SYNC_STATE_OUTPUT );
                    }
                    else
                    {
                        if ( bypassSyncOK )
                        {
                            TransferSyncState( SYNC_STATE_BYPASS );
                        }
                        else
                        {
                            if ( !inputSyncOK )
                            {
                                TransferSyncState( SYNC_STATE_BASE_FREE_RUN );
                            }
                        }
                    }            
                } 
                break;
            
            case SYNC_STATE_OUTPUT:
                if ( baseSyncExtOK )
                {
                    TransferSyncState( SYNC_STATE_BASE_EXT_SYNC );
                }
                else if( bypassSyncOK )
                {
                    TransferSyncState( SYNC_STATE_BYPASS );
                }
                else if( !outputSyncOK )
                {
                    if ( inputSyncOK )
                    {
                        TransferSyncState( SYNC_STATE_INPUT );
                    }
                    else
                    {
                        TransferSyncState( SYNC_STATE_BASE_FREE_RUN );
                    }
                }
                else
                {
                    
                }
                break;

            case SYNC_STATE_BASE_EXT_SYNC:
                if ( !baseSyncExtOK )
                {
                    if ( outputSyncOK )
                    {
                        TransferSyncState( SYNC_STATE_OUTPUT );
                    }
                    else
                    {
                        if ( bypassSyncOK )
                        {
                            TransferSyncState( SYNC_STATE_BYPASS );
                        }
                        else
                        {
                            if ( inputSyncOK )
                            {
                                TransferSyncState( SYNC_STATE_INPUT );
                            }
                            else
                            {
                                TransferSyncState( SYNC_STATE_BASE_FREE_RUN );
                            }
                        }    
                    }            
                }    
                break;            

            case SYNC_STATE_BASE_FREE_RUN:
            default:
                if ( baseSyncExtOK )
                {
                    TransferSyncState( SYNC_STATE_BASE_EXT_SYNC );
                }
                else
                {
                    if ( outputSyncOK )
                    {
                        TransferSyncState( SYNC_STATE_OUTPUT );
                    }
                    else
                    {
                        if ( bypassSyncOK )
                        {
                            TransferSyncState( SYNC_STATE_BYPASS );
                        }
                        else
                        {
                            if ( inputSyncOK )
                            {
                                TransferSyncState( SYNC_STATE_INPUT );
                            }
                        }    
                    }            
                }    
                break;            
        } // end switch sync_state

        // 
        // ext sync signal
        //
      //  ExternalSyncState();
    }
    else
    {
        TransferSyncState( SYNC_STATE_BASE_FREE_RUN );
    }
    
    if (ParallelCan.ParGlobalAndData.SyncStatus.bit.SyncState != Sync_State ||
        ParallelCan.ParGlobalOrData.SyncStatus.bit.SyncState != Sync_State)
    {
        // A 4ms retransmit time gives sufficient opportunity for 7 other nodes to
        // respond and transmit their change as well as allowing at least three 
        // transmit attempts per line cycle.
        if (SyncMatchedTimer.CheckTimeout(SYNC_TIMEOUT_4MS))
        {
            // Force retransmission by changing a bit
            LocalSyncStatus.bit.ForceRetransmit = !LocalSyncStatus.bit.ForceRetransmit;
            SyncMatchedTimer.ClearTimer();
        }
    }
    else
    {
        SyncMatchedTimer.ClearTimer();
    }
}// End of SyncState()

namespace {
	/*
	 * Clamp a floating-point value to be between a defined minimum and maximum range
	 * Precondition: min < max
	 */
	float clampf(float min, float value, float max)
	{
		if (value < min)
			return min;
		else if (value > max)
			return max;
		else
			return value;
	}
}
// ***********************************************************************
// *
// *    FUNCTION: ExternalSyncState 
// *
// *    DESCRIPTION: Sets sync reference for external sync out
// *
// *    ARGUMENTS: 
// *
// *    RETURNS:
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
static void ExternalSyncState( void )
{
//	// Four seconds
//    const uint16_t SyncQualCount = 5000;
//    const uint16_t SyncDisQualCount = 1;
//    // 10 degrees, in radians
//    const float PhaseErrorSyncLimit = PI/18.0f;
//    // 0.5 Hz/s, divided by the call rate
//    const float BaseSlewRate = 0.5f / 1250.0f;
//    const float FreqErrorSyncLimit = 0.05; // Hz
//
//    static FilteredBit bypassSyncGood( SyncQualCount, SyncDisQualCount, NOT_STICKY );
//    static FilteredBit inputSyncGood( SyncQualCount, SyncDisQualCount, NOT_STICKY );
//    static StateTimer canRestoredTimer;
//
//    if ( !BypassPLL.IsSourcePresent()                           ||
//         NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY )    ||
//         ( ParallelCan.UPSIsParallel()                &&            //PCAN fail, not enable sync to bypass
//           NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) &&
//           !SyncConfig.bit.EnableCanFailSyncToByp     &&
//           !NB_GetNodebit(UPM_NB_PULL_CHAIN) )                  ||
//         ( ParallelCan.UPSIsParallel()                &&            //PCAN fail, enable sync to bypass and bypass fail
//           NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) &&
//           SyncConfig.bit.EnableCanFailSyncToByp      &&
//           LocalSyncStatus.bit.BypFailAfterCanFail )  )
//    {
//        bypassSyncGood.Debounce( false );
//    }
//    else
//    {
//        bypassSyncGood.Debounce( true );
//    }
//
//    if ( !Rectifier.UtilityPLL.IsSourcePresent()                ||
//         NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE )        ||
//         !EEP_InputSyncEnabled() )
//    {
//        inputSyncGood.Debounce( false );
//    }
//    else
//    {
//        inputSyncGood.Debounce( true );
//    }
//
//    LocalSyncStatus.bit.BypassSyncAvail = bypassSyncGood.GetState() ? 1 : 0;
//    LocalSyncStatus.bit.InputSyncAvail = inputSyncGood.GetState() ? 1 : 0;
//    LocalSyncStatus.bit.CanBusFailed = NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR );
//
//
//    if ( EEP_ExtSyncEnabled() && !ParallelCan.UPSIsParallel() )
//    {
//        switch ( ExtSyncState )
//        {
//            case SYNC_SOURCE_BYPASS:
//                if ( !LocalSyncStatus.bit.BypassSyncAvail )
//                {
////                     if ( Rectifier.UtilityPLL.IsSourcePresent()             &&
////                          !NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE )   &&
////                          EEP_InputSyncEnabled() )
////                     {
////                         ExtSyncSource.SetSource( &RawAdcDataPtr->st.InputVoltage );
////                         ExtSyncState = SYNC_SOURCE_INPUT;
////                     }
////                     else
////                     {
//                        ExtSyncSource.SetSource( &BaseSineRef.ThreePhaseRef );
//                        ExtSyncState = SYNC_SOURCE_BASE;
////                     }
//                }
//                else
//                {
//                    // set base frequency to bypass frequency for smooth transition
//                    BaseSineRef.UpdateFrequencyAndAngle( BypassPLL.GetFrequency(), BypassPLL.SineRef.Angle );
//                }
//                break;
//
//            case SYNC_SOURCE_INPUT:
//            //
//            // not complete or tested since input sync disabled in 9E
//            //
//                if ( !Rectifier.UtilityPLL.IsSourcePresent()           ||
//                     NB_GetNodebit( UPM_NB_INPUT_SYNC_OUT_OF_RANGE )   ||
//                     !EEP_InputSyncEnabled() )
//                {
//                    ExtSyncSource.SetSource( &BaseSineRef.ThreePhaseRef );
//                    ExtSyncState = SYNC_SOURCE_BASE;
//                }
//                else
//                {
//                    if ( BypassPLL.IsSourcePresent() && !NB_GetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY ) )
//                    {
//                        ExtSyncSource.SetSource( &RawAdcDataPtr->st.BypassVoltage );
//                        ExtSyncState = SYNC_SOURCE_BYPASS;
//                    }
//                }
//                break;
//
//            case SYNC_SOURCE_BASE:
//            default:
//                if ( LocalSyncStatus.bit.BypassSyncAvail )
//                {
//                    float phaseError = BypassPLL.SineRef.Angle - BaseSineRef.Angle;
//					float frequencyError = BypassPLL.GetFrequency() - BaseSineRef.Frequency;
//
//                    if ( phaseError > PI )
//                    {
//                        phaseError -= ( 2.0f * PI );
//                    }
//                    else if ( phaseError < -PI )
//                    {
//                        phaseError += ( 2.0f * PI );
//                    }
//
//                    if ( fabs(frequencyError) < FreqErrorSyncLimit  &&
//                    	 fabs(phaseError) < PhaseErrorSyncLimit     &&
//                    	 Inverter.IsPhaseLocked() )
//                    {
//                    	// Jump to bypass sync
//                        ExtSyncSource.SetSource( &RawAdcDataPtr->st.BypassVoltage );
//                        BaseSineRef.AngleCorrection = 0.0f;
//                        ExtSyncState = SYNC_SOURCE_BYPASS;
//                    }
//                    else
//                    {
//			            // PLL to new phase offset
//			            float frequencyGain = 0.015;
//			            float phaseGain = 0.0025;
//
//						// Limit the slew rate induced by the frequency correction
//	            		float frequencyCorrection = clampf(-BaseSlewRate,
//	            			frequencyError * frequencyGain,
//	            			BaseSlewRate);
//
//	            		// TODO: Need to limit the slew rate induced by the phase correction, too
//			            float phaseCorrection = phaseError * phaseGain;
//
//	            		BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.Frequency + frequencyCorrection);
//	            		BaseSineRef.AngleCorrection = phaseCorrection;
//                    }
//                }
//                else
//                {
//                    // walk back to nominal frequency at limited slew rate
//                    if ( fabs( BaseSineRef.GetBaseFrequency() - BaseSineRef.Frequency ) < BaseSlewRate * 2 )
//                    {
//                        BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.GetBaseFrequency() );
//                    }
//                    else
//                    {
//                        if ( BaseSineRef.GetBaseFrequency() < BaseSineRef.Frequency )
//                        {
//                            BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.Frequency - BaseSlewRate );
//                        }
//                        else
//                        {
//                            BaseSineRef.UpdateFrequencyAndAngleStep( BaseSineRef.Frequency + BaseSlewRate );
//                        }
//                    }
//                }
//                break;
//        }
//    }
}
// ***********************************************************************
// *
// *    FUNCTION: InverterSyncedToBypass 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: returns true if inverter is phase locked to bypass, false if not
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool InverterSyncedToBypass( void )
{
    bool syncBypass = false;
    
    if ( Inverter.IsPhaseLocked() &&
         ( ( SYNC_STATE_BYPASS == Sync_State ) || 
           ( ( SYNC_STATE_BASE_EXT_SYNC == Sync_State ) && ( SYNC_SOURCE_BYPASS == ExtSyncState ) ) ) )
    {
        syncBypass = true;
    }
    
    return syncBypass;
}

// ***********************************************************************
// *
// *    FUNCTION: InverterSyncedToOutput 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: returns true if inverter is phase locked to output, false if not
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool InverterSyncedToOutput( void )
{
    bool syncOutput = false;
    
    if ( Inverter.IsPhaseLocked() && 
         ( ( SYNC_STATE_OUTPUT == Sync_State ) || 
           ( ( SYNC_STATE_BASE_EXT_SYNC == Sync_State ) && ( SYNC_SOURCE_OUTPUT == ExtSyncState ) ) ) )
    {
        syncOutput = true;
    }
    
    return syncOutput;
}

// ***********************************************************************
// *
// *    FUNCTION: SetParaSyncOut 
// *
// *    DESCRIPTION: Master UPM drives PARA_SYNC_OUT from sine ref
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void SetParaSyncOut( void )
{
//    if ( 0 == MyUPMNumber )
//    {
//        if ( ExtSyncSource.SineRef.Angle > 0 )
//        {
//            SetExtSyncLow();
//        }
//        else
//        {
//            SetExtSyncHigh();
//        }
//    }
}

// ***********************************************************************
// *
// *    FUNCTION: GetSyncStatus 
// *
// *    DESCRIPTION: returns sync status word
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
uint16_t GetSyncStatus( void )
{
    return LocalSyncStatus.words[ SYNC_WORD ];
}
// ********************************************************************
//          End of InvSync.cpp
// ********************************************************************
