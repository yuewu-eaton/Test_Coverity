// *****************************************************************************
// *            RectState.c
// *****************************************************************************
// *****************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// *****************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// *****************************************************************************
// *****************************************************************************
// *    FILE NAME: RectState.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *
// *    DATE: 31.10.2003
// *
// *    HISTORY: See CVS history
// *****************************************************************************


// *****************************************************************************
// *        INCLUDE FILES
// *****************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "RectifierStateControl.h"
#include "MCUState.h"
#include "Eeprom_Map.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "F28335Port.h"
#include "Meters.h"
#include "DQPhaseLockLoop.h"
#include "IOexpansion.h"
#include "BatteryStateControl.h"
#include "BTR.h"
#include "InverterControl.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
#include "Alarms_AC.h"
#include "ACMeter.h"
#include "ACPowerMeter.h"

using namespace std;

// *****************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// *****************************************************************************

// *****************************************************************************
// * GLOBAL VARIABLES
// *****************************************************************************
RectifierStateControl Rectifier;

// *****************************************************************************
// * LOCAL VARIABLES
// *****************************************************************************

float L1driveAngle = 0;
float LastUtilityAngle = 0;
//Alex Add for detecting the precharge terminal disconnect situation.
float PreDCLinkVoltagePos = 0;
float PreDCLinkVoltageNeg = 0;
//Alex add end.
const float MinPercentLoadForWalkin = 10.0;

inline void RectifierStateControl::PreChargeOff()
{
    PrechargeTimer.ClearTimer();
    ::PreChargeOff();
}
        
// *****************************************************************************
// *
// * Function: RectState(void);
// *
// * Purpose:  See description.
// *
// * Description:   Executes rectifier state.
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void RectifierStateControl::RunRectiferState( void )	//2.25k 444us
{
    if ( RectMachineState > RECTIFIER_INIT_STATE )
    {
        // check utility present, set/clear nodebit
        if ( UtilityPLL.IsSourcePresent() )
        {
            NB_DebounceAndQue( UPM_NB_UTILITY_NOT_PRESENT, false );
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_UTILITY_NOT_PRESENT, true, !UtilityPLL.IsSourcePresent() + !UtilityPLL.IsPhaseLocked() * 2 );
        }
        if( !NB_GetNodebit(UPM_NB_UTILITY_NOT_PRESENT) )
        {
            SetGLUtilityNotPresent(false);
        }
        //GL-139 add for GL only. -0.1 10% of inverter current, and BUS is higher than ref 15V or more
        if( ( MCUStateMachine.GetUtiLossDetEnable() && (UPMSystem == HV_30K) )      &&
            !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT )            &&
            (SYNC_STATE_BYPASS == Sync_State)                       &&
            ParallelCan.UPSIsParallel()                             &&
            ( ScreenMeters.OutputPower.sum < UtilityLossLoadPercentLimit ) &&
            ( Inverter.InverterCurrentDQ0Data.Sd < -0.1 )           &&
            ( RawAdcDataPtr->st.RailVoltagePositive > ( DCLinkReference / 2.0 + 15.0 ) ||
              ( ( -RawAdcDataPtr->st.RailVoltageNegative ) > ( DCLinkReference / 2.0 + 15.0 ) ) ) )
        {
			//for jira479, delete this func,as may mistake trip jira479 abnormal(simul Vinv_U2 sample fail).
			//Loadshare/inv loop not as 9e,not need this func,9e may also need recheck under this condi
//            NB_SetNodebit( UPM_NB_UTILITY_NOT_PRESENT, true,  2 );
//            NB_SetNodebit( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY, true, 2 );
//            SetGLUtilityNotPresent( true );             //Set the flag, used for choicing the inverter activePower. It will be clear after utility recover or 10 seconds later
//            Inverter.GLUtiLossTransientCounter = 0;
        }
    }            
    
    if (RectMachineState != RECTIFIER_SHUTDOWN_STATE)
    {
        // Increment this timer in every state but the shutdown state.  That
        // state will manage the timer explicitly as a prerequisite to starting
        // up the precharger.
        PrechargeTimer.CheckTimeout(RECT_DELAY_10_SEC);
    }
    
    // Increment the delayed failure counter
    DelayedFailureTimer.CheckTimeout( RECT_DELAY_FAILURE_TIME );
    
    // run the state machine
    switch ( RectMachineState )
    {
        case RECTIFIER_INIT_STATE:
            RectInitializingState();
            break;

        case RECTIFIER_SHUTDOWN_STATE:
            RectShutdownState();
            break;

        case RECTIFIER_PRECHARGE_STATE:
            RectPrechargingState();
            break;

        case RECTIFIER_NORMAL_STATE:
            RectOnNormalState();
            break;

        case RECTIFIER_TO_BATTERY_STATE:
            RectToBatteryState();
            break;

        case RECTIFIER_ON_BATTERY_STATE:
            RectOnBatteryState();
            break;

        case RECTIFIER_FROM_BATTERY_STATE:
            RectFromBatteryState();
            break;
            
        case RECTIFIER_WALKIN_STATE:
            RectWalkinState();
            break;    

        case RECTIFIER_SUSPEND_STATE:
            RectSuspendState();
            break;

        case RECTIFIER_FROM_SUSPEND_STATE:
            RectFromSuspendState();
            break;
                        
        default:
            RectInitializingState();
            break;
            
    }
    
    // clear any commands
    RectifierStatus.words[ RECTIFIER_COMMAND_WORD ] = 0;
}


// *****************************************************************************
// *
// * Function: RectTransferState(rectifier_states_t new_state);
// *
// * Purpose:
// *
// * Parms Passed   :   new_state
// * Returns        :   Nothing
// *
// * Description: Handles the tranfers between states
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void RectifierStateControl::RectTransferState( rectifier_states_t new_state )
{
    switch (new_state)
    {
        case RECTIFIER_PRECHARGE_STATE:
            RectifierStatus.bit.RectifierPrecharging = 1;
            break;
            
        case RECTIFIER_SHUTDOWN_STATE:
        case RECTIFIER_INIT_STATE:
        default:
            RectifierOff();						//Rec pwm pin off ->Bat_legB pwm also off
            Rectifier.BoostReuseRectST = 0;
            Rectifier.ReuseRectSTPWMEnable = 0;	//Bat_legB disable, so legB loop/pin must change to rec
            BoostLegBPWMOff();
			ConfigBoostLegBPWMOff();			//rec pwm need to 26k		
            NB_SetNodebit( UPM_NB_RECTIFIER_ON, false );
			Rectifier.RectifierOnReal = false;		//JIRA:WOMBAT-164			
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, true );
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, true );
            break;
            
        case RECTIFIER_TO_BATTERY_STATE:
            NB_SetNodebit( UPM_NB_RECTIFIER_ON, true ); //DC Start, rectifier will come from initial to tobattery			
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, true );
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, true );
            break;
            
        case RECTIFIER_FROM_BATTERY_STATE:
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, false );
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, false );
            break;
        
        case RECTIFIER_ON_BATTERY_STATE:
            if( NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) )
            {
                RectifierStatus.bit.SiteWiringLocked = 1;
            }
            break;
        case RECTIFIER_WALKIN_STATE:
            break;
            
        case RECTIFIER_NORMAL_STATE:
            RectifierStatus.bit.ACPreChargeFail = 0;
            NB_SetNodebit( UPM_NB_RECTIFIER_ON, true );
			Rectifier.RectifierOnReal = true;			
            break;
            
        case RECTIFIER_SUSPEND_STATE:
            RectifierOff();
            NB_SetNodebit( UPM_NB_RECTIFIER_ON, false );
			Rectifier.RectifierOnReal = false;
            break;

        case RECTIFIER_FROM_SUSPEND_STATE:
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, true );
            NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, true );
            NB_SetNodebit( UPM_NB_RECTIFIER_ON, true );
			Rectifier.RectifierOnReal = true;
            break;
    }
    RectifierTimer1.ClearTimer();
    RectifierTimer2.ClearTimer();
    RectifierPhState = 0;

    RectMachineState = new_state;
    NB_LogStateChange(UPM_NB_RECTIFIER_STATE_CHANGED, RectMachineState);
}

// *****************************************************************************
// *
// * Function: RectInitializingState(void);
// *
// * Purpose:
// *
// * Description:   Intializes State machine
// *
// *****************************************************************************
void RectifierStateControl::RectInitializingState(void)
{
    if ( MCUStateMachine.GetADCReadyFlag() )
    {
        // not much else to do at this point
        RectTransferState( RECTIFIER_SHUTDOWN_STATE );    
    }
    NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, true );
    NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, true );	
	BatteryConverter.SetUpmsToBatTogether(false);	
} // End of RectInitializingState()


// *****************************************************************************
// *
// * Function: RectShutdownState(void);
// *
// * Purpose:
// *
// * Description:   Everything off.  Waiting for command.
// *
// *****************************************************************************
void RectifierStateControl::RectShutdownState(void)
{	
	BatteryConverter.SetUpmsToBatTogether(false);		

    switch ( RectifierPhState )
    {
        case 0:
            if ( RectifierTimer1.CheckTimeout( RECT_DELAY_200MS ) )
            {
                OpenInputRelays();
                RectifierPhState++;

				//for hobbit-113, bat mode Bat_legB off, open batline rly						
				DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
				NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113
				//end				
            }
            break;
        
        case 1:
            if ( RectifierStatus.bit.StartupCommand &&
                 !BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging )
            {
                if ( !CheckRectifierFailure() &&
                     !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE ) &&
                     !NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE ) &&
                     !NB_GetNodebit( UPM_NB_RECTIFIER_PHASE_ROTATION )&&
                     PrechargeTimer.CheckTimeout(RECT_DELAY_10_SEC) )
                {
                    RectifierStatus.bit.StartupCommand = 0;
                    RectTransferState( RECTIFIER_PRECHARGE_STATE );
                }
			   }
				//Battery start,wait DC and battery capacity to be precharged, and then to battery state.
				else if ( BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging &&
						  !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) &&
						  !BatteryConverter.CheckBatteryShutdown() )
				{
					RectTransferState( RECTIFIER_TO_BATTERY_STATE );
				}
            else
            {
                PrechargeTimer.CheckTimeout(RECT_DELAY_10_SEC);
            }
            break;
            
        default:
            // Impossible.
            RectTransferState( RECTIFIER_SHUTDOWN_STATE );
            break;
    }
} // End of RectShutdownState()


// *****************************************************************************
// *
// * Function: RectPrechargingState(void);
// *
// * Purpose:
// *
// * Description:   Waits for the DC link to precharge
// *
// *****************************************************************************
void RectifierStateControl::RectPrechargingState(void)
{
    const float DCLinkRampupCompleteVoltage = 5.0;
    
    float thisAngle;
    float prechargelim;
    float dclinkerror;
	static uint16_t wPrechargeCnt = 0;
    
	BatteryConverter.SetUpmsToBatTogether(false);	
    prechargelim = float( ACPreChargeRailLimit );
	
    if ( CheckRectifierFailure()                                ||
         NB_GetNodebit( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT )   ||
         NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE )          ||
         RectifierStatus.bit.OffCommand )
    {
        // abort precharge
        PreChargeOff();
        RectifierStatus.bit.RectifierPrecharging = 0;
        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
    }
    else
    {    
        switch ( RectifierPhState )
        {
            case 0:
                // set relays, should be set here already
                OpenInputRelays();
                RectifierPhState++;
                break;
                
            case 1:
                thisAngle = UtilityPLL.SineRef.Angle;
                if (RectifierTimer1.CheckTimeout( RECT_DELAY_100MS) )
                {
                	// Record DClink voltage
                	PreDCLinkVoltagePos = DCLinkVoltagePositive.FastFiltered;
                	PreDCLinkVoltageNeg = DCLinkVoltageNegative.FastFiltered;
                    // Allow the neutral relay to close before engaging the
                    // precharge.  This provides safety to the relay by ensuring
                    // that it is not completing a circuit.
                    if ( ( thisAngle > 0 ) && ( LastUtilityAngle < 0 ) )
                    {                   
                        PreChargeOn();
                        RectifierPhState++;
                    }
                }
                LastUtilityAngle = thisAngle;
                break;
                
            case 2:
                if ( RectifierTimer1.CheckTimeout( RECT_DELAY_1000MS ) )
                {
                	//Alex Add for detecting the precharge terminal disconnect situation.
                	float PreChargeFailedHighLimit = UtilityPLL.SourceDQOData.Sd * 0.8;
                    float PreChargeFailedLimit = 10.0;
                    
//	                    // check that rail is at least charging
//	                    if ( ( DCLinkVoltagePositive.FastFiltered < PreChargeFailedLimit )                          ||
//	                         ( DCLinkVoltageNegative.FastFiltered > -PreChargeFailedLimit )                         ||
//	                         ( DCLinkVoltagePositive.FastFiltered - PreDCLinkVoltagePos > PreChargeFailedHighLimit) ||
//	                         ( DCLinkVoltageNegative.FastFiltered - PreDCLinkVoltageNeg < -PreChargeFailedHighLimit) )
//	                    {
//	                        PreChargeOff();
//	                        RectifierStatus.bit.RectifierPrecharging = 0;
//	                        NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 2 );
//	                        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
//	                    }
//	                    else
//	                    {
//	                        RectifierTimer1.ClearTimer();
//	                        RectifierPhState++;
//	                    }     

					// check that rail is at least charging
					if ( ( DCLinkVoltagePositive.FastFiltered < PreChargeFailedLimit )							||
						 ( DCLinkVoltageNegative.FastFiltered > -PreChargeFailedLimit ) )					
					{
						if ( wPrechargeCnt++ > 20)
						{
							wPrechargeCnt = 0;
							PreChargeOff();
							RectifierStatus.bit.RectifierPrecharging = 0;
							NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 2 );
							RectTransferState( RECTIFIER_SHUTDOWN_STATE );
						}
					}
					else if(RectifierTimer1.CheckTimeout( RECT_DELAY_2000MS ))
					{
					
						wPrechargeCnt = 0 ;
						RectifierTimer1.ClearTimer();
						RectifierPhState++;
						// for hobbit-113
					}	  
                }
                break;
                
            case 3:
                float RphaseInputVoltPeak;
                float SphaseInputVoltPeak;
		        RphaseInputVoltPeak = ScreenMeters.InputVoltageRMS.phA * SQRT_2;
		        SphaseInputVoltPeak = ScreenMeters.InputVoltageRMS.phB * SQRT_2;
	            if ( RectifierTimer1.CheckTimeout( (uint32_t)PrechargeTime * RECT_DELAY_1000MS) )  //RECT_DELAY_19_SEC
                {
                    // if we're here, the link is not charging correctly
                    // Requirement: Do not allow the precharger to run for greater
                    // than 30 seconds continuously.  Allocate one second for
                    // the initial operational check, plus 10 seconds for the
                    // final top-off leaves 19 to reach the precharge limit.
                    PreChargeOff();
                    RectifierStatus.bit.RectifierPrecharging = 0;

					if( ++PrechargeAttempts >= 3)
					{
						// if there is only utility, active precharge fail
						if( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED) ||
							!NB_GetNodebit( UPM_NB_BATTERY_INSTALLED ) )
						{
							NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 3 );
						}
						// otherwise, transfer to battery start
						else
						{
							RectifierStatus.bit.ACPreChargeFail = 1;
						}
					}
                    RectTransferState( RECTIFIER_SHUTDOWN_STATE );
                }
                else
                {
                    if ( PrechargeType == RESISTOR )
		            {
		                if ( ( DCLinkVoltagePositive.FastFiltered > ( RphaseInputVoltPeak - prechargelim ) ) &&
		                     ( DCLinkVoltageNegative.FastFiltered < ( prechargelim - SphaseInputVoltPeak ) ) )
                        {
						    RectifierTimer1.ClearTimer();
				            RectifierPhState++;
			            }						
		            }
		            else
		            {
		    	        if ( ( DCLinkVoltagePositive.FastFiltered > ( UtilityPLL.SourceDQOData.Sd - prechargelim ) ) &&
                              ( DCLinkVoltageNegative.FastFiltered < ( prechargelim - UtilityPLL.SourceDQOData.Sd ) ) ) 
                        {
                        	RectifierTimer1.ClearTimer();
                        	RectifierPhState++;
                    	}
		            }
                }
                break;
                
            case 4:
		        if ( ( PrechargeType == RESISTOR ) ||
		             ( RectifierTimer1.CheckTimeout( RECT_DELAY_19_SEC ) ||
                       ( ( DCLinkVoltagePositive.FastFiltered > UtilityPLL.SourceDQOData.Sd ) &&
                         ( DCLinkVoltageNegative.FastFiltered < (- UtilityPLL.SourceDQOData.Sd ) ) ) ) )
                {
                    // let go 10 more seconds, then shut precharge off
                    PreChargeOff();
                    // calculate phase angle to drive the L1 input relay
                    // EERelL1TransferTime in us.
                    // angle in radians = frequency in Hz * 2*pi * 1/1,000,000 * transfer time in us
                    L1driveAngle = UtilityPLL.GetFrequency() * ( 2.0 * PI );
                    L1driveAngle *= 0.000001;
                    L1driveAngle *= (float)EERelL1TransferTime;
                    // we want to close when angle = 0, so negate
                    L1driveAngle = -L1driveAngle;
                    if ( L1driveAngle < -PI )
                    {
                        L1driveAngle += ( 2.0 * PI );
                    }
                    // save the current angle
                    LastUtilityAngle = UtilityPLL.SineRef.Angle;
                    RectifierTimer1.ClearTimer();   
                    RectifierPhState++;
                }            
                break;
            
            case 5:
                // here's where the silly part starts, we need to 'sync up' with the utility frequency, so the next
                // phase state change needs to happen around the zero cross
                thisAngle = UtilityPLL.SineRef.Angle;
                if ( ( thisAngle > 0 ) && ( LastUtilityAngle < 0 ) )
                {
                    RectifierPhState++;
                }
                LastUtilityAngle = thisAngle;
                break;
                
            case 6:
                // OK, now we know we just crossed zero and phase is going up. If the closing time is less than 1/2 the
                // utility period, then our target angle is negative. If it's longer than half the period, then it's positive
                // So, if drive angle is positive AND utility angle is greater than the drive angle OR negative, it's time to fire
                // But if drive angle is negative AND utility angle is greater than the drive angle AND negative, it's time to fire
                // Otherwise we wait. 
                thisAngle = UtilityPLL.SineRef.Angle;
                if ( L1driveAngle > 0 )
                {
                    if ( ( thisAngle > L1driveAngle ) || ( thisAngle < 0 ) )
                    {
                        DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
//	                        NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, false );
						NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, false );	//for hobbit-113
                        RectifierPhState++;
						//for hobbit-113, before on Rec_ST, must open BatLine_Rly(ST reuse bat_legB)						
						DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
						NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113
						//end
                    }
                }
                else
                {
                    if ( ( thisAngle > L1driveAngle ) && ( thisAngle < 0 ) )
                    {
                        DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
//	                        NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, false );
						NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R, false );
                        RectifierPhState++;
						//for hobbit-113, before on Rec_ST, must open BatLine_Rly(ST reuse Bat_legB)						
						DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
						NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113
						//end
                    }
                }
                break;

            case 7:    
			    if ( PrechargeType == RESISTOR )
			    {
			    	if ( RectifierTimer1.CheckTimeout( RECT_DELAY_500MS ) )
			        {
//				    		RectifierL1PWMOn();
//				            RectifierStatus.bit.RectifierOnNormal = 1;
						// close the other relay
						//DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
						//turn on rec to charge input cap before we close input relay L23
						//so that the voltage difference between input relay L23 can be zero.
						//input inrush current can be elimilated
						if( !RectifierStatus.bit.RectifierOnNormal )
						{
							OnNormal();
						}
			            RectifierTimer1.ClearTimer();
			            RectifierPhState++;
					}
			    }
				else
				{
					RectifierPhState++;
				}	
				break;	

            case 8:
			    if( PrechargeType == RESISTOR )
				{				
	            	float STphaseInputVoltPeak;
	                STphaseInputVoltPeak = ( ScreenMeters.InputVoltageRMS.phC > ScreenMeters.InputVoltageRMS.phB ) ? ScreenMeters.InputVoltageRMS.phC : ScreenMeters.InputVoltageRMS.phB;
	                STphaseInputVoltPeak *= SQRT_2;
					
					if ( RectifierTimer1.CheckTimeout( RECT_DELAY_10_SEC ) )   //check max time 
                    {
					    PreChargeOff();
						RectifierStatus.bit.RectifierPrecharging = 0;
						NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 8 );
						RectTransferState( RECTIFIER_SHUTDOWN_STATE );
					}
					else
					{
                        if ( ( DCLinkVoltagePositive.FastFiltered >  STphaseInputVoltPeak ) &&
							 ( DCLinkVoltageNegative.FastFiltered < (  - STphaseInputVoltPeak ) ) )
						{
							RectifierPWMOff();
							RectifierStatus.bit.RectifierOnNormal = 0;

							// close the other relay
							DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
							NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, false );	//for hobbit-113						
							SetRectHWCurrentLimit();  // HV system to set different Rectifier HWCL when on recitifer and on battery
							RectifierTimer1.ClearTimer();
							RectifierPhState++;
						}
					}
                }
                else
				{
					if ( RectifierTimer1.CheckTimeout( RECT_DELAY_500MS ) )
	                {
					    // close the other relay
						DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
						NB_SetNodebit( UPM_NB_RECTIFIER_SWITCHGEAR_OPEN, false );
						SetRectHWCurrentLimit();  // HV system to set different Rectifier HWCL when on recitifer and on battery
						RectifierTimer1.ClearTimer();
						RectifierPhState++;
					}
				}               
	
                break;  
                
            case 9:
                if ( RectifierTimer1.CheckTimeout( RECT_DELAY_1000MS ) )//possible avoid IGBT burn because of site 15V fail alarm(3c3pro)
                {
					OnNormal();//fix 3c3pro jira NEW3C3-258 start up online blow up igbt
                    RectifierTimer1.ClearTimer();
                    RectifierPhState++;
                }
                break;
                
            case 10:
                dclinkerror = DCLinkReference - ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered );
                if ( dclinkerror < DCLinkRampupCompleteVoltage )
                {
					PrechargeAttempts = 0;
                    NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, false );
                    RectifierStatus.bit.RectifierPrecharging = 0;
                    RectTransferState( RECTIFIER_NORMAL_STATE );    
                }
                else
                {
                    if ( RectifierTimer1.CheckTimeout( RECT_DELAY_2000MS ) )
                    {
                        RectifierStatus.bit.RectifierPrecharging = 0;
                        NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 10 );
                        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
                    } 
                }
                break;    
                
            default:
                PreChargeOff();
                RectifierStatus.bit.RectifierPrecharging = 0;
                RectTransferState( RECTIFIER_SHUTDOWN_STATE );
                break;
                                             
        }
    }    
} // End of RectPrechargingState()


// *****************************************************************************
// *
// * Function: RectOnNormalState(void);
// *
// * Purpose:
// *
// * Description:   Rectifier running.
// *
// *****************************************************************************
inline void RectifierStateControl::RectOnNormalState(void)
{
    // Check for transfers out of normal, in priority order
    if ( CheckRectifierFailure()                 ||
         !RectifierStatus.bit.RectifierOnNormal )
    {
        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
    }
//	    else if ( !InputAvailable() ||
//					((ParallelCan.ParGlobalOrData.BatteryStatus.bit.UpmsToBatTogether)&&(NumOfUPMs>=2)))
	else if ( !InputAvailable() )
    {
        if ( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
        {
            // transfer to battery if able to
            RectTransferState( RECTIFIER_TO_BATTERY_STATE );
			Rectifier.RectifierOnReal = false;	
			BatteryConverter.SetUpmsToBatTogether(true);	//1.request other upms turn to bat mode, together
        }
        else
        {
            // Otherwise shutdown
            RectTransferState( RECTIFIER_SHUTDOWN_STATE );
        }
    }
    else if ( RectifierStatus.bit.GoToSuspendState )
    {
        RectTransferState( RECTIFIER_SUSPEND_STATE );
    }
    else if (RectifierStatus.bit.BatteryNormalCycleTest &&
        RectifierTimer1.CheckTimeout(RECT_DELAY_19_SEC))
    {
        RectTransferState(RECTIFIER_TO_BATTERY_STATE);
    }
    else if( FuseFailRestart && !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
    {
		FuseFailRestart = false;
		FuseFailRestartTimes++;

		RectTransferState( RECTIFIER_TO_BATTERY_STATE );
    }
} // End of RectOnNormalState()


// *****************************************************************************
// *
// * Function: RectToBatteryState(void);
// *
// * Purpose:
// *
// * Description: Moving from rectifier on normal state to battery state
// *
// *****************************************************************************
inline void RectifierStateControl::RectToBatteryState(void)
{
    switch ( RectifierPhState )
    {
        case 0:
            // rectifier turn off, open relays
            //open all input relay except neutral relay Yanming 20110331
            RectifierOff(); //disable ST PWM
            if( RectifierStatus.bit.BatteryECT )
            {
                DSPOutRegister.GpoB.bit.InputRelay_L1 = 0;
                DSPOutRegister.GpoB.bit.InputRelay_L23 = 0;
            }
            else
            {
                OpenInputRelays();
            }            

            //turn on batt legA first
            BatteryConverter.BoostCmdOn();
            OnBattery();
            RectifierTimer1.ClearTimer();
            BatteryTransferCounter++; // for "Too Many Transfers" logic
            RectifierPhState++;    
            break;
            
        case 1:
            // wait input ST relay to open
			if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Open ) )
			{
				if ( ( RawAdcDataPtr->st.RailVoltagePositive > (RawAdcDataPtr->st.BatteryVoltageChgPos -20.0f) ) &&
					 ( RawAdcDataPtr->st.RailVoltageNegative < (RawAdcDataPtr->st.BatteryVoltageChgNeg +20.0f) )	)
				{
					DSPOutRegister.GpoB.bit.BattLegB_Relay = 1; //LegB, reuse rectifier phase S/T
					NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, false );	//for hobbit-113					
					RectifierTimer1.ClearTimer();
					RectifierPhState++;
				}
			}
            break;
            
        case 2:
            // Start legB after BattLine relay closes
            if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Close ) )               
            {
            	ConfigBoostLegBPWMOn();
            	Rectifier.BoostReuseRectST = 1;
            	Rectifier.ReuseRectSTPWMEnable = 1;
                BoostLegBPWMOn(); //can re-enable ST PWM
                RectTransferState( RECTIFIER_ON_BATTERY_STATE );
            }
            break;
            
        default:
            RectTransferState( RECTIFIER_SHUTDOWN_STATE );
            break;
    }

} // End of RectToBatteryState()

// *****************************************************************************
// *
// * Function: RectOnBatteryState(void);
// *
// * Purpose: Use balancer only
// *
// * Description: On battery state.
// *
// *****************************************************************************
inline void RectifierStateControl::RectOnBatteryState(void)
{
    if ( RectifierTimer1.CheckTimeout( RECT_DELAY_3000MS ) )
    {
        if ( InputAvailable() &&
             !RectifierInhibitted &&
             //Rectifier back online must neutral loss is clear and DC voltage loss 380V(360 is ref)
             !NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) &&
             ( DCLinkVoltagePositive.FastFiltered < DCLinkUnbalanceOVLimSetting ) &&
             ( -DCLinkVoltageNegative.FastFiltered < DCLinkUnbalanceOVLimSetting ) )
        {			
			BatteryConverter.SetUpmsToBatTogether(false);	
//				if(ParallelCan.ParGlobalOrData.BatteryStatus.bit.UpmsToBatTogether == false)	//2.Wait all upms ready, then to line mode
//				if((NumOfUPMs>=2))	//80k
//				{
//					if(ParallelCan.ParGlobalOrData.BatteryStatus.bit.UpmsToBatTogether == false)	//2.Wait all upms ready, then to line mode
//					{
//			            RectTransferState( RECTIFIER_FROM_BATTERY_STATE );
//						RectifierOnReal = true;			
//					}
//				}
//				else
			{
	            RectTransferState( RECTIFIER_FROM_BATTERY_STATE );
				RectifierOnReal = true;			
			}
        }
    }
    
    if ( CheckRectifierFailure() ||
         !RectifierStatus.bit.RectifierOnBattery ||
         NB_GetNodebit(UPM_NB_BATTERIES_DISCONNECTED))
    {
        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
    }         

	if(!InputAvailable())		//add for jira wombat-318, to show real RecOn
	{
		RectifierOnReal = false;
	}	
} // End of RectOnBatteryState()

// *****************************************************************************
// *
// * Function: RectFromBatteryState(void);
// *
// * Purpose:
// *
// * Description: Moving from battery state to rectifier on normal state
// *
// *****************************************************************************
inline void RectifierStateControl::RectFromBatteryState(void)
{
	float RphInputCurrMaxTemp;

    // go back to battery is any of these become active
    if ( !InputAvailable() )
//		if ( !InputAvailable() ||
//				((ParallelCan.ParGlobalOrData.BatteryStatus.bit.UpmsToBatTogether)&&(NumOfUPMs>=2)))	//only 80k
    {
		Rectifier.RectifierOnReal = false;	
		BatteryConverter.SetUpmsToBatTogether(true);	//1.request other upms turn to bat mode, together
        RectTransferState( RECTIFIER_TO_BATTERY_STATE );
    }             
    else
    { 
        switch ( RectifierPhState )
        {
            case 0:                 // walkin delay
                if( RectifierWalkInDelay == 0 )
                {
                    RectifierPhState++;
                }
                else if( RectifierTimer1.CheckTimeout( RectifierWalkInDelay * RECT_DELAY_1000MS) )
                {
                    RectifierTimer1.ClearTimer();
                    RectifierPhState++;
                }
                break;
            case 1:
				DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;	//close R rly first				
                BoostLegBPWMOff();  //disable ST PWM
				Rectifier.ReuseRectSTPWMEnable = 0;
				ConfigBoostLegBPWMOff();
				RectifierTimer1.ClearTimer();
				RectifierPhState++;
            	break;
            case 2:                 //open battline relay
                //need low current condition, same as battery relay?
//					if( UtilityCurrentRMS.RawRMS.phB < 10.0f && UtilityCurrentRMS.RawRMS.phC < 10.0f )
//					{
//						DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
//						NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113										
//						RectifierTimer1.ClearTimer();
//						RectifierPhState++;
//					}
            	if(RectifierTimer1.CheckTimeout(RECT_DELAY_4MS))
            	{
					if( ((RawAdcDataPtr->st.InputCurrent.phB) < 10.0f) && ((-RawAdcDataPtr->st.InputCurrent.phC) < 10.0f ))
					{
						DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
						NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113										
						RectifierTimer1.ClearTimer();
						RectifierPhState++;
					}					
            	}
				break;
            case 3:                 //wait for Battline relay to open, close input relay.                            	   
            	if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Open ) )
            	{
					// close input ST relays
					DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
            		DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
					RectifierTimer1.ClearTimer();
					RectifierPhState++;
            	}
                break;
            case 4:                 // wait for input relays to close, walkin
                if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Close ) )
                {
                	Rectifier.BoostReuseRectST = 0;
                    if ( !CheckRectifierFailure() )
                    {
                        if( ( ScreenMeters.PercentLoad.sum < MinPercentLoadForWalkin ) ||
							( RectifierStatus.bit.ACPreChargeFail == TRUE) || NB_GetNodebit(UPM_NB_UPS_ON_BYPASS) )
                        {
                            OnNormal(); 
                            RectTransferState( RECTIFIER_NORMAL_STATE );
                        }
                        else
                        {
                            RphInputCurrMaxTemp = InverterPower.ActivePowerFiltered.sum*1.20f / (UtilityVoltageRMS.RawRMS.phA + UtilityVoltageRMS.RawRMS.phB + UtilityVoltageRMS.RawRMS.phC);
                            if (OutNomVolts != 0)
                            {
                                RphInputCurrMaxTemp = (RphInputCurrMaxTemp * 10.0) / (float)OutNomVolts;
                            }
                            else
                            {
                                RphInputCurrMaxTemp = RphInputCurrMaxTemp / 230.0;
                            }
                            ScaledIrefMax = RphInputCurrMaxTemp * ((float)WalkinStartPercentLoad / 100.0);						
                            OnNormal( true );
                            RectTransferState( RECTIFIER_WALKIN_STATE );
                        }    
                    }
                    else
                    {
                        RectTransferState( RECTIFIER_SHUTDOWN_STATE );
                    }             
                }
                break;                
                
            default:
                RectTransferState( RECTIFIER_SHUTDOWN_STATE );
                break;
        }
    }    
} // End of RectFromBatteryState()

// *****************************************************************************
// *
// * Function: RectPowerShare(void);
// *
// * Purpose:
// *
// * Description: powershare with the booster
// *
// *****************************************************************************
inline void RectifierStateControl::RectWalkinState( void )
{
    uBatteryConverterStatus bStat = BatteryConverter.GetStatus();	
	BatteryConverter.SetUpmsToBatTogether(false);	
    
    // check if need to go back to battery
    if ( !InputAvailable() )
    {
        RectTransferState( RECTIFIER_TO_BATTERY_STATE );
    }
    else
    {          
        if ( CheckRectifierFailure() )
        {
            RectTransferState( RECTIFIER_SHUTDOWN_STATE );
        }
        else
        {
            // check for time to end walk-in
//	            if ( ( ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered ) > DCLinkReference ) ||
            // HOBBIT-154... hw hobbit adc sample disturb issue not fix(Vbus_sample same big jam cause mistake end walkin, change use slowfiler
            if ( ( (DCLinkVoltagePositive.SlowFiltered - DCLinkVoltageNegative.SlowFiltered) > DCLinkReference ) ||
                 ( ScaledIrefMax >= RectifierInputCurrentMax )                                                   ||
                 !bStat.bit.BoostOn )
            {
                ScaledIrefMax = RectifierInputCurrentMax; 
                RectifierStatus.bit.WalkIn = 0;
                RectTransferState( RECTIFIER_NORMAL_STATE );
            }
            else
            {
                ScaledIrefMax += RectifierWalkInRate;
            }
        }
    }                 
}

// *****************************************************************************
// *
// * Function: RectSuspendState(void);
// *
// * Purpose:
// *
// * Description: rectifier off and input relays closed
// *
// *****************************************************************************
inline void RectifierStateControl::RectSuspendState( void )
{
    RectifierStatus.bit.GoToSuspendState = false;	
	BatteryConverter.SetUpmsToBatTogether(false);	

    switch ( RectifierPhState )
    {
        case 0:
            // open input relays.
        	OpenInputRelays();
            RectifierTimer1.ClearTimer();
            RectifierPhState++;
            break;
            
        case 1:
            // wait for input relays to open
            if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Open ) )
            {
                // close the bat legB(reuse Rec_ST) relay
            	DSPOutRegister.GpoB.bit.BattLegB_Relay = 1;
				NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, false );	//for hobbit-113					
                RectifierTimer1.ClearTimer();
                RectifierPhState++;
            }
            break;
            
        case 2:
                // Delay for closing the balancer relay
            if ( RectifierTimer1.CheckTimeout( RecBatInv_Delay_Relay_Close ) )	//no balancer in Hobbit
            {
                RectifierTimer1.ClearTimer();
                RectifierPhState++;
            }
            break;
        case 3:
                // transition to eco is done.  dont do anything until time to leave.
            break;
        default:
            RectTransferState( RECTIFIER_FROM_SUSPEND_STATE );
            break;
    }

    if( !MCUStateMachine.GetStatus().bit.InverterSuspend )
    {      
        if ( RectifierPhState > 2 )
        {
            OnBattery();
            RectTransferState( RECTIFIER_FROM_SUSPEND_STATE );
        }
    }
       
}

// *****************************************************************************
// *
// * Function: RectFromSuspendState(void);
// *
// * Purpose:
// *
// * Description: turn boost on then go to battery state (open input relays)
// *
// *****************************************************************************
inline void RectifierStateControl::RectFromSuspendState( void )
{
    RectifierStatus.bit.GoToSuspendState = false;	
	BatteryConverter.SetUpmsToBatTogether(false);	

    switch ( RectifierPhState )
    {
        case 0:
//				//For jira WOMBAT-209: add for Ess to bat mode, on boost not fast enough (9P exist this)
//				if(Inverter.GetStatus().bit.InverterOn)
//				{
//					if(BatteryConverter.GetBatteryState() != BATTERY_ON_BOOST_STATE )
//					{
//						BatteryConverter.BoostCmdOn();				
//					}
//	
//		        	ConfigBoostLegBPWMOn();
//					Rectifier.BoostReuseRectST = 1;
//					Rectifier.ReuseRectSTPWMEnable = 1;
//	            	BoostLegBPWMOn();
//	                RectifierTimer1.ClearTimer();
//	                RectifierPhState++;
//	            }

			//For jira WOMBAT-209: add for Ess to bat mode, on boost not fast enough (9P exist this)
			if(Inverter.GetStatus().bit.InverterOn)
			{
				if(BatteryConverter.GetBatteryState() != BATTERY_ON_BOOST_STATE )
				{
					BatteryConverter.BoostCmdOn();				
				}
	
				ConfigBoostLegBPWMOn();
				Rectifier.BoostReuseRectST = 1;
				Rectifier.ReuseRectSTPWMEnable = 1;
				BoostLegBPWMOn();
			}

            //Wait a couple of cycles before going to battery state
            // this delays disconnecting the neutral and B,C relays until load is on inv.
            if ( RectifierTimer1.CheckTimeout( RECT_DELAY_500MS ) )
            {
//					// open input relays.
//					OpenInputRelays();		//Suspend have open rly_in				
				RectifierTimer1.ClearTimer();
				RectifierPhState++;

//					ConfigBoostLegBPWMOn();
//					Rectifier.BoostReuseRectST = 1;
//					Rectifier.ReuseRectSTPWMEnable = 1;
//					BoostLegBPWMOn();				
            }
            break;
        case 1:
            if ( RectifierTimer1.CheckTimeout( RECT_DELAY_1000MS ) )
            {
				ConfigBoostLegBPWMOn();
				Rectifier.BoostReuseRectST = 1;
				Rectifier.ReuseRectSTPWMEnable = 1;
				BoostLegBPWMOn();					
                RectTransferState( RECTIFIER_ON_BATTERY_STATE );
            }
           break;
           
        default:
            RectTransferState( RECTIFIER_TO_BATTERY_STATE );
            break;
    }

}

// *****************************************************************************
// *
// * Function: bool CheckRectifierFailure(void);
// *
// * Purpose: Check all reasons to shut down rectifier IGBTs or prevent it from
// *          starting
// *
// * Parms Passed   :   none
// * Returns        :   TRUE/FALSE
// *
// * Description: Returns TRUE if there is a failure that disables rectifier
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool RectifierStateControl::CheckRectifierFailure( void )
{
    bool failed =  false;
    
    if ( CheckDelayedRectifierFailure()                             ||
         NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF )         ||
         NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE )               ||
         NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )              ||
         NB_GetNodebit( UPM_NB_NON_VOLATILE_RAM_FAILURE )           ||
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT )          ||
         NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )         ||
         NB_GetNodebit( UPM_NB_DRIVER_FAULT )                       ||
         NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE) )
    {
        failed = true;
    }
    
    return failed;         
}


// *****************************************************************************
// *
// * Function: bool CheckDelayedRectifierFailure(void);
// *
// * Purpose: Check all reasons to shut down rectifier IGBTs or prevent it from
// *          starting
// *
// * Parms Passed   :   none
// * Returns        :   TRUE/FALSE
// *
// * Description: Returns TRUE if there is a failure that disables rectifier
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool RectifierStateControl::CheckDelayedRectifierFailure( void )
{
    if ( !NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP )  &&
         !NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE_TRIP ))
    {
        DelayedFailureTimer.ClearTimer();
    }
    
    return ( DelayedFailureTimer.TimerValue() >= RECT_DELAY_FAILURE_TIME );
}


// *****************************************************************************
// *
// * Function: bool InputAvailable(void);
// *
// * Purpose: Check all reasons to consider the input unfit for use as a power
// *          source.
// *
// * Parms Passed   :   none
// * Returns        :   TRUE/FALSE
// *
// * Description: Returns TRUE if the input is usable.
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool RectifierStateControl::InputAvailable( void )
{
    stBTRStatus BTRStatus = BTR.GetBTRStatus();
        
    return !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT )         &&
        !NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE )          &&
        !NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE)          &&
        !NB_GetNodebit( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT )   &&
        !NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY )     &&

        !( NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT ) &&
           ( ( DCLinkVoltagePositive.FastFiltered > DCLinkUnbalanceOVLimSetting ) ||
             ( -DCLinkVoltageNegative.FastFiltered > DCLinkUnbalanceOVLimSetting )  ) ) &&//transfer to battery when site wiring fault
		!( InstantVoltageState                         &&
		  UnitConfig.bit.EnableSurgeToBattery		   &&		
		  ( ( DCLinkVoltagePositive.FastFiltered > DCLinkUnbalanceOVLimSetting ) ||
             ( -DCLinkVoltageNegative.FastFiltered > DCLinkUnbalanceOVLimSetting ) ) )&&
        !RectifierStatus.bit.BatteryECT                         &&
        !BTRStatus.BatteryQuickTest;
}

// ***********************************************************************
// *
// *    FUNCTION: Rectifier100msTask
// *
// *    DESCRIPTION: Update "Too many transfers" logic
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none
// *
// ***********************************************************************
void RectifierStateControl::Rectifier100msTask( void )
{
	if ( DisableTooManyBatTrans )
	{
		BatteryTransferCounter = 0;
		RectifierInhibitted = 0;
	}
    if( RectifierInhibitted )RectifierInhibitted--;
    NB_SetNodebit( UPM_NB_TOO_MANY_BATTERY_TRANSFERS, RectifierInhibitted );

    if( BatteryTransferCounter > 0 )
    {
        if( BatteryTransferCounter >= RECTIFIER_INHIBIT_TRANSFER_LIMIT )
        {
            NB_SetNodebit( UPM_NB_TOO_MANY_BATTERY_TRANSFERS, true );
            // Don't clear the counter, only cap it; we want to keep slowing by the inhibit time if it keeps transferring
            BatteryTransferCounter = RECTIFIER_INHIBIT_TRANSFER_LIMIT;
            RectifierInhibitted = RECTIFIER_INHIBIT_TIME;
        }

        if( RectifierInhibitTimer.CheckTimeout( RECTIFIER_INHIBIT_DECREMENT_TIMEOUT ) )
        {
            RectifierInhibitTimer.ClearTimer();
            BatteryTransferCounter--;
        }
    }
    else
    {
        RectifierInhibitTimer.ClearTimer();
    }
}


// ********************************************************************************************************
// *
// * Function:    void ResetRectifierAlarms( void )
// *
// * Purpose:     Reset "rectifier inhibit" counters and timers.
// *
// ********************************************************************************************************
void RectifierStateControl::ResetRectifierAlarms( void )
{
    BatteryTransferCounter = 0;
    RectifierInhibitTimer.ClearTimer();
    RectifierInhibitted = 0;
    PrechargeAttempts = 0;
	NB_SetNodebit( UPM_NB_TOO_MANY_BATTERY_TRANSFERS, false );
}

// *****************************************************************************
//      End of RectifierStateControl.cpp
// *****************************************************************************

