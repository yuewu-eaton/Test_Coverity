// ********************************************************************
// *    ChargerState.c
// ********************************************************************
// ********************************************************************
// *
// * THIS information is proprietary to Eaton Corporation
// *
// ********************************************************************
// *
// *  Copyright (c) 2003 Eaton Powerware
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************
// ********************************************************************
// *   FILE NAME:   ChargerState.c
// *
// *   DESCRIPTION: State machine for charger and boost.
// *
// *   ORIGINATORS: Pasi Pulkkinen
// *
// *   DATE:        23.12.2003
// *
// *   HISTORY:     See CVS history.
// ********************************************************************

// ********************************************************************
// *        INCLUDE FILE
// ********************************************************************
#include "BatteryStateControl.h"
#include <cmath>
#include "Abm.h"
#include "Eeprom_map.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "RectifierStateControl.h"
#include "Meters.h"
#include "Alarms.h"
#include "MCUState.h"
#include "ParallelCan.h"
#include "FCTState.h"
#include "Version.h"

using namespace std;

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************

BatteryStateControl BatteryConverter;

float BatUVWrnLimVPC = 0;                   // EEP
float BatUVShtLimVPC = 0;                   // EEP
float BatUVMinLimVPC = 0;                   // EEP
float BatteryOVLevelVPC = 0;                // EEP
float KeepAlive_BatUVPC = 0;                // EEP
//TEK: same variable name as inside class? uint16_t BatteryOVDelayCntr;            // EEP, load something, such that sensible value prior loading it from EEPROM
uint16_t OnBatteryAlarmDelay = 0;           // EEP        
uint16_t BatOVTime = 0;                      // EEP

uint16_t ChargerOVCounter = 0;              
uint16_t ChargerOVTimer = 0;
uint16_t BatteryBreakerCounter = 0;     // Pan/20120914,add
uint16_t BatteryBreakerFeedbackCounter = 0; // Pan/20120919,add

#define BAT_BREAKER_FEEDBACK	Driver_Detect//Pan/20130313

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************


// ********************************************************************************************************
// *
// * Function:      ChargerState 
// *
// * Purpose:       Executes State Machine for Charger Control.
// *
// * Description:   Executed at 196 uSec rate, this function determines whether the state of the charger
// *                should change, and does it.
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void BatteryStateControl::RunBatteryState(void)		//4.33k
{
    rectStatus = Rectifier.GetStatus();
    rectState = Rectifier.GetState();
    
    // rectifier dc ref is not fixed, so boost ref needs to be updated
    float RectifierRef = Rectifier.GetDCLinkVoltageRef();
    float DCLinkVoltage = DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered;
    static float BatStartRectifierRef = DCLinkVoltage;
    static StateTimer BatteryStartBoostOnTimer;

    if( !FCTStateMachine.GetFCTMode() ) //Alex add for FCT
    {
        SetDCLinkVoltage( RectifierRef - 10.0 );
    }
    else
    {
        return;
    }

    if ( BatteryStatus.bit.BatteryStartPrecharging && BatteryStatus.bit.BoostCmdOn )
    {
        //Battery boost on DC link slow start up:BatStartRectifierRef increase 2V in every utility cycle
        //if worst condition: battery lowest voltage 10V/cell, 28 cells, battery voltage 280V, BUS after pre-charge 280V - 20V = 260V, BUS target 810V
        //cycles:(810-260) / 2 = 275
        //Battery Start Boost longest time: 275 * 20ms = 5.5s
        if ( BatteryStartBoostOnTimer.CheckTimeout( BATT_20MSECOND ) )
        {
            BatStartRectifierRef += 2;
            BatteryStartBoostOnTimer.ClearTimer();
        }

        if ( BatStartRectifierRef < RectifierRef - 10.0f )
        {
            SetDCLinkVoltage( BatStartRectifierRef - 10.0f );
        }
        else
        {
            SetDCLinkVoltage( RectifierRef - 10.0f );
        }
    }
    else
    {
//	        BatStartRectifierRef = DCLinkVoltage;
		//start from min:Vbus+ (as sps get power from Vbus+
		if(DCLinkVoltagePositive.FastFiltered < (-DCLinkVoltageNegative.FastFiltered))
		{
			BatStartRectifierRef = DCLinkVoltagePositive.FastFiltered*2.0f;
		}
		else
		{
			BatStartRectifierRef = (-DCLinkVoltageNegative.FastFiltered)*2.0f;			
		}		
        BatteryStartBoostOnTimer.ClearTimer();
        SetDCLinkVoltage( RectifierRef - 10.0f );
    }

    //Rail boost only when link is under DC link set 17V
//		RailBoostLim = ( Rectifier.GetDCLinkSet() * 0.5 ) - 17.0f;
	RailBoostLim = ( Rectifier.GetDCLinkSet() * 0.5 ) - 20.0f;		//5V margin for unbalance load(ripple+-15V)
    if(NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) && 
		(BatteryBreakerInstalled))	//for hobbit147,like 9P no bat break install
    {
        CheckBatteryBreaker();
    }
	//for jira hobbit-113, faster log nobbit(debounce still 1/1)
    NB_DebounceAndQue(UPM_NB_CHARGER_ON, BatteryConverterStatus.bit.ChargerOn); 

    // Pan/20120914 end
    
    switch ( BoChMachineState )
    {
        case BATTERY_INIT_STATE:
            BatteryInitState();
            break;
        
        case BATTERY_SHUTDOWN_STATE:
            BatteryShutdownState();
            break;
        
        case BATTERY_CHARGE_STATE:
            BatteryOnChargeState();
            break; 
        
        case BATTERY_ON_BOOST_STATE:
            BatteryOnBoostState();
            break; 

        case BATTERY_TEST_STATE:
            BatteryTestState();
            break; 
        
        case BATTERY_TRIP_STATE:
            BatteryOnCBTripState();
            break;
        
        default:
            break;
    }
    
    BatteryCLReset();
}

// ********************************************************************************************************
// *
// * Function: BatteryTransferState(INT8U new_state);
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
void BatteryStateControl::BatteryTransferState(battery_states_t new_state)
{
    switch( new_state )
    {
        case BATTERY_INIT_STATE:
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = true;
            break;
        case BATTERY_SHUTDOWN_STATE:
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = true;
            break;  
        case BATTERY_CHARGE_STATE:
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = false;
            break;
        case BATTERY_TEST_STATE:            
            ChargerOff();
            BoostPowerModeOn();
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = false;
            break;
        case BATTERY_ON_BOOST_STATE:
            ChargerOff();
            if( rectState == RECTIFIER_ON_BATTERY_STATE || rectState == RECTIFIER_TO_BATTERY_STATE ||
                MCUStateMachine.GetStatus().bit.ForwardTransfer )
            {
            	// Engage the boost controller with some feedforward based on
            	// the inverter's power history
                BoostOnFF();
            }
            else
            {
                BoostOn();
            }       
            BatteryStatus.bit.BoostCmdOn = TRUE;
            BatteryStatus.bit.AutoZeroBatteryCurrentSensors = false;
            break;
        case BATTERY_TRIP_STATE:
            break;

    }
    

    SetPowerModeTarget( 0.0 ); 
    
    BatteryStatus.bit.OnBatteryStatus = FALSE;

    BatteryTimer1.ClearTimer();
    BatteryTimer2.ClearTimer();
    BatteryTimer3.ClearTimer();
    
    BatteryStatus.bit.BatteryTestCancel = 0;
    BatteryStatus.bit.BattOVTripDone = FALSE;
    ChargerPhState = 0;
	PreChargePhState = 0;
    BoChMachineState = new_state;
    NB_LogStateChange(UPM_NB_BATTERY_STATE_CHANGED, BoChMachineState);
}

// ********************************************************************************************************
// *
// * Function: void BatteryInitState(void)
// *
// * Purpose:       
// *
// * Description:   
// *
// ********************************************************************************************************
void BatteryStateControl::BatteryInitState(void)
{
//    float tempDCLinkRef;
    BatteryStatus.bit.BattLowAlarmEnabled = FALSE;
//		float BatteryVoltageHalf = RawAdcDataPtr->st.BatteryVoltage * 0.5;
	float BatteryVoltageHalfPos = 0.0f;
	float BatteryVoltageHalfNeg = 0.0f;

	if((InterfaceBoardRevID == CONST_InterfaceBrdRev_ID_P0) || 
		(InterfaceBoardRevID == CONST_InterfaceBrdRev_ID_P1))	//Board P0/1, no Vbat+ sample
	{
		BatteryVoltageHalfPos = RawAdcDataPtr->st.BatteryVoltage * 0.5;
		BatteryVoltageHalfNeg = RawAdcDataPtr->st.BatteryVoltage * 0.5;
	}
	else														//Board P2 or more, with Vbat+
	{
		BatteryVoltageHalfPos = RawAdcDataPtr->st.BatteryVoltagePos;
		BatteryVoltageHalfNeg = -RawAdcDataPtr->st.BatteryVoltageNeg;		
	}

    switch ( ChargerPhState )
    {
        case 0:
            // battery converters off
            if ( !MCUStateMachine.UPMTestMode )
            {
			 	if( fabs(BatteryCurrentPos.FastFiltered) < float( BattRelayOpenCurrent ) && 
				    fabs(BatteryCurrentNeg.FastFiltered) < float( BattRelayOpenCurrent ) )
				{
					if( ++K2OpenDelayTimer > BATT_100MSECOND )
					{
						K2OpenDelayTimer = BATT_100MSECOND;
					}
				}
				else
				{
					K2OpenDelayTimer = 0;
				}
				
            	if( ( K2OpenDelayTimer == BATT_100MSECOND ) ||
            		BatteryTimer2.CheckTimeout( 10L * BATT_ONE_SECOND ) )
            	{
	                SetBatteryRelay( RELAY_OPEN );
	                ChargerFixedDutyOff();
	                BoostOffLegA();
                
	                if ( MCUStateMachine.GetADCReadyFlag() && 
	                	!NB_GetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE))
	                {
	                    BatteryTimer2.ClearTimer();
	                    ChargerPhState++;
	                }
				}
            }
            break;
            
        case 1:
            // wait for battery voltage measurements to settle
            if ( BatteryTimer2.CheckTimeout( (5L * BATT_ONE_SECOND ) ) )
            {               
                // Make sure that battery is connected correctly (no negative voltage)
                if ( RawAdcDataPtr->st.BatteryVoltage <= BATT_NEG_FAIL_LIMIT )
                { 
                    BatteryStatus.bit.BattNegFail = TRUE;  
                    BatteryTransferState(BATTERY_TRIP_STATE);
                } 
                else if ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                          !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED))
                {
                	BatteryTimer2.ClearTimer();
                }
                else
                {         
                    BatteryTimer2.ClearTimer();
                    ChargerPhState++;
                }
            }
            break;
            
        case 2:
            // Wait until rectifier normal and commanded on, battery start
            // Check that batteries are connected before advancing
            if ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                 !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED)       || 
                 Abm().GetStatus().bit.BatteryOV )
            {
            	ChargerPhState = 0;
            }
            else if ( BatteryStatus.bit.ChrgRelCmdOn     &&
                 rectStatus.bit.RectifierOnNormal        )
            {
                DSPOutRegister.GpoC.bit.PLDBoostMode = TRUE;//EOSD-128: fix the JIRA buck&boost PWM interleve when precharge cap, blow up.				
                ChargerPhState++;
            }
            else if ( BatteryStatus.bit.StartupCommand     &&
                      !rectStatus.bit.RectifierPrecharging )
            {
                BatteryStatus.bit.BatteryStartPrecharging = 1;
                BatteryStatus.bit.StartupCommand = 0;
                PreChargePhState = 0;
                ChargerPhState = SUBSTATE_TO_BAT_START_PRECHARGE_DC;
            }
            else
            {
                //null
            }
            break;
                
         case 3: 
            //if timeout > 30 secs charger fail
            if ( BatteryTimer2.CheckTimeout( BATT_ONE_SECOND * 30 ) )
            {
      		   if ( RawAdcDataPtr->st.BatteryVoltageChgPos > (BatteryVoltageHalfPos+10.0f) ||
					(-RawAdcDataPtr->st.BatteryVoltageChgNeg) > (BatteryVoltageHalfNeg+10.0f) )
               {
                    BatteryStatus.bit.BattPrechargeVHigh = true; 
                    ChargerPhState = 0;
               }
			   
//	               if ( RawAdcDataPtr->st.BatteryVoltageChgNeg < RawAdcDataPtr->st.BatteryVoltage )
			   if ( RawAdcDataPtr->st.BatteryVoltageChgPos < (BatteryVoltageHalfPos-10.0) ||
					(-RawAdcDataPtr->st.BatteryVoltageChgNeg) < (BatteryVoltageHalfNeg-10.0) )
               {
                    BatteryStatus.bit.BattPrechargeVLow = true;
                    ChargerPhState = 0;                             
               } 
            }
         
            //precharge the battery cap
             if ( BatteryStatus.bit.ChrgRelCmdOn && 
                  !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
                  NB_GetNodebit(UPM_NB_BATTERY_INSTALLED)         &&
                  rectStatus.bit.RectifierOnNormal  ) 
             {
                 if ( (( RawAdcDataPtr->st.BatteryVoltageChgPos >= ( BatteryVoltageHalfPos - BATTVOLTBACKUP_MARGIN ) ) &&
                       ( RawAdcDataPtr->st.BatteryVoltageChgPos <= ( BatteryVoltageHalfPos + BATTVOLTBACKUP_MARGIN ) ))   &&
                      (( -RawAdcDataPtr->st.BatteryVoltageChgNeg >= ( BatteryVoltageHalfNeg - BATTVOLTBACKUP_MARGIN ) ) &&
                       ( -RawAdcDataPtr->st.BatteryVoltageChgNeg <= ( BatteryVoltageHalfNeg + BATTVOLTBACKUP_MARGIN ) ) ))
                 {
                    ChargerFixedDutyOff();
                    SetBatteryRelay( RELAY_CLOSED );
                    ChargerPhState++;
                    BatteryTimer2.ClearTimer();                           
                } 
                else
                {    
                    if ( RawAdcDataPtr->st.BatteryVoltageChgPos < BatteryVoltageHalfPos )
                    {
                        if (!BatteryConverterStatus.bit.ChargerPosOn)
                        {
                            ChargerFixedDutyPosOn();
                        }
                    }
                    else
                    {
                        ChargerFixedDutyPosOff();
                    }

                    if ( -RawAdcDataPtr->st.BatteryVoltageChgNeg < BatteryVoltageHalfNeg )
                    {
                        if (!BatteryConverterStatus.bit.ChargerNegOn)
                        {
                            ChargerFixedDutyNegOn();
                        }
                    }
                    else
                    {
                        ChargerFixedDutyNegOff();
                    }
                }
            }
            else
            {
                // not able to close relay, restart
                ChargerPhState = 0;
                ChargerFixedDutyOff();
                BatteryTimer2.ClearTimer();    
            }    
                                    
            break;
                            
                            
        case 4:
            // Wait 100ms after relay command
            // Check for battery faults 
            if ( BatteryTimer2.CheckTimeout( BATT_ONE_SECOND * 2 ) ) 
            {
                if( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
                {
                    BatteryTransferState( BATTERY_SHUTDOWN_STATE );
                }
                else
                {
                    ChargerPhState = 0;
                    BatteryTimer2.ClearTimer();
                    BatteryTimer3.ClearTimer();
                }
            }

                
             //put wait timer in here for check
             if( BatteryTimer3.CheckTimeout( BATT_ONE_SECOND ) )
             {
                 if( RawAdcDataPtr->st.BatteryVoltageChgPos < 20.0 )
                 {
                     BatteryStatus.bit.BattFaultPos = true;
                     ChargerPhState = 0;
                     BatteryTimer2.ClearTimer();
                     BatteryTimer3.ClearTimer();
                 }
                         
                 if( -RawAdcDataPtr->st.BatteryVoltageChgNeg < 20.0 )
                 {
                     BatteryStatus.bit.BattFaultNeg = true;
                     ChargerPhState = 0;
                     BatteryTimer2.ClearTimer();
                     BatteryTimer3.ClearTimer();
                 }
             }
            break;
                
        case SUBSTATE_TO_BAT_START_PRECHARGE_DC:		//1.DC start
            // TOOD: This should really all be in it's own state (e.g. BatteryPrechargingState())
            if ( BatteryConverter.CheckBatteryShutdown() ||
                 BatteryStatus.bit.CancelBatteryStart )
            {
                PreChargeOff();
                ChargerPhState = 0;
                PreChargePhState = 0;
                BatteryStatus.bit.BatteryStartPrecharging = 0;
                if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                {
                    NB_SetNodebit( UPM_NB_CHECK_PRECHARGE, true, 11 );//failure that disables batter control
                }
            }
            else
            {
                switch ( PreChargePhState )
                {
                    case 0:
                        // set relays, should be set here already
                        OpenInputRelays();
                       // DSPOutRegister.GpoB.bit.BalancerRelay = 0; //hobbit no need balance
                        PreChargePhState++;
                        BatteryTimer1.ClearTimer();
                        break;
                    case 1:
                        if ( BatteryTimer1.CheckTimeout( BATT_100MSECOND) &&
                             Rectifier.PrechargerReady() ) // wait the 10s cooldown before turning the precharger on
                        {
                            PreChargeOn();
                            PreChargePhState++;
                            BatteryTimer1.ClearTimer();
                        }
                        break;
                    case 2:
                        if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND) )
                        {
                            // check that rail is at least charging
                            float PreChargeFailedLimit = 25.0f;
                            if ( ( DCLinkVoltagePositive.FastFiltered < PreChargeFailedLimit ) ||
                                 ( DCLinkVoltageNegative.FastFiltered > -PreChargeFailedLimit ) )
                            {
                                PreChargeOff();
                                BatteryStatus.bit.BatteryStartPrecharging = 0;
                                ChargerPhState = 0;
                                PreChargePhState = 0;
                                BatteryTimer1.ClearTimer();
                                if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                                {
                                    NB_SetNodebit( UPM_NB_CHECK_PRECHARGE, true, 12);//No precharge voltage detected
                                }
                            }
                            else
                            {
                                BatteryTimer1.ClearTimer();
                                PreChargePhState++;
                            }
                        }
                        break;
                    case 3:
                        // Requirement: Do not allow the precharger to run for greater than 30 seconds continuously.
                        // If the DClink has not reached (Battery voltage + 50V) in 19 seconds, the precharge supply is turned off
                        // The possible largest time of pre-charge on continuously is: 1+ 19 + 5 = 25 seconds, then cool for 10 seconds in UPM state machine.
                        // The test shows that it takes about 10s to pre-charge the DC link when setting RawAdcDataPtr->st.BatteryVoltage + 50.0f very high(300).
//	                        if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 19 ) )	//25sec
						if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 40 ) )	//25sec
                        {
                            PreChargeOff();
                            ChargerPhState = 0;
                            PreChargePhState = 0;
                            BatteryStatus.bit.BatteryStartPrecharging = 0;
                            BatteryTimer1.ClearTimer();
                            if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                            {
                                NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 13 );//Can not reach reference voltage
                            }
                        }
                        else
                        {
                            float prechargelim = RawAdcDataPtr->st.BatteryVoltage - float( DCPreChargeRailLimit );
                            float DCLinkVoltage = fabs( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered );
//	                            if ( DCLinkVoltage >= prechargelim )
							if( (DCLinkVoltage >= prechargelim) 	&&
								(DCLinkVoltagePositive.FastFiltered >= (prechargelim*0.5f)) &&
								((-DCLinkVoltageNegative.FastFiltered) >= (prechargelim*0.5f)) )
                            {
                                BatteryTimer1.ClearTimer();
                                BatteryStatus.bit.ChrgRelCmdOn = 1;//BatteryShutdownState condition compatibility
                                PreChargePhState++;
                            }
                        }
                        break;
                    case 4:
                        //The test shows that it takes about 0.2s to ChargerFixedDutyOn
//	                        if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 5 ) )
//							if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 10 ) )
						if ( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 60 ) )	//same as ac
                        {
                            ChargerFixedDutyOff();
                            PreChargeOff();
                            ChargerPhState = 0;
                            BatteryStatus.bit.BatteryStartPrecharging = 0;
                            BatteryTimer1.ClearTimer();
                            if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                            {
                                NB_SetNodebit( UPM_NB_CHECK_PRECHARGE, true, 14);//Can not reach reference voltage
                            }

//	                            if ( RawAdcDataPtr->st.BatteryVoltageChgNeg > RawAdcDataPtr->st.BatteryVoltage )
							if ( (RawAdcDataPtr->st.BatteryVoltageChgPos > (BatteryVoltageHalfPos+10.0)) ||
								 (-RawAdcDataPtr->st.BatteryVoltageChgNeg > (BatteryVoltageHalfNeg+10.0)) )
                            {
                                BatteryStatus.bit.BattPrechargeVHigh = 1;
                            }

//	                            if ( RawAdcDataPtr->st.BatteryVoltageChgNeg < RawAdcDataPtr->st.BatteryVoltage )
							if ( (RawAdcDataPtr->st.BatteryVoltageChgPos < (BatteryVoltageHalfPos-10.0)) ||
								 (-RawAdcDataPtr->st.BatteryVoltageChgNeg < (BatteryVoltageHalfNeg-10.0))	)
                            {
                                BatteryStatus.bit.BattPrechargeVLow = 1;
                            }
                        }

                        //precharge the battery cap
                        if ( BatteryStatus.bit.ChrgRelCmdOn && !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) )
                        {
                            if ( (( RawAdcDataPtr->st.BatteryVoltageChgPos >= ( BatteryVoltageHalfPos - BATTVOLTBACKUP_MARGIN ) ) &&
                                  ( RawAdcDataPtr->st.BatteryVoltageChgPos <= ( BatteryVoltageHalfPos + BATTVOLTBACKUP_MARGIN ) ))   &&
                                 (( (-RawAdcDataPtr->st.BatteryVoltageChgNeg) >= ( BatteryVoltageHalfNeg - BATTVOLTBACKUP_MARGIN ) ) &&
                                  ( (-RawAdcDataPtr->st.BatteryVoltageChgNeg) <= ( BatteryVoltageHalfNeg + BATTVOLTBACKUP_MARGIN ) ) ))
                            {
                                ChargerFixedDutyOff();
                                PreChargeOff();
                                SetBatteryRelay( RELAY_CLOSED );
                                PreChargePhState++;
                                BatteryTimer1.ClearTimer();
                            }
                            else
                            {
                                if ( RawAdcDataPtr->st.BatteryVoltageChgPos < BatteryVoltageHalfPos )
                                {
                                    if (!BatteryConverterStatus.bit.ChargerPosOn)
                                    {
                                        ChargerFixedDutyPosOn();
                                    }
                                }
                                else
                                {
                                    ChargerFixedDutyPosOff();
                                }
            
                                if ( (-RawAdcDataPtr->st.BatteryVoltageChgNeg) < BatteryVoltageHalfNeg )
                                {
                                    if (!BatteryConverterStatus.bit.ChargerNegOn)
                                    {
                                        ChargerFixedDutyNegOn();
                                    }
                                }
                                else
                                {
                                    ChargerFixedDutyNegOff();
                                }

                            }
                        }
                        break;
                    case 5:
                        // Wait 100ms after relay command
                        // Check for battery faults
                        if ( BatteryTimer1.CheckTimeout( BATT_100MSECOND ) )
                        {
                            if( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
                            {
                                BatteryTransferState( BATTERY_SHUTDOWN_STATE );


                            }
                            else
                            {
                                ChargerPhState = 0;
                                BatteryStatus.bit.BatteryStartPrecharging = 0;
                                BatteryTimer1.ClearTimer();
                                if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                                {
                                    NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 15 );//battery switch gear closed failed
                                }
                            }
                        }
                        break;
                    default:
                        PreChargeOff();
                        ChargerPhState = 0;
                        PreChargePhState = 0;
                        break;
                }
            }
            break;
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: void BatteryShutdownState(void)
// *
// * Purpose:       
// *
// * Description:   
// *
// ********************************************************************************************************
void BatteryStateControl::BatteryShutdownState(void)
{
     
     if (NB_GetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE))
     {
         BatteryTransferState(BATTERY_TRIP_STATE);
         return;
     }
     
    if ( !BatteryStatus.bit.ChrgRelCmdOn ||
         NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
         !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED)       ||
         CheckBatteryShutdown() )
    {
        BatteryTransferState( BATTERY_INIT_STATE );
        return;
    }

    // Check if we need to switch to boost mode
    //fix APACTS-348,boost on when ups inverter off.
    if ( ( ( ( ( DCLinkVoltagePositive.FastFiltered < RailBoostLim ) ||( (-DCLinkVoltageNegative.FastFiltered) < RailBoostLim ) )
    		&& ( RELAY_CLOSED == MCUStateMachine.GetInverterRelayState() ) ) ||
           NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) ||
           BatteryStatus.bit.BoostCmdOn
         ) && ( !MCUStateMachine.GetStatus().bit.InverterSuspend )
       )
    {
        BatteryTransferState(BATTERY_ON_BOOST_STATE);
        return;
    }

    // Check if charger commanded on
    if ( BatteryStatus.bit.ChrgCmdOn && !BatteryStatus.bit.ChargerFailOV )
    {
        // allow time for auto zero of currents
        if( BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 5 ) )
        {
            ChargerOn();
            BatteryTransferState(BATTERY_CHARGE_STATE);
            return;
        }
        
    }
    
    // Check if battery test is commanded on
    
    if ( BatteryStatus.bit.BatteryTestCmd )
    {
        BatteryTransferState(BATTERY_TEST_STATE);
        return;        
    }
}

// ********************************************************************************************************
// *
// * Function: void BatteryOnChargeState(void)
// *
// * Purpose:       
// *
// * Description:   
// *
// ********************************************************************************************************
inline void BatteryStateControl::BatteryOnChargeState(void)
{
    BatteryStatus.bit.BattLowAlarmEnabled = FALSE;

    if ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
         !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED))
    {
        ChargerOff();
        
        if( BatteryTimer1.CheckTimeout( BATT_100MSECOND ) )
        {
            BatteryTransferState( BATTERY_INIT_STATE );
        }
		return;  // fix STKEV-35
    } 
    else
    {
        BatteryTimer1.ClearTimer();
    }

    // Check if charger commanded off
    if ( !BatteryStatus.bit.ChrgCmdOn )
    {
        ChargerOff();
        BatteryTransferState(BATTERY_SHUTDOWN_STATE);
        return;
    }
    
    // Check if battery test is commanded on    
    if ( BatteryStatus.bit.BatteryTestCmd )
    {
        BatteryTransferState(BATTERY_SHUTDOWN_STATE);
        return;        
    }
    
    if ( !BatteryStatus.bit.ChrgRelCmdOn ||
         CheckBatteryShutdown() )
    {
        ChargerOff();
        BatteryTransferState( BATTERY_INIT_STATE );
        return;
    }
    // Check if we need to switch to boost mode
    //fix APACTS-348,boost on when ups inverter off.
    if ( ( ( ( ( DCLinkVoltagePositive.FastFiltered < RailBoostLim ) ||( (-DCLinkVoltageNegative.FastFiltered) < RailBoostLim ) )
    		&& ( RELAY_CLOSED == MCUStateMachine.GetInverterRelayState() ) ) ||
           NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) ||
           BatteryStatus.bit.BoostCmdOn
         ) && ( !MCUStateMachine.GetStatus().bit.InverterSuspend )
       )
    {
    	//fix LFP-32:abnormal pulse occured when online to battery mode
    	if( BatteryConverterStatus.bit.ChargerOn )
    	{
    		ChargerOff();
    		DSPOutRegister.GpoC.bit.PLDBoostMode = FALSE;
            WriteDSPOutputs_TSK();
    	}
    	else
    	{
    		BatteryTransferState(BATTERY_ON_BOOST_STATE);
    	} 
        return;
    }
    else
    {
    	// fix STKEV-35 when single side DC-link voltage oscillates around RailBoostLim,
    	// charge is tuned off but battery converter does not transit to boost mode
        //merge from 93E500K VAVE
		if( !BatteryConverterStatus.bit.ChargerOn
		   && BatteryStatus.bit.ChrgCmdOn )
		{
//		        ChargerOn();
			//for hobbit-119, dc ov/uv FastAuxPowerShutdown() to charger off, charger mistake on/off log, 
			// cause no led flash/sci(interrupt still ok) by spi block 
			if(isFastAuxPowerShutdown() == false)		//mean no FastAuxPowerShutdown()
	        {
				ChargerOn();
	        }
		}
    }
}

// ********************************************************************************************************
// *
// * Function: void BatteryOnBoostState(void)
// *
// * Purpose:       
// *
// * Description:   
// *
// ********************************************************************************************************
inline void BatteryStateControl::BatteryOnBoostState(void)
{
    BatteryStatus.bit.BattLowAlarmEnabled = TRUE;
   
    BatteryStatus.bit.OnBatteryStatus = TRUE;

     if(NB_GetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE))
     {
         BatteryStatus.bit.BoostCmdOn = FALSE;
         BatteryStatus.bit.ChrgCmdOn = FALSE;
         BatteryTransferState(BATTERY_TRIP_STATE);
         return;
     }
    
	if ( !BatteryStatus.bit.ChrgRelCmdOn )
    {
        BoostOffLegA();		//can use legA/B off: bat to init,mean rec have off, 
		BoostCmdOff();
        BatteryTransferState( BATTERY_INIT_STATE );
        return;
    }

    if ( CheckBatteryShutdown() )
    {        
        BatteryTransferState(BATTERY_INIT_STATE);
    } 

    // Check conditions to shut down boost after minimum delay
    if( BatteryTimer3.CheckTimeout( MinBoostTimeBatt ) )	//5s
    {
        if ( rectStatus.bit.RectifierOnNormal && 
             !rectStatus.bit.WalkIn &&
             !rectStatus.bit.PowerShare)
        {
            BatteryStatus.bit.BoostCmdOn = FALSE;
            BoostOffLegA(); 
            BatteryTransferState(BATTERY_SHUTDOWN_STATE);
            return;
        }
    }

    if ( BatteryStatus.bit.BatteryStartPrecharging )
    {
        const float DCLinkRampupCompleteVoltage = 5.0f;//DCLink error bounds for complete battery start precharging
        float DCLinkVoltage = DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered;
        //Error between Actual DCLink voltage and target DCLink voltage.
        float DCLinkError = fabs( (Rectifier.GetDCLinkVoltageRef() - 10.0f) - DCLinkVoltage);

        if ( DCLinkError < DCLinkRampupCompleteVoltage )
        {
            BatteryStatus.bit.BatteryStartPrecharging  = 0;

            NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, false );//pre-charge completes.
        }
        else
        {
            if ( ( (RECTIFIER_ON_BATTERY_STATE != rectState) &&
			       (RECTIFIER_TO_BATTERY_STATE != rectState) ) ||
                 BatteryTimer1.CheckTimeout( BATT_ONE_SECOND * 5 ) )
            {
                ChargerFixedDutyOff();
                BoostOff();
                BoostCmdOff();
                BatteryStatus.bit.BatteryStartPrecharging  = 0;
                BatteryTimer1.ClearTimer();
                if ( MCUStateMachine.BatStartupAttempts() >= 3 )
                {
                    NB_DebounceAndQue( UPM_NB_CHECK_PRECHARGE, true, 16 );//Could not reach target DC link voltage level
                }

                BatteryTransferState( BATTERY_INIT_STATE );
            }
        }
    }
    else
    {
        BatteryTimer1.ClearTimer();
    }
}

// ********************************************************************************************************
// *
// * Function: void BatteryTestState(void)
// *
// * Purpose:       
// *
// * Description:   
// *
// ********************************************************************************************************
float ScaledIrefBattestMin = 0.0f;
float ScaledIrefBattestSet = 0.0f;
float CurrentCmdP_Record = 0.0f;
float OutputPowerBatBoost = 0.0f;
uint16_t FlagCurrentCmdP_Record = false;
float PercentLoadSum_Record = 0.0f;
uint16_t FlagLoadOver50 = 0;
extern float DynamicLoadPercent;
extern float DynamicLoadPercentRec;

inline void BatteryStateControl::BatteryTestState(void)		//4.333k = 26k/6
{
    float RampRate = 4.0;  // Ramp is 4 W * 510 1/s = 2040 W/s
	uint16_t FlagLoadVariate = 0;
	uint16_t FlagBatNotGood = 0;	//bat lose or bat bad
	float tempCalcu = 0.0f;

	//For Bat test Control:
	//		Rec under limit ScaledIrefMax;  Bat under Boost mode

	//0.Record the CurrentCmdP when first in Bat test, only one time
	if(FlagCurrentCmdP_Record == false)
	{
		FlagCurrentCmdP_Record = true;
		CurrentCmdP_Record = Rectifier.CurrentCmdP;
		PercentLoadSum_Record = ScreenMeters.PercentLoad.sum;
	}

	//1.Limit to 50%Load_Rate for LegA_boost (as LegB_boost_ST reuse Rec_ST, always run in Rec)
	if(PercentLoadSum_Record > 50.0f)
	{
		FlagLoadOver50 = true;
	}

	//1.1 if Load >50%: Load_Boost=50%; and, Load_rec= Load-Load_boost(50%)
	if(FlagLoadOver50 == true)
	{
//			tempCalcu = ScreenMeters.PercentLoad.sum - 50.0f;
//			tempCalcu = tempCalcu / ScreenMeters.PercentLoad.sum;	//Iref_min = (load- 50%)/load * Iref_record
//			ScaledIrefBattestMin = CurrentCmdP_Record * tempCalcu;	//todo  49 a;
//			OutputPowerBatBoost = ScreenMeters.OutputPower.sum*0.5f; //todo	//load_boost=50%, load_rec=Load - 50%

		tempCalcu = ScreenMeters.PercentLoad.sum - 50.0f;
		tempCalcu = tempCalcu / 100.0f;	//Iref_min = (load- 50%)/load * Iref_record
		ScaledIrefBattestMin = Rectifier.ScaledIrefMax_Ori * tempCalcu;	//todo-ok	
		OutputPowerBatBoost = (OutputkVARating*100.0f)*0.5f; //todo-ok //load_boost=50%, load_rec=Load - 50%
	}
	//1.2 else Load <= 50%: Load_boost=Load; Load_rec=0
	else 
	{
		ScaledIrefBattestMin = 0.0f;
		OutputPowerBatBoost = ScreenMeters.OutputPower.sum;			//load_boost=Load; load_rec=0%
	}	
	//end

	//2.If Load variate:   abs(load-load_1) > 10%, temporary abort Battest
	if(fabs(DynamicLoadPercent - PercentLoadSum_Record) >= 15.0f)	//todo-ok
	{
		FlagLoadVariate = true;
	}	
	//(2.1~2.2, tempor not use)
	//2.1. Load switch 0~50%, not need action	
	//2.2. Load switch >50%, Over_50% part immediate active Rec ScaledIrefMax
			
        // downsample 5102Hz to 510.2Hz
    if ( ++BattTestDownSampleCount >= 10 )	//40k -20kload
    {
        BattTestDownSampleCount = 0;
          //Check timer for battery test 1
        if( !BatteryTimer1.CheckTimeout( BatteryTest1Time * BATT_TEST_ONE_SECOND ) &&
            !BatteryStatus.bit.BatteryTestCancel )
        {
            //ramp to the power target*0.5
            tempCalcu = RampRate * (float) BatteryTimer1.TimerValue();
			
            if(tempCalcu < (OutputPowerBatBoost*0.5f))
            {
				SetPowerModeTarget(tempCalcu);
				//3.1. Set the Load_rec: ScaledIrefMax
				tempCalcu = 1.0f - (tempCalcu / OutputPowerBatBoost);		//load_rec descend: percent 1.0f->0.5f
            }			
			else
			{
				tempCalcu = 0.5f;	//50%
			}
			
			//tempCalcu: K
			//CurrentCmdP_Record: x0
			//ScaledIrefBattestMin: x1
			//ScaledIrefBattestSet: y
			//y=x1+K*(x0-x1)
			ScaledIrefBattestSet = ScaledIrefBattestMin + tempCalcu*(CurrentCmdP_Record-ScaledIrefBattestMin);				
        }
          //Check timer for battery test 2
        else if( !BatteryTimer2.CheckTimeout( BatteryTest2Time * BATT_TEST_ONE_SECOND ) &&
                 !BatteryStatus.bit.BatteryTestCancel )
        {
            // Record Power and Voltage of %50 load battery test
            if( BatteryTimer2.TimerValue() == 1 )
            {    
                BatteryTestResults.FiftyPercentTest.Power = fabs( BatteryCurrentPos.FastFiltered * BatteryVoltage.FastFiltered );
                BatteryTestResults.FiftyPercentTest.Voltage = BatteryVoltage.FastFiltered;
                BatteryTestResults.FiftyPercentTest.Current = fabs( BatteryCurrentPos.FastFiltered );
            }   
            
//	            //ramp to the power target
//	            float tempTarget = RampRate * (float) BatteryTimer2.TimerValue() + BatteryTestResults.FiftyPercentTest.Power; 
//	            
//	            if( ( tempTarget) < ( ScreenMeters.OutputPower.sum ) )
//	            {        
//	                SetPowerModeTarget( tempTarget ); 
//	            }  

			//ramp to the power target*1.0
			float tempCalcu = RampRate * (float) BatteryTimer2.TimerValue() + BatteryTestResults.FiftyPercentTest.Power;
			if(tempCalcu < (OutputPowerBatBoost))
			{

				SetPowerModeTarget(tempCalcu);
				//3.1. Set the Load_rec: ScaledIrefMax
				tempCalcu = 1.0f - (tempCalcu / OutputPowerBatBoost);		//load_rec descend: percent 0.5f->0.0f
			}			
			else
			{
				tempCalcu = 0.0f;
			}
			//tempCalcu: K
			//CurrentCmdP_Record: x0
			//ScaledIrefBattestMin: x1
			//ScaledIrefBattestSet: y
			//y=x1+K*(x0-x1)
			ScaledIrefBattestSet = ScaledIrefBattestMin + tempCalcu*(CurrentCmdP_Record-ScaledIrefBattestMin);	
        }
          //Battery test is compelte
        else
        {
            //Record the power and voltage of 100% load battery test
            if( ( BatteryTimer3.TimerValue() == 0 ) )
            {
                BatteryTestResults.OneHundredPercentTest.Power = fabs( BatteryCurrentPos.FastFiltered * BatteryVoltage.FastFiltered );
                BatteryTestResults.OneHundredPercentTest.Voltage = BatteryVoltage.FastFiltered;
                BatteryTestResults.OneHundredPercentTest.Current = fabs( BatteryCurrentPos.FastFiltered );
                                 
                BTR.TestResultsReady();  //tell BTR to record the test results
            }
            
            BatteryTimer3.CheckTimeout( 5 );
            
            //ramp down the battery test 4x is 4kw/sec
            if( GetPowerModeTarget() > 100.0 )  //exit if less than 100 watts
            {
//	                SetPowerModeTarget( GetPowerModeTarget() - ( RampRate * (float)BatteryTimer3.TimerValue() ));				
            	tempCalcu = GetPowerModeTarget() - ( RampRate * (float)BatteryTimer3.TimerValue() );
				SetPowerModeTarget(tempCalcu);				
				//3.1. Set the Load_rec: ScaledIrefMax
				tempCalcu = 1.0f - (tempCalcu / OutputPowerBatBoost);		//load_rec descend: percent 1.0f->0.5f
				ScaledIrefBattestSet = ScaledIrefBattestMin + tempCalcu*(CurrentCmdP_Record-ScaledIrefBattestMin);					
            }
            else  
            {
                BatteryStatus.bit.BatteryTestCmd = 0;               
				ScaledIrefBattestSet = Rectifier.ScaledIrefMax_Ori;
            } 						
        }
    }    
    
        // Check if we need to switch to boost mode
    ////fix APACTS-348,boost on when ups inverter off.
//	    if( ( ( ( DCLinkVoltagePositive.FastFiltered < RailBoostLim ) || ( (-DCLinkVoltageNegative.FastFiltered) < RailBoostLim ) )
//	    		&& ( RELAY_CLOSED == MCUStateMachine.GetInverterRelayState() ) ) ||
//	           NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) ||
//	           BatteryStatus.bit.BoostCmdOn )
	if( NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) ||		//2ms-line lose; 80ms- line low(+-15%)
		BatteryStatus.bit.BoostCmdOn )

    {
		
        BatteryStatus.bit.BatteryTestCmd = 0;
        BatteryTransferState(BATTERY_ON_BOOST_STATE);
		//6.Clear Battest boost param
//			ScaledIrefBattestMin = 0.0f;
//			ScaledIrefBattestSet = 0.0f;
		CurrentCmdP_Record = 0.0f;
		OutputPowerBatBoost = 0.0f;
		FlagCurrentCmdP_Record = false;
		PercentLoadSum_Record = 0.0f;
		//end
	
		Rectifier.ScaledIrefMax = Rectifier.ScaledIrefMax_Ori;	//Rec fast back
        return;
    }
	else
	{
			
		if((DCLinkVoltagePositive.FastFiltered < (RailBoostLim-10.0f)) || ((-DCLinkVoltageNegative.FastFiltered) < (RailBoostLim-10.0f)) )
		{
			//4.mean: 4.1)load variate or 4.2)Bat disconnect
			if(FlagLoadVariate == false)
			{
				//not load vari, so only  4.2)Bat bad or Bat lose
				FlagBatNotGood = true;
			}
		}		
	}
		
	// Check for reasons to leave battery test (batt UV, rect on batt, current limit, power < 5%); 0x01 remined for battery fail
	BatTestAbandonReason = ( BatteryCLCount > 0 ) * 																  0x02 +
						   ( Rectifier.GetState() != RECTIFIER_NORMAL_STATE ) * 									  0x04 +
						   ( ScreenMeters.OutputPower.sum < ((float)OutputkWRating * 100.0f * 0.05f ) ) *			  0x08 +
//							   ( RawAdcDataPtr->st.BatteryVoltageChgNeg < ( BatUVShtLimVPC * (float)NumberOfBatteryCells ) ) * 0x10 +
						   ( (RawAdcDataPtr->st.BatteryVoltageChgPos - RawAdcDataPtr->st.BatteryVoltageChgNeg) < ( BatUVShtLimVPC * (float)NumberOfBatteryCells ) ) * 0x10 +
						   ( (RawAdcDataPtr->st.BatteryVoltageChgPos) < ( BatUVShtLimVPC * (float)NumberOfBatteryCells *0.5f) ) * 0x20 +
						   ( (-RawAdcDataPtr->st.BatteryVoltageChgNeg) < ( BatUVShtLimVPC * (float)NumberOfBatteryCells *0.5f) ) * 0x40 +
						   ( FlagLoadVariate == true ) * 0x80 + 		//new: mean load variate > 10%
						   ( FlagBatNotGood == true ) * 0x0100 ;		 //new: mean bat lose or bad
	
	if ( BatTestAbandonReason							||
		 ( NB_GetNodebit( UPM_NB_BATTERY_TEST_FAILED ) ) ||
		 !BatteryStatus.bit.BatteryTestCmd
	   )
	{
		BatteryStatus.bit.BatteryTestCmd = 0;
		BoostOffLegA();
		BatteryTransferState(BATTERY_SHUTDOWN_STATE);	 
		//7.Clear Battest boost param
//			ScaledIrefBattestMin = 0.0f;
//			ScaledIrefBattestSet = 0.0f;
		CurrentCmdP_Record = 0.0f;
		OutputPowerBatBoost = 0.0f;
		FlagCurrentCmdP_Record = false;
		PercentLoadSum_Record = 0.0f;
		//end
		Rectifier.ScaledIrefMax = Rectifier.ScaledIrefMax_Ori;	//Rec fast back
	}

}

// ********************************************************************************************************
// *
// * Function: void ChrgOnCBTripState(void)
// *
// * Purpose: Battery breaker trip state
// *
// ********************************************************************************************************
void BatteryStateControl::BatteryOnCBTripState(void)
{
//    BatteryStatus.bit.BatteryOnCBTripState = TRUE;
    BatteryStatus.bit.BattLowAlarmEnabled = FALSE;

    ChargerOff();
    BoostOff();
 

    if( BatteryTimer1.CheckTimeout( BATT_100MSECOND ) )
    {
        //OutRegister.bit.Batt_CB_Trip = 0;
        if( BatteryTimer3.CheckTimeout( BATT_ONE_SECOND*5 ) )
        {
            //ChrgStatusBits.bit.BattCBCmdTrip = FALSE;
            // Fix HOBBIT-96
            if( !BatteryStatus.bit.BattNegFail )
            {
                BatteryTransferState(BATTERY_INIT_STATE);			// This gives minimum time off, in case someone calls trip again.
            }
        }
    } 
    else
    {
        //OutRegister.bit.Batt_CB_Trip = 1;
    }
}

// ********************************************************************************************************
// *
// * Function: bool CheckBatteryShutdown(void);
// *
// * Purpose: Check some reasons to shut down battery control or prevent it from starting
// *
// * Parms Passed   :   none
// * Returns        :   TRUE/FALSE
// *
// * Description: Returns TRUE if there is a failure that disables battery control
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool BatteryStateControl::CheckBatteryShutdown(void)
{
    // UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP is not placed here because the
    // Rectifier state machine takes care of that.
    if( NB_GetNodebit( UPM_NB_BATTERY_CURRENT_LIMIT ) )
        //ChrgStatusBits.bit.PermamentDisable ||
        //ChrgStatusBits.bit.HWOverloadShtD )
    {
        return true;
    } 
    else
    {
        return false;
    }
    
} 

// ********************************************************************************************************
// *
// * Function: void DebounceCharger(void);
// *
// * Purpose: Debounce, alarm logic and alarm counter for battery alarms
// *
// * Description: Debounce on battery and on charge states
// *              Called by 500ms periodic function.
// *              If this is called from any other periodic interval, adjust LOW_BATT_SHUTDOWN_COUNTER!
// *
// ********************************************************************************************************
void BatteryStateControl::DebounceCharger(void)		
{                                                             
    eAbmState ABMState = Abm().GetState(); 
    ABM_Status_Bits ABMStat = Abm().GetStatus().bit;
    float LimitVoltage = 20.0;
    
    static uint16_t battDisconnectedTime = 0;
    static float lastBatteryVoltage = BatteryVoltage.SlowFiltered;
    
    // * Low battery alarm *
//	    if ((BattVoltsperCell <= BatUVWrnLimVPC) && BatteryStatus.bit.BattLowAlarmEnabled)
	if (((BattVoltsperCell <= BatUVWrnLimVPC) || 
		(BattVoltsperCell1 <= BatUVWrnLimVPC) || 
		(BattVoltsperCell2 <= BatUVWrnLimVPC)) && 
		BatteryStatus.bit.BattLowAlarmEnabled)
    {
        BatteryStatus.bit.BatteryLow = TRUE;
    } 
    else
    {
            // Clear alarm only when not on battery
            // Changes on load can cause variations on battery voltage
        if (!BatteryConverterStatus.bit.BoostOn &&
            !NB_GetNodebit(UPM_NB_LOW_BATTERY_SHUTDOWN))
        {
            BatteryStatus.bit.BatteryLow = FALSE;
        }
   }
   
    // Suppress this alarm while on battery test, this is a failure, test will abort and fail.
    // This alarm is supposed to initiate a panic shutdown in monitoring software, don't want to do that.
    //If the system is not redundant, all UPS will alarm.when any one batterylow
    //If the system is redundant, the UPS without batterylow shoule not alarm even though others batterylow
    //If system is not redundant after All batterylow UPSs are shutdowm already, \
    //the online UPS shoule not alarm even though shutdown UPS is still batterylow
    NB_DebounceAndQue(UPM_NB_BATTERY_LOW,
                        BatteryStatus.bit.BatteryLow ||
                        ( ParallelCan.ParGlobalOrData.BatteryStatus.bit.BatteryLow          &&
                          ParallelCan.ParGlobalOrData.BatteryStatus.bit.ShutdownTimerActive &&
                          !NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT ) ), ((BattVoltsperCell <= BatUVWrnLimVPC) |
                          ((BattVoltsperCell1 <= BatUVWrnLimVPC) << 1) | ((BattVoltsperCell2 <= BatUVWrnLimVPC) << 2)) );    

        // * Low Battery shutdown alarm *
//	    if (BattVoltsperCell <= BatUVShtLimVPC && 
	if(((BattVoltsperCell <= BatUVShtLimVPC) ||
		(BattVoltsperCell1 <= BatUVShtLimVPC) ||
		(BattVoltsperCell2 <= BatUVShtLimVPC))&& 
        BatteryConverterStatus.bit.BoostOn &&
        !FCTStateMachine.GetFCTMode())
    {
        BatteryStatus.bit.ShutdownTimerActive = TRUE;
    }
    else
    {
        if( ( Rectifier.GetState() != RECTIFIER_ON_BATTERY_STATE ) ||
//	            (( BattVoltsperCell >= BatUVShtLimVPC + 0.20) &&
//	            ( BattVoltsperCell1 >= BatUVShtLimVPC + 0.20) &&
//	            ( BattVoltsperCell2 >= BatUVShtLimVPC + 0.20)))
			(( BattVoltsperCell >= BatUVShtLimVPC + 0.30) &&	//for hobbit-72,no bat+- sample
			( BattVoltsperCell1 >= BatUVShtLimVPC + 0.30) &&
			( BattVoltsperCell2 >= BatUVShtLimVPC + 0.30)))
        {
            BatteryStatus.bit.ShutdownTimerActive = FALSE;
        }    
    }       
	NB_DebounceAndQue(UPM_NB_LOW_BATTERY_SHUTDOWN, (BatteryStatus.bit.ShutdownTimerActive), ((BattVoltsperCell <= BatUVShtLimVPC) |
						((BattVoltsperCell1 <= BatUVShtLimVPC) << 1) | ((BattVoltsperCell2 <= BatUVShtLimVPC) << 2)) );

    if ( BatteryStatus.bit.ShutdownTimerActive )
    {
            // This is filtered enough that debounce isn't necessary
//	        if ( (BattVoltsperCell <= BatUVMinLimVPC) ||
//	             (--BatteryLowCounter == 0) )
		if (((BattVoltsperCell <= BatUVMinLimVPC) || 
			(BattVoltsperCell1 <= BatUVMinLimVPC) ||
			(BattVoltsperCell2 <= BatUVMinLimVPC)) ||
			 (--BatteryLowCounter == 0) )
        {
            if( 0 == BatteryLowCounter )
            {
                BatteryLowCounter = 1;
            }
            BatteryStatus.bit.ShutdownTimerActive = FALSE;
            BatteryStatus.bit.BattUVShutDown = TRUE;
            if ( !NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT ) && ( MCUStateMachine.GetStatus().bit.OnInverter ) )
            {
                //sync only when my inverter is on
                BatteryStatus.bit.UVShutdownSync = TRUE;
            }
        }
    }
    else
    {
        BatteryLowCounter = LOW_BATT_SHUTDOWN_COUNTER;

        if ( !NB_GetNodebit( UPM_NB_BATTERY_LOW ) )
        {
            BatteryStatus.bit.BattUVShutDown = FALSE;
            BatteryStatus.bit.UVShutdownSync = FALSE;
        }
    }

        // * On battery alarm *
        // Don't give this alarm if ABM battery test running
    if ( BatteryConverterStatus.bit.BoostOn )
    {
        // UPS_ON_BATTERY alarm is NOT given if
        //   - UPS is on battery and ABM is running battery support test
        //
        // UPS_ON_BATTERY alarm is given if
        //   - UPS is on battery for at least the defined alarm delay time
        //   - "Battery Low" alarm is active and UPS is on battery (despite the delay and ABM BST)

        if (BatteryStatus.bit.BatteryLow ||
            ((OnBatteryAlarmDelayCntr >= (OnBatteryAlarmDelay << 1)) ) )
        {
            NB_DebounceAndQue(UPM_NB_UPS_ON_BATTERY, BatteryStatus.bit.OnBatteryStatus);
        }
        else
        {
            OnBatteryAlarmDelayCntr++;
        }
        
    }
    else 
    {
        NB_DebounceAndQue(UPM_NB_UPS_ON_BATTERY, 0);
        OnBatteryAlarmDelayCntr = 0;
    }
    
//	    NB_DebounceAndQue(UPM_NB_CHARGER_ON, BatteryConverterStatus.bit.ChargerOn);

 #define THIRTY_MINUTES 3600
 
    if( ChargerOVCounter )  //decrement charger ov off counter 
    {
        if( ChargerOVCounter > 2) //give 3 attempts before locking the charger out
        {
            BatteryStatus.bit.ChargerFailOV = TRUE;
        }
        if( ChargerOVTimer++ > THIRTY_MINUTES )
        {
            ChargerOVCounter--;
            ChargerOVTimer = 0;
        }
        if( NB_GetNodebit(UPM_NB_BATTERIES_DISCONNECTED) ||
            !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )
        {
            BatteryStatus.bit.ChargerFailOV = false;
            ChargerOVCounter = 0;
        }
    } 
    else
    {
        ChargerOVTimer = 0;
    }
    
    // Pan/20120919 add to check mixed battery breaker,begin
    if( BatteryBreakerFeedbackCounter )
    {
        BatteryBreakerFeedbackCounter--;
    }
    else
    {
        BatteryStatus.bit.BattBreakerFeedbackOpen = false;
    }
   // Pan/20120919 end 
    
    //battery disconnected
    if (  ( BatteryVoltage.SlowFiltered < ( Batt_Disconnect_VPC * (float)NumberOfBatteryCells ) ) )
    {
        BatteryStatus.bit.BattDisconnected = TRUE;
    }
    else if ( ( ABMState == ABM_RESET                                    ) &&
              ( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN )          ) &&
              ( lastBatteryVoltage - RawAdcData.st.BatteryVoltage > 0.5 ) )  //GPE-1486 // voltage drop more than 0.5V for 0.5 seconds
    {
        if ( ++battDisconnectedTime >= 10 )  // For 10 * 0.5 seconds
        {
            battDisconnectedTime = 0;
            BatteryStatus.bit.BattDisconnected = TRUE;
        }
    }
    else
    {
        battDisconnectedTime = 0;
        
        if (BatteryVoltage.SlowFiltered >= (BATT_DISCONNECT_VPC * (float)NumberOfBatteryCells + 10.0))
        {
            BatteryStatus.bit.BattDisconnected = FALSE;
        }
    }
    
    lastBatteryVoltage = RawAdcData.st.BatteryVoltage;  //GPE-1486
            
    NB_DebounceAndQue(UPM_NB_BATTERIES_DISCONNECTED, BatteryStatus.bit.BattDisconnected &&
                                                     NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) );

     //battery relay failure
     if( NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )          
     {
         if( fabs( BatteryCurrentPos.FastFiltered ) > 10.0 )
         {
            BatteryStatus.bit.BattRelayFailToOpen = true;
         }
         else
         {
            BatteryStatus.bit.BattRelayFailToOpen = false;
         }
         
         BatteryStatus.bit.BattRelayFailToClose = false;
     }
     else
     {
         LimitVoltage = 50.0;

         if( fabs( BatteryVoltageBU.FastFiltered - BatteryVoltage.FastFiltered ) > LimitVoltage )
         {
            BatteryStatus.bit.BattRelayFailToClose = true;
         } 
         else
         {
            BatteryStatus.bit.BattRelayFailToClose = false;
         }
         
         BatteryStatus.bit.BattRelayFailToOpen = false;
     }

     NB_DebounceAndQue(UPM_NB_BATTERY_CONTACTOR_FAIL, 
                       (BatteryStatus.bit.BattRelayFailToOpen || BatteryStatus.bit.BattRelayFailToClose)&&NB_GetNodebit(UPM_NB_BATTERY_INSTALLED), 
                             BatteryStatus.bit.BattRelayFailToOpen + BatteryStatus.bit.BattRelayFailToClose * 2
                         ); //APACTS-385

    //charger failure
     NB_DebounceAndQue(UPM_NB_CHARGER_FAILURE, 
                         BatteryStatus.bit.ChargerFailed || 
                         BatteryStatus.bit.BattPrechargeVHigh || 
                         BatteryStatus.bit.BattPrechargeVLow ||
                         BatteryStatus.bit.ChargerFailOV, 
                             BatteryStatus.bit.ChargerFailed +
                             BatteryStatus.bit.BattPrechargeVHigh * 2 +
                             BatteryStatus.bit.BattPrechargeVLow * 4 +
                             BatteryStatus.bit.ChargerFailOV * 8
                          ); 
 
     //battery failure
     NB_DebounceAndQue(UPM_NB_BATTERY_NEEDS_SERVICE, 
                         ABMStat.OpportunityCharge ||
                         BatteryStatus.bit.BattNegFail ||
                         BatteryStatus.bit.BattFaultPos ||
                         BatteryStatus.bit.BattFaultNeg,
                             ABMStat.OpportunityCharge +
                             BatteryStatus.bit.BattNegFail * 2 +
                             BatteryStatus.bit.BattFaultPos * 4 +
                             BatteryStatus.bit.BattFaultNeg * 8
                         );
                                               
}

// ****************************************************************************
// *
// * Function:     void SetBatteryRelay( uint16_t state )
// *
// * Purpose:      Open/Close battery relays relay
// *
// * Parms Passed: State    RELAY_OPEN / RELAY_CLOSED
// * Returns:      Nothing
// *
// * Description:  Opens/Closes the relays and logs the event.
// *
// ****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void BatteryStateControl::SetBatteryRelay( uint16_t state )
{
	K2OpenDelayTimer = 0;
	
    if ( RELAY_CLOSED == state )
    {
        if ( NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
        {
          //  if ( RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative > RawAdcDataPtr->st.BatteryVoltageChgNeg )
            if ( ( RawAdcDataPtr->st.RailVoltagePositive > RawAdcDataPtr->st.BatteryVoltageChgPos ) &&
            	 ( RawAdcDataPtr->st.RailVoltageNegative < RawAdcDataPtr->st.BatteryVoltageChgNeg )	)
            {
                DSPOutRegister.GpoB.bit.BatteryRelay1 = 1;
                NB_DebounceAndQue( UPM_NB_BATTERY_CONTACTOR_OPEN, false );
            }
        }    
    }
    else
    {
        if ( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) )
        {
        	// On startup, the relay is typically open.  However, if there is
        	// residual energy in the DC link, then we may choose not to set
        	// the nodebit even though the relays are already open.
        	if ( !DSPOutRegister.GpoB.bit.BatteryRelay1 &&
                 !DSPOutRegister.GpoB.bit.BattLegB_Relay &&
                 NB_GetNodebit(UPM_NB_BATTERY_INSTALLED))
            {
            	NB_DebounceAndQue( UPM_NB_BATTERY_CONTACTOR_OPEN, true);
            }
            
           // if ( RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative > RawAdcDataPtr->st.BatteryVoltageChgNeg )
        	if ( ( RawAdcDataPtr->st.RailVoltagePositive > RawAdcDataPtr->st.BatteryVoltageChgPos ) &&
        	     ( RawAdcDataPtr->st.RailVoltageNegative < RawAdcDataPtr->st.BatteryVoltageChgNeg ) )
            {
                DSPOutRegister.GpoB.bit.BatteryRelay1 = 0;
                DSPOutRegister.GpoB.bit.BattLegB_Relay = 0;
				NB_SetNodebit( UPM_NB_BAT_LEGB_RELAY_OPEN, true );	//for hobbit-113					
                if( NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )
                {
                    NB_DebounceAndQue( UPM_NB_BATTERY_CONTACTOR_OPEN, true );
                }
            }
        }    
    }
}

// ****************************************************************************
// *
// * Function:     void ClearBatteryAlarms( void )
// *
// * Purpose:      clears battery stick alarms
// *
// * Parms Passed: nothing
// * Returns:      boolean
// *
// * Description:  
// *
// ****************************************************************************
void BatteryStateControl::ClearBatteryAlarms( void )
{
    BatteryStatus.bit.ChargerFailed = 0;
    BatteryStatus.bit.BattNegFail = 0;  
    BatteryStatus.bit.BattPrechargeVHigh = 0;  
    BatteryStatus.bit.BattPrechargeVLow = 0;  
    BatteryStatus.bit.BattFaultPos = 0; 
    BatteryStatus.bit.BattFaultNeg = 0;  
    BatteryStatus.bit.ChargerFailOV = 0;
    ChargerOVCounter = 0;
}

// ****************************************************************************
// *
// * Function:     void CheckBatteryUVShutDown( void )
// *
// * Purpose:      Checks for a low battery shutdown.  Active if low battery shutdown or
// *                if another UPS has low battery shutdown and parallel system isn't redundant.
// *
// * Parms Passed: nothing
// * Returns:      boolean
// *
// * Description:  
// *
// ****************************************************************************
bool BatteryStateControl::CheckBatteryUVShutDown( void )
{

  return ( BatteryStatus.bit.BattUVShutDown ||
           ParallelCan.ParGlobalOrData.BatteryStatus.bit.UVShutdownSync );
}

// ****************************************************************************
// *
// * Function:     void CheckBatteryBreaker( void )
// *
// * Purpose:      Conditions trip battery breaker include: 
// *               over voltage || over current || EPO
// *               ANY of the condition above occurs, drive GPIO high for 1s to trip breaker
// *
// * Parms Passed: nothing
// * Returns:      nothing
// *
// * Description:  
// *
// ****************************************************************************
// Pan/20120925 add begin
void BatteryStateControl::CheckBatteryBreaker()		//4.3k                  
{    
    
    switch( BatBreakerState )
    {
        case BAT_BRK_OPENED_STATE:
            DSPOutRegister.GpoB.bit.TripBatteryBreaker = 0;
            BatteryBreakerCounter = 0;
            if( !NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) &&
                NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )  
            {   
                BatBreakerState = BAT_BRK_CLOSED_STATE;
            }
            break;
                        
        case BAT_BRK_TRIP_STATE:
            if( ++BatteryBreakerCounter >= BATT_ONE_SECOND )
            {
                DSPOutRegister.GpoB.bit.TripBatteryBreaker = 0;
                //wait 8 seconds,battery breaker should be open already, or the breaker is fail;
                if( BatteryBreakerCounter >= 8L * BATT_ONE_SECOND )
                {
                    BatteryBreakerCounter = 0;
                    BatBreakerState = BAT_BRK_FAIL_STATE;
                    NB_SetNodebit( UPM_NB_BATTERY_BREAKER_FAILURE, true );                    
                }
                else if( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                         !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )
                {
                    BatteryBreakerCounter = 0;
                    BatBreakerState = BAT_BRK_OPENED_STATE;   
                }
            }
            else
            {
                DSPOutRegister.GpoB.bit.TripBatteryBreaker = 1;
            }             
            break;
            
        case BAT_BRK_FAIL_STATE:
            // stay here until breaker fail alarm clears ;
            if( !NB_GetNodebit( UPM_NB_BATTERY_BREAKER_FAILURE ) )
            {
                if( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                    !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )
                {
                    BatBreakerState = BAT_BRK_OPENED_STATE;
                }
                else
                {
                    BatBreakerState = BAT_BRK_CLOSED_STATE;
                }
            }
            break;
        
        case BAT_BRK_CLOSED_STATE:
        default:
            if( ( NB_GetNodebit( UPM_NB_REMOTE_EMERGENCY_POWER_OFF ) ||
                  NB_GetNodebit( UPM_NB_BATTERY_DC_OVER_VOLTAGE    ) ||
                  NB_GetNodebit( UPM_NB_BATTERY_CURRENT_LIMIT )    ) )
            {
                BatBreakerState = BAT_BRK_TRIP_STATE;    
            }
            if( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED))
            {
                BatBreakerState = BAT_BRK_OPENED_STATE;
            }
            break;
    }    

}
// Pan/20120925 end

// ********************************************************************
//          End of chargerstate.c
// ********************************************************************
