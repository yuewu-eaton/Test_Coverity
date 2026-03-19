// ******************************************************************************************************
// *        ABM.C
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 EATON
// *                      ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *     FILE NAME:   ABM.C
// *
// *     DESCRIPTION: ABM (Advanced Battery Management) charging cycles
// *                  implementation
// *
// *     ORIGINATOR:  Tuomo Kaikkonen
// *
// *     DATE:        02/19/2004
// *
// *     HISTORY:     See HPO CVS history
// ******************************************************************************************************

// ******************************************************************************************************
// *        INCLUDE FILES
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Abm.h"
#include "Eeprom_Map.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include "Meters.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "RectifierStateControl.h"
#include "Btr.h"
#include "ParallelCan.h"
#include "math.h"
#include <cmath>
#include "Spi_Task.h"

// ******************************************************************************************************
// *        EEPROM VARIABLES
// ******************************************************************************************************
const uint16_t BatteryCellsDefault_20K_30K = 216;
const uint16_t BatteryCellsDefault_100k_120K = 240;
uint16_t NumberOfBatteryCells;      //
float    NumberOfBatteryStrings;    // maybe a float number
uint16_t BattChrgTimeMax;           // maximum time in constant current charge
uint16_t BattFloatTime;             // ABM cycling float time
uint16_t BattMaxRestTime;           // maximum rest mode time, duration of rest mode
uint16_t BattMaxDischTime;          // max discharge time to reset ABM
uint16_t BattRestFailTime;          // battery OCV failure time in rest mode
uint16_t CommonBattery;             // 1 if internal parallel common battery.  
                                    // 2 if external parallel common battery.
                                    // 0 for separate batteries.

uint16_t ABMDisabled = 0;           //Jacob/20130823/add

// all voltages are in Volts/cell!!
float BattABMFloatVPC;              // charge voltage, the level where charge mode changes to float mode
float BattConstFloatVPC;            // ABM disabled float voltage??
float BattRestMinVoltage;           // battery voltage for early termination of rest mode
float LowRateTestThreshold;         // Battery test fail threshold for discharge time > 15 min
float HighRateTestThreshold;        // Battery test fail threshold for discharge time <= 15 min
float BattChrgCurrentMax;
float MinInputForBatteryCharging_Percent;   // Charger off below this voltage, 1.0 = 100 % Nominal
float ABMBattOVClear;
float BattMaxchrgCurrentforESS;     //max ESS charge current.

uint32_t total_Float_Time;     // 
uint16_t BattFloatTimeExt;     // ABM cycling float time extension; BattFloatTimeExt = 1.5 * TotalChargeTime
float CompensateVPC = 0.0f;
uint16_t EE_BatUVWrnLimVPC = 1810;  //add to configure low battery waring for HV system
uint16_t EE_BatUVShtLimVPC = 1750;
uint16_t EE_BatUVMinLimVPC = 1670;

namespace {
SlaveAbm AbmSlave;
}
AdvancedBatteryManager AbmMaster;

AbmActions& Abm( void )
{
	if (CommonBattery == CommonBattery_Internal && MyUPMNumber != 0)
	{
		return AbmSlave;
	}
	else if (CommonBattery == CommonBattery_External && 
		(MyUPSNumber != 0 ||
		 MyUPMNumber != 0) )
	{
		return AbmSlave;
	}
	else
	{
		return AbmMaster;
	}
}


// ******************************************************************************************************
// *        EEPROM FUNCTIONS
// ******************************************************************************************************
void ee_calc_abm( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_Batt_ChargeV_PerCell:
            BattABMFloatVPC = float(*data) / 1000.0;
            break;
        
        case PARAM_Batt_ConstFloatV_PerCell:
            BattConstFloatVPC = float(*data) / 1000.0;
            break;
        
        case PARAM_batOpChrgCellV:
            BattRestMinVoltage = float(*data) / 1000.0;
            break;
        
        case PARAM_BattTestFailVLow:
            LowRateTestThreshold = float(*data) / 1000.0;
            break;
        
        case PARAM_BattTestFailVHigh:
            HighRateTestThreshold = float(*data) / 1000.0;
            break;
        
        case PARAM_Batt_OV_Time:
        	Abm().SetOVTime(uint16_t(*data)*10);
        	break;
        	
        case PARAM_LowBatteryWarnLimit:
            BatUVWrnLimVPC = float(*data) / 1000.0;
            break;
        
        case PARAM_Batt_UV_PerCell:
            BatUVShtLimVPC = float(*data) / 1000.0;
            break;
        
        case PARAM_Batt_UV_Absolute_PerCell:
            BatUVMinLimVPC = float(*data) / 1000.0;
            break;
        
        case PARAM_Batt_OV_PerCell:
            BatteryOVLevelVPC = float(*data) / 1000.0;
            // use 5% hysteresis before allowing the charger to come back on.
            ABMBattOVClear = BattConstFloatVPC - BattConstFloatVPC * 0.05;
            break;
        
        case PARAM_Batt_ChrgCurrent:
//	        	if( *data > 1000)
//				{
//					BattChrgCurrentMax = 100.0f;
//				}
//				else
//				{
//					BattChrgCurrentMax = float(*data) / 10.0;
//				}
            if(UPMSystem == HV_20K)
            {
                if( *data > 200)
                {
                    BattChrgCurrentMax = 20.0f;
                }
                else
                {
                    BattChrgCurrentMax = float(*data) / 10.0;
                }
            }
            else //40k
            {
                if( *data > 400)
                {
                    BattChrgCurrentMax = 40.0f;
                }
                else
                {
                    BattChrgCurrentMax = float(*data) / 10.0;
                }
            }
            BatteryConverter.SetChargeCurrentLimit( BattChrgCurrentMax );
            break;
        
        case PARAM_MinInputForCharging:
            MinInputForBatteryCharging_Percent = float(100 - *data)/1000.0;
            break;
        
        case PARAM_KeepAlive_BatUVPC:
            KeepAlive_BatUVPC = float(*data) / 1000.0;
            break;

        case PARAM_BatteryTest1Time:
            BatteryConverter.BatteryTest1Time = uint32_t(*data);
            break;
            
        case PARAM_BatteryTest2Time:
            BatteryConverter.BatteryTest2Time = uint32_t(*data);
            break;

        case PARAM_batMaxChrgCurrentforESS:
        	BattMaxchrgCurrentforESS = float(*data) / 100.0 ;   // total UPS current limit is MaxchrgCurrentforESS *4
        	break;

        case PARAM_ABMDisabled:
            ABMDisabled = uint16_t(*data);
            break;

        case PARAM_NumCells:
            if( 0 == *data )
            {
                if( (HV_20K == UPMSystem) || (HV_30K == UPMSystem) ||(HV_40K == UPMSystem))
                {
                    NumberOfBatteryCells = BatteryCellsDefault_20K_30K;
                }
                else
                {
                    NumberOfBatteryCells = BatteryCellsDefault_100k_120K;
                }
                Abm().SetBatterySetupRequired(true);
            }
            else
            {
                NumberOfBatteryCells = *data;
                Abm().SetBatterySetupRequired(false);
                //fix jira new3C3-166
        		if((NumberOfBatteryCells >= 168)&&
        		    (NumberOfBatteryCells < 192))
    		   	{
    		   		//34752 = 181 * 192
                    EE_BatUVWrnLimVPC = ((uint32_t)347520)/NumberOfBatteryCells;
                    //33600 = 175 * 192
                    EE_BatUVShtLimVPC = ((uint32_t)336000)/NumberOfBatteryCells;
                    //32064 = 167 * 192
                    EE_BatUVMinLimVPC = ((uint32_t)320640)/NumberOfBatteryCells;		        		   	
    		   	}
    		   	else
    		   	{
                    EE_BatUVWrnLimVPC = 1810;
                    EE_BatUVShtLimVPC = 1750;
                    EE_BatUVMinLimVPC = 1670;	        		       
    		   	}

                if ( EEStatusBits.bit.EEDataInitialized )
                {
                    EE_ID* ee = GetParameterEE( PARAM_LowBatteryWarnLimit );
                    PutEepData(ee->eep_addr, 1, &EE_BatUVWrnLimVPC, 0);

                    ee = GetParameterEE( PARAM_Batt_UV_PerCell );
                    PutEepData(ee->eep_addr, 1, &EE_BatUVShtLimVPC, 0);

                    ee = GetParameterEE( PARAM_Batt_UV_Absolute_PerCell );
                    PutEepData(ee->eep_addr, 1, &EE_BatUVMinLimVPC, 0);
                }        	
            }
            break;
			
		case PARAM_Battery_disconnect_VPC:
        	Batt_Disconnect_VPC = float(*data) / 1000.0f;
            break;

        default:
            break;
    }
}


// ******************************************************************************************************
// *
// * Function:    AdvancedBatteryManager( void )
// *
// * Purpose:     For initializing ABM prior to use
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************
AdvancedBatteryManager::AdvancedBatteryManager( void )
	: TotalChargeTime(0)
	, TotalDischargeTime(0)
	, CumulativeStateTime(0)
	, BSTCompletionPercent(0)
	, BattStartChrgTimer(0)
	, BSTTestTime(0)
{
    PreviousState = ABM_RESET;
}

AdvancedBatteryManager::~AdvancedBatteryManager()
{
}

void
AdvancedBatteryManager::ResetAlarms()
{
    ABM_Status.bit.OpportunityCharge = 0;
    ABM_Status.bit.Battery_Failure = 0;
    ABM_Status.bit.BatteryOV = 0;
    BatteryConverter.BatTestAbandonReason = 0;
    NB_SetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE, false);
}

void 
AdvancedBatteryManager::ChargerCmdOn( void )
{
    ABM_Status.bit.ChargerCmdOn = 1;
    ABM_Status.bit.ManualChargerOff = 0;
}

/*
 * Function:   bool BatteryOV( void )
 * Purpose:     Determine if this node's battery pack has an over-voltage alarm
 *              active.
 * 
 * Description: Return true if any charger that is connected to this battery
 *              pack is observing an over-voltage alarm.  False otherwise.
 * 
 */
bool 
AdvancedBatteryManager::BatteryOV() const
{
	bool alarm = BatteryOVSelf();
	
     // Check OV alarms for slave nodes
     switch (CommonBattery)
     {
     	case CommonBattery_Separate:
     	default:
     		// Do nothing special, only pay attention to self
     		break;
     	
     	case CommonBattery_Internal: {
     		// Loop over CAN nodes common to this UPS only
     		for (unsigned upm = 1; upm < ParallelCanNetwork::MAX_NUM_UPM; ++upm)
     		{
     			// If any node (other than self) has an OV alarm, so does the master.
     			if (upm != MyUPMNumber &&
     				ParallelCan.UpmData[MyUPSNumber][upm].AbmStatus.bit.BatteryOV)
     			{
					alarm = true;
     			}
     		}
         	break;
     	}
     	case CommonBattery_External:
     		// Any node in the system is alarming
     		alarm = ParallelCan.ParGlobalOrData.AbmStatus.bit.BatteryOV;
     		break;
     }
     
     return alarm;
}

// ******************************************************************************************************
// *
// * Function:    ABM_Run( void )
// *
// * Purpose:     For running the ABM state machine, to be called periodically
// *              (for example 10 times/sec is enough)
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: Runs the ABM state machine and runs the ABM functions
// *
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Run( void )
{
	AbmActions::Run();
	    
    static uint16_t  ABM_HE_Timer = 0;
    const float EsschargeCurrentHysteresis = 0.25f * BattChrgCurrentMax; //25% hysteresis for Ess max charge current
    const float LowLineChargerHysteresis = 5.0;
    float min_input_for_batt_charging = MinInputForBatteryCharging_Percent * float(OutNomVolts);
    rectifier_states_t tempRectState = Rectifier.GetState();
    uBatteryConverterStatus BattStatus = BatteryConverter.GetStatus();

	// Set this nodebit status in the status bits so that slave nodes are aware
	// of it.
    ABM_Status.bit.BatteryOVTrip = NB_GetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE);
    
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // low line charger off
        if ( !ABM_Status.bit.LowLineChargerOff )
        {
            if ( ( ScreenMeters.InputVoltageRMS.phA < min_input_for_batt_charging ) ||
                 ( ScreenMeters.InputVoltageRMS.phB < min_input_for_batt_charging ) ||
                 ( ScreenMeters.InputVoltageRMS.phC < min_input_for_batt_charging ) )
            {
                ABM_Status.bit.LowLineChargerOff = true;    
            }
        }
        else
        {
            if ( ( ScreenMeters.InputVoltageRMS.phA > ( min_input_for_batt_charging + LowLineChargerHysteresis ) ) &&
                 ( ScreenMeters.InputVoltageRMS.phB > ( min_input_for_batt_charging + LowLineChargerHysteresis ) ) &&
                 ( ScreenMeters.InputVoltageRMS.phC > ( min_input_for_batt_charging + LowLineChargerHysteresis ) ) )
            {
                ABM_Status.bit.LowLineChargerOff = false;    
            }     
        }         
    }
 
    if( tempRectState > RECTIFIER_PRECHARGE_STATE )  // temporary
     {
         // increase time counter
         CurrentStateTime.CheckTimeout(0xfffffffful);

         // are batteries connected ?
         if ( NB_GetNodebit(UPM_NB_BATTERIES_DISCONNECTED) ||
              !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED)     ||
              NB_GetNodebit(UPM_NB_BATTERY_CONTACTOR_OPEN) ||
              ((fabs(NumberOfBatteryStrings)  < FLT_EPSILON) ) || // && (NumStrings_Batt_2 == 0)) )
               ABM_Status.bit.BatterySetupRequired )
         {
             ABM_Status.bit.No_Battery = TRUE;
         }
         else
         {
             ABM_Status.bit.No_Battery = FALSE;
         }

         //if the connected battery is discharging, no matter which UPM is it, ABM is DISCHARGING
         if ( CheckBatteryDischarge() )
         {
              if (ABM_Status.bit.State != ABM_DISCHARGING)
              {
                  Transfer_State(ABM_DISCHARGING);
              }
         }
         else
         {
             if ( ABM_Status.bit.No_Battery ||
                  ABM_Status.bit.LowLineChargerOff ||
                  ABM_Status.bit.BldInpChargerOff )
             {
                 if (ABM_Status.bit.State != ABM_RESET)
                 {
                     Transfer_State(ABM_RESET);
                 }
             }
         }
 
         if( ABM_Status.bit.ChargerCmdOn )
         {
             ABM_Status.bit.ChargerCmdOn = 0;
             NB_DebounceAndQue( UPM_NB_CHARGER_ON_COMMAND, false );
             Transfer_State(ABM_RESET);
         }
 
         if( ABM_Status.bit.ChargerCmdOff )
         {
             ABM_Status.bit.ChargerCmdOff = 0;
             NB_DebounceAndQue( UPM_NB_CHARGER_OFF_COMMAND, false );
             if(!ABMDisabled)
             {
                 Transfer_State(ABM_REST);
             }
             else
             {
                 Transfer_State(ABM_RESET);
             }
         }
         
         ABM_Status.bit.BatteryOV = BatteryOVSelf();
         if (BatteryOV())
         {
             if (ABM_Status.bit.State != ABM_RESET)
             {
				ChargerOVCounter++;
				Transfer_State(ABM_RESET);
             }
         }

         //When HV unit is floating, 
         //set BelowHEChargeLimit when charger current < BattMaxchrgCurrentforESS for 20s
         //clear BelowHEChargeLimit once charger current > BattMaxchrgCurrentforESS + Hysteresis         
         if( ( ABM_Status.bit.State == ABM_FLOAT ) ||
             ( ABM_Status.bit.State == ABM_REST )  ||
             ( ( ABM_Status.bit.State == ABM_RESET ) && ABMDisabled ) )
         {
            if( ScreenMeters.BatteryCurrent <= BattMaxchrgCurrentforESS )
            {
                if( ++ABM_HE_Timer > ABM_CLK_20S )
                {
                    ABM_HE_Timer = ABM_CLK_20S;
                    ABM_Status.bit.BelowHEChargeLimit = true;
                }
            }
            else if( ScreenMeters.BatteryCurrent > ( BattMaxchrgCurrentforESS + EsschargeCurrentHysteresis ) )
            {
                ABM_HE_Timer = 0;
                ABM_Status.bit.BelowHEChargeLimit = false;
            }
         }
         else
         {
            ABM_HE_Timer = 0;
            ABM_Status.bit.BelowHEChargeLimit = false;
         }
        
         switch(ABM_Status.bit.State)
         {
             case ABM_CHARGE:
                 Charge_State();
                 break;
             case ABM_FLOAT:
                 Float_State();
                 break;
             case ABM_REST:
                 Rest_State();
                 break;
             case ABM_DISCHARGING:
                 Discharging_State();
                 break;
             case ABM_RESET:
             default:
                 Reset_State();
                 break;
         }
     }
     else
     {
         Transfer_State(ABM_RESET);
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Transfer_State( uint16_t new_State )
// *
// * Purpose:     To change the ABM state to the wanted state
// *
// *
// * Parms Passed : new_State  the wanted state
// * Returns      : None
// *
// *
// * Description: Changes the ABM state machine state to the wanted state.
// *              Is responsible of updating the CumulativeStateTime according
// *              to defined rules.
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Transfer_State( eAbmState newState )
{
	AbmActions::Transfer_State(newState);
	
     if (newState != ABM_Status.bit.State)
     {
         // switch-case to update CumulativeStateTime variable when state changes
         switch(newState)
         {
             case ABM_RESET:
             case ABM_CHARGE:
             case ABM_FLOAT:
                 BattStartChrgTimer = START_CHRG_TIME;
                 //lint -fallthrough
             case ABM_REST:
                 if (ABM_Status.bit.State != ABM_DISCHARGING)
                 {
                     CumulativeStateTime = 0;
                 }
                 break;
 
             case ABM_DISCHARGING:
                 CumulativeStateTime += CurrentStateTime.TimerValue();
                 break;
                 
             default:
                break;
         }


         PreviousState = eAbmState(ABM_Status.bit.State);
         ABM_Status.bit.State = newState;
         CurrentStateTime.ClearTimer();
         Timer2.ClearTimer();
         Timer3.ClearTimer();
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Reset_State( void )
// *
// * Purpose:     For resetting the ABM operation
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: Resets ABM and restarts from charging state.
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Reset_State( void )
{
    AbmActions::Reset_State();
    uRectifierStatus RectStatus = Rectifier.GetStatus();
    uBatteryConverterStatus BattStatus = BatteryConverter.GetStatus();
    
    CumulativeStateTime = 0;
    ABM_Status.bit.BST_Enable = FALSE;
     
    // Set status for XCP etc.
    if (ABM_Status.bit.No_Battery)
    {
        ABM_Batt_Status = UI_STATUS_NO_BATT;
    }
    else
    {
        ABM_Batt_Status = XCP_STATUS_REST;
    }
 
    //if the connected battery is discharging, no matter which UPM is it, ABM is DISCHARGING
    if ( CheckBatteryDischarge() )
    {
         Transfer_State(ABM_DISCHARGING);
    }
    else
    {
        // Two separate OV alarm thresholds.  Only trip for the activemost-high
        // threshold.  Re-enable the charger when below the lower threshold.
        if (BatteryOV())
        {
            if (Timer2.CheckTimeout(BatteryOVAlarmTimeout))
            {
                NB_SetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE, true);
            }
        }
        else
        {
            NB_SetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE, false);
            Timer2.ClearTimer();
           
            // Allow a charging cycle to resume when the battery voltage lowers
            // to less than this threshold.
            bool BatteryOVResumeCharging = BattVoltsperCell <= ABMBattOVClear &&
                    BattVoltsperCell1 <= ABMBattOVClear &&
                    BattVoltsperCell2 <= ABMBattOVClear &&
                    !BatteryConverter.GetBatteryStateStatus().bit.ChargerFailOV;
            
            if (ABM_Status.bit.BatteryOV)
            {
                // Resume charing 30 seconds after battery voltage drops
                // below the resumption threshold.
                if (BatteryOVResumeCharging)
                {
                    if (Timer3.CheckTimeout(ABM_CLK_30S))
                    {
                        ABM_Status.bit.BatteryOV = false;
                        Transfer_State(PreviousState);
                    }
                    else
                    {
                        Timer3.ClearTimer();
                    }
                }
            }
            else
            {
                // begin a new charge cycle on timer expiration.
                if ( !ABM_Status.bit.No_Battery &&
                     ( !NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN ) ) &&
                     RectStatus.bit.RectifierOnNormal &&
                     !ABM_Status.bit.BldInpChargerOff &&
                     !ABM_Status.bit.LowLineChargerOff &&
                     !NB_GetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE) )
                {
                    if( !ABMDisabled    ||
                        !ABM_Status.bit.ManualChargerOff )
                    {
                        if (--BattStartChrgTimer == 0)
                        {
                           ABM_Status.bit.ManualChargerOff = 0;
                           Transfer_State(ABM_CHARGE);
                        }
                    }
                    else
                    {
                        if( Timer1.CheckTimeout(OCV_DISABLE_TIME) )
                        {
//	                            if ( BattVoltsperCell < BattRestMinVoltage )	//<2.1V
							if ((BattVoltsperCell < BattRestMinVoltage)	||
								(BattVoltsperCell1 < BattRestMinVoltage) ||
								(BattVoltsperCell2 < BattRestMinVoltage))
                            {
                                ABM_Status.bit.ManualChargerOff = 0;
                                Transfer_State(ABM_CHARGE);
                            }
                        }
                    }
                }
                else
                {
                    BattStartChrgTimer = START_CHRG_TIME;
                }
            }
         }
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Charge_State( void )
// *
// * Purpose:     Implements the charging state of the ABM state machine
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: Charges the batteries, until the wanted voltage level is reached.
// *              If maximum time of charging is exceeded and the voltage does not
// *              reach the wanted level, an alarm is given.
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Charge_State( void )
{
	AbmActions::Charge_State();
	
    battery_states_t BattState = BatteryConverter.GetBatteryState();
	
    uint32_t total_Charge_Time;
 
    // status to show outside
    ABM_Batt_Status = XCP_STATUS_CHARGE;
 
    // Battery Support Test not ready
    ABM_Status.bit.BST_Enable = FALSE;
 
    // Clear discharge time, we're charging the batteries !
    TotalDischargeTime = 0;
 
 
         // get total state time in h*10 format
    total_Charge_Time = Get_Total_State_Time();
 
         // If we go to battery in this state, ABM is reset
     if ( (TotalDischargeTime != 0) ||
          (total_Charge_Time > 100  && 
          	( BattState != BATTERY_CHARGE_STATE ) ) )
     {
         Transfer_State(ABM_RESET);
     }
     else
     {
         // The battery float voltage is greater than or equal to BattABMFloatVoltage
         // and the charge current in less than or equal to 75% of maximum charge current
//			 if( ( BattVoltsperCell >= ( BattABMFloatVPC - CompensateVPC ) ) &&
 		 if( ( BattVoltsperCell >= ( BattABMFloatVPC - CompensateVPC ) ) &&
		 	 ( BattVoltsperCell1 >= ( BattABMFloatVPC - CompensateVPC ) ) &&
			 ( BattVoltsperCell2 >= ( BattABMFloatVPC - CompensateVPC ) ) &&
		  	 (( BatteryCurrentPos.FastFiltered <= ( 0.75 * BattChrgCurrentMax ) ) &&
			  ( BatteryCurrentNeg.FastFiltered <= ( 0.75 * BattChrgCurrentMax ) ) ) )
         {
                 // total time charge mode
             TotalChargeTime = (uint16_t)total_Charge_Time;
 
                 // calculate float time extension time
             BattFloatTimeExt = FLOAT_T_EXT_MULTIPLIER * TotalChargeTime;
 
                 // and goto float state
             Transfer_State(ABM_FLOAT);
         }
         else
         {
             if (total_Charge_Time >= (uint32_t)BattChrgTimeMax)
             {
                 //just go to rest mode
                 // TODO: Lift this up to common base class, somehow?
                 BatteryConverter.SetChargerFailed();
                              // ... turn charger off here
                 BatteryConverter.ChargerCmdOff();
                 if(!ABMDisabled)
                 {
                     Transfer_State(ABM_REST);
                 }
                 else
                 {
                     Transfer_State(ABM_RESET);
                 }
             }
         }
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Float_State( void )
// *
// * Purpose:     Implements the floating state of the ABM state machine
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: Keeps the charger on the float level. When the time of running battery support
// *              test comes, transfers to BST state if the conditions are ok. If the condition
// *              check fails, tries for 1 minute to start the test. If the test cannot be run,
// *              it won't be tried again before the next float cycle.
// *              After the maximum float time is exceeded, transfers to rest state.
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Float_State( void )
{
	AbmActions::Float_State();
	
         // status to show outside
     ABM_Batt_Status = XCP_STATUS_FLOAT;
 
         // BST not ready
     ABM_Status.bit.BST_Enable = FALSE;
 

     total_Float_Time = Get_Total_State_Time();
 
         // check if the total time in the state exceeds the floating mode time
     if ( (total_Float_Time >= (BattFloatTime + BattFloatTimeExt) ) &&
          (!ABMDisabled                                           ) )   
     {
         Transfer_State(ABM_REST);
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Rest_State( void )
// *
// * Purpose:     Implements the rest state of the ABM state machine
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: Checks the total time spent in rest mode, and goes to charging state when
// *              the limit is exceeded. Checks the cumulative discharge time and
// *              transfers to charging state if it exceeds the limit.
// *              Checks the OCV (Open Cell Voltage) condition, but not
// *              during the first 30 minutes after starting/resuming this state
// *              and activates the nodebit if necessary.
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Rest_State( void )
{
	AbmActions::Rest_State();
	
     enum
     {
         OCV_FAIL_DEBOUNCE_TIME = 10
     };
 
     static uint16_t ocv_fail_debounce = 0;
     uint32_t rest_Time;
 
         // status for XCP
     ABM_Batt_Status = XCP_STATUS_REST;
 
         // batteries fully charged, enable BST
     if( BTR.PercentOfCapacity == 1  )
     {
        ABM_Status.bit.BST_Enable = TRUE;           
     }
 
         // get total state time in h*10 format
     rest_Time = Get_Total_State_Time();

 
     if (rest_Time >= BattMaxRestTime)
     {
         // Reset the state machine to start over
         Transfer_State(ABM_RESET);
     }
 
     if ( Timer2.CheckTimeout( OCV_DISABLE_TIME ) )
     {
         if (BattVoltsperCell < BattRestMinVoltage)
//	 		 if((BattVoltsperCell < BattRestMinVoltage) ||
//			 	(BattVoltsperCell1 < BattRestMinVoltage) ||
//			 	(BattVoltsperCell2 < BattRestMinVoltage))
         {
             if (rest_Time <= BattRestFailTime)
             {
                 if (ocv_fail_debounce++ > OCV_FAIL_DEBOUNCE_TIME)
                 {
                     Transfer_State(ABM_RESET);
                     ABM_Status.bit.OpportunityCharge = true;
                 }
             }
             else
             {
                 Transfer_State(ABM_RESET);
             }
         }
         else
         {
             ocv_fail_debounce = 0;
         }
     }
}

// ******************************************************************************************************
// *
// * Function:    ABM_Discharging_State( void )
// *
// * Purpose:     Implements the Discharging state (someone other than ABM is discharging batteries).
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description: When discharging ends, adds the time of discharge to the cumulative discharge time
// *              and resumes the previous state from which the discharging state was entered.
// *              Discharge time accumulated to cumulative discharging time is rounded to seconds
// *              (but minimum of 1 sec will be added if short discharge is below 0.5secs).
// *
// ******************************************************************************************************
void AdvancedBatteryManager::Discharging_State( void )
{
	AbmActions::Discharging_State();
	
     float BSTthresh;
     float CurrentDischargeLoad;
     float CheckLoadChanged;
     static uint16_t BSTfaildebounce;
     uBatteryStateStatus BattStateStatus = BatteryConverter.GetBatteryStateStatus();
 
 
     enum {
         BST_DEBOUNCE_TIME = 10
     };
 
     // status to show outside
     if (BatteryConverter.BatteryStatus.bit.OnBatteryStatus)
     {
         // Only show discharging if on battery status is active, else leave at whatever it was
         ABM_Batt_Status = XCP_STATUS_DISCHARGE;
     }
 
     if (CurrentStateTime.TimerValue() % ABM_TIMEBASE == 0)
     {
         TotalDischargeTime++;
     }
 
     if ( !CheckBatteryDischarge() )     // if "discharging batteries ended"
     {
     	// if the discharge time is less than 20s, resume previous state.  Else
     	// start a new ABM cycle.
     	if (TotalDischargeTime < BattMaxDischTime)
     	{
        	Transfer_State(PreviousState);
     	}
     	else
     	{
     		Transfer_State(ABM_RESET);
     	}
     }
     
     // check load change or not since battery start discharge from fully charged
     if( ABM_Status.bit.BST_Enable )
     {
        if( BTR.PercentOfCapacity >= 0.99 )    // if discharge not start yet
        {
            PreviousDischargeLoad = ScreenMeters.OutputPower.sum;       //save the load when discharge begin from fully charged
            if( PreviousDischargeLoad <= 700.0)//500.0  JIRA APACFA-59
            {
                PreviousDischargeLoad = 700.0;//500.0  JIRA APACFA-59
            }
            ABM_Status.bit.DischargeLoadChanged = 0;
        }
        else
        {
            CurrentDischargeLoad = ScreenMeters.OutputPower.sum;
            if( CurrentDischargeLoad <= 700.0 )//500.0 JIRA APACFA-59
            {
                CurrentDischargeLoad = 700.0;//500.0 JIRA APACFA-59
            }
        
            CheckLoadChanged = std::fabs( CurrentDischargeLoad - PreviousDischargeLoad ) / CurrentDischargeLoad;
        
            if( CheckLoadChanged >= 0.1 )       //if load changed 10%, stop discharge test
            {
                ABM_Status.bit.DischargeLoadChanged = 1;
            }                
        }  
     }
 
      
     if ( ( ABM_Status.bit.BST_Enable && !ABM_Status.bit.DischargeLoadChanged ) ||       // check if load changed
           BattStateStatus.bit.BattCmsgOn                                       ||
           BattStateStatus.bit.BattTestOn)
     {
        uint16_t tempTime;
        if( ( BTR.FullCapacityBTR * 0.25 ) > 65535.0 ) tempTime = 65535;
        else tempTime = (uint16_t) ( BTR.FullCapacityBTR * 0.25 );
        
        BSTTestTime = tempTime;
             // If batteries fully charged, check battery voltage against failure limits
         if (BSTTestTime > BST_LOW_RATE_THRESH)
         {
                 // Use 'low rate' voltage threshold
             BSTthresh = LowRateTestThreshold;  
         }
         else
         {
                 // Use 'high rate' theshold
             BSTthresh = HighRateTestThreshold;
         }
 
         if (TotalDischargeTime < BSTTestTime)
         {
                 // Battery voltage needs to stay above failure threshold for first 25% of discharge
//	             if (BattVoltsperCell < BSTthresh)
 			 if((BattVoltsperCell < BSTthresh) ||
			 	(BattVoltsperCell1 < BSTthresh) ||
			 	(BattVoltsperCell2 < BSTthresh))
             {
                 if (BSTfaildebounce++ > BST_DEBOUNCE_TIME)
                 {
                     ABM_Status.bit.Battery_Failure = TRUE;
                     ABM_Status.bit.BST_Enable = FALSE;
                 }
             }
             else
             {
                 BSTfaildebounce = 0;
             }
         }
         else
         {
             if (BattStateStatus.bit.BattCmsgOn ||
                 BattStateStatus.bit.BattTestOn)
             {
                 if (BattStateStatus.bit.ShutdownTimerActive ||
                     BattStateStatus.bit.BatteryLow)
                 {
                     ABM_Status.bit.Battery_Failure = TRUE;
                 }
             }
         }
     }
}

// ******************************************************************************************************
// *
// * Function:    Get_Total_State_Time( void )
// *
// * Purpose:     For calculating the total time spent in a state (rest/charge/float)
// *
// *
// * Parms Passed : None
// * Returns      : Total time in hours*10 format used by EEPROM variables
// *                BattChrgTimeMax, BattFloatTime, BattMaxRestTime, BattRestFailTime, & batSuppTestP
// *
// * Description: Calculates the total time in a state (rest/charge/float/const-float),
// *              with total time we mean the time in the state when visits to ABM_DISCHARGING
// *              or ABM_BATT_SUPP_TEST do not reset the total time.
// *
// *              (for example, 3 hours in rest state + 15min ABM_DISCHARGING + 5 hours in rest
// *                            = 3 + 5 = 8 hours in rest state)
// *
// ******************************************************************************************************
uint32_t AdvancedBatteryManager::Get_Total_State_Time(void)
{
    uint32_t total_State_Time;
    uint32_t secs;

        // total state time
    total_State_Time = CumulativeStateTime + CurrentStateTime.TimerValue();
    secs = total_State_Time / ABM_TIMEBASE;

        // and scaled to hours*10 format used by EEPROM time values in hour format
    total_State_Time = secs / 360;

    return total_State_Time;
}
// ******************************************************************************************************
// * Function:    CheckBatteryDischarge( void )
// * Purpose:     Check if the battery is discharging 
// * Returns:     BatteryDischarge
// * Description: Battery is discharging as long as any UPM connect to the battery is discharging 
// ******************************************************************************************************
bool AdvancedBatteryManager::CheckBatteryDischarge(void)
{
    bool BatteryDischarge = false;
    switch(CommonBattery)
    {
        case CommonBattery_Internal:
            for (uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; ++upm)
            {
                BatteryDischarge = BatteryDischarge                                                        ||
                                   ParallelCan.UpmData[MyUPSNumber][upm].BatteryStatus.bit.OnBatteryStatus ||
                                   ParallelCan.UpmData[MyUPSNumber][upm].BatteryStatus.bit.BattTestOn;
            };
            break;
        case CommonBattery_External:
            BatteryDischarge = ParallelCan.ParGlobalOrData.BatteryStatus.bit.OnBatteryStatus ||
                               ParallelCan.ParGlobalOrData.BatteryStatus.bit.BattTestOn;
            break;
        case CommonBattery_Separate:
            BatteryDischarge = BatteryConverter.BatteryStatus.bit.OnBatteryStatus ||
                               BatteryConverter.BatteryStatus.bit.BattTestOn;
            break;
        default:
            break;
    }
    return(BatteryDischarge);
}

// ******************************************************************************************************
// * Function:    TempCompensateVPC( void )
// * Purpose:     To adjust battery charging voltage depending on battery cabinet temperature
// * Parms Passed : temperature. the unit is 0.1 degree; e.g. 250 = 25degree 
// * Returns      : None
// * Description  : CompensateVPC = CompensateRate(unit is mV) * temperature difference
// ******************************************************************************************************
void CalTempCompensateVPC(uint16_t temperature)
{
    if( UnitConfig.bit.TempCompensateEnable )
    {
        CompensateVPC = float( CompensateRate ) * 0.001f * ( float( temperature ) * 0.1f - 25.0f );
        
    }
    else
    {
        CompensateVPC = 0.0f;
    }
}

// ******************************************************************************************************
// * END OF ABM.C
// ******************************************************************************************************
