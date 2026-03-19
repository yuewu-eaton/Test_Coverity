// ********************************************************************************************************
// *            Alarms.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Alarms.c
// *
// *    DESCRIPTION: Contains monitoring of aux voltages, temperatures, etc.
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *    DATE: 10/8/2003
// *
// *    HISTORY: See CVS history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "RectifierStateControl.h"
#include "InverterControl.h"
#include "Meters.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "F28335Port.h"
#include "Eeprom_Map.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include "Thermal.h"
#include "IOexpansion.h"
#include "Alarms.h"
#include "ACMeter.h"
#include "VirtualNeutralMeter.h"
#include "InternalCan.h"

#include <algorithm>
#include <cmath>

using namespace std;

#define EPO_DELAY_100MS     ( (uint16_t)255 )		// Pan/20120411 add
// *********************************************************************************************************
// *        Global Variables
// *********************************************************************************************************
uint16_t MaxInverterHWCLCycles;       // EE Var
uint16_t MaxRectifierHWCLCycles;      // EE Var
uint16_t MaxBatteryHWCLCycles;        // EE var    

//debugger variables
/*
uint16_t MaxInverterCLCount = 0;
uint16_t MaxRectifierCLCount = 0;
uint16_t MaxBatteryCLCount = 0;

uint16_t InvTempWarningRT = 0;
uint16_t RecTempWarningRT = 0;
uint16_t BatTempWarningRT = 0;

uint16_t InvTempTripRT = 0;
uint16_t RecTempTripRT = 0;
uint16_t BatTempTripRT = 0;
*/
// *********************************************************************************************************
// *        Local Variables
// *********************************************************************************************************
uint16_t InverterCLCount;
uint16_t RectifierCLCount;
uint16_t BatteryCLCount;
uint16_t BalanceCLCount;
uint16_t EPODelayCount;
uint16_t InverterCLResult;
uint16_t RectifierCLResult;

bool AdaptOverloadCapacityOff = false;

float AbsDCOVSet;                   // EE var
float AbsDCUVSet;                   // EE var
float RelDCUVSet;				
float ESSAbsDCUVSet;

// Tolerate at least MIN_LPS_PER_ERROR% error in logic levels
const uint16_t MIN_LPS_PER_ERROR = 5;
float LogicPower5vLimit  = (float)MIN_LPS_PER_ERROR * 5.00  / 100.00;      // EE var
float LogicPower12vLimit = (float)MIN_LPS_PER_ERROR * 12.00 / 100.00;      // EE var 
float LogicPower15vLimit = (float)MIN_LPS_PER_ERROR * 15.00 / 100.00;      // EE var
bool AteMaskAlarm5VforRsEep = false;

void ee_calc_protection( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_AbsDCOVSet:       // AbsDCOVSet
            AbsDCOVSet = (float)(*data);
            break;
            
        case PARAM_AbsDCUVSet:
            AbsDCUVSet = (float)(*data);
            break;
            
        case PARAM_Battery_Current_Limit_Set:
            BatteryConverter.SetHWCurrentLimit( *data );
            BattHWCurrentLimit = *data ;
            break;            

		case PARAM_RelDCUVSet:
			RelDCUVSet = (float)(*data);
			break;
			
        default:
            break;
    }            
}

// ********************************************************************************************************
// *
// * Function    : ee_calc_logicpower_limits
// *
// * Purpose     : Get logic power alarm level in percentage and convert to voltage level
// *
// * Description : Called from ProcessA2D_Intr (2 * 196us = 392us)
// *
// ****************************************************************************
void ee_calc_logicpower_limits( const EE_ID* ee, const uint16_t* data)
{
    switch ( ee->paramNum )
    {
        case PARAM_LogicPower5v:
            {
                uint16_t bounded_data = std::max( MIN_LPS_PER_ERROR, *data );
                LogicPower5vLimit = (float)(bounded_data) * 5.00 / 100.00;
            }
            break;
            
        case PARAM_LogicPower15v:
            {
                uint16_t bounded_data = std::max( MIN_LPS_PER_ERROR, *data );
                LogicPower12vLimit = (float)(bounded_data) * 12.00 / 100.00;
                LogicPower15vLimit = (float)(bounded_data) * 15.00 / 100.00;
            }
            break;
            
        default:
            break;
    }
}

// ****************************************************************************
// *
// * Function: InitAlarms(void);
// *
// * Purpose: Initialize alarms
// *
// ****************************************************************************
void InitAlarms( void )
{
}

// ****************************************************************************
// *
// * Function: FastAuxPowerShutdown(void);
// *
// * Purpose: Shutdown everything needed for aux power alarms
// *
// ****************************************************************************
void FastAuxPowerShutdown( void )
{
    // Immediate shutdown 
    BatteryConverter.BoostOff();
    BatteryConverter.ChargerOff();
    Rectifier.RectifierOff();
    Inverter.Off();
    PreChargeOff();
}

// ********************************************************************************************************
// *
// * Function: CheckAlarmsFast(void);
// *
// * Purpose: Poll aux alarms
// *
// * Description: Called from ProcessA2D_Intr (2*196us = 392us)
// *
// ****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void CheckAlarmsFast( void )
{
    CheckEpoAlarm();
    uint16_t errorData = 0;
    int16_t errorData1 = 0;
    int16_t errorData2 = 0;
    const uint16_t DCOVTime = 510; //200ms/392us = 510
    static int16_t DCOVcounter = 0;
    static int16_t DCOVcounterFastFilt = 0;	//3time
	const float DCLinkMarginESS = 10.0f;
	//jira wombat318,chang for bus uv protect, as 93p 
	float DCUVNBLevel = (Rectifier.GetDCLinkVoltageRef() - RelDCUVSet) * 0.5f;	//700V-120V,			

    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        rectifier_states_t rState = Rectifier.GetState();
        
        // Rail overvoltage monitoring
        if ( ( RawAdcDataPtr->st.RailVoltagePositive > AbsDCOVSet ) || ( -RawAdcDataPtr->st.RailVoltageNegative > AbsDCOVSet ) )
        {
            errorData1 = int16_t(RawAdcDataPtr->st.RailVoltagePositive - 250.0f);
            errorData2 = int16_t(-RawAdcDataPtr->st.RailVoltageNegative - 250.0f);
            if(errorData1 < 0)
            {
            	errorData1 = 0;
            }
            else if(errorData1 > 254)//255 is the max value for 8bit type data
            {
            	errorData1 = 254;
            }
			if(errorData2 < 0)
			{
				errorData2 = 0;
			}
			else if(errorData2 > 254)
			{
				errorData2 = 254;
			}           
            errorData = (uint16_t( errorData1 ) << 8 ) +
                        uint16_t( errorData2 ); //AbsDCUVSet = 250V default
            if( ( RawAdcDataPtr->st.RailVoltagePositive > (AbsDCOVSet + 10.0f) ) || ( -RawAdcDataPtr->st.RailVoltageNegative > (AbsDCOVSet + 10.0f) ) ) //default 450V
			{
				if((++DCOVcounterFastFilt) >= 3)
				{
					NB_DebounceAndQue( UPM_NB_DC_LINK_OVER_VOLTAGE, true, errorData);
				}
			}

			if( ++DCOVcounter > DCOVTime )
			{
				NB_SetNodebit(UPM_NB_DC_LINK_OVER_VOLTAGE,true, errorData);
				DCOVcounter = 0;
			}
        }
        else
        {
            //APACTS-35: discharge all DC voltage to avoid others transient fault condition lead DC OV again
            if( ( RawAdcDataPtr->st.RailVoltagePositive < DCUVNBLevel ) &&
                ( -RawAdcDataPtr->st.RailVoltageNegative < DCUVNBLevel )  )
            {
                NB_DebounceAndQue( UPM_NB_DC_LINK_OVER_VOLTAGE, false );
            }

            if( DCOVcounter )
			{
				--DCOVcounter;
			}

            if( DCOVcounterFastFilt )
			{
				--DCOVcounterFastFilt;
			}			
        }
        
//	        // Rail undervoltage monitoring
//	        if ( ( rState != RECTIFIER_INIT_STATE        &&
//	               rState != RECTIFIER_SHUTDOWN_STATE    &&
//	               rState != RECTIFIER_PRECHARGE_STATE ) &&
//				   !BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging &&
//	             ( ( DCLinkVoltagePositive.FastFiltered < AbsDCUVSet ) || ( -DCLinkVoltageNegative.FastFiltered < AbsDCUVSet ) ) )
//	        {
//	            NB_DebounceAndQue( UPM_NB_DC_LINK_UNDER_VOLTAGE, true );
//	        }
//	        else
//	        {
//	            NB_DebounceAndQue( UPM_NB_DC_LINK_UNDER_VOLTAGE, false );
//	        }

			// Rail undervoltage monitoring
//				float DCUVNBLevel = AbsDCUVSet; 	//1. not ess mode, use eep_dcuv 
			
			if ( (rState == RECTIFIER_SUSPEND_STATE) || (rState == RECTIFIER_FROM_SUSPEND_STATE) )
			{				
				// set the DC UV limit for ESS mode: peak to peak lower limit minus voltage drop, especially for NLL
				ESSAbsDCUVSet = float(OutNomVolts) * 0.240414f - 15.0f ; // 0.240414 = 0.1 * 1.4142 * 2 * 0.85
				//use 545min,when Vnom=220V, mint=220*1.414*0.85*2-15 =514.
				if ( ESSAbsDCUVSet < 514.0f )
				{
					ESSAbsDCUVSet =  514.0f;  
				}  
				
				//when ess, Vbus=Vac_max, Vuv=Vac_max+15-60-10=..-55
				DCUVNBLevel = (ESSAbsDCUVSet-DCLinkMarginESS)*0.5f;		//2. ess mode, use eep_dcuv		
			}
	
			// Rail undervoltage monitoring
			if ( ( rState != RECTIFIER_INIT_STATE		 &&
				   rState != RECTIFIER_SHUTDOWN_STATE	 &&
				   rState != RECTIFIER_PRECHARGE_STATE ) &&
				 !BatteryConverter.BatteryStatus.bit.BatteryStartPrecharging &&
	//				 ( ( DCLinkVoltagePositive.FastFiltered < AbsDCUVSet ) || ( -DCLinkVoltageNegative.FastFiltered < AbsDCUVSet ) ) )
				 ( ( DCLinkVoltagePositive.FastFiltered < DCUVNBLevel ) || ( -DCLinkVoltageNegative.FastFiltered < DCUVNBLevel ) ) )
			{					
				//(2.25k)debounce:4-active;  5102-deactive			  
				if(NB_DebounceAndQue( UPM_NB_DC_LINK_UNDER_VOLTAGE, true ))
				{
					FastAuxPowerShutdown();
				}
			}
			else
			{
				NB_DebounceAndQue( UPM_NB_DC_LINK_UNDER_VOLTAGE, false );
			}


       
        // +5v logic power supply monitoring
        if( ( fabs( RawAdcDataPtr->st.LogicPower5v - 5.00 ) > LogicPower5vLimit ) )
        {
            if(AteMaskAlarm5VforRsEep == false)
            {
                NB_DebounceAndQue( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT, true );
            }
            else
            {
                NB_DebounceAndQue( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT, false );
            }
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT, false );            
        }
        
        // +12v logic power supply and driver fault monitoring
        if( ( fabs( RawAdcDataPtr->st.LogicPower15v - 15.00 ) > LogicPower15vLimit ) )
        {
            NB_DebounceAndQue( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT, true );
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT, false );
        }
            
//	        if ( NB_GetNodebit( UPM_NB_DRIVER_FAULT ) 			||
//				 NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )	||
//				 NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE ) )
		if(isFastAuxPowerShutdown())
        {
            FastAuxPowerShutdown(); 
        }

	
    } 

}// End of CheckAlarmsFast

// ********************************************************************************************************
// *
// * Function: CheckAlarms20ms(void);
// *
// * Purpose: Poll 20ms alarms
// *
// * Description: Called from 20ms periodic task
// *
// ****************************************************************************
void CheckAlarms20ms( void )
{
    static bool check_enable = false;
    static uint16_t check_time = 0;
    
    // Zhangjun : before check of MOB_FAILURE, we should ensure 
    // (1) CSB has sent BldInput bits to UPM and (2) Meters is ready
    // (3) Delay 1 second and make sure OUTPUT ACUV is logged if active 
    if( !check_enable )
    {
        if( InternalCan.CanHardwareReady() && MCUStateMachine.GetMetersReadyFlag() )
        {
            if( ++check_time >= 50 )
            {
                check_time = 0;
                check_enable = true;
            }
        }        
    }
    else
    {
        // MOB failure alarm check
        if( NB_GetNodebit( UPM_NB_MOB_OPEN ) )
        {
            if( ( !MCUStateMachine.SupportingLoad() && !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) ) ||
                ( ScreenMeters.PercentLoad.sum > 25.0f ) )
            {
                if( ++check_time >= 150 )
                {
                    check_time = 0;    
                    NB_SetNodebit( UPM_NB_MOB_FAILURE, true );
                }
            }
            else
            {
                check_time = 0; 
            }    
        }    
    }
}// End of CheckAlarms20ms

// ********************************************************************************************************
// *
// * Function: CheckTemperatureAlarms(void);
// *
// * Purpose: Poll slow alarms
// *
// * Description: Called from 100ms periodic task
// *
// ****************************************************************************
#define AmbientTempWarningLimitHyst 2.0
void CheckTemperatureAlarms( void )
{
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        if( (UPMSystem == HV_20K)||(UPMSystem == HV_30K) ||(UPMSystem == HV_40K))
        {
            uint16_t over_thermal = 0;
            int16_t inv_margin = 0;
//			int16_t Rec_margin = 0;
            int16_t Bat_margin = 0;
//			int16_t Sts_margin = 0;
            int16_t Scr_margin = 0;

            //fan ok trip temperature
            //20k: 
            //	   rec         105
            //     inv         107
            //     battery      78
            //30k: 
            //     rec          104
            //     inv          104
            //     battery      95            
            //40k: 
            //     rec          101
            //     inv          103
            //     battery      101                 
            //fan fail trip temperature
            //20k:
            //	   rec         105 - 25
            //     inv         107 - 27
            //     battery      78 - 8
            //30k: 
            //     rec          104 - 25
            //     inv          104 - 25
            //     battery      95 -  25           
            //40k: 
            //     rec          101 - 21
            //     inv          103 - 23
            //     battery      101 - 21
        	if(NB_GetNodebit(UPM_NB_FAN_FAILURE))
    		{
				inv_margin = 0 - FanFailTempPrtDec;		//EEP243 
//				Rec_margin = 0 - RecFanFailTempPrtDec;  //EEP251
				Bat_margin = 0 - BatFanFailTempPrtDec;  //EEP252      		
    		}
			else
			{
				inv_margin = 0;//EEP(220),EEP(219)
//				Rec_margin = 0;//EEP(220),EEP(219)
				Bat_margin = 0;//EEP(220),
			}    
//				// ambinent temperature warning
//				if (AMB_Thermal_C >= ((float)AmbientTempWarningLimit))
//	            {
//	                NB_DebounceAndQue(UPM_NB_AMBIENT_OVERTEMPERATURE, true, 0);
//	            }
//	            else if (NB_GetNodebit(UPM_NB_AMBIENT_OVERTEMPERATURE))
//	            {
//	                if (AMB_Thermal_C < ((float)(AmbientTempWarningLimit) - AmbientTempWarningLimitHyst))
//	                {
//	                    NB_DebounceAndQue(UPM_NB_AMBIENT_OVERTEMPERATURE, false, 0);
//	                }
//	            }
            
	   	// over temperature warning     	
			//Hobbit new thermal request:
			//	For Rec_ST reuse to Bat_LegB, so in bat mode: Rec_ST use bat otp 		
//            static uint16_t LinetobatWarnDelay = 0;			//1min
//            static uint16_t LinetobatFaultDelay = 0;		//1min
			
//			if( BatteryConverter.GetStatus().bit.BoostOn && Rectifier.ReuseRectSTPWMEnable)
//			{
//				//1)bat mode
//				if(++LinetobatWarnDelay >= 600)		//delay 1min then use bat otp threshold,600=1min/100ms
//				{
//					LinetobatWarnDelay = 600;
//					if ( PHASEB_Thermal_C >= (BatHeatsinkTempWarningLimit + Bat_margin ) )		//EEP(249)
//					{
//						over_thermal |= ((uint16_t)1 << 4);		//err_code=1<<4, mean bat_st otp
//					}
//				}
//				else		//during 1min, still keep rec otp
//				{
//					if ( PHASEB_Thermal_C >= ( PhaseBHeatsinkTempWarningLimit + Rec_margin ) )
//					{
//						over_thermal |= (uint16_t)1;
//					}
//				}
//			}
//			else
//			{
//				//2)line mode
//				LinetobatWarnDelay = 0;
//				if ( PHASEB_Thermal_C >= ( PhaseBHeatsinkTempWarningLimit + Rec_margin ) )	//EEP(247)
//				{
//					over_thermal |= (uint16_t)1;
//				}
//			}

			if ( PHASEA_Thermal_C >= ( PhaseAHeatsinkTempWarningLimit + inv_margin ) )	//(219)INV over temp warning
			{
				over_thermal |= ((uint16_t)1 << 1);
			}

			if ( PHASEB_Thermal_C >= ( PhaseAHeatsinkTempWarningLimit + inv_margin ) )	//(219)INV over temp warning
			{
				over_thermal |= ((uint16_t)1 << 2);
			}

			if ( PHASEC_Thermal_C >= ( PhaseAHeatsinkTempWarningLimit + inv_margin ) )	//(219)INV over temp warning
			{
				over_thermal |= ((uint16_t)1 << 3);
			}

			if ( BAT_Thermal_C >= (BatHeatsinkTempWarningLimit + Bat_margin ) )		//EEP(249)
			{
				over_thermal |= ((uint16_t)1 << 4);
			}

            NB_DebounceAndQue( UPM_NB_PM_OVERTEMPERATURE, over_thermal, over_thermal );

	        // SCR over temperature warning
			over_thermal = 0;

            if ( SCR_Thermal_C >= (SCRHeatsinkTempWarningLimit + Scr_margin ) )
            {
                over_thermal |= (uint16_t)1;
            }

            NB_DebounceAndQue( UPM_NB_STS_OVER_TEMPERATURE, over_thermal, over_thermal );



		// over temperature trip
			over_thermal = 0;

			//Hobbit new thermal request:
			//	For Rec_ST reuse to Bat_LegB, so in bat mode: Rec_ST use bat otp 			
//			if( BatteryConverter.GetStatus().bit.BoostOn && Rectifier.ReuseRectSTPWMEnable)
//			{
//				//1.bat mode
//				if(++LinetobatFaultDelay >= 600)		//600=1min/100ms
//				{
//					LinetobatFaultDelay = 600;
//					if ( PHASEB_Thermal_C >= BatHeatsinkTempTripLimit + Bat_margin)  //EEP(250)
//					{
//						over_thermal |= ((uint16_t)1 << 4);
//					}
//				}
//				else
//				{
//					if ( PHASEB_Thermal_C >= ( PhaseBHeatsinkTempTripLimit + Rec_margin ) )  //EEP(248)
//					{
//						over_thermal |= (uint16_t)1;
//					}
//				}
//			}
//			else
//			{
//				//2.line mode
//				LinetobatFaultDelay = 0;
//				if ( PHASEB_Thermal_C >= ( PhaseBHeatsinkTempTripLimit + Rec_margin ) )  //EEP(248)
//				{
//					over_thermal |= (uint16_t)1;
//				}
//			}


			if ( PHASEA_Thermal_C >= ( PhaseAHeatsinkTempTripLimit + inv_margin ) )	//EEP(220) INV over temp trip
			{
				over_thermal |= ((uint16_t)1 << 1);
			}

			if ( PHASEB_Thermal_C >= ( PhaseAHeatsinkTempTripLimit + inv_margin ) )
			{
				over_thermal |= ((uint16_t)1 << 2);
			}

			if ( PHASEC_Thermal_C >= ( PhaseAHeatsinkTempTripLimit + inv_margin ) )	//EEP(220) INV over temp trip
			{
				over_thermal |= ((uint16_t)1 << 3);
			}
			//InvTempTripRT = HeatsinkTempTripLimit + inv_margin;

			if ( BAT_Thermal_C >= BatHeatsinkTempTripLimit + Bat_margin)  //EEP(250)
			{
				over_thermal |= ((uint16_t)1 << 4);
			}

			NB_DebounceAndQue( UPM_NB_PM_OVERTEMPERATURE_TRIP, over_thermal, over_thermal );

        }
        else
        {

        }
    }    
}// End of CheckTemperatureAlarms

// ***********************************************************************
// *
// *    FUNCTION:  CheckFuseFailures
// *
// *    DESCRIPTION: Check for fuse failures and set the appropriate nodebit
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void CheckFuseFailures( void )
{
    uint16_t fuses_blown = 0x0000;
    static StateTimer FuseFailRestartResetTimer;
    
    if ( MCUStateMachine.GetMetersReadyFlag() &&
         Rectifier.GetStatus().bit.RectifierOnNormal )
    {
        float curr_sum = 0;
        
        curr_sum = UtilityCurrentRMS.FilteredRMS.phA + \
                   UtilityCurrentRMS.FilteredRMS.phB + \
                   UtilityCurrentRMS.FilteredRMS.phC;

        // require 1/4 of max current before checking. Max current is per phase: 3*FuseFailLoadPercentLimit/100
        float min_curr = Rectifier.GetMaxRMSCurrent() * FuseFailLoadPercentLimit * 3 / 100.0f;
        if ( curr_sum > min_curr )
        {
            // fail limit is 7% max
            float fail_cur = Rectifier.GetMaxRMSCurrent() * 0.07f;

            if ( UtilityCurrentRMS.FilteredRMS.phA < fail_cur )
            {
                fuses_blown |= FUSE_RECT_PH_A;
            }
            if ( UtilityCurrentRMS.FilteredRMS.phB < fail_cur )
            {
                fuses_blown |= FUSE_RECT_PH_B;
            }
            if ( UtilityCurrentRMS.FilteredRMS.phC < fail_cur )
            {
                fuses_blown |= FUSE_RECT_PH_C;
            }
        }
    }
    
    //fix EXS-71, 1 times restart allowed within 1hour
    if( fuses_blown && !NB_GetNodebit( UPM_NB_FUSE_FAILURE ) )
	{
		if( Rectifier.FuseFailRestartTimes < FuseFailMaxRestartTimes  )
    	{
			Rectifier.FuseFailRestart = true;
    	}
		else
		{
			NB_DebounceAndQue( UPM_NB_FUSE_FAILURE, fuses_blown, fuses_blown );
		}
	}

	if( !NB_GetNodebit( UPM_NB_FUSE_FAILURE ) )
	{
	    if( Rectifier.FuseFailRestartTimes )
	    {
	    	if ( FuseFailRestartResetTimer.CheckTimeout( FUSE_WAIT_1_HOUR ) )
			{
	    		Rectifier.FuseFailRestartTimes = 0;
	    		FuseFailRestartResetTimer.ClearTimer();
			}
	    }
	}
	else
	{
	    Rectifier.FuseFailRestartTimes = 0;
	    FuseFailRestartResetTimer.ClearTimer();
	}
    // Battery Fuses
    
    // Inverter Fuses
    

}// End of CheckFuseFailures

// ***********************************************************************
// *
// *    FUNCTION:  
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void CheckCurrentLimits( void )
{
    uInverterStatus iStat;
    static uint16_t FlagbatLegA_CL = 0;
    static uint16_t FlagbatLegB_CL = 0;
	
    iStat = Inverter.GetStatus();
    
    // inverter, TZ1, TZ2 and TZ3
    if ( MCUStateMachine.GetMetersReadyFlag() && iStat.bit.InverterOn )
    {
        if ( EPwm1Regs.TZFLG.bit.OST ||
        	 EPwm2Regs.TZFLG.bit.OST ||
        	 EPwm3Regs.TZFLG.bit.OST )
        {
            InverterCLCount++;                          // increment count

            if( EPwm1Regs.TZFLG.bit.OST )
            {
            	InverterCLResult |= (uint16_t)1;
            }                      // increment count

            if( EPwm2Regs.TZFLG.bit.OST )
            {
            	InverterCLResult |= ((uint16_t)1 << 1);
            }                      // increment count

            if( EPwm3Regs.TZFLG.bit.OST )
            {
            	InverterCLResult |= ((uint16_t)1 << 2);
            }

            if ( InverterCLCount > MaxInverterHWCLCycles )
            {
                InverterCLCount = MaxInverterHWCLCycles;
                Inverter.Off();                 // just shutdown the inverter.
                NB_DebounceAndQue( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT, true, InverterCLResult );
//					FastAuxPowerShutdown();				
            }            
        }
        else
        {
            if ( InverterCLCount )
            {
                --InverterCLCount;
            }
            else
            {
                NB_DebounceAndQue( UPM_NB_INVERTER_OUTPUT_OVER_CURRENT, false );
                InverterCLResult = 0;
            }
        }
    }
    else
    {
        InverterCLCount = 0;
        InverterCLResult = 0;
    }    
    
    // clear flags
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;
        
    // rectifier, TZ4, TZ5, TZ6
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        //Update for HV system to activate alarm UPM_NB_BALANCER_LOOP_OVER_CURRENT when UPS is on battery mode
        if ( EPwm4Regs.TZFLG.bit.OST ||
             EPwm5Regs.TZFLG.bit.OST ||
             EPwm6Regs.TZFLG.bit.OST )
        {
            rectifier_states_t RectState = Rectifier.GetState();
            
//	            if ( ( ( RectState == RECTIFIER_TO_BATTERY_STATE ) ||
//	               ( RectState == RECTIFIER_ON_BATTERY_STATE ) ) )
			if (( RectState == RECTIFIER_TO_BATTERY_STATE ) ||
			    ( RectState == RECTIFIER_ON_BATTERY_STATE ) ||
			    ( RectState == RECTIFIER_SUSPEND_STATE )    ||
			    ( RectState == RECTIFIER_FROM_SUSPEND_STATE) )
            {
                BalanceCLCount++;                         // increment count
                if ( BalanceCLCount > MaxBatteryHWCLCycles )
                {
                    BalanceCLCount = MaxBatteryHWCLCycles;
					FlagbatLegB_CL = true;
                } 
            }
            else
            {
                RectifierCLCount++;                         // increment count

                if( EPwm4Regs.TZFLG.bit.OST )
                {
                    RectifierCLResult |= (uint16_t)1;
                }                      // increment count

                if( EPwm5Regs.TZFLG.bit.OST )
                {
                	RectifierCLResult |= ((uint16_t)1 << 1);
                }                      // increment count

                if( EPwm6Regs.TZFLG.bit.OST )
                {
                	RectifierCLResult |= ((uint16_t)1 << 2);
                }

                if ( RectifierCLCount > MaxRectifierHWCLCycles )
                {
                    RectifierCLCount = MaxRectifierHWCLCycles;
                    NB_DebounceAndQue( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT, true, RectifierCLResult );					
//						FastAuxPowerShutdown();
                }            
            }
    
        }     
        else
        {
            if ( RectifierCLCount )
            {
                --RectifierCLCount;
            }
            else
            {
                NB_DebounceAndQue( UPM_NB_RECTIFIER_INPUT_OVER_CURRENT, false );
                RectifierCLResult = 0;
            }
            
            if ( BalanceCLCount )
            {
                --BalanceCLCount;
            }
            else
            {
//	                NB_DebounceAndQue( UPM_NB_BALANCER_LOOP_OVER_CURRENT, false );
				FlagbatLegB_CL = false;

            }
        }
    }
    else
    {
        RectifierCLCount = 0;    
        BalanceCLCount = 0;
        RectifierCLResult = 0;
    }    
    // clear flags
    EALLOW;
    EPwm4Regs.TZCLR.all = 7;
    EPwm5Regs.TZCLR.all = 7;
    EPwm6Regs.TZCLR.all = 7;
    EDIS;
    

    
    // battery converter latched from PLD 
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {

        if ( !DSPInRegister.GpiC.bit.BatteryCurrentLimit ) 
        {
            DSPOutRegister.GpoB.bit.BattCLimReset = 1;
            WriteDSPOutputs_TSK();
            
            // Allow the bit to be cleared on the next iteration of the ADC ISR
           
            BatteryCLCount++;                         // increment count
            if ( BatteryCLCount > MaxBatteryHWCLCycles )
            {
                BatteryCLCount = MaxBatteryHWCLCycles;
				FlagbatLegA_CL = true;
            }    
        }     
        else
        {
            if ( BatteryCLCount )
            {
                --BatteryCLCount;
            }
            else
            {
				FlagbatLegA_CL = false;
            }
        }
    }
    else
    {
        BatteryCLCount = 0;    
    }

	NB_DebounceAndQue( UPM_NB_BATTERY_CURRENT_LIMIT, FlagbatLegA_CL||FlagbatLegB_CL, 
					(FlagbatLegA_CL + (FlagbatLegB_CL<<1)));
}

// ***********************************************************************
// *
// *    FUNCTION:  
// *
// *    DESCRIPTION:
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryCLReset( void ) 
{
    if( DSPOutRegister.GpoB.bit.BattCLimReset )
    {
        //if there isn't an active current limit then turn off the reset
        if( DSPInRegister.GpiC.bit.BatteryCurrentLimit )
        {
            DSPOutRegister.GpoB.bit.BattCLimReset = 0;
        }
    }
}


// ***********************************************************************
// *
// *    FUNCTION:  
// *
// *    DESCRIPTION:
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void CheckEpoAlarm( void ) 
{
   if( ( EEStatusBits.bit.EEDataInitialized == 1 ) )
   {
       if( EPOEnabled == 1 )
       {            
            // Enable hardware EPO
            DSPOutRegister.GpoC.bit.EPO_Enable = 0;

            // Check EPO alarms
            if ( DSPInRegister.GpiA.bit.PLD_EPO_Latch )
            {
            	//fix APACTS-591 Input break open at bypass,then ups load off issue
            	if( EEEPODelay > 200 )
				{
					EEEPODelay = 200; //max delay 20s
				}

                // Pan/20130401 add for P9EPIEB-10
                // For LV system, when utility is off and 12V drops, EPO may be triggered accidently
                // Add 100ms delay before check EPO signal in this situation
                if( EPODelayCount++ >= EPO_DELAY_100MS * EEEPODelay )
                {
                    EPODelayCount = EPO_DELAY_100MS * EEEPODelay;
                    NB_DebounceAndQue( UPM_NB_REMOTE_EMERGENCY_POWER_OFF, true );
                }
            }
            else
            {
               NB_DebounceAndQue( UPM_NB_REMOTE_EMERGENCY_POWER_OFF, false );
               EPODelayCount = 0;
            }
       }
       else
       {
           // Disable hardware EPO
           DSPOutRegister.GpoC.bit.EPO_Enable = 1;

           // Clear EPO alarms
           NB_DebounceAndQue( UPM_NB_REMOTE_EMERGENCY_POWER_OFF, false );
           EPODelayCount = 0;
       }
   }
   else
   {
       // Disable hardware EPO
       DSPOutRegister.GpoC.bit.EPO_Enable = 1;
        
       EPODelayCount = 0;
   }                
}

// ***********************************************************************
// *
// *    FUNCTION:  void CheckSiteWiring( void)
// *
// *    DESCRIPTION: Check the status of site wiring fault, and drive its nodebit.
// *                Called from the 5ms TSK, it must be invoked with regular
// *                periodicity since filtering happens inside here.
// *
// *    ARGUMENTS: None
// *
// *    RETURNS: Nothing
// *
// ***********************************************************************
void CheckSiteWiring( void)
{
	bool neutralFault = false;
	static bool groundFault = false;
	static FilteredBit SiteWiringFault( PASS_4, PASS_200 );

	if ( !DisableSiteWiringFault				&&
		 MCUStateMachine.GetMetersReadyFlag() )
	{
//			if( Rectifier.IsNeutralFault() )	//HW: the N-phase(VGND) sample circle not correct
//			{
//				neutralFault = true;
//			}
		
		//Chassic adc:hw use for N-phase;  phA, phB, phC are the same value
//			if( ChassisVoltageRMS.FilteredRMS.phA >= SiteWiringFaultHighLimit )
		if( ChassisVoltageRMS.RawRMS.phA >= SiteWiringFaultHighLimit )
		{
			groundFault = true;
		}
		else if( ChassisVoltageRMS.RawRMS.phA < ( 0.66f*SiteWiringFaultHighLimit ) )
		{
			groundFault = false;
		}
//			NB_DebounceAndQue( UPM_NB_SITE_WIRING_FAULT, neutralFault || groundFault, neutralFault | ( groundFault << 1 ) );
		NB_DebounceAndQue( UPM_NB_SITE_WIRING_FAULT, groundFault, groundFault );
	}
	else
	{
		NB_DebounceAndQue( UPM_NB_SITE_WIRING_FAULT, false );
	}
}

//	{
//		static FilteredBit InputNeutral(UPM_NB_SITE_WIRING_FAULT);
//		static FilteredBit BypassNeutral(UPM_NB_SITE_WIRING_FAULT);
//		
//	    if ( MCUStateMachine.GetMetersReadyFlag() )
//	    {
//	        // Check thresholds
//	        const float activeThreshold = SiteWiringFaultHighLimit;
//	        // const float clearThreshold = 0.50f * activeThreshold;  
//		    float clearThreshold;    // Keming/20120620,add
//	        
//	        float neutralA = std::abs(VirtualInputNeutralA.FilteredNeutral);
//	        float neutralB = std::abs(VirtualInputNeutralB.FilteredNeutral);
//	        float neutralC = std::abs(VirtualInputNeutralC.FilteredNeutral);
//	        float neutralInput = DirectInputNeutral.FilteredNeutral;
//	        float neutralBypass = DirectBypassNeutral.FilteredNeutral;
//	        float neutralSum = DirectInputVoltSum.FilteredNeutral;
//	
//	        // Detection method relies on the input neutral being tied to
//	        // the output neutral.  This condition is met only when the neutral
//	        // relay is closed, or the internal maintenance bypass is not installed.
//	        // Additionally, when we are on battery or in the process of transferring
//	        // to battery, we can observe false positives.  Note that this is only
//	        // true for evaluating the 3-phase sum relative to VGND.  The input and
//	        // bypass VGND and VGND2 sensing are relative to Earth, and those work
//	        // all of the time.
//			clearThreshold =  0.75 * activeThreshold;
//			neutralBypass = 0;  // there is no bypass neutral in hardware for HV system.
//	
//	        rectifier_states_t rectifierState = Rectifier.GetState();
//	        bool observable = 
//	            ( !NB_GetNodebit(UPM_NB_INTERNAL_MBS_INSTALLED) ||
//	                DSPOutRegister.GpoA.bit.NeutralRelay == 1)        &&
//	            ( rectifierState == RECTIFIER_SHUTDOWN_STATE          ||
//	                rectifierState == RECTIFIER_PRECHARGE_STATE       ||
//	                rectifierState == RECTIFIER_SUSPEND_STATE         ||
//	                rectifierState == RECTIFIER_NORMAL_STATE          ||
//	                rectifierState == RECTIFIER_WALKIN_STATE          ||
//	                rectifierState == RECTIFIER_ON_BATTERY_STATE);
//	        
//	        uint16_t active = 0;
//	        //site wiring fault is disabled on revision 5 hardware
//	        if (!DisableSiteWiringFault &&
//	              ((ExpansionInputReg.bit.RevID != Rev5) ||
//	              DisableRevisionCheck))
//	        {
//	            active = ( ( observable && 
//	                         ( rectifierState != RECTIFIER_ON_BATTERY_STATE ) &&
//	                         !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) &&
//	                        ( ( neutralA > activeThreshold &&
//	                            neutralB > activeThreshold &&
//	                            neutralC > activeThreshold ) ||
//	                          (neutralSum > activeThreshold) ) ) ? 1 << 0 : 0)      |
//	                     ( (neutralInput > activeThreshold) ? 1 << 1 : 0)   |
//	                     ( (neutralBypass > activeThreshold) ? 1 << 2 : 0);
//	        }
//	        
//	        bool clear = observable &&
//	            neutralA < clearThreshold &&
//	            neutralB < clearThreshold &&
//	            neutralC < clearThreshold &&
//	            neutralInput < clearThreshold &&
//	            neutralBypass < clearThreshold &&
//	            (neutralSum < clearThreshold);
//	        
//	        NB_DebounceAndQue_Hysterisis(UPM_NB_SITE_WIRING_FAULT, active != 0, clear, active);
//	    }
//	}

// ***********************************************************************
// *
// *    FUNCTION:  void CheckAdapOverloadCapacity( void)
// *
// *    DESCRIPTION: Check the status of adaptive overload capacity, and drive its nodebit.
// *
// *    ARGUMENTS: None
// *
// *    RETURNS: Nothing
// *
// ***********************************************************************
void CheckAdapOverloadCapacity( void)
{
    uint16_t AdapOverloadOffConditions = 0;
    static bool HighAmbTemperature = false;
    if (AMB_Thermal_C > AolAmbTempLimit)
    {
        HighAmbTemperature = true;
    }
    else if (AMB_Thermal_C < (AolAmbTempLimit - AolAmbTempHyst_x10 * 0.1f))
    {
        HighAmbTemperature = false;
    }
    else
    {
        //do nothing
    }

    AdapOverloadOffConditions = /*NB_GetNodebit( UPM_NB_INTERNAL_CAN_ERROR )            * 0x0001 +   // internal CAN failed*/ 
                                HighAmbTemperature                                    * 0x0002 +   // high AMB temperature
                                NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED )        * 0x0004 +   // Battery disconnected
                                ( BatteryConverter.GetBatteryState() == BATTERY_ON_BOOST_STATE &&
                                ( Rectifier.GetState() != RECTIFIER_NORMAL_STATE ) )  * 0x0008 +   // UPM on battery mode (not power share).
                                NB_GetNodebit(UPM_NB_FAN_FAILURE)                     * 0x0010 ;   // Fan failed

    AdaptOverloadCapacityOff = (0 == AdapOverloadOffConditions)? false: true;
    NB_DebounceAndQue ( UPM_NB_ADAPTIVE_OVERLOAD_CAPACITY_OFF, AdapOverloadOffConditions, AdapOverloadOffConditions);
}


// ****************************************************************************
// *
// * Function: isFastAuxPowerShutdown(void);
// *
// * Purpose: if need FastAuxPowerShutdown, return true-yes; false-no
// *
// ****************************************************************************
bool isFastAuxPowerShutdown( void )
{
    //check if Immediate shutdown 
	if ( NB_GetNodebit( UPM_NB_DRIVER_FAULT )			||
		 NB_GetNodebit( UPM_NB_DC_LINK_UNDER_VOLTAGE )	||
		 NB_GetNodebit( UPM_NB_DC_LINK_OVER_VOLTAGE ) )
	{
		return true;
	}
	else
	{
		return false;
	}
}


// *********************************************************************************************************
// *        End of File
// ********************************************************************************************************* 

