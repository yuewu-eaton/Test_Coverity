// ********************************************************************************************************
// *            Alarms_AC.c
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
// *    FILE NAME: Alarms_AC.c
// *
// *    DESCRIPTION: Contains monitoring of mcu status bits for all ac alarms
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *    DATE: 17.12.2003
// *
// *    HISTORY: See CVS history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Eeprom_Map.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "MCUState.h"
#include "DQPhaseLockLoop.h"
#include "ProcessAdc.h"
#include "RectifierStateControl.h"
#include "Meters.h"
#include "BypassInterface.h"
#include "InverterControl.h"
#include "ACPowerMeter.h"
#include "Alarms_AC.h"
#include "Alarms.h"
#include "Eeprom_Map.h"
#include <math.h>

#define Overload_Level1_Bat_1Min            ((uint16_t)60)
#define Overload_Level2_Bat_30Sec           ((uint16_t)30000)
#define Overload_Level3_Bat_10Sec           ((uint16_t)10000)
#define Overload_Level4_Bat_150Ms           ((uint16_t)150)
#define Overload_LineLvl1_60Min_Eq_BatLvl1_10Min           ((uint16_t)6)
#define Overload_LineLvl2_10Min_Eq_BatLvl2_30Sec           ((uint16_t)20)
#define Overload_LineLvl3_60Sec_Eq_BatLvl3_10Sec           ((uint16_t)6)
#define OUTPUT_RMSOV_DELAY_200mS        ((uint16_t)10)
#define OUTPUT_RMSOV_DELAY_500mS        ((uint16_t)25)
#define ECT_MAX_ACUV_Count              ((uint16_t) 7)
// *********************************************************************************************************
// *        GLOBAL VARIABLES
// *********************************************************************************************************
uint16_t OverloadTime1;           // EE var
uint16_t OverloadTime2;           // EE var
uint16_t OverloadTime3;           // EE var
uint16_t OverloadTime4;           // EE var
uint16_t OpACUVActiveTime;    // EE var

uint16_t AdapOverloadTime1;       // EE var
uint16_t AdapOverloadTime2;       // EE var
uint16_t AdapOverloadTime3;       // EE var
uint16_t AdapOverloadTime4;       // EE var

int16_t OverloadLevel1;           // EE var    
int16_t OverloadLevel2;           // EE var

float OverloadLevel2VA_L;         // EE var
float OverloadLevel3VA_L;         // EE var
float OverloadLevel4VA_L;         // EE var

// ********************************************************************************************************
//      Local variables
// ********************************************************************************************************
bool OverloadTripSlow = false;
extern uint16_t NumOfModule;
//	uint32_t OverloadCnt1 = 0;
uint16_t OverloadCnt1 = 0;
uint16_t OverloadCnt2 = 0;
uint16_t OverloadCnt3 = 0;
uint16_t OverloadCnt4 = 0;
uint16_t OutRMSOVDelayCount = 0;
uint16_t OutputACUVECTCount = 0;    //Jacob/20130815/Merge 120K // Pan/20120108 add
uint16_t InstantVoltageState = 0;
uint16_t NegPowerCount = 0;
uint16_t NegPowerTrip = 0;
uint16_t OpAcuvTime1;
uint16_t OpAcuvTime2;
uint16_t OpAcuvTime3 = 3; //2.4ms

float NegPowerLimit;
float NegPowerLimNorm;

float UtilityMaxRMSLimit;
float UtilityMinRMSLimit;
float UtilityMaxMagnitudeLimit;
float InputFrequencyLowLimit;
float InputFrequencyHighLimit;

float BypassMaxLimitPercentage;
float BypassMinLimitPercentage;

float OutputDQLowLimit;
float OutputDQLowMinLimit;

const float VoltageRMSHysteresis = 2.5;          // 5V rms

FilteredBit BypassRMSOV( PASS_4, PASS_128, NOT_STICKY );
FilteredBit BypassRMSUV( PASS_5, PASS_128, NOT_STICKY );
FilteredBit Bypass25RMSUV( PASS_4, PASS_128, NOT_STICKY );
FilteredBit OutputRMSOV( PASS_2, PASS_3, NOT_STICKY );
FilteredBit OutputRMSUV_LV( PASS_2, PASS_3, NOT_STICKY );
FilteredBit OutputRMSUV_HV( PASS_16, PASS_3, NOT_STICKY );
FilteredBit Output25RMSUV( PASS_4, PASS_128, NOT_STICKY );
FilteredBit OutVolRMS_NotPresent( PASS_16, PASS_64, NOT_STICKY );

// ********************************************************************************************************
//      Local function prototypes
// ********************************************************************************************************
void CheckOverloadSlow( void );
void Lev1OverloadCheck( void );
void Lev2OverloadCheck( void );
void OverloadCheck( float OverloadLevel_L, float OverloadHyst_L, upm_nb_id_t overload_phA,
       upm_nb_id_t overload_phB, upm_nb_id_t overload_phC );
uint16_t AcuvCheck(float VoltageSd, float UnderLimit, uint16_t ActiveTime, uint16_t *Counter);

// ********************************************************************************************************
// *
// * Function: ee_ac_limits
// *
// * Purpose: EE function: Updates limits when bypass related ee var is read or changed
// *
// ********************************************************************************************************
void ee_ac_limits( const EE_ID* ee, const uint16_t* const data )
{
    switch ( ee->eep_addr )
    {
        case PARAM_InpACOVLevel:
            UtilityMaxRMSLimit = (float)(*data);
            
            // Fast OV +20%
            UtilityMaxMagnitudeLimit = UtilityMaxRMSLimit * SQRT_2 * 1.2;
            break;
            
        case PARAM_InpACUVLevel:
            UtilityMinRMSLimit = float( *data );
            break;
            
        case PARAM_InputFreqLowLimit:
            InputFrequencyLowLimit = float( *data );
            break;
            
        case PARAM_InputFreqHighLimit:
            InputFrequencyHighLimit = float( *data );
            break;

        case PARAM_OpACUVActiveTime:
        	OpACUVActiveTime = ( *data );
        	OpAcuvTime1 = (uint16_t)( OpACUVActiveTime * 5 / 4 );
        	OpAcuvTime2 = (uint16_t)( ( OpACUVActiveTime * 5 / 4 ) >> 1 );
			break;
			
        case PARAM_OpACUVFastLevel:
        	OutputDQLowLimit = (float)(*data) / 100.0;
			break;
			
        case PARAM_OpACUVFastMinLevel:
            OutputDQLowMinLimit = (float)(*data) / 100.0;
			break;
			
        default:
            break;              
    }
}

// ********************************************************************************************************
// *
// * Function: CheckFastACAlarms(void);
// *
// * Purpose: Checks only periodicly critical measurements
// *          called at 1.25kHz rate
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void CheckFastACAlarms(void)
{
	static uint16_t AcuvCounter1 = 0;
	static uint16_t AcuvCounter2 = 0;
	static uint16_t AcuvCounter3 = 0;
    const float OutputVoltageHysteresis = 0.025;            // 2.5%
    float OutputMaxLimit = float( OutNomVolts ) * 0.1 * BypassState().GetBypassRMSHighLimit();
    float OutputMinLimit = float( OutNomVolts ) * 0.1 * BypassState().GetBypassRMSLowLimit();
	static bool bFlagInvOff = false;
	static uint16_t wDelay = 0;

    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // fast utility OV check, only set here, not cleared
        if ( Rectifier.UtilityPLL.SourceNominalDQO.Sd > UtilityMaxMagnitudeLimit * Rectifier.UtilityPLL.GetNominalNormFactor() )
        {
            NB_DebounceAndQue( UPM_NB_INPUT_AC_OVER_VOLTAGE, true );
        }    
        
        // fast output UVOV checking, uses the same limits as bypass
        float OutputVoltageHighLimit = BypassInterface::GetBypassDQHighLimit();
        float OutputVoltageLowLimit  = BypassInterface::GetBypassDQLowLimit();
        
        uint16_t data1 = 0, data2 = 0;
        
        if( OutputPLL.SourceNominalDQO.Sd > OutputVoltageHighLimit )
        {
            data1 += uint16_t(OutputPLL.SourceNominalDQO.Sd * 1000);
        }
        
        if( OutputRMSOV.GetState() )
        {
            if(OutputVoltageRMS.RawRMS.phA > OutputMaxLimit)
            {
                data1 += 1000;
            }
            if(OutputVoltageRMS.RawRMS.phB > OutputMaxLimit)
            {
                data1 += 2000;
            }
            if(OutputVoltageRMS.RawRMS.phC > OutputMaxLimit)
            {
                data1 += 4000;
            }
        }
        
        if( OutputPLL.SourceNominalDQO.Sd < OutputDQLowLimit )
        {
            data2 += uint16_t(OutputPLL.SourceNominalDQO.Sd * 1000);
        }
        
                
        NB_DebounceAndQue_Hysterisis( UPM_NB_OUTPUT_AC_OVER_VOLTAGE,
                ( OutputPLL.SourceNominalDQO.Sd > OutputVoltageHighLimit || OutputRMSOV.GetState() ),
                ( OutputPLL.SourceNominalDQO.Sd < ( OutputVoltageHighLimit - OutputVoltageHysteresis ) && !OutputRMSOV.GetState() ), data1
        );

        if( OutputRMSUV_HV.GetState() )
        {
            if(OutputVoltageRMS.RawRMS.phA < OutputMinLimit)
            {
                data2 += 1000;
            }
            if(OutputVoltageRMS.RawRMS.phB < OutputMinLimit)
            {
                data2 += 2000;
            }
            if(OutputVoltageRMS.RawRMS.phC < OutputMinLimit)
            {
                data2 += 4000;
            }
        }
			//Jacob/20130814/Merge 120k modify begin...
            // Pan/20130108 add Output ACUV active count from 3 to 10 during ECT
        if( ( MCUStateMachine.GetState() == EASY_CAPACITY_TEST_STATE ) &&
            !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE           ) )
        {
                //Process 7 time debounce added in ECT first 
                //add active count
            if( OutputPLL.SourceNominalDQO.Sd < OutputDQLowLimit || OutputRMSUV_HV.GetState() )
            {
                OutputACUVECTCount++;
                if( OutputACUVECTCount > ECT_MAX_ACUV_Count )
                {
                    OutputACUVECTCount = ECT_MAX_ACUV_Count;
                }
            }
                
                //clr active count 
            if( ( OutputPLL.SourceNominalDQO.Sd > ( OutputDQLowLimit + OutputVoltageHysteresis ) ) &&
                 !OutputRMSUV_HV.GetState() )
            {
                if( OutputACUVECTCount )
                {
                    OutputACUVECTCount--;
                }
            }
                
                //When ECTCount reaches MAX or 0, process 3 time debounce left 
            NB_DebounceAndQue_Hysterisis( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE,
                ( OutputACUVECTCount == ECT_MAX_ACUV_Count ),( OutputACUVECTCount == 0 ), data2 );	
                                
        }
        else
        {
//	            OutputACUVECTCount = 0;
//	
//	            if( !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) )
//	            {
//					if( ( AcuvCheck(OutputPLL.SourceNominalDQO.Sd, OutputDQLowLimit, OpAcuvTime1, &AcuvCounter1 ) ) ||
//					    ( AcuvCheck(OutputPLL.SourceNominalDQO.Sd, (OutputDQLowLimit - 0.1), OpAcuvTime2, &AcuvCounter2 ) ) ||
//	                    ( AcuvCheck(OutputPLL.SourceNominalDQO.Sd, OutputDQLowMinLimit, OpAcuvTime3, &AcuvCounter3 ) )  ||
//					    OutputRMSUV_HV.GetState())
//					{
//						NB_SetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE, true, data2 );	
//					}
//	            }
//	            else
//	            {
//	            	AcuvCounter1 = 0;
//					AcuvCounter2 = 0;
//					AcuvCounter3 = 0;
//	
//					if( OutputPLL.SourceNominalDQO.Sd > ( OutputDQLowLimit + OutputVoltageHysteresis ) && !OutputRMSUV_HV.GetState() )
//					{
//						NB_DebounceAndQue( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE, false );
//					}
//	            }
			OutputACUVECTCount = 0;

			//AC UV base follow 9P method: 
			//	 when ESS->Online(by Vo UV),during150ms time,expand the threshold; else keep original
			if(MCUStateMachine.ForceStayOnline != 0)
			{
				// Use looser hard coded limits (-40%) for post ESS mode.
				// 1.0 - ( 40 + 10 ) / 100.0 ) = 0.5
				OutputVoltageLowLimit = 0.5f;
			}			
			//end		
			
			if((MCUStateMachine.GetState() == INITIALIZATION_STATE) || (MCUStateMachine.GetState() == SHUTDOWN_STATE)
				|| (MCUStateMachine.GetState() == STANDBY_STATE) )
			{
				bFlagInvOff = true;
				wDelay = 0;
				
				//wombat jira398/450, when shutdown mode, no output uv
				NB_DebounceAndQue( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE,false);	
//					NB_DebounceAndQue( UPM_NB_OUTPUT_AC_OVER_VOLTAGE,false);
			}
			else
			{
				//when output exit, wait1000ms(special for master), UV check after Vo stable.
				if(bFlagInvOff != false)
				{
					if(wDelay++ >= 1125)	//1125=1000ms/(1/1.125)
					{
						wDelay = 0;
						bFlagInvOff = false;
					}
				}
				else
				{
					NB_DebounceAndQue_Hysterisis( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE,
						( OutputPLL.SourceNominalDQO.Sd < OutputVoltageLowLimit || OutputRMSUV_HV.GetState() ),
						( OutputPLL.SourceNominalDQO.Sd > ( OutputVoltageLowLimit + OutputVoltageHysteresis ) && !OutputRMSUV_HV.GetState() ), data2 );
				}
			}
        }
    }
}

// ********************************************************************************************************
// *
// * Function: CheckFasterACAlarms(void);
// *
// * Purpose: Checks only periodicly critical measurements
// *          called at 2.5kHz rate
// *
// ********************************************************************************************************

#pragma CODE_SECTION("ramfuncs")
void CheckFasterACAlarms(void)
{

}

// ********************************************************************************************************
// *
// * Function: CheckMediumACAlarms(void);
// *
// * Purpose: Checks only faster than 100ms alarms
// *          Called from 20ms periodic task
// *
// ********************************************************************************************************
void CheckMediumACAlarms(void)
{
    const float inputFreqHysteresis = 0.5;
    float out_nom_volts = float( OutNomVolts ) * 0.1;
    
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        // check utility UFOF
        NB_DebounceAndQue_Hysterisis( UPM_NB_INPUT_UNDER_OVER_FREQUENCY,
             ( Rectifier.UtilityPLL.GetFrequency() > InputFrequencyHighLimit ) ||
             ( Rectifier.UtilityPLL.GetFrequency() < InputFrequencyLowLimit  ) ||
             ( !Rectifier.UtilityPLL.IsPhaseLocked() ),
             
             ( Rectifier.UtilityPLL.GetFrequency() < ( InputFrequencyHighLimit - inputFreqHysteresis ) ) &&
             ( Rectifier.UtilityPLL.GetFrequency() > ( InputFrequencyLowLimit + inputFreqHysteresis ) ) &&
             ( Rectifier.UtilityPLL.IsPhaseLocked() )
        );
      
        float BypassFrequencyHighLimit = BypassInterface::GetBypassHighFreqLimit();
        float BypassFrequencyLowLimit  = BypassInterface::GetBypassLowFreqLimit();
        
        // check bypass UFOF
        NB_DebounceAndQue_Hysterisis( UPM_NB_BYPASS_UNDER_OVER_FREQUENCY,
                        !FrequencyConverterMode && //disable when frequency mode
                        ( ( BypassPLL.GetFrequency() > BypassFrequencyHighLimit ) ||
                          ( BypassPLL.GetFrequency() < BypassFrequencyLowLimit  ) ||
                          ( !BypassPLL.IsPhaseLocked() ) ),

                        FrequencyConverterMode || //clear when frequency mode
                        ( ( BypassPLL.GetFrequency() < ( BypassFrequencyHighLimit - inputFreqHysteresis ) ) &&
                          ( BypassPLL.GetFrequency() > ( BypassFrequencyLowLimit + inputFreqHysteresis ) ) &&
                          ( BypassPLL.IsPhaseLocked() ) )
        );
        
        if ( EEP_InputSyncEnabled() )
        {
            // check utility sync range
            NB_DebounceAndQue_Hysterisis( UPM_NB_INPUT_SYNC_OUT_OF_RANGE,
                 ( Rectifier.UtilityPLL.GetFrequency() > BypassFrequencyHighLimit ) ||
                 ( Rectifier.UtilityPLL.GetFrequency() < BypassFrequencyLowLimit ) ||
                 ( !Rectifier.UtilityPLL.IsPhaseLocked() ),
            
                 ( Rectifier.UtilityPLL.GetFrequency() < ( BypassFrequencyHighLimit - inputFreqHysteresis ) ) &&
                 ( Rectifier.UtilityPLL.GetFrequency() > ( BypassFrequencyLowLimit + inputFreqHysteresis ) ) &&
                 ( Rectifier.UtilityPLL.IsPhaseLocked() )
            );
        }        
                
        // Utility OV checking

        // redundant, fast OV should catch this before it's ever seen here
         NB_DebounceAndQue_Hysterisis( UPM_NB_INPUT_AC_OVER_VOLTAGE,
             ( ScreenMeters.InputVoltageRMS.phA > UtilityMaxRMSLimit ) ||
             ( ScreenMeters.InputVoltageRMS.phB > UtilityMaxRMSLimit ) ||
             ( ScreenMeters.InputVoltageRMS.phC > UtilityMaxRMSLimit ),        

             ( ScreenMeters.InputVoltageRMS.phA < ( UtilityMaxRMSLimit - VoltageRMSHysteresis ) ) &&
             ( ScreenMeters.InputVoltageRMS.phB < ( UtilityMaxRMSLimit - VoltageRMSHysteresis ) ) &&
             ( ScreenMeters.InputVoltageRMS.phC < ( UtilityMaxRMSLimit - VoltageRMSHysteresis ) )
        );    

        
        // Utility UV checking
        // Isolated: fast UV is not checked.
        NB_DebounceAndQue_Hysterisis( UPM_NB_INPUT_AC_UNDER_VOLTAGE,
             ( ScreenMeters.InputVoltageRMS.phA < UtilityMinRMSLimit ) ||
             ( ScreenMeters.InputVoltageRMS.phB < UtilityMinRMSLimit ) ||
             ( ScreenMeters.InputVoltageRMS.phC < UtilityMinRMSLimit ),       

             ( ScreenMeters.InputVoltageRMS.phA > ( UtilityMinRMSLimit + VoltageRMSHysteresis ) ) &&
             ( ScreenMeters.InputVoltageRMS.phB > ( UtilityMinRMSLimit + VoltageRMSHysteresis ) ) &&
             ( ScreenMeters.InputVoltageRMS.phC > ( UtilityMinRMSLimit + VoltageRMSHysteresis ) ) 
        );    
        
    
        // Inverter ACUV alarm
        if ( !MCUStateMachine.UPMTestMode                           &&
             ( Inverter.GetStatus().bit.InverterOn == true )        &&
             !Inverter.GetStatus().bit.MatchBypassRMS )
        {
            float UV_set_level = Inverter.InverterUVAlarmSetLevel * out_nom_volts;
            float UV_clear_level = Inverter.InverterUVAlarmClearLevel * out_nom_volts;
    
            NB_DebounceAndQue_Hysterisis(UPM_NB_INVERTER_AC_UNDER_VOLTAGE,
                 ( InverterVoltageRMS.RawRMS.phA < UV_set_level ) ||
                 ( InverterVoltageRMS.RawRMS.phB < UV_set_level ) ||
                 ( InverterVoltageRMS.RawRMS.phC < UV_set_level ) ,      
    
                 ( InverterVoltageRMS.RawRMS.phA > UV_clear_level ) &&
                 ( InverterVoltageRMS.RawRMS.phB > UV_clear_level ) &&
                 ( InverterVoltageRMS.RawRMS.phC > UV_clear_level ) 
            );
        }
        else
        {      
            NB_DebounceAndQue( UPM_NB_INVERTER_AC_UNDER_VOLTAGE, false );
        } 
    
        //Bypass over voltage
        float BypassMaxLimit = out_nom_volts * BypassState().GetBypassRMSHighLimit();
    
        BypassRMSOV.Debounce_Hysterisis(
             ( BypassVoltageRMS.RawRMS.phA > BypassMaxLimit ) ||
             ( BypassVoltageRMS.RawRMS.phB > BypassMaxLimit ) ||
             ( BypassVoltageRMS.RawRMS.phC > BypassMaxLimit ),
       
             ( BypassVoltageRMS.RawRMS.phA < ( BypassMaxLimit - VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phB < ( BypassMaxLimit - VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phC < ( BypassMaxLimit - VoltageRMSHysteresis ) ) 
        );    
    
        //Bypass under voltage
        float BypassMinLimit = out_nom_volts * BypassState().GetBypassRMSLowLimit();
    
        BypassRMSUV.Debounce_Hysterisis(
             ( BypassVoltageRMS.RawRMS.phA < BypassMinLimit ) ||
             ( BypassVoltageRMS.RawRMS.phB < BypassMinLimit ) ||
             ( BypassVoltageRMS.RawRMS.phC < BypassMinLimit ) ,
            
             ( BypassVoltageRMS.RawRMS.phA > ( BypassMinLimit + VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phB > ( BypassMinLimit + VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phC > ( BypassMinLimit + VoltageRMSHysteresis ) ) 
        );
        
        //Bypass under voltage for BypassAvailableOnBypass ( -25% limit )
        float Bypass25Limit = out_nom_volts * .75f;
    
        Bypass25RMSUV.Debounce_Hysterisis(
             ( BypassVoltageRMS.RawRMS.phA < Bypass25Limit ) ||
             ( BypassVoltageRMS.RawRMS.phB < Bypass25Limit ) ||
             ( BypassVoltageRMS.RawRMS.phC < Bypass25Limit ) ,
            
             ( BypassVoltageRMS.RawRMS.phA > ( Bypass25Limit + VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phB > ( Bypass25Limit + VoltageRMSHysteresis ) ) &&
             ( BypassVoltageRMS.RawRMS.phC > ( Bypass25Limit + VoltageRMSHysteresis ) ) 
        );
		
        //Output over voltage
        float OutputMaxLimit = out_nom_volts * BypassState().GetBypassRMSHighLimit();
    
        if(Inverter.GetStatus().bit.MatchBypassRMS)  //when match bypass, branch out OV
        {
		    OutputMaxLimit += 15;
        }
        // Pan/20130412 add for P9EPIEB-28
        // add delay when load power off, because output voltage is still high and drops slowly.	    
        if( NB_GetNodebit ( UPM_NB_LOAD_POWER_OFF ) )
        {
            if( OutRMSOVDelayCount++ >= OUTPUT_RMSOV_DELAY_500mS )      //Jacob/20130826/200ms->500ms fix P9EPIEB-28
            {
                OutRMSOVDelayCount = OUTPUT_RMSOV_DELAY_500mS;	            
            }
        }
        else
        {
            OutRMSOVDelayCount = 0;	
        }
               
        if( ( !NB_GetNodebit( UPM_NB_LOAD_POWER_OFF ) ) ||
            ( OutRMSOVDelayCount >= OUTPUT_RMSOV_DELAY_500mS ) )      //Jacob/20130826/200ms->500ms fix P9EPIEB-28
        {
            OutputRMSOV.Debounce_Hysterisis(
                    ( OutputVoltageRMS.RawRMS.phA > OutputMaxLimit ) ||
                    ( OutputVoltageRMS.RawRMS.phB > OutputMaxLimit ) ||
                    ( OutputVoltageRMS.RawRMS.phC > OutputMaxLimit ),
       
                    ( OutputVoltageRMS.RawRMS.phA < ( OutputMaxLimit - VoltageRMSHysteresis ) ) &&
                    ( OutputVoltageRMS.RawRMS.phB < ( OutputMaxLimit - VoltageRMSHysteresis ) ) &&
                    ( OutputVoltageRMS.RawRMS.phC < ( OutputMaxLimit - VoltageRMSHysteresis ) ) 
            );
        }
    
    
        //Output under voltage
        float OutputMinLimit = out_nom_volts * BypassState().GetBypassRMSLowLimit();

        if(Inverter.GetStatus().bit.MatchBypassRMS) //when match bypass, branch out  UV
        {
        	if(OutputMinLimit >= 15)
        	{
            	OutputMinLimit -= 15;
        	}
        }

        OutputRMSUV_HV.Debounce_Hysterisis(
             ( OutputVoltageRMS.RawRMS.phA < OutputMinLimit ) ||
             ( OutputVoltageRMS.RawRMS.phB < OutputMinLimit ) ||
             ( OutputVoltageRMS.RawRMS.phC < OutputMinLimit ) ,
            
             ( OutputVoltageRMS.RawRMS.phA > ( OutputMinLimit + VoltageRMSHysteresis ) ) &&
             ( OutputVoltageRMS.RawRMS.phB > ( OutputMinLimit + VoltageRMSHysteresis ) ) &&
             ( OutputVoltageRMS.RawRMS.phC > ( OutputMinLimit + VoltageRMSHysteresis ) )
        );
    
        OutVolRMS_NotPresent.Debounce_Hysterisis(
             ( OutputVoltageRMS.RawRMS.phA < OutputMinLimit ) &&
             ( OutputVoltageRMS.RawRMS.phB < OutputMinLimit ) &&
             ( OutputVoltageRMS.RawRMS.phC < OutputMinLimit ) ,
            
             ( OutputVoltageRMS.RawRMS.phA > ( OutputMinLimit + VoltageRMSHysteresis ) ) ||
             ( OutputVoltageRMS.RawRMS.phB > ( OutputMinLimit + VoltageRMSHysteresis ) ) ||
             ( OutputVoltageRMS.RawRMS.phC > ( OutputMinLimit + VoltageRMSHysteresis ) )
        );
        //Output under voltage for BypassAvailableOnBypass ( -25% limit )
        float Output25Limit = out_nom_volts * .75f;
    
        Output25RMSUV.Debounce_Hysterisis(
             ( OutputVoltageRMS.RawRMS.phA < Output25Limit ) ||
             ( OutputVoltageRMS.RawRMS.phB < Output25Limit ) ||
             ( OutputVoltageRMS.RawRMS.phC < Output25Limit ) ,
            
             ( OutputVoltageRMS.RawRMS.phA > ( Output25Limit + VoltageRMSHysteresis ) ) &&
             ( OutputVoltageRMS.RawRMS.phB > ( Output25Limit + VoltageRMSHysteresis ) ) &&
             ( OutputVoltageRMS.RawRMS.phC > ( Output25Limit + VoltageRMSHysteresis ) ) 
        );
    }
}

// ********************************************************************************************************
// *
// * Function: CheckSlowACAlarms(void);
// *
// * Purpose: Checks AC alarms
// *          Called from 100ms periodic task
// *
// ********************************************************************************************************
void CheckSlowACAlarms(void)
{
    CheckOverloadSlow();
} //  End of CheckSlowACAlarms


// ********************************************************************************************************
// *
// * Function: ee_update_overload_limits( EE_ID* ee, uint16_t* data )
// *
// * Purpose:  Update due to changed overload related EEPs
// *
// ********************************************************************************************************
void ee_update_overload_limits( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_OvrldLowPercent:
            OverloadLevel1          = (*data);
            break;
            
        case PARAM_OvrldMedPercent:
            OverloadLevel2          = (*data);
            OverloadLevel2VA_L      = float(*data) / 3.0;
            break;
            
        case PARAM_OvrldHiPercent: 
            OverloadLevel3VA_L      = float(*data) / 3.0;
            break;

        case PARAM_OvrldXtrmPercent: 
            OverloadLevel4VA_L      = float(*data) / 3.0;
            break;
            
        default:
            break;
    }
}

// ********************************************************************************************************
// *
// * Function: CheckOverloadMedium(void);
// *
// * Purpose: Checks fast overload alarms
// *          Called from 5ms periodic task.
// *
// ********************************************************************************************************
void CheckOverloadMedium( void )
{
    bool tmpOverloadTrip = false;
    uint16_t OverloadTime3Temp;
    uint16_t OverloadTime4Temp;

    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        OverloadCheck( OverloadLevel3VA_L, OverloadLevel2VA_L, UPM_NB_LEVEL_3_OVERLOAD_PHASE_A,
            UPM_NB_LEVEL_3_OVERLOAD_PHASE_B, UPM_NB_LEVEL_3_OVERLOAD_PHASE_C );
            
        OverloadCheck( OverloadLevel4VA_L, OverloadLevel3VA_L, UPM_NB_LEVEL_4_OVERLOAD_PHASE_A,
            UPM_NB_LEVEL_4_OVERLOAD_PHASE_B, UPM_NB_LEVEL_4_OVERLOAD_PHASE_C );
        
        if ( ( NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_A ) ) ||
             ( NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_B ) ) ||
             ( NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_C ) ) )
        {
            if ((AOLEnable && !AdaptOverloadCapacityOff ))
            {
                OverloadTime3Temp = AdapOverloadTime3;
            }
            else
            {
                OverloadTime3Temp = OverloadTime3;		//60s
            }
            
            int16_t olTime = ( OverloadTime3Temp / 5 ) - (NB_Cfg_Flash[UPM_NB_LEVEL_3_OVERLOAD_PHASE_A].bit.ActiveCount>>1);
            
            //add for HV system to satisfy Overload specification when UPS on battery: 126-150% 30 seconds
            rectifier_states_t RectState = Rectifier.GetState();
            
            if ( ( NB_GetNodebit( UPM_NB_UPS_ON_BATTERY )  ) &&
                 ( RectState == RECTIFIER_ON_BATTERY_STATE ) )
            {				
				if ((AOLEnable && !AdaptOverloadCapacityOff ))
				 {
					 OverloadTime3Temp = AdapOverloadTime3;
				 }
				 else
				 {
					 OverloadTime3Temp = OverloadTime3; 	 //60s
				 }
				 OverloadTime3Temp = OverloadTime3Temp / Overload_LineLvl3_60Sec_Eq_BatLvl3_10Sec;	//10s in bat
				 	
				 olTime = ( OverloadTime3Temp / 5 ) - (NB_Cfg_Flash[UPM_NB_LEVEL_3_OVERLOAD_PHASE_A].bit.ActiveCount>>1);				

            }
            
            if (olTime < 0) olTime = 0;
            
            if ( ++OverloadCnt3 > olTime )
            {
                OverloadCnt3 = olTime;

                // hey MCU, bypass please
                tmpOverloadTrip = true;
            }
        }
        else
        {
            if ( OverloadCnt3 )
            {
                --OverloadCnt3;
            }    
        }

        if ( ( NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_A ) ) ||
             ( NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_B ) ) ||
             ( NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_C ) ) )
        {
            if ((AOLEnable && !AdaptOverloadCapacityOff ))
            {
                OverloadTime4Temp = AdapOverloadTime4;
            }
            else
            {
                OverloadTime4Temp = OverloadTime4;
            }
            
            int16_t olTime = ( OverloadTime4Temp / 5 ) - NB_Cfg_Flash[UPM_NB_LEVEL_4_OVERLOAD_PHASE_A].bit.ActiveCount;
            if (olTime < 0) olTime = 0;

            if ( ++OverloadCnt4 > olTime )
            {
                OverloadCnt4 = olTime;

                // hey MCU, bypass please
                tmpOverloadTrip = true;
            }
        }
        else
        {
            if ( OverloadCnt4 )
            {
                --OverloadCnt4;
            }    
        }
    }
    else
    {
        NB_SetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_A, false );
        NB_SetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_B, false );
        NB_SetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_C, false );
        NB_SetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_A, false );
        NB_SetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_B, false );
        NB_SetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_C, false );
    }

    NB_DebounceAndQue( UPM_NB_OUTPUT_OVERLOAD_TRIP, 
    	 	MCUStateMachine.SupportingLoad() && ( OverloadTripSlow || tmpOverloadTrip ) );
}

// ********************************************************************************************************
// *
// * Function: CheckOverloadSlow(void);
// *
// * Purpose: Checks slow overload alarms
// *          Called from 100ms periodic task
// *
// ********************************************************************************************************
void CheckOverloadSlow(void)
{
    bool tmpOverloadTrip = false;
    uint16_t OverloadTime1Temp;
    uint16_t OverloadTime2Temp;

    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        Lev1OverloadCheck();
        Lev2OverloadCheck();
        
        // increment/decrement the silly counters
        if ( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD ) )
        {
            //add for HV system to satisfy Overload specification when UPS on battery: 102-125% 60 seconds
            rectifier_states_t RectState = Rectifier.GetState();

            if ((AOLEnable && !AdaptOverloadCapacityOff ))
            {
                OverloadTime1Temp = AdapOverloadTime1;
            }
            else
            {
                OverloadTime1Temp = OverloadTime1;
            }
            
            if ( ( NB_GetNodebit( UPM_NB_UPS_ON_BATTERY )  ) &&
                 ( RectState == RECTIFIER_ON_BATTERY_STATE ) )
            {
//	            	OverloadCnt1 += ( OverloadTime1Temp / Overload_LineLvl1_60Min_Eq_BatLvl1_10Min - 1 );
	            if ((AOLEnable && !AdaptOverloadCapacityOff ))
	            {
	                OverloadTime1Temp = ( AdapOverloadTime1 / Overload_LineLvl1_60Min_Eq_BatLvl1_10Min ) ;
	            }
	            else
	            {
	                OverloadTime1Temp = ( OverloadTime1 / Overload_LineLvl1_60Min_Eq_BatLvl1_10Min );
	            }            
			}
            
            if ( ++OverloadCnt1 > ( (uint32_t)OverloadTime1Temp  * 10 ) )
            {
                OverloadCnt1 = ( OverloadTime1Temp * 10 );

                // hey MCU, bypass please   
                tmpOverloadTrip = true;
            }
        }
        else
        {
            if ( OverloadCnt1 )
            {
                --OverloadCnt1;
            }    
        }
        
        if ( ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_A ) ) ||		
             ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_B ) ) ||
             ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_C ) ) )
        {
            //add for HV system to satisfy Overload specification when UPS on battery: 102-125% 60 seconds
            rectifier_states_t RectState = Rectifier.GetState();

            if ((AOLEnable && !AdaptOverloadCapacityOff ))
            {
                OverloadTime2Temp = AdapOverloadTime2;
            }
            else
            {
                OverloadTime2Temp = OverloadTime2;									//1.1.Line mode:60min
            }
            
            if ( ( NB_GetNodebit( UPM_NB_UPS_ON_BATTERY )  ) &&
                 ( RectState == RECTIFIER_ON_BATTERY_STATE ) )
            {
//	                OverloadCnt2 += ( OverloadTime2Temp / Overload_Level1_1Min - 1 );	//1.2.Bat mode-:1min
//					OverloadCnt2 += ( OverloadTime2Temp / Overload_LineLvl2_10Min_Eq_BatLvl2_30Sec - 1 );	//1.2.Bat mode-:1min
	            if ((AOLEnable && !AdaptOverloadCapacityOff ))
	            {
	                OverloadTime2Temp = ( AdapOverloadTime2 / Overload_LineLvl2_10Min_Eq_BatLvl2_30Sec);
	            }
	            else
	            {
	                OverloadTime2Temp = ( OverloadTime2 / Overload_LineLvl2_10Min_Eq_BatLvl2_30Sec);									//1.1.Line mode:60min
	            }
            }
            
            if ( ++OverloadCnt2 > ( OverloadTime2Temp * 10 ) )
            {
                OverloadCnt2 = ( OverloadTime2Temp * 10 );	

                // hey MCU, bypass please    
                tmpOverloadTrip = true;
            }
        }
        else
        {
            if ( OverloadCnt2 )
            {
                --OverloadCnt2;
            }    
        }
    }
    
    OverloadTripSlow = tmpOverloadTrip;
}

// ********************************************************************************************************
// *
// * Function: Lev1OverloadCheck(void);
// *
// * Purpose: Checks level 1 overload
// *
// ********************************************************************************************************
void Lev1OverloadCheck( void )
{
    bool tmpStatus;

    if ( NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD ) )
    {        
        float hyst = 5;
        if ( OverloadLevel1 > 101 )
        {
            hyst = (float)OverloadLevel1 - 1.0;
        }
            
        // clear when below 100%
        tmpStatus = !( ScreenMeters.PercentLoad.sum <= hyst );
    }
    //Jacob/20130924/modify begin...GPE-1146
    else
    {
        tmpStatus = ScreenMeters.PercentLoad.sum > float(OverloadLevel1);
    }
    tmpStatus = tmpStatus || 
                NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_A) ||
                NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_B) ||
                NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_C) ||
                NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_A) ||
                NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_B) ||
                NB_GetNodebit( UPM_NB_LEVEL_3_OVERLOAD_PHASE_C) ||
        		NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_A) ||
                NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_B) ||
                NB_GetNodebit( UPM_NB_LEVEL_4_OVERLOAD_PHASE_C);
    //Jacob/20130924/...modify end
    NB_DebounceAndQue( UPM_NB_OUTPUT_OVERLOAD, tmpStatus );
}


// ********************************************************************************************************
// *
// * Function: Lev2OverloadCheck(void);
// *
// * Purpose: Checks level 2 overload
// *
// ********************************************************************************************************
void Lev2OverloadCheck( void )
{
    bool tmpStatus;

    if ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_A ) )
    {
        // clear when below level1
        tmpStatus = !( (int16_t)ScreenMeters.PercentLoad.phA < OverloadLevel1 );
    }
    else
    {
        tmpStatus = ( (int16_t)ScreenMeters.PercentLoad.phA > OverloadLevel2 );
    } 
    NB_DebounceAndQue( UPM_NB_LEVEL_2_OVERLOAD_PHASE_A, tmpStatus );
    
    if ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_B ) )
    {
        // clear when below level1
        tmpStatus = !( (int16_t)ScreenMeters.PercentLoad.phB < OverloadLevel1 );
    }
    else
    {
        tmpStatus = ( (int16_t)ScreenMeters.PercentLoad.phB > OverloadLevel2 );
    } 
    NB_DebounceAndQue( UPM_NB_LEVEL_2_OVERLOAD_PHASE_B, tmpStatus );
    
    if ( NB_GetNodebit( UPM_NB_LEVEL_2_OVERLOAD_PHASE_C ) )
    {
        // clear when below level1
        tmpStatus = !( (int16_t)ScreenMeters.PercentLoad.phC < OverloadLevel1 );
    }
    else
    {
        tmpStatus = ( (int16_t)ScreenMeters.PercentLoad.phC > OverloadLevel2 );
    } 
    NB_DebounceAndQue( UPM_NB_LEVEL_2_OVERLOAD_PHASE_C, tmpStatus );
}

// ********************************************************************************************************
// *
// * Function: OverloadCheck(void);
// *
// * Purpose: Checks level X overload
// *
// ********************************************************************************************************
void OverloadCheck( float OverloadLevel_L, float OverloadHyst_L, upm_nb_id_t overload_phA,
       upm_nb_id_t overload_phB, upm_nb_id_t overload_phC )
{
    upm_nb_id_t arr[] = { overload_phA, overload_phB, overload_phC };
    float active_power[] = { LoadPower.ActivePower.phA, LoadPower.ActivePower.phB, LoadPower.ActivePower.phC };
    float total_power[] = { LoadPower.TotalPower.phA, LoadPower.TotalPower.phB, LoadPower.TotalPower.phC };
    
    float OverloadLevel_VA_L      = OutputkVARating         * OverloadLevel_L;
    float OverloadLevel_VA_L_Hyst = OutputkVARating         * OverloadHyst_L;
    float OverloadLevel_W_L       = OverloadLevel_VA_L      * float(OutputPowerFactorRating) / 10000.0;
    float OverloadLevel_W_L_Hyst  = OverloadLevel_VA_L_Hyst * float(OutputPowerFactorRating) / 10000.0;

    if ( ( NumOfModule != 0 ) && MCUStateMachine.BypassSupportingLoad() )
    {
        OverloadLevel_VA_L      *= float( NumOfModule );
        OverloadLevel_VA_L_Hyst *= float( NumOfModule );
        OverloadLevel_W_L       *= float( NumOfModule );
        OverloadLevel_W_L_Hyst  *= float( NumOfModule );
    }

    for ( size_t i = 0; i < sizeof(arr); i++)
    {
        bool tmpStatus;
        
        if ( NB_GetNodebit( arr[i] ) )
        {
            tmpStatus = ( active_power[i] >= OverloadLevel_W_L_Hyst ||
                           total_power[i] >= OverloadLevel_VA_L_Hyst );
        }
        else
        {
            tmpStatus = ( active_power[i] >= OverloadLevel_W_L ||
                          total_power[i]  >= OverloadLevel_VA_L );
        } 
        NB_DebounceAndQue( arr[i], tmpStatus );
    }
}

// ****************************************************************************
// *
// *  Function: CheckSTSWshort(void)
// *
// *  Purpose : To check for a Static Switch short.
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description: Checks for current and backfeed voltage.
// *               (called every 500ms)
// *
// ****************************************************************************
void CheckSTSWshort(void)
{
    static FilteredBit STSWshortFilterL1( UPM_NB_STATIC_SWITCH_SHORT, IS_STICKY );
    static FilteredBit STSWshortFilterL2( UPM_NB_STATIC_SWITCH_SHORT, IS_STICKY );
    static FilteredBit STSWshortFilterL3( UPM_NB_STATIC_SWITCH_SHORT, IS_STICKY );

    uint16_t tmpSTSWshort = 0;
    float STSWshortCur;
    
    if ( ( OutNomVolts != 0 ) && ( NumOfUPMs != 0 ) )
    {
        STSWshortCur = (float)OutputkVARating * NumOfUPMs * 0.333333 * STSWshortCurrentPercent / OutNomVolts;
    }
    else
    {
        STSWshortCur = 30.0;
    }

        // check for Static Switch short current
    if (// DSPOutRegister.GpoB.bit.BackFeedRelay                      &&  don't worry about the relay, could be welded shut
         MCUStateMachine.GetMetersReadyFlag()                       &&
         !NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE )           &&
         !NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT )               &&
         BYPASS_FIRE_STATE != BypassState().GetMonitoredBypassState() &&
         NB_GetNodebit( UPM_NB_BYPASS_INSTALLED ) )
    {
        if ( STSWshortFilterL1.Debounce( BypassCurrentRMS.FilteredRMS.phA > STSWshortCur ) )
        {
            tmpSTSWshort = 1;
        }

        if ( STSWshortFilterL2.Debounce( BypassCurrentRMS.FilteredRMS.phB > STSWshortCur ) )
        {
            tmpSTSWshort *= 10;
            tmpSTSWshort += 2;
        }

        if ( STSWshortFilterL3.Debounce( BypassCurrentRMS.FilteredRMS.phC > STSWshortCur ) )
        {
            tmpSTSWshort *= 10;
            tmpSTSWshort += 3;
        }

        NB_SetNodebit( UPM_NB_STATIC_SWITCH_SHORT, tmpSTSWshort, tmpSTSWshort );
    }
    else
    {
        STSWshortFilterL1.SetState( false );
        STSWshortFilterL2.SetState( false );
        STSWshortFilterL3.SetState( false );
    }
    
    // Update the status bit for parallel systems
    MCUStateMachine.SetStaticSwitchShort( NB_GetNodebit( UPM_NB_STATIC_SWITCH_SHORT ) );
}


// ****************************************************************************
// *
// *  Function: CheckInstantaneousVol(void)
// *
// *  Purpose : To check instantaneous value of voltage.
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description: call by ProcessA2D_HWI(), it is 5.102KHz
// *
// ****************************************************************************
#pragma CODE_SECTION("ramfuncs")
void CheckInstantVoltage( void )
{
    float instantVolatgeLimit = 0.0f;
    uint16_t instantVoltage = 0;

	if(!UnitConfig.bit.EnableSurgeToBattery)
	{
		instantVolatgeLimit = UtilityMaxRMSLimit * SQRT_2 + 20.0f;
	}
	else
	{
	    instantVolatgeLimit = EEInputHighLimit;
	}
	InstantVoltageState = ( fabs(RawAdcDataPtr->st.InputVoltage.phA) > instantVolatgeLimit)         |
                          ( (fabs(RawAdcDataPtr->st.InputVoltage.phB) > instantVolatgeLimit) << 1 ) |
                          ( (fabs(RawAdcDataPtr->st.InputVoltage.phC) > instantVolatgeLimit) << 2 );
    if(InstantVoltageState)
    {
    	if(InstantVoltageState & 0x01)
    	{
    		instantVoltage =  uint16_t( fabs(RawAdcDataPtr->st.InputVoltage.phA) );
    	}
    	else if(InstantVoltageState & 0x02)
    	{
    		instantVoltage = uint16_t( fabs(RawAdcDataPtr->st.InputVoltage.phB) );
    	}
    	else
    	{
    		instantVoltage = uint16_t( fabs(RawAdcDataPtr->st.InputVoltage.phC) );
    	}
    }
    NB_DebounceAndQue( UPM_NB_INPUT_VOLTAGE_ABNORMAL, InstantVoltageState, instantVoltage );
	
}


// ********************************************************************************************************
// *
// * Function: AcuvCheck(void);
// *
// * Purpose: Checks output under volatge
// *
// ********************************************************************************************************
uint16_t AcuvCheck(float VoltageSd, float UnderLimit, uint16_t ActiveTime, uint16_t *Counter)
{
    if(VoltageSd < UnderLimit)
	{
		if(++(*Counter) >= ActiveTime)
		{
			(*Counter) = 0;
			return(true);
		}
	}
	else
	{
		(*Counter) = 0;
	}

	return(false);
}
// *********************************************************************************************************
// *        End of File
// *********************************************************************************************************

