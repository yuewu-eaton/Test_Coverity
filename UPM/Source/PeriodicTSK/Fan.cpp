// ********************************************************************
// *            Fan.c
// ********************************************************************
// ********************************************************************
// * 
// * This information is proprietry to Eaton Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 2005 Eaton Corporation                       
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   Fan.cpp
// *                                                                      
// *     DESCRIPTION: 
// *                  
// *     ORIGINATOR:  Jun Zhang                                         
// *                                                                      
// *     DATE:        05/04/2010                                            
// *                                                                      
// *     HISTORY:                                                         
// ********************************************************************

// ********************************************************************
// *            Include files                                                                           
// ********************************************************************
#include "DSP28x_Project.h"
#include "F28335Port.h"
#include "IOexpansion.h"
#include "NB_Funcs.h"
#include "RectifierStateControl.h"
#include "MCUState.h"
#include "Fan.h"
#include "Thermal.h"
#include "BypassInterface.h"
#include "BatteryStateControl.h"

#include <cmath>
#include <algorithm>

using namespace std;

// Constant definition
//STS Fan
#define cSpeed_0      0     // level 0
#define cSpeed_100    100   //level 3, 100% duty of FANDSPEED_DUTY  = 22.9V based on test with 5 Fans no failure

#define cLoad20          20   // 20%
#define cLoad40          40   // 40%
#define cLoad55          55   // 55%
#define cLoad60          60   // 60%
#define cLoad80          80   // 80%
#define cLoad100         100  //100%
#define cLoad125         125  //100%

// CSB Control: 60-80K Bypass STS Fan
#define FanDutyLevel1    40
#define FanDutyLevel2    65
#define FanDutyLevel3    75
#define FanDutyLevel4    85

//UPM Fan
#define cSpeed0          0
#define cSpeed1          1    // 0.5 duty
#define cSpeed2          2    // 100KVA:0.65 duty
#define cSpeed3          3    // 100KVA:0.85 duty
#define cSpeed4          4    // 100KVA:0.99 duty

#define cFullduty        100  // fan full duty
                              
#define cLoad17          17   // 17%
#define cLoad20          20   // 20%
#define cLoad33          33   // 33%
#define cLoad38          38   // 38%
#define cLoad40          40   // 40%
#define cLoad50          50   // 50%
#define cLoad66          66   // 66%
#define cLoad75          75   // 75%
#define cLoad78			 78   // 78%
#define cLoad80          80   // 80%

// STS Fan Fail Logic
#define cSTSFanFailLogic 0    // STS Fan Fail Logic Level = 'L'
#define cSTSFanOKLogic   1    // STS Fan OK   Logic Level = 'H'

#define cFanFailLogic 0    // Fan Fail Logic Level = 'L'
#define cFanOKLogic   1    // Fan OK   Logic Level = 'H'

#define cFanControlTime  3  // 3 seconds
#define cFanFailTime     50 // 50*100ms times about 5s, time will vary per fan speed
#define cFanOKTime       5  // 5*100ms times about 0.5s,time will vary per fan speed
#define STSfanBit        6

#define FanPWMPeriod     ( (CPUFrequency / ADCFrequency)*2 )

#define FAN_UpCONTROL_TIMER    	5    //0.1S = 5 * 0.02S 
#define FAN_DownCONTROL_TIMER   150  //3S = 150 * 0.02S 

// Global data
uint16_t FanSpeed = cSpeed1;
uint16_t FanSpeedNew = cSpeed1;
uint16_t FanStatus = 0x0000;
uint16_t FanTestSpeedUPM = 0;
uint16_t FanTestSpeedSTS = 0;
uint16_t FanTestSpeedUpmEnable = false;
uint16_t FanTestSpeedSTSEnable = false;
uint16_t STSFanStateCheckEnable = false;
uint16_t FanLoad;
uint16_t FanTestLoadEnalbe;
uint16_t FanTestLoad;

uint16_t FanA_Speed_Temp = 0;
uint16_t FanB_Speed_Temp = 0;
uint16_t FanC_Speed_Temp = 0;

uint16_t FanA_Speed_Final = 0;
uint16_t FanB_Speed_Final = 0;
uint16_t FanC_Speed_Final = 0;

uint16_t FanFailLowLimit = 5;
uint16_t FanFailErrorLimit;

uint16_t FanSpeed_Ref = 0;

uint16_t FanMaxSpeed;

uint16_t FanDutyToPLD = 0;

uint16_t STSFanSpeed = 0;
uint16_t STSFanDutyToPLD = 0;

static uint16_t UpmFanSpeedUpCheckCount = 0;
static uint16_t UpmFanSpeedDownCheckCount = 0;

// Local function declaration
uint16_t FanSpeedAdjustCheckUp(uint16_t CurrentLoad,uint16_t SettingLoad,uint16_t CheckTime);
uint16_t FanSpeedAdjustCheckDown(uint16_t CurrentLoad,uint16_t SettingLoad,uint16_t CheckTime);
void FansOn( void );
void FansOff( void );
void FanControlHV20K_40K( float LegAmpsRMS );    		
static uint16_t SpeedChangeCheck(uint16_t Condition, uint16_t* pCounter, uint16_t CheckTime, uint16_t* pPreSpeedType, uint16_t CurrentSpeed);
static void SpeedChangeCheckSTS(const uint16_t Condition, StateTimer * Counter, const uint16_t CheckTime, uint16_t *SpeedType, const uint16_t Speed);
static uint16_t CheckFanTimeout(uint16_t* pCurrentCnt, uint16_t TimeOutCnt);
static uint16_t SpeedUpCheck(uint16_t* pCurrentFanSpeed, uint16_t MaxInputAmpsRMS);
static uint16_t SpeedDownCheck(uint16_t* pCurrentFanSpeed, uint16_t MaxInputAmpsRMS);
static bool SpeedUpLoad(const float CurrentLoad,const float SettingLoad);
static bool SpeedDownLoad(const float CurrentLoad,const float SettingLoad,const float LoadHysteresisLevel);
void FanTestWithSpeed( const uint16_t Testfan, const uint16_t SpeedPercent);
void FanTestWithLoad(const uint16_t TestEnable, const uint16_t LoadPercent);

// ********************************************************************
// *                                                                                                                     
// * Function:  FanHv20_40KControl() 
// *
// * Purpose:   Check fan speed and perform fan control FOR 3C3PRO 40K/80K
// *                    
// * Parms Passed:  percent_load - Load level in percent
// *
// * Returns:   Nothing                                                                           
// *
// * Description: This function firstly compares six current(utility,inverter)
// *              value to decide the new speed,then performs fan control.      
// ********************************************************************
void FanHv20_40KControl(void)
{
	float MaxCurrentRms = 0.0;

	MaxCurrentRms = ScreenMeters.InputCurrentRMS.phA;
    if(MaxCurrentRms < ScreenMeters.InputCurrentRMS.phB)
    {
    	MaxCurrentRms = ScreenMeters.InputCurrentRMS.phB;
    }
    if(MaxCurrentRms < ScreenMeters.InputCurrentRMS.phC)
    {
    	MaxCurrentRms = ScreenMeters.InputCurrentRMS.phC;
    }
    if(MaxCurrentRms < ScreenMeters.OutputCurrentRMS.phA)
    {
    	MaxCurrentRms = ScreenMeters.OutputCurrentRMS.phA;
    }
    if(MaxCurrentRms < ScreenMeters.OutputCurrentRMS.phB)
    {
    	MaxCurrentRms = ScreenMeters.OutputCurrentRMS.phB;
    }
    if(MaxCurrentRms < ScreenMeters.OutputCurrentRMS.phC)
    {
    	MaxCurrentRms = ScreenMeters.OutputCurrentRMS.phC;
    }	
	FanControlHV20K_40K(MaxCurrentRms);
}

// ********************************************************************
// *                                                                                                                     
// * Function:  FanSpeedAdjustCheckUp() 
// *
// * Purpose:   Check whether the fan should speed up
// *                    
// * Parms Passed:  CurrentLoad - Current Load level in percent 
// *                SettingLoad - Setting Load level in percent for Check
// *                CheckTime   - Filter time for stability of fan control
// * Returns:   Nothing                                                                           
// *
// * Description: This function continuously checks current load and setting
// *              load for a given period of time and makes decision whether
// *              fan should speed up or not.       
// ********************************************************************
uint16_t FanSpeedAdjustCheckUp(uint16_t CurrentLoad,uint16_t SettingLoad,uint16_t CheckTime)
{
    static uint16_t CheckCount = 0;
    
    if(CurrentLoad >= SettingLoad)
    {
        if(++CheckCount >= CheckTime)
        {
            CheckCount = 0;
            return(true);
        }
    }
    else
    {
        CheckCount = 0;
    }
    
    return(false);
}

// ********************************************************************
// *                                                                                                                     
// * Function:  FanSpeedAdjustCheckDown() 
// *
// * Purpose:   Check whether the fan should slow down
// *                    
// * Parms Passed:  CurrentLoad - Current Load level in percent 
// *                SettingLoad - Setting Load level in percent for Check
// *                CheckTime   - Filter time for stability of fan control
// * Returns:   Nothing                                                                           
// *
// * Description: This function continuously checks current load and setting
// *              load for a given period of time and makes decision whether
// *              fan should slow down or not.       
// ********************************************************************
uint16_t FanSpeedAdjustCheckDown(uint16_t CurrentLoad,uint16_t SettingLoad,uint16_t CheckTime)
{
    static uint16_t CheckCount = 0;
    
    if(CurrentLoad < SettingLoad)
    {
        if(++CheckCount >= CheckTime)
        {
            CheckCount = 0;
            return(true);
        }
    }
    else
    {
        CheckCount = 0;
    }
    
    return(false);
}

// ********************************************************************
// *                                                                                                                     
// * Function:  STSFanStateCheck() 
// *
// * Purpose:   Check STS fan state.
// *                    
// * Parms Passed:  None 
// *
// * Returns:   Nothing                                                                           
// *
// * Description: This function runs every 300ms. 
// *      
// ********************************************************************
void STSFanStateCheck(void)
{
	uint16_t OutputKVA = OutputkVARating * NumOfModule;
    if( ( STSFanStateCheckEnable == true) && ( ExpansionInputReg.Ready == true ) )
    {
		// bit6: StsFan status  20-80K delete
    	if( OutputKVA != 200 && OutputKVA != 300 && OutputKVA != 400 && OutputKVA != 600 && OutputKVA != 800)
    	{
    		if((FanStatus&0x064) != 0)
    		{
    			if( ExpansionInputReg.bit.FanFailSTS == cSTSFanOKLogic )
    			{
    				FanStatus &= ~( ( ExpansionInputReg.bit.FanFailSTS == cSTSFanOKLogic ) << STSfanBit );
    			}
    		}
    		else
    		{
    			if( ( ExpansionInputReg.bit.FanFailSTS == cSTSFanFailLogic )  )
    			{
    				FanStatus |= ( ( ExpansionInputReg.bit.FanFailSTS == cSTSFanFailLogic ) << STSfanBit );
    			}
    		}
    	}

        // byp not ready/or not fire
		if( !BypassPLL.IsSourcePresent() || BypassState().GetBypassState() != BYPASS_FIRE_STATE )
		{
			FanStatus &= ~( 1 << STSfanBit); //Bypass is not present or bypass is off, Fan must OK.
		}
    }
}

// ********************************************************************
// *
// * Function:  FanSpeedCalculate()
// *
// * Purpose:   Calculate fan speed.
// *
// * Parms Passed:  None
// *
// * Returns:   Nothing
// *
// * Description: This function runs every 1000ms.
// *
// ********************************************************************
void FanSpeedCalculate(void)
{
    uint16_t ClrCount = 0;
    static uint16_t PldReadZeroCnt = 0;

    ReadPldFanA();
    FanA_Speed_Temp =  PLDFanAReg * 30;//60//The fan has two pulses at one turn
    ReadPldFanB();
    FanB_Speed_Temp =  PLDFanBReg * 30;
    ReadPldFanC();
    FanC_Speed_Temp =  PLDFanCReg * 30;

    if((PLDFanAReg == 0) && (PLDFanBReg == 0) && (PLDFanCReg == 0))   // Use fan speed instantaneous values in the initial phase or all fan speed is 0
    {
        FanA_Speed_Final = 0;
        FanB_Speed_Final = 0;
        FanC_Speed_Final = 0;
    }
    else
    {
        FanA_Speed_Final = (uint16_t)((float)FanA_Speed_Final * 0.7 + (float)FanA_Speed_Temp * 0.3);
        FanB_Speed_Final = (uint16_t)((float)FanB_Speed_Final * 0.7 + (float)FanB_Speed_Temp * 0.3);
        FanC_Speed_Final = (uint16_t)((float)FanC_Speed_Final * 0.7 + (float)FanC_Speed_Temp * 0.3);

    }       
}

// ********************************************************************
// *
// * Function:  FanSpeedCheck_pld()
// *
// * Purpose:    check fan speed.
// *
// * Parms Passed:  None
// *
// * Returns:   Nothing
// *
// * Description: This function runs every 100ms.
// *
// ********************************************************************
void FanSpeedCheck_pld(void)
{
//    static uint16_t FanChkCnt = 0;
    uint16_t error_code = 0;
    static uint16_t PreviousFanSpeed_Ref = 0;
	static uint16_t Checkout5sTime = 50; //when FanDutyToPLD is changed, wait 5s to check fan failed
	static uint16_t FanSpeedChanged = false;	
	
//    FanChkCnt++;
//    if(FanChkCnt >= 3) // check fan 2.2s(22*100ms)
//    {

	if (CONST_InterfaceBrdRev_ID_P3 == InterfaceBoardRevID)
	{
    	if( ( STSFanStateCheckEnable == true) && ( DSPInRegister.GpiC.bit.FanCheck_RevID_P3 == cFanFailLogic ) )
    	{
    		 error_code = (uint16_t)1;
    	}
	}
    else if(CONST_InterfaceBrdRev_ID_P6 == InterfaceBoardRevID)
    {
    	if( ( STSFanStateCheckEnable == true) && ( ExpansionInputReg.bit.FanCheck_RevID_P6 == cFanFailLogic )  )
    	{
    		error_code = (uint16_t)1;
    	}
    }
    else
    {
		error_code = (uint16_t)2;
    }

#if 0
    	//In previous version is 5100-21*FanDutyToPLD. To compatible with Jaguar new fan code, change to 5000-20*FanDutyToPLD.
    	const uint16_t dutyStep = (FanMaxSpeed + 128)/255; // RPM per duty, rounded up. =20@5000.
    	
    	if(FanDutyToPLD > 230)//means fan duty less than 10%
    	{
    		FanSpeed_Ref = 0;
    	}
    	else
    	{
    		FanSpeed_Ref = uint16_t(FanMaxSpeed - dutyStep * FanDutyToPLD);
    	}

    	if(FanSpeed_Ref > FanMaxSpeed)  // max fan speed
    	{
    	    FanSpeed_Ref = FanMaxSpeed;
    	}

		if(PreviousFanSpeed_Ref != FanSpeed_Ref) 
		{
		    PreviousFanSpeed_Ref = FanSpeed_Ref;
			Checkout5sTime = 50;
			FanSpeedChanged = true;
		}

        if(FanSpeedChanged)
    	{
            Checkout5sTime--;
		
		    if( 0 == Checkout5sTime)
		    {
		        Checkout5sTime = 50;
				FanSpeedChanged = false;
		    }
    	}		
		
		if( ( STSFanStateCheckEnable == true) && ( ExpansionInputReg.Ready == true ) )
        {
		    if(FanA_Speed_Final <= FanFailLowLimit)
            {
                error_code |= (uint16_t)1;
            }

		    if(FanB_Speed_Final <= FanFailLowLimit)
		    {
                error_code |= ((uint16_t)1 << 1);
            }

		    if(FanC_Speed_Final <= FanFailLowLimit)
            {
                error_code |= ((uint16_t)1 << 2);
            }

			if(FanSpeedChanged == false)//Fan speed need 5s to be stable, if fan speed is changed, wait 5s before check fan failed
			{
			    if(fabs((float)FanSpeed_Ref - (float)FanA_Speed_Temp) > FanFailErrorLimit)
		    	{
                    error_code |= ((uint16_t)1 << 3);
		    	}

				if(fabs((float)FanSpeed_Ref - (float)FanB_Speed_Temp) > FanFailErrorLimit)
				{
                    error_code |= ((uint16_t)1 << 4);
				}

				if(fabs((float)FanSpeed_Ref - (float)FanC_Speed_Temp) > FanFailErrorLimit)
				{
                    error_code |= ((uint16_t)1 << 5);
				}
			}
        }
#endif
		STSFanStateCheck();

		FanStatus &= 0xFFC0;	//for jira hobbit156 clear fan, clear upm bit0~5
		FanStatus |= error_code;
        NB_DebounceAndQue( UPM_NB_FAN_FAILURE, FanStatus, FanStatus );	
		
//        FanChkCnt = 0;
//	}
}

// ********************************************************************
// *
// * Function:  FanTestWithSpeed()
// *
// * Purpose:   Test fan speed using debugger.
// *
// * Parms Passed:  Testfan - 0-disable 1-upm fan 2-STS fan
// *                SpeedPercent - target speed
// * Returns:   Nothing
// *
// * Description:
// *
// ********************************************************************
void FanTestWithSpeed(const uint16_t Testfan, const uint16_t SpeedPercent)
{
    if((Testfan <= 2) && (SpeedPercent <= 100))
    {
        if(Testfan == 1)
        {
            FanTestSpeedUPM = SpeedPercent;
            FanTestSpeedUpmEnable = 1;
        }
        
        if(Testfan == 2)
        {
            FanTestSpeedSTS = SpeedPercent;
            FanTestSpeedSTSEnable = 1;
        }
    }
    else
    {
        FanTestSpeedUPM = 0;
        FanTestSpeedSTS = 0;
        FanTestSpeedUpmEnable = 0;
        FanTestSpeedSTSEnable = 0;
    }
}

// ********************************************************************
// *
// * Function:  FanTestWithLoad()
// *
// * Purpose:   Test  fan with set Load using debugger.
// *
// * Parms Passed:  TestEnable - enable speed test
// *                Load - test Load
// * Returns:   Nothing
// *
// * Description:
// *
// ********************************************************************
void FanTestWithLoad(const uint16_t TestEnable, const uint16_t LoadPercent)
{

    if((TestEnable <= 1) && (LoadPercent <= 150))
    {
        FanTestLoadEnalbe = TestEnable;

        if(TestEnable == 1)
        {
            FanTestLoad = LoadPercent;
        }
        else
        {
            FanTestLoad = 0;
        }
    }
}

// ********************************************************************
// *
// * Function:  FanControlHV20K_40K()
// *
// * Purpose:   fan control of HV_20K,HV_30K and HV_40K
// *
// * Parms Passed:  LegAmpsRMS
// *
// * Returns:   Nothing
// *
// * Description:
// *
// ********************************************************************
void FanControlHV20K_40K( float LegAmpsRMS )
{
	static uint16_t delay = 0;
    static uint16_t speedDelay = 0;
	static float duty = 0.0;
	uint16_t MaxLegAmpsRMSTemp = 0;
	int16_t RecFullSpeedPoint,InvFullSpeedPoint;

	RecFullSpeedPoint = RecHeatsinkTempFanFull;	//eep244
	InvFullSpeedPoint = InvHeatsinkTempFanFull ;//eep253
	
    if(FanTestSpeedUpmEnable== false)
    {
        MaxLegAmpsRMSTemp = uint16_t( LegAmpsRMS );
    }
    else
    {
        MaxLegAmpsRMSTemp = FanTestSpeedUPM;
    }
	
    if( delay <= cFanControlTime )    // fan run at full speed during the first 3 sec @ 20ms
    {
        delay++;
        FanSpeed = cSpeed4;

        duty = 0.99;
        FanDutyToPLD = uint16_t((1.0 - duty) * 255);
        WritePldFan1( FanDutyToPLD );
        return;
    }
	STSFanStateCheckEnable = true;
	
    rectifier_states_t RectState = Rectifier.GetState();


    if( ( FanTestSpeedUpmEnable == false                         ) &&
        ( ( NB_GetNodebit( UPM_NB_UPS_ON_NORMAL )     &&
            ( ( RECTIFIER_ON_BATTERY_STATE == RectState ) ||
              ( RECTIFIER_FROM_BATTERY_STATE == RectState ) ) ) ||
          NB_GetNodebit( UPM_NB_FAN_FAILURE )) )         
    {
          
        FanSpeed = cSpeed4;
		UpmFanSpeedDownCheckCount = 0;
		UpmFanSpeedUpCheckCount = 0;
    }
//	else if((FanTestSpeedUpmEnable == false ) && ( !NB_GetNodebit( UPM_NB_INVERTER_ON )))
//	{
//        FanSpeed = cSpeed1;
//		UpmFanSpeedDownCheckCount = 0;
//		UpmFanSpeedUpCheckCount = 0;
//	}
	else
	{
		if( (PHASEB_Thermal_C >=  InvFullSpeedPoint)  ||
			(PHASEA_Thermal_C >=  InvFullSpeedPoint)  ||
			(PHASEC_Thermal_C >=  InvFullSpeedPoint) )
		{
			FanSpeed = cSpeed4;
			UpmFanSpeedDownCheckCount = 0;
			UpmFanSpeedUpCheckCount = 0;
		}
		else if( (PHASEB_Thermal_C < (InvFullSpeedPoint - 10))&&
			     (PHASEA_Thermal_C < (InvFullSpeedPoint - 10))&&
			     (PHASEC_Thermal_C < (InvFullSpeedPoint - 10)) )
		{
			if((cSpeed1 == FanSpeed) ||
			   (cSpeed2 == FanSpeed) ||
			   (cSpeed3 == FanSpeed) ||
			   (cSpeed4 == FanSpeed))
			{
				//judge whether need to speed up fan first
				if(!SpeedUpCheck(&FanSpeed, MaxLegAmpsRMSTemp))
				{
					//judge whether need to speed down fan
					SpeedDownCheck(&FanSpeed, MaxLegAmpsRMSTemp);
				}		
			}
			else
			{
				FanSpeed = cSpeed4;
				UpmFanSpeedDownCheckCount = 0;
				UpmFanSpeedUpCheckCount = 0;
			}
		}
	}

	//for hobbit-thermal request:
	//   Ichg>10A, Fanspeed+1; Ichg<7A, Fanspeed ori
	static uint16_t FlagChgFanUp = false;
	if(BatteryConverter.GetStatus().bit.ChargerOn)
	{
		if((BatteryCurrentPos.SlowFiltered > 10.0f) || (BatteryCurrentNeg.SlowFiltered > 10.0f))
		{
			FlagChgFanUp = true;
		}
		else if((BatteryCurrentPos.SlowFiltered < 7.0f) && (BatteryCurrentNeg.SlowFiltered < 7.0f))
		{
			FlagChgFanUp = false;
		}
	}
	else
	{
		FlagChgFanUp = false;
	}

	if(FlagChgFanUp)
	{
		FanSpeedNew = FanSpeed+cSpeed1;
		if(FanSpeedNew >= cSpeed4)
		{
			FanSpeedNew = cSpeed4;
		}
	}
	else
	{
		FanSpeedNew = FanSpeed;
	}
	
	if(cSpeed1 == FanSpeedNew)
	{
		duty = 0.5; //0.82;	//16.092V
	}
	else if(cSpeed2 == FanSpeedNew)
	{
		duty = 0.65;	//0.90;    //18.073V
	}
	else if(cSpeed3 == FanSpeedNew)
	{
		duty = 0.85;//0.96;	//19.783V
	}
	else	//lvl4
	{
		duty = 0.99;//24v	
	}	
	//end

    if (FanTestSpeedUpmEnable)
    {
        FanDutyToPLD = uint16_t((100.0f - FanTestSpeedUPM) / 100.0f * 255);
        WritePldFan1( FanDutyToPLD );
    }
	else
    {
        FanDutyToPLD = uint16_t((1.0 - duty) * 255);
        WritePldFan1( FanDutyToPLD );
    } 
}

// ********************************************************************
// *
// * Function:  STSFanControl()
// *
// * Purpose:   Check fan speed and perform fan control
// *
// * Parms Passed:  None
// *
// * Returns:   Nothing
// *
// * Description: This function firstly compares current load,current input voltage and current UPM
// *              Ambient temperature with setting value based on current speed level to decide
// *              the new speed, then performs fan control.
// ********************************************************************
void STSFanControl( const float LoadPercent, const float UpmAmbTemp )
{
    bool SpeedUp = false;
    bool SpeedDown = false;
    float AmbTempHighLevel = 40.0f;
//	    float LoadHysteresisLevel = 0.01;
	//for jira, paral*2 load80%,line mode+no byp,  fan speek mistake up/down by ups show 7.8~8.1k
	//correct bug, follow koala,FanLoadHysteresisLevel=10=1000*0.01f
	float LoadHysteresisLevel = 10.0f; // FanLoadHysteresisLevel is in percent

    static StateTimer STSFanSpeedCheckUpCount;
    static StateTimer STSFanSpeedCheckDownCount;

    if( BypassState().GetBypassState() == BYPASS_FIRE_STATE )
    {
        if ( NB_GetNodebit( UPM_NB_FAN_FAILURE ) ||
             ( UpmAmbTemp > AmbTempHighLevel )   ||
             ( LoadPercent > cLoad125 ) )
        {
            STSFanSpeed = cSpeed_100;
        }
        else
        {
            if( STSFanSpeed == FanDutyLevel2 )
            {
                SpeedUp = SpeedUpLoad(LoadPercent,cLoad80);
                SpeedChangeCheckSTS(SpeedUp, &STSFanSpeedCheckUpCount, cFanControlTime, &STSFanSpeed, FanDutyLevel4);
            }
            else if( STSFanSpeed == FanDutyLevel4 )
            {
                SpeedDown = SpeedDownLoad(LoadPercent, cLoad80, LoadHysteresisLevel);
                SpeedChangeCheckSTS(SpeedDown, &STSFanSpeedCheckDownCount, cFanControlTime, &STSFanSpeed, FanDutyLevel2);
            }
            else
            {
                STSFanSpeed = FanDutyLevel2;//other unexpected fan speed,changed to FanDutyLevel2 to check.
                SpeedDown = true; //Fix Koala400V-338.
                STSFanSpeedCheckUpCount.ClearTimer();
                STSFanSpeedCheckDownCount.ClearTimer();
            }
        }
    }
    else
    {
        STSFanSpeed = cSpeed_0;
    }

    if( FanTestSpeedSTSEnable )
    {
        STSFanSpeed = FanTestSpeedSTS;
    }

    STSFanDutyToPLD = uint16_t((100.0f - STSFanSpeed) / 100.0f * 255);
    WritePldFanSTS( STSFanDutyToPLD );
}

// ********************************************************************
// *
// * Function:  SpeedChangeCheck()
// *
// * Purpose:   continuously checks whether the fan can change speed
// *
// * Parms Passed:  Condition - condition to change speed
// *                Counter - counter for check time
// *                CheckTime - time bound to keep condition
// *                PreSpeedType - STS fan or chimney fan type
// *                CurrentSpeed - target fan speed
// * Returns:       None
// *
// * Description: This function continuously checks condition to decide whether fan should speed up or not.
// ********************************************************************
static uint16_t SpeedChangeCheck(uint16_t Condition, uint16_t* pCounter, uint16_t CheckTime, uint16_t *pPreSpeedType, uint16_t CurrentSpeed)
{
    uint16_t Result = FALSE;
    if ((pCounter != NULL) && (pPreSpeedType != NULL))
    {
        if (Condition != 0)
        {
            if(CheckFanTimeout(pCounter, CheckTime))
            {
                *pCounter = 0;
                *pPreSpeedType = CurrentSpeed;
                Result = TRUE;
            }
        }
        else
        {
            *pCounter = 0;
        }
    }

    return Result;
}

// ********************************************************************
// *
// * Function:  SpeedChangeCheckSTS()
// *
// * Purpose:   continuously checks whether the fan can change speed
// *
// * Parms Passed:  Condition - condition to change speed
// *                Counter - counter for check time
// *                CheckTime - time bound to keep condition
// *                SpeedType - STS fan or chimney fan type
// *                Speed - target fan speed
// * Returns:       None
// *
// * Description: This function continuously checks condition to decide whether fan should speed up or not.
// ********************************************************************
static void SpeedChangeCheckSTS(const uint16_t Condition, StateTimer * Counter, const uint16_t CheckTime, uint16_t *SpeedType, const uint16_t Speed)
{
    if (Counter == 0 || SpeedType == 0)
    {
        return;
    }
    if (Condition != 0)
    {
        if(Counter->CheckTimeout(CheckTime))
        {
            Counter->ClearTimer();
            *SpeedType = Speed;
        }
    }
    else
    {
        Counter->ClearTimer();
    }
}

// ********************************************************************************************************
// *
// * Function: CheckFanTimeout(uint16_t* pCurrentCnt, uint16_t TimeOutCnt)
// *
// * Purpose:
// *
// * Parms Passed   :   pCurrentCnt - current cnt
// *                    TimeOutCnt  - timeout value
// * Returns        :   TRUE/FALSE
// *
// * Description: This module will decrement the count value if it is not
// *              zero. It returns true when the count value is zero.
// ********************************************************************************************************
static uint16_t CheckFanTimeout(uint16_t* pCurrentCnt, uint16_t TimeOutCnt)
{
    if ( *pCurrentCnt < TimeOutCnt )
    {
        ++(*pCurrentCnt);
        return (FALSE);
    }
    else
    {
        return (TRUE);
    }
}

// ********************************************************************
// *
// * Function:  SpeedDownCheck()
// *
// * Purpose:   Check whether the fan should slow down based on max input current RMS
// *
// * Parms Passed:  pCurrentFanSpeed - Current Fan speed value
// *                MaxInputAmpsRMS - Max input current RMS
// * Returns:   TRUE if slow down based on current value, else FALSE
// *
// * Description: This function continuously checks whether fan should slow down or not.
// ********************************************************************
static uint16_t SpeedDownCheck(uint16_t* pCurrentFanSpeed, uint16_t MaxInputAmpsRMS)
{
    uint16_t Result = FALSE;
    uint16_t NewFanSpeed = 0;
    uint16_t CurrentFanSpeed = *pCurrentFanSpeed;
	

	if(cSpeed2 == CurrentFanSpeed)
	{
        //The reason to slow down Fan speed: MaxInputCurrentRMS <= (Level_1 - hysteresis)
        if(MaxInputAmpsRMS < (FanInputAmpsRMSLevel_1 - FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed1;
        }		
	}
	else if(cSpeed3 == CurrentFanSpeed)
	{
        //The reason to slow down Fan speed: MaxInputCurrentRMS <= (Level_2 - hysteresis)
        if(MaxInputAmpsRMS < (FanInputAmpsRMSLevel_2 - FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed2;
        }		
	}
	else if(cSpeed4 == CurrentFanSpeed)
	{
        //The reason to slow down Fan speed: MaxInputCurrentRMS <= (Level_3 - hysteresis)
        if(MaxInputAmpsRMS < (FanInputAmpsRMSLevel_3 - FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed3;
        }	
	}
 
    Result = SpeedChangeCheck(Result, &UpmFanSpeedDownCheckCount, cFanControlTime, pCurrentFanSpeed, NewFanSpeed);
    return Result;
}
// ********************************************************************
// *
// * Function:  SpeedUpCheck()
// *
// * Purpose:   Check whether the fan should speed up based on input phaseA current RMS
// *
// * Parms Passed:  InputAmpsRMS - max input current RMS
// *
// * Returns:   TRUE if speed up based on load, else FALSE
// *
// * Description: This function continuously checks current load and setting
// *              load to  decide whether fan should speed up or not.
// ********************************************************************
static uint16_t SpeedUpCheck(uint16_t* pCurrentFanSpeed, uint16_t MaxInputAmpsRMS)
{
    uint16_t Result = FALSE;
    uint16_t NewFanSpeed = 0;
    uint16_t CurrentFanSpeed = *pCurrentFanSpeed;
	

    if(cSpeed1 == CurrentFanSpeed)
	{
		//The reason to speed up Fan: MaxInputCurrentRMS > Level_1 + hysteresis
        if(MaxInputAmpsRMS > (FanInputAmpsRMSLevel_1 + FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed2;
        }		
	}
	else if(cSpeed2 == CurrentFanSpeed)
	{
		//The reason to speed up Fan: MaxInputCurrentRMS > Level_2 + hysteresis
        if(MaxInputAmpsRMS > (FanInputAmpsRMSLevel_2 + FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed3;
        }			
	}
	else if(cSpeed3 == CurrentFanSpeed)
	{
		//The reason to speed up Fan: MaxInputCurrentRMS > Level_3 + hysteresis
        if(MaxInputAmpsRMS > (FanInputAmpsRMSLevel_3 + FanInputAmpsRMSHysteresis))
        {
            Result = TRUE;
            NewFanSpeed = cSpeed4;
        }			
	}

    Result = SpeedChangeCheck(Result, &UpmFanSpeedUpCheckCount, cFanControlTime, pCurrentFanSpeed, NewFanSpeed);
    return Result;
}

// ********************************************************************
// *
// * Function:  SpeedUpLoad()
// *
// * Purpose:   Check whether the fan should speed up based on load
// *
// * Parms Passed:  CurrentLoad - Current Load level in percent
// *                SettingLoad - Setting Load level in percent for Check
// * Returns:   true if speed up based on load, else false
// *
// * Description: This function continuously checks current load and setting
// *              load to  decide whether fan should speed up or not.
// ********************************************************************
bool SpeedUpLoad(const float CurrentLoad,const float SettingLoad)
{
    bool Result = false;
    if( CurrentLoad > SettingLoad )
    {
        Result = true;
    }

    return Result;
}
// ********************************************************************
// *
// * Function:  SpeedDownLoad()
// *
// * Purpose:   Check whether the fan should speed down based on load
// *
// * Parms Passed:  CurrentLoad - Current Load level in percent
// *                SettingLoad - Setting Load level in percent for Check
// *                LoadHysteresisLevel - load hysteresis to speed down
// * Returns:   true if speed down based on load, else false
// *
// * Description: This function continuously checks current load and setting
// *              load to  decide whether fan should speed down or not.
// ********************************************************************
bool SpeedDownLoad(const float CurrentLoad,const float SettingLoad,const float LoadHysteresisLevel)
{
    bool Result = false;
    if( CurrentLoad < SettingLoad * (1.0f - LoadHysteresisLevel * 0.01f))
    {
        Result = true;
    }

    return Result;
}

// ********************************************************************
// *            End of Fan.cpp   
// ********************************************************************
