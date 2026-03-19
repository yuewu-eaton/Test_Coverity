// ******************************************************************************************************
// *            BTR.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BTR.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: 
// *
// *    DATE: 2/8/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#include "BTR.h"
#include <cmath>
#include "Abm.h"
#include "Meters.h"
#include "ACPowerMeter.h"
#include "Spi_Task.h"
#include "ParallelCan.h"
#include "Alarms.h"

using namespace std;

#define SDT_INITIAL_GUESS   2.5         //2.5 minutes
#define LDT_INITIAL_GUESS   20.0        //20 minutes
#define MAX_NUM_OF_INTERATION   20 
#define BTR_ONE_MINUTE          60     

BatteryTimeRemaining BTR;

// Pan/20120914 begin
// ******************************************************************************************************
// * BTR Constant Table and Efficiency Constans Table definitions
// ******************************************************************************************************
const stBTRParameters InternalBatteries_SDT = { -0.6582484,      5.425931,       0.99 };    //SDT: m1, b1, c1
const stBTRParameters InternalBatteries_LDT = { -0.813428,       5.8710247,      0.99 };    //LDT: m1, b1, c1
const stBTRParameters ExternalBatteries_SDT = { -0.011184136,    4.164654238,    0.97 };    //SDT: m2, b2, c2
//const stBTRParameters ExternalBatteries_LDT = { -1.247700732,    8.781242964,    1 };       //LDT: m2, b2, c2
const stBTRParameters ExternalBatteries_LDT = { -1.247700732,    8.781242964,    0.985 };       //LDT: m2, b2, c2

const stEffParameters EffParameters_30KVA = { 75.63998678,    0.001073315,    3.50314E-08,     -5.17122E-12,    1.17563E-16 };
const stEffParameters EffParameters_60KVA = { 72.76171899,    0.001415942,    -4.19207E-08,    5.05165E-13,     -1.92676E-18 };
// Pan/20120914 end

// ******************************************************************************************************
// *
// * Function:    Init( void )
// *
// * Purpose:     For initializing BTR prior to use
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************
// Pan/20121010 delete
/*
void BatteryTimeRemaining::Init( void )
{
    
}
*/

// ******************************************************************************************************
// *
// * Function:    BTR( void )
// *
// * Purpose:     Calculate fully charged BTR: for 30K&60KVA, if Internal Battery Breaker Installed,use 
// *              parallel batteries algorithm;
// *              if Internal Battery Breaker not Installed, or other units, use the exsiting algorithm:
// *              t = P^( 1 / m ) * exp( -b * c / m )
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::MonitorFullCapacityBTR( void )
{    
    // Pan/20120914 add parallel batteries algorithm, begin
    float CurrentPower = 0;
    stEffParameters const * EffParaTablePtr = NULL;
    float CheckLoadChanged = 0;
        
    if( MyUPMNumber == 0 )  //only master upm calculate BTR
    {
        if( NumOfUPMs == 1 )        //30KVA unit
        {
            CurrentPower = ScreenMeters.OutputPower.sum * NumOfModule;
            EffParaTablePtr = &EffParameters_30KVA;         // only for 30KVA unit
        }
        else                        //60KVA unit
        {   
            CurrentPower = ParallelCan.SystemLoad * (float)( OutputkWRating * NumOfModule );        //%load * KVArating
            EffParaTablePtr = &EffParameters_60KVA;
        }
		
            
        if( CurrentPower < 700.0 )//500.0 JIRA APACFA-59
        {
            CurrentPower = 700.0; //500.0
        } 
        CheckLoadChanged = fabs( PreviousPower - CurrentPower ) / CurrentPower;
            
        //calculate only when load or battery string changes
        if( ( CheckLoadChanged >= 0.03 ) || BTRStatus.bit.BTRParaChanged )
        {                       
            PreviousPower = CurrentPower;
            UPSPower = CurrentPower;    // Pan/20121112 add for debugger
                
            BTRStatus.bit.BTRParaChanged = false;
            CalculateBatPower( EffParaTablePtr ); 

            float BatteryPowerPerCell;

            if( ( NumberOfBatteryStrings > 0.00001 ) && ( NumberOfBatteryCells != 0 ) )
            {
                BatteryPowerPerCell = BatPower / ( (float) NumberOfBatteryCells * (float) NumberOfBatteryStrings );
                //t = P^(1/m) * e^(m)
                	
                HRDFullCapacityBTR = 60.0 * pow( double(BatteryPowerPerCell), double(HRDBattTimeSlopeReciprocal)) * HRDBattTimeExpConstant;
                LRDFullCapacityBTR = 60.0 * pow( double(BatteryPowerPerCell), double(LRDBattTimeSlopeReciprocal)) * LRDBattTimeExpConstant;
                	
                if( BTRStatus.bit.CustomDischargeCurve )
                {
                	FullCapacityBTR = 60.0 * pow( double(BatteryPowerPerCell), double(CustomDischargeSlopeReciprocal)) * CustomDischargeExpConstant;
                }
                else
                {
                	if( HRDFullCapacityBTR < (20.0 * 60.0) )
                    {
                        BTRStatus.bit.UseHRDFullCapacityBTR = true;
                    }
                    else if( HRDFullCapacityBTR > (22.0 * 60.0) )
                    {
                        BTRStatus.bit.UseHRDFullCapacityBTR = false;
                    }

                    if( BTRStatus.bit.UseHRDFullCapacityBTR )
                    {
                        FullCapacityBTR = HRDFullCapacityBTR;
                    }
                    else
                    {
                        FullCapacityBTR = LRDFullCapacityBTR;
                    }
                }
            }
            else
            {
                FullCapacityBTR = 0;
            }

        }
    }
   // Pan/20120914 end
}

// ******************************************************************************************************
// *
// * Function:    CalculateBatPower(  )
// *
// * Purpose:     Calculate BatteryPower:
// *              For 30K&60KVA:
// *              BatteryPower = UPSLoad /Efficiency
// *              Efficiency = ( a0 + a1 * UPSLoad + a2 * UPSLoad^2 + a3 * UPSLoad^3 + a4 * UPSLoad^4)
// *              For the other units,use rough guess 1.1 as efficiency
// *              BatteryPower = UPSLoad *1.10
// *
// * Parms Passed :     EffParaTablePtr : pointer to efficiency parameters table
// *
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::CalculateBatPower( stEffParameters const * EffParaTablePtr )
{
    //use rough guess(1.1 * UPSPower) as BatPower
    BatPower = UPSPower *1.10;
    BatEff = 1.0/1.10;
}

// ******************************************************************************************************
// *
// * Function:    ComputeMixedBTR( void )
// *
// * Purpose:     Calculate mixed fully charged BTR: for 30K&60KVA
// *              eff = ( a0 + a1 * P + a2 * P^2 + a3 * P^3 + a4 * P^4)
// *
// *              SDT: P = n1 * ( t ^ m1 ) * exp( c1 * b1 ) + n2 * ( ( ln(t) / m2 ) - c2 *b2 / m2 )
// *              LDT: P = n1 * ( t ^ m1 ) * exp( c1 * b1 ) + n2 * ( t ^ ( 1 / m2 ) ) * exp( - c2 *b2 / m2 )
// *              if the iteration fails, FullCapacityBTR = 0;
// *
// * Parms Passed :     EffParaTablePtr : pointer to efficiency parameters table
// *                    UPSLoad         : current load of UPS
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::ComputeMixedBTR( void )
{
    float PowerBatteryPerCell = 0;
    float BTRInternalPowResult = 0;
    float BTRExternalPowResult = 0;
    float CheckIterationStop = 0;
    float PrevIterationResult = SDT_INITIAL_GUESS;          //initial guess of SDT is 2.5 minutes    
    float CurrentIterationResult = 0;
    bool InterationFailFlag = true;    
    uint16_t NumOfIteration = 0;
 
    PowerBatteryPerCell = BatPower / ( ( float )NumberOfBatteryCells );     // Pan/20121112 add for debugger
    
    if( NumOfUPMs == 2 )
    {
        PowerBatteryPerCell = PowerBatteryPerCell / 2.0;    
    } 
    //use SDT calculate BTR first
    for ( NumOfIteration = 1; NumOfIteration <= MAX_NUM_OF_INTERATION; NumOfIteration++ )
    {   
        BTRInternalPowResult = pow( double( PrevIterationResult ), double( InternalBatteries_SDT.Slope - 1.0 ) );
        float SDT  = SDTInternalExp * BTRInternalPowResult * PrevIterationResult + \
                     SDTExternalSlope *log( PrevIterationResult ) - SDTExternalFactor - PowerBatteryPerCell;
        float dSDT = dSDTInternalExp * BTRInternalPowResult + SDTExternalSlope / PrevIterationResult;  
        
        if( fabs( dSDT ) >= 0.00001 )       //make sure dSDT != 0;
        {
            CurrentIterationResult = PrevIterationResult - SDT / dSDT;
            if( CurrentIterationResult >= 0.00001 )     // make sure iteration result is positive;
            {
                CheckIterationStop = fabs( ( CurrentIterationResult- PrevIterationResult ) / CurrentIterationResult );
                PrevIterationResult = CurrentIterationResult;
                if ( CheckIterationStop < 0.00001 )      //iteration success
                {
                    InterationFailFlag = false;
                    break;
                }
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }                                    
    }
                
    if ( InterationFailFlag )                     //iteration fail
    {
        CurrentIterationResult = 0;             
    }
    NumOfSDTIteration = NumOfIteration;         //for debugger
    NumOfLDTIteration = 0;
        
    // if the result is greater than 20 minutes, use LDT;
    if ( PrevIterationResult > LDT_INITIAL_GUESS )    
    {
        InterationFailFlag = true; 
        PrevIterationResult = LDT_INITIAL_GUESS;          //initial guess of LDT is 20 minutes
        for ( NumOfIteration = 1; NumOfIteration <= MAX_NUM_OF_INTERATION ; NumOfIteration++ )
        {
            BTRInternalPowResult = pow( double( PrevIterationResult ), double( InternalBatteries_LDT.Slope - 1.0 ) );
            BTRExternalPowResult = pow( double( PrevIterationResult ), double( 1.0 / ExternalBatteries_LDT.Slope - 1.0 ) );
            float LDT  = LDTInternalExp * BTRInternalPowResult * PrevIterationResult + \
                         LDTExternalExp * BTRExternalPowResult * PrevIterationResult - PowerBatteryPerCell;
            float dLDT = dLDTInternalExp * BTRInternalPowResult + dLDTExternalExp * BTRExternalPowResult;
            
            if( fabs( dLDT ) >= 0.00001 )       //make sure dLDT != 0;
            {
                CurrentIterationResult = PrevIterationResult - LDT / dLDT;
                if( CurrentIterationResult >= 0.00001 )     // make sure iteration result is positive;
                {
                    CheckIterationStop = fabs( ( CurrentIterationResult- PrevIterationResult ) / CurrentIterationResult );
                    PrevIterationResult = CurrentIterationResult;
                    if ( CheckIterationStop < 0.00001 )      //iteration success
                    {
                        InterationFailFlag = false;
                        break;
                    }
                }
                else
                {
                    break;
                }
            }                                            
        }
                    
        if ( InterationFailFlag )                        //iteration fail
        {
            CurrentIterationResult = 0;
        }
        NumOfLDTIteration = NumOfIteration;         //for debugger
                               
    }
               
    FullCapacityBTR = CurrentIterationResult * BTR_ONE_MINUTE;       
    
} 

// ******************************************************************************************************
// *
// * Function:    StartDualStageBatteryTest( void )
// *
// * Purpose:     Start the quick battery test if condiitons are met.
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::StartDualStageBatteryTest( void )
{
    battery_states_t BatteryStateCopy = BatteryConverter.GetBatteryState();

    // if battery is available and bypass is available
    if( BatteryStateCopy == BATTERY_SHUTDOWN_STATE         &&
        !NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR )          &&
		!NB_GetNodebit( UPM_NB_IN_EASY_CAPACITY_TEST_MODE ) 
      )
    {
        BatteryConverter.BatteryTestCmdOn();
    }
}

// ******************************************************************************************************
// *
// * Function:    GetBatteryTestResults( void )
// *
// * Purpose:     Looks for when battery test results are available and then stores them.
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************
#define BTR_ONE_SEC 50
void BatteryTimeRemaining::FindBatteryTestResults( void )
{
    BatteryTestInfo temp50per = BatteryConverter.GetBatteryTestResults().FiftyPercentTest;
    BatteryTestInfo temp100per = BatteryConverter.GetBatteryTestResults().OneHundredPercentTest;
    
    if( BTRStatus.bit.BattTestResultsReady )
    {
        BTRStatus.bit.BattTestResultsReady = 0;
        if( CommissioningTest )
        {
            CommissioningTest = 0;
            WriteParameter( PARAM_CommissioningTest, &CommissioningTest, 1 );
            CommisioningTestResults.BatteryTest = BatteryConverter.GetBatteryTestResults();
            CommisioningTestResults.Resistance = ( temp50per.Voltage - temp100per.Voltage ) / ( temp100per.Current - temp50per.Current );
            CommisioningTestResults.Health = 1.0;
        }
        else
        {
            LastBatteryTestResults.BatteryTest = BatteryConverter.GetBatteryTestResults();
            LastBatteryTestResults.Resistance = ( temp50per.Voltage - temp100per.Voltage ) / ( temp100per.Current - temp50per.Current );
            LastBatteryTestResults.Health = CommisioningTestResults.Resistance / LastBatteryTestResults.Resistance;
        }  
    }
 
}

// ******************************************************************************************************
// *
// * Function:    CheckAlarms( void )
// *
// * Purpose:     Looks for conditions to end battery test (500 ms)
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::CheckAlarms( void )
{
    static eAbmState OldABMStateCopy;   
    eAbmState ABMStateCopy = Abm().GetState(); 
    
//Alarms    
    //battery test fail
    if( ABMStateCopy != OldABMStateCopy )
    {
        if( ABMStateCopy == ABM_DISCHARGING && OldABMStateCopy == ABM_REST )
        {       
            BTRStatus.bit.BTMonitorBattTestFail = true;    
            if( ( FullCapacityBTR / 60.0 ) > 15.0 )
            {
                BatteryTestFailVLevel = HighRateTestThreshold * NumberOfBatteryCells;  //battery test fail level        
            }
            else
            {
                BatteryTestFailVLevel = LowRateTestThreshold * NumberOfBatteryCells;  //battery test fail level
            }           
        }
        
        if( ABMStateCopy != ABM_DISCHARGING )
        {
            BTRStatus.bit.BTMonitorBattTestFail = false;
        } 
    }     
    OldABMStateCopy = ABMStateCopy;
    
    if( ABMStateCopy == ABM_DISCHARGING && BTRStatus.bit.BTMonitorBattTestFail ) //only monitor when battery was resting before
    {
        if( !BatteryTestFailTimer.CheckTimeout( (uint32_t)(0.25 * FullCapacityBTR) ) )
        {
            if( BatteryVoltage.FastFiltered < BatteryTestFailVLevel )
            {
                BTRStatus.bit.BTFailLowBattery = true;
            }
        }
    }
    else
    {
        BatteryTestFailTimer.ClearTimer();
    }

    BatteryConverter.BatteryStatus.bit.BattTestOn = NB_DebounceAndQue( UPM_NB_BATTERY_TEST_IN_PROGRESS, BTRStatus.bit.BatteryQuickTest ||
                                                        BTRStatus.bit.BatteryTest || 
                                                        BatteryConverter.GetBatteryStateStatus().bit.BatteryTestCmd );
    NB_DebounceAndQue( UPM_NB_BATTERY_TEST_FAILED, Abm().GetStatus().bit.Battery_Failure || 
                                                   BatteryConverter.BatTestAbandonReason,
                                                   Abm().GetStatus().bit.Battery_Failure * 0x01 +
                                                   BatteryConverter.BatTestAbandonReason & 0xFE);

    BTRStatus.bit.BTDone = false;
    BTRStatus.bit.BTAbortLoadChange = false;
    BTRStatus.bit.BTFailLowBattery = false;
    BTRStatus.bit.BTAbortBypassNotAvailable = false;
}



// ******************************************************************************************************
// *
// * Function:    MonitorStateOfCapacity( void )
// *
// * Purpose:     Updates how much capacity the batterys have
// *
// *
// * Parms Passed : None
// * Returns      : None
// *
// *
// * Description:
// *
// ******************************************************************************************************

void BatteryTimeRemaining::MonitorStateOfCapacity( void )
{
    eAbmState AbmStateCopy = Abm().GetState();
    static eAbmState oldAbmStateCopy;
    float FloatVoltage = ( BattABMFloatVPC - CompensateVPC )* (float)NumberOfBatteryCells;
   	static float PercentOfHistory = 0.001f;
   	float  PercentOfCharge;
   	static float ChargeTimerSecCnt = 0;
   	
    if( AbmStateCopy != oldAbmStateCopy )
    {
        if( AbmStateCopy == ABM_CHARGE )
        {
            StartChargeVoltage = 2.0 * (float)NumberOfBatteryCells;  
        }
        
        if( AbmStateCopy == ABM_DISCHARGING)
        {
            TimeOnBattery = (1.0 - PercentOfCapacity) * FullCapacityBTR;  //initialize TimeOnBattery
            OldPercentOfCapacity = PercentOfCapacity;
        }
    }
    oldAbmStateCopy = AbmStateCopy;


//  % Charge voltage =  (Battery voltage �start charge voltage) / (float voltage �start charge voltage) 0 <= x <=
    if( AbmStateCopy == ABM_CHARGE )
    {
        float ChargeVoltageDiff = ( BatteryVoltage.SlowFiltered - StartChargeVoltage );
        float FloatVMinusSCV = ( FloatVoltage - StartChargeVoltage );
        
        if( ChargeVoltageDiff < 0.0 ) ChargeVoltageDiff = 0.0;
        if( FloatVMinusSCV <= 0.0 ) 
        {
            ChargeVoltageDiff = 1.0;
            FloatVMinusSCV = 1.0;
        }
    
        ChargeVoltagePercent = ChargeVoltageDiff / FloatVMinusSCV;
        if( ChargeVoltagePercent >= 1.0 ) ChargeVoltagePercent = 1.0;
    } 

//  % Charge current     = (max charge current �charge current) / (max charge current)     0 <= x <= 1
    if(( AbmStateCopy != ABM_DISCHARGING )&&( AbmStateCopy != ABM_RESET ))//JIRA APACFA-59
    {
        float ChargeCurrentRatio;
        
        ChargeCurrentRatio = fabs( BatteryCurrentPos.FastFiltered / BattChrgCurrentMax );
        if( ChargeCurrentRatio > 1.0 ) ChargeCurrentRatio = 1.0;
        
        ChargeCurrentPercent = 1.0 - ChargeCurrentRatio;
    }
    else
    {
        ChargeCurrentPercent = 0;
    }
    
//  % Charge time = elapsed time on charge / total charge time    0 <= x <= 1
    if( AbmStateCopy == ABM_FLOAT )
    {
        ChargeVoltagePercent = 1.0;         // Pan/20121018 add
        ChargeTimePercent = (float)total_Float_Time / (float)(BattFloatTime + BattFloatTimeExt); 
        if( ChargeTimePercent > 1.0 ) ChargeTimePercent = 1.0;
    }
    else if( AbmStateCopy == ABM_DISCHARGING )
    {
        ChargeTimePercent = 0;
    }


    if( AbmStateCopy == ABM_DISCHARGING)
    {
        //TimeOnBattery = TimeOnBattery + 1.0;                    //called at a 1 second rate   // Pan/20120914 delete
        TimeOnBattery = (1.0 - PercentOfCapacity) * FullCapacityBTR + 1.0;      // Pan/20120924 add
        //GPE-1537
        if( abs(FullCapacityBTR) < 0.000001 )
        {
            PercentOfCapacity = 0;
        }
        else
        {
        	PercentOfCapacity = 1.0 - ( TimeOnBattery / FullCapacityBTR );
        }
        
        if( PercentOfCapacity < 0 ) PercentOfCapacity = 0;
        PercentOfHistory = PercentOfCapacity;
        ChargeTimerSecCnt = 0;
    }     
    else if( ( AbmStateCopy == ABM_REST )&&( total_Float_Time >= ( BattFloatTime + BattFloatTimeExt ) ) )         // Pan/20121018 add
    {
        PercentOfCapacity = 1.0;
		PercentOfHistory = PercentOfCapacity;
        ChargeTimerSecCnt = 0;
    }    // Pan/20121018 end
    else if( AbmStateCopy != ABM_RESET )//JIRA APACFA-59
    {
        //  Total % charge = 0.8 * % charge voltage + 0.15 * % charge current + 0.05 * % charge time 0 <= x <= 1    
		//PercentOfCapacity = 0.8 * ChargeVoltagePercent + 0.15 * ChargeCurrentPercent + 0.05 * ChargeTimePercent;
    	PercentOfCharge = 0.8 * ChargeVoltagePercent + 0.15 * ChargeCurrentPercent + 0.05 * ChargeTimePercent;
		PercentOfCapacity = PercentOfHistory;
    	if(PercentOfCharge < PercentOfCapacity )
    	{
    		ChargeTimerSecCnt ++;
    		if(PercentOfCapacity > PercentOfHistory + 0.1f)
    		{
    			PercentOfCapacity = PercentOfHistory + 0.1f;
    		}
    		else
    		{
				if(ChargeTimerSecCnt > 36)//0.6min,increase 0.01%;10h increase max 10%
				{
					ChargeTimerSecCnt = 0;
					PercentOfCapacity += 0.0001;
				}    			
    		}
    		
    		if(PercentOfCapacity > 1.0) PercentOfCapacity = 1.0;
    	}
    	else
    	{
    		PercentOfCapacity = PercentOfCharge;
    		ChargeTimerSecCnt = 0;
    		if(PercentOfCapacity > 1.0) PercentOfCapacity = 1.0;
    	}
    }
	else
	{
		if(!BTRStatus.bit.BTRInit)
		{
			PercentOfCapacity = 0.8 * ChargeVoltagePercent + 0.15 * ChargeCurrentPercent + 0.05 * ChargeTimePercent;
			PercentOfHistory = PercentOfCapacity;
			BTRStatus.bit.BTRInit = 1;
		}
	}

    BatteryTimeLeft = PercentOfCapacity * FullCapacityBTR;
}



// ***********************************************************************
// *
// *    FUNCTION: EEFunc_BTR
// *
// *    DESCRIPTION: Btr ee data function
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryTimeRemaining::EEFunc_BTR( const EE_ID* ee, const uint16_t* data )
{
    int16_t temp;
    const float c = 0.97;

    switch ( ee->paramNum )
    {
        case PARAM_HRDBattTimeIntercept:
            temp = *data;
            HRDBattTimeIntercept = float( temp ) / 1024.0;  //eep is in q10
            break;
            
        case PARAM_HRDBattTimeSlope:
            temp = *data;
            
            if ( temp )
            {
                HRDBattTimeSlopeReciprocal = 1.0 / (float( temp ) / 8192.0);  //eep is in q13
            }
            else
            {
                HRDBattTimeSlopeReciprocal = 1.0;
            }
            break;
            
        case PARAM_LRDBattTimeIntercept:
            temp = *data;
            LRDBattTimeIntercept = float( temp ) / 1024.0;  //eep is in q10
            break;
            
        case PARAM_LRDBattTimeSlope:
            temp = *data;
            
            if ( temp )
            {
                LRDBattTimeSlopeReciprocal = 1.0 / (float( temp ) / 8192.0);  //eep is in q13
            }
            else
            {
                LRDBattTimeSlopeReciprocal = 1.0;
            }
            break; 
            
        // Pan/20120914  for new added eep parameters, begin
        case PARAM_NumStrings:                  
            temp = *data;
            NumberOfBatteryStrings = (float)temp / 300.0; // EEP has a base of 300
            break;
            
        case PARAM_NumExternalStrings:
            temp = *data;
            NumberOfExternalBatteryStrings = temp;
            break; 
        // Pan/20120914 end   
            
        case PARAM_WattsPerCell15Min:
            temp = *data;
            WattsPerCell15Min = temp; 
            break;
            
        case PARAM_WattsPerCell60Min:
            temp = *data;   
            WattsPerCell60Min = temp;
            break;
        
        case PARAM_WattsPerCellUnits:
            temp = *data;
            WattsPerCellUnits = temp;
            break;
            
        default:
            break;
    }
    
    // Update the exp constant whenever these change
    switch ( ee->paramNum )
    {
        case PARAM_HRDBattTimeIntercept:
        case PARAM_HRDBattTimeSlope:
            HRDBattTimeExpConstant = exp( -c * HRDBattTimeIntercept * HRDBattTimeSlopeReciprocal );
            break;
            
        case PARAM_LRDBattTimeIntercept:
        case PARAM_LRDBattTimeSlope:
            LRDBattTimeExpConstant = exp( -c * LRDBattTimeIntercept * LRDBattTimeSlopeReciprocal );
            break; 
        
        // Pan/20120914 calculate BTR coeffiecent when internal or exteranl batteries strings change,begin   
        case PARAM_NumStrings:                
            SDTInternalExp = (float)NumberOfBatteryStrings * exp( InternalBatteries_SDT.Intercept * InternalBatteries_SDT.ScaleFactor );
            dSDTInternalExp = InternalBatteries_SDT.Slope * SDTInternalExp;
            
            LDTInternalExp = (float)NumberOfBatteryStrings * exp( InternalBatteries_LDT.Intercept * InternalBatteries_LDT.ScaleFactor );
            dLDTInternalExp = InternalBatteries_LDT.Slope * LDTInternalExp;
            break;
            
        case PARAM_NumExternalStrings:
            SDTExternalSlope = (float)NumberOfExternalBatteryStrings / ExternalBatteries_SDT.Slope;
            SDTExternalFactor = SDTExternalSlope * ExternalBatteries_SDT.Intercept * ExternalBatteries_SDT.ScaleFactor;
            
            LDTExternalExp = (float)NumberOfExternalBatteryStrings * \
                              exp( - ExternalBatteries_LDT.Intercept * ExternalBatteries_LDT.ScaleFactor / ExternalBatteries_LDT.Slope );
            dLDTExternalExp = LDTExternalExp / ExternalBatteries_LDT.Slope;
                                    
            // Pan/20121031 when battery strings != 0 and battery breaker installed, charge current = 20A, begin
            if( BatteryBreakerInstalled && NumberOfExternalBatteryStrings )
            {
                EE_BattChrgCurrentMax = 200;
            }
            else
            {
                EE_BattChrgCurrentMax = 120;
            }

            // Pan/20121031 add end.
            break;    
        // Pan/20120914 end
        
        case PARAM_WattsPerCell15Min:
        case PARAM_WattsPerCell60Min:
        case PARAM_WattsPerCellUnits:
            if ( ( WattsPerCell15Min > 0 ) && ( WattsPerCell60Min > 0 ) && ( WattsPerCell15Min != WattsPerCell60Min ) )
            {
                if( WattsPerCellUnits == 0 )
                {
                    WattsPerCellUnits = 100;    //set to default value
                }
                
                BTRStatus.bit.CustomDischargeCurve = true;
                CustomDischargeSlopeReciprocal =   ( log(15.0) - log(60.0) ) / ( log(float(WattsPerCell15Min) / float(WattsPerCellUnits)) - log(float(WattsPerCell60Min) / float(WattsPerCellUnits)) ) ;
                CustomDischargeIntercept = log(float(WattsPerCell15Min) / float(WattsPerCellUnits)) - ( log(15.0) / CustomDischargeSlopeReciprocal );
                CustomDischargeExpConstant = exp( -c * CustomDischargeIntercept * CustomDischargeSlopeReciprocal );
            }
            else
            {
                BTRStatus.bit.CustomDischargeCurve = false;
                CustomDischargeSlopeReciprocal = 1.0;
                CustomDischargeIntercept = 1.0;
                CustomDischargeExpConstant = 1.0;
            }
            break;
            
        default:
            break;                        
    }
        
    //set this bit to recalculate btr;
    BTRStatus.bit.BTRParaChanged = true;
}


// ******************************************************************************************************
// *            End of BTR.cpp
// ******************************************************************************************************
