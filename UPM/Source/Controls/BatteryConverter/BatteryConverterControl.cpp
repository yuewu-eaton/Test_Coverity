// ******************************************************************************************************
// *            BatteryConverter.cpp
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
// *    FILE NAME: BatteryConverter.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: 
// *
// *    DATE: 2/8/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "Constants.h"     
#include "F28335Port.h"  
#include "adc.h"
#include "algos.h"
#include "InitPWM.h"
#include "Meters.h"
#include "BatteryConverterControl.h"
#include "BatteryStateControl.h"
#include "RectifierStateControl.h"
#include "ACPowerMeter.h"
#include "BypassInterface.h"
#include "ParallelCan.h"
#include "C9Commands.h"
#include "Version.h"

using namespace std;

extern "C"
{
    void InitBattConv(void);
}

//HV 20K Bat control structure coefficients                 B0,       B1,       B2,       A1,       A2,       X1,       X2
//const stFirstOrderIIRFP BoostVGains_20KVA_HV          = { 0.01652,		-0.85,		-1.0, 	0	};
const stFirstOrderIIRFP BoostVGains_20KVA_HV          = { 0.01284,		-0.85,		-1.0, 	0	};  // B0*7/9
const stFirstOrderIIRFP BoostPowerModeGains_20KVA_HV  = { 0.000008,  -0.8338,   -0.999,   0   };
//wombat:Dz_v = 0.008(z - 0.5)/(z - 1.0)
//const stFirstOrderIIRFP ChargeVGains_20KVA_HV         = { 0.008, 	-0.5,	  -1.0,	  0	};     //Vloop modify param basic 9P
const stFirstOrderIIRFP ChargeVGains_20KVA_HV         = { 0.5643, 	-0.8884,	  -1.0,	  0	};     //Vloop modify param basic 9P
//wombat:Dz_i = 0.003(z - 0.1)/(z - 1.0)
const stFirstOrderIIRFP OuterChargeIGains_20KVA_HV    = { 0.003,       -0.9,         -1.000,    0 	}; //Iloop modify param basic 9P

//HV 40K Bat control structure coefficients                 B0,       B1,       B2,       A1,       A2,       X1,       X2
const stFirstOrderIIRFP BoostVGains_40KVA_HV          = { 0.01284*1.8f,		-0.85,		-1.0, 	0	};
const stFirstOrderIIRFP BoostPowerModeGains_40KVA_HV  = { 0.000008,  -0.8338,   -0.999,   0   };
//wombat:Dz_v = 0.008(z - 0.5)/(z - 1.0)
const stFirstOrderIIRFP ChargeVGains_40KVA_HV         = { 0.5643, 	-0.8884,	  -1.0,	  0	};     //Vloop modify param basic 9P
//wombat:Dz_i = 0.003(z - 0.1)/(z - 1.0)
//	const stFirstOrderIIRFP OuterChargeIGains_40KVA_HV    = { 0.003,       -0.9,         -1.000,    0 	}; //Iloop modify param basic 9P
const stFirstOrderIIRFP OuterChargeIGains_40KVA_HV    = { 0.0015,       -0.85,         -1.000,    0 	}; //Iloop modify param basic 9P

const stFirstOrderIIRFP BoostLegShareI_20KVA          = { 0.002,    -0.9990,        -1.0,     0    };
const stFirstOrderIIRFP BoostLegShareI_40KVA          = { 0.0012,    -0.9993,        -1.0,     0    };

const stFirstOrderIIRFP ChargeVGains_FloatLowCurr      = { 0.008, 	-0.5,	  -1.0,	  0	};     //Vloop modify param basic 9P


uint16_t BattHWCurrentLimit = 0;
//	float Kls = 0.0002f;
uint16_t FlagLowCurrChg_p = 0;
uint16_t FlagLowCurrChg_n = 0;
extern float DynamicLoadPercent;
extern float DynamicLoadPercentRec;
extern uint16_t FlagHighBusP;
extern uint16_t FlagHighBusN;
uint16_t BoostLegBOn = 0;
extern uint16_t InterfaceBoardRevID;
float BatChargeCurrLoopGain = 1.0f;
float BatChargeFFGain = 0.0f;//Voltage feed-forward in the charging current loop to eliminate current ripple

// global battery
//BatteryConverterControl BatteryConverter; 

void InitBattConv(void)
{
    BatteryConverter.Init();
}

// ***********************************************************************
// *
// *    FUNCTION: BatteryConverter_init 
// *
// *    DESCRIPTION: initializes inverter data to known default 
// *
// *    ARGUMENTS: pointer to inverter type
// *
// *    RETURNS:
// *
// *    Keng@ note: freq is Hertz(60Hz = 60), voltage is deci-volts (120Vac = 1200)
// *                TBD in Panda 
// *
// ***********************************************************************
void BatteryConverterControl::Init( void )
{
	SetHWCurrentLimit(BattHWCurrentLimit);
    SetChargeCurrentLimit(12.0);  
    SetChargeVoltage(441.6);              
    SetDCLinkVoltage(390.0);   
   
    // initialize it again after the module being identified.
	//up down mode
	ChargePWMPeriod = float( float( CPUFrequency / ChargeFrequency) / 2.0f );
	BoostPWMPeriod =  float( float( CPUFrequency / BoostFrequency)  / 2.0f );

    ChargeVGainsPos = *ChargeVGains_ptr;
    ChargeVGainsNeg = *ChargeVGains_ptr;
    ChargeVSatFactor = 1.0f / ( ChargeVGainsPos.B1 - ChargeVGainsPos.A1 );

	ChargeVGainsPos_Float = ChargeVGains_FloatLowCurr;
	ChargeVGainsNeg_Float = ChargeVGains_FloatLowCurr;
    ChargeVSatFactor_Float = 1.0f / ( ChargeVGainsPos_Float.B1 - ChargeVGainsPos_Float.A1 );

    OuterChargeIGainsPos = *OuterChargeIGains_ptr;
	OuterChargeIGainsNeg = *OuterChargeIGains_ptr;
    OuterChargeISatFactor = 1.0f / ( OuterChargeIGainsPos.B1 - OuterChargeIGainsPos.A1 );

    BoostVGainsPos = *BoostVGains_ptr;
    BoostVGainsNeg = *BoostVGains_ptr;
    BoostVSatFactor = 1.0f / ( BoostVGainsPos.B1 - BoostVGainsPos.A1);

	if(UPMSystem == HV_20K)
	{
	    BoostLegShareGainsPos = BoostLegShareI_20KVA;
	    BoostLegShareGainsNeg = BoostLegShareI_20KVA;
	}
	else
	{
		BoostLegShareGainsPos = BoostLegShareI_40KVA;
		BoostLegShareGainsNeg = BoostLegShareI_40KVA;
	}
    BoostLegShareSatFactor = 1.0 / ( BoostLegShareGainsPos.B1 - BoostLegShareGainsPos.A1 );

//	    BoostPowerModeGains = *BoostPowerModeGains_ptr;          
//	    BoostPowerModeSatFactor = 1.0 / ( BoostPowerModeGains.B1 - BoostPowerModeGains.A1 );
      
    BatteryConverterStatus.all = 0;
    
    BattPrechargeDuty = 0.02f;//0.01;
    
    BoostDownSampleCount = 0;
//	    BoostPowerModeDownSampleCount = 0;
    
    BoostDutyPos = 0;
    BoostDutyNeg = 0;
    
    BatteryCurrentCalEnable = false;
    BatteryCurrentPhase = 0;
//	KfwBatBoost = 0.1;
//	ErrVoltPos  = 0.0;
	ErrVoltPos_1 = 0.0;
//	ErrVoltNeg   = 0.0;
	ErrVoltNeg_1 = 0.0;

//    ChargeDownSampleCount = 0;
	ChgFlagCVmode_pos = false;
	ChgFlagCVmode_neg = false;
	MaxDutyChargePos = 0.5f;
	MaxDutyChargeNeg = 0.5f;
	BoostLegBOn = 0;
	MaxDutyPos = 0.7f;
	MaxDutyNeg = 0.7f;

}
    
// ***********************************************************************
// *
// *    FUNCTION: RunBoost - ISR function 
// *
// *    DESCRIPTION: Inner inverter controller
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
//	#pragma CODE_SECTION( "ramfuncs" )
void BatteryConverterControl::RunBoost( void )		
{   
    float dutyPos = 0.0f;
    float dutyNeg = 0.0f;
    float DCLinkVoltagePos = 0.0f;
    float DCLinkVoltageNeg = 0.0f;
    float satFactor = 0.0f;
   
    // downsample 4333Hz to 867Hz
    if(( BatteryConverterStatus.bit.BoostOn ) || (BoostLegBOn))
    {   
        if ( ++BoostDownSampleCount >= 5 )
        {   
            BoostDownSampleCount = 0;
            DCLinkVoltagePos = RawAdcDataPtr->st.RailVoltagePositive;
            DCLinkVoltageNeg = - RawAdcDataPtr->st.RailVoltageNegative;
     
            if( BatteryConverterStatus.bit.BoostOpenLoop )
            {
                //open loop
                dutyPos = 1.0 - RawAdcDataPtr->st.BatteryVoltageChgPos / DCLinkVoltageTarget;
                dutyNeg = 1.0 + RawAdcDataPtr->st.BatteryVoltageChgNeg / DCLinkVoltageTarget;
            }
            else
            {    
                    
                satFactor = BoostVSatFactor;
                float errorPos = DCLinkVoltageTargetNewPos - DCLinkVoltagePos;
                float errorNeg = DCLinkVoltageTargetNewNeg - DCLinkVoltageNeg;
				
                if(errorPos > 50.0f)
				{
                	errorPos = 50.0f;
				}
                else if(errorPos < -50.0f)
                {
                	errorPos = -50.0f;
                }

                if(errorNeg > 50.0f)
				{
                	errorNeg = 50.0f;
				}
			    else if(errorNeg < -50.0f)
			    {
			    	errorNeg = -50.0f;
			    }
//	                duty = SecondOrderIIRFP( DCLinkVoltageTargetNew - DCLinkVoltage, &BoostVGains );
				dutyPos = FirstOrderIIRFP( errorPos, &BoostVGainsPos );
				dutyNeg = FirstOrderIIRFP( errorNeg, &BoostVGainsNeg );
                            
                if(dutyPos > MaxDutyPos)
                {
                    dutyPos = MaxDutyPos;
                    BoostVGainsPos.X1 = MaxDutyPos * satFactor;
                }  
                else if( dutyPos < 0 )
                {
                    dutyPos = 0;
                    BoostVGainsPos.X1 = 0;
                }

                if(dutyNeg > MaxDutyNeg)
				{
					dutyNeg = MaxDutyNeg;
					BoostVGainsNeg.X1 = MaxDutyNeg * satFactor;
				}
				else if( dutyNeg < 0 )
				{
					dutyNeg = 0;
					BoostVGainsNeg.X1 = 0;
				}
				
            }
        
            BoostDutyPos = dutyPos;
            BoostDutyNeg = dutyNeg;

        }
    }
    else
    {
        // When initially told to turn the boost on, ensure that it is the very
        // first pass through this loop that the boost actually turns on.
        BoostDownSampleCount = 10;
        BoostDutyPos = 0;
        BoostDutyNeg = 0;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: RunBoost - ISR function
// *
// *    DESCRIPTION: Boost Leg load share controller
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
//#pragma CODE_SECTION( "ramfuncs" )
void BatteryConverterControl::BoostLegShare( void )//13K 26K better
{
    //Kls = 0.1/50 : (0.1/50A; 0.01/50~0.2/50)
    //Ileg_a; Ileg_b
    //duty_in
	//1.duty_a_pos = duty_in_pos - Kls*(Ileg_a - Ileg_b)
    //2.duty_b_pos = duty_in_pos + Kls*(Ileg_a - Ileg_b)

    //3.duty_a_neg = duty_in_neg - Kls*(Ileg_a - Ileg_b)
    //4.duty_b_neg = duty_in_neg + Kls*(Ileg_a - Ileg_b)
	//float Kls = 0.002;
	float temp_pos = 0.0f;
	float temp_neg = 0.0f;
	float duty_a_pos = 0.0f;
	float duty_b_pos = 0.0f;
	float duty_a_neg = 0.0f;
	float duty_b_neg = 0.0f;
    uRectifierStatus RectStatus = Rectifier.GetStatus();
	static uint16_t FlagLegshareEn = false;

	if(DynamicLoadPercent > 10.0)
	{
		FlagLegshareEn = true;
	}
	else if(DynamicLoadPercent < 7.0)
	{		
		FlagLegshareEn = false;
	}

	if( BatteryConverterStatus.bit.BoostOn && (BoostLegBOn && Rectifier.ReuseRectSTPWMEnable))	//0.both LegA/LegB on
    {
		if(FlagLegshareEn)
    	{
			//1.1 Positive
			float error_pos = -RawAdcDataPtr->st.BatteryCurrentPositive - RawAdcDataPtr->st.InputCurrent.phB; //Ileg_a is negative
			temp_pos = FirstOrderIIRFP( error_pos, &BoostLegShareGainsPos );

			if(temp_pos > MaxDutyPos)
			{
				BoostLegShareGainsPos.X1 = MaxDutyPos * BoostLegShareSatFactor;
			}
			else if(temp_pos < -MaxDutyPos)
			{
				BoostLegShareGainsPos.X1 = -MaxDutyPos * BoostLegShareSatFactor;
			}

			//1.2 Negative
			float error_neg = -RawAdcDataPtr->st.BatteryCurrentNegative + RawAdcDataPtr->st.InputCurrent.phC; //Ileg_b is negative
			temp_neg = FirstOrderIIRFP( error_neg, &BoostLegShareGainsNeg );

			if(temp_neg > MaxDutyNeg)
			{
				BoostLegShareGainsNeg.X1 = MaxDutyNeg * BoostLegShareSatFactor;
			}
			else if(temp_neg < -MaxDutyNeg)
			{
				BoostLegShareGainsNeg.X1 = -MaxDutyNeg * BoostLegShareSatFactor;
			}
    	}
    	else
    	{
    		temp_pos = 0.0;
    		temp_neg = 0.0;
			BoostLegShareGainsPos.X1 = 0.0f;
			BoostLegShareGainsNeg.X1 = 0.0f;
    	}

		if(temp_pos >= 0.0) //I.A>I.B, duty.B should improve
		{
			duty_a_pos = BoostDutyPos;
			duty_b_pos = BoostDutyPos + temp_pos;
		}
		else           //I.A<I.B, duty.A should improve
		{
			duty_a_pos = BoostDutyPos - temp_pos;
			duty_b_pos = BoostDutyPos;
		}

		if(temp_neg >= 0.0)
		{
			duty_a_neg = BoostDutyNeg;
			duty_b_neg = BoostDutyNeg + temp_neg;
		}
		else
		{
			duty_a_neg = BoostDutyNeg - temp_neg;
			duty_b_neg = BoostDutyNeg;
		}
		
		//1.3 duty limit code need optimize
		if( duty_a_pos > CurrMaxDutyPos )
		{
			duty_a_pos = CurrMaxDutyPos;
		}
		else if(duty_a_pos < 0)
		{
			duty_a_pos = 0;
		}
		
		if( duty_b_pos > CurrMaxDutyPos )
		{
			duty_b_pos = CurrMaxDutyPos;
		}
		else if(duty_b_pos < 0)
		{
			duty_b_pos = 0;
		}
		
		if( duty_a_neg > CurrMaxDutyNeg )
		{
			duty_a_neg = CurrMaxDutyNeg;
		}
		else if(duty_a_neg < 0)
		{
			duty_a_neg = 0;
		}
		
		if( duty_b_neg > CurrMaxDutyNeg )
		{
			duty_b_neg = CurrMaxDutyNeg;
		}
		else if(duty_b_neg < 0)
		{
			duty_b_neg = 0;
		}

		//Ov protect
		if(FlagHighBusP)
		{
			duty_a_pos = 0;
			duty_b_pos = 0;
		}
		if(FlagHighBusN)
		{
			duty_a_neg = 0;
			duty_b_neg = 0;
		}
		//end

		EPwm2Regs.CMPA.half.CMPA = uint16_t( duty_a_pos * BoostPWMPeriod );
		EPwm2Regs.CMPB			 = uint16_t( duty_a_neg * BoostPWMPeriod );

		EPwm3Regs.CMPA.half.CMPA = uint16_t( duty_b_pos * BoostPWMPeriod );
		EPwm3Regs.CMPB = uint16_t( duty_b_neg * BoostPWMPeriod );

    }
	else if(BatteryConverterStatus.bit.BoostOn)		//1.only LegA on
    {
		//Ov protect
		if(FlagHighBusP)
		{
			BoostDutyPos = 0.0;
		}
		if(FlagHighBusN)
		{
			BoostDutyNeg = 0.0;
		}
		//end
		
    	EPwm2Regs.CMPA.half.CMPA = uint16_t( BoostDutyPos * BoostPWMPeriod );
    	EPwm2Regs.CMPB = uint16_t( BoostDutyNeg * BoostPWMPeriod );
    }
//	    else if( BatteryConverterStatus.bit.BoostOn && Rectifier.ReuseRectSTPWMEnable )
	else if(BoostLegBOn && Rectifier.ReuseRectSTPWMEnable)	//2.only LegB on
	{
		//Ov protect
		if(FlagHighBusP)
		{
			BoostDutyPos = 0.0;
		}
		if(FlagHighBusN)
		{
			BoostDutyNeg = 0.0;
		}
		//end		
    	EPwm3Regs.CMPA.half.CMPA = uint16_t( BoostDutyPos * BoostPWMPeriod );
    	EPwm3Regs.CMPB = uint16_t( BoostDutyNeg * BoostPWMPeriod );
	}
    else
    {

    }
}

// ***********************************************************************
// *
// *    FUNCTION: RunBoostPowerMode - ISR function 
// *
// *    DESCRIPTION: battest mode, power boost controller
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
//	#pragma CODE_SECTION( "ramfuncs" )		//20171225 mask for cpu-improve
void BatteryConverterControl::RunBoostPowerMode( void )	//4.5k down cnt to 0.9k
{
}
//	{   
//	    float duty;
//	    
//	    // downsample 4500Hz to 900Hz
//	    if ( ++BoostPowerModeDownSampleCount >= 5 )
//	    {
//	        BoostPowerModeDownSampleCount = 0;
//	        
//	        if( BatteryConverterStatus.bit.BoostPowerModeOn )
//	        {    
//				//BatteryCurrentPos: '+' mean charger; '-' mean boost 
//	            float PowerError =  PowerTarget + ( BatteryCurrentPos.FastFiltered * BatteryVoltageChgPos.FastFiltered );
//	                
//	            // run compensator
//	//	            duty = SecondOrderIIRFP( PowerError , &BoostPowerModeGains );                            
//				duty = FirstOrderIIRFP( PowerError , &BoostPowerModeGains );							 
//	
//				
//	            if( duty > MaxDutyPos )
//	            {
//	                duty = MaxDutyPos;
//	                BoostPowerModeGains.X1 = MaxDutyPos * BoostPowerModeSatFactor;
//	            }  
//	            else
//	            {
//	                if ( duty < 0 ) 
//	                {
//	                    duty = 0;                        
//	                    BoostPowerModeGains.X1 = 0;
//	                }
//	            }
//	
//	
//	            uint16_t TCmpr = uint16_t( duty * BoostPWMPeriod );
//	 			EPwm2Regs.CMPA.half.CMPA = TCmpr;
//				EPwm2Regs.CMPB			 = TCmpr;
//	         }
//	    }    
//	}

// ***********************************************************************
// *
// *    FUNCTION: RunCharger - ISR function 
// *
// *    DESCRIPTION: Charger Control
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::RunCharger( void )	//1.low down 5ms,follow 9E 
{	
	float BattVoltagePos;
	float BattVoltageNeg;
		
    CalculateLimits();		//near curr loop

    if( BatteryConverterStatus.bit.ChargerOn )
    {
        BattVoltagePos = BatteryVoltageChgPos.FastFiltered;
        BattVoltageNeg = -BatteryVoltageChgNeg.FastFiltered;
		float BatChargeFFK =1.0f/(DCLinkVoltageTarget*DCLinkVoltageTarget);
        BatChargeFFGain = ChargeVoltageTarget*BatChargeFFK;
        
        if( BatteryConverterStatus.bit.ChargerOpenLoop)
        {            
            dutyPos_chg = 1.0f - ChargeVoltageTarget * OneOverDCLinkVoltPos;
            dutyNeg_chg = 1.0f - ChargeVoltageTarget * OneOverDCLinkVoltNeg;
        }
        else
        {
        	if ( BatteryConverterStatus.bit.fixedDuty )
			{
				dutyPos_chg = BattPrechargeDuty;
				dutyNeg_chg = BattPrechargeDuty;
			}
        	else
        	{
            	//1.1 Positive charge
	            MaxDutyChargePos = ChargeVoltageTarget * OneOverDCLinkVoltPos - 0.01f;// + 0.1f;	//then curr drop
				//1.2 Negative charge
				MaxDutyChargeNeg = ChargeVoltageTarget * OneOverDCLinkVoltNeg - 0.01f;// + 0.1f;	//then curr drop
				ChargeDutyLimit = MaxDutyChargePos;

				if(FlagLowCurrChg_p == false)
				{
	            	//1.1 Positive charge
					IrefPos = FirstOrderIIRFP(ChargeVoltageTarget - BattVoltagePos , &ChargeVGainsPos);
					
					if ( IrefPos > ChargeSWCurrentLimit ) 			//D = Vbat/Vbus
					{
						IrefPos = ChargeSWCurrentLimit;
						ChargeVGainsPos.X1 = IrefPos * ChargeVSatFactor;
					}

					//1).init CV/Float mode
					ChargeVGainsPos_Float.X1 = dutyPos_chg * ChargeVSatFactor_Float;					
				}
				else
				{
	            	//1.1.2 Positive charge:Float
					dutyPos_chg = FirstOrderIIRFP(ChargeVoltageTarget - BattVoltagePos , &ChargeVGainsPos_Float);
					
					if ( dutyPos_chg > MaxDutyChargePos ) 			//D = Vbat/Vbus
					{
						dutyPos_chg = MaxDutyChargePos;
						ChargeVGainsPos_Float.X1 = dutyPos_chg * ChargeVSatFactor_Float;
					}				
					
					//2).init CC mode
					IrefPos 		   = BatteryCurrentPos.SlowFiltered;
					ChargeVGainsPos.X1 = IrefPos * ChargeVSatFactor;
				}
	
				if(FlagLowCurrChg_n == false)
				{
					//1.2 Negative charge
					IrefNeg = FirstOrderIIRFP(ChargeVoltageTarget - BattVoltageNeg , &ChargeVGainsNeg);

					if ( IrefNeg > ChargeSWCurrentLimit ) 			//D = Vbat/Vbus
					{
						IrefNeg = ChargeSWCurrentLimit;
						ChargeVGainsNeg.X1 = IrefNeg * ChargeVSatFactor;
					}

					//1).init CV/Float mode
					ChargeVGainsNeg_Float.X1 = dutyNeg_chg * ChargeVSatFactor_Float;
					
				}
				else
				{
					//1.2.2 Negative charge: Float
					dutyNeg_chg = FirstOrderIIRFP(ChargeVoltageTarget - BattVoltageNeg , &ChargeVGainsNeg_Float);

					if ( dutyNeg_chg > MaxDutyChargeNeg ) 			//D = Vbat/Vbus
					{
						dutyNeg_chg = MaxDutyChargeNeg;
						ChargeVGainsNeg_Float.X1 = dutyNeg_chg * ChargeVSatFactor_Float;
					}		
					
					//2).init CC mode
					IrefNeg = BatteryCurrentNeg.SlowFiltered;
					ChargeVGainsNeg.X1 = IrefNeg * ChargeVSatFactor;					
				}

        	}
        }
    }
	else
	{
		IrefPos = 0.0f;
		IrefNeg = 0.0f;
		ChargeVGainsPos.X1	= 0.0f;
		ChargeVGainsNeg.X1	= 0.0f;
	}
    
}

// ***********************************************************************
// *
// *    FUNCTION: RunCharger - ISR function
// *
// *    DESCRIPTION: Charger Control
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void BatteryConverterControl::ChargerCurrControl( void )  //13K
{
	if( BatteryConverterStatus.bit.ChargerOn )
	{
		if ( !BatteryConverterStatus.bit.fixedDuty )
		{
			if(FlagLowCurrChg_p == false)
			{
				dutyPos_chg = FirstOrderIIRFP(((IrefPos - RawAdcDataPtr->st.BatteryCurrentPositive)*BatChargeCurrLoopGain), &OuterChargeIGainsPos );
				if (  ( BatteryVoltageChgPos.FastFiltered > (ChargeVoltageTarget +10.0f ) ) )
				{
					dutyPos_chg = 0.0f;
					OuterChargeIGainsPos.X1 = 0.0f;
				}

				dutyPos_chg -= BatChargeFFGain*(RawAdcDataPtr->st.RailVoltagePositive);

				if ( dutyPos_chg > 0.8f)			//D = Vbat/Vbus
				{
					dutyPos_chg = 0.8f;
					OuterChargeIGainsPos.X1 = dutyPos_chg * OuterChargeISatFactor;
				}
				if ( (dutyPos_chg < 0) )
				{
					dutyPos_chg = 0.0f;
				}
			}

			if(FlagLowCurrChg_n == false)
			{
				dutyNeg_chg = FirstOrderIIRFP(((IrefNeg - RawAdcDataPtr->st.BatteryCurrentNegative)*BatChargeCurrLoopGain), &OuterChargeIGainsNeg );
				if (  (-BatteryVoltageChgNeg.FastFiltered) > (ChargeVoltageTarget +10.0f  ) )
				{
					dutyNeg_chg = 0.0f;
					OuterChargeIGainsNeg.X1 = 0.0f;
				}

				dutyNeg_chg += BatChargeFFGain*(RawAdcDataPtr->st.RailVoltageNegative);

				if ( dutyNeg_chg > 0.8f ) 			//D = Vbat/Vbus
				{
					dutyNeg_chg = 0.8f;
					OuterChargeIGainsNeg.X1 = dutyNeg_chg * OuterChargeISatFactor;
				}

				if ( (dutyNeg_chg < 0))
				{
					dutyNeg_chg = 0.0f;
				}
			}

		}
		
		EPwm2Regs.CMPA.half.CMPA = uint16_t( dutyPos_chg * ChargePWMPeriod );
		EPwm2Regs.CMPB = uint16_t( dutyNeg_chg * ChargePWMPeriod );
	}
}

// ***********************************************************************
// *
// *    FUNCTION: SetChargeCurrentLimit - TSK function 
// *
// *    DESCRIPTION: charger output current, in amps.
// *
// *    ARGUMENTS: freq: charger will provide this maximum current
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::SetChargeCurrentLimit( float currentLimit )
{
    ChargeMaxCurrent = currentLimit;
}

// ***********************************************************************
// *
// *    FUNCTION: SetChargeVoltage - TSK function 
// *
// *    DESCRIPTION: set charge voltage
// *
// *    ARGUMENTS: voltage: charger will charge to this voltage level.
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::SetChargeVoltage( float voltage )
{
    ChargeVoltageTarget = voltage * 0.5f;
}

// ***********************************************************************
// *
// *    FUNCTION: SetDCLinkVoltage - TSK function 
// *
// *    DESCRIPTION: sets boost target voltage
// *
// *    ARGUMENTS: voltage: booster will regulate the link to this voltage.
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::SetDCLinkVoltage( float voltage )
{
    DCLinkVoltageTarget = voltage * 0.5f;
}

// ***********************************************************************
// *
// *    FUNCTION: CalculateLimits - periodic function 
// *
// *    DESCRIPTION: calculates pwm limits: charge current limit and 
// *            discontinuous boundary duty for both charge and discharge.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::CalculateLimits( void )	//5ms
{
    const float ChargerRampRate = 0.0025;        // = 0.5A/s @5ms
    float tempILimitduty;
    float DCLinkVoltagePos;
    float DCLinkVoltageNeg;
    float BattVoltagePos;
    float BattVoltageNeg;
    float tempMaxDuty;
    uRectifierStatus rectStat = Rectifier.GetStatus();
    
    //a2d values need an antialias filter at the a2d level
    BattVoltagePos = BatteryVoltageChgPos.SlowFiltered;
    BattVoltageNeg = - BatteryVoltageChgNeg.SlowFiltered;
//	    DCLinkVoltage = ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered );

//software current limit
    //divide by 0 protection
    if ( BattVoltagePos < 10.0 ) BattVoltagePos = 10.0;
    if ( BattVoltageNeg < 10.0 ) BattVoltageNeg = 10.0;
//	    if ( DCLinkVoltage < 20.0 ) DCLinkVoltage = 20.0;
    
    if ( !BatteryConverterStatus.bit.ChargerOn )
    {
        ChargeSWCurrentLimit = 0;    
    }    
    else
    {
        DetermineMaxChargeCurrent();
        
        // charger ramp up
        if ( ChargeSWCurrentLimit < WorkingMaxChargeCurrent )
        {
            ChargeSWCurrentLimit += ChargerRampRate;
        }
        else
        {
            ChargeSWCurrentLimit = WorkingMaxChargeCurrent;
        }               
    }


//continuous conduction boundary limit

	DCLinkVoltagePos = DCLinkVoltagePositive.FastFiltered;
	DCLinkVoltageNeg = - DCLinkVoltageNegative.FastFiltered;
			
	if ( DCLinkVoltagePos < 20.0 ) DCLinkVoltagePos = 20.0;
	if ( DCLinkVoltageNeg < 20.0 ) DCLinkVoltageNeg = 20.0;
    OneOverDCLinkVoltPos = 1.0 / DCLinkVoltagePos;
    OneOverDCLinkVoltNeg = 1.0 / DCLinkVoltageNeg;
    
	if(BatteryConverterStatus.bit.BoostPowerModeOn)
	{			
		//Bat test, Pbat control ensure no steady error  
	    tempMaxDuty = 1.03 - ( BattVoltagePos / DCLinkVoltageTarget );
	    MaxDutyPos = tempMaxDuty;
	    tempMaxDuty = 1.03 - ( BattVoltageNeg / DCLinkVoltageTarget );
	    MaxDutyNeg = tempMaxDuty;
	}
	else
	{
		//Wombat Boost ensure under DCM 0.975: steady on Vbat error under real by -5.0V~+7.0V
	    tempMaxDuty = 0.975 - ( BattVoltagePos / DCLinkVoltageTargetNewPos );
	    MaxDutyPos = tempMaxDuty;
	    tempMaxDuty = 0.975 - ( BattVoltageNeg / DCLinkVoltageTargetNewNeg );
	    MaxDutyNeg = tempMaxDuty;
	}
	
	if(MaxDutyPos > 0.7)				//Vbat_min:285/Vbus_max:800,  Dmax=1-285/8000=0.65
	{
		MaxDutyPos = 0.7;
	}
	if(MaxDutyNeg > 0.7)				//Vbat_min:285/Vbus_max:800,  Dmax=1-285/8000=0.65
	{
		MaxDutyNeg = 0.7;
	}

	CurrMaxDutyPos = 0.7f;
	CurrMaxDutyNeg = 0.7f;
}

// ***********************************************************************
// *
// *    FUNCTION: BoostOn  
// *
// *    DESCRIPTION: turns boost on
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostOn( void )	//only LegA on
{
	SetHWCurrentLimit(BattHWCurrentLimit);
	
    EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm2Regs.CMPB = 0;

	BoostVGainsPos.X1 = 0;
	BoostVGainsNeg.X1 = 0;
	BoostLegShareGainsPos.X1 = 0.0f;
	BoostLegShareGainsNeg.X1 = 0.0f;
	
    BatteryConverterStatus.bit.BoostOn = 1;
    BatteryConverterStatus.bit.BoostPowerModeOn = 0;    
    BatteryConverterStatus.bit.BoostOpenLoop = 0;
    MasterPWMOn();
    BoostPWMOn();
}

// ***********************************************************************
// *
// *    FUNCTION: BoostPWMOn
// *
// *    DESCRIPTION: turns boost PWM on
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void BatteryConverterControl::BoostPWMOn()
{
	SetHWCurrentLimit(BattHWCurrentLimit);
	
   	BoostPWMTurnOn();
}


// ***********************************************************************
// *
// *    FUNCTION: BoostPowerModeOn  
// *
// *    DESCRIPTION: turns boost on
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostPowerModeOn( void )	//Battest:for Hobbit topo: Battest mode, use Rec control Iref=50%_Iload
{
	SetHWCurrentLimit(BattHWCurrentLimit);
	
	EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm2Regs.CMPB = 0;

	BoostVGainsPos.X1 = 0;
	BoostVGainsNeg.X1 = 0;
	BoostLegShareGainsPos.X1 = 0.0f;
	BoostLegShareGainsNeg.X1 = 0.0f;
	
	BatteryConverterStatus.bit.BoostOn = 1;
	BatteryConverterStatus.bit.BoostPowerModeOn = 0;	
	BatteryConverterStatus.bit.BoostOpenLoop = 0;
	MasterPWMOn();
	BoostPWMOn();
}


// ***********************************************************************
// *
// *    FUNCTION: BoostOn  
// *
// *    DESCRIPTION: turns boost on
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostOnFF( void )
{
    float InvPower = 0;
    float InitializeDuty = 0;
    
    if( MCUStateMachine.GetStatus().bit.ForwardTransfer )
    {
        // TODO: For a distributed-bypass system, this isn't the right divisor.
        InvPower = BypassState().GetActivePower().sum / ParallelCan.TotalNumberOfUPMs;       
    }
    else
    {
        InverterPower.ActivePower.UpdateSum();
        InvPower = InverterPower.ActivePower.sum;        
    }
	
    InvPower = InvPower * 0.5f;
	
    InitializeDuty = 1e-05 * InvPower + 0.1109;		//
                
    BoostOn();
	
    if( InvPower > 10000 )
    {
		if(InitializeDuty > 0.5f)
		{
			InitializeDuty = 0.5f;
		}
		
        BoostVGainsPos.X1 = InitializeDuty * BoostVSatFactor;
        BoostVGainsNeg.X1 = InitializeDuty * BoostVSatFactor;
    } 	
}
// ***********************************************************************
// *
// *    FUNCTION: BoostOnOpenLoop  
// *
// *    DESCRIPTION: turns on boost open loop
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostOnOpenLoop( void )
{
	SetHWCurrentLimit(BattHWCurrentLimit);
	
    BatteryConverterStatus.bit.BoostOn = 1;
    BatteryConverterStatus.bit.BoostOpenLoop = 1;      
    MasterPWMOn();
    BoostPWMOn();
}

// ***********************************************************************
// *
// *    FUNCTION: BoostOff  
// *
// *    DESCRIPTION: turns off discharger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostOff( void )	//both legA/B off
{
    BatteryConverterStatus.bit.BoostOn = 0;
    BatteryConverterStatus.bit.BoostPowerModeOn = 0;
    BatteryConverterStatus.bit.BoostOpenLoop = 0;
    BoostVGainsPos.X1 = 0;
    BoostVGainsNeg.X1 = 0;
	BoostLegShareGainsPos.X1 = 0.0f;
	BoostLegShareGainsNeg.X1 = 0.0f;
	
//	    BoostPowerModeGains.X1 = 0;
    BoostPWMOff();
	if(BoostLegBOn)
	{
		BoostLegBOn = 0;
		Rectifier.ReuseRectSTPWMEnable = 0;
		BoostLegBPWMOff();
	}	
//		BoostLegBPWMOff();
}    

// ***********************************************************************
// *
// *    FUNCTION: BoostOff  
// *
// *    DESCRIPTION: turns off discharger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostOffLegA( void )	//only off legA
{
    BatteryConverterStatus.bit.BoostOn = 0;
    BatteryConverterStatus.bit.BoostPowerModeOn = 0;
    BatteryConverterStatus.bit.BoostOpenLoop = 0;
    BoostVGainsPos.X1 = 0;
    BoostVGainsNeg.X1 = 0;
	BoostLegShareGainsPos.X1 = 0.0f;
	BoostLegShareGainsNeg.X1 = 0.0f;
	
//	    BoostPowerModeGains.X1 = 0;
    BoostPWMOff();
}  

// ***********************************************************************
// *
// *    FUNCTION: ChargerOn  
// *
// *    DESCRIPTION: turns on charger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::ChargerOn( void )
{

	ChgFlagCVmode_pos = false;
	ChgFlagCVmode_neg = false;
//		SetHWCurrentLimit(80);

    EPwm2Regs.CMPA.half.CMPA = 1;
	EPwm2Regs.CMPB = 1;
	IrefPos = 0.0f;
	IrefNeg = 0.0f;
	dutyPos_chg = 0.0f;
	dutyNeg_chg = 0.0f;

    ChargerCurrentReference = 0;
    BatteryConverterStatus.bit.ChargerOn = 1;
    BatteryConverterStatus.bit.ChargerOpenLoop = 0;  
    MasterPWMOn();
    ChargerPWMOn();
}    

// ***********************************************************************
// *
// *    FUNCTION: ChargerFixedDutyOn  
// *
// *    DESCRIPTION: turns on charger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::ChargerFixedDutyOn( void )
{
	//for jira558,new add clr pwm reg to about 0%
    EPwm2Regs.CMPA.half.CMPA = 1;
	EPwm2Regs.CMPB = 1;
	dutyPos_chg = 0.0f;
	dutyNeg_chg = 0.0f;
	
    BatteryConverterStatus.bit.fixedDuty = 1;
    BatteryConverterStatus.bit.ChargerOn = 1;
    BatteryConverterStatus.bit.ChargerOpenLoop = 0;  
    MasterPWMOn();
    ChargerPWMOn();
    
//   BatteryConverterStatus.bit.fixedDuty = 1;
//   ChargerOn();
}

// ***********************************************************************
// *
// *    FUNCTION: ChargerFixedDutyOn  
// *
// *    DESCRIPTION: turns on charger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::ChargerFixedDutyPosOn( void )
{
	//for jira558,new add clr pwm reg to about 0%
    EPwm2Regs.CMPA.half.CMPA = 1;
	EPwm2Regs.CMPB = 1;
	dutyPos_chg = 0.0f;
	dutyNeg_chg = 0.0f;
	
    BatteryConverterStatus.bit.fixedDuty = 1;
    BatteryConverterStatus.bit.ChargerOn = 1;
    BatteryConverterStatus.bit.ChargerPosOn = 1;
    BatteryConverterStatus.bit.ChargerOpenLoop = 0;  
    MasterPWMOn();
    ChargerPWMPosOn();
    
//   BatteryConverterStatus.bit.fixedDuty = 1;
//   ChargerOn();
}

 
 // ***********************************************************************
 // *
 // *    FUNCTION: ChargerFixedDutyNegOn  
 // *
 // *    DESCRIPTION: turns on charger
 // *
 // *    ARGUMENTS: 
 // *
 // *    RETURNS: 
 // *
 // ***********************************************************************
 void BatteryConverterControl::ChargerFixedDutyNegOn( void )
 {
     //for jira558,new add clr pwm reg to about 0%
     EPwm2Regs.CMPA.half.CMPA = 1;
     EPwm2Regs.CMPB = 1;
	 dutyPos_chg = 0.0f;
	 dutyNeg_chg = 0.0f;
     
     BatteryConverterStatus.bit.fixedDuty = 1;
     BatteryConverterStatus.bit.ChargerOn = 1;
     BatteryConverterStatus.bit.ChargerNegOn = 1;
     BatteryConverterStatus.bit.ChargerOpenLoop = 0;  
     MasterPWMOn();
     ChargerPWMNegOn();
     
 //   BatteryConverterStatus.bit.fixedDuty = 1;
 //   ChargerOn();
 }

 // ***********************************************************************
// *
// *    FUNCTION: ChargerFixedDutyOff  
// *
// *    DESCRIPTION: turns off charger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************       
void BatteryConverterControl::ChargerFixedDutyOff( void)
{
    BatteryConverterStatus.bit.fixedDuty = 0;
    BatteryConverterStatus.bit.ChargerOn = 0;
    BatteryConverterStatus.bit.ChargerPosOn = 0;
    BatteryConverterStatus.bit.ChargerNegOn = 0;
    ChargerOff();   
} 

  // ***********************************************************************
// *
// *    FUNCTION: ChargerFixedDutyPosOff  
// *
// *    DESCRIPTION: turns off charger pos
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************       
void BatteryConverterControl::ChargerFixedDutyPosOff( void)
{
    BatteryConverterStatus.bit.ChargerPosOn = 0;
    ChargerPWMPosOff();   
} 

  // ***********************************************************************
// *
// *    FUNCTION: ChargerFixedDutyNegOff  
// *
// *    DESCRIPTION: turns off charger Neg
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************       
void BatteryConverterControl::ChargerFixedDutyNegOff( void)
{
    BatteryConverterStatus.bit.ChargerNegOn = 0;
    ChargerPWMNegOff();   
} 

// ***********************************************************************
// *
// *    FUNCTION: ChargerOnOpenLoop  
// *
// *    DESCRIPTION: turns charger on open loop
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::ChargerOnOpenLoop( void )
{
    EPwm2Regs.CMPA.half.CMPA = 1;
	EPwm2Regs.CMPB = 1;
	dutyPos_chg = 0.0f;
	dutyNeg_chg = 0.0f;
	
    BatteryConverterStatus.bit.ChargerOn = 1;
    BatteryConverterStatus.bit.ChargerOpenLoop = 1;  
    MasterPWMOn();
    ChargerPWMOn();
}
// ***********************************************************************
// *
// *    FUNCTION: ChargerOff  
// *
// *    DESCRIPTION: turns off charger
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::ChargerOff( void )
{
    ChargerPWMOff();
    BatteryConverterStatus.bit.ChargerOn = 0;
    BatteryConverterStatus.bit.ChargerOpenLoop = 0;
    BatteryConverterStatus.bit.fixedDuty = 0;
	ChargeVGainsPos.X1	= 0.0f;
	OuterChargeIGainsPos.X1 = 0.0f;
	ChargeVGainsNeg.X1	= 0.0f;
	OuterChargeIGainsNeg.X1 = 0.0f;

	ChargeVGainsPos_Float.X1	= 0.0f;
	ChargeVGainsNeg_Float.X1	= 0.0f;
	
}    

// ***********************************************************************
// *
// *    FUNCTION: SetHWCurrentLimit  
// *
// *    DESCRIPTION: sets battery converter hardware current limit
// *
// *    ARGUMENTS: currentLimit(amps)- the hardware current limit you want
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::SetHWCurrentLimit( uint16_t currentLimit )
{
    float tempDuty;
    const float CurrentLimPWMPeriod = ( CPUFrequency / PWMFrequency );

    //Puck/20131220/add
	switch(UPMSystem)
	{

	    case HV_20K:
	    case HV_30K:
	    	// 0.00745108 = 0.625/50 * 200/202 * 10/(10+2.49+4.12) * (100+10)/100 /3.3 * 3
	    	tempDuty = (1.5 + ( (float)currentLimit * 0.00745108 ))/3.0;
	    	break;
	
		//windy/20160822/add HV 40k for 3c3pro
	   	case HV_40K:
	   	// 0.00372554 = 0.625/50 * 200/202 * 10/(10+2.49+4.12) * (100+10)/100 /3.3 * 3*0.5
	   	tempDuty = (1.5 + ( (float)currentLimit * 0.00372554))/3.0;
		break;

	   default:
			tempDuty = (1.5 + ( (float)currentLimit * 0.00745108 ))/3.0;
			break;
	}		

    ECap6Regs.CAP4 = (uint16_t)( tempDuty * CurrentLimPWMPeriod );

    //EPwm6Regs.CMPB = (uint16_t)( tempDuty * CurrentLimPWMPeriod );
}

// ***********************************************************************
// *
// *    FUNCTION: DetermineMaxChargeCurrent 
// *
// *    DESCRIPTION: Figures out how much charge current is available based
// *                 on current input power and max input power
// *
// *    ARGUMENTS: none 
// *
// *    RETURNS: none
// *
// ***********************************************************************
void BatteryConverterControl::DetermineMaxChargeCurrent( void )
{
    float rectifierCurrentMargin = 2.0;
    
    // Goldilocks may need to deal with rectifierCurrentMargin
    rectifierCurrentMargin = Rectifier.GetRectifierCurrentMargin();

    //add end 
    // calculate maximum available power, use max input current - margin 
    float maxRectifierCurrent = Rectifier.GetMaxRMSCurrent();//eep121
    maxRectifierCurrent -= rectifierCurrentMargin;
    
    // assume unity power factor
    float maxRectifierPower = ScreenMeters.InputVoltageRMS.phA * maxRectifierCurrent;
    maxRectifierPower += ScreenMeters.InputVoltageRMS.phB * maxRectifierCurrent;
    maxRectifierPower += ScreenMeters.InputVoltageRMS.phC * maxRectifierCurrent;
    
    // subtract current rectifier power
    maxRectifierPower -= ScreenMeters.InputPower.sum;
    
    if ( maxRectifierPower < 0 )
    {
        // too much input current, reduce charger
        if ( WorkingMaxChargeCurrent > 0 )
        {
            float battPower = ScreenMeters.BatteryVoltage * ScreenMeters.BatteryCurrent;
            WorkingMaxChargeCurrent = ( battPower + maxRectifierPower ) / ScreenMeters.BatteryVoltage; 
        }
        // nothing we can do, already 0
    }
    else
    {
        // there's available input, can go up
        if ( WorkingMaxChargeCurrent < ChargeMaxCurrent )
        {
            float battPower = ScreenMeters.BatteryVoltage * ScreenMeters.BatteryCurrent;
            WorkingMaxChargeCurrent = ( battPower + maxRectifierPower ) / ScreenMeters.BatteryVoltage; 
        }
    }
    // check against max
    if ( WorkingMaxChargeCurrent > ChargeMaxCurrent )
    {
        WorkingMaxChargeCurrent = ChargeMaxCurrent;
    }        
}

// ***********************************************************************
// *
// *    FUNCTION: BoostTargetCal - 20ms task function 
// *
// *    DESCRIPTION: loop for vbus_target
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BoostTargetNewCal( void )	//20ms
{
	float KfwBatBoost = 0.1;
	float ErrVoltPos;
	float ErrVoltNeg;
	float DCLinkVoltageSlowPos;
	float DCLinkVoltageSlowNeg;
    float temp; 


    if( BatteryConverterStatus.bit.BoostOn )
    {      
	    DCLinkVoltageSlowPos = DCLinkVoltagePositive.FastFiltered;
	    DCLinkVoltageSlowNeg = - DCLinkVoltageNegative.FastFiltered;

	    //Pos loop
		ErrVoltPos = DCLinkVoltageTarget - DCLinkVoltageSlowPos;
		
		if(abs(ErrVoltPos) < 0.5f)		//steady, 0.5V
		{
			ErrVoltPos = 0.0f;
		}
		else
		{
			if(ErrVoltPos > 10.0f)
			{
				ErrVoltPos = 10.0f;
			}
			else if(ErrVoltPos < -10.0f)
			{
				ErrVoltPos = -10.0f;
			}
		}

		//Step max about 0.4V/20ms (10.0V*0.04)
		temp = KfwBatBoost*(2.0*ErrVoltPos - 1.0*ErrVoltPos_1);
		ErrVoltPos_1 = ErrVoltPos;

		DCLinkVoltageTargetNewPos = DCLinkVoltageTargetNewPos + temp;

		//max compen +30V,  bus+- each +15V, meet 170%load/32string bat
		if(DCLinkVoltageTargetNewPos > (DCLinkVoltageTarget+30.0))		//15.0: Err_Vbat:+4/-4V
		{
			DCLinkVoltageTargetNewPos = DCLinkVoltageTarget+30.0;
		}
		else if(DCLinkVoltageTargetNewPos < DCLinkVoltageTarget)
		{
			DCLinkVoltageTargetNewPos = DCLinkVoltageTarget;
		}

		//Neg loop
		ErrVoltNeg = DCLinkVoltageTarget - DCLinkVoltageSlowNeg;
		if(abs(ErrVoltNeg) < 0.5f)		//steady, 0.5V
		{
			ErrVoltNeg = 0.0f;
		}
		else
		{
			if(ErrVoltNeg > 10.0f)
			{
				ErrVoltNeg = 10.0f;
			}
			else if(ErrVoltNeg < -10.0f)
			{
				ErrVoltNeg = -10.0f;
			}
		}
			
		//Step max about 0.4V/20ms (10.0V*0.04)
		temp = KfwBatBoost*(2.0*ErrVoltNeg - 1.0*ErrVoltNeg_1);
		ErrVoltNeg_1 = ErrVoltNeg;

		DCLinkVoltageTargetNewNeg = DCLinkVoltageTargetNewNeg + temp;
	
		//max compen +30V,  bus+- each +15V, meet 170%load/32string bat
		if(DCLinkVoltageTargetNewNeg > (DCLinkVoltageTarget+30.0))		//15.0: Err_Vbat:+4/-4V
		{
			DCLinkVoltageTargetNewNeg = DCLinkVoltageTarget+30.0;
		}
		else if(DCLinkVoltageTargetNewNeg < DCLinkVoltageTarget)
		{
			DCLinkVoltageTargetNewNeg = DCLinkVoltageTarget;
		}

    }
	else
	{
		DCLinkVoltageTargetNewPos = DCLinkVoltageTarget;
		ErrVoltPos   = 0.0f;
		ErrVoltPos_1 = 0.0f;
		DCLinkVoltageTargetNewNeg = DCLinkVoltageTarget;
		ErrVoltNeg   = 0.0f;
		ErrVoltNeg_1 = 0.0f;
	}

}


// ***********************************************************************
// *
// *    FUNCTION: BatConverter20msTask - 20ms task function 
// *
// *    DESCRIPTION: 20ms task for 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BatteryConverterControl::BatConverter20msTask( void )	//20ms
{
    eAbmState ABMState = Abm().GetState(); 
	static uint16_t tempCntP = 0;
	static uint16_t tempCntN = 0;
	
	BoostTargetNewCal();

	if(ABMState == ABM_FLOAT)
	{
		if(FlagLowCurrChg_p == false)
		{
			if(BatteryCurrentPos.SlowFiltered < 3.0f)
			{
				if(tempCntP++ >= 250)		//1s
				{
					tempCntP = 0;
					FlagLowCurrChg_p = true;
				}
			}
			else
			{
				tempCntP = 0;
			}
		}
		else
		{
			if(BatteryCurrentPos.SlowFiltered > 5.0f)
			{
				if(tempCntP++ >= 5)		
				{
					tempCntP = 0;
					FlagLowCurrChg_p = false;
				}
			}	
			else
			{
				tempCntP = 0;
			}
		}

		if(FlagLowCurrChg_n == false)
		{				
			if(BatteryCurrentNeg.SlowFiltered < 3.0f)
			{
				if(tempCntN++ >= 250)
				{
					tempCntN = 0;
					FlagLowCurrChg_n = true;
				}
			}
			else
			{
				tempCntN = 0;	
			}
		}
		else
		{
			if(BatteryCurrentNeg.SlowFiltered > 5.0f)
			{
				if(tempCntN++ >= 5)
				{
					tempCntN = 0;
					FlagLowCurrChg_n = false;
				}
			}		
			else
			{
				tempCntN = 0;
			}			
		}
	}
	else
	{
		FlagLowCurrChg_p = false;
		FlagLowCurrChg_n = false;
		tempCntP = 0;
		tempCntN = 0;
	}


				
}

// ******************************************************************************************************
// *            End of BatteryConverter.cpp
// ******************************************************************************************************
