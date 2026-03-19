// ******************************************************************************************************
// *            Rectifier.cpp
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
// *    FILE NAME: Rectifier.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 3/30/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "RectifierStateControl.h"  
#include "Constants.h"    
#include "F28335Port.h"  
#include "Meters.h"
#include "Adc.h"
#include "Alarms.h"
#include "Eeprom_Map.h"
#include "NB_Funcs.h"
#include "BatteryStateControl.h"
#include "DQPhaseLockLoop.h"
#include "C28x_FPU_FastRTS.h"
#include "ACMeter.h"
#include "Abm.h"
#include "IOexpansion.h"
#include "Spi_Task.h"
#include "BodeScan.h"
#include "InverterControl.h"
#include "complex.h"
#include "ParallelCan.h"
#include "Alarms_AC.h"
#include "FCTState.h"
#include "complex.h"
#include "ControlFuncs.h"
#include <algorithm>
#include "Version.h"
#include "InverterControl.h"
#include "ParallelCan.h"

using namespace std;

extern "C"
{
    void InitRec(void);    
}

// control structure coefficients                               B0,             B1,             B2,             A1,                 A2,         X1,     X2                        
// WARNING: All of these filters are mechanized as first-order filters.  Any extension
// to second-order filtering must verify that the appropriate call sites use 
// SecondOrderIIRFP() vice FirstOrderIIRFP().
//HV 20K Rec.Control coefficients                               B0,             B1,       B2,         A1,          A2,        X1,     X2
//	const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_20KVA_HV = { 0.0095,    -0.843023,     -1.0,      0  }; //B0 B1 A1 X1
//karl 3.15. tune2
//	const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_20KVA_HV = { 0.001796735501090*3.0f,  	-0.93273,  		-1.0,      0  }; //B0 B1 A1 X1
// 3.25, 13k
const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_20KVA_HV = { 0.0025f,  	-0.93273,  		-1.0,      0  }; //B0 B1 A1 X1
const stFirstOrderIIRFP RectifierOffsetCoefficients_20KVA_HV      = { 0.01331,    	0.046051,      	-0.97832,  0  };
// Milder gains for even more phase margin: v0.70 (HRS-like).
// This is a ladder of poles and zeros, spaced 1/6th of a decade apart, that form a -10 dB/decade lag compensator with
// corner frequencies from 50 Hz to about 2 kHz, while only consuming ~30 degrees of lag
// 988.7=1/1.011399061161718e-003
// 285.7=1/3.5e-003
// 160=1/6.25e-003
const stSecondOrderIIRFP RectifierCurrentLoopCoefficients1_20KVA_HV = {2.3684e-3,  -1.193598461190070,   0.341661212345357,  -1.409067311462108,   0.487573232902253,  0, 0 };
//4.14
const stSecondOrderIIRFP RectifierCurrentLoopCoeffiNewRec_20KVA_HV = {0.0050f,  -1.456057335444714,   0.526849011725288,  -1.541621087495845,   0.541621087495845, 0, 0};
const stSecondOrderIIRFP RectifierCurrentLoopCoefficients2_20KVA_HV_LightLoad = {0.0042f,  -1.456057335444714,   0.526849011725288,  -1.541621087495845,   0.541621087495845, 0, 0};

// 2nd-order unity gain butterworth filter, 350 Hz corner frequency at 18 kHz sample rate
const stSecondOrderIIRFP RectifierVoltageFilterGains_20KVA_HV       = {3.4312384e-3,    2, 1,    -1.82759957,  0.8413245246,   0, 0};
// First-order high-pass filter, at 2 Hz to auto-zero the input voltage measurement.
// This eliminates any DC offset in the measurement.
const stFirstOrderIIRFP RectifierAutoZeroFilterGains_20KVA_HV      = {1.0, -1.0, -0.999650995, 0};

//HV 40K Rec.Control coefficients
//3.26
//	const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_40KVA_HV = { 0.001796735501090*2.0f,  	-0.93273,  		-1.0,      0  }; //B0 B1 A1 X1
const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_40KVA_HV = { 0.0045f,  	-0.93273,  		-1.0,      0  }; //B0 B1 A1 X1
const stFirstOrderIIRFP RectifierOffsetCoefficients_40KVA_HV      = { 0.0065,         0,        	-0.979274,   0   };
const stSecondOrderIIRFP RectifierCurrentLoopCoefficients1_40KVA_HV = { 8.2e-003,   -1.379586623818239,    4.662765742648666e-001,   -1.552924463103310,    5.974681418264173e-001, 0, 0 };
//7.5
const stSecondOrderIIRFP RectifierCurrentLoopCoeffiNewRec_40KVA_HV = {0.0016f,  -1.456057335444714,   0.526849011725288,  -1.541621087495845,   0.541621087495845, 0, 0};
const stSecondOrderIIRFP RectifierCurrentLoopCoefficients2_40KVA_HV_LightLoad = {0.0042f*0.35f,  -1.456057335444714,   0.526849011725288,  -1.541621087495845,   0.541621087495845, 0, 0};

const stSecondOrderIIRFP RectifierVoltageFilterGains_40KVA_HV       = {3.4312384e-3,    2, 1,    -1.82759957,  0.8413245246,   0, 0};
const stFirstOrderIIRFP RectifierAutoZeroFilterGains_40KVA_HV      = {1.0, -1.0, -0.999650995, 0};

//Hobbit 30K
const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_30KVA_HV = { 0.00351f,  	-0.92681,  		-1.0,      0  }; //B0 B1 A1 X1
const stSecondOrderIIRFP RectifierCurrentLoopCoeffiNewRec_30KVA_HV = {0.004530518079447,  -1.447127101352568,	0.523007764189569, -1.468097120271033,   0.468097120271033, 0, 0};

// 2nd-order unity gain butterworth filter, 25 Hz corner frequency at 2.25 kHz sample rate
  const stSecondOrderIIRFP RectifierFrequencyFilterGains = {7.5224e-5,   2, 1,   -1.97532, 0.97562,   0, 0};
  // Resonant peak control filters.  In order of increasing harmonic, defined for 3, 5, 7, and 9 harmonics of 50 Hz.
  // All are tuned for fsw/nCompensators{AB,G}
  // the others are tuned for fsw/2.
  // Each filter has -20 dB broadband gain, with +20 dB gain at the resonant frequency.
  const stSecondOrderIIRFP ResonantFilterGains[] = {
    {  1.008245468139648e-1, -1.9733528,  9.8269659e-001,  -1.9888566,   9.9981171e-001, 0, 0},
    {  1.013270258903503e-1, -1.9456164,  9.7139883e-001,  -1.9693079,   9.9968761e-001, 0, 0},
    {  1.017888903617859e-1, -1.9101579,  9.6033680e-001,  -1.9401691,   9.9956477e-001, 0, 0},
    {  1.021110653877258e-1, -1.8701543,  9.5022321e-001,  -1.9015844,   9.9944407e-001, 0, 0},
};
//powershare filter 2Hz
//	0.00695 z + 0.00695
//	---------------------
//	     z - 0.9861
const stFirstOrderIIRFP RectifierFilterPowershare         = {0.00695, 1, -0.9861, 0}; //B0,B1,A1,X1
const stSecondOrderIIRFP NullCoefficients  = { 0,              0,          0,          0,          0,          0,      0   };
//For line ripple: unbalance load & RCD load
const stSecondOrderIIRFP RectifierLoopVoFilterCoef_1th50Hz = { 0.907914269701868,	-1.866224666826336,	0.993005516372777,	-1.694372005481245,	0.809478147909384, 0, 0}; 
const stSecondOrderIIRFP RectifierLoopVoFilterCoef_3th50Hz = { 0.743047222843790,	-0.919450453155625,	0.978484825682528,	-0.733829742951290,	0.520742292353521, 0, 0}; 
const stSecondOrderIIRFP RectifierLoopVoFilterCoef_1th60Hz = { 0.892893220906303,	-1.811831853991719,	0.991727258929397,	-1.617772379851304,	0.778399767392351, 0, 0};      
const stSecondOrderIIRFP RectifierLoopVoFilterCoef_3th60Hz = { 0.703601029540990,	-0.518824804916224,	0.974237660971302,	-0.433002870109533,	0.457032854037078, 0, 0};      

uint16_t RectHWCurrentLimit = 0;
uint16_t OptimizeInputPFCunt = 0;   
uint16_t RectHWCurrentLimitRealTime = 0;

float MaxRectifierOffset     = 5.0f; //35.0f;
//const float MaxRectifierOffsetFast = 50.0f;
float ScaledIrefMin = -0.1087f; //-25.0A /230.0V;  -0.00434f;  //-1.0A /230.0V
float VoltageForwardGain = 1.0; 
float RectVoltageLoopGain = 1.0f;
float RectCurrentLoopGain = 1.0f;
float RectCurrentLoopGainReal =1.0f;
uint16_t  THDiStartFlag = 0;
uint16_t  THDiCount = 0;	
uint16_t  FlagUnblanceLoad = 0;
uint16_t  LightloadFlag = 0;
uint16_t  LightloadCnt = 0;
float DynamicLoadPercent = 0.0f;
float DynamicLoadPercentRec = 0.0f;
float VoltNormRecVa = 1.0f;
float VoltNormRecVb = 1.0f;
float VoltNormRecVc = 1.0f;
float VoltNormRecVa_Part1 = 1.0f;
float VoltNormRecVb_Part1 = 1.0f;
float VoltNormRecVc_Part1 = 1.0f;

float  KvfwRec = 3.0f;//1.0f; 
uint16_t  RecOnPhase = 0; 		//1:A_phase; 2:B_phase;  3:C_phase; other all phase

eMCUState previous_state = INITIALIZATION_STATE;
extern float ScaledIrefBattestSet;

uint16_t FlagHighBusP = false;
uint16_t FlagHighBusN = false;
extern uint16_t InterfaceBoardRevID;
float RailVoltagePositiveFastFilt = 0.0f;
float RailVoltageNegativeFastFilt = 0.0f;
extern float BatChargeCurrLoopGain;

extern float utilityPhaseError;
extern float utilityPhaseErrorLimit;
extern float DebugFloat[6];
float MaxRMSInputCurrentEEValue = 0.0;
//Puck/20120215, add
void InitRec(void)
{
    Rectifier.Init();
}

// ***********************************************************************
// *
// *    FUNCTION: RECTIFIER_init 
// *
// *    DESCRIPTION: initializes inverter data to known default 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS:
// *
// ***********************************************************************
void RectifierControl::Init( void )
{
    // PWM period
    PWMPeriod = ( CPUFrequency / PWMFrequency ) / 2L; 
            
    // clear status
    RectifierStatus.all = 0;
            
    DCLinkReference = 300;//380.0;  // EEP value, gets over-written
    RectifierGenInputCurrentMax = 0.833333333;
    RectifierInputCurrentMax = 0.833333333;
    RectifierWalkInRate = 0.833333333;
    RectifierWalkInDelay = 0;
    VoltageNormFactor = 0.002;
    VoltNormRecVa = 1.0f;
            
    // Initialize outer voltage loop
    LinkVoltageTableP = *RectifierVoltageLoopCoefficients_ptr;
    LinkVoltageTableN = *RectifierVoltageLoopCoefficients_ptr;
    // can't run this line from _c_int00, floating point library not copied to RAM yet.
    LinkVoltageSaturationFactor = 1.0f / ( LinkVoltageTableP.B1 - LinkVoltageTableP.A1 );
            
   // CurrentCmd = 0;
    CurrentCmdP = 0;
    CurrentCmdN = 0;
    CurrentCmd_A = 0;
    CurrentCmd_B = 0;
    CurrentCmd_C = 0;
    // Initialize link offset controller
  //  LinkOffsetTable = *RectifierOffsetCoefficients_ptr;
  //  FastLinkOffsetTable = *FastRectifierOffsetCoefficients_ptr;
//    OffsetCmd = 0;
//    OffsetCmdNewTopoA = 0;
//    OffsetCmdNewTopoB = 0;
//    OffsetCmdNewTopoC = 0;
    // Initialize current controller to zeros, off state
    /*PFCpha1 = NullCoefficients;
    PFCphb1 = NullCoefficients;
    PFCphc1 = NullCoefficients;

    PFCpha2 = NullCoefficients;
    PFCphb2 = NullCoefficients;
    PFCphc2 = NullCoefficients;*/

//	    PFCpha1 = *RectifierCurrentLoopCoefficients1_ptr;
//	    PFCphb1 = *RectifierCurrentLoopCoefficients1_ptr;
//	    PFCphc1 = *RectifierCurrentLoopCoefficients1_ptr;

    PFCpha2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCpha2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;

    PFCISatFactor2.phA = 1.0f / ( ( PFCpha2P.B1 + PFCpha2P.B2 ) - ( PFCpha2P.A1 + PFCpha2P.A2 ) );
    PFCISatFactor2.phB = 1.0f / ( ( PFCphb2P.B1 + PFCphb2P.B2 ) - ( PFCphb2P.A1 + PFCphb2P.A2 ) );
    PFCISatFactor2.phC = 1.0f / ( ( PFCphc2P.B1 + PFCphc2P.B2 ) - ( PFCphc2P.A1 + PFCphc2P.A2 ) );
    SineRefGain = 0.0;
	UtilityGain = 1.0 - SineRefGain;
	SineRefForwardEnabled = 0;

    SineRefGain_OptimizeInputPF = 0.0;
    UtilityGain_OptimizeInputPF = 1.0 - SineRefGain_OptimizeInputPF;
    RectifierAngleOffset = 0.0;

    ReadyForPI = 0;
    ReadyForOffset =0;
    PLLDownSampleCount = 0;
            
    FastOffsetCnt = 0;
    SlowOffsetCnt = 0;
    DCLinkRampCurrent = 2.0;//15.0;
	RectiferCurrentMargin = 6.0;

    // Initialize resonant peak controller gains
    FastFilteredFrequencyTable = RectifierFrequencyFilterGains;
    // second harmonic, nominal frequency of 55 Hz, 10204 Hz angle-step
    //TODO:update 10204 to variable depend on system type?,does ResonantPeakFilter used?
//	    if( (HV_20K == UPMSystem) || (HV_30K == UPMSystem) ||(HV_40K == UPMSystem) )
//	    {
//	        RectifierStatus.bit.UseResonantComp = false;
//	        RectifierStatus.bit.UseDynamicRCGains = false;
//	    }
//	    else
//	    {
//	        RectifierStatus.bit.UseResonantComp = true;
//	        RectifierStatus.bit.UseDynamicRCGains = true;
//	    }
//	RectifierStatus.bit.UseResonantComp = true;
	RectifierStatus.bit.UseDynamicRCGains = true;
	ReadyForPIRef = 0;
	
    CalculateResonantPeakFilterGains(50*2 * 2*PI / PWMFrequency);
	RectifierFilterPowershrP = RectifierFilterPowershare;
	RectifierFilterPowershrN = RectifierFilterPowershare;
    NeutralLoss.SetState( false );
	
	MaxRectifierOffset = 5.0f; //35.0f;
	Rectifier.BoostReuseRectST = 0;
	SetRectifierFilterLoopVFil(OutNomFreq);

	KvfwRec = 3.0f;
}
// ***********************************************************************
// *
// *    FUNCTION: RectifierControl::Run_ISR -- ISR function 
// *
// *    DESCRIPTION: Rectifier PFC controller
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
uint16_t CurrCntLoop = 0;
#pragma CODE_SECTION( "ramfuncs" )
void RectifierControl::Run_ISR( void )
{
    //
    // * This function runs from DMA ISR, no BIOS calls allowed
    //
    float tempA;
    float tempB;
    float tempC;
	//About 1.1kHz filter
	RailVoltagePositiveFastFilt = (RailVoltagePositiveFastFilt*0.6f) + (RawAdcDataPtr->st.RailVoltagePositive*0.4f);
	RailVoltageNegativeFastFilt = (RailVoltageNegativeFastFilt*0.6f) + (RawAdcDataPtr->st.RailVoltageNegative*0.4f);

//	    UtilityPLL.SineRef.UpdateAngle();
	//1.Smooth filter on CurrCmd
    CurrCntLoop++;
	
	float tempCurrentCmdP = 0.0f;
	float tempCurrentCmdN = 0.0f;
	tempCurrentCmdP = CurrentCmdP_Delta*CurrCntLoop + CurrentCmdP_1;
	tempCurrentCmdN = CurrentCmdN_Delta*CurrCntLoop + CurrentCmdN_1;		
	
	//Load off dynamic high bus protect begin
	float VbusHighOff = (DCLinkVoltageHalf + 28.0f);
	float VbusHighBack = (DCLinkVoltageHalf + 10.0f);
	if(RailVoltagePositiveFastFilt > VbusHighOff) //360+30
	{
		FlagHighBusP = true;		
	}
	else if(RailVoltagePositiveFastFilt < VbusHighBack)
	{
		FlagHighBusP = false;
	}

	if((-RailVoltageNegativeFastFilt) > VbusHighOff)	//360+30
	{
		FlagHighBusN = true;
	}
	else if((-RailVoltageNegativeFastFilt) < VbusHighBack)
	{
		FlagHighBusN = false;
	}		
	//end

    if ( RectifierStatus.bit.RectifierOnNormal )
    {
	    RectiferResonantCompensateControl();
        RecCtrlThreePhase();

		if(RawAdcDataPtr->st.InputVoltage.phA > 0.0f)
		{
			CurrentCmd_A = tempCurrentCmdP;//(CurrentCmdP-CurrentCmdP_1)*(float)CurrCntLoop/(float)30.0f + CurrentCmdP_1;	//26k/0.89k= 6*5			
			RectifierReference.abc.phA = RectifierSineRef.phA * CurrentCmd_A;			
			tempA = SecondOrderIIRFP(((RectifierReference.abc.phA - RectifierCurrent.abc.phA)*VoltNormRecVa), &PFCpha2P );	//temp			
			RectifierOut.abc.phA = RectifierOut.abc.phA - tempA;		//temp++ -> duty++			
			if ( tempA > 0.0 )
			{
				PFCpha2P.X1 = 0.0;
				PFCpha2P.X2 = 0.0;
			}
			else if ( tempA < -1.0 )
			{
				tempA = -1.0;
				PFCpha2P.X1 = tempA * PFCISatFactor2.phA;
				PFCpha2P.X2 = PFCpha2P.X1;
			}

			LimitRectifierDuty( &RectifierOut.abc.phA );	//0~1: 100%~0% duty
			
			if(FlagHighBusP == true)
			{
				//off pwm
				SetRect3LvlPwm( 0.0f, PWMPeriod, 1, 1 );	//1~0: 100%~0% duty
			}
			else
			{
				SetRect3LvlPwm( (1.0 - RectifierOut.abc.phA), PWMPeriod, 1, 1 );	//1~0: 100%~0% duty
			}				
		}
		else
		{
			CurrentCmd_A = tempCurrentCmdN;

			RectifierReference.abc.phA = RectifierSineRef.phA * CurrentCmd_A;
			tempA = SecondOrderIIRFP(((RectifierReference.abc.phA - RectifierCurrent.abc.phA)*VoltNormRecVa), &PFCpha2N );
			RectifierOut.abc.phA = RectifierOut.abc.phA - tempA;
			if ( tempA > 0.0 )
			{
				PFCpha2N.X1 = 0.0;
				PFCpha2N.X2 = 0.0;
			}
			else if ( tempA < -1.0 )
			{
				tempA = -1.0;
				PFCpha2N.X1 = tempA * PFCISatFactor2.phA;
				PFCpha2N.X2 = PFCpha2N.X1;
			}

			LimitRectifierDuty( &RectifierOut.abc.phA );

			if(FlagHighBusN == true)
			{
				SetRect3LvlPwm( (0.0f), PWMPeriod, 1, 0 );
			}
			else
			{
				SetRect3LvlPwm( (-(1.0 - RectifierOut.abc.phA)), PWMPeriod, 1, 0 );
			}
		}

		if( !Rectifier.BoostReuseRectST )
		{
			if(RawAdcDataPtr->st.InputVoltage.phB > 0.0f)
			{
				CurrentCmd_B = tempCurrentCmdP;
				RectifierReference.abc.phB = RectifierSineRef.phB * CurrentCmd_B;
				tempB = SecondOrderIIRFP(((RectifierReference.abc.phB - RectifierCurrent.abc.phB)*VoltNormRecVb), &PFCphb2P );
				RectifierOut.abc.phB = RectifierOut.abc.phB - tempB;
				if ( tempB > 0.0 )
				{
					PFCphb2P.X1 = 0.0;
					PFCphb2P.X2 = 0.0;
				}
				else if ( tempB < -1.0 )
				{
					tempB = -1.0;
					PFCphb2P.X1 = tempB * PFCISatFactor2.phB;
					PFCphb2P.X2 = PFCphb2P.X1;
				}

				LimitRectifierDuty( &RectifierOut.abc.phB );

				if(FlagHighBusP == true)
				{
					//off pwm
					SetRect3LvlPwm( 0.0f, PWMPeriod, 2, 1 );	//1~0: 100%~0% duty
				}
				else
				{
					SetRect3LvlPwm( (1.0 - RectifierOut.abc.phB), PWMPeriod, 2, 1 );	//1~0: 100%~0% duty
				}

			}
			else
			{
				CurrentCmd_B = tempCurrentCmdN;
				RectifierReference.abc.phB = RectifierSineRef.phB * CurrentCmd_B;
				tempB = SecondOrderIIRFP(((RectifierReference.abc.phB - RectifierCurrent.abc.phB)*VoltNormRecVb), &PFCphb2N );
				RectifierOut.abc.phB = RectifierOut.abc.phB - tempB;
				if ( tempB > 0.0 )
				{
					PFCphb2N.X1 = 0.0;
					PFCphb2N.X2 = 0.0;
				}
				else if ( tempB < -1.0 )
				{
					tempB = -1.0;
					PFCphb2N.X1 = tempB * PFCISatFactor2.phB;
					PFCphb2N.X2 = PFCphb2N.X1;
				}

				LimitRectifierDuty( &RectifierOut.abc.phB );

				if(FlagHighBusN == true)
				{
					SetRect3LvlPwm( (0.0f), PWMPeriod, 2, 0 );
				}
				else
				{
					SetRect3LvlPwm( (-(1.0 - RectifierOut.abc.phB)), PWMPeriod, 2, 0 );
				}
			}

			if(RawAdcDataPtr->st.InputVoltage.phC > 0.0f)
			{
				CurrentCmd_C = tempCurrentCmdP;
				RectifierReference.abc.phC = RectifierSineRef.phC * CurrentCmd_C;
				tempC = SecondOrderIIRFP(((RectifierReference.abc.phC - RectifierCurrent.abc.phC)*VoltNormRecVc), &PFCphc2P );
				RectifierOut.abc.phC = RectifierOut.abc.phC - tempC;
				if ( tempC > 0.0 )
				{
					PFCphc2P.X1 = 0.0;
					PFCphc2P.X2 = 0.0;
				}
				else if ( tempC < -1.0 )
				{
					tempC = -1.0;
					PFCphc2P.X1 = tempC * PFCISatFactor2.phC;
					PFCphc2P.X2 = PFCphc2P.X1;
				}

				LimitRectifierDuty( &RectifierOut.abc.phC );

				if(FlagHighBusP == true)
				{
					//off pwm
					SetRect3LvlPwm( 0.0f, PWMPeriod, 3, 1 );	//1~0: 100%~0% duty
				}
				else
				{
					SetRect3LvlPwm( (1.0 - RectifierOut.abc.phC), PWMPeriod, 3, 1 );	//1~0: 100%~0% duty
				}

			}
			else
			{
				CurrentCmd_C = tempCurrentCmdN;
				RectifierReference.abc.phC = RectifierSineRef.phC * CurrentCmd_C;
				tempC = SecondOrderIIRFP(((RectifierReference.abc.phC - RectifierCurrent.abc.phC)*VoltNormRecVc), &PFCphc2N );
				RectifierOut.abc.phC = RectifierOut.abc.phC - tempC;
				if ( tempC > 0.0 )
				{
					PFCphc2N.X1 = 0.0;
					PFCphc2N.X2 = 0.0;
				}
				else if ( tempC < -1.0 )
				{
					tempC = -1.0;
					PFCphc2N.X1 = tempC * PFCISatFactor2.phC;
					PFCphc2N.X2 = PFCphc2N.X1;
				}
				LimitRectifierDuty( &RectifierOut.abc.phC );

				if(FlagHighBusN == true)
				{
					SetRect3LvlPwm( (0.0f), PWMPeriod, 3, 0 );
				}
				else
				{
					SetRect3LvlPwm( (-(1.0 - RectifierOut.abc.phC)), PWMPeriod, 3, 0 );
				}
			}
		}
    }
    else if( RectifierStatus.bit.FixedDutyCycle ) //for "cr 1 xx" test command
    {
    	if( RectOut.phA >= 0)  //positive
		{
			SetRect3LvlPwm( RectOut.phA, PWMPeriod, 1, 1 ); 
			SetRect3LvlPwm( RectOut.phB, PWMPeriod, 2, 1 );
			SetRect3LvlPwm( RectOut.phC, PWMPeriod, 3, 1 );
		}
		else                //nagetive
	    {
		    SetRect3LvlPwm( RectOut.phA, PWMPeriod, 1, 0 );
			SetRect3LvlPwm( RectOut.phB, PWMPeriod, 2, 0 );
			SetRect3LvlPwm( RectOut.phC, PWMPeriod, 3, 0 );
		}
    }
    else
	{
		//do nothing for "cb 1 xx" test command
	}
}

// ***********************************************************************
// *
// *    FUNCTION: RectifierControl::RecCtrlThreePhase-- ISR function 
// *
// *    DESCRIPTION: Rectifier Three phase PFC controller, call by Run_ISR
// *                   Use for Wombat33/31
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void RectifierControl::RecCtrlThreePhase( void )
{
    float tempSin;
    float tempCos;

    // Convert input voltage to ABY space, and run a high-pass filter on the alpha and beta channels only.
    // For the filter's performance, this should be run even when the rectifier isn't running.
    //2.Vabc
	RectifierVoltage.abc.phA = fabs(RawAdcDataPtr->st.InputVoltage.phA);
	RectifierVoltage.abc.phB = fabs(RawAdcDataPtr->st.InputVoltage.phB);
	RectifierVoltage.abc.phC = fabs(RawAdcDataPtr->st.InputVoltage.phC);

    if ( !RectifierStatus.bit.FixedDutyCycle )
    {
        //4.1)set sin/cos
        RectifierSineRef.phA = fabs(sin( UtilityPLL.SineRef.Angle ) * SineReferenceMagnitude);
        RectifierSineRef.phB = fabs(sin( UtilityPLL.SineRef.Angle - TwoThirdsPI ) * SineReferenceMagnitude);
        RectifierSineRef.phC = fabs(sin( UtilityPLL.SineRef.Angle + TwoThirdsPI ) * SineReferenceMagnitude);

        // Voltage feed-forward to dampen the plant's resonant pole.
        //4.3)UtilityGain=0.8
        //SineRefGain = 0.2
        //VoltageNormFactor = 2/Vdc
        RectifierOut.abc.phA = (RectifierVoltage.abc.phA * UtilityGain + RectifierSineRef.phA * SineRefGain) * VoltageNormFactor;
        RectifierOut.abc.phB = (RectifierVoltage.abc.phB * UtilityGain + RectifierSineRef.phB * SineRefGain) * VoltageNormFactor;
        RectifierOut.abc.phC = (RectifierVoltage.abc.phC * UtilityGain + RectifierSineRef.phC * SineRefGain) * VoltageNormFactor;

    }
}


// ***********************************************************************
// *
// *    FUNCTION: RectifierControl::RectiferResonantCompensateControl -- ISR function
// *
// *    DESCRIPTION: Rectifier Resonant Compensate controller
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
inline void RectifierControl::RectiferResonantCompensateControl( void )
{
//    InputControlCurrentPhA = RawAdcDataPtr->st.InputCurrent.phA;
//    InputControlCurrentPhB = RawAdcDataPtr->st.InputCurrent.phB;
//    InputControlCurrentPhC = RawAdcDataPtr->st.InputCurrent.phC;

    //4.2)Iabc_fb
	RectifierCurrent.abc.phA = fabs(RawAdcDataPtr->st.InputCurrent.phA);
	RectifierCurrent.abc.phB = fabs(RawAdcDataPtr->st.InputCurrent.phB);
	RectifierCurrent.abc.phC = fabs(RawAdcDataPtr->st.InputCurrent.phC);

	    if (RectifierStatus.bit.UseResonantComp)
	    {
	        // Run the resonant compensators on alternating switching cycles in a parallel structure
	        ResonantCommandPhA[ResonantState] = SecondOrderIIRFP(RectifierCurrent.abc.phA, &ResonantCompensatorsPhA[ResonantState]);
	        ResonantCommandPhB[ResonantState] = SecondOrderIIRFP(RectifierCurrent.abc.phB, &ResonantCompensatorsPhB[ResonantState]);
	        ResonantCommandPhC[ResonantState] = SecondOrderIIRFP(RectifierCurrent.abc.phC, &ResonantCompensatorsPhC[ResonantState]);

	        ResonantState++;
	        if(ResonantState > (nCompensators - 1))       //add 3th,5th,7th compensation
	        {
	            ResonantState = 0;
	        }

	        // Implement resonant peak control as parallel resonators in the feedback path.
	        // This adds attenuation for harmonic disturbances in both the reference and the plant.
	        for (unsigned i = 0; i < nCompensators; ++i)
	        {
	        	RectifierCurrent.abc.phA += ResonantCommandPhA[i];
	        	RectifierCurrent.abc.phB += ResonantCommandPhB[i];
	            RectifierCurrent.abc.phC += ResonantCommandPhC[i];
	        }
	        // Each compensator adds + 0.1x broadband gain.  Adjust the overall scale factor for the number of
	        // compensators in parallel with the rectifier current feedback.  The compiler folds these constant
	        // operations.
	        RectifierCurrent.abc.phA *= (1.0/(1.0f + nCompensators * 0.1f));
	        RectifierCurrent.abc.phB *= (1.0/(1.0f + nCompensators * 0.1f));
	        RectifierCurrent.abc.phC *= (1.0/(1.0f + nCompensators * 0.1f));
	    }
}

// ***********************************************************************
// *
// *    FUNCTION: RectifierControl::Run -- HWI or SWI should be SWI function 
// *
// *    DESCRIPTION: Rectifier DC link controller
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void RectifierControl::OuterVoltageControl( void )	//4.33kHz
{   
//    const float OffsetSaturationFactor = MaxRectifierOffset / ( RectifierOffsetCoefficients_ptr->B1 - RectifierOffsetCoefficients_ptr->A1 );
   // float DCLinkVoltage = (RectifierStatus.bit.UseFilteredVoltage) ? FastFilteredDCVoltage : RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative;
   // float DCLinkVoltage = FastFilteredDCVoltage;
//    const uint16_t ReadyForOffsetRef = 10;
    static uint16_t SdAccumCnt = 0;
    static float UtilityNominalSdSum = 0;
    static uint16_t PowwershareExitCnt = 0; 
	
    DCLinkVoltageHalf =  DCLinkReference * 0.5f;

    // 2x downsample for 2160Hz
    if ( PLLDownSampleCount & 0x1 )		//2.16kHz
    {
        UtilityPLL.RunPLLFast();



        FastFilteredFrequency = SecondOrderIIRFP(UtilityPLL.GetAngleStep(), &FastFilteredFrequencyTable);
        if( fabs(RawAdcDataPtr->st.InputVoltage.phA) > UtilityPeak )
        {
            UtilityPeak = fabs(RawAdcDataPtr->st.InputVoltage.phA);
        }

        if( fabs(RawAdcDataPtr->st.InputVoltage.phB) > UtilityPeak )
        {
            UtilityPeak = fabs(RawAdcDataPtr->st.InputVoltage.phB);
        }

        if( fabs(RawAdcDataPtr->st.InputVoltage.phC) > UtilityPeak )
        {
            UtilityPeak = fabs(RawAdcDataPtr->st.InputVoltage.phC);
        }
    }
    ++PLLDownSampleCount;
    
    //fix JIRA LFP156:8% harmoric input,parallel boost on and off periodly
	UtilityNominalSdSum += UtilityPLL.SourceNominalDQO.Sd;
	if( SdAccumCnt++ >= 100 )//5.1KHZ/100=51HZ
	{
		SdAccumCnt = 0;
		UtilityNominalAvgSd = UtilityNominalSdSum * 0.01f;
		UtilityNominalSdSum = 0;
	}

    // Goldilocks 20K/30K run voltage loop in 5102Hz
    // 93E 100K/120K, run voltage loop in 1700Hz
    // Hobbit:26k/6=4.33k
    if( ++ReadyForPI >= 5)//ReadyForPIRef )
    {
        ReadyForPI = 0;
        if( RectifierStatus.bit.RectifierOnNormal )
        {
			//1.1 Vdc sum loop
            // error = reference - link voltage, need to check signs
        	float IcmdP = 0.0f;
        	float IcmdN = 0.0f;
			float dcErrorP = 0.0f;
			float dcErrorN = 0.0f;
			if((THDiStartFlag == true))
			{
				Inverter.SetTHDiImprove(true);
			}
			else
			{
				Inverter.SetTHDiImprove(false);
			}

			if(ParallelCan.ParGlobalOrData.InverterStatus.bit.THDiImprove)
			{	
				dcErrorP = (DCLinkVoltageHalf - DCLinkVoltagePositive.FastFiltered)*KvfwRec;
				dcErrorN = (DCLinkVoltageHalf - (-DCLinkVoltageNegative.FastFiltered))*KvfwRec;
			}
			else
			{
//					dcErrorP = (DCLinkVoltageHalf - RailVoltagePositiveFastFilt)*KvfwRec;
//					dcErrorN = (DCLinkVoltageHalf - (-RailVoltageNegativeFastFilt))*KvfwRec;
				dcErrorP = (DCLinkVoltageHalf - RawAdcDataPtr->st.RailVoltagePositive)*KvfwRec; 
				dcErrorN = (DCLinkVoltageHalf - (-RawAdcDataPtr->st.RailVoltageNegative))*KvfwRec;
			}
						
            // run compensator
			IcmdP = FirstOrderIIRFP( dcErrorP, &LinkVoltageTableP );
			IcmdN = FirstOrderIIRFP( dcErrorN, &LinkVoltageTableN );			

            //
			IcmdP = SecondOrderIIRFP(IcmdP, &RectifierFilterLoopVFilP_3th); 				
			IcmdN = SecondOrderIIRFP(IcmdN, &RectifierFilterLoopVFilN_3th); 

			if(FlagUnblanceLoad == true)	//for unblance load
			{
				IcmdP = SecondOrderIIRFP(IcmdP, &RectifierFilterLoopVFilP_1th);					
				IcmdN = SecondOrderIIRFP(IcmdN, &RectifierFilterLoopVFilN_1th);						
			}
            // limit result to programmed current limit
            if ( IcmdP > ScaledIrefMax )
            {
                IcmdP = ScaledIrefMax;
                LinkVoltageTableP.X1 = ScaledIrefMax * LinkVoltageSaturationFactor;
            }
			else if( IcmdP < 0.0f )
			{
				IcmdP = 0.0f;
				LinkVoltageTableP.X1 = 0.0f;
			}

			if ( IcmdN > ScaledIrefMax )
			{
				IcmdN = ScaledIrefMax;
				LinkVoltageTableN.X1 = ScaledIrefMax * LinkVoltageSaturationFactor;
			}
			else if( IcmdN < 0.0f )
			{
				IcmdN = 0.0f;
				LinkVoltageTableN.X1 = 0.0f;
			}
			
//				RawAdcData.st.ControlDebug[0] = IcmdP;
//				RawAdcData.st.ControlDebug[1] = IcmdN;

			//add lowpass filter 2~3Hz attenuate 100hz ripple for  powershare
			IcmdPowershareP = FirstOrderIIRFP(IcmdP, &RectifierFilterPowershrP);
			IcmdPowershareN = FirstOrderIIRFP(IcmdN, &RectifierFilterPowershrN);
//				RawAdcData.st.ControlDebug[2] = IcmdPowershareP;
//				RawAdcData.st.ControlDebug[3] = IcmdPowershareN;
			
            // limit result to programmed current limit
            if(!RectifierStatus.bit.PowerShare )
            {
	            if ( (IcmdPowershareP >= ScaledIrefMax) || (IcmdPowershareN >= ScaledIrefMax) )
	            {
	                RectifierStatus.bit.PowerShare = true;		//need add debounce filter
					PowwershareExitCnt = 0;
	            }				
            }
			else
			{
	            if( ((IcmdPowershareP+RectifierPowerShareBuffer) < ScaledIrefMax) &&
	            	((IcmdPowershareN+RectifierPowerShareBuffer) < ScaledIrefMax) )	//for wombat JIRA-270(when set ScaledIrefMax little, the debounce 0.98 not enough
	            {
					
					if(PowwershareExitCnt++ > 2000)		//=((1/900Hz)*2000), max will trip about 1300
					{
						PowwershareExitCnt = 0;
		                RectifierStatus.bit.PowerShare = false;
					}		
	            }	
				else
				{
					PowwershareExitCnt = 0;
				}

			}

            CurrentCmdP_1 = CurrentCmdP;
            CurrentCmdN_1 = CurrentCmdN;          
            CurrentCmdP = IcmdP;
            CurrentCmdN = IcmdN;
			CurrentCmdP_Delta = (CurrentCmdP-CurrentCmdP_1)/15.0;		// 1volt loop ->15curr loop
			CurrentCmdN_Delta = (CurrentCmdN-CurrentCmdN_1)/15.0;		
			CurrCntLoop = 0;			

//				//2.This not use in Hobbit: Line N phase loss detect
//	            if ( ( fabs( InputNeutral.SlowFiltered ) > SiteWiringFaultHighLimit ))
//	            {
//	                NeutralLoss.SetState( true );
//	            }
//	
//				//2.2
//	            if ( NeutralLoss.GetState() )
//	            {
//	                // Provide some hysteresis for the alarm threshold.
//	                const float clearThreshold = 0.50f * SiteWiringFaultHighLimit;
//	
//	                // if neutral returns (!) and offset control is inverted the offset will run away
//	                // If neutral does not return, the neutral voltage will still slowly decay to zero.
//	                if ( ( fabs(InputNeutral.SlowFiltered) < clearThreshold ))
//	                {
//	                    NeutralLoss.Debounce( false );
//	                }
//	            }			
        }
		else
		{
			Inverter.SetTHDiImprove(false);
		}
        
    }    


}

// ***********************************************************************
// *
// *    FUNCTION: Rectifier::OnNormal  
// *
// *    DESCRIPTION: Turns rectifier on in normal ( PFC ) mode
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::OnNormal( bool walkin )
{
    if ( walkin )
    {
        RectifierStatus.bit.WalkIn = 1;
//	        ScaledIrefMax = 0;
    }
    else
    {
        RectifierStatus.bit.WalkIn = 0;
    }
	
	if(UPMSystem == HV_40K)
	{
		RectifierVoltageLoopCoefficients_ptr = &RectifierVoltageLoopCoefficients_40KVA_HV;
		RectifierCurrentLoopCoeffiNewRecTopo_ptr = &RectifierCurrentLoopCoeffiNewRec_40KVA_HV;
	}
	else if(UPMSystem == HV_30K)
	{
		RectifierVoltageLoopCoefficients_ptr = &RectifierVoltageLoopCoefficients_30KVA_HV;
		RectifierCurrentLoopCoeffiNewRecTopo_ptr = &RectifierCurrentLoopCoeffiNewRec_30KVA_HV;
	}
	else
	{
		RectifierVoltageLoopCoefficients_ptr = &RectifierVoltageLoopCoefficients_20KVA_HV;
		RectifierCurrentLoopCoeffiNewRecTopo_ptr = &RectifierCurrentLoopCoeffiNewRec_20KVA_HV;
	}
	LinkVoltageTableP = *RectifierVoltageLoopCoefficients_ptr;
	LinkVoltageTableN = *RectifierVoltageLoopCoefficients_ptr;
	LinkVoltageSaturationFactor = (float)1.0 / ( LinkVoltageTableP.B1 - LinkVoltageTableP.A1 );

	//Clear Rec curr loop
	PFCpha2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCpha2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
    PFCISatFactor2.phA = 1.0f / ( ( PFCpha2P.B1 + PFCpha2P.B2 ) - ( PFCpha2P.A1 + PFCpha2P.A2 ) );
    PFCISatFactor2.phB = 1.0f / ( ( PFCphb2P.B1 + PFCphb2P.B2 ) - ( PFCphb2P.A1 + PFCphb2P.A2 ) );
    PFCISatFactor2.phC = 1.0f / ( ( PFCphc2P.B1 + PFCphc2P.B2 ) - ( PFCphc2P.A1 + PFCphc2P.A2 ) );

    ResonantState = 0;
    for (uint16_t i = 0; i < nCompensators; ++i)
    {
        ResonantCommandPhA[i] = 0.0f;
        ResonantCommandPhB[i] = 0.0f;
        ResonantCommandPhC[i] = 0.0f;
    }
	
    if (RectifierStatus.bit.UseDynamicRCGains)
    {
        // Only zero out the states
        for (uint16_t i = 0; i < nCompensators; ++i)
        {
            ResonantCompensatorsPhA[i].X1 = 0.0f;
            ResonantCompensatorsPhA[i].X2 = 0.0f;
            ResonantCompensatorsPhB[i].X1 = 0.0f;
            ResonantCompensatorsPhB[i].X2 = 0.0f;
            ResonantCompensatorsPhC[i].X1 = 0.0f;
            ResonantCompensatorsPhC[i].X2 = 0.0f;
        }
    }
    else
    {
        // Fully initialize the gains and states.  This should get lowered
        // to a handful of memcpy() calls.
        ResonantCompensatorsPhA[0] = ResonantFilterGains[0]; // 3rd
//	        ResonantCompensatorsPhA[1] = ResonantFilterGains[1]; // 5th
//	        ResonantCompensatorsPhA[2] = ResonantFilterGains[2]; // 7th
	
        ResonantCompensatorsPhB[0] = ResonantFilterGains[0]; // 3rd
//	        ResonantCompensatorsPhB[1] = ResonantFilterGains[1]; // 5th
//	        ResonantCompensatorsPhB[2] = ResonantFilterGains[2]; // 7th
	
        ResonantCompensatorsPhC[0] = ResonantFilterGains[0]; // 3rd
//	        ResonantCompensatorsPhC[1] = ResonantFilterGains[1]; // 5th
//	        ResonantCompensatorsPhC[2] = ResonantFilterGains[2]; // 7th
    }

	//Clear Rec volt loop
	CurrentCmdP = 0;
	CurrentCmdN = 0;
	CurrentCmdP_1 = 0;
	CurrentCmdN_1 = 0;
	LinkVoltageTableP.X1 = 0;
	LinkVoltageTableN.X1 = 0;
	CurrCntLoop = 0;
	RectifierFilterLoopVFilP_1th.X1 = 0;
	RectifierFilterLoopVFilP_1th.X2 = 0;
	RectifierFilterLoopVFilN_1th.X1 = 0;
	RectifierFilterLoopVFilN_1th.X2 = 0;
	RectifierFilterLoopVFilP_3th.X1 = 0;
	RectifierFilterLoopVFilP_3th.X2 = 0;
	RectifierFilterLoopVFilN_3th.X1 = 0;
	RectifierFilterLoopVFilN_3th.X2 = 0;

    RectifierPWMOn();
    RectifierStatus.bit.RectifierOnNormal = 1;
    RectifierStatus.bit.RectifierOnBattery = 0;

	if(walkin)
	{
		CurrentCmdP = ScaledIrefMax;
		CurrentCmdN = ScaledIrefMax;
		LinkVoltageTableP.X1 = ScaledIrefMax * LinkVoltageSaturationFactor;
		LinkVoltageTableN.X1 = ScaledIrefMax * LinkVoltageSaturationFactor;			
	}
}

// ***********************************************************************
// *
// *    FUNCTION: Rectifier::OnNormal  
// *
// *    DESCRIPTION: Turns rectifier on in normal ( PFC ) mode
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::RecOnOnePhase(uint16_t phase)
{
	RecOnPhase = phase;
}

// ***********************************************************************
// *
// *    FUNCTION: Rectifier::OnBattery  
// *
// *    DESCRIPTION: Turns rectifier on in battery ( balancer ) mode
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::OnBattery( void )
{

    RectifierStatus.bit.RectifierOnNormal = 0;
    RectifierStatus.bit.RectifierOnBattery = 1;

	//Clear Rec volt loop
    CurrentCmdP = 0;
    CurrentCmdN = 0;
    CurrentCmdP_1 = 0;
    CurrentCmdN_1 = 0;
	CurrentCmdP_Delta = 0.0;
	CurrentCmdN_Delta = 0.0;
    LinkVoltageTableP.X1 = 0;
    LinkVoltageTableN.X1 = 0;
	CurrCntLoop = 0;
	
	//Clear Rec curr loop
	PFCpha2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCpha2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphb2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
	PFCphc2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;

	
}

// ***********************************************************************
// *
// *    FUNCTION: Rectifier::RectifierOff  
// *
// *    DESCRIPTION: Turns rectifier off
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::RectifierOff( void )
{
    MCUStateMachine.SetVFModeReady(false);
    RectifierPWMOff();
    RectifierStatus.bit.RectifierOnNormal = 0;
    RectifierStatus.bit.RectifierOnBattery = 0;
    RectifierStatus.bit.OffCommand = 1;
	RectifierStatus.bit.FixedDutyCycle = 0;
	
	//Clear Rec volt loop
    CurrentCmdP = 0;
    CurrentCmdN = 0;
    CurrentCmdP_1 = 0;
    CurrentCmdN_1 = 0;
	CurrentCmdP_Delta = 0.0;
	CurrentCmdN_Delta = 0.0;
    LinkVoltageTableP.X1 = 0;
    LinkVoltageTableN.X1 = 0;
	CurrCntLoop = 0;
	RectifierFilterLoopVFilP_1th.X1 = 0;
	RectifierFilterLoopVFilP_1th.X2 = 0;
	RectifierFilterLoopVFilN_1th.X1 = 0;
	RectifierFilterLoopVFilN_1th.X2 = 0;
	RectifierFilterLoopVFilP_3th.X1 = 0;
	RectifierFilterLoopVFilP_3th.X2 = 0;
	RectifierFilterLoopVFilN_3th.X1 = 0;
	RectifierFilterLoopVFilN_3th.X2 = 0;

    CriticalSection enter;
    // Initialize current controller to zeros, off state
//    PFCpha1 = NullCoefficients;
//    PFCphb1 = NullCoefficients;
//    PFCphc1 = NullCoefficients;
    PFCpha2P.X1 = 0.0;
    PFCpha2P.X2 = 0.0;
    PFCphb2P.X1 = 0.0;
    PFCphb2P.X2 = 0.0;
    PFCphc2P.X1 = 0.0;
    PFCphc2P.X2 = 0.0;

    PFCpha2N.X1 = 0.0;
    PFCpha2N.X2 = 0.0;
    PFCphb2N.X1 = 0.0;
    PFCphb2N.X2 = 0.0;
    PFCphc2N.X1 = 0.0;
    PFCphc2N.X2 = 0.0;	
}

// ***********************************************************************
// *
// *    FUNCTION: SetHWCurrentLimit  
// *
// *    DESCRIPTION: sets current limit
// *
// *    ARGUMENTS: currentLimit(amps)- the hardware current limit you want
// *               based on the TAMURA(LEM ripoff) sensor
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::SetHWCurrentLimit( int16_t currentLimit)    
{
    float tempDuty;
    float CurrentLimPWMPeriod = float( CPUFrequency / PWMFrequency );

	switch(UPMSystem)
	{
		case HV_20K:
		case HV_30K:
			// 0.00745108 = 0.625/50 * 200/202 * 10/(10+2.49+4.12) * (100+10)/100 /3.3 * 3
			tempDuty = (1.5 + ( (float)currentLimit * 0.00745108 ))/3.0;
			break;

	   case HV_40K:
	    	// 0.00372554 = 0.625/50 * 200/202 * 10/(10+2.49+4.12) * (100+10)/100 /3.3 * 3*0.5
	    	tempDuty = (1.5 + ( (float)currentLimit * 0.00372554 ))/3.0;
	    	break;	   	

		default:
	    	tempDuty = (1.5 + ( (float)currentLimit * 0.00372554 ))/3.0;
			break;
	}

    ECap4Regs.CAP4 = (uint16_t)( tempDuty * CurrentLimPWMPeriod );
}
// ***********************************************************************
// *
// *    FUNCTION: RefreshHWCurrLimit  
// *
// *    DESCRIPTION: sets current limit
// *
// *    EXECUTE Period:20ms
// *    ARGUMENTS: currentLimit(amps)- the hardware current limit you want
// *               based on the TAMURA(LEM ripoff) sensor
// *
// *    RETURNS: 
// *
// ***********************************************************************

void RectifierControl::RefreshHWCurrLimit(void)
{
//		int16_t CurrHwLimit;
//		float tempDuty;
//	    float CurrentLimPWMPeriod = float( CPUFrequency / PWMFrequency );
//		
//		CurrHwLimit = int16_t((float)RectHWCurrentLimit * DCLinkVoltRefNormFactor);
//		RectHWCurrentLimitRealTime = CurrHwLimit;
//	
//		// 0.00372554 = 0.625/50 * 200/202 * 10/(10+2.49+4.12) * (100+10)/100 /3.3 * 3*0.5
//		tempDuty = (1.5 + ( (float)CurrHwLimit * 0.00372554 ))/3.0;
//	    ECap4Regs.CAP4 = (uint16_t)( tempDuty * CurrentLimPWMPeriod );
}


// ***********************************************************************
// *
// *    FUNCTION: SetMaxRMSInputCurrent
// *
// *    DESCRIPTION: Set Rectifier max rms input current
// *
// *    ARGUMENTS: 
// *
// *    RETURNS:
// *
// ***********************************************************************

void RectifierControl::SetMaxRMSInputCurrent(int16_t RecRMSCurrentLimit)
{
	float tempcurrent = 0.0;
	float tempdata = 0.0;
    float tempNomV = 0.0;
    if ( 0 != OutNomVolts )
    {
        tempNomV = (float)OutNomVolts / 10.0;
    }
    else
    {
        tempNomV = 230.0;
    }

	if( ScreenMeters.PercentLoad.sum < 55 && ScreenMeters.PercentLoad.sum > 45 )
	{
		switch(UPMSystem)
		{
			case HV_20K:
				tempcurrent = (float)RecRMSCurrentLimit * 1.19;
	            break;
	        case HV_30K:
	            tempcurrent = (float)RecRMSCurrentLimit * 1.15;
	            break;
	        case HV_40K:
	        	tempcurrent = (float)RecRMSCurrentLimit * 1.3;
	            break;
	        default:
	        	tempcurrent = (float)RecRMSCurrentLimit * 1.15;
	            break;
		}
	}
	else if( ScreenMeters.PercentLoad.sum > 95)
	{
		switch(UPMSystem)
		{
			case HV_20K:
				tempcurrent = (float)RecRMSCurrentLimit * 0.88;
	            break;
	        case HV_30K:
	        	tempcurrent = (float)RecRMSCurrentLimit * 0.84;
	        	break;
	        case HV_40K:
	        	tempcurrent = (float)RecRMSCurrentLimit * 0.96;
	        	break;
	        default:
	        	tempcurrent = (float)RecRMSCurrentLimit * 0.96;
	        	break;
		}
	}
	else
	{
		tempcurrent = (float)RecRMSCurrentLimit;
	}
	MaxRMSInputCurrent = tempcurrent;

    // scale ref to nominal
    tempdata = MaxRMSInputCurrent / tempNomV;
    // store
    RectifierInputCurrentMax = ( tempdata * 1.05f ) ;
}

// ***********************************************************************
// *
// *    FUNCTION: StartFixedDuty 
// *
// *    DESCRIPTION: Loads fixed duty value to PWM CMPR and enables gates
// *
// *    ARGUMENTS: int16_t duty cycle 0 - 100
// *               boot  phAonly    true:  enable pha gates only
// *                                false: enable all gates
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::StartFixedDuty(int16_t duty)
{
    if ( !RectifierStatus.bit.RectifierOnNormal     &&
         !RectifierStatus.bit.RectifierOnBattery    &&
         ( duty <= 100 ) )
    {
        float temp = (float)(duty-50)  / (float)50; //convert to range -1...+1

        RectifierStatus.bit.FixedDutyCycle = 1;
		
        RectOut.phA = temp;
        RectOut.phB = temp;
        RectOut.phC = temp;

        RectifierPWMOn();

    }     
}

// ***********************************************************************
// *
// *    FUNCTION: SetMixedModeGain 
// *
// *    DESCRIPTION: Loads new sine ref gain
// *
// *    ARGUMENTS: uint16_t gain 0 - 100
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::SetMixedModeGain( int32_t gain )
{
    // rectifier must be off
    if ( !RectifierStatus.bit.RectifierOnNormal )
    {
        if ( ( gain < 100 ) &&
             ( gain >= 0 ) )
        {
            float tempGain = (float)gain / 100.0;
        
            SineRefGain = tempGain;
            UtilityGain = 1.0 - SineRefGain;
        }
    }    
}        

//Jacob/20130815/add begin...
// ***********************************************************************
// *
// *    FUNCTION: SetMixedModeGainForOptimizeInputPF
// *
// *    DESCRIPTION: Loads new sine ref gain
// *
// *    ARGUMENTS: uint16_t gain 0 - 100
// *
// *    RETURNS:
// *
// ***********************************************************************
void RectifierControl::SetMixedModeGainForOptimizeInputPF( uint16_t gain )
{
    // rectifier must be off
    if ( !RectifierStatus.bit.RectifierOnNormal )
    {
        if ( gain <= 100 )
        {
            float tempGain = (float)gain / 100.0;

            SineRefGain_OptimizeInputPF = tempGain;
            UtilityGain_OptimizeInputPF = 1.0 - SineRefGain_OptimizeInputPF;
        }
    }
}
// ***********************************************************************
// *
// *    FUNCTION: SetRectifierAngleOffset
// *
// *    DESCRIPTION: Loads new sine ref gain
// *
// *    ARGUMENTS: uint16_t gain 0 - 100
// *
// *    RETURNS:
// *
// ***********************************************************************
void RectifierControl::SetRectifierAngleOffset( uint16_t tempData )
{
    if ( tempData <= 40 )
    {
        RectifierAngleOffset = float( tempData ) * PI / ( 1800.0 );
    }
}

// ***********************************************************************
// *
// *    FUNCTION: SetRectCurrentloopGain
// *
// *    DESCRIPTION: set rectifier current loop gain
// *
// *    ARGUMENTS: Default value = 100
// *
// *    RETURNS: none
// *
// ***********************************************************************
void RectifierControl::SetRectCurrentloopGain( uint16_t gain )
{
	RectCurrentLoopGainReal = (float) gain / 100.0f;
}

//Jacob/20130815/...add end

// ***********************************************************************
// *
// *    FUNCTION: Rectifier20msTask 
// *
// *    DESCRIPTION: Sets DC link voltage ref based on operating conditions
// *                 slow housekeeping
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none
// *
// ***********************************************************************
void RectifierControl::Rectifier20msTask( void )
{
    const float DCLinkHeadroom = 30.0;                  // rectifier/inverter need some room to work
    float maxLinkRef = 0.0;
    static uint16_t surgeProtectionDelay = 0;
    static uint16_t SurgePwmOffHistoryDelay = 0;
    static float DCLinkRefPre = 0;
                                     
	//1.Vbus_ref cal                    
    // minimum link voltage = inverter voltage peak 
    float minLinkRef = (float)OutNomVolts / 10.0;
    //output 240V  need 750V bus
    if(OutNomVolts == 2400)
    {
    	minLinkRef *= 3.0;               //240 * 3 = 720
    	minLinkRef += DCLinkHeadroom;    //720 + 30 = 750
    }
    else//output 220 & 230 need 730V bus
    {
    	minLinkRef *= 2.0;
    	minLinkRef *= SQRT_2;
    	minLinkRef += DCLinkHeadroom;
    }

    // max link voltage based on DCOV set point
	maxLinkRef = DCLINK_780_VOLT;//DCLINK_810_VOLT;
    // range protect
    if( minLinkRef < RectifierDCLinkSet )
    {
    	minLinkRef = RectifierDCLinkSet;
    }
    if( minLinkRef > maxLinkRef )
    {
    	minLinkRef = maxLinkRef;
    }         
    // check high input line, dc link must be higher than input voltage for PFC to work
    // determine highest input phase
    float highestInputPhase = ScreenMeters.InputVoltageRMS.phA;
    highestInputPhase = ( ScreenMeters.InputVoltageRMS.phB > highestInputPhase ) ? ScreenMeters.InputVoltageRMS.phB : highestInputPhase;
    highestInputPhase = ( ScreenMeters.InputVoltageRMS.phC > highestInputPhase ) ? ScreenMeters.InputVoltageRMS.phC : highestInputPhase;
    highestInputPhase *= SQRT_2;
    
    if( MCUStateMachine.GetState() == ESS_MODE_STATE ) 
    {
        float highestInputPhase = OutputVoltageRMS.RawRMS.phA;
        highestInputPhase = ( OutputVoltageRMS.RawRMS.phB > highestInputPhase ) ? OutputVoltageRMS.RawRMS.phB : highestInputPhase;
        highestInputPhase = ( OutputVoltageRMS.RawRMS.phC > highestInputPhase ) ? OutputVoltageRMS.RawRMS.phC : highestInputPhase;
        highestInputPhase *= SQRT_2;        
    }
    
    float pfcLinkRef = ( highestInputPhase * 2.0 ) + DCLinkHeadroom;
    // check for fast transient OV
    if ( ( UtilityPLL.SourceDQOData.Sd * 2.0 ) > pfcLinkRef )
    {
        pfcLinkRef = ( UtilityPLL.SourceDQOData.Sd * 2.0 );
    }
    if( UnitConfig.bit.EnableSurgeProtection &&
        ( RECTIFIER_NORMAL_STATE == Rectifier.GetState()) )
    {
        if( ++surgeProtectionDelay > REC_100MS_BASE_20MS )
        {
            surgeProtectionDelay = REC_100MS_BASE_20MS;
            RectifierStatus.bit.SurgeProtectionOn = 1;
        }
        else
        {
            RectifierStatus.bit.SurgeProtectionOn = 0;
        }
    }
    else
    {
        surgeProtectionDelay = 0;
        RectifierStatus.bit.SurgeProtectionOn = 0;
    }

    if ( ( UtilityPeak * 2.0f ) > pfcLinkRef )
    {
        pfcLinkRef = ( UtilityPeak * 2.0f );
    }
    UtilityPeak = 0.0f;

    // check for highline boost needed    
    if ( pfcLinkRef > minLinkRef )
    {
        minLinkRef = pfcLinkRef;
    }
    
    // finally, check against max
    if ( minLinkRef > maxLinkRef )
    {
        minLinkRef = maxLinkRef;
    }

	//delete to same as 9P/wombat
//		//battery mode without line present,set bus ref as eep set,line restore,
//		//dynamic bus ref also corresponding with line peak
//		if( ( RectifierStatus.bit.RectifierOnBattery ) && ( !Rectifier.UtilityStable_OnBattery ) )
//		{
//			minLinkRef = RectifierDCLinkSet;
//		}
	//follow 9P, add byp effect vbus
	DCLinkSafety();
    minLinkRef = std::min(maxLinkRef, std::max(VdcSafety, minLinkRef ));
	
    // load
    DCLinkRefPre = DCLinkReference;
    if(minLinkRef > DCLinkRefPre)
    {
    	if(minLinkRef > DCLinkRefPre + DCLinkStepMax)
    	{
    		DCLinkReference = DCLinkRefPre + DCLinkStepMax;
    	}
    	else
    	{
    		DCLinkReference = minLinkRef;
    	}
    }
    else
    {
    	if(minLinkRef < DCLinkRefPre - DCLinkStepMax)
    	{
    		DCLinkReference = DCLinkRefPre - DCLinkStepMax;
    	}
    	else
    	{
    	    DCLinkReference = minLinkRef;
    	}
    }

	//2. state use
    DCLinkUnbalanceOVLimSetting = DCLinkReference * 0.5f + 20.0f;
    if(DCLinkUnbalanceOVLimSetting > 400.0f)
    {
        DCLinkUnbalanceOVLimSetting = 400.0f;
    }
    
    //3.iref max cal
    // set rectifier limit
    if ( !RectifierStatus.bit.WalkIn )  // walk-in handled by state controller
    {
        float tempInputMax;
        if ( RectifierStatus.bit.RectifierPrecharging )
        {
            // during pre-charge limit the max input current to a small value. The
            // pre-charge max is well below the target voltage, so prevent very large step
            // input current to go from ~280 -> 400V by limiting the max current. 'soft-start'
            
            // check for divide by 0
            if ( 0 != OutNomVolts )
            {
                tempInputMax = (float)OutNomVolts / 10.0;
                tempInputMax =  DCLinkRampCurrent / tempInputMax;
            }
            else
            {
            	// useless default value of 5A @ 230V
            	tempInputMax = 0.02174f;     // 5 / 230
            }    
        }
        else
        {
            if ( NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR ) )
            {
                tempInputMax = RectifierGenInputCurrentMax;
            }
            else
            {
                tempInputMax = RectifierInputCurrentMax;
            }
        }    
        
        // scale to current input voltage, unless 0 then do nothing
        if ( UtilityPLL.SourceNominalDQO.Sd > 0.45f )
        {
            float inputRef = UtilityNominalAvgSd;//Fix jira LFP-156
            // unit is getting rectifier current limits at low line, too much ripple +
            // high RMS is ticking the current limit, hold scaling @90, RMS current 
            // will start to go down at the point, avoiding HWCL
            if ( inputRef < 0.80f )
            {
                inputRef = 0.80f;
            }    
            tempInputMax = tempInputMax / inputRef;
        }    
    
//	        // load
        ScaledIrefMax = tempInputMax;
		ScaledIrefMax_Ori = tempInputMax;
		if(BatteryConverter.GetBatteryState() != BATTERY_TEST_STATE) 
		{
			ScaledIrefMax = tempInputMax;			
		}
		else
		{
			ScaledIrefMax = ScaledIrefBattestSet;			
		}
    }
	

	//4.phase lock update
	UtilityPLL.PLLPeriodicFunction();

    // load
    if ( UtilityPLL.GetSourceNormFactor() > 0.0f )
    {
        SineReferenceMagnitude = (float)1.0f / UtilityPLL.GetSourceNormFactor();
    }
    else
    {
        SineReferenceMagnitude = 0.0f;
    } 
    
    //resonant freq calculation           
    if (RectifierStatus.bit.UseDynamicRCGains)
    {
        CalculateResonantPeakFilterGains(FastFilteredFrequency);
    }    

	if(previous_state != MCUStateMachine.GetState() )
	{
		if( MCUStateMachine.GetState() == EASY_CAPACITY_TEST_STATE )
		{
			SineRefGain = ECTSineRefGain;
			UtilityGain = 1.0 - ECTSineRefGain;
			SineRefForwardEnabled = ECTSineRefForwardEnabled;
		}
		else
		{
			SineRefGain = NormalSineRefGain;
			UtilityGain = 1.0 - NormalSineRefGain;
			SineRefForwardEnabled = NormalSineRefForwardEnabled;
		}
	}
	previous_state = MCUStateMachine.GetState();
	
	//JIRA EXS-71 Rectifier R phase current be zero
	if( ++SurgePwmOffHistoryDelay > DELAY_5S_BASE_20MS )
	{
		NB_DebounceAndQue(UPM_NB_REC_PWM_OFF,RectifierStatus.bit.SurgeRPWMOff_History||RectifierStatus.bit.SurgeSTPWMOff_History,
				          RectifierStatus.bit.SurgeRPWMOff_History + (RectifierStatus.bit.SurgeSTPWMOff_History << 1));
		RectifierStatus.bit.SurgeRPWMOff_History = 0;
		RectifierStatus.bit.SurgeSTPWMOff_History = 0;
		SurgePwmOffHistoryDelay = 0;
	}

    // normalize voltage to the link
    float vNorm;
	float DCLinkVoltage = (DCLinkVoltagePositive.FastFiltered - (-DCLinkVoltageNegative.FastFiltered));
    if ( DCLinkVoltage > 0 )
    {
        if ( DCLinkVoltage < ( UtilityPLL.SourceDQOData.Sd * 2.0f ) )
        {
            DCLinkVoltage = ( UtilityPLL.SourceDQOData.Sd * 2.0f );
        }
        vNorm = 2.0f / DCLinkVoltage; //1.0 / DCLinkVoltage;
    }
    else
    {
        vNorm = 0.002f;      // 1 / DCLinkOV
    } 

	if(RectifierStatus.bit.RectifierPrecharging)
	{
		VoltageNormFactor = vNorm; //vNorm;
	}
	else
	{
		VoltageNormFactor = 2.0f / DCLinkReference; //vNorm;
	}


	//Rec Iac Norm
    float NormI;
	if(UPMSystem == HV_40K)
	{
		NormI = 60.0f;
	}
	else if(UPMSystem == HV_30K)
	{
		NormI = 45.0f;
	}
	else
	{
		NormI = 30.0f;
	}
	
	if(UtilityCurrentRMS.RawRMS.phA > 5.0f)		//Vac_pk>50V
	{
		VoltNormRecVa_Part1 = NormI/UtilityCurrentRMS.RawRMS.phA;

		if(VoltNormRecVa_Part1 > 1.0f)		//276V/230V
		{
			VoltNormRecVa_Part1 = 1.0f;
		}
		else if(VoltNormRecVa_Part1 < 0.8f)	//187V/230V
		{
			VoltNormRecVa_Part1 = 0.8f;
		}
	}
	else
	{
		VoltNormRecVa_Part1 = 1.0;
	}
	
	if(UtilityCurrentRMS.RawRMS.phB > 5.0f)		//Vac_pk>50V
	{
		VoltNormRecVb_Part1 = NormI/UtilityCurrentRMS.RawRMS.phB;
		if(VoltNormRecVb_Part1 > 1.0f)		//276V/230V
		{
			VoltNormRecVb_Part1 = 1.0f;
		}
		else if(VoltNormRecVb_Part1 < 0.8f)	//187V/230V
		{
			VoltNormRecVb_Part1 = 0.8f;
		}
	}
	else
	{
		VoltNormRecVb_Part1 = 1.0;
	}
	
	if(UtilityCurrentRMS.RawRMS.phC > 5.0f)		//Vac_pk>50V
	{
		VoltNormRecVc_Part1 = NormI/UtilityCurrentRMS.RawRMS.phC;
		if(VoltNormRecVc_Part1 > 1.0f)		//276V/230V
		{
			VoltNormRecVc_Part1 = 1.0f;
		}
		else if(VoltNormRecVc_Part1 < 0.8f)	//187V/230V
		{
			VoltNormRecVc_Part1 = 0.8f;
		}
	}
	else
	{
		VoltNormRecVc_Part1 = 1.0;
	}		
	
	//Rec THDi
	if(ParallelCan.ParGlobalOrData.InverterStatus.bit.THDiImprove)
	{
		KvfwRec = 1.0f; 
	}
	else
	{			
		KvfwRec = 3.0f; 
	}	

	static bool bFlagRecCurrLow = false;
	static uint16_t wFilCntRecCurrLow = 0;
	if(bFlagRecCurrLow == false)
	{
		if(DynamicLoadPercentRec < 60.0f)
		{
			if(wFilCntRecCurrLow++ >= 150)	//3s filter, 150=3s/20ms 
			{
				wFilCntRecCurrLow = 0;
				bFlagRecCurrLow = true; 
			}		
		}
		else
		{
			wFilCntRecCurrLow = 0;
		}
	}
	else
	{
		if(DynamicLoadPercentRec > 65.0f)
		{
			if(wFilCntRecCurrLow++ >= 150)	//3s filter, 150=3s/20ms 
			{
				wFilCntRecCurrLow = 0;
				bFlagRecCurrLow = false; 
			}		
		}
		else
		{
			wFilCntRecCurrLow = 0;
		}		
	}
	
//		if(DynamicLoadPercentRec < 60.0f)
//		{
//	//			bFlagRecCurrLow = true;
//			if(wFilCntRecCurrLow++ >= 200)	//4s filter, 200=4s/20ms 
//			{
//				wFilCntRecCurrLow = 200;
//				bFlagRecCurrLow = true; 
//			}
//		}
//		else if(DynamicLoadPercentRec > 65.0f)
//		{		
//			wFilCntRecCurrLow = 0;
//			bFlagRecCurrLow = false;
//		}

//		if(bFlagRecCurrLow)
//		{
//			VoltNormRecVa_Part1 = VoltNormRecVa_Part1*1.20f;
//			VoltNormRecVb_Part1 = VoltNormRecVb_Part1*1.20f;
//			VoltNormRecVc_Part1 = VoltNormRecVc_Part1*1.20f;
//		}
		
	if( MCUStateMachine.GetState() == EASY_CAPACITY_TEST_STATE )
	{
		VoltNormRecVa_Part1 = VoltNormRecVa_Part1 / 1.20f;	//1/1.20f
		VoltNormRecVb_Part1 = VoltNormRecVb_Part1 / 1.20f;	
		VoltNormRecVc_Part1 = VoltNormRecVc_Part1 / 1.20f;	
	}

	VoltNormRecVa = (VoltNormRecVa_Part1) * RectCurrentLoopGainReal;
	VoltNormRecVb = (VoltNormRecVb_Part1) * RectCurrentLoopGainReal;
	VoltNormRecVc = (VoltNormRecVc_Part1) * RectCurrentLoopGainReal;
	
}
// ***********************************************************************
// *
// *    FUNCTION:    DCLinkSafety
// *
// *    DESCRIPTION: Sets DC link Safety voltage based on max
// *                 input voltage or max bypass voltage to prevent
// *                 diode conduction (bypass voltage
// *                 and phase error received over can bus)
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none
// *
// ***********************************************************************
void RectifierControl::DCLinkSafety( void )
{
    VdcSafety = std::max( UtilityVoltageRMS.FilteredRMS.phA, std::max( UtilityVoltageRMS.FilteredRMS.phB, UtilityVoltageRMS.FilteredRMS.phC ));
    VdcSafety = std::max( VdcSafety, std::max( BypassVoltageRMS.FilteredRMS.phA, std::max( BypassVoltageRMS.FilteredRMS.phB, BypassVoltageRMS.FilteredRMS.phC )));

    // Line peak, full link voltage
	VdcSafety *= SQRT_2 * 2.0f;
}

// ***********************************************************************
// *
// *    FUNCTION:  CalculateResonantPeakFilterGains
// *
// *    DESCRIPTION:  Periodically computes the coefficients for the resonant-peak control
// *                  IIR filters
// *
// *    ARGUMENTS: The line frequency in units of radians/angle-step
// *
// *    RETURNS:
// *
// ***********************************************************************
void RectifierControl::CalculateResonantPeakFilterGains(float frequency)
{
    // +40 dB of gain at the resonant peaks
    const float inversePeakGain = 0.01;
    // -20 dB of gain everywhere else.  Net +20 dB gain at the peaks.
    const float dcGain = 0.1;
    
    enum phase_t {
        PHASE_Enable,
        PHASE_Disable
    };
    
    struct {
        float frequency;
        float phase;
        phase_t phase_leg;
        int clockDivider;
    } zplist[] = {
        { 3, -40*PI/180, PHASE_Enable, nCompensators },
        { 5, -40*PI/180.0, PHASE_Enable, nCompensators },
        { 7, -40*PI/180.0, PHASE_Enable, nCompensators },
        { 9, -40*PI/180.0, PHASE_Disable, nCompensators },
        { 0, 0, PHASE_Disable } // Sentinnel, do not delete
    };
    
    // Index to the next resonant compensator gain to be updated
    uint16_t Index = 0;
    
    for (uint16_t i = 0; zplist[i].frequency != 0; ++i)
    {
        // Compute pole and zero locations in S-plane as a multiple of the line frequency, including the
        // relative peak gain at the resonant frequency
        complex<float> pole, zero;
        phaseAdjustableResonantCompensator(&pole, &zero, zplist[i].frequency, zplist[i].phase, inversePeakGain);
        
        // Convert to the Z-plane, accounting for line frequency, the subdivided clock in the execution
        // structure, and the desired DC gain.
        stSecondOrderIIRFP coefficients;
        matchedZTransform(&coefficients, frequency*zplist[i].clockDivider, pole, zero, dcGain);
        
        // Copy coefficients to the required filter structure.
        switch (zplist[i].phase_leg)
        {
            case PHASE_Enable:
                if (Index < nCompensators) // Safety, should always be true
                {
                    copyIIRGainsOnly(ResonantCompensatorsPhA + Index, coefficients);
                    copyIIRGainsOnly(ResonantCompensatorsPhB + Index, coefficients);
                    copyIIRGainsOnly(ResonantCompensatorsPhC + Index, coefficients);
                    Index += 1;
                }

                break;
                
            case PHASE_Disable:
            default:
                break;
        }
    }
}

// ***********************************************************************
// *
// *    FUNCTION:  EEFunc_Rectifier
// *
// *    DESCRIPTION:  ee function for rectifier params
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::EEFunc_Rectifier( const EE_ID* ee, const uint16_t* data )
{
    float tempdata;
    float tempNomV;
    if ( 0 != OutNomVolts )
    {
        tempNomV = (float)OutNomVolts / 10.0;
    }
    else
    {
        tempNomV = 230.0;
    }        
    
    switch ( ee->paramNum )
    {
        case PARAM_Rectifer_DCLink_Set:
            RectifierDCLinkSet = (float)(*data);
            break;
            
        case PARAM_RectifierCurrentMax:
        	Rectifier.SetMaxRMSInputCurrent( *data );
        	MaxRMSInputCurrentEEValue = (float)(*data);
            break;
            
        case PARAM_RectifierGenCurrentMax:
            MaxGeneratorRMSInputCurrent = (float)(*data);
            // scale ref to nominal
            tempdata = MaxGeneratorRMSInputCurrent / tempNomV;
            // store
            RectifierGenInputCurrentMax = ( tempdata * 1.05f ) ;
            break; 
            
        case PARAM_RectifierWalkinRate:
            tempdata = (float)(*data);
            // scale ref to nominal
            tempdata = tempdata / tempNomV;
            // divide by the state control sample rate
            tempdata = tempdata / RECT_STATE_MACHINE_RUN_FREQUENCY;
            // load
            RectifierWalkInRate = tempdata;
            break;
            
        case PARAM_Rectifier_Current_Limit_Set:
            Rectifier.SetHWCurrentLimit( *data );
            break;     

        case PARAM_ECTRectifier_Sine_Ref_Gain:
				// rectifier must be off
			if ( !RectifierStatus.bit.RectifierOnNormal )
			{
				ECTSineRefGain = (float)*data / 100.0;
			}
			break;

        case PARAM_Rectifier_Sine_Ref_Gain:      // Pan/20130306,add for configure SineRefGain
                // rectifier must be off
            if ( !RectifierStatus.bit.RectifierOnNormal )
            {
            	NormalSineRefGain = (float)*data / 100.0;
            }    
            break;
        //Jacob/20130815/add begin...
		case PARAM_UtilityPLLAngleOffset:
            Rectifier.SetRectifierAngleOffset( *data );
            break;

        case PARAM_SineRefGain:
        	Rectifier.SetMixedModeGainForOptimizeInputPF( *data );
            break;
        //Jacob/20130815/...add end

        case PARAM_RectifierWalkinDelay:
            //Max delay time 180 seconds
            if((*data) > 180)
            {
                RectifierWalkInDelay = 180;
            }
            else
            {
                RectifierWalkInDelay = *data;
            }
            break;
            
		case PARAM_RectifierVoltageLoopGain:
			RectVoltageLoopGain = (float)(*data) / 100.0f;
			break;
            
		case PARAM_RectifierCurrentLoopGain:
            RectCurrentLoopGain = (float)(*data);
            RectCurrentLoopGainReal = RectCurrentLoopGain / 100.0f;
			break;  
			
		case PARAM_DCLinkRampCurrent:
            DCLinkRampCurrent = (float)(*data);
            break;

		case PARAM_RectiferCurrentMargin:
			RectiferCurrentMargin = (float)(*data);
			break;

		case PARAM_ECTSineRefForwardEnabled:
			if ( !RectifierStatus.bit.RectifierOnNormal )
			{
				ECTSineRefForwardEnabled = *data;
			}
			break;

		case PARAM_SineRefForwardEnabled:
			if ( !RectifierStatus.bit.RectifierOnNormal )
			{
				NormalSineRefForwardEnabled = *data;
			}
			break;

        case PARAM_UPSPowerShareBuffer:
			tempdata = float(*data) / 10.0f;
			// scale ref to nominal
			tempdata = tempdata / tempNomV;
			RectifierPowerShareBuffer = tempdata;
			break;
		
		case PARAM_BatChargeCurrLoopGain:
			BatChargeCurrLoopGain = (float)(*data) / 100.0f;
			break;	

		case PARAM_RecThdiImprove:
			if((*data) == 1)
			{
				RectifierStatus.bit.UseResonantComp = true;
			}
			else
			{
				RectifierStatus.bit.UseResonantComp = false;
			}
			break;

        default:
            break;          
    }
}

// ***********************************************************************
// *
// *    FUNCTION:  UpdateRectifierCurrentMax
// *
// *    DESCRIPTION:  ee function for rectifier params
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void RectifierControl::UpdateMaxRectifierCurrent( uint16_t voltage )
{
	float tempNomV;

	// convert to float
	tempNomV = float( voltage ) / 10.0f;

	// update RectifierInputCurrentMax base on tempNomV
    RectifierInputCurrentMax = (MaxRMSInputCurrent / tempNomV) * 1.05f;

    // update RectifierGenInputCurrentMax base on tempNomV
    RectifierGenInputCurrentMax = (MaxGeneratorRMSInputCurrent / tempNomV) * 1.05f;
}
// ***********************************************************************
// *
// *    FUNCTION:  GetMaxRMSCurrent
// *
// *    DESCRIPTION:  returns max rms input current in amps. Depends
// *                  on "On Generator" status
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
float RectifierControl::GetMaxRMSCurrent( void )
{
    float max;

    if ( NB_GetNodebit( UPM_NB_UPS_ON_GENERATOR ) )
    {
        max = MaxGeneratorRMSInputCurrent;
    }
    else
    {
        max = MaxRMSInputCurrent;
    }
	    
    return max;
}
// ***********************************************************************
// *
// *    FUNCTION:  GetRectifierCurrentMargin
// *
// *    DESCRIPTION:  returns max rms input current in amps. 
// *                  for charger current magin 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************

float RectifierControl::GetRectifierCurrentMargin( void )
{
	 return ( RectiferCurrentMargin );
}

// ***********************************************************************
// *
// *    FUNCTION: SetRectifierFilterLoopVFil 
// *
// *    DESCRIPTION: init the filter freq in 50hz/60hz
// *
// *    ARGUMENTS: freq:output freq 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void RectifierControl::SetRectifierFilterLoopVFil( uint16_t freq)
{
    // rectifier must be off
    if ( !RectifierStatus.bit.RectifierOnNormal )
    {
		if(freq == 50)
		{
			RectifierFilterLoopVFilP_1th = RectifierLoopVoFilterCoef_1th50Hz;
			RectifierFilterLoopVFilN_1th = RectifierLoopVoFilterCoef_1th50Hz;
			RectifierFilterLoopVFilP_3th = RectifierLoopVoFilterCoef_3th50Hz;
			RectifierFilterLoopVFilN_3th = RectifierLoopVoFilterCoef_3th50Hz;
		}
		else
		{
			RectifierFilterLoopVFilP_1th = RectifierLoopVoFilterCoef_1th60Hz;
			RectifierFilterLoopVFilN_1th = RectifierLoopVoFilterCoef_1th60Hz;
			RectifierFilterLoopVFilP_3th = RectifierLoopVoFilterCoef_3th60Hz;
			RectifierFilterLoopVFilN_3th = RectifierLoopVoFilterCoef_3th60Hz;
		}		
    }    
}     

// ***********************************************************************
// *
// *    FUNCTION: SetRectifierFilterLoopVFil 
// *
// *    DESCRIPTION: init the filter freq in 50hz/60hz
// *
// *    ARGUMENTS: freq:output freq 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#define cLoad30_Idq_20kVA		0.067558f	 		
#define cLoad40_Idq_20kVA		0.090078f	//0.090078f=16.4/182; 40%*20e3/3/230*1.414=16.4A; 		

#define cLoad30_Idq_40kVA		0.135116f	 		
#define cLoad40_Idq_40kVA		0.180156f	//0.090078f=32.8/182; 40%*40e3/3/230*1.414=32.8A; 		
void RectifierControl::OptimizedPFmodeLoop(void)
{
	static bool bFlagLightLoad = false;
	static bool bFlagLightLoad_1 = false;
	static bool bFlagLoopChange = false;
	float tempX1 = 0.0f;
	float tempX2 = 0.0f;

	if(DynamicLoadPercent < 35.0f)
	{
		bFlagLightLoad = true;
	}
	else if(DynamicLoadPercent > 45.0f)
	{			
		bFlagLightLoad = false;
	}

	if(bFlagLightLoad != bFlagLightLoad_1)
	{
		bFlagLoopChange = true;
	}
	bFlagLightLoad_1 = bFlagLightLoad;

	
	if(RectifierStatus.bit.OptimizedPFmode)
	{
		if(bFlagLoopChange)
		{
			bFlagLoopChange = false;
			
			if(bFlagLightLoad == false)	//1.load < 30
			{
//					tempX1 = PFCpha2P.X1;
//					tempX2 = PFCpha2P.X2;
//					PFCpha2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCpha2P.X1 = tempX1;
//					PFCpha2P.X2 = tempX2;
//	
//					tempX1 = PFCphb2P.X1;
//					tempX2 = PFCphb2P.X2;
//					PFCphb2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphb2P.X1 = tempX1;
//					PFCphb2P.X2 = tempX2;
//	
//					tempX1 = PFCphc2P.X1;
//					tempX2 = PFCphc2P.X2;
//					PFCphc2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphc2P.X1 = tempX1;
//					PFCphc2P.X2 = tempX2;
//					
//					tempX1 = PFCpha2N.X1;
//					tempX2 = PFCpha2N.X2;
//					PFCpha2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCpha2N.X1 = tempX1;
//					PFCpha2N.X2 = tempX2;
//					
//					tempX1 = PFCphb2N.X1;
//					tempX2 = PFCphb2N.X2;
//					PFCphb2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphb2N.X1 = tempX1;
//					PFCphb2N.X2 = tempX2;
//					
//					tempX1 = PFCphc2N.X1;
//					tempX2 = PFCphc2N.X2;
//					PFCphc2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphc2N.X1 = tempX1;
//					PFCphc2N.X2 = tempX2;
			}
			else
			{
//					tempX1 = PFCpha2P.X1;
//					tempX2 = PFCpha2P.X2;
//					PFCpha2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCpha2P.X1 = tempX1;
//					PFCpha2P.X2 = tempX2;
//	
//					tempX1 = PFCphb2P.X1;
//					tempX2 = PFCphb2P.X2;
//					PFCphb2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphb2P.X1 = tempX1;
//					PFCphb2P.X2 = tempX2;
//	
//					tempX1 = PFCphc2P.X1;
//					tempX2 = PFCphc2P.X2;
//					PFCphc2P = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphc2P.X1 = tempX1;
//					PFCphc2P.X2 = tempX2;
//					
//					tempX1 = PFCpha2N.X1;
//					tempX2 = PFCpha2N.X2;
//					PFCpha2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCpha2N.X1 = tempX1;
//					PFCpha2N.X2 = tempX2;
//					
//					tempX1 = PFCphb2N.X1;
//					tempX2 = PFCphb2N.X2;
//					PFCphb2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphb2N.X1 = tempX1;
//					PFCphb2N.X2 = tempX2;
//					
//					tempX1 = PFCphc2N.X1;
//					tempX2 = PFCphc2N.X2;
//					PFCphc2N = *RectifierCurrentLoopCoeffiNewRecTopo_ptr;
//					PFCphc2N.X1 = tempX1;
//					PFCphc2N.X2 = tempX2;				
			}
		}
			
	}
	
}
// *****************************************************************************
// *
// * Function: bool ee_calc_rectifier_limits(void);
// *
// * Purpose: Forward call to the rectifier state machine member function
// *
// * Parms Passed   :   none
// * Returns        :   TRUE/FALSE
// *
// * Description: Returns TRUE if the input is usable.
// *
// *****************************************************************************
void ee_calc_rectifier_limits( const EE_ID* ee, const uint16_t* data )
{
    Rectifier.EEFunc_Rectifier( ee, data );
}

// ***********************************************************************
// *
// *    FUNCTION:   SetRect3LvlPwm
// *
// *    DESCRIPTION: Converts pwm pulse ratio value to 2 signals for 3-level switch.
// *    maps gpio pin and pwm to phase
// *    pw is in range 0..1 where 0.5= 0v out, 1=dc+ , 0 = dc- . this is converted to 3-level pwm signals a,b
// *    in 2-level switch pw is directly the pwm pulse ratio.
// *    a= magnitude, b= polarity. 1=+, 0=-
// ***************************************************************************
// *    ARGUMENTS:  phase=number indicates which leg
// *
// *    RETURNS:
// *
// ***********************************************************************
inline void SetRect3LvlPwm( float pw, float period, const uint16_t phase, uint16_t Polarity  )
{
    uint16_t t;

    if ( Polarity == 1 )
    {
        t = uint16_t( pw * period );

        switch ( phase )
        {
            case 1: PIN_RECT_R_POL_to1(); break;
            case 2: PIN_RECT_S_POL_to1(); break;
            case 3: PIN_RECT_T_POL_to1(); break;
            default: break;
        }
    }
    else
    {
        t = uint16_t( -pw * period );

        switch ( phase )
        {
            case 1: PIN_RECT_R_POL_to0(); break;
            case 2: PIN_RECT_S_POL_to0(); break;
            case 3: PIN_RECT_T_POL_to0(); break;
            default: break;
        }
    }
    switch( phase )
    {
        case 1: EPwm1Regs.CMPB = t; break;
        case 2: EPwm3Regs.CMPA.half.CMPA = t; break;
        case 3: EPwm3Regs.CMPB = t; break;
        default: break;
    }
}

// ******************************************************************************************************
// *            End of Rectifier.cpp
// ******************************************************************************************************


