// ******************************************************************************************************
// *            Inverter.cpp
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
// *    FILE NAME: Inverter.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/8/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"    
#include "InverterControl.h"
#include "BypassState.h"
#include "RectifierStateControl.h"
#include "F28335Port.h"  
#include "adc.h"
#include "InitPWM.h"
#include "Meters.h"
#include "MCUState.h"
#include "NB_Funcs.h"
#include "C28x_FPU_FastRTS.h"
#include "DQPhaseLockLoop.h"
#include "BypassInterface.h"
#include "DQPhaseLockLoop.h"
#include "InvSync.h"
#include "ACMeter.h"
#include "ACPowerMeter.h"
#include "ParallelCan.h"
#include "ExtSignalPLL.h"
#include "IOexpansion.h"
#include "Spi_Task.h"
#include "BodeScan.h"
#include "Coefficients.h"
#include "AutoCal.h"
#include "FCTState.h"
#include "complex.h"
#include "ControlFuncs.h"
#include <cmath>
#include <algorithm>
#include "Version.h"

using namespace std;

extern "C"
{
    void InitInv(void);    
}

// voltage are in volts * 10
const uint16_t ValidInverterVoltages[] =
{
    2200,
    2300,
    2400 
};
const uint16_t NumberOfValidVoltages = sizeof( ValidInverterVoltages ) / sizeof( ValidInverterVoltages[0] ); 
const float MaxInverterDCOffset     = 5.0;
//	const float PLLMaxPhaseError      = ( ( 10.0f * PI ) / 180.0 );        // 10 degrees
const float PLLMaxPhaseError      = ( ( 5.0f * PI ) / 180.0 );        // 5 degrees  

//InnerLoopCount_BC_Max=(1.125k*2)/(45hz*3) +1=17;    (1.125k/45hz=25)
//RunPLLFast run freq:1.125k=PWMFrequency/4/4,
const uint16_t InnerLoopCount_BC_Max = uint16_t ( ( (PWMFrequency/16) * 2 ) / ( INVETER_MIN_FREQ * 3 ) + 1 );
const float InverterTs_HV  = ( 2.0f / float( PWMFrequency ) ); //13K
float InitDropPower = 0.0f;
//HV 20K inv control structure coefficients                              B0,             B1,             B2,         A1,             A2,         X1,     X2
//const stSecondOrderIIRFP InverterVoltageLoopCoefficients_20KVA_HV    = { 0.058083 ,      -0.133526,      0.97762,    -1.118683,      0.1787292,    0,      0   };
//const stSecondOrderIIRFP InverterVoltageLoopCoefficients2_20KVA_HV   = { 0.415364,       -1.808741,      0.967932,   -1.384767,      0.429091,     0,      0   };
//	const stSecondOrderIIRFP InverterVoltageLoopCoefficients_20KVA_HV   = { 1.14066,  -1.7766,   0.9879,  -1.00378,   0.134786,    0,      0   };//350Hz
//4.20. B0_new = B0*Vnorm(1/325)
const stSecondOrderIIRFP InverterVoltageLoopCoefficients_20KVA_HV   = { 0.0035073488715331f,  -1.7766,   0.9879,  -1.00378,   0.134786,    0,      0   };//350Hz
//	const stSecondOrderIIRFP InverterVoltageLoopCoefficients2_20KVA_HV    = { 0.059761,  -0.2652,   0.9667,  -1.256385,   0.316424,    0,      0   };
//tune01.for paral
//	const stSecondOrderIIRFP InverterVoltageLoopCoefficients2_20KVA_HV    = { 0.0300f,  -0.2652,   0.9667,  -1.256385,   0.316424,    0,      0   };
const stSecondOrderIIRFP InverterVoltageLoopCoefficients2_20KVA_HV    = { 0.0500f,  -0.2652,   0.9667,  -1.256385,   0.316424,    0,      0   };
//	const stSecondOrderIIRFP InverterCurrentLoopCoefficients_20KVA_HV    = { 1.328022E-4,    -2.0103,        1.0211,     -1.2420,        0.3151,     0,      0   };
//	const stSecondOrderIIRFP InverterCurrentLoopCoefficients2_20KVA_HV   = { 8.818e-05,      -1.6761,        0.8933,     -1.2310,        0.3046,     0,      0   };
//test01
const stSecondOrderIIRFP InverterCurrentLoopCoefficients_20KVA_HV    = { 6.642E-4 ,        -1.4611,    0.99506,     -1.31,      0.3858,     0,      0   };
const stSecondOrderIIRFP InverterCurrentLoopCoefficients2_20KVA_HV   = { 0.2735,        -0.93053,  -0.05839,  -1.052,     0.08127,  0,      0   };
const stFirstOrderIIRFP InverterRMSLoopCoefficients_20KVA_HV        = { 0.25,           0,              -1.0,           0   };
const stSecondOrderIIRFP InverterDCCompensationCoefficients_20KVA_HV = { 0.01,           -0.5,           0,          -1.0,           0,          0,      0   };

const stFirstOrderIIRFP CapCurrentEstimatorCoefficients_20KVA_HV    = { 0.0867,         -1,             -0.8395,        0   };

//	const stFirstOrderIIRFP ECTCurrentLoopCoefficients_20KVA_HV         = { 0.007742,       -0.75072,       -1.0,           0   };
//	const stFirstOrderIIRFP ECTCurrentLoopCoefficients_20KVA_HV         = { 0.008113*0.55f,       -0.67063,       -1.0,           0   };
const stFirstOrderIIRFP ECTCurrentLoopCoefficients_20KVA_HV         = { 0.007742*0.52f,       -0.75072,       -1.0,           0   };
const stFirstOrderIIRFP ECTPowerLoopCoefficients_20KVA_HV           = { 0.01,           0,              -1.0,           0   };

//for jira372, filter freq change to 10Hz (original 5Hz), 
//	to improve the dynamic(line to powershare,loadshare error issue)
//1.
//	const stFirstOrderIIRFP LoadShareDroopCoefficients_20KVA_HV         = { 0.001836,       -0.9336,        -0.5634,        0   };
//	const stFirstOrderIIRFP ReactiveLoadShareDroopCoefficients_20KVA_HV = { 0.00035,        0,              0,              0   };
//	const stFirstOrderIIRFP LoadShareDCFeedForwardCoefficients_20KVA_HV = {0.02717,         1,              -0.9457,        0   };
//	const stFirstOrderIIRFP ReactivePowerDroopCoefficients_20KVA_HV     = {0.0014498381,    -0.999944151,   -0.999721,      0   };
//	const stSecondOrderIIRFP HighDCVPhaseDroopCoefficients_20KVA_HV      = { 0.01,           -0.9650655,     0,          -1.0,           0,          0,      0   };
//	const stSecondOrderIIRFP ReactiveCurrentFilterCoefficients_20KVA_HV  = { 0.015822339,    -1.94609748775, 1.0,        -1.96919919061, 0.97005201, 0,      0   };

//HV 40K inv control structure coefficients                          B0,             B1,             B2,         A1,             A2,         X1,     X2
//8.26. new 80k
const stSecondOrderIIRFP InverterVoltageLoopCoefficients_40KVA_HV    = { 0.000082580182088,  -0.291326314537030,   0.980372108413917,  -0.903697802282735,   0.198615473923303,    0,      0   };//350Hz
const stSecondOrderIIRFP InverterVoltageLoopCoefficients2_40KVA_HV   = { 0.915712439008603,  -1.760210024290771,   0.940917326804260,  -1.747377996286174,   0.762147338316880,    0,      0   };

const stSecondOrderIIRFP InverterCurrentLoopCoefficients_40KVA_HV    = { 2.392676857594595e-5*1.9f,   0.168070298559128,   0.692862940143796,  -0.676223677073078,   0.112780966336496,    0,      0   };
const stSecondOrderIIRFP InverterCurrentLoopCoefficients2_40KVA_HV   = { 0.877230950866199,      -1.863347456251460,  0.939008479718264,  -1.694630216614067,   0.708302075915558,    0,      0   };

const stFirstOrderIIRFP InverterRMSLoopCoefficients_40KVA_HV        = { 0.25,           0,              -1.0,           0   };
const stSecondOrderIIRFP InverterDCCompensationCoefficients_40KVA_HV = { 0.01,           -0.5,           0,          -1.0,           0,          0,      0   };
const stFirstOrderIIRFP CapCurrentEstimatorCoefficients_40KVA_HV    = { 0.0867,         -1,             -0.8395,        0   };
const stFirstOrderIIRFP ECTCurrentLoopCoefficients_40KVA_HV         = { 0.007742*0.26f,       -0.75072,       -1.0,           0   };
const stFirstOrderIIRFP ECTPowerLoopCoefficients_40KVA_HV           = { 0.01,           0,              -1.0,           0   };

//3.1. Id_fil droop
//  0.0018(0.08hz/100%)~0.0029(0.12hz/100%);  
const stFirstOrderIIRFP LoadShareDroopCoefficients_40KVA_HV         = { 0.0012,       -0.9336,        -0.5634,        0   };
// 0.0014(0.07~9hz/100%)
const stFirstOrderIIRFP LoadShareDroopCoefficients_20KVA_HV         = { 0.0014,       -0.9336,        -0.5634,        0   };
//3.2. Iq_fil droop pos
const stFirstOrderIIRFP ReactiveLoadShareDroopCoefficients_40KVA_HV = { -0.0005f,        0,              0,              0   };
const stFirstOrderIIRFP ReactiveLoadShareDroopCoefficients_20KVA_HV = { -0.0005f,        0,              0,              0   };
//	//test2.2	5Hz-Kdc=1.2364(B0=0.0170)  
//tune4.6	10Hz-Kdc=1.2364(B0=0.0336)  
//tune8.1	B0=0.0150 (0.0336: oscilate in bat )  
const stFirstOrderIIRFP LoadShareDCFeedForwardCoefficients_40KVA_HV = {0.05479f,        	1.0f,         	-0.89042f,      0   };
//	const stFirstOrderIIRFP LoadShareDCFeedForwardCoefficients_20KVA_HV = {0.02715,         1.0f,           -0.9457,        0   };
const stFirstOrderIIRFP LoadShareDCFeedForwardCoefficients_20KVA_HV = {0.05479f,        	1.0f,         	-0.89042f,      0   };
//3.3. Iq_nofil droop
//	const stFirstOrderIIRFP ReactivePowerDroopCoefficients_40KVA_HV     = {0.0014498381,    -0.999944151,   -0.999721,      0   };
//tune1
const stFirstOrderIIRFP ReactivePowerDroopCoefficients_40KVA_HV     = {0.0012,    -0.999944151,   -0.999721,      0   };
const stFirstOrderIIRFP ReactivePowerDroopCoefficients_20KVA_HV     = {0.0014498381,    -0.999944151,   -0.999721,      0   };
//3.4. Vdc_high
const stSecondOrderIIRFP HighDCVPhaseDroopCoefficients_40KVA_HV      = { 0.01,           -0.9650655,     0,          -1.0,           0,          0,      0   };
const stSecondOrderIIRFP HighDCVPhaseDroopCoefficients_20KVA_HV      = { 0.01,           -0.9650655,     0,          -1.0,           0,          0,      0   };
//3.5. filter_Iq
const stSecondOrderIIRFP ReactiveCurrentFilterCoefficients_40KVA_HV  = { 0.015822339,    -1.94609748775, 1.0,        -1.96919919061, 0.97005201, 0,      0   };
const stSecondOrderIIRFP ReactiveCurrentFilterCoefficients_20KVA_HV  = { 0.015822339,    -1.94609748775, 1.0,        -1.96919919061, 0.97005201, 0,      0   };

const stFirstOrderIIRFP InvCurAutoZeroFilterGains = {0.9977, -1.0, -0.9951, 0};

const float HighDCVPhaseDroopGainArray[] =
{
	0.01f,	//hobbit20k
	0.0f   	//40k/30k, 9P:0.01f   
};

// voltage magnitude droop coefficient upon active power for Koala 20K_HV and 15K_HV
const float VoltageDroopCoefficient = 5.0e-6f;
  // these limits are slightly above target
const stSlewRateLimits SlewRateLimitTable[] =			//18khz
{
	//Kfw			Q_limit			Kfw_freq 	Freq_limit
    { 0.006272052f, 0.000383972f,   0.1f,   0.022f  },              // parallel
    { 0.006272052f, 0.000383972f,   0.1f,   0.022f  },              // 1Hz/s limit, 1.1 Hz/s
    { 0.0025f,      0.001151917f,   0.5f,   0.066f  },              // 3Hz/s limit, 3.3 Hz/s
    { 0.0025f,      0.001919862f,   0.5f,   0.11f   },              // 5Hz/s limit, 5.5 Hz/s
    { 0.0025f,      0.002687807f,   0.5f,   0.142f  }               // 7Hz/s limit, 7.7 Hz/s
};
  //1.10Hz: 	0.0028	  1.0000   -0.9944
  const stFirstOrderIIRFP FilterOnVbusCoefficients = {0.0028,	 1.0000,   -0.9944, 	   0   };

const float Capacitance_INV20K = 10e-6f;	
const float Capacitance_INV40K = 20e-6f;	
//	const float InitialNormVmag_INV = 1.28f;
const float InitialNormVmag_INV[] = 
{
	1.323f, //Hobbit20k
	1.517f  //Hobbit40k
};
//CAN fail Vmag
const float Vmag_CANfail[] = 
{
	0.040f, //Hobbit20k 100%load +0.080
	0.060f  //Hobbit40k 100%load +0.120
};
//DCFF off threshold, Id< ~7.5%(230)
const float DcffOffIdfil[] = 
{
    0.0180f, //Hobbit20k: 0.0180f=2.32Arms*1.414/182A
    //0.0360f  //Hobbit40k 
    0.0400f  //Hobbit 30/40k update to fix HB93E-36
};

const float OnePhaseLegLoadShareGain = 0.003f;


uint16_t InvHWCurrentLimit = 0;
uint16_t InvHWCurrentLimitReal = 0;
float InverterTs = InverterTs_HV;
float  InverterCap;

uint16_t HVDCLinkVoltage;
uint16_t DCOVLevel;
stThreePhase SelectiveTripThreePhase;
// global inverter
InverterControl Inverter;
float LoadSharePhaseCalByp = 0.0f;
stFirstOrderIIRFP FilterOnVbusTable;    
extern float DynamicLoadPercent;
extern uint16_t InterfaceBoardRevID;
extern float RailVoltagePositiveFastFilt;
extern float RailVoltageNegativeFastFilt;
float InverterVoltLoopGain = 1.0f;
float InverterCurrLoopGain = 1.0f;
float InvEctCurrLoopGain = 1.0f;
float InvDutyCompensation = 0.019f;
uint16_t LoopCoeffiIndex = LoopCoeffiIndex20K;
float DelayCount = 0.0f;


void InitInv(void)
{
    Inverter.ParametersInit();
}

// ***********************************************************************
// *
// *    FUNCTION: INVERTER_init 
// *
// *    DESCRIPTION: initializes inverter data to known default 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::ParametersInit( void )
{

    // initialize it again after the module being identified.
	if(UPMSystem == HV_20K)
	{
		InverterCap = Capacitance_INV20K;
	}
	else 	//40k
	{
		InverterCap = Capacitance_INV40K;		
	}
	InitialNormVmag = InitialNormVmag_INV[LoopCoeffiIndex];

	MasterRMSData.phA = 0;
    MasterRMSData.phB = 0;
    MasterRMSData.phC = 0;
    SyncSource = &BypassPLL.SineRef;
    ECTPFAngle = 0;
    ECTPFAngleTemp = 0;
    SlewRate = SLEW_RATE_PAR;

    RMSVoltageNormFactor = float( 1.0f / 230.0f);
    RMSVoltageRef.phA = 1.0;
    RMSVoltageRef.phB = 1.0;
    RMSVoltageRef.phC = 1.0;
    
    ee_LoadShareVoltageCal.phA = 0.0;
    ee_LoadShareVoltageCal.phB = 0.0;
    ee_LoadShareVoltageCal.phC = 0.0;

    VoltageNormFactor = float( 1.0f / 325.2691193f );       // 1/(230*sqrt(2))
    // PWM period
    InvPWMPeriod = ( CPUFrequency / PWMFrequency ) / 2L; 

    // current max
    RMSCurrentNormFactor = float( 1.0f / 182.0f);           //eep will over-write
    CurrentNormFactor = float( 1.0f / 182.0f);              // 1/182A

    InverterVGainsA = *InverterVoltageLoopCoefficients_ptr;
    InverterVGainsA.B0 = InverterVGainsA.B0 / VoltageNormFactor;
    InverterVGainsB = InverterVGainsA;
    InverterVGainsC = InverterVGainsA;
    InverterVSatFactorNew = 1.0f / ( ( InverterVGainsA.B1 + InverterVGainsA.B2 ) - ( InverterVGainsA.A1 + InverterVGainsA.A2 ) );

    InverterVGainsA2 = *InverterVoltageLoopCoefficients2_ptr;
    InverterVGainsB2 = InverterVGainsA2;
    InverterVGainsC2 = InverterVGainsA2;
    
    InverterVSatFactorNew2 = 1.0f / ( ( InverterVGainsA2.B1 + InverterVGainsA2.B2 ) - ( InverterVGainsA2.A1 + InverterVGainsA2.A2 ) );
    
    
    InverterIGainsA = *InverterCurrentLoopCoefficients_ptr;
    InverterIGainsA.B0 = InverterIGainsA.B0 / CurrentNormFactor;
    InverterIGainsB = InverterIGainsA;
    InverterIGainsC = InverterIGainsA;        
    InverterISatFactorNew = 1.0f / ((InverterIGainsA.B1 + InverterIGainsA.B2) - (InverterIGainsA.A1 + InverterIGainsA.A2 ));

    InverterIGainsA2 = *InverterCurrentLoopCoefficients2_ptr;
    InverterIGainsB2 = InverterIGainsA2;
    InverterIGainsC2 = InverterIGainsA2;

    IcapEstimatorGainsA = *CapCurrentEstimatorCoefficients_ptr;
    IcapEstimatorGainsB = *CapCurrentEstimatorCoefficients_ptr;
    IcapEstimatorGainsC = *CapCurrentEstimatorCoefficients_ptr;
    InverterVRMSGainsA = *InverterRMSLoopCoefficients_ptr;
    InverterVRMSGainsB = InverterVRMSGainsA;
    InverterVRMSGainsC = InverterVRMSGainsA;
    
    VRMSSatFactor = 1.0 / ( InverterVRMSGainsA.B1 - InverterVRMSGainsA.A1 );   
    
    InverterDCCompensatorA = *InverterDCCompensationCoefficients_ptr;
    InverterDCCompensatorB = *InverterDCCompensationCoefficients_ptr;
    InverterDCCompensatorC = *InverterDCCompensationCoefficients_ptr;
    
    InverterDCOffset.phA = 0;
    InverterDCOffset.phB = 0;
    InverterDCOffset.phC = 0;
    
    // clear status
    InverterStatus.all = 0;
    
    // 1 Hz sec default
    // since this is proportional controller, no attempt was made to set gains to achieve any particular slew rate
    // gains are high with rate limit
    FrequencyGain = SlewRateLimitTable[0].FrequencyGain;
    PhaseGain = SlewRateLimitTable[0].PhaseGain;
    SlewRateLimit = SlewRateLimitTable[0];
    PhaseError = 0.0f;
    
    LoadShareDroopTable = *LoadShareDroopCoefficients_ptr;
    ReactiveLoadShareDroopTable = *ReactiveLoadShareDroopCoefficients_ptr;
    LoadShareDCFeedForwardTable = *LoadShareDCFeedForwardCoefficients_ptr;
    ReactivePowerDroopTable = *ReactivePowerDroopCoefficients_ptr;
	
    HighDCVPhaseDroopTable = *HighDCVPhaseDroopCoefficients_ptr;
	HighDCVPhaseDroopGain = HighDCVPhaseDroopGainArray[LoopCoeffiIndex];
	
    ECTInverterIGainsA = *ECTCurrentLoopCoefficients_ptr;
    ECTInverterIGainsA.B0 = ECTInverterIGainsA.B0 / CurrentNormFactor;
    ECTInverterIGainsB = ECTInverterIGainsA;
    ECTInverterIGainsC = ECTInverterIGainsA;
        
//	    ReactiveCurrentFilterA = *ReactiveCurrentFilterCoefficients_ptr;
//	    ReactiveCurrentFilterB = *ReactiveCurrentFilterCoefficients_ptr;
//	    ReactiveCurrentFilterC = *ReactiveCurrentFilterCoefficients_ptr;
    InverterECTCurrentMag = 0;
    ECTPowerGains = *ECTPowerLoopCoefficients_ptr;
    ZeroSequenceMagnitude = 0;
    NominalVoltage = 230.0f;
    InvCurAutoZeroFilter = InvCurAutoZeroFilterGains;
    
    ActivePowerFiltered = 0.0f;
    ReactivePowerFiltered = 0.0f;
    FeedForwardGain = 1.0f;
    Off();
    
    MatchBypassForced = false;
    StoredBypassMatchState = 0;

	//resonant not need/ rc gain not need
//	    UPMControlsRMS = true;
        
    VoltageRefCorrection = 0.0f;
//	    DCFeedForwardParallelOnBoost = 1.043f;
//	    dcCompensation = 0.0f;
    
    // Puck/20120524, Inverter.SetSyncSource() should be initilized, and this can fix output frequency unstable when BASE_FREE_RUN
    InitSync();

	DCFeedForwardEnable = 1; 	//enable
	RMSPowerNormFactor = 0.0001;//1/10000;
	
	OnePhaseLegLoadShareGainLeg = OnePhaseLegLoadShareGain;	
	InverterLegOn = CAL_INV_All_ON;

	FilterOnVbusTable = FilterOnVbusCoefficients;
}

// ***********************************************************************
// *
// *    FUNCTION: INVERTER_RunVoltageRMSLoop - run once per cycle at zero cross
// *
// *    DESCRIPTION: RMS voltage loop
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::RunVoltageRMSLoop( void )	//20ms inv zero cross
{
    static stThreePhase temprms      = { 0.0f, 0.0f, 0.0f };

    NormRMSVoltageFeedback.phA = InverterVoltageRMS.RawRMS.phA * RMSVoltageNormFactor;
    NormRMSVoltageFeedback.phB = InverterVoltageRMS.RawRMS.phB * RMSVoltageNormFactor;
    NormRMSVoltageFeedback.phC = InverterVoltageRMS.RawRMS.phC * RMSVoltageNormFactor;

	//Wombat WOMBAT-271,   follow 9P: P9P100-404 fix. Load share phase droop gain increased when external CAN failure occurs.
//	    if ( NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR) && !AutoCal.GetState() ) 
	if ( ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed && !AutoCal.GetState() )
    {
		//3:3 same as 93PS
        LoadShareDroopTable.B0 = LoadShareDroopCoefficients_ptr->B0 * 2.8f;		//value debug both ok 2.8*(15/10)
		//for jira wombat515
		MasterRMSData.phA = Vmag_CANfail[LoopCoeffiIndex];
		MasterRMSData.phB = MasterRMSData.phA;
		MasterRMSData.phC = MasterRMSData.phA;

		//for V1.3.24 regression, when CAN short back, may cause trip
		// If a slave becomes the CAN master, it should start off with the same 
		// master RMS control value.  Sync this node's feedback loop to the master's 
		// feedback loop.
		FirstOrderIIRFPBackInitialize(MasterRMSData.phA, &InverterVRMSGainsA );
		FirstOrderIIRFPBackInitialize(MasterRMSData.phB, &InverterVRMSGainsB );
		FirstOrderIIRFPBackInitialize(MasterRMSData.phC, &InverterVRMSGainsC );
    }
    else
    {
    	LoadShareDroopTable.B0 = LoadShareDroopCoefficients_ptr->B0;
    }

    
    if ( InverterStatus.bit.VoltageRMSLoopOn )
    {
        if ( ParallelCan.ParGlobalOrData.InverterStatus.bit.MatchBypassRMS )
        {
            // Puck/20120529, Set Low limit for Inverter Voltage to slove issue that UPS can not transfer to online from bypass
            // when Bypass voltage is close to the Low loss voltage.
            float OutputMinLimit = float( OutNomVolts ) * 0.1 * BypassState().GetBypassRMSLowLimit() + 8;		//1/vbus * 0.1*
                
            if ( InverterVoltageRMS.RawRMS.phA > OutputMinLimit )
            {
                RMSVoltageRef.phA = ( ScreenMeters.BypassVoltageRMS.phA ) * RMSVoltageNormFactor;		//pu: rms  (1/Vrms)
            }
            else
            {
                RMSVoltageRef.phA = OutputMinLimit * RMSVoltageNormFactor;
            }
                
            if ( InverterVoltageRMS.RawRMS.phB > OutputMinLimit )
            {
                RMSVoltageRef.phB = ( ScreenMeters.BypassVoltageRMS.phB ) * RMSVoltageNormFactor;
            }
            else
            {
                RMSVoltageRef.phB = OutputMinLimit * RMSVoltageNormFactor;
            }
                
            if ( InverterVoltageRMS.RawRMS.phC > OutputMinLimit )
            {
                RMSVoltageRef.phC = ( ScreenMeters.BypassVoltageRMS.phC ) * RMSVoltageNormFactor;
            }
            else
            {
                RMSVoltageRef.phC = OutputMinLimit * RMSVoltageNormFactor;
            }
                
            if ( !InverterStatus.bit.BypassMatched )
            {
                if ( ( fabs( BypassVoltageRMS.RawRMS.phA - InverterVoltageRMS.RawRMS.phA ) < 3 ) &&
                     ( fabs( BypassVoltageRMS.RawRMS.phB - InverterVoltageRMS.RawRMS.phB ) < 3 ) &&
                     ( fabs( BypassVoltageRMS.RawRMS.phC - InverterVoltageRMS.RawRMS.phC ) < 3 ) )
                {
                    InverterStatus.bit.BypassMatched = 1;
                }
            }
        }
        else
        {
            RMSVoltageRef.phA = 1.0;
            RMSVoltageRef.phB = 1.0;
            RMSVoltageRef.phC = 1.0;
            InverterStatus.bit.BypassMatched = 0;//Yanming add for check voltage error
        }

        DebugFloat[1] = RMSVoltageRef.phA * 100;

        if ( !( InverterStatus.bit.DischargeCaps || InverterStatus.bit.OutputShort ))
        {
            if ( ( !EEP_IsInternalParallel() && !ParallelCan.UPSIsParallel() ) || UPMCalEnabled )
            {     
				temprms.phA = FirstOrderIIRFP( RMSVoltageRef.phA - NormRMSVoltageFeedback.phA, &InverterVRMSGainsA );	//2.rms(rms pu) loop
				temprms.phB = FirstOrderIIRFP( RMSVoltageRef.phB - NormRMSVoltageFeedback.phB, &InverterVRMSGainsB );
				temprms.phC = FirstOrderIIRFP( RMSVoltageRef.phC - NormRMSVoltageFeedback.phC, &InverterVRMSGainsC );
            }
            else
            {
                if ( ParallelCan.ParallelStatus.bit.Master )
                {
                    // master RMS control
                    if ( --ParallelRMSDownSample == 0 )	//40ms
                    { 
                        ParallelRMSDownSample = 2;
                        
						//for wombat jira483, unit join in may can fail, and cause other vrms abnormal
						if((!ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed)
							&& (!NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR)) )
						{
							MasterRMSData.phA = FirstOrderIIRFP( RMSVoltageRef.phA - NormRMSVoltageFeedback.phA, &InverterVRMSGainsA );
							MasterRMSData.phB = FirstOrderIIRFP( RMSVoltageRef.phB - NormRMSVoltageFeedback.phB, &InverterVRMSGainsB );
							MasterRMSData.phC = FirstOrderIIRFP( RMSVoltageRef.phC - NormRMSVoltageFeedback.phC, &InverterVRMSGainsC );
						
							ParallelCan.TransmitMasterRMSPacket( MasterRMSData );
						}
                    }
                    else
                    {
                        // deliberately delayed one round, so that other UPMS and this one apply the new mag command at the same time
                        temprms.phA = MasterRMSData.phA;
                        temprms.phB = MasterRMSData.phB;
                        temprms.phC = MasterRMSData.phC;
                    }
                }
                else
                {
                    // slave unit use master data
                    temprms.phA = MasterRMSData.phA;
                    temprms.phB = MasterRMSData.phB;
                    temprms.phC = MasterRMSData.phC;
                    // If a slave becomes the CAN master, it should start off with the same 
                    // master RMS control value.  Sync this node's feedback loop to the master's 
                    // feedback loop.
                    FirstOrderIIRFPBackInitialize(MasterRMSData.phA, &InverterVRMSGainsA );
                    FirstOrderIIRFPBackInitialize(MasterRMSData.phB, &InverterVRMSGainsB );
                    FirstOrderIIRFPBackInitialize(MasterRMSData.phC, &InverterVRMSGainsC );					
                }
                
            }   // end if ( UPMIsParallel ) 
            
            if ( ParallelCan.GetState() != CanDriver::Initialization )
            {
                ParallelCan.MyCalData().id = int16_t( ActivePowerFiltered  * 32767.0f  );
                ParallelCan.MyCalData().powerA = int16_t( InverterPower.ActivePowerFiltered.phA );
                ParallelCan.MyCalData().powerB = int16_t( InverterPower.ActivePowerFiltered.phB );
                ParallelCan.MyCalData().powerC = int16_t( InverterPower.ActivePowerFiltered.phC );
            }
        }   // end if ( !InverterStatus.bit.SoftwareCurrentLimited ) 

        float rmsLimit = 0;
        //Koala rms loop limit is larger, since feedforward magnitude is larger.
        //normal steady error(20k): cmd_1.55->real_1.0 =>  real_0.25->cmd_0.3875
        //about 0.387, then set to 0.4
        rmsLimit = 0.4f;

        // Limit to about +/- 20% of nominal, set to the max OV/UV limits + 5%
        if ( temprms.phA > rmsLimit )				
        {
            temprms.phA = rmsLimit;					
            InverterVRMSGainsA.X1 = temprms.phA * VRMSSatFactor;
        }
        else
        {
            if ( temprms.phA < -rmsLimit )
            {
                temprms.phA = -rmsLimit;
                InverterVRMSGainsA.X1 = temprms.phA * VRMSSatFactor;
            }
        }    
		
        
        if ( temprms.phB > rmsLimit )
        {
            temprms.phB = rmsLimit;
            InverterVRMSGainsB.X1 = temprms.phB * VRMSSatFactor;
        }
        else
        {
            if( temprms.phB < -rmsLimit )
            {
                temprms.phB = -rmsLimit;
                InverterVRMSGainsB.X1 = temprms.phB * VRMSSatFactor;
            }
        }
            
        if ( temprms.phC > rmsLimit )
        {
            temprms.phC = rmsLimit;
            InverterVRMSGainsC.X1 = temprms.phC * VRMSSatFactor;
        }
        else
        {
            if ( temprms.phC < -rmsLimit )
            {
                temprms.phC = -rmsLimit;
                InverterVRMSGainsC.X1 = temprms.phC * VRMSSatFactor;
            }
        }    
           
    }
    else
    {
        // !InverterStatus.bit.VoltageRMSLoopOn
        temprms.phA = 0;
        temprms.phB = 0;
        temprms.phC = 0;
        
        MasterRMSData.phA = 0;
        MasterRMSData.phB = 0;
        MasterRMSData.phC = 0;

        InverterVRMSGainsA.X1 = 0;
        InverterVRMSGainsB.X1 = 0;
        InverterVRMSGainsC.X1 = 0;
        InverterStatus.bit.BypassMatched = 0; 

		//for wombat jira611, paral shutdown to online, may ac ov(random 1~2/20times)
		ParallelRMSDownSample = 2;
    }

    if ( InverterStatus.bit.DischargeCaps )
    {
        InverterVmag.phA = .03 * ( InitialNormVmag + ( temprms.phA + ee_LoadShareVoltageCal.phA ) );
        InverterVmag.phB = .03 * ( InitialNormVmag + ( temprms.phB + ee_LoadShareVoltageCal.phB ) );
        InverterVmag.phC = .03 * ( InitialNormVmag + ( temprms.phC + ee_LoadShareVoltageCal.phC ) );
    }    
	else if (InverterStatus.bit.InverterOnFromSuspend )		//1.ESS on 1*20ms
    {
        // current compensator initialization
        InverterIGainsA.X1=(0.02199*InverterCurrentFeedback.abc.phA-InverterIGainsA.B0*(0.028274-InverterCurrentFeedback.abc.phA))*InverterISatFactorNew;
        InverterIGainsA.X2=InverterIGainsA.X1;
        InverterIGainsB.X1=(0.02199*InverterCurrentFeedback.abc.phB-InverterIGainsB.B0*(-0.014137-InverterCurrentFeedback.abc.phB))*InverterISatFactorNew;
        InverterIGainsB.X2=InverterIGainsB.X1;
        InverterIGainsC.X1=(0.02199*InverterCurrentFeedback.abc.phC-InverterIGainsC.B0*(-0.014137-InverterCurrentFeedback.abc.phC))*InverterISatFactorNew;
        InverterIGainsC.X2=InverterIGainsC.X1;
        temprms.phA = 0;
        temprms.phB = 0;
        temprms.phC = 0;

        MasterRMSData.phA = 0;
        MasterRMSData.phB = 0;
        MasterRMSData.phC = 0;

        InverterVRMSGainsA.X1= 0;
        InverterVRMSGainsB.X1= 0;
        InverterVRMSGainsC.X1= 0;

        InverterStatus.bit.InverterOnFromSuspend = 0;
    }
    else
    {
        float vdroopA = 0;
        float vdroopB = 0;
        float vdroopC = 0;

//	        if (!AutoCal.GetState())
//			if((InverterStatus.bit.InverterOpenLoop) || (!AutoCal.GetState()))
        {	
			//Under auto-cali, still use v-droop 
			//InitDropPower: paral-3.3k->0; single- 0
	        vdroopA = ( InitDropPower + LoadPower.ActivePowerFiltered.phA) * (VoltageDroopCoefficient) * DelayCount;	//1.(Po_active+0)Kp_droop
	        vdroopB = ( InitDropPower + LoadPower.ActivePowerFiltered.phB) * (VoltageDroopCoefficient) * DelayCount;
	        vdroopC = ( InitDropPower + LoadPower.ActivePowerFiltered.phC) * (VoltageDroopCoefficient) * DelayCount;
			
        }

		//end
        DebugFloat[1] += temprms.phA;
        DebugFloat[0] = vdroopA;
		InverterVmag.phA = FeedForwardGain * ( InitialNormVmag 
							+ ( temprms.phA + ee_LoadShareVoltageCal.phA) ) - vdroopA;
		InverterVmag.phB = FeedForwardGain * ( InitialNormVmag 
							+ ( temprms.phB + ee_LoadShareVoltageCal.phB) ) - vdroopB;
		InverterVmag.phC = FeedForwardGain * ( InitialNormVmag 
							+ ( temprms.phC + ee_LoadShareVoltageCal.phC) ) - vdroopC;
		//end
			
		
    }	
}

    
// ***********************************************************************
// *
// *    FUNCTION: INVERTER_run - ISR function 
// *
// *    DESCRIPTION: Inner inverter controller
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void InverterControl::Run_ISR( void )
{   
    //
    // * This function runs from DMA ISR, no BIOS calls allowed
    //
    
    UpdateInverterAngle_ISR();
    
    // normalize measurements
    stThreePhase tempThreePhase;
    tempThreePhase.phA = RawAdcDataPtr->st.InverterVoltage.phA - InverterDCOffset.phA;
    tempThreePhase.phB = RawAdcDataPtr->st.InverterVoltage.phB - InverterDCOffset.phB;
    tempThreePhase.phC = RawAdcDataPtr->st.InverterVoltage.phC - InverterDCOffset.phC;
    
    InverterVoltageFeedback.abc = tempThreePhase;
    InverterCurrentFeedback.abc.phA = RawAdcDataPtr->st.InverterCurrent.phA;
    InverterCurrentFeedback.abc.phB = RawAdcDataPtr->st.InverterCurrent.phB;
    InverterCurrentFeedback.abc.phC = RawAdcDataPtr->st.InverterCurrent.phC;
    InverterVoltageFeedback.abc.phA *= VoltageNormFactor;		//1.  1/(230*sqrt(2)
    InverterVoltageFeedback.abc.phB *= VoltageNormFactor;
    InverterVoltageFeedback.abc.phC *= VoltageNormFactor;

    InverterCurrentFeedback.abc.phA *= CurrentNormFactor;		//2.  1/182A
    InverterCurrentFeedback.abc.phB *= CurrentNormFactor;
    InverterCurrentFeedback.abc.phC *= CurrentNormFactor;

    InverterPWMController_ISR();   
}

// ***********************************************************************
// *
// *    FUNCTION: UpdateInverterAngle_ISR - ISR function  
// *
// *    DESCRIPTION: Updates inverter reference
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
inline void InverterControl::UpdateInverterAngle_ISR( void )	//18k
{   
    // Update for pullchain(rsd), follow GWS, no byp theta on
    if ( !InverterStatus.bit.InverterOn && MCUStateMachine.PullChainSyncStart )
    {
        // Hold angle at zero and reset frequency for pull-chain start
        SineRef.UpdateFrequencyAndAngle( BaseSineRef.GetBaseFrequency(), 0.0f );
    }
    else
    {
        SineRef.UpdateAngle();	// Thrta update
    }

	if ( SineRef.Angle > PI )
    {
        SineRef.Angle -= ( 2.0 * PI );
    }
    else if ( SineRef.Angle < -PI )
    {
        SineRef.Angle += ( 2.0 * PI );		
    }	
	
    float tempAngle = SineRef.Angle - ECTPFAngle;	

    //3.InverterSineRef.phA/B/C:  1*sin(t)/1*sin(t-2/3pi)/1*sin(t-2/3pi)
    sin_3_phase(tempAngle, &InverterSineRef.phA, &InverterSineRef.phB, &InverterSineRef.phC); //change based on code review 1.06

	//3.1: pu translate 1/(230*1.414)
    SelectiveTripThreePhase.phA = (RawAdcDataPtr->st.InverterVoltage.phA - InverterDCOffset.phA) * VoltageNormFactor;
    SelectiveTripThreePhase.phB = (RawAdcDataPtr->st.InverterVoltage.phB - InverterDCOffset.phB) * VoltageNormFactor;
    SelectiveTripThreePhase.phC = (RawAdcDataPtr->st.InverterVoltage.phC - InverterDCOffset.phC) * VoltageNormFactor;
    
    if ( InverterStatus.bit.ECTCurrentLoop )		//5.1.Solar: Curr_Inv_ref
    {
        // voltage reference is really current reference in ECT
        InverterVoltageReference.abc.phA = InverterECTCurrentMag * InverterSineRef.phA;
        InverterVoltageReference.abc.phB = InverterECTCurrentMag * InverterSineRef.phB;
        InverterVoltageReference.abc.phC = InverterECTCurrentMag * InverterSineRef.phC;
    }
    else        //5.2.UPS: VoltInvRef
    {
        //(1)Vabc_ref = Vm*sin(wt)/ :   vm=1.4(normal:1.0)*sin(wt)
        InverterVoltageReference.abc.phA = InverterVmag.phA * InverterSineRef.phA;
        InverterVoltageReference.abc.phB = InverterVmag.phB * InverterSineRef.phB;
        InverterVoltageReference.abc.phC = InverterVmag.phC * InverterSineRef.phC;
        
        //(2)Iabc_ref = Vabc*jwC (Vabc/(1/wC))
        InverterCurrentReference.abc.phA = FirstOrderIIRFP(  InverterVoltageFeedback.abc.phA, &IcapEstimatorGainsA );
        InverterCurrentReference.abc.phB = FirstOrderIIRFP(  InverterVoltageFeedback.abc.phB, &IcapEstimatorGainsB );
        InverterCurrentReference.abc.phC = FirstOrderIIRFP(  InverterVoltageFeedback.abc.phC, &IcapEstimatorGainsC );
    }
}

// ***********************************************************************
// *
// *    FUNCTION: InverterPWMController_ISR - ISR function 
// *
// *    DESCRIPTION: Inverter PWM control
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
inline void InverterControl::InverterPWMController_ISR( void )	//13k
{
    //
    // * This function runs from DMA ISR, no BIOS calls allowed
    //

    if ( InverterStatus.bit.InverterOn )
    {
        if ( InverterStatus.bit.InverterOpenLoop )
        {
			float coefficient = 0.00136986f;
			coefficient = 1.0f / ( RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative );
            //coefficient = 1.0f / (RailVoltagePositiveFastFilt - RailVoltageNegativeFastFilt);
            //coefficient = 1.0f / Rectifier.GetDCLinkVoltageRef();		//(0.5/365)	
//	        coefficient *= (float(OutNomVolts) * 0.1f) * 2.0f * SQRT_2 * (1.0f/1.36f); 	//K=650.44* (1/700)
			coefficient *= (float(OutNomVolts) * 0.1f) * 2.0f * SQRT_2 * (1.0f/InitialNormVmag);	

			if(coefficient > 1.0f)
			{
				coefficient = 1.0f;
			}
	
			PWMCommand.phA = (InverterVoltageReference.abc.phA + (InverterDCOffset.phA*VoltageNormFactor)) * coefficient;		//rms pu:1.4*sin(wt)
			PWMCommand.phB = (InverterVoltageReference.abc.phB + (InverterDCOffset.phB*VoltageNormFactor)) * coefficient;
			PWMCommand.phC = (InverterVoltageReference.abc.phC + (InverterDCOffset.phC*VoltageNormFactor)) * coefficient;				
        }
        else
        {
            CombinedVector invOut;
	
            if ( InverterStatus.bit.ECTCurrentLoop )	//2.1.Solar: only Curr Loop
            {
                // voltage reference is really current reference in ECT
                //(2.1). Curr loop: IIR
                invOut.aby.alpha = FirstOrderIIRFP( (( InverterVoltageReference.aby.alpha - InverterCurrentFeedback.aby.alpha )*InvEctCurrLoopGain), &ECTInverterIGainsA );
                invOut.aby.beta = FirstOrderIIRFP( (( InverterVoltageReference.aby.beta - InverterCurrentFeedback.aby.beta )*InvEctCurrLoopGain), &ECTInverterIGainsB );
                invOut.aby.gamma = FirstOrderIIRFP( (( InverterVoltageReference.aby.gamma - InverterCurrentFeedback.aby.gamma )*InvEctCurrLoopGain), &ECTInverterIGainsC );
                PWMCommand = invOut.abc;
            }
            else
            {  
                //
                // written as alpha-beta-gamma, but is really either abc or aby depending on what was loaded into the reference and feedback structures
                //
                
                //2).Vab_ref*IIR * (1+Vdc_out) 
                invOut.aby.alpha  = ( VoltageRefCorrection + 1.0f ) * SecondOrderIIRFP( (( InverterVoltageReference.aby.alpha - InverterVoltageFeedback.aby.alpha )*InverterVoltLoopGain), &InverterVGainsA );
                invOut.aby.beta   = ( VoltageRefCorrection + 1.0f ) * SecondOrderIIRFP( (( InverterVoltageReference.aby.beta  - InverterVoltageFeedback.aby.beta )*InverterVoltLoopGain), &InverterVGainsB );
                //4).3/4 : ((1+Vdc_out)Vg_ref - Vg_fb) *IIR 				
                invOut.aby.gamma  = (VoltageRefCorrection + 1.0f) * SecondOrderIIRFP( (( InverterVoltageReference.aby.gamma	- InverterVoltageFeedback.aby.gamma	)*InverterVoltLoopGain), &InverterVGainsC );

//					RawAdcData.st.ControlDebug[0] = InverterVoltageReference.aby.alpha;
//					RawAdcData.st.ControlDebug[1] = InverterVoltageFeedback.aby.alpha;
//					RawAdcData.st.ControlDebug[2] = ( InverterVoltageReference.aby.alpha - InverterVoltageFeedback.aby.alpha );
//	
//					RawAdcData.st.ControlDebug[3] = invOut.aby.alpha;
				
                //2.Iabr Curr Loop
                //2.1)Vout_Loop * IIR_VGainA2
                invOut.aby.alpha  = SecondOrderIIRFP( (invOut.aby.alpha) * InverterCurrLoopGain, &InverterVGainsA2);
                invOut.aby.beta   = SecondOrderIIRFP( (invOut.aby.beta) * InverterCurrLoopGain, &InverterVGainsB2);
                invOut.aby.gamma  = SecondOrderIIRFP( (invOut.aby.gamma) * InverterCurrLoopGain, &InverterVGainsC2);
//					RawAdcData.st.ControlDebug[4] = invOut.aby.alpha;
                if (!InverterStatus.bit.InverterOnFromSuspend)		//during 20ms Ess->online, not run i loop
                {
                    //2.4)Loop Out:  InvOut= Vabr_out+Iabr_out 
                    //2.2)Curr Loop:  =IIR_IGA(IIR_IGA2(Err_Iabr)) 
//						RawAdcData.st.ControlDebug[5] = invOut.aby.alpha;

                    invOut.aby.alpha += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.alpha - InverterCurrentFeedback.aby.alpha )), &InverterIGainsA )  , &InverterIGainsA2);
//						RawAdcData.st.ControlDebug[6] = InverterCurrentReference.aby.alpha;
//						RawAdcData.st.ControlDebug[7] = InverterCurrentFeedback.aby.alpha;
//						RawAdcData.st.ControlDebug[8] = invOut.aby.alpha - RawAdcData.st.ControlDebug[5];
//						RawAdcData.st.ControlDebug[9] = invOut.aby.alpha;

                    invOut.aby.beta  += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.beta  - InverterCurrentFeedback.aby.beta  )), &InverterIGainsB )  , &InverterIGainsB2);
                    invOut.aby.gamma += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.gamma - InverterCurrentFeedback.aby.gamma )), &InverterIGainsC )  , &InverterIGainsC2);
                }
                else if ( MCUStateMachine.GetStatus().bit.PostESSMode )
                {
                    invOut.aby.alpha += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.alpha - InverterCurrentFeedback.aby.alpha )), &InverterIGainsA ) , &InverterIGainsA2);
                    invOut.aby.beta  += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.beta  - InverterCurrentFeedback.aby.beta  )), &InverterIGainsB ) , &InverterIGainsB2);
                    invOut.aby.gamma += SecondOrderIIRFP(  SecondOrderIIRFP( (( InverterCurrentReference.aby.gamma - InverterCurrentFeedback.aby.gamma )), &InverterIGainsC ) , &InverterIGainsC2);
                }
                
                PWMCommand = invOut.abc;
            }
        }
		
        
        //Prevents polarity from being wrong and possible blow up when the PWM turns on during a forward transfer from ESS
        if (DSPOutRegister.GpoA.bit.InvPWMdisable)
        {
            DSPOutRegister.GpoA.bit.InvPWMdisable = 0;
        }
    }

	
	DeadZoneCompenste(PWMCommand.phA,InvDutyCompensation,&PWMCommand.phA);
	DeadZoneCompenste(PWMCommand.phB,InvDutyCompensation,&PWMCommand.phB);
	DeadZoneCompenste(PWMCommand.phC,InvDutyCompensation,&PWMCommand.phC);
	
	
	LimitDuty( &PWMCommand.phA );	// use limits to avoid PLDs deadtime region.
	LimitDuty( &PWMCommand.phB );
	LimitDuty( &PWMCommand.phC );
	
//		RawAdcData.st.ControlDebug[11] = PWMCommand.phA;

    SetInv3LvlPwm( PWMCommand.phA, InvPWMPeriod, 1);
    SetInv3LvlPwm( PWMCommand.phB, InvPWMPeriod, 2);
    SetInv3LvlPwm( PWMCommand.phC, InvPWMPeriod, 3);
}


extern bool dcff_test;
// ***********************************************************************
// *
// *    FUNCTION: RunPLLFast  
// *
// *    DESCRIPTION: Gets phase error, updates phase correction, runs load share
// *                 fast call from HWI, 1.125Hz rate 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void InverterControl::RunPLLFast( void )		// 1.125kHz
{
    float dcLinkError=0.0f;
    float reactivePower;

    if ( MCUStateMachine.GetMetersReadyFlag() &&
         ( SyncSource != NULL ) )
    {
        float pllPhaseCorrection;
        float activePower;
			
        // get phase error
        {
            CriticalSection enter;
            PhaseError = SyncSource->Angle - SineRef.Angle;		//1.threta Q_err
        }
        // apply fixed offset
//	        PhaseError += SyncSource->GetOffset();
		if((!EnableOpenLoop) || (InverterStatus.bit.ECTCurrentLoop == 1))	//1.close loop and ect add offset_phase		
		{
			PhaseError += SyncSource->GetOffset();
		}
		
        // wrap error
        if ( PhaseError > PI )
        {
            PhaseError -= ( 2.0f * PI );
        }
        else
        {    
            if ( PhaseError < -PI )
            {
                PhaseError += ( 2.0f * PI );
            }        
        }
        
        pllPhaseCorrection = PhaseError * PhaseGain;		//2.Kgain=0.006272052f

        // rate limit
        if ( pllPhaseCorrection > SlewRateLimit.PhaseLimit )
        {
            pllPhaseCorrection = SlewRateLimit.PhaseLimit;
        }
        else
        {
            if ( pllPhaseCorrection < -SlewRateLimit.PhaseLimit )
            {
                pllPhaseCorrection = -SlewRateLimit.PhaseLimit;
            }
        }
		//1.loadshare cal: //200/32767*
		
//	        if ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() )
//			if ((EEP_IsInternalParallel() && (!UPMCalEnabled)) || 
//				ParallelCan.UPSIsParallel() )
//	        {

	        // loadshare, get active current component
	        // if this is three wire, current feedback is alpha-beta so re-normalize. Is not needed for 4W but
	        // doesn't hurt.
	        stThreePhase currentNorm;
	        
	        currentNorm.phA = RawAdcDataPtr->st.InverterCurrent.phA * CurrentNormFactor;
	        currentNorm.phB = RawAdcDataPtr->st.InverterCurrent.phB * CurrentNormFactor;
	        currentNorm.phC = RawAdcDataPtr->st.InverterCurrent.phC * CurrentNormFactor;
	        
	        //changes based on code review 1.06
	        float cos_th = 0.0f;
	        float cos_thMINUS120 = 0.0f;
	        float cos_thPLUS120 = 0.0f; // used to hold values for cos(SineRef.Angle) cos(SinRef.Angle - 120) cos(SineRef.Angle +120)
	        cos_3_phase(SineRef.Angle, &cos_th, &cos_thMINUS120,  &cos_thPLUS120);
	           

	        // loadshare, get active current component
	        abc_to_dq0( &currentNorm, &InverterCurrentDQ0Data, SineRef.Angle );

	        // apply filter
	        ActivePowerFiltered = ActivePowerFilter.Run( InverterCurrentDQ0Data.Sd );
	        
	        activePower = ActivePowerFiltered;
	        reactivePower = ReactivePowerFilter.Run( InverterCurrentDQ0Data.Sq );
			ReactivePowerFiltered = reactivePower;
//			RawAdcData.st.ControlDebug[0] = activePower*1000.0f;
//			RawAdcData.st.ControlDebug[1] = InverterCurrentDQ0Data.Sq*1000.0f;
//			RawAdcData.st.ControlDebug[2] = reactivePower*1000.0f;
			
		if ((EEP_IsInternalParallel() && (!UPMCalEnabled)) || 
			ParallelCan.UPSIsParallel() )
		{
			// auto-calibration load share error offset. Is put here so that it's not
			// subject to slew rate limit, since it shouldn't be slewing
			pllPhaseCorrection += ( SyncSource->GetPhaseCal() * PhaseGain );

			//9P
			//*********************************************************************************
			// LoadShare Algorithm: DC Voltage Compensation
			//
			// If the link is rising when On Battery, Phase forward because the other inverters
			// are backfeeding the link making it go up.
			//*********************************************************************************			
            // P9P-425&413 fix. No dc compensation when rectifier is in "from suspend" state due to DC link voltage greater than rectifier set point voltage on 480V systems.
	        if( ((BatteryConverter.BoChMachineState == BATTERY_ON_BOOST_STATE ) &&
	             (Rectifier.GetState()!=RECTIFIER_FROM_SUSPEND_STATE))) //||
//		             (Rectifier.GetState()==RECTIFIER_NORMAL_STATE))
	        {
	            // rectifier ref is 10V higher than boost ref
	            dcLinkError = (Rectifier.GetDCLinkVoltageRef()+10.0f) - ( RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative );
				
	            // do nothing for low link, only high
				if(dcLinkError < 0.0f)
	            {
	                // feed error forward to active power. Idea is to help at no load/light loads on
	                // battery, not to regulate the link
	                //
	            	 activePower +=  dcLinkError * HighDCVPhaseDroopGain;		//HighDCVPhaseDroopGain=0.01

	            }
	            else
	            {
	            	HighDCVPhaseDroopTable.X1=0.0f;
	            	HighDCVPhaseDroopTable.X2=0.0f;
	            }
	        }
	        else if (Rectifier.GetState()==RECTIFIER_FROM_SUSPEND_STATE)  // PANPRE-1839 fix
	        {
	            dcLinkError = DCLINK_810_VOLT - ( RawAdcDataPtr->st.RailVoltagePositive - RawAdcDataPtr->st.RailVoltageNegative );
	            
	            if (dcLinkError < 0.0f)
	            {
	                activePower +=  dcLinkError * HighDCVPhaseDroopGain;
	            }
	        }
	        else
	        {
	            HighDCVPhaseDroopTable.X1=0.0f;
	            HighDCVPhaseDroopTable.X2=0.0f;
	        }	

	        // run control
	        //Id_fil: PD 0.0012
	        float loadSharePhaseDroop = FirstOrderIIRFP( activePower, &LoadShareDroopTable );
//			RawAdcData.st.ControlDebug[3] = activePower*1000.0f;
//			RawAdcData.st.ControlDebug[4] = loadSharePhaseDroop*1000.0f;
	        
	        if( !AutoCal.GetState()       )
	        {
				//Iq_no_fil: PD 0.00144
	            ReactivePowerPhaseDroop = FirstOrderIIRFP(InverterCurrentDQ0Data.Sq, &ReactivePowerDroopTable);
//				RawAdcData.st.ControlDebug[5] = InverterCurrentDQ0Data.Sq*1000.0f;
//				RawAdcData.st.ControlDebug[6] = ReactivePowerPhaseDroop*1000.0f;

				//Iq_fil: P 0.00035
	            ReactivePowerPhaseDroop -= FirstOrderIIRFP( reactivePower, &ReactiveLoadShareDroopTable);
//				RawAdcData.st.ControlDebug[7] = reactivePower*1000.0f;
//				RawAdcData.st.ControlDebug[8] = RawAdcData.st.ControlDebug[6] - ReactivePowerPhaseDroop*1000.0f;	//reactivePower_droop
//				RawAdcData.st.ControlDebug[9] = ReactivePowerPhaseDroop*1000.0f;

				//as, AngleStep:50hz*2pi/18k = 0.0175
				//0.000175f equal to: 0.5Hz
	            if (ReactivePowerPhaseDroop > 0.000175f)  // 0.5Hz
	            {
	                ReactivePowerPhaseDroop = 0.000175f;
	            }
	            if (ReactivePowerPhaseDroop <= -0.000175f)
	            {
	                ReactivePowerPhaseDroop = -0.000175f;
	            }
//				RawAdcData.st.ControlDebug[10] = ReactivePowerPhaseDroop*1000.0f;

	            loadSharePhaseDroop += ReactivePowerPhaseDroop;
//				RawAdcData.st.ControlDebug[11] = loadSharePhaseDroop*1000.0f;
					
	        }
	        else
	        {
	        	loadSharePhaseDroop=0;
	        }
	        // combined result
	        SineRef.AngleCorrection = pllPhaseCorrection - loadSharePhaseDroop;
//			RawAdcData.st.ControlDebug[12] = pllPhaseCorrection*1000.0f;
//			RawAdcData.st.ControlDebug[13] = loadSharePhaseDroop*1000.0f;
        }
        else	//droop under parral mode
        {
            SineRef.AngleCorrection = pllPhaseCorrection;
        }		
    }
    else	
    {
        SineRef.AngleCorrection = 0.0f;

    }

	
    // DC link feedforward
    if ( DCFeedForwardEnable            &&
         !AutoCal.GetState()            && // Enable DC ff only when all units stop autocalibration
         InverterStatus.bit.InverterOn  &&
         //KOALA400-635 fix, remove UpmIsParallel().
         //Here is a reserved code. After Koala 15k VAL, will remove it. 
         ((EEP_IsInternalParallel() && (!UPMCalEnabled)) || 
           ParallelCan.UPSIsParallel()) 
//	           ( (DCFeedForwardEnable&0x0002) == 0 ) )		//1&0b10=0
        )
    {
		float fDCReference;
		
			fDCReference = 730.0f;

			VoltageRefCorrection = fDCReference / (RailVoltagePositiveFastFilt - RailVoltageNegativeFastFilt) - 1.0f;
		{
            //For Id < load~7.5%(230V), off dcff
            if((ActivePowerFiltered > DcffOffIdfil[LoopCoeffiIndex]))
            {
                //lowpass filter: 3:3->20Hz cut-off freq
                if(!dcff_test)
                {
                    VoltageRefCorrection = FirstOrderIIRFP( VoltageRefCorrection, &LoadShareDCFeedForwardTable );
                }
            }
            else
            {
                VoltageRefCorrection = 0.0f;
                LoadShareDCFeedForwardTable.X1 = 0.0f;
            }
        }

		//Delta_Vmag = 325*2/725 - 325*2/(730)=0.8966 - 0.8904=0.0062
        //Max DClink 810v, Min DCLinkSet 730v, (730/810-1)=-0.136
		if (VoltageRefCorrection > 0.014)	//0.0139=730/720- 1.0;
		{
			VoltageRefCorrection = 0.014f;
		}
		else if (VoltageRefCorrection < -0.120f)		//-0.0987=730/810-1.0;   -0.0135=730/740-1.0
		{
			VoltageRefCorrection = -0.120f;
		}
			
    }
	else
	{
        VoltageRefCorrection = 0.0f;		
	}

	//parral inv volt error feed check
    if ( ParallelCan.UPSIsParallel() 
		&& EnableSelectiveTrip )
    {
        ParallelSystemSelectiveTrip();	//1.Neg protect
    }

}

// ***********************************************************************
// *
// *    FUNCTION: PLLPeriodicFunction  
// *
// *    DESCRIPTION: updates norm factor and frequency. Called every
// *                 20ms  
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::PLLPeriodicFunction( void )		//about 20ms inv zero cross
{
    if ( MCUStateMachine.GetMetersReadyFlag() )		
    {
        // get target frequency
        float targetFrequency = SyncSource->Frequency;


        // limit check
        if ( targetFrequency > ( BypassInterface::GetBypassHighFreqLimit() + 0.5f ) )
        {
            targetFrequency = BypassInterface::GetBypassHighFreqLimit() + 0.5f;
        }
        else
        {
            if ( targetFrequency < ( BypassInterface::GetBypassLowFreqLimit() - 0.5f ) )
            {
                targetFrequency = BypassInterface::GetBypassLowFreqLimit() - 0.5f;
            }
        }

		
        // get frequency error
        //Kfreq=0.1f
        float frequencyCorrection = ( targetFrequency - SineRef.Frequency ) * FrequencyGain;
        
        if ( frequencyCorrection > SlewRateLimit.FrequencyLimit )
        {
            frequencyCorrection = SlewRateLimit.FrequencyLimit;
        }
        else
        {
            if ( frequencyCorrection < -SlewRateLimit.FrequencyLimit )
            {
                frequencyCorrection = -SlewRateLimit.FrequencyLimit;
            }
        }
        
        // apply correction
        SineRef.UpdateFrequencyAndAngleStep( SineRef.Frequency + frequencyCorrection );
        
        // check for phase lock
        if ( fabs( PhaseError ) < PLLMaxPhaseError )
        {
            Locked.Debounce( true );	//active:16
            
//				SetRsdTestIOHigh();
        }
        else
        {
            Locked.Debounce( false );	//deactive:4*20ms
//				SetRsdTestIOLow();

        }

		//for jira473 begin,Vin=127~147v, paral loadshare may fail
		if((UtilityVoltageRMS.FilteredRMS.phA > 170.0f) 
			|| (UtilityVoltageRMS.FilteredRMS.phB > 170.0f)
			|| (UtilityVoltageRMS.FilteredRMS.phC > 170.0f))
		{
			InverterStatus.bit.LoadShareDCFFB0Tune = false; //One unit Vac>170V
		}
		else if((UtilityVoltageRMS.FilteredRMS.phA < 160.0f) 
			&& (UtilityVoltageRMS.FilteredRMS.phB < 160.0f)
			&& (UtilityVoltageRMS.FilteredRMS.phC < 160.0f))
		{
			InverterStatus.bit.LoadShareDCFFB0Tune = true;	//all unit Vac<160
			
		}
		//end
    }
} 

// ***********************************************************************
// *
// *    FUNCTION: SetInverterVoltage - TSK function 
// *
// *    DESCRIPTION: set inverter frequency
// *
// *    ARGUMENTS: voltage: desired voltage in V*10.
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::SetInverterVoltage( uint16_t voltage )
{
    if ( ValidInverterVoltage( voltage ) )
    {
		NominalVoltage = float( voltage ) / 10.0f;
		RMSVoltageNormFactor = 1.0f / NominalVoltage;

		// multipy by sqrt(2)
		float temp = NominalVoltage * SQRT_2;
		// invert
		VoltageNormFactor = 1.0 / temp;

		InverterVGainsA.B0 = InverterVoltageLoopCoefficients_ptr->B0 * temp;
		InverterVGainsB.B0 = InverterVGainsA.B0;
		InverterVGainsC.B0 = InverterVGainsA.B0;

		PeakVoltage = NominalVoltage * SQRT_2;
	//	    PeakVoltageLL = PeakVoltage * SQRT_3;
	
//	        if (UPMSystem == HV_20K)
//			{
//		        switch ( OutNomVolts )
//		        {
//		            case 2200:
//		                HVDCLinkVoltage = 720;
//		                RectHWCurrentLimit = 76;
//		                InvHWCurrentLimit = 83;
//		                DCOVLevel = 420;
//		                break;
//		            case 2300:
//		                HVDCLinkVoltage = 720;
//		                RectHWCurrentLimit = 73;
//		                InvHWCurrentLimit = 80;
//		                DCOVLevel = 420;
//		                break;
//		            case 2400:
//		                HVDCLinkVoltage = 760;
//		                RectHWCurrentLimit = 70;
//		                InvHWCurrentLimit = 76;
//		                DCOVLevel = 440;
//		                break;
//		            default:
//		                break;
//		        }
//			}
//			else if (UPMSystem == HV_40K)
//			{			
//		        switch ( OutNomVolts )
//		        {
//		            case 2200:
//		                HVDCLinkVoltage = 730;
//		                RectHWCurrentLimit = 124;
//		                InvHWCurrentLimit = 151;
//		                DCOVLevel = 420;
//		                break;
//		            case 2300:
//		                HVDCLinkVoltage = 730;
//		                RectHWCurrentLimit = 124;
//		                InvHWCurrentLimit = 155;
//		                DCOVLevel = 420;
//		                break;
//		            case 2400:
//		                HVDCLinkVoltage = 760;
//		                RectHWCurrentLimit = 124;
//		                InvHWCurrentLimit = 151;
//		                DCOVLevel = 440;
//		                break;
//		            default:
//		                break;
//		        }
//			}
//			else
//			{
//	
//			}
//	
//	        if ( EEStatusBits.bit.EEDataInitialized )
//	        {
//	        	EE_ID* ee = GetParameterEE( PARAM_Rectifer_DCLink_Set );
//	            PutEepData(ee->eep_addr, 1, &HVDCLinkVoltage, 0);
//	            
//	            ee = GetParameterEE( PARAM_Rectifier_Current_Limit_Set );
//	            PutEepData(ee->eep_addr, 1, &RectHWCurrentLimit, 0);
//	            
//	            ee = GetParameterEE( PARAM_Inverter_Current_Limit_Set );
//	            PutEepData(ee->eep_addr, 1, &InvHWCurrentLimit, 0);
//	
//	            ee = GetParameterEE( PARAM_AbsDCOVSet );
//	            PutEepData(ee->eep_addr, 1, &DCOVLevel, 0);
//	        }		
    }
}

// ***********************************************************************
// *
// *    FUNCTION: ValidInverterVoltage 
// *
// *    DESCRIPTION: Return if the requested inverter voltage is valid
// *
// *    ARGUMENTS: voltage: desired voltage in V*10.
// *
// *    RETURNS: 
// *
// ***********************************************************************
bool InverterControl::ValidInverterVoltage( uint16_t voltage )
{
    bool voltvalid = false;
    
    // check for valid voltage
    for ( uint16_t idx = 0; idx < NumberOfValidVoltages; idx++ )
    {
        if ( ValidInverterVoltages[ idx ] == voltage )
        {
            voltvalid = true;
        }
    }
    
    return voltvalid;
}

// ***********************************************************************
// *
// *    FUNCTION: On  
// *
// *    DESCRIPTION: turns on inverter
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::On( void )
{
	
//	    if ( ParallelCan.UPSIsParallel() )
	if ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() )
    {
    	LoadShareDroopTable.B0 = LoadShareDroopCoefficients_ptr->B0;
    }
    else
    {
        LoadShareDroopTable.B0 = 0;
    }
    
    if( EnableOpenLoop )
	{
		InverterVmag.phA = InitialNormVmag;
		InverterVmag.phB = InitialNormVmag;
		InverterVmag.phC = InitialNormVmag;

		InverterVRMSGainsA.X1= 0;	
		InverterVRMSGainsB.X1= 0;
		InverterVRMSGainsC.X1= 0;
    	InverterStatus.bit.InverterOpenLoop = 1;
	}
	else
	{
		InverterVmag.phA = InitialNormVmag;
		InverterVmag.phB = InitialNormVmag;
		InverterVmag.phC = InitialNormVmag;

		InverterVGainsA.X1 = ( 1.0f/InitialNormVmag - InverterVGainsA.B0) * InverterVoltageReference.abc.phA * InverterVSatFactorNew;
		InverterVGainsA.X2 = InverterVGainsA.X1;
		InverterVGainsB.X1 = ( 1.0f/InitialNormVmag - InverterVGainsB.B0) * InverterVoltageReference.abc.phB * InverterVSatFactorNew;
		InverterVGainsB.X2 = InverterVGainsB.X1;
		InverterVGainsC.X1 = ( 1.0f/InitialNormVmag - InverterVGainsC.B0) * InverterVoltageReference.abc.phC * InverterVSatFactorNew;
		InverterVGainsC.X2 = InverterVGainsC.X1;

		InverterVGainsA2.X1 = ( 1.0f/InitialNormVmag - InverterVGainsA2.B0) * InverterVoltageReference.abc.phA * InverterVSatFactorNew2;
		InverterVGainsA2.X2 = InverterVGainsA2.X1;
		InverterVGainsB2.X1 = ( 1.0f/InitialNormVmag - InverterVGainsB2.B0) * InverterVoltageReference.abc.phB * InverterVSatFactorNew2;
		InverterVGainsB2.X2 = InverterVGainsB2.X1;
		InverterVGainsC2.X1 = ( 1.0f/InitialNormVmag - InverterVGainsC2.B0) * InverterVoltageReference.abc.phC * InverterVSatFactorNew2;
		InverterVGainsC2.X2 = InverterVGainsC2.X1;

		InverterVRMSGainsA.X1= 0;	//koala rms loop from MCU;
		InverterVRMSGainsB.X1= 0;
		InverterVRMSGainsC.X1= 0;
		InverterStatus.bit.InverterOpenLoop = 0;
	}
    
    InverterPWMOn();
    InverterStatus.bit.DischargeCaps = 0;
    InverterStatus.bit.InverterOn = 1;
    InverterStatus.bit.ECTCurrentLoop = 0;
   // InverterStatus.bit.InverterOpenLoop = 0;
    InverterStatus.bit.VoltageRMSLoopOn = 1;
    ECTPFAngle = 0;
    NB_SetNodebit( UPM_NB_INVERTER_ON, 1 );
}
// ***********************************************************************
// *
// *    FUNCTION: OnFromESS
// *
// *    DESCRIPTION: turns on inverter ON Freom ESS state transient
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::OnFromESS( void )
{
    float NormalizedVoltageReferenceESS = 0.5 * ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered ) * VoltageNormFactor / InitialNormVmag;
    
    if ( ParallelCan.UPSIsParallel() )
    {
        LoadShareDroopTable.B0 = LoadShareDroopCoefficients_ptr->B0;
    }
    else
    {
        LoadShareDroopTable.B0 = 0;
    }
    
    InverterVmag.phA = InitialNormVmag;
    InverterVmag.phB = InitialNormVmag;
    InverterVmag.phC = InitialNormVmag;
    
    // To enhance stability, inverter output magnitude should match bypass.
    // Initial RMS magnitude follows output voltage (a little larger). Range: 91~109% nominal.
//	    InverterVmag.phA *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phA * RMSVoltageNormFactor * 1.05f ) );
//	    InverterVmag.phB *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phB * RMSVoltageNormFactor * 1.05f ) );
//	    InverterVmag.phC *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phC * RMSVoltageNormFactor * 1.05f ) );

    if( EnableOpenLoop )
	{
    	InverterStatus.bit.InverterOpenLoop = 1;
		
		InverterVmag.phA *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phA * RMSVoltageNormFactor * 1.0f ) );
		InverterVmag.phB *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phB * RMSVoltageNormFactor * 1.0f ) );
		InverterVmag.phC *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phC * RMSVoltageNormFactor * 1.0f ) );
	}
	else
	{
		InverterStatus.bit.InverterOpenLoop = 0;
		
		InverterVmag.phA *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phA * RMSVoltageNormFactor * 1.02f ) );
		InverterVmag.phB *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phB * RMSVoltageNormFactor * 1.02f ) );
		InverterVmag.phC *= std::min( 1.09f, std::max( 0.91f, ScreenMeters.OutputVoltageRMS.phC * RMSVoltageNormFactor * 1.02f ) );
	}

	
    // correct the phase
    if(!EnableOpenLoop )
	{    
	    SineRef.Angle -= SyncSource->GetOffset();
    }

    if ( SineRef.Angle > PI )
    {
        SineRef.Angle -= ( 2.0f * PI );
    }
    else if ( SineRef.Angle < -PI )
    {
        SineRef.Angle += ( 2.0f * PI );
    }
    
    // Optimize the controller initialization upon the controllers' gain and phase @ 50Hz
    // InverterVGains2: gain2, phase2; InverterVGains: gain1, phase1.
    float gain2, gain1, phase2, phase1;
        gain2 = 0.84333f;
        gain1 = 2.103778f;
        phase2 = 10.0f * DegreesToRadians;
        phase1 = 10.0f * DegreesToRadians;
    
    stThreePhase InvSineRef = {0, 0, 0};
    stThreePhase TempSineRef2 = {0, 0, 0};
    stThreePhase TempSineRef1 = {0, 0, 0};
    
    sin_3_phase(SineRef.Angle, &InvSineRef.phA, &InvSineRef.phB, &InvSineRef.phC);
    sin_3_phase(SineRef.Angle + phase2, &TempSineRef2.phA, &TempSineRef2.phB, &TempSineRef2.phC);
    sin_3_phase(SineRef.Angle + phase2 + phase1, &TempSineRef1.phA, &TempSineRef1.phB, &TempSineRef1.phC);
    
    float mag = 1.0f / (0.5f*( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered )) / VoltageNormFactor;
    
    InverterVGainsA2.X1 = (InvSineRef.phA - TempSineRef2.phA / gain2 * InverterVGainsA2.B0) * mag * InverterVSatFactorNew2;
    InverterVGainsB2.X1 = (InvSineRef.phB - TempSineRef2.phB / gain2 * InverterVGainsB2.B0) * mag * InverterVSatFactorNew2;
    InverterVGainsC2.X1 = (InvSineRef.phC - TempSineRef2.phC / gain2 * InverterVGainsC2.B0) * mag * InverterVSatFactorNew2;
    InverterVGainsA2.X2 = InverterVGainsA2.X1;
    InverterVGainsB2.X2 = InverterVGainsB2.X1;
    InverterVGainsC2.X2 = InverterVGainsC2.X1;
    
    InverterVGainsA.X1 = (TempSineRef2.phA / gain2 - TempSineRef1.phA / gain2 / gain1 * InverterVGainsA.B0) * mag * InverterVSatFactorNew;
    InverterVGainsB.X1 = (TempSineRef2.phB / gain2 - TempSineRef1.phB / gain2 / gain1 * InverterVGainsB.B0) * mag * InverterVSatFactorNew;
    InverterVGainsC.X1 = (TempSineRef2.phC / gain2 - TempSineRef1.phC / gain2 / gain1 * InverterVGainsC.B0) * mag * InverterVSatFactorNew;
    InverterVGainsA.X2 = InverterVGainsA.X1;
    InverterVGainsB.X2 = InverterVGainsB.X1;
    InverterVGainsC.X2 = InverterVGainsC.X1;
    
    InverterPWMOn();
    InverterStatus.bit.DischargeCaps = 0;
    InverterStatus.bit.InverterOn = 1;
    InverterStatus.bit.ECTCurrentLoop = 0;
//	    InverterStatus.bit.InverterOpenLoop = 0;
    InverterStatus.bit.VoltageRMSLoopOn = 1;
    ECTPFAngle = 0;
    InverterStatus.bit.InverterOnFromSuspend = 1;
    NB_SetNodebit( UPM_NB_INVERTER_ON, 1 );
}

// ***********************************************************************
// *
// *    FUNCTION: OnOpenLoop  
// *
// *    DESCRIPTION: turns on inverter open loop
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::OnOpenLoop( void )		//int:1.36(0.4)->1.76 
{
    InverterStatus.bit.VoltageRMSLoopOn = 1;	//1:enable RMS loop;  0:dis;  if dis,will high volt under bus high
//	    InverterVmag.phA = 0.96;
//	    InverterVmag.phB = 0.96;
//	    InverterVmag.phC = 0.96;
	InverterVmag.phA = InitialNormVmag;
	InverterVmag.phB = InitialNormVmag;
	InverterVmag.phC = InitialNormVmag;

    InverterVRMSGainsA.X1 = -0.1 * VRMSSatFactor;
    InverterVRMSGainsB.X1 = -0.1 * VRMSSatFactor;
    InverterVRMSGainsC.X1 = -0.1 * VRMSSatFactor;
    InverterPWMOn();
    InverterStatus.bit.DischargeCaps = 0;
    InverterStatus.bit.InverterOpenLoop = 1;
    InverterStatus.bit.ECTCurrentLoop = 0;
    InverterStatus.bit.InverterOn = 1;
    NB_SetNodebit( UPM_NB_INVERTER_ON, InverterStatus.bit.InverterOn );
}

// ***********************************************************************
// *
// *    FUNCTION: DischargeCaps
// *
// *    DESCRIPTION: turns on inverter and discharges the caps
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::DischargeCaps( void )
{
    On();
    InverterStatus.bit.DischargeCaps = 1;
}

// ***********************************************************************
// *
// *    FUNCTION: FCT  OnOpenLoop
// *
// *    DESCRIPTION: turns on Rectify open loop for FCT test
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::OnOpenLoop_FCT( void )
{
    //InverterStatus.bit.VoltageRMSLoopOn = 1;
    InverterVmag.phA = 0.85;
    InverterVmag.phB = 0.85;
    InverterVmag.phC = 0.85;
    InverterVRMSGainsA.X1 = -0.1 * VRMSSatFactor;
    InverterVRMSGainsB.X1 = -0.1 * VRMSSatFactor;
    InverterVRMSGainsC.X1 = -0.1 * VRMSSatFactor;
    RectifierPWMOn();
    InverterStatus.bit.InverterOpenLoop = 1;
    InverterStatus.bit.ECTCurrentLoop = 0; //Yanming add for ECT 20100919
    InverterStatus.bit.InverterOn = 1;
    NB_SetNodebit( UPM_NB_INVERTER_ON, InverterStatus.bit.InverterOn );
}

// ***********************************************************************
// *
// *    FUNCTION: Off  
// *
// *    DESCRIPTION: turns off inverter
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::Off( void )
{
    InverterPWMOff();
    InverterStatus.bit.InverterOn = 0;
    InverterStatus.bit.InverterOpenLoop = 0;
    InverterStatus.bit.VoltageRMSLoopOn = 0;
    InverterStatus.bit.ECTCurrentLoop = 0;
    InverterStatus.bit.DischargeCaps = 0;	
    InverterStatus.bit.BypassMatched = 0;
    
    InverterVGainsA.X1 = 0;
    InverterVGainsB.X1 = 0;
    InverterVGainsC.X1 = 0;
    InverterVGainsA.X2 = 0;
    InverterVGainsB.X2 = 0;
    InverterVGainsC.X2 = 0;
    
    InverterVGainsA2.X1 = 0;
    InverterVGainsB2.X1 = 0;
    InverterVGainsC2.X1 = 0;
    InverterVGainsA2.X2 = 0;
    InverterVGainsB2.X2 = 0;
    InverterVGainsC2.X2 = 0;
    
    InverterIGainsA.X1 = 0;
    InverterIGainsB.X1 = 0;
    InverterIGainsC.X1 = 0;
    
    InverterIGainsA2.X1 = 0;
    InverterIGainsB2.X1 = 0;
    InverterIGainsC2.X1 = 0;
	
    LoadShareDroopTable.B0 = 0;
    ECTPFAngle = 0;
    NB_SetNodebit( UPM_NB_INVERTER_ON, InverterStatus.bit.InverterOn );
}


// ***********************************************************************
// *
// *    FUNCTION: Off for FCT
// *
// *    DESCRIPTION: turns off Rectify for FCT
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::Off_FCT( void )
{
    RectifierPWMOff();
    InverterStatus.bit.InverterOn = 0;
    InverterStatus.bit.InverterOpenLoop = 0;
    InverterStatus.bit.VoltageRMSLoopOn = 0;
    InverterStatus.bit.BypassMatched = 0;

    InverterVGainsA.X1 = 0;
    InverterVGainsB.X1 = 0;
    InverterVGainsC.X1 = 0;
    InverterVGainsA.X2 = 0;
    InverterVGainsB.X2 = 0;
    InverterVGainsC.X2 = 0;
    
    InverterVGainsA2.X1 = 0;
    InverterVGainsB2.X1 = 0;
    InverterVGainsC2.X1 = 0;
    InverterVGainsA2.X2 = 0;
    InverterVGainsB2.X2 = 0;
    InverterVGainsC2.X2 = 0;
    
    InverterIGainsA.X1 = 0;
    InverterIGainsB.X1 = 0;
    InverterIGainsC.X1 = 0;
    
    InverterIGainsA2.X1 = 0;
    InverterIGainsB2.X1 = 0;
    InverterIGainsC2.X1 = 0;

    NB_SetNodebit( UPM_NB_INVERTER_ON, InverterStatus.bit.InverterOn );
}    

// ***********************************************************************
// *
// *    FUNCTION: SetHWCurrentLimit  
// *
// *    DESCRIPTION: turns off inverter
// *
// *    ARGUMENTS: currentLimit(amps)- the hardware current limit you want
// *               gain based on the ALLEGRO sensor
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::SetHWCurrentLimit( int16_t currentLimit )
{
    float tempDuty;
    float CurrentLimPWMPeriod = ( (float)CPUFrequency / PWMFrequency );
    //This is calculated by the 1.5V offset of the hardware cl circuit.  0.00333 is the
    // gain of the current sensor and 3V is the maximum the hw cl circuit will acceept.
    InvHWCurrentLimitReal = currentLimit;
    //Koala 15K: 0.007458699 = ((9.09+12.7)/9.09) * (12.1/(0.01+36.5+12.1)) *(1/1/80)
    //Wombat 10K: 0.014917398 = ((9.09+12.7)/9.09) * (12.1/(0.01+36.5+12.1)) *(1/1/160)
    //tempDuty = (1.5 + ( (float)currentLimit * 0.014917398f ))/3.0;

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
	
    ECap5Regs.CAP4 = (uint16_t)( tempDuty * CurrentLimPWMPeriod );
}

// ***********************************************************************
// *
// *    FUNCTION: InverterDCCompensation 
// *
// *    DESCRIPTION: DC Compensation function
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::InverterDCCompensation( void )	//inv zero, 20ms
{
    if ( InverterStatus.bit.InverterOn && !InverterStatus.bit.ECTCurrentLoop )
    {
        float offset;

		//InverterDCphA.SlowFiltered: pu-V,  0.5V
        offset = SecondOrderIIRFP( InverterDCphA.SlowFiltered, &InverterDCCompensatorA );
        if ( offset > MaxInverterDCOffset )		//5V
        {
            offset = MaxInverterDCOffset;
        }
        else
        {
            if ( offset < -MaxInverterDCOffset )
            {
                offset = -MaxInverterDCOffset;
            }
        }
        InverterDCOffset.phA = offset;

        offset = SecondOrderIIRFP( InverterDCphB.SlowFiltered, &InverterDCCompensatorB );
        if ( offset > MaxInverterDCOffset )
        {
            offset = MaxInverterDCOffset;
        }
        else
        {
            if ( offset < -MaxInverterDCOffset )
            {
                offset = -MaxInverterDCOffset;
            }
        }
        InverterDCOffset.phB = offset;

        offset = SecondOrderIIRFP( InverterDCphC.SlowFiltered, &InverterDCCompensatorC );
        if ( offset > MaxInverterDCOffset )
        {
            offset = MaxInverterDCOffset;
        }
        else
        {
            if ( offset < -MaxInverterDCOffset )
            {
                offset = -MaxInverterDCOffset;
            }
        }
        InverterDCOffset.phC = offset;

    }
}

// ***********************************************************************
// *
// *    FUNCTION: InverterLineCycleTask 
// *
// *    DESCRIPTION: Once per line cycle inverter tasks, RMS control and
// *                 dc compensation
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::InverterLineCycleTask( void )	//inv soft zero cross
{
    PLLPeriodicFunction();
    RunVoltageRMSLoop();
    InverterDCCompensation();		//delete sw_limit
    InverterShortDetect();
    UpdateECTCurrentRef();
}

// ***********************************************************************
// *
// *    FUNCTION: SetSWCurrentLimit 
// *
// *    DESCRIPTION: Set the software current limit, in amps.
// *
// *    ARGUMENTS: limit: the new software current limit, in amps.
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::SetSWCurrentLimit(float limit)
{
    CurrentNormFactor = 1.0f / limit;

    float ftemp = InverterCurrentLoopCoefficients_ptr->B0 * limit;
    InverterIGainsA.B0 = ftemp;
    InverterIGainsB.B0 = ftemp;
    InverterIGainsC.B0 = ftemp;

    float icaptemp = CapCurrentEstimatorCoefficients_ptr->B0 * NominalVoltage * SQRT_2 * CurrentNormFactor;

    IcapEstimatorGainsA.B0 = icaptemp;
    IcapEstimatorGainsB.B0 = icaptemp;
    IcapEstimatorGainsC.B0 = icaptemp;
}

// ***********************************************************************
// *
// *    FUNCTION: InverterShortDetect
// *
// *    DESCRIPTION: check inverter short
// *                 
// *                 
// *    Precondition: 
// *
// *    ARGUMENTS: None.
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::InverterShortDetect(void)
{
	float voltageThreshold = 50.0;  // 50.0v
    static uint16_t OutputShort = false;

    bool extraLowRMSVoltage = InverterStatus.bit.VoltageRMSLoopOn
        && (InverterVoltageRMS.RawRMS.phA < voltageThreshold
            || InverterVoltageRMS.RawRMS.phB < voltageThreshold
            || InverterVoltageRMS.RawRMS.phC < voltageThreshold);

    bool SoftwareCurrentLimited =  (InverterCurrentRMS.RawRMS.phA >ee_SlowSWCurrentLimit)
       		||(InverterCurrentRMS.RawRMS.phB >ee_SlowSWCurrentLimit)
       		||(InverterCurrentRMS.RawRMS.phC >ee_SlowSWCurrentLimit);
			
    if( extraLowRMSVoltage && SoftwareCurrentLimited)
    {   	
       		OutputShort = OutputShortDetectionTimer.CheckTimeout(ee_SlowSWCurrentLimitCycles);     	     	
    }
   	
    if (!NB_GetNodebit(UPM_NB_INVERTER_AC_UNDER_VOLTAGE) && !extraLowRMSVoltage)
    {
    	OutputShort = false;
	    OutputShortDetectionTimer.ClearTimer();
    }
	
	 NB_SetNodebit(UPM_NB_OUTPUT_SHORT_CIRCUIT, OutputShort);
}


// ***********************************************************************
// *
// *    FUNCTION: EEFunc_Inverter 
// *
// *    DESCRIPTION: inverter ee data function
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::EEFunc_Inverter( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_OutFrequency:
            {
                uint16_t eeFreq = *data;
                
                // only load if inverter is off
                if ( !InverterStatus.bit.InverterOn )
                {
                    switch ( eeFreq )
                    {
                        // 50 or 60 only, 60 = default
                        case 50:
                            SineRef.SetBaseFrequency( NOM_FREQUENCY_50HZ );
                            BaseSineRef.SetBaseFrequency( NOM_FREQUENCY_50HZ );
                            *((uint16_t*)ee->ram_ptr) = eeFreq;
							Rectifier.SetRectifierFilterLoopVFil(50);
                            break;
                            
                        case 60:
                        default:
                            SineRef.SetBaseFrequency( NOM_FREQUENCY_60HZ );
                            BaseSineRef.SetBaseFrequency( NOM_FREQUENCY_60HZ );
                            *((uint16_t*)ee->ram_ptr) = eeFreq;
							Rectifier.SetRectifierFilterLoopVFil(60);
                            break;
                    }
                }
            }
            break;
            
        case PARAM_InverterSlewRate:
            {
                uint16_t        slewRateHz_sec = *data;
            
                // check for valid slew rate, default to 1 Hz/sec if not valid
                switch ( slewRateHz_sec )
                {
                    case 3:
                        SlewRate = SLEW_RATE_3_HZ;
                        break;
                    case 5:
                        SlewRate = SLEW_RATE_5_HZ;
                        break;
                    case 7:
                        SlewRate = SLEW_RATE_7_HZ;
                        break;
                    case 1:
                    default:
                        SlewRate = SLEW_RATE_1_HZ;
                        break;            
                }
            
                // parallel units limited to low rate
                if ( EEP_IsInternalParallel() ||
                     ParallelCan.UPSIsParallel() )
                {
                    SlewRate = SLEW_RATE_PAR;
                } 
            
                // get slew rate limit
                SlewRateLimit = SlewRateLimitTable[ uint16_t( SlewRate ) ];
                FrequencyGain = SlewRateLimit.FrequencyGain;
                PhaseGain = SlewRateLimit.PhaseGain;
				InitSync(); //solve for JIRA 247
            }            
            break;
            
        case PARAM_InverterSWCurrentLim:
//	            ee_DefaultSWCurrentLimit = float(*data);
//	            SetSWCurrentLimit(ee_DefaultSWCurrentLimit); 
            break;
            
        case PARAM_Inverter_Current_Limit_Set:
            SetHWCurrentLimit( *data );
            InvHWCurrentLimit = *data;
            break;
        
        case PARAM_InverterUVSetLevel:
            InverterUVAlarmSetLevel = float(*data) / (float)1000.0;
            break;
            
        case PARAM_InverterUVClearLevel:
            InverterUVAlarmClearLevel = float(*data) / (float)1000.0;
            break;
        
        case PARAM_InverterShortCurrentLimit:
	            ee_SlowSWCurrentLimit = float(*data);
            break;
        
        case PARAM_InverterShortCurrentLimitCycles:
	            ee_SlowSWCurrentLimitCycles = float(*data);
            break;

        case PARAM_LoadShareVoltageCalA:
            ee_LoadShareVoltageCal.phA = float( int16_t( *data ) ) / 32767.0f;
            break;
            
        case PARAM_LoadShareVoltageCalB:
            ee_LoadShareVoltageCal.phB = float( int16_t( *data ) ) / 32767.0f;
            break;
            
        case PARAM_LoadShareVoltageCalC:
            ee_LoadShareVoltageCal.phC = float( int16_t( *data ) ) / 32767.0f;
            break;

		//neg power fault
        case PARAM_SelectiveTripDQLimit:
            SelectiveTripDQLimit = float(*data) * -0.01f;
            break;

        case PARAM_SelectiveTripVDiffLimit:
            SelectiveTripVDiffLimit = float( *data ) * -0.01f;
            break;

		case PARAM_InverterVoltLoopGain:
			InverterVoltLoopGain = (float)(*data) / 100.0f;
			break;

		case PARAM_InvEctCurrLoopGain:
			InvEctCurrLoopGain = (float)(*data) / 100.0f;
			break;

		case PARAM_InverterCurrLoopGain:
			InverterCurrLoopGain = (float)(*data) / 100.0f;
			break;

		case PARAM_InvDutyCompensation:
			InvDutyCompensation = (float)(*data) / 1000.0f;
			break;

		case PARAM_FeedForwardGain:
			FeedForwardGain = (float)(*data) / 100.0f;
			break;

		case PARAM_DelayCount:
			DelayCount = (float)(*data) / 100.0f;
			break;



//        case PARAM_EnableSelectiveTrip:
//            EnableSelectiveTrip = *data;
//            break;

        default:
            break;            
            
    }
}

// ***********************************************************************
// *
// *    FUNCTION: Receives master RMS data 
// *
// *    DESCRIPTION: Since the RMS command is range checked before use,
// *                 this data is not sanity checked
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::SetMasterRMSData( int16_t pA, int16_t pB, int16_t pC )
{
    MasterRMSData.phA = float( pA ) / 32767.0f;
    MasterRMSData.phB = float( pB ) / 32767.0f;
    MasterRMSData.phC = float( pC ) / 32767.0f;
}

// ***********************************************************************
// *
// *    FUNCTION: Receives SetSyncSource RMS data 
// *
// *    DESCRIPTION: Sets the sync source based on the sync state.
// *                 Phase gain is set to 0 in free run mode so that
// *                 PARA_SYNC delays can be auto-calibrated out.
// *
// *    ARGUMENTS: sync state
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void InverterControl::SetSyncSource( sync_states_t state )
{
    // Disable PLL while changing sync source
    CriticalSection enter( IER_DMA_ONLY );

    switch ( state )
    {
        case SYNC_STATE_BASE_FREE_RUN:
            PhaseGain = 0.0f;
            SyncSource = &BaseSineRef;
            break;

        case SYNC_STATE_BYPASS:
            PhaseGain = SlewRateLimit.PhaseGain;
            SyncSource = &BypassPLL.SineRef; 
            break;

        case SYNC_STATE_INPUT:
            PhaseGain = SlewRateLimit.PhaseGain;
            SyncSource = &Rectifier.UtilityPLL.SineRef;
            break;

        case SYNC_STATE_OUTPUT:
            PhaseGain = SlewRateLimit.PhaseGain;
            SyncSource = &OutputPLL.SineRef;
            break;

        case SYNC_STATE_BASE_EXT_SYNC:
            PhaseGain = SlewRateLimit.PhaseGain;
//	            SyncSource = &ExtSignalSync.SineRef;
            break;

        default:
            break;
    }
}
// ***********************************************************************
// *
// *    FUNCTION: OnCurrentLoop 
// *
// *    DESCRIPTION: In ECT mode, turn off RMS and voltage loop, turn on 
// *                 current loop and PWM signals and set nodebit.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::OnECTCurrentLoop( void )
{   
    ECTInverterIGainsA = *ECTCurrentLoopCoefficients_ptr;
    ECTInverterIGainsA.B0 = ECTInverterIGainsA.B0 / CurrentNormFactor;
    ECTInverterIGainsB = ECTInverterIGainsA;
    ECTInverterIGainsC = ECTInverterIGainsA;
    InverterECTCurrentMag = 0; 
    InverterStatus.bit.VoltageRMSLoopOn = 0;
    InverterPWMOn();
    InverterStatus.bit.ECTCurrentLoop = 1;
    InverterStatus.bit.InverterOpenLoop = 0;
    InverterStatus.bit.InverterOn = 1;    
    ECTPFAngle = ECTPFAngleTemp;
    NB_SetNodebit( UPM_NB_INVERTER_ON, InverterStatus.bit.InverterOn );
}

// ***********************************************************************
// *
// *    FUNCTION: UpdateECTCurrentRef
// *
// *    DESCRIPTION: In ECT mode, to update current reference peak value.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void InverterControl::UpdateECTCurrentRef( void )	// 20ms
{
    static uint16_t unbalancepowerCnt = 0;
    float tempA,tempB,tempC,temp;
    float power;
	static uint16_t wEctDelayCnt = 0; 

    if(InverterStatus.bit.ECTCurrentLoop)
    {        
    	float OutputPF = (float)OutputPowerFactorRating / 10000;
        tempA = LoadPower.TotalPower.phA / OutputPF * RMSCurrentNormFactor * RMSVoltageNormFactor;
        tempB = LoadPower.TotalPower.phB / OutputPF * RMSCurrentNormFactor * RMSVoltageNormFactor;
        tempC = LoadPower.TotalPower.phC / OutputPF * RMSCurrentNormFactor * RMSVoltageNormFactor;

        temp = ( tempA + tempB +tempC )/3; 

        //Check unbalance output nominal power
        if( temp > 0.25 )
        {
            if( ( fabs( tempA - tempB ) > 0.2 ) ||
                ( fabs( tempB - tempC ) > 0.2 ) ||
                ( fabs( tempC - tempA ) > 0.2 ) )
            {
                unbalancepowerCnt++;
                if( unbalancepowerCnt == 3 )
                {
                    MCUStateMachine.ECTAbandonReason |= 0x0002;
                }
            }
            else
            {
                unbalancepowerCnt = 0;
            }
        }
        else
        {
            unbalancepowerCnt = 0;
        }

        float normTargetPower = ECTTotalPowerSet;//MCUStateMachine.ECTPowerSet;//
        float totalPower = LoadPower.TotalPower.phA + LoadPower.TotalPower.phB + LoadPower.TotalPower.phC;
        // Use kW's instead of kVA's at lower power levels.
        if ( ( normTargetPower < 0.55f ) &&       // Target power < 55%.
//	             ( totalPower <  7500) ) // ECT75PercentPower Actual power < 75%. Should always be true, safety check.
			 ( totalPower <   (float(OutputkVARating)*75.0f)) ) // ECT75PercentPower Actual power < 75%. Should always be true, safety check.
        {
            totalPower = LoadPower.ActivePower.phA + LoadPower.ActivePower.phB + LoadPower.ActivePower.phC;
            normTargetPower = ECTActivePowerSet;
        }

		float outputVARating = float(OutputkVARating) * 100.0f;
		RMSPowerNormFactor = 1.0f / outputVARating;

        float normTotalPower = totalPower * RMSPowerNormFactor;

		float power = 0.3f + FirstOrderIIRFP( normTargetPower - normTotalPower, &ECTPowerGains );

        if( power > 1.3f )
        {
            power = 1.3f;
            ECTPowerGains.X1 = 1.3f;
        }
        else if( power <= 0 )
        {
            power = 0;
            ECTPowerGains.X1 = 0.0f;
        }      

        tempA = InverterVoltageRMS.RawRMS.phA * RMSVoltageNormFactor;
        tempB = InverterVoltageRMS.RawRMS.phB * RMSVoltageNormFactor;
        tempC = InverterVoltageRMS.RawRMS.phC * RMSVoltageNormFactor;

        temp = ( tempA > tempB ) ? tempA : tempB;
        temp = ( temp  > tempC ) ? temp  : tempC;
        temp = temp * RMSCurrentNormFactor;

        if( temp != 0 )
        {
	        float ectCurrentLim = 0.0f;
	        	
			if(UPMSystem == HV_20K)
			{
				ectCurrentLim = 0.27f;	// =20000/3/230*1.414*1.2/182				
			}
			else if(UPMSystem == HV_40K)
			{
				ectCurrentLim = 0.54f;	// =40000/3/230*1.414*1.2/182				
			}
			else
			{
				ectCurrentLim = 0.54f;	// =40000/3/230*1.414*1.2/182				
			}
			
            InverterECTCurrentMag = power * CurrentNormFactor * SQRT_2 /temp;//CurrentNormFactor=1/182

			if( InverterECTCurrentMag >= ectCurrentLim)
	    	{
	    		InverterECTCurrentMag = ectCurrentLim;
	    	}
        }
        else
        {
            InverterECTCurrentMag = 0.05f;
        }


		if( ECTTotalPowerSet > 0.89f )	//
		{
			float pfOffset = 0.0f;
			
			if(wEctDelayCnt++ >= 25)		//25=500ms/20ms		//2s stable
			{
				wEctDelayCnt = 25;
				
				if(OutputkVARating == 300)	//30k: 5.2a
				{
					pfOffset = -0.63; 	//-0.31 -> -0.51
				}
				else	//20/40k		//40k:5.6a;    20k: 2.8a
				{
					pfOffset = -0.51; //40k 	//-0.31 -> -0.51: 40k
				}				
			}
			else
			{			
				pfOffset = -0.31f;
			}		

		    float pf = 1.00f;
			uint16_t powerfactor = (uint16_t )ECTPowerFactor;
			
		    if( powerfactor >= 100 )
		    {
		        pf = float(200-powerfactor) / 100.0f;
		        ECTPFAngleTemp = - acos( pf ) + pfOffset;
		    }
		    else
		    {
		        pf = float(powerfactor) / 100.0f;
		        ECTPFAngleTemp = acos( pf ) + pfOffset;
		    }
		    ECTActivePowerSet = ECTTotalPowerSet * pf;			

			ECTPFAngle = ECTPFAngleTemp;
		}
		else
		{
			wEctDelayCnt = 0;			
		}				
    }
    else
    {
        ECTPowerGains.X1 = 0.0f;
        InverterECTCurrentMag = 0.0f;
        ECTInverterIGainsA.X1 = 0.0f;
        ECTInverterIGainsB.X1 = 0.0f;
        ECTInverterIGainsC.X1 = 0.0f;
		wEctDelayCnt = 0;
    } 

}

// ***********************************************************************
// *
// *    FUNCTION:  void SetECTPowerfactor( int16_t powerfactor)
// *
// *    DESCRIPTION: Set power factor in ECT mode, 0.06 is about inverter
// *                 capacitor angle.
// *
// ***********************************************************************
void InverterControl::SetECTPowerfactor( uint16_t powerfactor )
{
    if( powerfactor >= 100 )
    {
        ECTPFAngleTemp = - acos(((float)(200-powerfactor))/100) - 0.3;//0.2;//0.06;
    }
    else
    {
        ECTPFAngleTemp = acos((float)powerfactor/100) - 0.3;//0.2;//0.06;
    }
}

// ***********************************************************************
// *
// *    FUNCTION:  SetECTPowerAndPf
// *
// *    ARGUMENTS:  Power: 2000= 20%, 10000 = 100% of nominal power
// *                Power factor: 70 -> PF=0.70, 130 -> PF=-0.70
// *
// *    DESCRIPTION: Set power and power factor in ECT mode.
// *
// ***********************************************************************
void InverterControl::SetECTPowerAndPf( uint16_t power, uint16_t powerfactor )
{
    float pfOffset;

    if( power > 10000)
    {
        power -= 10000;
        //UPMStateMachine.ECTRestartEnable = true; //>10000 means enable ECT restart
    }
    else
    {
        //UPMStateMachine.ECTRestartEnable = false;
    }
    ECTTotalPowerSet = float(power)/10000.0f;
    ECTPowerFactor = powerfactor;
    // ECTTotalPowerSet = MCUStateMachine.ECTPowerSet;
    // PF correction

	if(OutputkVARating == 300)
	{
		float deratetemp = ECTTotalPowerSet*0.75;
		
		// pfOffset is a compensation of power factor. In order output pf=1 when pf_set=100:
		// POWER	 0.1	0.2    0.3	 0.4   0.5		0.6 	 0.7	   0.8		 >=0.9
		// pfOffset  -1.23	-1.04  -0.94 -0.8  -0.7027	-0.5838  -0.5003   -0.4415	 -0.31
		if( deratetemp < 0.20f )
		{
			pfOffset = 1.90f * deratetemp - 1.42f;	// linear curve fitting	
		}
		else if( deratetemp < 0.40f )
		{
			pfOffset = 1.20f * deratetemp - 1.28f;	// linear curve fitting 					
		}
		else if( deratetemp > 0.89f )
		{
			pfOffset = -0.31f;	//
		}
		else		
		{
			pfOffset = deratetemp - 1.20f;		//0.75*0.9=0.67  (0.67-1.2=0.5
				
		}
	}
	else
	{
		// pfOffset is a compensation of power factor. In order output pf=1 when pf_set=100:
		// POWER	 0.1	0.2    0.3	 0.4   0.5		0.6 	 0.7	   0.8		 >=0.9
		// pfOffset  -1.23	-1.04  -0.94 -0.8  -0.7027	-0.5838  -0.5003   -0.4415	 -0.31
		if( ECTTotalPowerSet < 0.20f )
		{
			pfOffset = 1.90f * ECTTotalPowerSet - 1.42f;	// linear curve fitting 
		}
		else if( ECTTotalPowerSet < 0.40f )
		{
			pfOffset = 1.20f * ECTTotalPowerSet - 1.28f;	// linear curve fitting 					
		}
		else if( ECTTotalPowerSet > 0.89f )
		{
			pfOffset = -0.31f;	//
		}
		else
		{
			pfOffset = ECTTotalPowerSet - 1.20f;
				
		}
	}
	
    float pf = 1.00f;
    if( powerfactor >= 100 )
    {
        pf = float(200-powerfactor) / 100.0f;
        ECTPFAngleTemp = - acos( pf ) + pfOffset;
    }
    else
    {
        pf = float(powerfactor) / 100.0f;
        ECTPFAngleTemp = acos( pf ) + pfOffset;
    }
    ECTActivePowerSet = ECTTotalPowerSet * pf;
}

// ********************************************************************
// *
// * Function:  ParallelSystemSelectiveTrip()
// *
// * Purpose:  Fast detection of inverter failure on parallel bus
// *
// * Parms Passed:  None
// * Returns:   Nothing
// *       
// ********************************************************************
//	#pragma CODE_SECTION( "ramfuncs" )
void InverterControl::ParallelSystemSelectiveTrip(void)	//call by RunPLLFast 1.125k
{

//		float diffA = fabs( SelectiveTripThreePhase.phA ) - fabs( InverterSineRef.phA );
//		float diffB = fabs( SelectiveTripThreePhase.phB ) - fabs( InverterSineRef.phB );
//		float diffC = fabs( SelectiveTripThreePhase.phC ) - fabs( InverterSineRef.phC );
	float diffA;
	float diffB;
	float diffC;

	diffA = fabs( SelectiveTripThreePhase.phA ) - fabs( InverterSineRef.phA );
	diffB = fabs( SelectiveTripThreePhase.phB ) - fabs( InverterSineRef.phB );
	diffC = fabs( SelectiveTripThreePhase.phC ) - fabs( InverterSineRef.phC );		
		
	//Negpower fault protect frollow 9P
	if ((ONLINE_STATE == MCUStateMachine.GetState())						 && 
		(ParallelCan.TotalNumberOfUpmsOnline > 1)							   && 
		(MCUStateMachine.GetSelectiveTripReadyFlag())						   &&
		( InverterCurrentDQ0Data.Sd < SelectiveTripDQLimit )				   &&
		(!NB_GetNodebit( UPM_NB_OUTPUT_AC_UNDER_VOLTAGE ) ) 				   &&  
		(( diffA < SelectiveTripVDiffLimit ) ||
         ( diffB < SelectiveTripVDiffLimit ) ||
         ( diffC < SelectiveTripVDiffLimit )))
	{
		NB_DebounceAndQue(UPM_NB_SELECTIVE_TRIP_OF_MODULE, true);
	}
	else if ( !NB_GetNodebit( UPM_NB_SELECTIVE_TRIP_OF_MODULE ) )
	{
		NB_DebounceAndQue(UPM_NB_SELECTIVE_TRIP_OF_MODULE, false);
	}
	else
	{
		//nodebit is active, can not be cleared here
	}
}


// ***********************************************************************
// *
// *    FUNCTION:  SetInv3LvlPwm
// *
// *    DESCRIPTION: Converts pwm pulse ratio value to 2 signals for 3-level switch.
// *    maps gpio pin and pwm to phase
// *    pw is in range -1..1 where 0= 0v out, 1=dc+ , -1 = dc- . this is converted to 3-level pwm signals a,b
// *    in 2-level switch pw is directly the pwm pulse ratio.
// *    a= magnitude, b= polarity. 1=+, 0=-
//
// *    Note that compiler reduces this function to few assembler lines, as "phase" is constant. Use optimizer level=2.
// ***************************************************************************
// *    ARGUMENTS:  phase=number indicates which leg
// *
// *    RETURNS:
// *
// ***********************************************************************
inline void SetInv3LvlPwm(float pw, float period, const uint16_t phase)
{
    uint16_t t;

    if ( pw > 0 )
    {
        t = uint16_t( pw * period );

        switch ( phase)
        {
            case 1: PIN_INV_R_POL_to1(); break;
            case 2: PIN_INV_S_POL_to1(); break;
            case 3: PIN_INV_T_POL_to1(); break;
            default: break;
        }
    }
    else
    {
        t = uint16_t( -pw * period );

        switch ( phase)
        {
            case 1: PIN_INV_R_POL_to0(); break;
            case 2: PIN_INV_S_POL_to0(); break;
            case 3: PIN_INV_T_POL_to0(); break;
            default: break;
        }
    }
    switch( phase)
    {
        case 1: EPwm1Regs.CMPA.half.CMPA = t ;break;
        case 2: EPwm5Regs.CMPA.half.CMPA = t ;break;
        case 3: EPwm5Regs.CMPB = t ;break;
        default: break;
    }
}

// ********************************************************************
// *
// * Function:  LoadshareThetaCompen()
// *
// * Purpose: 
	//JIRA loadshare(line &bat) issue; 
	//when 1.PLL_Byp & 2.line mode(or powershare mode)(online mode& rec on) 
// *
// * Parms Passed:  None
// * Returns:   Nothing
// *       
// ********************************************************************
void LoadshareThetaCompen(void)		//5ms task
{	
	//JIRA loadshare(line &bat) issue; 
	//when 1.PLL_Byp & 2.line mode(or powershare mode):(online mode& rec on) 
	if(Sync_State == SYNC_STATE_BYPASS) 				
	{
//			if((MCUStateMachine.GetState() == ONLINE_STATE)
//	//				&& (!BatteryConverter.BatteryStatus.bit.OnBatteryStatus))	//line mode no powershare
//				&& ( Rectifier.GetStatus().bit.RectifierOnNormal )) //Line mode or Powershare() mode
	//for 11 jira, paral line to bat mode;
	//compen4,fast the phase compen, as original a little slow
		if((MCUStateMachine.GetState() == ONLINE_STATE)
			&& ((Rectifier.GetState()==RECTIFIER_NORMAL_STATE) || (Rectifier.GetState()==RECTIFIER_WALKIN_STATE))) //Line mode or Powershare() mode
		{
			if((NB_GetNodebit(UPM_NB_BYPASS_AC_OVER_VOLTAGE))
				&& (!NB_GetNodebit(UPM_NB_BATTERIES_DISCONNECTED)) )	//Byp UV/OV && Bat connect
			{
//						//0.0085451827=280/32767: about 25us(deg:0.0085451827/6.28318*360=0.489
//						//add for jira 372/375/376:-0.006408886989959=-210/32767 (about 0.367deg)
//						BypassPLL.SineRef.LoadSharePhaseCal = 0.0085451827 + (-0.006408886989959) 
//												+0.0015259254738 + LoadSharePhaseCalByp;

				//for regression v1.3.24(dvt folder) unit3:3, when 251v, U1 byp avail;U2 byp ov, may cause loadshare fail
				BypassPLL.SineRef.LoadSharePhaseCal = 0.0085451827 + LoadSharePhaseCalByp;	
				
			}
			else
			{
				//Wombat33 for Byp UV not need compen
				//0.0085451827=280/32767: about 25us(deg:0.0085451827/6.28318*360=0.489
				//					BypassPLL.SineRef.LoadSharePhaseCal = 0.0085451827; 	
				BypassPLL.SineRef.LoadSharePhaseCal = 0.0085451827 + LoadSharePhaseCalByp;				
			}
		}
		else
		{
			BypassPLL.SineRef.LoadSharePhaseCal = LoadSharePhaseCalByp;
		}
	}
	else
	{		
		BypassPLL.SineRef.LoadSharePhaseCal = LoadSharePhaseCalByp;
	}
}

// ***********************************************************************
// *
// *    FUNCTION: StartFixedDuty
// *
// *    DESCRIPTION: Loads fixed duty value to PWM CMPR and enables gates
// *
// *    ARGUMENTS: int16_t duty cycle 0 - 100
// *
// *    RETURNS:
// *
// ***********************************************************************
void InverterControl::StartFixedDuty( int16_t duty )
{
    if ( !InverterStatus.bit.InverterOn &&
         ( duty <= 100 ) )
    {
        float temp = (float)(duty-50)  / (float)50; //convert to range -1...+1

        PWMCommand.phA = temp;
		PWMCommand.phB = temp;
		PWMCommand.phC = temp;
    }
}

// ******************************************************************************************************
// *            End of Inverter.c
// ******************************************************************************************************
