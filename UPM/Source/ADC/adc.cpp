// ******************************************************************************************************
// *            adc.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: adc.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/19/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "adc.h"
#include "F28335Port.h"
#include "InverterControl.h"
#include "RectifierStateControl.h"
#include "Algos.h"
#include "Eeprom_map.h"
#include "debugger.h"
#include "BatteryConverterControl.h"
#include "CriticalSection.h"
#include "InvSync.h"
#include "ExtSignalPLL.h"
#include "MCUState.h"
#include "PeriodicTSKs.h"
#include "BatteryStateControl.h"
#include <math.h>
#include <clk.h>
#include "Version.h"
#include "BypassInterface.h"

extern "C" {
    void adc_isr( void );
    void AdcDMAConfigure( void );
}

inline void WaveformCapture( void );
inline void TestESSTransferTime(void);
//
// File data
//
const uint16_t NumberOfAdcFastChannels    = 24;               //
uint16_t ADMuxState = 0;                                      // state variable for A/D mux

uint16_t EssTimeCount = 0;
float EssTransferTime = 0;
bool EssTimeTest = false;
bool record_uv_time = false;

//const stSecondOrderIIRFP SdFilterCoefficients         = { 0.3811,   1, 0, -0.2378, 0,  0,  0 };
stSecondOrderIIRFP SdFilter_40KHz         = { 0.03706,   1, 0, -0.9259, 0,  0,  0 };
stPark             OutputVoltageDQ;

uAdcCalibration               CalibrationScaler;            // calibration table, local

// this one's volatile, even to us
volatile int16_t AdcDMABuffer[2][32];

uRawAdcData RawAdcData;                   // all raw adc data
volatile const uRawAdcData * const RawAdcDataPtr = &RawAdcData;

uAdcSlowDataRaw      SlowAdcData;                      // slow channels processed @5.1kHz ProcessADC_HWI
volatile const uAdcSlowDataRaw * const SlowAdcDataPtr = &SlowAdcData;

// zero ref data
typedef struct
{
    int16_t   phA;
    int16_t   phB;
    int16_t   phC;
} stACCurrentAutoZero;        

int16_t AnalogZeroRaw = 0;	  // referenced to OpAref signal
int16_t AnalogZeroDouble = 0; // referenced to OpAref signal * 2
int16_t AnalogZero = 0;       // referenced to OpAref signal
int16_t AnalogZeroRawDouble = 0;// referenced to OpAref signal*2

// referenced to OpAref signal, B7_0
int16_t NullData = 0;                                         // place holder
stACCurrentAutoZero InputCurrentAutoZero    = { 4096, 4096, 4096 };      // will need autozeroing of current sensors someday, setting to AD range for now
stACCurrentAutoZero InverterCurrentAutoZero = { 4096, 4096, 4096 };
stACCurrentAutoZero BypassCurrentAutoZero   = { 4096, 4096, 4096 };
int16_t BattCurrentPosAutoZero = 4096;
int16_t BattCurrentNegAutoZero = 4096;
float InvVoltScalerPhaseA = 1.0f;
float InvVoltScalerPhaseB = 1.0f;
float InvVoltScalerPhaseC = 1.0f;
extern uint16_t NewCtrlBoard;

//run time counter
//static unsigned long CLK_EnterISRCnt = 0;
//static unsigned long CLK_ExitISRCnt = 0;

#pragma DATA_SECTION( "ramdata" );
int16_t const * const FastAdcZero[] =
{
    &BattCurrentNegAutoZero,        // Ibat2,   A3_1
    &NullData,                      // REF1.5,  A4_1
    &NullData,                      // 12V,     A5_1
    &BypassCurrentAutoZero.phA,     // IoutA,   B3_1
    &BypassCurrentAutoZero.phB,     // IoutB,   B4_1
    &BypassCurrentAutoZero.phC,     // IoutC,   B5_1
    &AnalogZeroDouble,              // Vbat,    A0_1
    &AnalogZeroDouble,              // Vbat1,   A1_1
    &BattCurrentPosAutoZero,        // Ibat1,   A2_1
    &AnalogZeroRawDouble,           // VbypA,   B0_1,
    &AnalogZeroRawDouble,           // VbypB,   B1_1,
    &AnalogZeroRawDouble,           // VbypC,   B2_1,
    
    &InputCurrentAutoZero.phA,      // IinA,    A3_0
    &InputCurrentAutoZero.phB,      // IinB,    A4_0
    &InputCurrentAutoZero.phC,      // IinC,    A5_0
    &InverterCurrentAutoZero.phA,   // IinvA,   B3_0
    &InverterCurrentAutoZero.phB,   // IinvB,   B4_0
    &InverterCurrentAutoZero.phC,   // IinvC,   B5_0
    &AnalogZeroRawDouble,           // VinA,    A0_0
    &AnalogZeroRawDouble,           // VinB,    A1_0
    &AnalogZeroRawDouble,           // VinC,    A2_0
    &AnalogZeroRawDouble,           // VinvA,   B0_0
    &AnalogZeroRawDouble,           // VinvB,   B1_0
    &AnalogZeroRawDouble,           // VinvC,   B2_0
};

#pragma DATA_SECTION( "ramdata" );
int16_t const * const SlowAdcZero[] =
{
    &AnalogZero,                    // VinN,    B7_0 
    &AnalogZero,                    // InvdcA,  A7_0
    &NullData,                      // Rtemp,   B7_1
    &AnalogZero,                    // InvdcB,  A7_1
    &NullData,                      // Itemp,   B7_2
    &AnalogZero,                    // InvdcC,  A7_2
    &NullData,                      // Btemp,   B7_3
    &AnalogZeroRaw,                 // VoutA,   A7_3
    &NullData,                      // STStemp, B7_4
    &AnalogZeroRaw,                 // VoutB,   A7_4
    &NullData,                      // +15V     B7_5
    &AnalogZeroRaw,                 // VoutC,   A7_5
//	    &AnalogZeroRaw,                 // Vbat+,   B7_6
    &NullData,                      // +24V,   B7_6
    &AnalogZeroRaw,                 // Vbat,   A7_6
    &AnalogZeroRaw,                 // ChassisVoltage,B7_7
//	    &NullData                       // Ref1,    A7_7      
    &AnalogZeroRaw,                 // Vbat+,   A7_6
};

#pragma DATA_SECTION( "ramdata" );
int16_t * const SlowAdcResultRaw[] =
{
    &SlowAdcData.st.LogicPower24v,
    &SlowAdcData.st.InputVoltageNeutral,
    &SlowAdcData.st.PhaseATemperature,
    &SlowAdcData.st.PhaseCTemperature
};

#pragma DATA_SECTION( "ramdata" );
volatile const int16_t * const SlowAdcDataRaw[4][4] =
{
    {
        &AdcDMABuffer[1][SLOW_LOGIC_24V_IDX],
        &AdcDMABuffer[1][SLOW_VBATT_IDX],
        &AdcDMABuffer[1][SLOW_CHASSIS_VOLTS_IDX],
        &AdcDMABuffer[1][SLOW_VBATT_POS_IDX]
    },
    {
        &AdcDMABuffer[0][SLOW_UTIL_VOLTS_N_IDX],
        &AdcDMABuffer[0][SLOW_INV_DC_PHA_IDX],
        &AdcDMABuffer[0][SLOW_RECT_TEMP_IDX],
        &AdcDMABuffer[0][SLOW_INV_DC_PHB_IDX]
    },
    {
        &AdcDMABuffer[1][SLOW_INV_TEMP_IDX],
        &AdcDMABuffer[1][SLOW_INV_DC_PHC_IDX],    
        &AdcDMABuffer[1][SLOW_BAT_TEMP_IDX],
        &AdcDMABuffer[1][SLOW_OUT_V_PHA_IDX]
    },
    {
        &AdcDMABuffer[0][SLOW_STS_TEMP_IDX],
        &AdcDMABuffer[0][SLOW_OUT_V_PHB_IDX],
        &AdcDMABuffer[0][SLOW_LOGIC_15V_IDX],
        &AdcDMABuffer[0][SLOW_OUT_V_PHC_IDX]
    }
};

#pragma DATA_SECTION( "ramdata" );
float * const MedAdcResult[] =
{
    &RawAdcData.st.AMBTemperature,
    &RawAdcData.st.RailVoltageNegative
};

#pragma DATA_SECTION( "ramdata" );
float * const MedAdcCal[] =
{
//	    &CalibrationScaler.st.BypassNeutralCal,
	&CalibrationScaler.st.AMBTemperatureCal,
    &CalibrationScaler.st.RailVoltageNegativeCal
};

#pragma DATA_SECTION( "ramdata" );
volatile const int16_t * const MedAdcData[2][4] =
{
    {
        &AdcDMABuffer[1][MEDIUM_AMB_TEMP_IDX],
        &AdcDMABuffer[1][MEDIUM_BYPASS_V_B_IDX],
        &AdcDMABuffer[1][MEDIUM_SCR_TEMP_IDX],  //MEDIUM_RECTCAP_TEMP_IDX
        &AdcDMABuffer[1][MEDIUM_BYPASS_V_C_IDX]
    },
    {
        &AdcDMABuffer[0][MEDIUM_RAIL_NEGATIVE_IDX],
        &AdcDMABuffer[0][MEDIUM_RAIL_POSITIVE_IDX],
        &AdcDMABuffer[0][MEDIUM_INVCAP_TEMP_IDX],
        &AdcDMABuffer[0][MEDIUM_BYPASS_V_A_IDX]
    }
};

#pragma DATA_SECTION( "ramdata" );
int16_t * const MedAdcZero[2][4] =
{
    {
        &NullData,		//AMB_TEMP
        &AnalogZero,	//BYP_V_B
        &NullData,		//SCR_TEMP
        &AnalogZero		//BYP_V_C
    },
    {
        &AnalogZero,	//RAIL_NEGATIVE
        &AnalogZero,	//RAIL_POSITIVE
        &NullData,		//INV_CAP_TEMP
        &AnalogZero		//BYP_V_A
    }
};

// Zhangjun : the following code is for FCT purpose and should be integrated
typedef struct
{
    // 'medium' A/D Mux group 0 
    int16_t         RailVoltageNegative;            // A6
    int16_t         RailVoltagePositive;            // B6
    // 'medium' A/D Mux group 1
    int16_t         InvCapTemperature;              // A6
    int16_t         BypassVoltage2phA;              // B6
    // 'medium' A/D Mux group 2
    int16_t         AMBTemperature;                 // A6
    int16_t         BypassVoltage2phB;              // B6
    // 'medium' A/D Mux group 3
    int16_t         SCRTemperature;                  // A6    RecCapTemperature
    int16_t         BypassVoltage2phC;              // B6
    
    // slow A/D 0  
    int16_t         InputVoltageNeutral;            // B7
    int16_t         InverterDCPhaseA;               // A7
    // slow A/D 1
    int16_t         PhaseBTemperature;              //          RectifierTemperature;          // B7
    int16_t         InverterDCPhaseB;               // A7
    // slow A/D 2
    int16_t         PhaseATemperature;          // B7
    int16_t         InverterDCPhaseC;               // A7
    // slow A/D 3
    int16_t         BatteryTemperature;           // B7
    int16_t         OutputVoltagePhaseA;            // A7
    // slow A/D 4
    int16_t         PhaseCTemperature;                 // B7
    int16_t         OutputVoltagePhaseB;            // A7
    // slow A/D 5
    int16_t         LogicPower15v;                  // B7
    int16_t         OutputVoltagePhaseC;            // A7
    // slow A/D 6
    int16_t         LogicPower24v;         			// B7
    int16_t         BatteryVoltage;         // A7
    // slow A/D 7
    int16_t         ChassisVoltage;                // B7
    int16_t         CalibrationRef1;                // A7    
}stADC_STRUCT;

typedef union
{
    stADC_STRUCT st;
    int16_t words[sizeof(stADC_STRUCT)];
}unADC_UNION;

unADC_UNION Adc_Sample;

#pragma DATA_SECTION( "ramdata" );
int16_t * const SlowAdcSample[] =
{
    &Adc_Sample.st.LogicPower24v,
    &Adc_Sample.st.InputVoltageNeutral,
    &Adc_Sample.st.PhaseATemperature,
    &Adc_Sample.st.PhaseCTemperature
};

#pragma DATA_SECTION( "ramdata" );
int16_t * const MedAdcSample[] =
{
    &Adc_Sample.st.AMBTemperature,
    &Adc_Sample.st.RailVoltageNegative
};
// Zhangjun : the above code is for FCT purpose and should be integrated

//
// Global data
// 
stThreePhase OutputCurrent = {0, 0, 0};
stThreePhase InputCurrentWithCap = {0, 0, 0};


const uAdcCalibration UnitOfMeasureConversions_HV_20K =
//	{	//HW-Ver01(before 2021.4.7): 20K
//	    // 12-bit A/D, 0-3V = 3/(2^12) = 0.000732422 V/bit. Divide by circuit gain
//	    // ex. 30k LV input voltage is inverting op-amp, R2 = 5.1k, R1 = 3*330k + 18k, g = -R2/R1 = -0.005059524
//	    // so scaler =  0.000732422 / -0.005059524 = -0.144761029. Since fast channels are 2x oversampled
//	    // need to divide by 2 so final conversion = -0.072380515. Inverter voltage is, of course, different.
//		 //2x   
//	     0.04915506,        // UnitOfMeasureConversions.BatteryCurrentNegativeConversion
//	     0.000366211,	     // UnitOfMeasureConversions.OpARef
//	     0.0009137,          // UnitOfMeasureConversions.LogicPower5v
//	     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phA
//	     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phB
//	     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phC
//	     
//	    -0.126019646, // 0.196605009,        // UnitOfMeasureConversions.BatteryVoltageChgPosConversion  // Puck/20120725, modify
//	    -0.126019646, // 0.196605009,        // UnitOfMeasureConversions.BatteryVoltageChgNegConversion
//	    -0.04915506,        // UnitOfMeasureConversions.BatteryCurrentPositiveConversion
//	    -0.126019646,        // UnitOfMeasureConversions.BypassVoltageConversion.phA
//	    -0.126019646,        // UnitOfMeasureConversions.BypassVoltageConversion.phB
//	    -0.126019646,        // UnitOfMeasureConversions.BypassVoltageConversion.phC
//	   //0.04915506 = 1/1 * 1/0.0125 * 200/200 * 48.666/12.1 * 9.09/21.79 * 3/4096 * 0.5
//	     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phA
//	     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phB
//	     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phC     
//	     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phA
//	     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phB
//	     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phC     
//	    -0.126019646,        // UnitOfMeasureConversions.InputVoltageConversion.phA
//	    -0.126019646,        // UnitOfMeasureConversions.InputVoltageConversion.phB
//	    -0.126019646,        // UnitOfMeasureConversions.InputVoltageConversion.phC    
//	    -0.126019646,        // UnitOfMeasureConversions.InverterVoltageConversion.phA
//	    -0.126019646,        // UnitOfMeasureConversions.InverterVoltageConversion.phB
//	    -0.126019646,        // UnitOfMeasureConversions.InverterVoltageConversion.phC    
//	
//		//1x    
//	    -0.252039292,        // UnitOfMeasureConversions.RailVoltagePositiveConversion
//	    -0.252039292,        // UnitOfMeasureConversions.RailVoltageNegativeConversion
//	     0.3333,             // UnitOfMeasureConversions.InverterCapTemperatureConversion
//	    -0.252039292,        // UnitOfMeasureConversions.BypassVoltage2phAConversion
//	    -0.252039292,        // UnitOfMeasureConversions.BypassNeutralVoltageConversion
//	    -0.252039292,        // UnitOfMeasureConversions.BypassVoltage2phBConversion
//	     0.3333,            // UnitOfMeasureConversions.RectifierCapTemperatureConversion
//	    -0.252039292,        // UnitOfMeasureConversions.BypassVoltage2phCConversion
//	
//	     -0.252039292,        // UnitOfMeasureConversions.InputVoltageNeutralConversion
//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseAConversion
//	     0.3333,             // UnitOfMeasureConversions.RectifierTemperature0Conversion
//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseBConversion
//	     0.3333,             // UnitOfMeasureConversions.RectifierTemperature1Conversion
//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseCConversion
//	     0.3333,             // UnitOfMeasureConversions.InverterTemperature0Conversion
//	    -0.252039292,        // UnitOfMeasureConversions.OutputVoltagePhaseAConversion
//	     0.3333,             // UnitOfMeasureConversions.STSTemperatureConversion
//	    -0.252039292,        // UnitOfMeasureConversions.OutputVoltagePhaseBConversion
//		 0.004457056,         // UnitOfMeasureConversions.LogicPower15v
//	    -0.252039292,        // UnitOfMeasureConversions.OutputVoltagePhaseCConversion
//	     0.252039292,        // UnitOfMeasureConversions.BatteryVoltagePositiveConversion
//	     0.252039292,        // UnitOfMeasureConversions.BatteryVoltageConversion
//	     0,                  // UnitOfMeasureConversions.CalibrationRef2Conversion
//	     0                   // UnitOfMeasureConversions.CalibrationRef1Conversion    
//	};
{		//HW-Ver02(after 2021.4.7)
    // 12-bit A/D, 0-3V = 3/(2^12) = 0.000732422 V/bit. Divide by circuit gain
    // ex. 30k LV input voltage is inverting op-amp, R2 = 5.1k, R1 = 3*330k + 18k, g = -R2/R1 = -0.005059524
    // so scaler =  0.000732422 / -0.005059524 = -0.144761029. Since fast channels are 2x oversampled
    // need to divide by 2 so final conversion = -0.072380515. Inverter voltage is, of course, different.
	 //2x   
     0.04915506,        // UnitOfMeasureConversions.BatteryCurrentNegativeConversion
     0.000366211,	     // UnitOfMeasureConversions.OpARef
     0.0009137,          // UnitOfMeasureConversions.LogicPower5v
     0.43838704,         // UnitOfMeasureConversions.BypassCurrentConversion.phA
     0.43838704,         // UnitOfMeasureConversions.BypassCurrentConversion.phB
     0.43838704,         // UnitOfMeasureConversions.BypassCurrentConversion.phC
     
    -0.1495,        // UnitOfMeasureConversions.BatteryVoltageChgPosConversion  // Puck/20120725, modify
    -0.1495,        // UnitOfMeasureConversions.BatteryVoltageChgNegConversion
    -0.04915506,        // UnitOfMeasureConversions.BatteryCurrentPositiveConversion
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phA
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phB
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phC
   //0.04915506 = 1/1 * 1/0.0125 * 200/200 * 48.666/12.1 * 9.09/21.79 * 3/4096 * 0.5
     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phA
     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phB
     0.04915506,        // UnitOfMeasureConversions.InputCurrentConversion.phC     
     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phA
     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phB
     0.04915506,        // UnitOfMeasureConversions.InverterCurrentConversion.phC     
    -0.1495,        // UnitOfMeasureConversions.InputVoltageConversion.phA
    -0.1495,        // UnitOfMeasureConversions.InputVoltageConversion.phB
    -0.1495,        // UnitOfMeasureConversions.InputVoltageConversion.phC    
    -0.1495,        // UnitOfMeasureConversions.InverterVoltageConversion.phA
    -0.1495,        // UnitOfMeasureConversions.InverterVoltageConversion.phB
    -0.1495,        // UnitOfMeasureConversions.InverterVoltageConversion.phC    
	
	//1x    
    -0.299,        // UnitOfMeasureConversions.RailVoltagePositiveConversion
    -0.299,        // UnitOfMeasureConversions.RailVoltageNegativeConversion
     0.3333,             // UnitOfMeasureConversions.InverterCapTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phAConversion
//	    -0.299,        // UnitOfMeasureConversions.BypassNeutralVoltageConversion
//		 0.3333, 			// UnitOfMeasureConversions.AMBTemperatureConversion
    0.1098632812,       // UnitOfMeasureConversions.AmbientTemperatureConversion     // 1/9.1022222, LM335 10 mv/K, circuit gain = 2/3, 10*0.001*2/3*4096/3 = 9.1022222, meaning 1K/9.1022222sample value.
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phBConversion
     0.3333,            // UnitOfMeasureConversions.SCRTemperatureConversion //RECCAP
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phCConversion
	
     -0.299,        // UnitOfMeasureConversions.InputVoltageNeutralConversion
     0.001804,        // UnitOfMeasureConversions.InverterDCPhaseAConversion
     0.3333,             // UnitOfMeasureConversions.RectifierTemperatureConversion
     0.001804,        // UnitOfMeasureConversions.InverterDCPhaseBConversion
     0.3333,             // UnitOfMeasureConversions.PhaseATemperatureConversion
     0.001804,        // UnitOfMeasureConversions.InverterDCPhaseCConversion
     0.3333,             // UnitOfMeasureConversions.BatteryTemperature0Conversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseAConversion
     0.3333,             // UnitOfMeasureConversions.PhaseCTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseBConversion
	 0.004457056,         // UnitOfMeasureConversions.LogicPower15v
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseCConversion
//	     0.299,        // UnitOfMeasureConversions.BatteryVoltagePositiveConversion
	 0.0073242190,		// UnitOfMeasureConversions.LogicPower24v
     0.440,        // UnitOfMeasureConversions.BatteryVoltageConversion
//	     0,                  // UnitOfMeasureConversions.CalibrationRef2Conversion
    -0.147216822,        // UnitOfMeasureConversions.ChassisVoltageConversion
//	     0                   // UnitOfMeasureConversions.CalibrationRef1Conversion    
	-0.299		  // UnitOfMeasureConversions.BatteryVoltagePosConversion
};


//windy/20160816/add HV 40k for 3c3pro 
//different item with 30k :BatteryCurrent,InputCurrent,InverterCurrent
const uAdcCalibration UnitOfMeasureConversions_HV_40K =
//	{		//HW-V02(Denis bench )
//	    // 12-bit A/D, 0-3V = 3/(2^12) = 0.000732422 V/bit. Divide by circuit gain
//	    // ex. 30k LV input voltage is inverting op-amp, R2 = 5.1k, R1 = 3*330k + 18k, g = -R2/R1 = -0.005059524
//	    // so scaler =  0.000732422 / -0.005059524 = -0.144761029. Since fast channels are 2x oversampled
//	    // need to divide by 2 so final conversion = -0.072380515. Inverter voltage is, of course, different.
//	     //2x
//	     0.09831012,        // UnitOfMeasureConversions.BatteryCurrentNegativeConversion
//	     0.000366211,        // UnitOfMeasureConversions.OpARefConversion 
//	     0.0009137,        // UnitOfMeasureConversions.LogicPowersupply12v
//	
//	     0.048828125,		 // UnitOfMeasureConversions.BypassCurrentConversion.phA
//	     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phB
//	     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phC
//	
//	    -0.135077088,        // UnitOfMeasureConversions.BatteryVoltageChgPosConversion
//	    -0.135077088,         // UnitOfMeasureConversions.BatteryVoltageChgNegConversion
//	    
//	    -0.09831012,          //UnitOfMeasureConversions.BatteryCurrentPositiveConversion 
//	    -0.135077088,        // UnitOfMeasureConversions.BypassVoltageConversion.phA
//	    -0.135077088,        // UnitOfMeasureConversions.BypassVoltageConversion.phB
//	    -0.135077088,        // UnitOfMeasureConversions.BypassVoltageConversion.phC
//	
//	    //0.04915506 = 1/1 * 1/0.0125 * 200/200 * 48.666/12.1 * 9.09/21.79 * 3/4096 * 0.5
//	    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phA
//	    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phB
//	    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phC     
//	    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phA
//	    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phB
//	    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phC
//	
//	    -0.135077088,       // UnitOfMeasureConversions.InputVoltageConversion.phA          // klv: These voltages are not correct, but reflect reworked if board
//	    -0.135077088,       // UnitOfMeasureConversions.InputVoltageConversion.phB
//	    -0.135077088,       // UnitOfMeasureConversions.InputVoltageConversion.phC
//	    -0.135077088,       // UnitOfMeasureConversions.InverterVoltageConversion.phA
//	    -0.135077088,       // UnitOfMeasureConversions.InverterVoltageConversion.phB
//	    -0.135077088,       // UnitOfMeasureConversions.InverterVoltageConversion.phC    
//	
//		//1x
//	    -0.270154176,        // UnitOfMeasureConversions.RailVoltagePositiveConversion
//	    -0.270154176,        // UnitOfMeasureConversions.RailVoltageNegativeConversion
//	     0.3333,             // UnitOfMeasureConversions.InverterCapTemperatureConversion
//	    -0.270154176,        // UnitOfMeasureConversions.BypassVoltage2phAConversion
//	    -0.270154176,        // UnitOfMeasureConversions.BypassNeutralVoltageConversion
//	    -0.270154176,        // UnitOfMeasureConversions.BypassVoltage2phBConversion
//	     0.3333,            // UnitOfMeasureConversions.RectifierCapTemperatureConversion
//	    -0.270154176,        // UnitOfMeasureConversions.BypassVoltage2phCConversion
//	
//	    -0.270154176,        // UnitOfMeasureConversions.InputVoltageNeutralConversion
//	//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseAConversion
//		 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseAConversion
//	     0.3333,             // UnitOfMeasureConversions.RectifierTemperatureConversion
//	//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseBConversion
//		 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseBConversion
//	     0.3333,             // UnitOfMeasureConversions.InverterTemperatureConversion
//	//	     0.001796994,        // UnitOfMeasureConversions.InverterDCPhaseCConversion
//		 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseCConversion
//	     0.3333,             // UnitOfMeasureConversions.BatteryTemperatureConversion
//	    -0.270154176,        // UnitOfMeasureConversions.OutputVoltagePhaseAConversion
//	
//	     0.3333,             // UnitOfMeasureConversions.STSTemperatureConversion
//	    -0.270154176,        // UnitOfMeasureConversions.OutputVoltagePhaseBConversion
//	     0.004457056,         // UnitOfMeasureConversions.LogicPower15v
//	    -0.270154176,        // UnitOfMeasureConversions.OutputVoltagePhaseCConversion
//	     0.270154176,        // UnitOfMeasureConversions.BatteryVoltagePositiveConversion
//	     0.270154176,        // UnitOfMeasureConversions.BatteryVoltageConversion
//	     0,                  // UnitOfMeasureConversions.CalibrationRef2Conversion
//	     0                   // UnitOfMeasureConversions.CalibrationRef1Conversion    
//	};
{		//HW-Ver02(other release ver)
    // 12-bit A/D, 0-3V = 3/(2^12) = 0.000732422 V/bit. Divide by circuit gain
    // ex. 30k LV input voltage is inverting op-amp, R2 = 5.1k, R1 = 3*330k + 18k, g = -R2/R1 = -0.005059524
    // so scaler =  0.000732422 / -0.005059524 = -0.144761029. Since fast channels are 2x oversampled
    // need to divide by 2 so final conversion = -0.072380515. Inverter voltage is, of course, different.
     //2x
     0.09831012,        // UnitOfMeasureConversions.BatteryCurrentNegativeConversion
     0.000366211,        // UnitOfMeasureConversions.OpARefConversion 
     0.0009137,        // UnitOfMeasureConversions.LogicPowersupply12v
	
     0.43838704,//0.079784518,  //     0.048828125,		 // UnitOfMeasureConversions.BypassCurrentConversion.phA
     0.43838704,//0.079784518,  //     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phB
     0.43838704,//0.079784518,  //     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phC
	
    -0.1495,         // UnitOfMeasureConversions.BatteryVoltageChgPosConversion
    -0.1495,         // UnitOfMeasureConversions.BatteryVoltageChgNegConversion
    
    -0.09831012,          //UnitOfMeasureConversions.BatteryCurrentPositiveConversion 
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phA
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phB
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phC
	
    //0.04915506 = 1/1 * 1/0.0125 * 200/200 * 48.666/12.1 * 9.09/21.79 * 3/4096 * 0.5
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phA
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phB
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phC     
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phA
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phB
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phC
	
    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phA          // klv: These voltages are not correct, but reflect reworked if board
    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phB
    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phC
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phA
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phB
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phC    

	//1x
    -0.299,        // UnitOfMeasureConversions.RailVoltageNegativeConversion
    -0.299,        // UnitOfMeasureConversions.RailVoltagePositiveConversion
     0.3333,             // UnitOfMeasureConversions.InverterCapTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phAConversion
//	    -0.299,        // UnitOfMeasureConversions.BypassNeutralVoltageConversion
//		0.3333, 			// UnitOfMeasureConversions.AMBTemperatureConversion
    0.1098632812,       // UnitOfMeasureConversions.AmbientTemperatureConversion     // 1/9.1022222, LM335 10 mv/K, circuit gain = 2/3, 10*0.001*2/3*4096/3 = 9.1022222, meaning 1K/9.1022222sample value.
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phBConversion
     0.3333,            // UnitOfMeasureConversions.SCRTemperatureConversion  //RECCAP
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phCConversion

    -0.299,        // UnitOfMeasureConversions.InputVoltageNeutralConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseAConversion
     0.3333,         //    // UnitOfMeasureConversions.PhaseBTemperatureConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseBConversion
     0.3333,             // UnitOfMeasureConversions.PhaseATemperatureConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseCConversion
     0.3333,             // UnitOfMeasureConversions.BatteryTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseAConversion
	
     0.3333,             // UnitOfMeasureConversions.PhaseCTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseBConversion
     0.004457056,         // UnitOfMeasureConversions.LogicPower15v
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseCConversion
//	     0.299,        // UnitOfMeasureConversions.BatteryVoltagePositiveConversion
	 0.0073242190,		 // UnitOfMeasureConversions.LogicPower24v
	 0.440,              // 0.299,        // UnitOfMeasureConversions.BatteryVoltageConversion
//	     0,                  // UnitOfMeasureConversions.CalibrationRef2Conversion
    -0.147216822,        // UnitOfMeasureConversions.ChassisVoltageConversion
//	     0                   // UnitOfMeasureConversions.CalibrationRef1Conversion    
	-0.299		  // UnitOfMeasureConversions.BatteryVoltagePosConversion

};


const uAdcCalibration UnitOfMeasureConversions_HV_40K_VbatRatio =
{		//HW-Ver02(other release ver)
    // 12-bit A/D, 0-3V = 3/(2^12) = 0.000732422 V/bit. Divide by circuit gain
    // ex. 30k LV input voltage is inverting op-amp, R2 = 5.1k, R1 = 3*330k + 18k, g = -R2/R1 = -0.005059524
    // so scaler =  0.000732422 / -0.005059524 = -0.144761029. Since fast channels are 2x oversampled
    // need to divide by 2 so final conversion = -0.072380515. Inverter voltage is, of course, different.
     //2x
     0.09831012,        // UnitOfMeasureConversions.BatteryCurrentNegativeConversion
     0.000366211,        // UnitOfMeasureConversions.OpARefConversion
     0.0009137,        // UnitOfMeasureConversions.LogicPowersupply12v

     0.048828125,		 // UnitOfMeasureConversions.BypassCurrentConversion.phA
     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phB
     0.048828125,         // UnitOfMeasureConversions.BypassCurrentConversion.phC

    -0.1495,         // UnitOfMeasureConversions.BatteryVoltageChgPosConversion
    -0.1495,         // UnitOfMeasureConversions.BatteryVoltageChgNegConversion

    -0.09831012,          //UnitOfMeasureConversions.BatteryCurrentPositiveConversion
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phA
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phB
    -0.1495,        // UnitOfMeasureConversions.BypassVoltageConversion.phC

    //0.04915506 = 1/1 * 1/0.0125 * 200/200 * 48.666/12.1 * 9.09/21.79 * 3/4096 * 0.5
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phA
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phB
    0.09831012,        // UnitOfMeasureConversions.InputCurrentConversion.phC
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phA
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phB
    0.09831012,        // UnitOfMeasureConversions.InverterCurrentConversion.phC

    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phA          // klv: These voltages are not correct, but reflect reworked if board
    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phB
    -0.1495,       // UnitOfMeasureConversions.InputVoltageConversion.phC
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phA
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phB
    -0.1495,       // UnitOfMeasureConversions.InverterVoltageConversion.phC

	//1x
    -0.299,        // UnitOfMeasureConversions.RailVoltageNegativeConversion
    -0.299,        // UnitOfMeasureConversions.RailVoltagePositiveConversion
     0.3333,             // UnitOfMeasureConversions.InverterCapTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phAConversion
//	    -0.299,        // UnitOfMeasureConversions.BypassNeutralVoltageConversion
//		0.3333, 			// UnitOfMeasureConversions.AMBTemperatureConversion
    0.1098632812,       // UnitOfMeasureConversions.AmbientTemperatureConversion     // 1/9.1022222, LM335 10 mv/K, circuit gain = 2/3, 10*0.001*2/3*4096/3 = 9.1022222, meaning 1K/9.1022222sample value.
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phBConversion
     0.3333,            // UnitOfMeasureConversions.SCRTemperatureConversion  RECCAP
    -0.299,        // UnitOfMeasureConversions.BypassVoltage2phCConversion

    -0.299,        // UnitOfMeasureConversions.InputVoltageNeutralConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseAConversion
     0.3333,             // UnitOfMeasureConversions.PhaseBTemperatureConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseBConversion
     0.3333,             // UnitOfMeasureConversions.PhaseATemperatureConversion
	 0.001804,		 // UnitOfMeasureConversions.InverterDCPhaseCConversion
     0.3333,             // UnitOfMeasureConversions.BatteryTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseAConversion

     0.3333,             // UnitOfMeasureConversions.PhaseCTemperatureConversion
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseBConversion
     0.004457056,         // UnitOfMeasureConversions.LogicPower15v
    -0.299,        // UnitOfMeasureConversions.OutputVoltagePhaseCConversion
//	     0.299,        // UnitOfMeasureConversions.BatteryVoltagePositiveConversion
	 0.0073242190,		 // UnitOfMeasureConversions.LogicPower24v
     0.440,        // UnitOfMeasureConversions.BatteryVoltageConversion
//	     0,                  // UnitOfMeasureConversions.CalibrationRef2Conversion
    -0.147216822,        // UnitOfMeasureConversions.ChassisVoltageConversion
//	     0                   // UnitOfMeasureConversions.CalibrationRef1Conversion
	-0.299		  // UnitOfMeasureConversions.BatteryVoltagePosConversion

};

// slow filtered analog reference
#define   __IQ30(A)      (long) ((A) * 1073741824.0L)
stSecondOrderIIR AnalogZeroFilter = { __IQ30( 0.0005 ),     // B0
                                      __IQ30( 0 )   ,       // X1
                                      __IQ30( -0.9995 ),    // A1
                                      __IQ30( 0 )   ,       // B1
                                      __IQ30( 0 )   ,       // X2
                                      __IQ30( 0 )   ,       // A2
                                      __IQ30( 0 )   } ;     // B2
                                      
                                      
    // Variables for Waveform Capture
stCaptureType CaptureControl;

#if CAPTURE_DUAL_CHANNEL == 0x55AA

uint16_t Capture2ndBuffer[CAPTUR_2ND_BUFFER_SIZE];
#endif /* end of CAPTURE_DUAL_CHANNEL */

#pragma DATA_SECTION("DataBuffer");
volatile stCaptureData CaptureData[CAPTURE_SIZE];
                                         
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
#pragma FUNC_EXT_CALLED;
void AdcDMAConfigure( void )
{
	memset(&Adc_Sample.words, 0, sizeof(Adc_Sample.words));
	memset(const_cast<int16_t*>(&AdcDMABuffer[0][0]), 0, sizeof(AdcDMABuffer));
	memset(&SlowAdcData.w, 0, sizeof(SlowAdcData.w));
	for (size_t i = 0; i < RAW_A2D_CAL_SIZE; ++i)
	{
		CalibrationScaler.w[i] = 0.0f;
	}
	
	for (size_t i = 0; i < RAW_ADC_SIZE; ++i)
	{
		RawAdcData.w[i] = 0.0f;
	}
	
	
    volatile int16_t* dmaDst =  AdcDMABuffer[0];
    volatile uint16_t* dmaSrc = &AdcMirror.ADCRESULT0;
    
    DMACH1AddrConfig( (uint16_t*)dmaDst, dmaSrc );
    DMACH1BurstConfig( 15, 1, 1 );      // 16 words/burst(X-1), increment source 1 word, increment dest 1 word
    DMACH1TransferConfig( 1, 1, 1 );    // 2 bursts/transfer(X-1), src transfer step, dest transfer step
    DMACH1WrapConfig( 0, 0, 1, 0 );     // wrap source after 1 burst, no offset, wrap dest after 2 bursts, no offset
    DMACH1ModeConfig( DMA_SEQ1INT, PERINT_ENABLE, ONESHOT_DISABLE, CONT_DISABLE, SYNC_DISABLE, SYNC_SRC,
                      OVRFLOW_DISABLE, SIXTEEN_BIT, CHINT_END, CHINT_ENABLE );
                      
    StartDMACH1();                   
}

// ***********************************************************************
// *
// *    FUNCTION: adc_isr 
// *
// *    DESCRIPTION: ADC EOC interrupt service routine
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma INTERRUPT( HPI );               // this is a high priority interrupt, it cannot be interrupted itself. Results in shorter context save/restore
#pragma CODE_SECTION( "ramfuncs" );
#pragma FUNC_EXT_CALLED;
void adc_isr( void )
{   
	static uint16_t  divideCounter=0;
    // make sure we can't be interrupted, if somebody calls "EINT" from within the context of this isr
    CriticalSection enter;
    uint32_t regValue;
    //get enter counter
    //CLK_EnterISRCnt = CLK_gethtime();

    ADMuxState++;
    
    // set DMA buffer from AD state
    volatile uint16_t* dmaSrc = &AdcMirror.ADCRESULT0;
    volatile int16_t*  dmaDst;
        
    if ( ADMuxState & 0x1 )
    {
        dmaDst = AdcDMABuffer[1];
    }
    else
    {
        dmaDst = AdcDMABuffer[0];
    }
    // reconfigure DMA    
    EALLOW;
    // Set up SOURCE address:
    DmaRegs.CH1.SRC_BEG_ADDR_SHADOW = (Uint32)dmaSrc;   // Point to beginning of source buffer
    DmaRegs.CH1.SRC_ADDR_SHADOW =     (Uint32)dmaSrc;

    // Set up DESTINATION address:
    DmaRegs.CH1.DST_BEG_ADDR_SHADOW = (Uint32)dmaDst;     // Point to beginning of destination buffer
    DmaRegs.CH1.DST_ADDR_SHADOW =     (Uint32)dmaDst;
    
    DmaRegs.CH1.CONTROL.bit.RUN = 1;
    
    EDIS;
    // write AD mux out
    DSPOutRegister.GpoB.bit.AD_Mux_1_2 = ( ADMuxState & 0x3 );

	/*regValue = DSPOutRegister.GpoA.all;
    GpioDataRegs.GPASET.all = regValue;
    regValue = ~DSPOutRegister.GpoA.all;
    GpioDataRegs.GPACLEAR.all = regValue;

    regValue = DSPOutRegister.GpoB.all;
    GpioDataRegs.GPBSET.all = regValue;
    regValue = ~DSPOutRegister.GpoB.all;
    GpioDataRegs.GPBCLEAR.all = regValue;

    regValue = DSPOutRegister.GpoC.all;
    regValue &= OUTREGC_UPDATE_MASK;//can't rewrite PWM polarity
    GpioDataRegs.GPCSET.all = regValue;
    regValue = ~DSPOutRegister.GpoC.all;
    regValue &= OUTREGC_UPDATE_MASK;//can't rewrite PWM polarity
    GpioDataRegs.GPCCLEAR.all = regValue;*/
    WriteDSPOutputs_ISR();
    // get fast zero reference
    AnalogZeroRawDouble = ( AdcDMABuffer[0][FAST_OPAREF_IDX] + AdcDMABuffer[1][FAST_OPAREF_IDX] );
    
    // process fast a/d channels    
    float* restrict fastAdc                                  = (float*)&RawAdcData.w[0];
    float* restrict fastAdcCal                               = &CalibrationScaler.w[0];
    volatile const int16_t* restrict oversampleBuffer0       = (int16_t*)&AdcDMABuffer[0];
    volatile const int16_t* restrict oversampleBuffer1       = (int16_t*)&AdcDMABuffer[1];
    const int16_t * restrict const * restrict FastAdcZeroPtr = &FastAdcZero[0];
    
    const size_t iter = NumberOfAdcFastChannels;
    
    // process first 12
    // NOTE: The compiler is very sensitive to the operations in these loops! For example,
    //  changing the ORDER of operations so that the subtract is last results in a 1% CPU
    //  time penalty!  Check the assembly after you change this!
    
    float tempFloat = float( -(**FastAdcZeroPtr++ ) + *oversampleBuffer0++ + *oversampleBuffer1++ ) ;
    int16_t tempAdc = -(**FastAdcZeroPtr++ ) + *oversampleBuffer0++ + *oversampleBuffer1++;
    for ( size_t loopIdx = 0; loopIdx < ( iter / 2 ) - 2; loopIdx++ )
    {
        float temp_alt = float(tempAdc);
        *fastAdc++ = tempFloat * ( *fastAdcCal++ );
        tempAdc = -(**FastAdcZeroPtr++ ) + *oversampleBuffer0++ + *oversampleBuffer1++;
        tempFloat = temp_alt;
    }
    *fastAdc++ = tempFloat * ( *fastAdcCal++ ); 
    *fastAdc++ = float(tempAdc) * ( *fastAdcCal++ );

    // next 4 are medium/slow channels, skip
    oversampleBuffer0   = (int16_t*)&AdcDMABuffer[0][16];
    oversampleBuffer1   = (int16_t*)&AdcDMABuffer[1][16];
         
    // process next 12
    tempFloat = float((*oversampleBuffer0++) + (*oversampleBuffer1++) - (**FastAdcZeroPtr++)) ;
    tempAdc =  - (**FastAdcZeroPtr++ ) + (*oversampleBuffer0++) + (*oversampleBuffer1++);
    for ( size_t loopIdx = iter / 2; loopIdx < iter - 2; loopIdx++ )
    {
        float temp_alt = float(tempAdc);
        *fastAdc++ = tempFloat * ( *fastAdcCal++ );
        tempAdc = -(**FastAdcZeroPtr++) + (*oversampleBuffer0++) + (*oversampleBuffer1++);
        tempFloat = temp_alt;
    }
    *fastAdc++ = tempFloat * ( *fastAdcCal++ );
    *fastAdc = float(tempAdc) * ( *fastAdcCal );

    //20241011  old ctrl board : R && T change
    if(NewCtrlBoard == 0)
    {
        float BypassVoltPhATemp = 0;
        float BypassVoltPhCTemp = 0;
        BypassVoltPhATemp = RawAdcData.st.BypassVoltage.phA;
        BypassVoltPhCTemp = RawAdcData.st.BypassVoltage.phC;
        RawAdcData.st.BypassVoltage.phA = BypassVoltPhCTemp;
        RawAdcData.st.BypassVoltage.phC = BypassVoltPhATemp;
    }

    

    // Medium adc channels, downsample by 2, each channel runs 20.4kHz
    float * restrict medAdcResult ;
    const float * restrict medAdcCal;
	//*********************************************
	//TODO,for medium adc, this writing will easy cause bug when channel variate 
	//		cause this not useless: int16_t * const MedAdcZero[2][4], 
	//*********************************************
	if(ADMuxState & 0x1)
	{
		medAdcResult = MedAdcResult[1];
		medAdcCal = MedAdcCal[1];
		tempFloat = float(AdcDMABuffer[0][MEDIUM_RAIL_NEGATIVE_IDX] - AnalogZero);
		tempAdc = AdcDMABuffer[0][MEDIUM_RAIL_POSITIVE_IDX] - AnalogZero;
		medAdcResult[0] = tempFloat * medAdcCal[0];
		tempFloat = float(AdcDMABuffer[0][MEDIUM_INVCAP_TEMP_IDX]);
		medAdcResult[1] = float(tempAdc) * medAdcCal[1];
		tempAdc = AdcDMABuffer[0][MEDIUM_BYPASS_V_A_IDX] - AnalogZero;
		medAdcResult[2] = tempFloat *  medAdcCal[2];
		medAdcResult[3] = float(tempAdc) *  medAdcCal[3];		
	}
	else
	{
		medAdcResult = MedAdcResult[0];
		medAdcCal = MedAdcCal[0];

//			tempFloat = float(AdcDMABuffer[1][MEDIUM_AMB_TEMP_IDX] - AnalogZero);
		tempFloat = float(AdcDMABuffer[1][MEDIUM_AMB_TEMP_IDX]);			
		tempAdc = AdcDMABuffer[1][MEDIUM_BYPASS_V_B_IDX] - AnalogZero;
		medAdcResult[0] = tempFloat * medAdcCal[0];
		tempFloat = float(AdcDMABuffer[1][MEDIUM_SCR_TEMP_IDX]);  //MEDIUM_RECTCAP_TEMP_IDX
		medAdcResult[1] = float(tempAdc) * medAdcCal[1];
		tempAdc = AdcDMABuffer[1][MEDIUM_BYPASS_V_C_IDX] - AnalogZero;
		medAdcResult[2] = tempFloat *  medAdcCal[2];
		medAdcResult[3] = float(tempAdc) *  medAdcCal[3];
	}
 
    // The following code within "if" range is for FCT and only runs in test mode

    // Slow adc channels, downsample by 4, each channel runs 10.2kHz
    int16_t * restrict slowAdcResult                               = SlowAdcResultRaw[ADMuxState & 0x3];
    volatile const int16_t * restrict const * restrict slowAdcData = &SlowAdcDataRaw[ADMuxState & 0x3][0];

    slowAdcResult[0] = *slowAdcData[0];
    slowAdcResult[1] = *slowAdcData[1];
    slowAdcResult[2] = *slowAdcData[2];
    slowAdcResult[3] = *slowAdcData[3];
    
    // The following code within "if" range is for FCT and only runs in test mode
    if( MCUStateMachine.UPMTestMode )
    {
        int16_t * restrict medAdcResult                                = MedAdcSample[ADMuxState & 0x1];
        volatile const int16_t * restrict const * restrict medAdcData  = &MedAdcData[ADMuxState & 0x1][0];
        medAdcResult[0] = *medAdcData[0];
        medAdcResult[1] = *medAdcData[1];
        medAdcResult[2] = *medAdcData[2];
        medAdcResult[3] = *medAdcData[3];

        int16_t * restrict slowAdcResult                               = SlowAdcSample[ADMuxState & 0x3];
        volatile const int16_t * restrict const * restrict slowAdcData = &SlowAdcDataRaw[ADMuxState & 0x3][0];
        slowAdcResult[0] = *slowAdcData[0];
        slowAdcResult[1] = *slowAdcData[1];
        slowAdcResult[2] = *slowAdcData[2];
        slowAdcResult[3] = *slowAdcData[3];
    }

    if(( ADMuxState & 0x0003) == 1 )	//01:inv,  52k/4=13k		//0b00~b11: 0~3
    {
        Inverter.Run_ISR();

        if( !Inverter.GetStatus().bit.InverterOn && MCUStateMachine.PullChainSyncStart )
        {
            // Hold angle at zero and reset frequency for pull-chain start
            BaseSineRef.UpdateFrequencyAndAngle( BaseSineRef.GetBaseFrequency() , 0.0f );
        }
        else
        {
            BaseSineRef.UpdateAngle();
        }

        BypassPLL.SineRef.UpdateAngle();
        // inverter updates it's own angle   
    }
    else if(( ADMuxState & 0x0003) == 3 )	//03:rec
    {
		Rectifier.UtilityPLL.SineRef.UpdateAngle();
		Rectifier.Run_ISR();
        OutputPLL.SineRef.UpdateAngle();
        // rectifier updates it's own angle
        BatteryConverter.BoostLegShare();
        BatteryConverter.ChargerCurrControl();

    }
//		Rectifier.Run_ISR();

	if(( ADMuxState & 0x0001) == 0)	//00-wave_cap, 52k/2
	{
//		if( BatteryConverterStatus.bit.BoostOn && Rectifier.ReuseRectSTPWMEnable)
		if(Rectifier.ReuseRectSTPWMEnable)
		{
		    RawAdcData.st.BatteryCurrentPositiveSum = RawAdcData.st.BatteryCurrentPositive + (-RawAdcData.st.InputCurrent.phB);
		    RawAdcData.st.BatteryCurrentNegativeSum = RawAdcData.st.BatteryCurrentNegative + (RawAdcData.st.InputCurrent.phC);
		}
		else
		{
		    RawAdcData.st.BatteryCurrentPositiveSum = RawAdcData.st.BatteryCurrentPositive;
		    RawAdcData.st.BatteryCurrentNegativeSum = RawAdcData.st.BatteryCurrentNegative;
		}
		
	    WaveformCapture();
	}
    // Test total output interrupt time when ESS transfer back to online.
    //TestESSTransferTime();

    divideCounter++;
	if(divideCounter >= 12) //26KHZ/6 = 4.333KHZ
	{
		divideCounter = 0;
		// trigger ProcessA2D_HWI every 6th pass
		ToggleDSPWatchDog();                    // 2.5kHz min signal so PLD doesn't reset
		PieCtrlRegs.PIEIFR2.bit.INTx1 = 1;
	}
//    if ( 0x0007 == ( ADMuxState & 0x0007 ) )	//36k/8
//    {
//        // trigger ProcessAdc_HWI every 8th pass
//        ToggleDSPWatchDog();                    // 2.5kHz min signal so PLD doesn't reset
//        PieCtrlRegs.PIEIFR2.bit.INTx1 = 1;
//    }

    //get exit time counter and update
    //CLK_ExitISRCnt = CLK_gethtime();
    //UpdateRuntime(eAdcIsr, fabs(CLK_ExitISRCnt - CLK_EnterISRCnt));

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;   // Acknowledge interrupt to PIE

    return;
}

/********************************************************************************************************
*
* Function Name: TestESSTransferTime
*
* Abstract: Test total output interrupt time when ESS transfer back to online.
*
********************************************************************************************************/
//#pragma CODE_SECTION( "ramfuncs" );
inline void TestESSTransferTime(void)
{
    // Test total output interrupt time when ESS transfer back to online.
    if ( EssTimeTest )
    {
    	//EssTimeCount++;

        // run abc-dq0 transform
        abc_to_dq0( ( stThreePhase* )&RawAdcDataPtr->st.OutputVoltageRaw, &OutputVoltageDQ, OutputPLL.SineRef.Angle );

        // nominal data for UV/OV detection
        OutputVoltageDQ.Sd = OutputVoltageDQ.Sd * OutputPLL.GetNominalNormFactor();
     //   OutputVoltageDQ.Sd = FirstOrderIIRFP( OutputVoltageDQ.Sd, &SdFilter_40KHz );   // Sd filtering

    	if(OutputVoltageDQ.Sd < 0.8)  //EEP 227
    	{
    		EssTimeCount++;
    		if(record_uv_time == false) // reset EssTimeCount at the first moment that output AC UV
    		{
        		EssTimeCount = 0;
        		record_uv_time = true;
    		}
    		else // record the last moment that output AC UV
    		{
    			EssTransferTime = EssTimeCount * 0.0245;  //transfer time in ms. 0.0245 = 1/40816 * 1000
    		}
    	}
    	else
    	{
    		if(record_uv_time == false)  // if Output AC UV never happens, transfer time is 0.
    		{
    			EssTransferTime = 0;
    		}
    	}
    }
    else
    {
    	EssTimeCount = 0;
    	record_uv_time = false;
    	EssTransferTime = 0;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: ProcessSlowAdc  
// *
// *    DESCRIPTION: Processing of slow Adc channels
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" );
#pragma FUNC_EXT_CALLED;
void ProcessSlowAdc( void )
{
    const size_t NumberOfSlowAdChannels = 16;
    
    // process A2D zero first, is different than the others
    {
        AnalogZeroDouble = SecondOrderIIR( ( (int32_t)AnalogZeroRawDouble << 16 ), &AnalogZeroFilter ) >> 16;
		AnalogZero = AnalogZeroDouble >> 1;
		AnalogZeroRaw = AnalogZeroRawDouble >> 1;
	}

	//for falcon PBW 454
	volatile float* restrict adcResult						= (float*)&RawAdcData.st.InputVoltageNeutral;	  // point to 1st slow channel
    const float* restrict adcCal                            = &CalibrationScaler.st.InputNeutralCal;          // get calibration scaler
    volatile const int16_t* restrict rawData                = &SlowAdcData.st.InputVoltageNeutral;            // get source data

    // A ptr that is the only reference to a mem location, consisting of
    // ptrs that are constant and the only refernece to a mem location, consisting of
    // int16_t values that cannot be modified, but can change at any instruction
    volatile const int16_t * restrict const * restrict SlowAdcZeroPtr = &SlowAdcZero[0];

	//for jira hobbit55 Vbat+ sample err, change to same as 93P, 
    const uint16_t iter = NumberOfSlowAdChannels-1; //unroll last loop round.
    float temp_alt;
    float temp = float( *rawData++ - **SlowAdcZeroPtr++ ) ;
    int32_t temp2 = *rawData++ - **SlowAdcZeroPtr++;

    for(uint16_t i=0; i< iter; i++ )
    {
        temp_alt = float(temp2);
        *adcResult++ = temp * ( *adcCal++ );
        if(i < iter - 1)
        {
        	temp2 = ( *rawData++ - **SlowAdcZeroPtr++ );
        }
        temp = temp_alt;
    }
    //the last round of loop is unrolled here and unecessary stuff removed for speed
    //temp_alt = float(temp2);
    *adcResult = temp * ( *adcCal );    // ++ removed as they are not needed
    //temp2 = ( *rawData++ - **SlowAdcZeroPtr++ );
    //temp = temp_alt;
    
    volatile const uRawAdcData * rawAdcDataPtr = &RawAdcData;	
		
    // populate the 3 phase data struct
    RawAdcData.st.OutputVoltageRaw.phA = rawAdcDataPtr->st.OutputVoltagePhaseA;
    RawAdcData.st.OutputVoltageRaw.phB = rawAdcDataPtr->st.OutputVoltagePhaseB;
    RawAdcData.st.OutputVoltageRaw.phC = rawAdcDataPtr->st.OutputVoltagePhaseC; 
    
    // Bypass Voltage stored to stThreePhase to minimize DP loads.
    stThreePhase byp_volt_temp;
    
    byp_volt_temp.phA = rawAdcDataPtr->st.BypassVoltage2phA;
    byp_volt_temp.phB = rawAdcDataPtr->st.BypassVoltage2phB;
    byp_volt_temp.phC = rawAdcDataPtr->st.BypassVoltage2phC;

    //20241912  A and C phases are sampled and swapped
    if(NewCtrlBoard == 1)
    {
    	RawAdcData.st.BypassVoltage2.phA = byp_volt_temp.phA;
    	RawAdcData.st.BypassVoltage2.phB = byp_volt_temp.phB;
    	RawAdcData.st.BypassVoltage2.phC = byp_volt_temp.phC;
    }
    else
    {
        RawAdcData.st.BypassVoltage2.phA = byp_volt_temp.phC;
        RawAdcData.st.BypassVoltage2.phB = byp_volt_temp.phB;
        RawAdcData.st.BypassVoltage2.phC = byp_volt_temp.phA;
    }


    
//	    RawAdcData.st.BatteryVoltage = rawAdcDataPtr->st.BatteryVoltageAD; // - rawAdcDataPtr->st.BatteryVoltage );
	RawAdcData.st.BatteryVoltageNeg = -(rawAdcDataPtr->st.BatteryVoltage - rawAdcDataPtr->st.BatteryVoltagePos);

}

// ***********************************************************************
// *
// *    FUNCTION: CurrentSensorAutoZero  
// *
// *    DESCRIPTION: Zeros out Hall sensors, not output CT.
// *                 To be called during initialization when there is no
// *                 current in the sensors.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma FUNC_EXT_CALLED;
void CurrentSensorAutoZero( void )
{
    // input current
    int16_t sample = AdcDMABuffer[0][FAST_UTIL_AMPS_A_IDX] + AdcDMABuffer[1][FAST_UTIL_AMPS_A_IDX];
    InputCurrentAutoZero.phA -= ( ( InputCurrentAutoZero.phA - sample ) >> 2 );//double of real value
    
    sample = AdcDMABuffer[0][FAST_UTIL_AMPS_B_IDX] + AdcDMABuffer[1][FAST_UTIL_AMPS_B_IDX];
    InputCurrentAutoZero.phB -= ( ( InputCurrentAutoZero.phB - sample ) >> 2 );//double of real value     

    sample = AdcDMABuffer[0][FAST_UTIL_AMPS_C_IDX] + AdcDMABuffer[1][FAST_UTIL_AMPS_C_IDX] ;
    InputCurrentAutoZero.phC -= ( ( InputCurrentAutoZero.phC - sample ) >> 2 ); //double of real value    

    // inverter current
    sample = AdcDMABuffer[0][FAST_INV_AMPS_A_IDX] + AdcDMABuffer[1][FAST_INV_AMPS_A_IDX] ;
    InverterCurrentAutoZero.phA -= ( ( InverterCurrentAutoZero.phA - sample ) >> 2 );//double of real value
    
    sample = AdcDMABuffer[0][FAST_INV_AMPS_B_IDX] + AdcDMABuffer[1][FAST_INV_AMPS_B_IDX];
    InverterCurrentAutoZero.phB -= ( ( InverterCurrentAutoZero.phB - sample ) >> 2 ); //double of real value    

    sample = AdcDMABuffer[0][FAST_INV_AMPS_C_IDX] + AdcDMABuffer[1][FAST_INV_AMPS_C_IDX] ;
    InverterCurrentAutoZero.phC -= ( ( InverterCurrentAutoZero.phC - sample ) >> 2 ); //double of real value

    // Bypass current
    if( BypassState().GetMonitoredBypassState() != BYPASS_FIRE_STATE )
    {
		sample = AdcDMABuffer[0][FAST_BYPASS_CURRENT_A_IDX] + AdcDMABuffer[1][FAST_BYPASS_CURRENT_A_IDX] ;
		BypassCurrentAutoZero.phA -= ( ( BypassCurrentAutoZero.phA - sample ) >> 2 );//double of real value

		sample = AdcDMABuffer[0][FAST_BYPASS_CURRENT_B_IDX] + AdcDMABuffer[1][FAST_BYPASS_CURRENT_B_IDX];
		BypassCurrentAutoZero.phB -= ( ( BypassCurrentAutoZero.phB - sample ) >> 2 ); //double of real value

		sample = AdcDMABuffer[0][FAST_BYPASS_CURRENT_C_IDX] + AdcDMABuffer[1][FAST_BYPASS_CURRENT_C_IDX] ;
		BypassCurrentAutoZero.phC -= ( ( BypassCurrentAutoZero.phC - sample ) >> 2 ); //double of real value
    }
}

// ***********************************************************************
// *
// *    FUNCTION: BatteryCurrentAutoZero  
// *
// *    DESCRIPTION: Zeros out Hall sensors, not output CT.
// *                 To be called during battery transitions when there is no
// *                 current in the sensors.
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma FUNC_EXT_CALLED;
void BatteryCurrentAutoZero( void )
{
    // battery current
    int16_t sample =  AdcDMABuffer[0][FAST_IBATT_POS_IDX] + AdcDMABuffer[1][FAST_IBATT_POS_IDX];
    BattCurrentPosAutoZero -= ( ( BattCurrentPosAutoZero - sample ) >> 2 );//double of real value 
    
    sample = AdcDMABuffer[0][FAST_IBATT_NEG_IDX] + AdcDMABuffer[1][FAST_IBATT_NEG_IDX];
    BattCurrentNegAutoZero -= ( ( BattCurrentNegAutoZero - sample ) >> 2 );//double of real value
}
 

/********************************************************************************************************
*
* Function Name: WaveformCapture
*
* Abstract: Samples data and stores it in an array.
*
********************************************************************************************************/
#pragma CODE_SECTION( "ramfuncs" );
inline void WaveformCapture(void)
{
	float TriggerSource = 0.0f;
#if CAPTURE_DUAL_CHANNEL == 0x55AA
	uint32_t SampleValue;
#endif

	if ((CaptureControl.ArmTrigger == true) && (CaptureControl.PtrTrig.ptrLong != 0))
	{
	#if !defined(CAPTURE_TRIGGER_ENHANCE)
		TriggerSource = *(float *)CaptureControl.PtrTrig.ptrLong;
	#else
		if (CaptureControl.TriggerSignalType == WAVECAPTURE_SIG_FLOAT)
		{
			TriggerSource = *(float *)CaptureControl.PtrTrig.ptrLong;
			if (CaptureControl.PtrTrigLevel.ptrLong != 0)
			{
				CaptureControl.TriggerLevel = *(float *)CaptureControl.PtrTrigLevel.ptrLong;
			}
		}
		else if (CaptureControl.TriggerSignalType == WAVECAPTURE_SIG_UINT32)
		{
			TriggerSource = (float)(*(uint32_t *)CaptureControl.PtrTrig.ptrLong);
			if (CaptureControl.PtrTrigLevel.ptrLong != 0)
			{
				CaptureControl.TriggerLevel = (float)(*(uint32_t *)CaptureControl.PtrTrigLevel.ptrLong);
			}
		}
		else
		{
			TriggerSource = (float)(*(uint16_t *)CaptureControl.PtrTrig.ptrInt);
			if (CaptureControl.PtrTrigLevel.ptrInt != 0)
			{
				CaptureControl.TriggerLevel = (float)(*(uint16_t *)CaptureControl.PtrTrigLevel.ptrInt);
			}
		}
	#endif
	#if defined(CAPTURE_TRIGGER_ENHANCE)
		if (CaptureControl.TriggerSignalType != WAVECAPTURE_SIG_BITSET)
	#endif
		{
			if (TriggerSource > CaptureControl.TriggerLevel)
			{
				CaptureControl.TriggerRising = true;
				if (TriggerSource > CaptureControl.TriggerLevel + 1.0f)
				{
					CaptureControl.TriggerFalling = false;
				}
			}
			if (TriggerSource < CaptureControl.TriggerLevel)
			{
				CaptureControl.TriggerFalling = true;
				if (TriggerSource < CaptureControl.TriggerLevel - 1.0f)
				{
					CaptureControl.TriggerRising = false;
				}
			}
		}
	#if defined(CAPTURE_TRIGGER_ENHANCE)
		else
		{
			if ((*(uint16_t *)CaptureControl.PtrTrig.ptrLong & (1U << CaptureControl.TriggerBitNo)) != 0U)
			{
				CaptureControl.TriggerRising = true;
				CaptureControl.TriggerFalling = false;
			}
			else
			{
				CaptureControl.TriggerFalling = true;
				CaptureControl.TriggerRising = false;
			}
		}
	#endif
	}
	else
	{
		CaptureControl.TriggerRising = false;
		CaptureControl.TriggerFalling = false;
	}

	if (CaptureControl.ArmTrigger)
	{
	#if defined(CAPTURE_TRIGGER_ENHANCE)
		CaptureControl.TriggerSource = TriggerSource;
	#endif
		if (CaptureControl.TriggerSel == WAVECAPTURE_TRIG_EDGE_RISING)
		{
			if (CaptureControl.TriggerRising && !CaptureControl.TriggerRisingOld)
			{
				CaptureControl.Trigger = true;
			}
		}
		else if (CaptureControl.TriggerSel == WAVECAPTURE_TRIG_EDGE_FALLING)
		{
			if (CaptureControl.TriggerFalling && !CaptureControl.TriggerFallingOld)
			{
				CaptureControl.Trigger = true;
			}
		}
		else if (CaptureControl.TriggerSel == WAVECAPTURE_TRIG_EDGE_BOTH)
		{
			if ((CaptureControl.TriggerRising && !CaptureControl.TriggerRisingOld)
				|| (CaptureControl.TriggerFalling && !CaptureControl.TriggerFallingOld))
			{
				CaptureControl.Trigger = true;
			}
		}
		else if (CaptureControl.TriggerSel == WAVECAPTURE_TRIG_LEVEL_HIGH)
		{
			CaptureControl.Trigger = CaptureControl.TriggerRising;
		}
		else if (CaptureControl.TriggerSel == WAVECAPTURE_TRIG_LEVEL_LOW)
		{
			CaptureControl.Trigger = CaptureControl.TriggerFalling;
		}
		else
		{
			/* do nothing */
		}
		CaptureControl.ArmTrigger = !CaptureControl.Trigger; // single trigger mode
	}
	CaptureControl.TriggerRisingOld = CaptureControl.TriggerRising;
	CaptureControl.TriggerFallingOld = CaptureControl.TriggerFalling;

    if( CaptureControl.Trigger )
    {
    	/* start sampling immediately once trigger condition is met */
        if ( CaptureControl.FrequencyCount-- == 0 )
        {
		#if CAPTURE_DUAL_CHANNEL == 0x55AA
        	if (CaptureControl.Count2ndChan < CAPTUR_2ND_BUFFER_SIZE - (uint16_t)CaptureControl.LongData2ndChan)
        	{
				if (CaptureControl.LongData2ndChan)
				{
					SampleValue = *(uint32_t *)CaptureControl.Address2ndChan.ptrLong;
					*(uint32_t *)(&Capture2ndBuffer[CaptureControl.Count2ndChan]) = SampleValue;
					CaptureControl.Count2ndChan += 2;
				}
				else
				{
					SampleValue = *(uint16_t *)CaptureControl.Address2ndChan.ptrInt;
					Capture2ndBuffer[CaptureControl.Count2ndChan] = (uint16_t)SampleValue;
					CaptureControl.Count2ndChan++;
				}
        	}
		#endif
        	if (CaptureControl.Address.ptrInt != 0)
        	{
				if ( CaptureControl.LongData )
				{
					*CaptureControl.DataPtr.ptrLong = *CaptureControl.Address.ptrLong;
					++CaptureControl.DataPtr.ptrLong;
					CaptureControl.Count += 2;
				}
				else
				{
					*CaptureControl.DataPtr.ptrInt = *CaptureControl.Address.ptrInt;
					++CaptureControl.DataPtr.ptrInt;
					CaptureControl.Count++;
				}
        	}
        	else
        	{
        		*CaptureControl.DataPtr.ptrInt = 0;
        		++CaptureControl.DataPtr.ptrInt;
        		CaptureControl.Count++;
        	}
            if ( CaptureControl.Count >= (CAPTURE_SIZE*2) )
            {
                CaptureControl.Trigger = false;
                CaptureControl.Complete = true;
            }
            
            CaptureControl.FrequencyCount = CaptureControl.Frequency;
        }
    }
}

// ****************************************************************************
// *
// *  Function:   ee_calc_calibration
// *
// *  Purpose :   Receives the EEPROM Data and Address, converts to float and 
// *              stores in the calibration array. (int16_t)10000 = (float)1.0
// *
// *  Parms Passed: uint16_t* pointer to the data in eeprom
// *                EE_ID pointer to eeprom table item that called this function
// *                
// *  Returns     :  Nothing
// *
// ***************************************************************************
#pragma FUNC_EXT_CALLED;
void ee_calc_calibration( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->eep_addr )
    {
        default:
            // use e-addr to index to calibration array, check for valid address
            if ( ee->eep_addr >= CALIBRATION_EEPROM_START_ADDRESS )
            {
                uint16_t e_addr = ee->eep_addr - CALIBRATION_EEPROM_START_ADDRESS;
        
                if ( e_addr < RAW_A2D_CAL_SIZE )
                {
                    // scale by default value
                    float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT;
                    
                    if( EnableOpenLoop == 1)
                    {
                    	if(ee->eep_addr == PARAM_ADC_CH11_0_CAL)
                    	{
                    	    InvVoltScalerPhaseA = (float)1.0/tempScaler;
                    	}
                    	else if(ee->eep_addr == PARAM_ADC_CH12_0_CAL)
                    	{
                    	    InvVoltScalerPhaseB = (float)1.0/tempScaler;
                    	}
                    	else if(ee->eep_addr == PARAM_ADC_CH13_0_CAL)
                    	{
                    	    InvVoltScalerPhaseC = (float)1.0/tempScaler;
                    	}
                    }                     
                    // multiply to unit conversion and store to table
                    // don't want this to happen during the A2D interrupt
                    CriticalSection enter;
                   
                	switch(UPMSystem)
                	{

                	    case HV_20K:
                	    case HV_30K:
                	    	CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
                	    	break;

                	    case HV_40K:
                	    	CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
                	    	break;							

                	    default:
                	    	CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
                	    	break;
                	}

                }
            }
            break;
        
        // These take care of non-modifiable EEPROM scalars
        case PARAM_ADC_CH6_1_CAL:		//ad_index=26, InvCapTemperature
        case PARAM_ADC_CH6_3_CAL:		//ad_index=30, RectCapTemperature
            {
                uint16_t e_addr = ee->eep_addr - CALIBRATION_EEPROM_START_ADDRESS;
                
                CriticalSection enter;
                CalibrationScaler.w[ e_addr ] = 1.0;		
            }
            break;
			
		//
		case PARAM_CAL_BatteryVoltageCal:
			{
                uint16_t e_addr = PARAM_CAL_BatteryVoltageCal_AdcIndex;//ee->eep_addr - CALIBRATION_EEPROM_START_ADDRESS;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT;				
		        CriticalSection enter;
				switch(UPMSystem)
				{
				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						if( InterfaceBoardRevID == CONST_InterfaceBrdRev_ID_P5 )
						{
							CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K_VbatRatio.w[ e_addr ];
						}
						else
						{
							CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						}
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}				
			}
			break;	
			
		case PARAM_CAL_InputVoltageNeutralCal:
			{
				uint16_t e_addr = PARAM_CAL_InputVoltageNeutralCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
		        CriticalSection enter;
				switch(UPMSystem)
				{
				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;
				
		case PARAM_CAL_AnalogZeroCal:
			{
				uint16_t e_addr = PARAM_CAL_AnalogZeroCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;
					
		case PARAM_CAL_ChassisVoltageCal:
			{
				uint16_t e_addr = PARAM_CAL_ChassisVoltageCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;
					
		case PARAM_CAL_BatteryVoltageChgPosCal:
			{	//1.Vbatchg_p
				uint16_t e_addr = PARAM_CAL_BatteryVoltageChgPosCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
				
			{	//2.Vbat_p
				uint16_t e_addr = PARAM_CAL_BatteryVoltagePosCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}			
			break;
					
		case PARAM_CAL_BatteryVoltageChgNegCal:
			{
				uint16_t e_addr = PARAM_CAL_BatteryVoltageChgNegCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;
					
		case PARAM_CAL_LogicPower15vCal:
			{
				uint16_t e_addr = PARAM_CAL_LogicPower15vCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;	
			
		case PARAM_CAL_LogicPower24vCal:
			{
				uint16_t e_addr = PARAM_CAL_LogicPower24vCal_AdcIndex;
				// scale by default value
				float tempScaler = (float)(*data) / (float)CALIBRATION_DEFAULT; 			
				CriticalSection enter;
				switch(UPMSystem)
				{				
					case HV_20K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				
					case HV_30K:
					case HV_40K:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_40K.w[ e_addr ];
						break;							
				
					default:
						CalibrationScaler.w[ e_addr ] = tempScaler * UnitOfMeasureConversions_HV_20K.w[ e_addr ];
						break;
				}
			}
			break;				
		//end			
    }
}

// ****************************************************************************
// *
// *  Function:   RandomNumber
// *
// *  Purpose :   Generate a truely random number.
// *
// *  Parms Passed: None
// *                
// *  Returns     :  A word composed entirely of the lowest-order bit of some ADC
// *                 channels.  The hope is that there is enough white noise in each
// *                 of the channels in order to build a completely random, unbiased
// *                 sample value.
// *
// ***************************************************************************
uint16_t RandomNumber(void)
{
    uint16_t random = 0;
    // first 12 fast ADC channels
    for (int i = 0; i < NumberOfAdcFastChannels / 2; ++i)
    {
        random |= (AdcDMABuffer[0][i] & 0x1) << i;
    }
    
    // Next 4 fast ADC sample channels
    for (int i = 0; i < NumberOfAdcFastChannels / 2 && i < 4; ++i)
    {
        random |= (AdcDMABuffer[1][i+16] & 0x1) << (i + 12);
    }
    return random;
}

// Zhangjun : the following code is for FCT
uint16_t Get_ADC_Data(uint16_t addr)
{
    uint16_t adc_result;
    
    if( addr < 32 )
    {
        adc_result = AdcDMABuffer[0][addr];   
    }
    else if( addr < 55 )
    {
        adc_result = Adc_Sample.words[addr-32];
    }
    else
    {
        adc_result = 0;
    }
    
    return adc_result;         
}

// ******************************************************************************************************
// *            End of adc.cpp
// ******************************************************************************************************
