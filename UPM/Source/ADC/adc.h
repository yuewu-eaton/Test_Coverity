// ******************************************************************************************************
// *            adc.h
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
// *    FILE NAME: adc.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/19/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _ADC_H
#define _ADC_H

#include "Constants.h"

typedef struct
{
    // 'fast' A/D Mux Group 1
    float           BatteryCurrentNegative;         // A3            //0
    float           OpARef;                         // A4            //1
    float           LogicPower5v;                   // A5            //2
    stThreePhase    BypassCurrent;                  // B3, B4, B5    //3,4,5
    float           BatteryVoltageChgPos;                // A0            //6
    float           BatteryVoltageChgNeg;                // A1            //7
    float           BatteryCurrentPositive;         // A2            //8
    stThreePhase    BypassVoltage;                  // B0, B1, B2    //9,10,11

    // 'fast' A/D mux group 0
    stThreePhase    InputCurrent;                   // A3, A4, A5    //12,13,14
    stThreePhase    InverterCurrent;                // B3, B4, B5    //15,16,17
    stThreePhase    InputVoltage;                   // A0, A1, A2    //18,19,20
    stThreePhase    InverterVoltage;                // B0, B1, B2    //21,22,23

    // 'medium' A/D Mux group 0 
    float           RailVoltageNegative;            // A6             //24
    float           RailVoltagePositive;            // B6             //25
    // 'medium' A/D Mux group 1
    float           InvCapTemperature;              // A6             //26 Channel reserved for battery temperature. the NTC is not exist on HW 
    float           BypassVoltage2phA;              // B6             //27
    // 'medium' A/D Mux group 2
    float           AMBTemperature;                 // A6             //28  //AMBTemperature
    float           BypassVoltage2phB;              // B6             //29
    // 'medium' A/D Mux group 3
    float           SCRTemperature;                 // A6             //30 //RectCapTemperature
    float           BypassVoltage2phC;              // B6             //31
    
    // slow A/D 0  
//  int32_t         AnalogZero;                     // B7            
    float           InputVoltageNeutral;            // B7             //32	InputVoltageNeutral: not use as hw not correct
    float           InverterDCPhaseA;               // A7             //33
    // slow A/D 1
    float           PhaseBTemperature;                                //RectifierTemperature;          // B7             //34
    float           InverterDCPhaseB;               // A7             //35
    // slow A/D 2
    float           PhaseATemperature;           	// B7             //36
    float           InverterDCPhaseC;               // A7             //37
    // slow A/D 3
	float			BatteryTemperature;				// B7			  //38
    float           OutputVoltagePhaseA;            // A7             //39
    // slow A/D 4
    float           PhaseCTemperature;                 // B7             //40
    float           OutputVoltagePhaseB;            // A7             //41
    // slow A/D 5
    float           LogicPower15v;                  // B7             //42
    float           OutputVoltagePhaseC;            // A7             //43
    // slow A/D 6
	float			LogicPower24v; 					// B7			  //44	//Hobbit:logicpower24V
    float           BatteryVoltage;         		// A7             //45  //Hobbit: BATBKP, BatteryAll
    // slow A/D 7
    float           ChassisVoltage;					// B7             //46  //Hobbit: Chassic volt N-GROUND, it is N-phase check
    float           BatteryVoltagePos;                // A7             //47

// this is needed because of the dowmsampling of the output voltage means
// that there's no 3phase raw data struct for output voltage
    stThreePhase    OutputVoltageRaw;                                 //48,49,50
// same here
    stThreePhase    BypassVoltage2;                                   //51,52,53
// secondary battery voltage measurement, calculated from measurements to neutral
    float           BatteryVoltageNeg;                                  //54
    
    stThreePhase    InputCurrentLast;               // last current   //55,56,57
    stThreePhase    InputCurrentAvg;                // average current//58,59,60
    stThreePhase    InverterCurrentLast;            // last current   //61,62,63
    stThreePhase    InverterCurrentAvg;             // average current//64,65,66
    
    // In 9E 100KVA, Battery Pos channel is used to sample leg A current,
    // and Battery Neg channel is used to sample leg B current.
    float           BatteryCurrentNegativeSum;                         //67
    float           BatteryCurrentPositiveSum;                         //68
                                                                       
    float           ControlDebug[18];                                   //69+x //add for control debug

} stRawAdcData;

#define RAW_ADC_SIZE    sizeof( stRawAdcData ) / sizeof( float )

typedef union
{
    stRawAdcData    st;
    float           w[ RAW_ADC_SIZE ];
} uRawAdcData;      

typedef struct
{
    // 'fast' A/D Mux Group 1
    float           BatteryCurrentNegativeCal;      // A3
    float           OpARefCal;                      // A4
    float           LogicPower5vCal;                // A5
    stThreePhase    BypassCurrentCal;               // B3, B4, B5
    float           BatteryVoltage1Cal;              // A0
    float           BatteryVoltage2Cal;             // A1
    float           BatteryCurrentPositiveCal;      // A2
    stThreePhase    BypassVoltageCal;               // B0, B1, B2

    // 'fast' A/D mux group 0
    stThreePhase    InputCurrentCal;                // A3, A4, A5
    stThreePhase    InverterCurrentCal;             // B3, B4, B5
    stThreePhase    InputVoltageCal;                // A0, A1, A2
    stThreePhase    InverterVoltageCal;             // B0, B1, B2

    // 'medium' A/D Mux group 0 
    float           RailVoltageNegativeCal;         // A6
    float           RailVoltagePositiveCal;         // B6
    // 'medium' A/D Mux group 1
    float           InvCapTemperatureCal;           // A6
    float           BypassVoltage2phACal;           // B6
    // 'medium' A/D Mux group 2
	float			AMBTemperatureCal;				// A6
    float           BypassVoltage2phBCal;           // B6
    // 'medium' A/D Mux group 3
    float           SCRTemperatureCal;           // A6  RectCapTemperatureCal
    float           BypassVoltage2phCCal;           // B6
    
    // slow A/D 0 
    float           InputNeutralCal;                // B7		Hobbit:Line_N& Byp_N_afterrelay
    float           InverterDCPhaseACal;            // A7
    // slow A/D 1
    float           PhaseBTemperatureCal;                //RectifierTemperatureCal;        // B7
    float           InverterDCPhaseBCal;            // A7
    // slow A/D 2
    float           PhaseATemperatureCal;        // B7
    float           InverterDCPhaseCCal;            // A7
    // slow A/D 3
	float			BatteryTemperatureCal;		  // B7
    float           OutputVoltagePhaseACal;         // A7
    // tslow A/D 4
    float           PhaseCTemperatureCal;              // B7
    float           OutputVoltagePhaseBCal;         // A7
    // slow A/D 5
    float           LogicPower15vCal;               // B7
    float           OutputVoltagePhaseCCal;         // A7
    // slow A/D 6
    float           LogicPower24vCal;                // B7
    float           BatteryVoltageCal;      		// A7
    // slow A/D 7
	float			ChassisVoltageCal; 				// B7
    float           BatteryVoltagePosCal;             // A7

} stAdcCalibration;

#define RAW_A2D_CAL_SIZE        sizeof( stAdcCalibration ) / sizeof( float ) 

typedef union
{
    stAdcCalibration    st;
    float               w[ RAW_A2D_CAL_SIZE ];
} uAdcCalibration;

// slow A/D channels
typedef struct
{
    // Mux group 0  
    int16_t          InputVoltageNeutral;             // B7
    int16_t          InverterDCPhaseA;                // A7
    // Mux group 1
	int16_t 		 PhaseBTemperature;               // B7 Power board S phase temp
    int16_t          InverterDCPhaseB;                // A7
    // Mux group 2
	int16_t 		 PhaseATemperature;			      // B7 Power board R phase temp
    int16_t          InverterDCPhaseC;                // A7
    // Mux group 3
	int16_t 		 BatteryTemperature;			  // B7 Power board BAT temp
    int16_t          OutputVoltagePhaseA;             // A7
    // Mux group 4
    int16_t          PhaseCTemperature;               // B7 Power board T phase temp
    int16_t          OutputVoltagePhaseB;             // A7
    // Mux group 5
    int16_t          LogicPower15v;                   // B7
    int16_t          OutputVoltagePhaseC;             // A7
    // Mux group 6
    int16_t          LogicPower24v;                   // B7
    int16_t          BatteryVoltage;          // A7 
    // Mux group 7
	int16_t 		 ChassisVoltage;				  // B7
    int16_t          BatteryVoltagePos;                 // A7
} stAdcSlowDataRaw;

typedef union
{
    stAdcSlowDataRaw    st;
    uint16_t              w[ sizeof( stAdcSlowDataRaw ) ];
} uAdcSlowDataRaw;

#define CALIBRATION_DEFAULT                     (uint16_t)10000
#define CALIBRATION_EEPROM_START_ADDRESS        401

// DMA buffer indices
#define FAST_IBATT_NEG_IDX                  0
#define FAST_OPAREF_IDX                     1
#define FAST_LOGIC_5V_IDX                   2
#define FAST_BYPASS_CURRENT_A_IDX           3
#define FAST_BYPASS_CURRENT_B_IDX           4
#define FAST_BYPASS_CURRENT_C_IDX           5
#define FAST_VBATT1_IDX                     6
#define FAST_VBATT2_IDX                     7
#define FAST_IBATT_POS_IDX                  8
#define FAST_BYPASS_V_A_IDX                 9
#define FAST_BYPASS_V_B_IDX                 10
#define FAST_BYPASS_V_C_IDX                 11

#define FAST_UTIL_AMPS_A_IDX                16
#define FAST_UTIL_AMPS_B_IDX                17
#define FAST_UTIL_AMPS_C_IDX                18
#define FAST_INV_AMPS_A_IDX                 19
#define FAST_INV_AMPS_B_IDX                 20
#define FAST_INV_AMPS_C_IDX                 21
#define FAST_UTIL_VOLTS_A_IDX               22
#define FAST_UTIL_VOLTS_B_IDX               23
#define FAST_UTIL_VOLTS_C_IDX               24
#define FAST_INV_VOLTS_A_IDX                25
#define FAST_INV_VOLTS_B_IDX                26
#define FAST_INV_VOLTS_C_IDX                27  

// medium
#define MEDIUM_AMB_TEMP_IDX               	28
#define MEDIUM_BYPASS_V_B_IDX               29

#define MEDIUM_SCR_TEMP_IDX                 12 //MEDIUM_RECTCAP_TEMP_IDX
#define MEDIUM_BYPASS_V_C_IDX               13

#define MEDIUM_RAIL_NEGATIVE_IDX            28
#define MEDIUM_RAIL_POSITIVE_IDX            29

#define MEDIUM_INVCAP_TEMP_IDX              12
#define MEDIUM_BYPASS_V_A_IDX               13

// slow
#define SLOW_UTIL_VOLTS_N_IDX               30
#define SLOW_INV_DC_PHA_IDX                 31

#define SLOW_RECT_TEMP_IDX                	14
#define SLOW_INV_DC_PHB_IDX                 15

#define SLOW_INV_TEMP_IDX                	30
#define SLOW_INV_DC_PHC_IDX                 31

#define SLOW_BAT_TEMP_IDX                 	14
#define SLOW_OUT_V_PHA_IDX                  15

#define SLOW_STS_TEMP_IDX                   30
#define SLOW_OUT_V_PHB_IDX                  31

#define SLOW_LOGIC_15V_IDX                  14
#define SLOW_OUT_V_PHC_IDX                  15

#define SLOW_LOGIC_24V_IDX                  30
#define SLOW_VBATT_IDX                  	31

#define SLOW_CHASSIS_VOLTS_IDX              14
#define SLOW_VBATT_POS_IDX                  15


// global data
extern volatile const uRawAdcData * const     RawAdcDataPtr;
extern volatile const uAdcSlowDataRaw * const SlowAdcDataPtr;
extern stThreePhase						      OutputCurrent;
extern uRawAdcData RawAdcData;

//sample variables
#define CAPTURE_SIZE                        512
typedef union
{
    uint32_t  l;
    uint16_t  s[sizeof( uint32_t)];
} stCaptureData;

typedef union
{
    int32_t* ptrLong;
    int16_t* ptrInt;
} stCapturePointer;

#define WAVECAPTURE_TRIG_EDGE_RISING 0U
#define WAVECAPTURE_TRIG_EDGE_FALLING 1U
#define WAVECAPTURE_TRIG_EDGE_BOTH 2U
#define WAVECAPTURE_TRIG_LEVEL_HIGH 3U
#define WAVECAPTURE_TRIG_LEVEL_LOW 4U

/* refer to enum eMemType */
#define WAVECAPTURE_SIG_FLOAT 0U
#define WAVECAPTURE_SIG_INT16 1U
#define WAVECAPTURE_SIG_UINT16 2U
#define WAVECAPTURE_SIG_INT32 3U
#define WAVECAPTURE_SIG_UINT32 4U
#define WAVECAPTURE_SIG_BITSET 6U

/*
* uncomment following macro definition to support trigger condition such as,
* timer expiration, counter reaches certain value, status or flag change,
* comment it out only if resources such as RAM, flash, CPU reach their bottleneck
*/
#define CAPTURE_TRIGGER_ENHANCE

/*
* uncomment following macro definition to support dual channel data log,
* this is useful when two variables must be captured simultanuously, because
* they are tightly related to each other
* comment it out only if resources such as RAM, flash, CPU reach their bottleneck
*/
#define CAPTURE_DUAL_CHANNEL 0

typedef struct
{
    volatile bool       Trigger;
    volatile bool       Complete;
    volatile bool       ArmTrigger;
    volatile bool       Start;
    bool                LongData;
    volatile uint16_t   Count;
    volatile uint16_t   FrequencyCount;
    uint16_t            Frequency;
    stCapturePointer    Address;
    stCapturePointer    DataPtr;
    stCapturePointer    PtrTrig;
    float               TriggerLevel;
    uint32_t            Timeout; // one tick is one millisecond
    // uint16_t TriggerMode;
    uint16_t            TriggerSel;
    uint16_t            SourceSignalType;
    bool                TriggerRising;
    bool                TriggerFalling;
    bool                TriggerRisingOld;
    bool                TriggerFallingOld;
#if defined(CAPTURE_TRIGGER_ENHANCE)
	stCapturePointer    PtrTrigLevel;
	float               TriggerSource;
	uint16_t            TriggerBitNo;
	uint16_t            TriggerSignalType;
#endif
#if CAPTURE_DUAL_CHANNEL == 0x55AA
    stCapturePointer    Address2ndChan;
    uint16_t            SourceSignalType2ndChan;
    volatile uint16_t   Count2ndChan;
    bool            LongData2ndChan;
#endif/* end of CAPTURE_DUAL_CHANNEL */
} stCaptureType;

#if CAPTURE_DUAL_CHANNEL == 0x55AA

#define CAPTUR_2ND_BUFFER_SIZE ((uint16_t)0x200)

extern uint16_t Capture2ndBuffer[CAPTUR_2ND_BUFFER_SIZE];
#endif /* end of CAPTURE_DUAL_CHANNEL */

    // Variables for Waveform Capture
extern  stCaptureType CaptureControl;
extern volatile stCaptureData CaptureData[];
//sample variables

extern int16_t BattCurrentPosAutoZero;
extern int16_t BattCurrentNegAutoZero;

extern float EssTransferTime;
extern bool EssTimeTest;

extern float InvVoltScalerPhaseA;
extern float InvVoltScalerPhaseB;
extern float InvVoltScalerPhaseC;

// ******************************************************************************************************
// *            Global function prototypes
// ******************************************************************************************************
void ProcessSlowAdc( void );
void CurrentSensorAutoZero( void );
void BatteryCurrentAutoZero( void );
void BypassSensorAutoZero( void );
uint16_t RandomNumber(void);
uint16_t Get_ADC_Data(uint16_t addr);

#endif
// ******************************************************************************************************
// *            End of adc.h
// ******************************************************************************************************
