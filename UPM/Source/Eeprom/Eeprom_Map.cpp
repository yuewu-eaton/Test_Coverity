// ******************************************************************************************************
// *            Eeprom_Map.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: Eeprom_Map.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *    DATE: 5/09/2003
// *
// *    HISTORY: See CVS history
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Eeprom_map.h"
#include "Spi_Driver.h"
#include "Adc.h"
#include "Alarms.h"
#include "Alarms_AC.h"
#include "Abm.h"
#include "InverterControl.h"
#include "InvSync.h"
#include "BatteryStateControl.h"
#include "BypassInterface.h"
#include "RectifierStateControl.h"
#include "BTR.h"
#include "BackfeedState.h"
#include "SineReference.h"
#include "ExtSignalPLL.h"
#include "HQ_Funcs.h"
#include "Coefficients.h"
#include "Spi_Task.h"
#include "sem.h"
#include "tsk.h"
#include "Fan.h"
#include "Version.h"


// ********************************************************************************************************
// * EEPROM FUNCTION PROTOTYPES
// ********************************************************************************************************
void ee_nothing( EE_ID* ee, uint16_t* data );
void ee_ram( const EE_ID* ee, const uint16_t* data );
void ee_ram_float( const EE_ID* ee, const uint16_t* data);
void ee_calc_out_volts( const EE_ID* ee, const uint16_t* data );
void ee_calc_out_freq( const EE_ID* ee, const uint16_t* data );
void ee_calc_out_power( const EE_ID* ee, const uint16_t* data );

// ee functions for function table
extern void ee_calc_calibration( const EE_ID* ee, const uint16_t* data );
extern void ee_calc_protection( const EE_ID* ee, const uint16_t* data );
extern void ee_calc_abm( const EE_ID* ee, const uint16_t* data );
extern void ee_ac_limits( const EE_ID* ee, const uint16_t* data );
extern void ee_update_parallel( const EE_ID* ee, const uint16_t* data );
extern void ee_system_type( const EE_ID* ee, const uint16_t* data );
extern void ee_calc_logicpower_limits( const EE_ID* ee, const uint16_t* data );

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
uint16_t dummy_eep;

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
uEEStatusWordsAndBits EEStatusBits;

// section 0 data
uint16_t EEVersion;
uint16_t EERevision;
uint16_t MyUPSNumber = 0;
uint16_t MyUPMNumber = 0;
uint16_t MyHardwareNumber = 0;
uint16_t ExtSyncEnabled = 0;
uint16_t UPMCalEnabled = 0;
uint16_t ParallelForRedundancy = 0;
uint16_t NumOfUPSs = 0;
uint16_t NumOfUPMs = 0;
uint16_t NumOfModule = 1;
uint16_t NewCtrlBoard = 1;
uint16_t MOBEnabled = 0;
uint16_t DisableAutoID = 1;
SYSTEM_TYPE SystemType;

// section 1 data
uint16_t BypassWithNormalCommand;
uint16_t EPOEnabled;                    
uint16_t InternalMBSInstalled;                  
uint16_t OutNomVolts;
uint16_t OutNomFreq;
uint16_t BypVoltMaxLimit;
uint16_t BypVoltMinLimit;
uint16_t OutputPowerFactorRating = 8000;
uint16_t OutputkVARating = 300;
uint16_t OutputkWRating = 240;
uint16_t DisableImmBypassTransfer;
uint16_t ImmBypassTransferDelay;
uint16_t InputSyncEnabled = 0;
uint16_t DisableTooManyBatTrans = 1;
uint16_t ParaLoadShareErrorLimit = 20;

uint16_t BatteryNotRequiredForOnline;
uint16_t EnableAutoStandby;
uint16_t EnableAutoRestart;
uint16_t AutoRestart;
uint16_t ESSInstalled;           // Enable ESS/ECO
uint16_t ESSEnabled;
uint16_t ESSMaxNumOutages;       // Max Number of disturbances causing ESS->Online transfer before activating Lockout
uint16_t ESSOutageResetTime;     // If ESSMaxNumOutages not reached within this time, reset outage count and timer
uint16_t ESSLockoutTime;         // Time to lockout ESS after too many disturbances
uint16_t STSWshortCurrentPercent;// Percentage of nominal
uint16_t EnableOpenLoop = 0;
uint16_t ATBBypassCMDEnable = 0;
uint16_t ATBEnabled = 0;
uint16_t PowerConditionerMode = 0;
uint16_t EEBalRelayTransferTime; 
uint16_t EERelL1TransferTime;    
uint16_t EEBattRelayTransferTime;
float  DCLinkStepMax;
uint16_t SetupRequired; // Always true after section reset
uint16_t BackfeedContactorInstalled = 0;
uint16_t BatteryBreakerInstalled;   // Pan/20120914
uint16_t EE_PLLMaxPhaseError;
uint16_t EnableVFMode;
uint16_t VFModeLoadPercentMax;
uint16_t VFModeLoadPercentHysteresis;
uint16_t VFModeEntryTime;
uint16_t ESSOutputSdLowTime;
uUnit_Setup1AndBits Unit_Setup1;
uSyncConfigWordsAndBits SyncConfig;
uUnitConfigWordsAndBits UnitConfig;
uint16_t BypVoltMaxLimitFast;
uint16_t BypVoltMinLimitFast;
uint16_t BypVoltMinLimitFast_ESS;
uint16_t DeadTimeConfig;
float RecDutyCompensate = 0.015f;
float InvDutyCompensate = 0.01f;
uint16_t EnableDeadTimeCompensate;
uint16_t EnablePullChainChk;
uint16_t EEEPODelay;
uint16_t K1MaxTimeToOpen;
uint16_t BalMaxTimeToOpen;
uint16_t InputCapCompRef;
uint16_t PrechargeTime;
uint16_t PrechargeType;
uint16_t InvControlParaOption;
uint16_t PhaseLimitGain;
uint16_t MinReqkVA;
uint16_t EnableParalPwmSync = 0;
uint16_t FrequencyConverterMode;

// section 2 data
uint16_t InvHWCurrentLimitResetValue;
uint16_t RecHWCurrentLimitResetValue;
int16_t PhaseAHeatsinkTempWarningLimit;
int16_t PhaseAHeatsinkTempTripLimit;
int16_t PhaseBHeatsinkTempWarningLimit;
int16_t PhaseBHeatsinkTempTripLimit;
int16_t BatHeatsinkTempWarningLimit;
int16_t BatHeatsinkTempTripLimit;
int16_t InvCapHeatsinkTempWarningLimit;
int16_t InvCapHeatsinkTempTripLimit;
int16_t SCRHeatsinkTempWarningLimit;
int16_t SCRHeatsinkTempTripLimit;
int16_t PhaseCHeatsinkTempWarningLimit;
int16_t FanFailTempPrtDec;
int16_t RecFanFailTempPrtDec;
int16_t BatFanFailTempPrtDec;
int16_t	RecHeatsinkTempFanFull;
int16_t	InvHeatsinkTempFanFull;
int16_t	AutoECTEnable;

int16_t AmbientTempWarningLimit;
int16_t ACPreChargeRailLimit;
int16_t DCPreChargeRailLimit;
uint16_t DisableSiteWiringFault = 0;
float SiteWiringFaultHighLimit;
float BackfeedRelayFailSetting;
float BackfeedRlyFailOpenSetting;
uint16_t DisableRevisionCheck = 0;
float SelectiveTripDQLimit;
float SelectiveTripVDiffLimit;
uint16_t EnableDisChargeLink;
float DisChargeLinkVoltage;
uint16_t FanInputAmpsRMSLevel_1;
uint16_t FanInputAmpsRMSLevel_2;
uint16_t FanInputAmpsRMSLevel_3;
uint16_t FanInputAmpsRMSHysteresis;
uint16_t FuseFailLoadPercentLimit;
float UtilityLossLoadPercentLimit;
uint16_t AOLEnable;
float AolAmbTempLimit = 25.0f;
float AolAmbTempHyst_x10 = 13.0f;
uint16_t WalkinStartPercentLoad;
uint16_t KeepAlive_Delay;
uint16_t SingleUPSStartUpEnabled = 1;

// section 3 data
uint16_t BatteryAhRating;        // Battery Ah rating
uint16_t BatteryTimeRating;      // Battery time in seconds at 100% load
uint16_t CommissioningTest;
uint16_t CompensateRate;
uint16_t NumberOfExternalBatteryStrings;    // Pan/20120914
uint16_t OptimizeInputPFEnabled;            //Jacob/20130815
uint16_t MinLoadForMeterDisplay;            // Jack/20150108
uint16_t EnableTransferBatDCF;
int16_t DiffVoltInvOutRms_X10;//add ups to exist parallel system,differece volt between inv and ouput (unit:0.1V)
uint16_t BattRelayOpenCurrent;

float  EEInputHighLimit;
float  EERecHighLimit;
float  EEDClinkDelta;
float  ESSChrgCurrentOffsetCal;

// section 7 data
uint16_t ServiceRequired;  // Defaults to false.  When set, unit requires field service prior to going online.

uint16_t PreOutputKVARating = 300;  //add to record the previous OutputkVARating
bool SystemChanged = 0;             //change the variable name from OutputKVARatingChanged to SystemChanged.
bool SystemUpdated = 0;
uint16_t UPMSystem = SystemInitializing;
uint16_t PreUPMSystem = SystemInitializing;

uint16_t RelayTime = 10 * WAIT_1_MS; // Puck/21020731, add, default: 10ms for LV
uint16_t EE_BattChrgCurrentMax = 120;   // Pan/20120914 add, default:12A for 30-60KAV internal batteries only

const uint16_t HRDBattSlope_default     = ((uint16_t)int16_t( -0.694045 * 8192.0 ));  // q13
const uint16_t HRDBattIntercept_default = ((uint16_t)int16_t( 5.471134 * 1024.0 ));   // q10
const uint16_t LRDBattSlope_default     = ((uint16_t)int16_t( -0.81843 * 8192.0 ));   // q13
const uint16_t LRDBattIntercept_default = ((uint16_t)int16_t( 5.871025 * 1024.0 ));   // q10

// ****************************************************************************
// *
// *  Function        : ValidSystemType
// *
// *  Purpose         :  Defines valid SystemType
// *
// *  Parms Passed  :  None, ROM TABLE
// *
// *  Returns       :  Nothing
// *
// *  Description:     This ROM TABLE defines valid OutputKVARating.
// *
// *
// ****************************************************************************

// ****************************************************************************
// *
// *  Function: EEPROM Map
// *
// *  Purpose :    Defines the sections of memory that the EEPROM is divided into
// *
// *  Parms Passed   :   Nothing
// *
// *  Returns        :   Nothing
// *
// *  Description:    This is not a module, it is a ROM Table that defines the EEPROM Mapping.
// *
// *
// *                Each Section of the EEPROM contains:
// *
// *                        Length:  Number of words in this Section
// *
// *                        Type  :  Bitword.. Checksum Protected, No checksum Protection
// *                                           See eeprom_map.h
// *
// *
// ****************************************************************************
const EE_SECTION EE_Map[] =
{
    {  50,      CHECKSUM            },          //00  0000.. 0049,      EE_System_ID,
    { 150,      CHECKSUM            },          //01  0050.. 0199,      EE_Unit_Setup,
    { 100,      CHECKSUM            },          //02  0200.. 0299,      EE_Protection,                  
    { 100,      CHECKSUM            },          //03  0300.. 0399,      EE_Battery_Setup,
    { 100,      CHECKSUM            },          //04  0400.. 0499,      EE_Calibration,
    {1500,      QUEUE               },          //05  0500.. 1999,      EE_history_queue,
    {1500,      QUEUE               },          //06  2000.. 3499,      EE_Battery_Test_queue,
    { 100,      CHECKSUM            },          //07  3500.. 3599,      EE_Private_Setup,
    { 100,      QUEUE               },          //08  3600.. 3699,      EE_Triggered_History
    { 496,      NOT_USED_SECTION    },          //09  3700.. 4095,      EE_Not_Used1,
    {   0,      LAST_SECTION        }           // END OF SECTIONS      EE_Last_Section  
};

const uint16_t NUM_SECTIONS = (sizeof (EE_Map) / sizeof (struct EE_SECTION) );

// ****************************************************************************
// *
// *  Function        : EEPROM DEFINITIONS
// *
// *  Purpose         :  Defines the contents of the EEPROM devices
// *
// *  Parms Passed  :  None, ROM TABLE
// *
// *  Returns       :  Nothing
// *
// *  Description:     This ROM TABLE defines the default values for EEPROM LOCATIONS
// *                 a RAM Loaction to save the Data and an associated function.
// *
// *
// ****************************************************************************
EEP_PTR const Eep_Functions[] =
{
    (EEP_PTR)ee_nothing,                // EE_NOTHING,
    (EEP_PTR)ee_ram,                    // EE_RAM,
    (EEP_PTR)ee_calc_calibration,       // EE_CALIBRATION,
    (EEP_PTR)ee_calc_protection,        // EE_PROTECTIONS
    (EEP_PTR)ee_calc_abm,               // EE_ABM_DATA
    (EEP_PTR)ee_calc_BTR,               // EE_BTR_DATA
    (EEP_PTR)ee_calc_out_volts,         // EE_OUT_VOLTS
    (EEP_PTR)ee_calc_out_freq,          // EE_OUT_FREQ
    (EEP_PTR)ee_ac_limits,              // EE_AC_LIMITS
    (EEP_PTR)ee_calc_out_power,         // EE_OUT_POWER
    (EEP_PTR)ee_calc_bypass_limits,     // EE_BYPASS_LIMITS
    (EEP_PTR)ee_update_parallel,        // EE_UPDATE_PARALLEL
    (EEP_PTR)ee_calc_rectifier_limits,  // EE_UPDATE_RECTIFIER
    (EEP_PTR)ee_calc_inverter_limits,   // EE_UPDATE_INVERTER
    (EEP_PTR)ee_update_overload_limits, // EE_UPDATE_OL
    (EEP_PTR)ee_ram_float,              // EE_RAM_FLOAT
    (EEP_PTR)ee_calc_logicpower_limits, // EE_LOGIC_POWER
    (EEP_PTR)ee_update_base_sync,       // EE_BASE_SYNC
    (EEP_PTR)ee_update_bypass_sync,     // EE_BYPASS_SYNC
    (EEP_PTR)ee_system_type,            // EE_SYSTEM_TYPE
	(EEP_PTR)ee_update_output_sync, 	// EE_OUTPUT_SYNC
	(EEP_PTR)ee_update_utility_sync 	// EE_UTILITY_SYNC    
};

const EE_ID eeprom[] =
{
        //  Section 0   0   EE_System_ID
        //  address,    default,    default,    default,    ram_ptr,    size,   function,    parameter number
    //  {   0,  	&unit_id    &unit_id
        {   1,  	CURRENT_EEMAP_VERSION_HV,   CURRENT_EEMAP_VERSION_HV,   CURRENT_EEMAP_VERSION_HV,  &EEVersion,     1,  EE_RAM, PARAM_NULL  },
        {   2,  	CURRENT_EEMAP_REVISION,     CURRENT_EEMAP_REVISION,     CURRENT_EEMAP_REVISION,    &EERevision,    1,  EE_RAM, PARAM_NULL  },
        {   3,  	0,  		0,          0,      &MyUPSNumber,   				1,  EE_UPDATE_PARALLEL, 	PARAM_MyUPSNumber   },
    //  {   4-5,    unused  unused  },
        {   6,  	0,  		0,          0,      &ExtSyncEnabled,    			1,  EE_RAM  |   EE_PARALLEL,    PARAM_ExtSyncEnabled },
        {   7,  	0,  		0,          0,      &UPMCalEnabled, 				1,  EE_RAM, 					PARAM_UPMCalEnabled },
        {   8,  	1,  		1,          1,      &ParallelForRedundancy, 		1,  EE_RAM  |   EE_PARALLEL,    PARAM_ParallelForRedundancy },
        {   9,  	1,  		1,          1,      &NumOfUPSs, 					1,  EE_RAM  |   EE_PARALLEL, 	PARAM_NumOfUPSs },
        {   10, 	0,  		0,          0,      &dummy_eep, 					1,  EE_NOTHING,                 PARAM_FirstTimeRun },
			//20K = 1<14 + 20=16384+20=16404; 40K = 1<14 + 20=16384+40=16424; 30K = 1<14 + 30=16384+40=16414
        {   11, 	HIGH_VOLTAGE<<VOLTAGE_LEVEL_OFFSET|UPMKVA_20K,  HIGH_VOLTAGE<<VOLTAGE_LEVEL_OFFSET|UPMKVA_40K,  HIGH_VOLTAGE<<VOLTAGE_LEVEL_OFFSET|UPMKVA_30K,  &dummy_eep, 1,  EE_SYSTEM_TYPE, PARAM_SystemType},
        {   12, 	1,  		1,          1,      &NumOfModule, 					1,  EE_RAM,                     PARAM_NumOfModule },
        {   13, 	1,  		1,          1,      &NewCtrlBoard, 					1,  EE_RAM,                     PARAM_NewCtrlBoard },
        //  {   14-48   unused  unused
    //  {   49, 	checksum    checksum

        //  Section 1   1   Unit    EE_Unit_Setup
        //  address,    default,    default,    default,    ram_ptr,    size,   function,    parameter number
    //  {   50, 	unit_id unit_id
//	        {   51, 	1,  		1,  	&BypassWithNormalCommand,   		1,  EE_RAM  |   EE_PARALLEL,    PARAM_BypassWithNormal  },
		{	51, 	0,			0,          0,      &BypassWithNormalCommand,			1,	EE_RAM	|	EE_PARALLEL,	PARAM_BypassWithNormal	},
        {   52, 	0,  		0,          0,      &dummy_eep, 						1,  EE_NOTHING, PARAM_NULL  },
        {   53, 	2300,   	2300,       2300,   &OutNomVolts,   					1,  EE_OUT_VOLTS    |   EE_PARALLEL,    PARAM_OutNomVolts   },
        {   54, 	50, 		50,         50,     &OutNomFreq,    					1,  EE_UPDATE_INVERTER  |   EE_PARALLEL,    PARAM_OutFrequency  },
        {   55, 	15, 		15,         15,     &dummy_eep,/*&BypVoltMaxLimit,*/    1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_BypACOVLevel  },
        {   56, 	15, 		15,         15,     &dummy_eep,/*&BypVoltMinLimit,*/    1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_BypACUVLevel  },
        {   57, 	4,  		4,          4,      &dummy_eep,/*&BypFreqDeviation,*/   1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_InvSyncLim    },
//	        {   58, 	0,  		0,  	&BatteryNotRequiredForOnline,   	1,  EE_RAM  |   EE_PARALLEL,    PARAM_BattNotRequired   },
		{	58, 	1,			1,          1,      &BatteryNotRequiredForOnline,		1,	EE_RAM	|	EE_PARALLEL,	PARAM_BattNotRequired	},
        {   59, 	0,  		0,          0,      &EnableAutoStandby, 				1,  EE_RAM, PARAM_EnableAutoStandby },
        {   60, 	0,  		0,          0,      &EnableAutoRestart, 				1,  EE_RAM  |   EE_PARALLEL,    PARAM_EnableAutoRestart },
        {   61, 	0,  		0,          0,      &AutoRestart,   					1,  EE_RAM, PARAM_AutoRestart   },
        {   62, 	0,  		0,          0,      &ESSInstalled,  					1,  EE_RAM  |   EE_PARALLEL,    PARAM_ESSInstalled  },
        {   63, 	3,  		3,          3,      &ESSMaxNumOutages,  				1,  EE_RAM  |   EE_PARALLEL,    PARAM_ESSMaxNumOutages  },
        {   64, 	60, 		60,         60,     &ESSOutageResetTime,    			1,  EE_RAM  |   EE_PARALLEL,    PARAM_ESSOutageResetTime    },
        {   65, 	60, 		60,         60,     &ESSLockoutTime,    				1,  EE_RAM  |   EE_PARALLEL,    PARAM_ESSLockoutTime    },
        {   66, 	250,    	250,        250,    &STSWshortCurrentPercent,   		1,  EE_RAM, PARAM_STSWshortCurrentPercent   },
        {   67, 	0,  		0,          0,      &EPOEnabled,    					1,  EE_RAM, PARAM_EPOEnabled    },
        {   68, 	0,  		0,          0,      &InternalMBSInstalled,  			1,  EE_RAM, PARAM_MBSInstalled  },
//	        {   69, 	1,  		1,  	&SetupRequired, 					1,  EE_RAM, PARAM_SetupRequired },
		{	69, 	0,			0,          0,      &SetupRequired, 					1,	EE_RAM, PARAM_SetupRequired },
//	        {   70, 	0,  		0,  	&InputSyncEnabled,  				1,  EE_RAM, PARAM_InputSyncEnabled  },
//			{	70, 	1,			1,		&InputSyncEnabled,					1,	EE_RAM, PARAM_InputSyncEnabled	},
		{	70, 	1,			1,          1,      &InputSyncEnabled,					1,	EE_RAM  |   EE_PARALLEL, PARAM_InputSyncEnabled	},	
        {   71, 	0,  		0,          0,      &BackfeedContactorInstalled,    	1,  EE_RAM, PARAM_BackfeedContactorInstalled    },
        {   72, 	0,  		0,          0,      &BatteryBreakerInstalled,           1,  EE_RAM  |   EE_PARALLEL,    PARAM_InternalBatteryBreakerInstalled   },
        {   73, 	0,  		0,          0,      &EnableOpenLoop,    				1,  EE_RAM  |   EE_PARALLEL,    PARAM_EnableOpenLoop    },
        {   74, 	1,  		1,          1,      &ATBEnabled,    					1,  EE_RAM  |   EE_PARALLEL,    PARAM_ATBEnabled    },
//	        {   75, 	50, 		50, 	&dummy_eep, /*atb_gap_cycles*/  	1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_ATBFixedDelay },
		{	75, 	10, 		10,         10,     &dummy_eep, /*atb_gap_cycles*/		1,	EE_BYPASS_LIMITS	|	EE_PARALLEL,	PARAM_ATBFixedDelay },
        {   76, 	1,  		1,          1,      &ATBBypassCMDEnable,    			1,  EE_RAM  |   EE_PARALLEL,    PARAM_ATBBypassCMDEnable    },
        {   77, 	0,  		0,          0,      &PowerConditionerMode,  			1,  EE_RAM  |   EE_PARALLEL,    PARAM_PowerConditionerMode  },
        {   78, 	1,  		1,          1,      &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_ECTSineRefForwardEnabled  },
//	        {   79, 	0,  		0,  	&dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_ECTRectifier_Sine_Ref_Gain    },
		{	79, 	50,			50,         50,     &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_ECTRectifier_Sine_Ref_Gain	},
//	        {   80, 	9000,   	9000,   &OutputPowerFactorRating,   		1,  EE_OUT_POWER    |   EE_PARALLEL,    PARAM_OutPF },
		{	80, 	9000,		9000,      9000,    &OutputPowerFactorRating,			1,	EE_OUT_POWER	|	EE_PARALLEL,	PARAM_OutPF },
    //  {   81, 	unused  unused  },
        {   82, 	53, 		100,        100,    &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_InverterShortCurrentLimit },
        {   83, 	5,  		5,          5,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_InverterShortCurrentLimitCycles   },
        {   84, 	0,  		0,          0,      &DisableImmBypassTransfer,  		1,  EE_RAM  |   EE_PARALLEL,    PARAM_DisableImmBypassTransfer  },
        {   85, 	3000,   	3000,       3000,   &ImmBypassTransferDelay,    		1,  EE_RAM  |   EE_PARALLEL,    PARAM_ImmBypassTransferDelay    },
        {   86, 	1,  		1,          1,      &DisableTooManyBatTrans,    		1,  EE_RAM, PARAM_DisableTooManyBatTrans    },
        {   87, 	20, 		20,         20,     &ParaLoadShareErrorLimit,   		1,  EE_RAM, PARAM_ParallelLoadShareErrorLimit   },
        {   88, 	0,  		0,          0,      &EnableVFMode,  					1,  EE_RAM  |   EE_PARALLEL,    PARAM_EnableVFMode  },
        {   89, 	70, 		70,         70,     &VFModeLoadPercentMax,  			1,  EE_RAM  |   EE_PARALLEL,    PARAM_VFModeLoadPercentMax  },
        {   90, 	5,  		5,          5,      &VFModeLoadPercentHysteresis,   	1,  EE_RAM  |   EE_PARALLEL,    PARAM_VFModeLoadPercentHysteresis   },
        {   91, 	600,    	600,        600,    &VFModeEntryTime,   				1,  EE_RAM, PARAM_VFModeEntryTime   },
        {   92, 	10, 		10,         10,     &dummy_eep, 						1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_BypACOVFastLevel  },
        {   93, 	25, 		25,         25,     &dummy_eep, 						1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_BypACUVFastLevel  },
        {   94, 	40, 		40,         40,     &dummy_eep,/*&OpACUVActiveTime,*/   1,  EE_AC_LIMITS,   PARAM_OpACUVActiveTime  },  //milliseconds
    //  {95,    reserved    for for 3level  },
        {   96, 	5,  		5,          5,      &dummy_eep,/*&ESSOutputSdLowTime,*/ 1,  EE_RAM, PARAM_ESSOutputSdLowTime    },  //milliseconds
        {   97, 	0,  		0,          0,      &Unit_Setup1,   					1,  EE_RAM  |   EE_PARALLEL,    PARAM_Unit_Setup1   },
    //  {98-99, reserved    for for 400-500k    },
        {   100,    10000,  	10000,      10000,  &EEBalRelayTransferTime,    		1,  EE_RAM, PARAM_Bal_Relay_Time    },
//      {   101,    6500,       6500,       6500,   &EERelL1TransferTime,               1,  EE_RAM, PARAM_L1_Relay_Time },
        {   101,    6500,       11000,      11000,  &EERelL1TransferTime,               1,  EE_RAM, PARAM_L1_Relay_Time },
        {   102,    10000,  	10000,      10000,  &EEBattRelayTransferTime,   		1,  EE_RAM, PARAM_Batt_Relay_Time   },
        {   103,    5,  		5,          5,      &DCLinkStepMax, 					1,  EE_RAM_FLOAT,   PARAM_DCLinkStepMax },
        {   104,    400,    	400,        400,    &EEInputHighLimit,  				1,  EE_RAM_FLOAT,   PARAM_InputHighLimit    },
        {   105,    400,    	400,        400,    &EERecHighLimit,    				1,  EE_RAM_FLOAT,   PARAM_RecHighLimit  },
//	        {   106,    4,  		9,  	&dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectiferCurrentMargin },
		{	106,	2,			3,          3,      &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_RectiferCurrentMargin },
        {   107,    20, 		20,         20,     &dummy_eep, 						1,  EE_RAM, PARAM_K1MaxTimeToOpen   },
        {   108,    20, 		20,         20,     &dummy_eep, 						1,  EE_RAM, PARAM_BalMaxTimeToOpen  },
        {   109,    70, 		70,         70,     &dummy_eep, 						1,  EE_AC_LIMITS    |   EE_PARALLEL,    PARAM_OpACUVFastLevel   },
        {   110,    20, 		20,         20,     &dummy_eep, 						1,  EE_AC_LIMITS    |   EE_PARALLEL,    PARAM_OpACUVFastMinLevel    },
        {   111,    10, 		10,         10,     &dummy_eep, 						1,  EE_BYPASS_LIMITS    |   EE_PARALLEL,    PARAM_ESSBypACUVFastLevel   },
        {   112,    20, 		20,         20,     &dummy_eep,/*&BypACUVActiveTime,*/  1,  EE_BYPASS_LIMITS,   PARAM_BypACUVActiveTime },  //milliseconds
        {   113,    20, 		20,         20,     &EEDClinkDelta, 					1,  EE_RAM_FLOAT,   PARAM_DClinkDelta   },
        {   114,    15,         15,         15,     &DCPreChargeRailLimit,              1,  EE_RAM, PARAM_DCPreChargeRailLimit  },
        {   115,    1,  		1,          1,      &PrechargeType, 					1,  EE_RAM, PARAM_PrechargeType },
//	        {   116,    19, 		19, 	&PrechargeTime, 					1,  EE_RAM, PARAM_PrechargeTime},
		{	116,	60, 		60,         60,     &PrechargeTime, 					1,	EE_RAM, PARAM_PrechargeTime},
        {   117,    2400,   	3800,       3800,   &InputCapCompRef,   				1,  EE_RAM, PARAM_InputCapCompRef},
        {   118,    10, 		10,         10,     &dummy_eep,/*&NegPowerLimit,*/  	1,  EE_AC_LIMITS,   PARAM_NegPowerLimit},
        {   119,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_SineRefForwardEnabled},
//	        {   120,    720,    	730,    &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_Rectifer_DCLink_Set   },
		{	120,	730,		730,        730,    &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_Rectifer_DCLink_Set	},
//	        {   121,    60, 		120, 	&dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierCurrentMax   },
//	        {   122,    60, 		120, 	&dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierGenCurrentMax},
//			{	121,	38, 		76,		&dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_RectifierCurrentMax	},
//			{	122,	38, 		76,		&dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_RectifierGenCurrentMax},
		{	121,	36, 		72,         54,     &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_RectifierCurrentMax	},
		{	122,	36, 		72,         54,     &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_RectifierGenCurrentMax},
        {   123,    4,  		8,          6,      &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierWalkinRate   },
        {   124,    5,  		5,          5,      &dummy_eep, 						1,  EE_BYPASS_SYNC  |   EE_PARALLEL,    Param_OutputPLLMaxPhaseError    },
		{	125,	25, 		25,         25,     &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_Rectifier_Sine_Ref_Gain	},
        {   126,    0,  		0,          0,      &OptimizeInputPFEnabled,    		1,  EE_RAM, PARAM_OptimizeInputPFEnabled},
        {   127,    27, 		27,         27,     &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_UtilityPLLAngleOffset },
        {   128,    99, 		99,         99,     &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_SineRefGain   },
        {   129,    5,  		5,          5,      &MinLoadForMeterDisplay,    		1,  EE_RAM, PARAM_MinLoadForMeterDisplay},
        {   130,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierWalkinDelay  },
        {   131,    5,  		5,          5,      &dummy_eep, 						1,  EE_BYPASS_SYNC  |   EE_PARALLEL,    PARAM_BypassPLLMaxPhaseError},
        {   132,    100,    	100,        100,    &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierVoltageLoopGain  },
        {   133,    100,    	100,        100,    &dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_RectifierCurrentLoopGain  },
        {   134,    47, 		47,         47,     &dummy_eep, 						1,  EE_BYPASS_SYNC  |   EE_PARALLEL,    PARAM_BypassPLLMinDQ},
        {   135,    0,  		0,          0,      &EnableTransferBatDCF,  			1,  EE_RAM, PARAM_EnableTransferBatDCF},
    //  {136-138    reserved    for for 3level},
        {   139,    0,  		0,          0,      &InvControlParaOption,  			1,  EE_RAM  |   EE_PARALLEL,    PARAM_InvControlParaOption},
        {   140,    3,  		3,          3,      &dummy_eep, 						1,  EE_UPDATE_INVERTER  |   EE_PARALLEL,    PARAM_InverterSlewRate  },
        {   141,    70, 		133,        100,    &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_InverterSWCurrentLim  },
        {   142,    900,    	900,        900,    &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_InverterUVSetLevel    },
        {   143,    918,    	918,        918,    &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_InverterUVClearLevel  },
//	        {   144,    0,  		0,  	&dummy_eep, 						1,  EE_BYPASS_SYNC, PARAM_BypassPhaseOffset },
//	        {   145,    0,  		0,  	&dummy_eep, 						1,  EE_NOTHING, PARAM_UtilityPhaseOffset    },
//	        {   146,    0,  		0,  	&dummy_eep, 						1,  EE_NOTHING, PARAM_OutputPhaseOffset },
//	        {   147,    0,  		0,  	&dummy_eep, 						1,  EE_BASE_SYNC,   PARAM_BasePhaseOffset   },
		{	144,	600,		850,        850,    &dummy_eep, 						1,	EE_BYPASS_SYNC, PARAM_BypassPhaseOffset },
		{	145,	600,		850,        850,    &dummy_eep, 						1,	EE_UTILITY_SYNC, PARAM_UtilityPhaseOffset	},
		{	146,	600,		850,        850,    &dummy_eep, 						1,	EE_OUTPUT_SYNC, PARAM_OutputPhaseOffset },
		{	147,	600,		850,        850,    &dummy_eep, 						1,	EE_BASE_SYNC,	PARAM_BasePhaseOffset	},
        {   148,    0,  		0,          0,      &dummy_eep, 						1,  EE_BYPASS_SYNC, PARAM_BypassLoadSharePhaseCal   },
        {   149,    0,  		0,          0,      &dummy_eep, 						1,  EE_NOTHING, PARAM_UtilityLoadSharePhaseCal  },
        {   150,    0,  		0,          0,      &dummy_eep, 						1,  EE_NOTHING, PARAM_OutputLoadSharePhaseCal   },
        {   151,    0,  		0,          0,      &dummy_eep, 						1,  EE_BASE_SYNC,   PARAM_BaseLoadSharePhaseCal },
        {   152,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_LoadShareVoltageCalA  },
        {   153,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_LoadShareVoltageCalB  },
        {   154,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_LoadShareVoltageCalC  },
        {   155,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_DCcomponentOffsetA    },
        {   156,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_DCcomponentOffsetB    },
        {   157,    0,  		0,          0,      &dummy_eep, 						1,  EE_UPDATE_INVERTER, PARAM_DCcomponentOffsetC    },
//	        {   158,    1,  		1,  	&SyncConfig,    					1,  EE_RAM  |   EE_PARALLEL,    PARAM_SyncConfiguration },
		{	158,	0,			0,          0,      &SyncConfig,						1,	EE_RAM	|	EE_PARALLEL,	PARAM_SyncConfiguration },
//	        {   159,    1,  		1,  	&UnitConfig,    					1,  EE_RAM  |   EE_PARALLEL,    PARAM_UnitConfiguration },
		{	159,	0,			0,          0,      &UnitConfig,						1,	EE_RAM	|	EE_PARALLEL,	PARAM_UnitConfiguration },
        {   160,    1,  		1,          1,      &dummy_eep, 						1,  EE_NOTHING, PARAM_DaylightSavingsTime   },
        {   161,    0xffffu,    0xffffu,    0xffffu,&HistoryTrigger,    				1,  EE_RAM, PARAM_HistoryTrigger    },
        {   162,    1,  		1,          1,      &DeadTimeConfig,    				1,  EE_RAM, PARAM_DeadTimeConfig    },
//	        {   163,    15, 		15, 	&dummy_eep, 						1,  EE_UPDATE_RECTIFIER,    PARAM_DCLinkRampCurrent},
		{	163,	2, 			4,          3,      &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_DCLinkRampCurrent},
    //  {164    reserved    for for 3level},
        {   165,    1,  		1,          1,      &EnableDeadTimeCompensate,  		1,  EE_RAM, PARAM_EnableDeadTimeCompensate  },
    //  {166    reserved    for for 3level},
        {   167,    1,  		1,          1,      &EnablePullChainChk,    			1,  EE_RAM, PARAM_EnablePullChainChk    },
        {   168,    0,  		0,          0,      &EEEPODelay,    					1,  EE_RAM, PARAM_EEEPODelay    },
        {   169,    0,          0,          0,      &MinReqkVA,                         1,  EE_RAM | EE_PARALLEL, PARAM_MinReqkVA },
		{	170,	100,		100,        100,    &dummy_eep, 						1,	EE_UPDATE_INVERTER,  PARAM_InverterVoltLoopGain },
		{	171,	100,		100,        100,    &dummy_eep, 						1,	EE_UPDATE_RECTIFIER, PARAM_BatChargeCurrLoopGain },
		{	172,	100,		100,        100,    &dummy_eep, 						1,	EE_UPDATE_INVERTER , PARAM_InvEctCurrLoopGain },
		{	173,	0,			0,          0,      &dummy_eep, 						1,	EE_RAM, 			 PARAM_DriverFaultMaxNumOutages },
		{	174,	1,			1,          1,      &EnableParalPwmSync, 				1,	EE_RAM | EE_PARALLEL, PARAM_EnableParalPwmSync	},
        {	175,	0,  		0,  		0,		&FrequencyConverterMode,			1,	EE_RAM | EE_PARALLEL, PARAM_FrequencyConverterMode	},
		{	176,	100,		100,        100,    &dummy_eep, 						1,	EE_UPDATE_INVERTER,  PARAM_InverterCurrLoopGain },
        {   177, 	0,  		0,          0,      &dummy_eep, 					    1,  EE_UPDATE_RECTIFIER, PARAM_RecThdiImprove },
        //  {178-191    unused},
        {   192, 	19,  		19,         19,     &dummy_eep, 					    1,  EE_UPDATE_INVERTER, PARAM_InvDutyCompensation },
        {   193, 	100,  		100,        100,    &dummy_eep, 					    1,  EE_UPDATE_INVERTER, PARAM_FeedForwardGain },
        {   194, 	0,  		0,          0,      &dummy_eep, 					    1,  EE_UPDATE_INVERTER, PARAM_DelayCount },
        //  {194-198    unused},
        //  {199,   checksum
        //  Section 2   2   EE_Protection
        //  address,    default,    default,    default,    ram_ptr,    size,   function,    parameter number
    //  {200,   &id
        {   201,    440,    	440,        440,    &dummy_eep, /*AbsDCOVSet*/  		1,  EE_PROTECTIONS, PARAM_AbsDCOVSet    },
//	        {   202,    250,    	250,    &dummy_eep, /*AbsDCUVSet*/  		1,  EE_PROTECTIONS, PARAM_AbsDCUVSet    },
		{	202,	280,		280,        280,    &dummy_eep, /*AbsDCUVSet*/			1,	EE_PROTECTIONS, PARAM_AbsDCUVSet	},
//	        {   203,    8,  		8,  	&MaxInverterHWCLCycles, 			1,  EE_RAM, PARAM_CurrLimOffMaxInv  },
//	        {   204,    20, 		20, 	&MaxRectifierHWCLCycles,    		1,  EE_RAM, PARAM_CurrLimOffMaxRect },
//	        {   205,    20, 		20, 	&MaxBatteryHWCLCycles,  			1,  EE_RAM, PARAM_CurrLimOffMaxBatt },
		{	203,	10,			10,         10,     &MaxInverterHWCLCycles, 			1,	EE_RAM, PARAM_CurrLimOffMaxInv	},
		{	204,	10, 		10,         10,     &MaxRectifierHWCLCycles,			1,	EE_RAM, PARAM_CurrLimOffMaxRect },
		{	205,	15, 		15,         15,     &MaxBatteryHWCLCycles,				1,	EE_RAM, PARAM_CurrLimOffMaxBatt },
        {   206,    105,        105,        105,    &dummy_eep,/*OverloadLevel1*/       1,  EE_UPDATE_OL,   PARAM_OvrldLowPercent   },
        {   207,    3600,   	3600,       3600,   &OverloadTime1, 					1,  EE_RAM, PARAM_OvrldLowTime  },  //  seconds
        {   208,    111,    	111,        111,    &dummy_eep,/*OverloadLevel2*/   	1,  EE_UPDATE_OL,   PARAM_OvrldMedPercent   },
        {   209,    600,    	600,        600,    &OverloadTime2, 					1,  EE_RAM, PARAM_OvrldMedTime  },  //  seconds
        {   210,    126,    	126,        126,    &dummy_eep,/*OverloadLevel3VA_L*/   1,  EE_UPDATE_OL,   PARAM_OvrldHiPercent    },
        {   211,    60000,      60000,      60000,  &OverloadTime3,                     1,  EE_RAM, PARAM_OvrldHiTime   },  //  milliseconds
//	        {   212,    151,    	151,    &dummy_eep,/*OverloadLevel4VA_L*/   1,  EE_UPDATE_OL,   PARAM_OvrldXtrmPercent  },
		{	212,	150,		150,        150,    &dummy_eep,/*OverloadLevel4VA_L*/	1,	EE_UPDATE_OL,	PARAM_OvrldXtrmPercent	},
//	        {   213,    500,    	500,    &OverloadTime4, 					1,  EE_RAM, PARAM_OvrldXtrmTime },  //  milliseconds
		{	213,	150,		150,        150,    &OverloadTime4, 					1,	EE_RAM, PARAM_OvrldXtrmTime },	//	milliseconds
        {   214,    276,        276,        276,    &dummy_eep,/*&UtilVoltMaxLimit,*/   1,  EE_AC_LIMITS,   PARAM_InpACOVLevel  },
        {   215,    115,        115,        115,    &dummy_eep,/*&UtilVoltMinLimit,*/   1,  EE_AC_LIMITS,   PARAM_InpACUVLevel  },
//	        {   216,    80, 		155,    &dummy_eep,/*&InvHWCurrentLimit,*/  1,  EE_UPDATE_INVERTER, PARAM_Inverter_Current_Limit_Set    },
//	        {   217,    73, 		124,    &dummy_eep,/*&RectHWCurrentLimit,*/ 1,  EE_UPDATE_RECTIFIER,    PARAM_Rectifier_Current_Limit_Set   },
//	        {   218,    88, 		180,    &dummy_eep,/*&BattHWCurrentLimit,*/ 1,  EE_PROTECTIONS, PARAM_Battery_Current_Limit_Set },
		//after hw dvt definition
		{	216,	95, 		190,        135,    &dummy_eep,/*&InvHWCurrentLimit,*/	1,	EE_UPDATE_INVERTER, PARAM_Inverter_Current_Limit_Set	},
		{	217,	90, 		190,        135,    &dummy_eep,/*&RectHWCurrentLimit,*/ 1,	EE_UPDATE_RECTIFIER,	PARAM_Rectifier_Current_Limit_Set	},
        {   218,    95, 		180,        135,    &dummy_eep,/*&BattHWCurrentLimit,*/ 1,  EE_PROTECTIONS, PARAM_Battery_Current_Limit_Set },
		//for 40k new
		{	219,	100, 		100,        100,    &PhaseAHeatsinkTempWarningLimit,	1,	EE_RAM, PARAM_PhaseAHeatsinkOTWarning },
		{	220,	103, 		103,        103,    &PhaseAHeatsinkTempTripLimit, 		1,	EE_RAM, PARAM_PhaseAHeatsinkOTTrip	},
		{	221,	40, 		40,         40,     &AmbientTempWarningLimit,			1,	EE_RAM, PARAM_AmbientTempLimit	},
        {   222,    15, 		15,         15,     &dummy_eep, 						1,  EE_LOGIC_POWER, PARAM_LogicPower5v  },
        {   223,    15, 		15,         15,     &dummy_eep, 						1,  EE_LOGIC_POWER, PARAM_LogicPower15v },
        {   224,    40, 		40,         40,     &dummy_eep, 						1,  EE_AC_LIMITS,   PARAM_InputFreqLowLimit },
        {   225,    72, 		72,         72,     &dummy_eep, 						1,  EE_AC_LIMITS,   PARAM_InputFreqHighLimit    },
        {   226,    20,         20,         20,     &ACPreChargeRailLimit,              1,  EE_RAM, PARAM_ACPreChargeRailLimit  },
        {   227,    0,  		0,          0,      &DisableSiteWiringFault,    		1,  EE_RAM, PARAM_DisableSiteWiringFault    },
//	        {   228,    30, 		30, 	&SiteWiringFaultHighLimit,  		1,  EE_RAM_FLOAT,   PARAM_SiteWiringFaultHighLimit  },
		{	228,	50, 		50,         50,     &SiteWiringFaultHighLimit,			1,	EE_RAM_FLOAT,	PARAM_SiteWiringFaultHighLimit	},
        {   229,    50, 		50,         50,     &BackfeedRelayFailSetting,  		1,  EE_RAM_FLOAT,   PARAM_BackfeedRelayFailSetting  },
        {   230,    0,  		0,          0,      &ESSEnabled,    					1,  EE_RAM | EE_PARALLEL, PARAM_ESSEnabled    },
        {   231,    0,  		0,          0,      &DisableRevisionCheck,  			1,  EE_RAM, PARAM_DisableRevisionCheck  },
        {   232,    80, 		80,         80,     &SelectiveTripDQLimit,  			1,  EE_RAM_FLOAT,   PARAM_SelectiveTripDQLimit  },
        {   233,    64, 		64,         64,     &SelectiveTripVDiffLimit,   		1,  EE_RAM_FLOAT,   PARAM_SelectiveTripVDiffLimit   },
        {   234,    0,  		0,          0,      &EnableDisChargeLink,   			1,  EE_RAM, PARAM_EnableDisChargeLink   },
        {   235,    390,        390,        390,    &DisChargeLinkVoltage,              1,  EE_RAM_FLOAT,   PARAM_DisChargeLinkVoltage  },
        {   239,    11,         21,         16,     &FanInputAmpsRMSLevel_1,            1,  EE_RAM, PARAM_FanInputAmpsRMSLevel_1    },
        {   240,    22,         44,         33,     &FanInputAmpsRMSLevel_2,            1,  EE_RAM, PARAM_FanInputAmpsRMSLevel_2    },
        {   241,    27,         56,         41,     &FanInputAmpsRMSLevel_3,            1,	EE_RAM, PARAM_FanInputAmpsRMSLevel_3    },
        {   242,    2,          2,          2,      &FanInputAmpsRMSHysteresis,         1,  EE_RAM, PARAM_FanInputAmpsRMSHysteresis },
        //new 40k
        {   243,    18,         18,         18,     &FanFailTempPrtDec,                 1,  EE_RAM, PARAM_FanFailTempPrtDec},
        {   244,    70,         77,         70,     &RecHeatsinkTempFanFull,            1,  EE_RAM, PARAM_HeatsinkTempFanFull},
        {   245,    0,          0,          0,      &AutoECTEnable,                     1,  EE_RAM, PARAM_AutoEctEnable},
        {   246,    200,        200,        200,    &DiffVoltInvOutRms_X10,             1,  EE_RAM, PARAM_DiffVoltInvOutRms_X10},
		//for 40k new
		{	247,	100, 		100,        100,    &PhaseBHeatsinkTempWarningLimit,	1,	EE_RAM, PARAM_PhaseBHeatsinkOTWarning	},	//CPSDVAVE-37
		{	248,	103, 		103,        103,    &PhaseBHeatsinkTempTripLimit,		1,	EE_RAM, PARAM_PhaseBHeatsinkOTTrip },	    //CPSDVAVE-37
		{	249,	95, 		95,         95,     &BatHeatsinkTempWarningLimit,		1,	EE_RAM, PARAM_BatHeatsinkOTWarning	},	//CPSDVAVE-37
		{	250,	100, 		100,        100,    &BatHeatsinkTempTripLimit,			1,	EE_RAM, PARAM_BatHeatsinkOTTrip },      //CPSDVAVE-37
		{	251,	18,			18,         18,     &RecFanFailTempPrtDec,				1,	EE_RAM, PARAM_RecFanFailTempPrtDec	},	//CPSDVAVE-37
		{	252,	20, 		20,         20,     &BatFanFailTempPrtDec,				1,	EE_RAM, PARAM_BatFanFailTempPrtDec	},	//CPSDVAVE-37
        {   253,    70,         82,         70,     &InvHeatsinkTempFanFull,            1,  EE_RAM, PARAM_InvHeatsinkTempFanFull},  //CPSDVAVE-37
        {   254,    200,    	200,        200,    &FuseFailLoadPercentLimit,  		1,  EE_RAM, PARAM_FuseFailLoadPercentLimit  },
        {   255,    5,  		5,          5,      &UtilityLossLoadPercentLimit,   	1,  EE_RAM_FLOAT,   PARAM_UtilityLossPercentLimit   },
        {   256,    90, 		90,         90,     &BackfeedRlyFailOpenSetting,    	1,  EE_RAM_FLOAT,   PARAM_BackfeedRlyOpenFailSetting    },
    //  {257-260    reserved    for for 3level},
        {   261,    25, 		25,         25,     &AolAmbTempLimit,               	1,  EE_RAM_FLOAT/*| EE_INT_EXT_SYNC*/,  PARAM_AdapAMBTempLimit    },
        {   262,    13, 		13,         13,     &AolAmbTempHyst_x10,            	1,  EE_RAM_FLOAT/*| EE_INT_EXT_SYNC*/,  PARAM_AdapAMBTempHyst     },
        {   263,    0,  		0,          0,      &AOLEnable,                     	1,  EE_RAM/*|EE_INT_EXT_SYNC*/,         PARAM_AOLEnable           },
        {   264,    600,		600,        600,    &AdapOverloadTime1,    				1,  EE_RAM/*| EE_INT_EXT_SYNC*/,        PARAM_AdapOvrldLowTime    },//  seconds
        {   265,    60, 		60,         60,     &AdapOverloadTime2,    				1,  EE_RAM/*| EE_INT_EXT_SYNC*/,        PARAM_AdapOvrldMedTime    },//  seconds
        {   266,    10000, 		10000,      10000,  &AdapOverloadTime3, 				1,  EE_RAM/*| EE_INT_EXT_SYNC*/,        PARAM_AdapOvrldHiTime     },//  milliseconds
        {   267,    300,		300,        300,    &AdapOverloadTime4,   				1,  EE_RAM/*| EE_INT_EXT_SYNC*/,        PARAM_AdapOvrldXtrmTime   },//  milliseconds
//	        {   268,	10,			10,		&dummy_eep, 					1,		EE_UPDATE_RECTIFIER,	PARAM_UPSPowerShareBuffer  },
		{	268,	10, 		10,         10,     &dummy_eep, 						1,	EE_UPDATE_RECTIFIER,	PARAM_UPSPowerShareBuffer  },
        {   269,    3600,       5200,       5200,   &FanMaxSpeed,                       1,  EE_RAM, PARAM_MAXFanSpeed},
//	        {   270,    1000,       1000,   &FanFailErrorLimit,                 1,  EE_RAM, PARAM_FanFailLimt  },
        {   270,    60000,      60000,      60000,  &FanFailErrorLimit,                 1,  EE_RAM, PARAM_FanFailLimt  },
//	        {   271,    30, 		30, 	&WalkinStartPercentLoad,  			1,  EE_RAM, PARAM_WalkinStartPercentLoad  },
		{	271,	40, 		40,         40,     &WalkinStartPercentLoad,			1,	EE_RAM, PARAM_WalkinStartPercentLoad  },
//			{	272,	65, 		65, 	&StsHeatsinkTempWarningLimit,		1,	EE_RAM, PARAM_StsHeatsinkOTWarning  },
	//new for 20~40
		{	272,	86, 		86,         86,     &PhaseCHeatsinkTempWarningLimit,	1,	EE_RAM, PARAM_PhaseCHeatsinkOTWarning	},
		{   273 ,	3600,		3600,       3600,   &KeepAlive_Delay,					1,	EE_RAM, PARAM_KeepAlive_Delay  },
	//new for 20~40
        {	274,	95, 		95,         95,     &SCRHeatsinkTempWarningLimit,	1,	EE_RAM, PARAM_SCRHeatsinkOTWarning  },  //HOBBIT-63
        {	275,	105, 		105,        105,    &SCRHeatsinkTempTripLimit,		1,	EE_RAM, PARAM_SCRHeatsinkOTTrip     },  //HOBBIT-63
        {	276,	95, 		95,         95,     &InvCapHeatsinkTempWarningLimit,	1,	EE_RAM, PARAM_InvCapHeatsinkOTWarning  },  //HOBBIT-63
        {	277,	105, 		105,        105,    &InvCapHeatsinkTempTripLimit,		1,	EE_RAM, PARAM_InvCapHeatsinkOTTrip     },  //HOBBIT-63
		//follow 9p, Vref-60V change to -70
//			{	278,	120,		120,	&dummy_eep, /*RelDCUVSet*/			1,	EE_PROTECTIONS, PARAM_RelDCUVSet	},
		{	278,	140,		140,        140,    &dummy_eep, /*RelDCUVSet*/			1,	EE_PROTECTIONS, PARAM_RelDCUVSet	},
     //279-280 reserved for load sync
		{   281,     0,         0,          0,      &SingleUPSStartUpEnabled,           1,  EE_RAM  |   EE_PARALLEL,     PARAM_SingleUPSStartUpEnabled},
    //  {299    checksum

        //  Section 3   3   EE_Battery_Setup
        //  address,    default,    default,    default,    ram_ptr,    size,   function,    parameter number
    //  {300    id
    //  {301,   0,  &unused,    &unused,    1,  EE_RAM, PARAM_NULL  },
//	        {   302,    0,  		0,  	&dummy_eep, 						1,  EE_ABM_DATA |   EE_PARALLEL,    PARAM_NumCells  },
		{	302,	192,		192,        192,    &dummy_eep, 						1,	EE_ABM_DATA |	EE_PARALLEL,	PARAM_NumCells	},
        {   303,    600,    	600,        600,    &dummy_eep,/*NumberOfBatteryStrings*/   1,  EE_BTR_DATA |   EE_PARALLEL,    PARAM_NumStrings    },
        {   304,    83, 		166,        124,    &dummy_eep,/*&BattChrgCurrentMax*/  1,  EE_ABM_DATA,    PARAM_Batt_ChrgCurrent  },
//	        {   305,    240,    	240,    &BattChrgTimeMax,   				1,  EE_RAM, PARAM_batChargeTMax },
		{	305,	480,		480,        480,    &BattChrgTimeMax,					1,	EE_RAM, PARAM_batChargeTMax },
        {   306,    480,    	480,        480,    &BattFloatTime, 					1,  EE_RAM, PARAM_batFloatTime  },
        {   307,    6720,   	6720,       6720,   &BattMaxRestTime,   				1,  EE_RAM, PARAM_batMaxRestT   },
        {   308,    20, 		20,         20,     &BattMaxDischTime,  				1,  EE_RAM, PARAM_batMinDischT  },
        {   309,    2400,   	2400,       2400,   &BattRestFailTime,  				1,  EE_RAM, PARAM_batRestFailT  },
        {   310,    200,    	200,        200,    &dummy_eep,/*BattMaxchrgCurrentforESS*/   1,  EE_ABM_DATA,    PARAM_batMaxChrgCurrentforESS   },
        {   311,    2300,   	2300,       2300,   &dummy_eep,/*BattABMFloatVPC*/  	1,  EE_ABM_DATA,    PARAM_Batt_ChargeV_PerCell  },
        {   312,    2280,   	2280,       2280,   &dummy_eep,/*BattConstFloatVPC*/    1,  EE_ABM_DATA,    PARAM_Batt_ConstFloatV_PerCell  },
    //  {	313,   	2300,   	2300,   &dummy_eep, &dummy_eep, },
        {   314,    2100,   	2100,       2100,   &dummy_eep,/*BattRestMinVoltage*/   1,  EE_ABM_DATA,    PARAM_batOpChrgCellV    },
        {   315,    1833,  	 	1833,       1833,   &dummy_eep,/*LowRateTestThreshold*/ 1,  EE_ABM_DATA,    PARAM_BattTestFailVLow  },
        {   316,    1810,   	1810,       1810,   &dummy_eep,/*HighRateTestThreshold*/1,  EE_ABM_DATA,    PARAM_BattTestFailVHigh },
        {   317,    5,  		5,          5,      &OnBatteryAlarmDelay,   			1,  EE_RAM, PARAM_QueOnBattery  },
        {   318,    60, 		60,         60,     &BatOVTime, 						1,  EE_ABM_DATA,    PARAM_Batt_OV_Time  },
        {   319,    1810,   	1810,       1810,   &dummy_eep,/*BatUVWrnLimVPC*/   	1,  EE_ABM_DATA,    PARAM_LowBatteryWarnLimit   },
        {   320,    1750,   	1750,       1750,   &dummy_eep,/*BatUVShtLimVPC*/   	1,  EE_ABM_DATA,    PARAM_Batt_UV_PerCell   },
        {   321,    1670,   	1670,       1670,   &dummy_eep,/*BatUVMinLimVPC*/   	1,  EE_ABM_DATA,    PARAM_Batt_UV_Absolute_PerCell  },
        {   322,    2450,   	2450,       2450,   &dummy_eep,/*BatteryOVLevelVPC*/    1,  EE_ABM_DATA,    PARAM_Batt_OV_PerCell   },
    //  {323,   unused},
        {   324,    15, 		15,         15,     &dummy_eep,/*MinInputForCharging*/  1,  EE_ABM_DATA,    PARAM_MinInputForCharging   },
        {   325,    HRDBattIntercept_default,   HRDBattIntercept_default,      HRDBattIntercept_default,    &dummy_eep,/*HRDBattTimeIntercept*/ 1,  EE_BTR_DATA,    PARAM_HRDBattTimeIntercept  },
        {   326,    HRDBattSlope_default,   	HRDBattSlope_default,          HRDBattSlope_default,        &dummy_eep,/*HRDBattTimeSlope*/ 	1,  EE_BTR_DATA,    PARAM_HRDBattTimeSlope  },
        {   327,    LRDBattIntercept_default,   LRDBattIntercept_default,      LRDBattIntercept_default,    &dummy_eep,/*LRDBattTimeIntercept*/ 1,  EE_BTR_DATA,    PARAM_LRDBattTimeIntercept  },
        {   328,    LRDBattSlope_default,   	LRDBattSlope_default,          LRDBattSlope_default,        &dummy_eep,/*LRDBattTimeSlope*/ 	1,  EE_BTR_DATA,    PARAM_LRDBattTimeSlope  },
        {   329,    1,  		1,          1,      &dummy_eep, 						1,  EE_NOTHING, PARAM_BatteryType   },
        {   330,    1810,   	1810,       1810,   &dummy_eep,/*KeepAlive_BatUVPC*/    1,  EE_ABM_DATA,    PARAM_KeepAlive_BatUVPC },
        {   331,    30, 		30,         30,     &BatteryAhRating,   				1,  EE_RAM, PARAM_BatteryAhRating   },
        {   332,    900,    	900,        900,    &BatteryTimeRating, 				1,  EE_RAM, PARAM_BatteryTimeRating },
        {   333,    0,  		0,          0,      &dummy_eep,/*WattsPerCell15Min*/    1,  EE_BTR_DATA,    PARAM_WattsPerCell15Min },
        {   334,    0,  		0,          0,      &dummy_eep,/*WattsPerCell60Min*/    1,  EE_BTR_DATA,    PARAM_WattsPerCell60Min },
        {   335,    60, 		60,         60,     &dummy_eep,/*BatteryTest1Time*/ 	1,  EE_ABM_DATA,    PARAM_BatteryTest1Time  },
        {   336,    15, 		15,         15,     &dummy_eep,/*BatteryTest2Time*/ 	1,  EE_ABM_DATA,    PARAM_BatteryTest2Time  },
        {   337,    1,  		1,          1,      &CommissioningTest, 				1,  EE_RAM, PARAM_CommissioningTest },
        {   338,    0,  		0,          0,      &CommonBattery, 					1,  EE_RAM  |   EE_PARALLEL,    PARAM_CommonBattery },
        {   339,    0,  		0,          0,      &dummy_eep, 						1,  EE_ABM_DATA,    PARAM_ABMDisabled   },
        {   340,    100,    	100,        100,    &dummy_eep,/*WattsPerCellUnits*/    1,  EE_BTR_DATA,    PARAM_WattsPerCellUnits},
        {   341,    3,  		3,          3,      &CompensateRate,    				1,  EE_RAM  |   EE_PARALLEL,    PARAM_Compensate_Rate},
    //  {342-349    unused
        {   350,    0,  		0,          0,      &NumberOfExternalBatteryStrings,    1,  EE_BTR_DATA |   EE_PARALLEL,    PARAM_NumExternalStrings    },
    //  {351-353    unused
        {   354,    1574,   	1574,       1574,   &dummy_eep, 						1,  EE_ABM_DATA,    PARAM_Battery_disconnect_VPC    },
        {   355,    5,  		5,          5,      &BattRelayOpenCurrent,  			1,  EE_RAM, PARAM_BattRelayOpenCurrent  },
        {   356,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*ESSChrgCurrentOffsetCal*/                          1,  EE_RAM,         PARAM_ESSChrgCurrentOffsetCal },
    //  {357-398    unused
    //  {399    checksum

        //  Section 4   4   EE_Calibration
        //  address,    default,    default,    default,    ram_ptr,    size,   function,    parameter number
    //  {400    &id
        {   401,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryCurrentNegativeCal,*/ 1,  EE_CALIBRATION, PARAM_ADC_CH3_1_CAL },
        {   402,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.OpARefCal,*/                 1,  EE_CALIBRATION, PARAM_ADC_CH4_1_CAL },
        {   403,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.AnalogUnused0Cal,*/          1,  EE_CALIBRATION, PARAM_ADC_CH5_1_CAL },
        {   404,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassCurrentCal.phA,*/      1,  EE_CALIBRATION, PARAM_ADC_CH8_1_CAL },
        {   405,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassCurrentCal.phB,*/      1,  EE_CALIBRATION, PARAM_ADC_CH9_1_CAL },
        {   406,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassCurrentCal.phC,*/      1,  EE_CALIBRATION, PARAM_ADC_CH10_1_CAL    },
//	        {   407,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryVoltageCal,*/ 1,  EE_CALIBRATION, PARAM_ADC_CH0_1_CAL },
		{	407,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryVoltageCal,*/         1,	EE_CALIBRATION, PARAM_CAL_BatteryVoltageCal },
//	        {   408,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputVoltageNeutralCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH1_1_CAL },
		{	408,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputVoltageNeutralCal,*/	1,	EE_CALIBRATION, PARAM_CAL_InputVoltageNeutralCal },
        {   409,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryCurrentPositiveCal*/  1,  EE_CALIBRATION, PARAM_ADC_CH2_1_CAL },
        {   410,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltageCal.phA,*/      1,  EE_CALIBRATION, PARAM_ADC_CH11_1_CAL    },
        {   411,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltageCal.phB,*/      1,  EE_CALIBRATION, PARAM_ADC_CH12_1_CAL    },
        {   412,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltageCal.phC,*/      1,  EE_CALIBRATION, PARAM_ADC_CH13_1_CAL    },
        {   413,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputCurrentCal.phA,*/       1,  EE_CALIBRATION, PARAM_ADC_CH3_0_CAL },
        {   414,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputCurrentCal.phB,*/       1,  EE_CALIBRATION, PARAM_ADC_CH4_0_CAL },
        {   415,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputCurrentCal.phC,*/       1,  EE_CALIBRATION, PARAM_ADC_CH5_0_CAL },
        {   416,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterCurrentCal.phA,*/    1,  EE_CALIBRATION, PARAM_ADC_CH8_0_CAL },
        {   417,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterCurrentCal.phB,*/    1,  EE_CALIBRATION, PARAM_ADC_CH9_0_CAL },
        {   418,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterCurrentCal.phC,*/    1,  EE_CALIBRATION, PARAM_ADC_CH10_0_CAL    },
        {   419,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputVoltageCal.phA,*/       1,  EE_CALIBRATION, PARAM_ADC_CH0_0_CAL },
        {   420,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputVoltageCal.phB,*/       1,  EE_CALIBRATION, PARAM_ADC_CH1_0_CAL },
        {   421,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InputVoltageCal.phC,*/       1,  EE_CALIBRATION, PARAM_ADC_CH2_0_CAL },
        {   422,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterVoltageCal.phA,*/    1,  EE_CALIBRATION, PARAM_ADC_CH11_0_CAL    },
        {   423,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterVoltageCal.phB,*/    1,  EE_CALIBRATION, PARAM_ADC_CH12_0_CAL    },
        {   424,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterVoltageCal.phC,*/    1,  EE_CALIBRATION, PARAM_ADC_CH13_0_CAL    },
		{   425,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.RailVoltageNegativeCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH6_0_CAL },
        {   426,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.RailVoltagePositiveCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH14_0_CAL    },
		{   427,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InvCapTemperatureCal,*/      1,  EE_CALIBRATION, PARAM_ADC_CH6_1_CAL },
        {   428,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltage2phACal,*/      1,  EE_CALIBRATION, PARAM_ADC_CH14_1_CAL    },
        {   429,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.AMBTemperatureCal,*/         1,  EE_CALIBRATION, PARAM_ADC_CH6_2_CAL },
        {   430,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltage2phBCal,*/      1,  EE_CALIBRATION, PARAM_ADC_CH14_2_CAL    },
        {   431,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.SCRTemperatureCal,*/         1,  EE_CALIBRATION, PARAM_ADC_CH6_3_CAL },
        {   432,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BypassVoltage2phCCal,*/      1,  EE_CALIBRATION, PARAM_ADC_CH14_3_CAL    },
//			{   433,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.AnalogZeroCal,*/ 1,  EE_CALIBRATION, PARAM_ADC_CH15_0_CAL    },
		{	433,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.AnalogZeroCal,*/             1,	EE_CALIBRATION, PARAM_CAL_AnalogZeroCal	},
        {   434,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterDCPhaseACal,*/       1,  EE_CALIBRATION, PARAM_ADC_CH7_0_CAL },
        {   435,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.PhaseBTemperatureCal,*/   1,  EE_CALIBRATION, PARAM_ADC_CH15_1_CAL    },
        {   436,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterDCPhaseBCal,*/       1,  EE_CALIBRATION, PARAM_ADC_CH7_1_CAL },
        {   437,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.PhaseATemperatureCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH15_2_CAL    },
        {   438,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.InverterDCPhaseCCal,*/       1,  EE_CALIBRATION, PARAM_ADC_CH7_2_CAL },
        {   439,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryTemperatureCal,*/     1,  EE_CALIBRATION, PARAM_ADC_CH15_3_CAL    },
        {   440,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.OutputVoltagePhaseACal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH7_3_CAL },
        {   441,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.PhaseCTemperatureCal,*/         1,  EE_CALIBRATION, PARAM_ADC_CH15_4_CAL    },
		{   442,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.OutputVoltagePhaseBCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH7_4_CAL },
//			{	443,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,	&dummy_eep,/*&CalibrationScaler.st.ChassisVoltageCal,*/ 1,	EE_CALIBRATION, PARAM_ADC_CH15_5_CAL	},
		{	443,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.ChassisVoltageCal,*/         1,	EE_CALIBRATION, PARAM_CAL_ChassisVoltageCal	},
        {   444,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.OutputVoltagePhaseCCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH7_5_CAL },
//			{	445,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,	&dummy_eep,/*&CalibrationScaler.st.BatteryVoltageChgPosCal*/	1,	EE_CALIBRATION, PARAM_ADC_CH15_6_CAL	},
		{	445,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryVoltageChgPosCal*/	1,	EE_CALIBRATION, PARAM_CAL_BatteryVoltageChgPosCal	},
//	        {   446,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryVoltageChgNegCal,*/ 1,  EE_CALIBRATION, PARAM_ADC_CH7_6_CAL },
		{	446,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.BatteryVoltageChgNegCal,*/   1,  EE_CALIBRATION, PARAM_CAL_BatteryVoltageChgNegCal },
		{	447,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.LogicPower15vCal,*/	        1,	EE_CALIBRATION, PARAM_CAL_LogicPower15vCal	},
        {   448,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.LogicPower24vCal,*/          1,  EE_CALIBRATION, PARAM_CAL_LogicPower24vCal },
//			{	449,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,	&dummy_eep,/*&CalibrationScaler.st.LogicPower15vCal,*/ 1,	EE_CALIBRATION, PARAM_CAL_LogicPower15vCal	},
//			{	450,	CALIBRATION_DEFAULT,	CALIBRATION_DEFAULT,	&dummy_eep,/*&CalibrationScaler.st.LogicPower24vCal*/	1,	EE_CALIBRATION, PARAM_CAL_LogicPower24vCal	},
//			//new add1:24V
//	        {   449,    CALIBRATION_DEFAULT,    CALIBRATION_DEFAULT,    &dummy_eep,/*&CalibrationScaler.st.LogicPower24vCal,*/    1,  EE_CALIBRATION, PARAM_ADC_CH15_6_CAL },
    //  {449-498    unused
    //  {499    checksum

        //  Section 5   5   EE_history_queue,   un-initialized data
    //  {500-1999

        //  Section 6   6   EE_Battery_Test_queue,  un-initialized data
    //  {2000-3499

        //  Section 7   7   EE_Private_Setup. All options related to sellable features go here.
        //  3500-3599
        //  kVA rating  rating  *10 Once    kVA upgrades    are in  place,  this    will    need    to  default
        //  to  200 200
        {   3501,   200,    	400,        300,    &OutputkVARating,   1,  EE_OUT_POWER    |   EE_PARALLEL,    PARAM_OutKVA    },
        {   3502,   0,  		0,          0,      &ServiceRequired,   1,  EE_RAM, PARAM_SvcRequired   },
        {   3503,   1,  		1,          1,      &NumOfUPMs, 		1,  EE_UPDATE_PARALLEL  |   EE_PARALLEL,    PARAM_NumOfUPMs },
        {   3504,   0,  		0,          0,      &MOBEnabled,    	1,  EE_RAM  |   EE_PARALLEL,    PARAM_MOBEnabled    },
        {   3505,   1,  		1,          1,      &DisableAutoID, 	1,  EE_RAM  |   EE_PARALLEL,    PARAM_DisableAutoID },

        //  Section 8   8   EE_Not_Used1
        //  address,    default,    default,    ram_ptr,    size,   function
    //  {3500   &id
    //  {3501-4094  unused
    //  {4095   checksum
};

#define EE_TABLE_SIZE (sizeof (eeprom)  / sizeof ( EE_ID) )
// ****************************************************************************
// *
// *  Function      :  GetEETableSize
// *
// *  Purpose         :  To find the Number of values  in eeprom[]
// *
// *  Parms Passed  :  none
// *
// *  Returns       :  The Number of Values in  eeprom_20KVA_HV[]
// *
// *  Description:     This module return the number of entries in eeprom[]. This
// *                 code segment table contains functions, defaults for EEPROm Values.
// *
// ****************************************************************************
uint16_t GetEETableSize(void)
{
    return EE_TABLE_SIZE;
}

// ****************************************************************************
// *
// *  Function      :  GetNumEESections
// *
// *  Purpose         :  To find the Number of values  in EE_Map[]
// *
// *  Parms Passed  :  none
// *
// *  Returns       :  The Number of Values in  EE_MAP[]
// *
// *  Description:     This module return the number of entries in EE_MAP[]. The number
// *                 of sections that the eeprom is divided into.
// *
// ****************************************************************************
uint16_t GetNumEESections(void)
{
    return NUM_SECTIONS;
}

// ****************************************************************************
// *
// *  Function      :  Find_Eep_Num
// *
// *  Purpose         :  To find the EEPROM address of a RAM variable
// *
// *  Parms Passed  :  pointer to variable
// *
// *  Returns       :  the EEPROM address or MAX_EE_ADDRESS
// *
// ****************************************************************************
//Jacob/20130813/Merge 120k/ eeprom -> eeprom_20KVA_HV
uint16_t Find_Eep_Num(const void* address)
{
    uint16_t index, j;
    uint16_t * ramptr;
    uint16_t returnVal = MAX_EE_NUMBER;   // invalid address

    for (index = 0; index < EE_TABLE_SIZE; index++)
    {
        for(j = 0, ramptr = (uint16_t *)eeprom[index].ram_ptr;  //Keming/edit change the
        		eeprom[index].eep_length > j;
            j++, ramptr++
           )
        {
            if ((void *)ramptr == address)
            {
                returnVal = eeprom[index].eep_addr+j;
                break;
            }
        }
    }

    return returnVal;
}

// ****************************************************************************
// *
// *  Function      :  FindEepromTableIndex
// *
// *  Purpose         : Finds the table index for an eeprom address
// *
// *  Parms Passed  :  eeprom address
// *
// *  Returns       :  table index, or EE_TABLE_SIZE if it doesn't exist
// *
// ****************************************************************************
//Jacob/20130813/Merge 120k/ eeprom -> eeprom_20KVA_HV
uint16_t FindEepromTableIndex(uint16_t eeprom_address)
{
    uint16_t index;
    uint16_t max = GetEETableSize();

    for (index = 0; index < max; index++)
    {
        if (eeprom_address >= eeprom[index].eep_addr &&
            eeprom_address < eeprom[index].eep_addr + eeprom[index].eep_length)
        {
            break;
        }
    }

    return index;
}

// ****************************************************************************
// *
// *  Function:  EepromAddressValid
// *
// *  Purpose :    Check the validity of the eeprom address and length
// *
// *  Parms Passed   :  starting Address, number of Words (Length)
// *
// *  Returns        :   True (non-zero) if GOOD, returns 0 or False if data is BAD
// *
// *  Description:     This module checks the validity of the EEPROM Address
// *                 and length.
// *
// *
// ****************************************************************************
bool EepromAddressValid(uint16_t start_address, uint16_t num_words)
{
    if ( ( ( start_address + num_words - 1) > MAX_EE_ADDRESS_W ) || 
         ( num_words > MAX_EE_LENGTH )                        || 
         ( 0 == num_words ) )
    {
        return false;
    }
    else
    {
        return true;
    }
}

// ****************************************************************************
// *
// *  Function      :  GetSectionNumber
// *
// *  Purpose       :  To find the index into EE_Map[] for the passed address
// *
// *  Parms Passed  :  start_address
// *
// *  Returns       :  The index into EE_MAP[] (section Number) corresponding to the start_address
// *
// *  Description:   This module finds the index in the EE_Map[] related to the
// *                 passed address. If the address is not found 0 is returned.
// *                 the returned value is the section number.
// *
// ****************************************************************************
uint16_t GetSectionNumber(uint16_t address)
{
    uint16_t  tmp1;       //misc temp value #1
    uint16_t  tmp2;       //misc temp value #1
    uint16_t  ee_addr;    // eeprom address
    uint16_t  result;     //return value, zero is no checksum or invalid

    result=0;
    ee_addr = 0;
    tmp2 = GetNumEESections();
    for (tmp1=0; tmp1 < tmp2; tmp1++)
    {
        ee_addr += EE_Map[tmp1].length;     //First address of the next section
        if (ee_addr > address)
        {
            result = tmp1;      //First adddress of a section is the MY_EE_ID
            break;
        }
    }
    return result;
}

// ****************************************************************************
// *
// *  Function      :  GetSectionAddress
// *
// *  Purpose       :  To get the Starting address for a EEPROM memory section
// *
// *  Parms Passed  :  section_number
// *
// *  Returns       :  MY_EE_ID address in EEPROM address range
// *
// *  Description:   This module uses the structure EE_MAP[section_number] compute the
// *                 location of the EEPROM MY_EE_ID (first address of the section) in the
// *                 eeprom.
// *
// ****************************************************************************
uint16_t GetSectionAddress(uint16_t section_number)
{
    uint16_t  address;
    uint16_t  index;

    address = 0;
    index = 0;

        // Checksum is always the first address
    while (section_number-- != 0)
    {
        address += EE_Map[index++].length;
    }

    return address;
}

// ***********************************************************************
// *
// *    FUNCTION: GetParameterEE 
// *
// *    DESCRIPTION: returns the eeprom table entry for a parameter number
// *                 if it exists
// *
// *    ARGUMENTS: parameter number
// *
// *    RETURNS: pointer to eeprom table or NULL it no such parameter exists 
// *
// ***********************************************************************
//Jacob/20130813/Merge 120k modify begin...
EE_ID* GetParameterEE( parameter_t param )
{
    EE_ID* paramID = NULL;
    
    if ( PARAM_NULL != param )
    {
        paramID = (EE_ID*)eeprom;
        for ( uint16_t idx = 0; idx < EE_TABLE_SIZE; idx++, paramID++ )
        {
            if ( paramID->paramNum == param )
            {
                break;
            }
        }            
    }
    
    return paramID;
}

// ***********************************************************************
// *
// *    FUNCTION: ee_nothing 
// *
// *    DESCRIPTION: dummy function for eeprom variables with no processing 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_nothing( EE_ID* ee, uint16_t* data )
{
    (void)ee;
    (void)data;
}

// ***********************************************************************
// *
// *    FUNCTION: ee_ram 
// *
// *    DESCRIPTION: copies data as-is from ee to RAM
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_ram( const EE_ID* ee, const uint16_t* data )
{
    uint16_t  idx;
    uint16_t* dest = (uint16_t*)ee->ram_ptr;

    for ( idx = 0; idx < ee->eep_length; idx++ )
    {
        *(dest + idx) = *(data + idx);
    }
    
    //add for HV system to set different low battery warning base on battery cells
    switch ( ee->paramNum )
    {
        case PARAM_L1_Relay_Time:
            RelayTime = (uint16_t)(((float)EERelL1TransferTime / 1000.0f) * WAIT_1_MS);
            break;
            
        //change battery charge current to 20A when BatteryBreakerInstalled
        case PARAM_InternalBatteryBreakerInstalled:
            // NumberOfExternalBatteryStrings != 0 is also needed.
            if( BatteryBreakerInstalled && NumberOfExternalBatteryStrings )
            {
                EE_BattChrgCurrentMax = 200;
            }
            else
            {
                EE_BattChrgCurrentMax = 120;
            }
            break;
             
        case PARAM_VFModeLoadPercentMax:
            if( VFModeLoadPercentMax > 70 )
            {
                VFModeLoadPercentMax = 70;
            }
            else if( VFModeLoadPercentMax < 55 )
            {
                VFModeLoadPercentMax = 55;
            }

            break;

        case PARAM_VFModeLoadPercentHysteresis:
            if( VFModeLoadPercentHysteresis < 2 )
            {
                VFModeLoadPercentHysteresis = 2;
            }
            else if ( VFModeLoadPercentHysteresis > 5 )
            {
                VFModeLoadPercentHysteresis = 5;
            }
            break;
		/***********************************************************************
		/////////////////////EEP162&EEP165 set/////////////////////////////////
		//CPSDVAVE 40K/80K, 
		// 1.EEP162 = 1,EEP165=1 to meet THDI/THDV and safety configure
		// 2.EEP162 = 0 OR 2,EEP165=1 to get bettery THDI/THDV
		// 3.EEP162 = 0,EEP165=1 to get safety configure
		***********************************************************************/
		case PARAM_DeadTimeConfig:
			if( CfgDeadTime2_0us == DeadTimeConfig )
			{
				RecDutyCompensate = 0.025f;
			}
			else
			{
				RecDutyCompensate = 0.015f;
			}
			InvDutyCompensate = 0.01f;
			break;

		case PARAM_K1MaxTimeToOpen:
			if( (*data) >= 20 )
			{
				K1MaxTimeToOpen = (uint16_t)(((uint32_t)(*data))*1000/RECT_SAMPLE_PERIOD_US);
			}
			else
			{
				K1MaxTimeToOpen = (uint16_t)(20000/RECT_SAMPLE_PERIOD_US);
			}
			break;

		case PARAM_BalMaxTimeToOpen:
			if( (*data) >= 20 )
			{
				BalMaxTimeToOpen = (uint16_t)(((uint32_t)(*data))*1000/RECT_SAMPLE_PERIOD_US);
			}
			else
			{
				BalMaxTimeToOpen = (uint16_t)(20000/RECT_SAMPLE_PERIOD_US);
			}
			break;

		case PARAM_ESSChrgCurrentOffsetCal:
			if( (*data) >= 10200 )
			{
				ESSChrgCurrentOffsetCal = 20.0f;
			}
			else if( (*data) <= 9800 )
			{
				ESSChrgCurrentOffsetCal = -20.0f;
			}
			else
			{
			    ESSChrgCurrentOffsetCal = 0.1f * ( (float)(*data) - (float)CALIBRATION_DEFAULT );
			}
			break;
	
        default:
            break;          
    }// End add
}

void ee_ram_float(const EE_ID* ee, const uint16_t* data)
{
    float *dest = (float *)ee->ram_ptr;
    for (uint16_t idx = 0; idx < ee->eep_length; idx++)
    {
        *dest++ = float(*data++);
    }
}

// ***********************************************************************
// *
// *    FUNCTION: ee_calc_out_volts 
// *
// *    DESCRIPTION: output voltage and related data
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_calc_out_volts( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_OutNomVolts:
            ee_ram( ee, data );
            
            // set inverter
            Inverter.SetInverterVoltage( OutNomVolts );

            // set PLL nominal references
            Rectifier.UtilityPLL.SetNominalSourceVoltage( OutNomVolts );
            BypassPLL.SetNominalSourceVoltage( OutNomVolts );
            OutputPLL.SetNominalSourceVoltage( OutNomVolts );

            // Update Max Rectifier current base on normal voltage
            Rectifier.UpdateMaxRectifierCurrent( OutNomVolts );
            break;
        default:
            break;
    }     
}

// ***********************************************************************
// *
// *    FUNCTION: ee_calc_out_freq 
// *
// *    DESCRIPTION: output voltage and related data
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_calc_out_freq( const EE_ID* ee, const uint16_t* data )
{
}

// ***********************************************************************
// *
// *    FUNCTION: ee_calc_out_power 
// *
// *    DESCRIPTION: output power and related data
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_calc_out_power( const EE_ID* ee, const uint16_t* data )
{
    switch( ee->paramNum )
    {   
        case PARAM_OutPF:
            ee_ram( ee, data );       
            if ( ( 0 != OutputkVARating ) &&
                 ( 0 != OutputPowerFactorRating ) )
            {
                // calculate output power
                OutputkWRating = ( (uint32_t)OutputkVARating * (uint32_t)OutputPowerFactorRating ) / 10000L;
            }     
            break;
        
        case PARAM_OutKVA:
            if ( CheckValidOutputKVARating ( *data ) )   // keming/20120808B  edit. if KVARating value is invalid, we will not put it to ram.
            {   
                // copy to RAM
                ee_ram( ee, data );   
                if ( ( 0 != OutputkVARating ) &&
                     ( 0 != OutputPowerFactorRating ) )
                {
                    // calculate output power
                    OutputkWRating = ( (uint32_t)OutputkVARating * (uint32_t)OutputPowerFactorRating ) / 10000L;
                }
                Inverter.RMSCurrentNormFactor = (float)OutNomVolts * 0.1 / ((float)OutputkVARating * 100.0 / 3.0);
//	                //Yivan/20141030/add begin...
//	                // Update the rectifer and inverter current limit if the UPM KVA rating has been changed by CSB
//	                if(EEStatusBits.bit.EEDataInitialized)
//	                {
//	                    if(267 == OutputkVARating)
//	                    {
//	                        InvHWCurrentLimitResetValue = 108;
//	                        RecHWCurrentLimitResetValue = 98;
//	                        PutEepData( PARAM_Inverter_Current_Limit_Set, 1, &InvHWCurrentLimitResetValue, 0 );
//	                        PutEepData( PARAM_Rectifier_Current_Limit_Set, 1, &RecHWCurrentLimitResetValue, 0 );
//	                    }
//	                }
//	                //Yivan/20141030/add end...
            }      
        break;
        
    default:    
        break;
    }
}
//Jacob/20130813/Merge 120K ...modify end
extern bool PreventWatchdogKick;
// ***********************************************************************
// *
// *    FUNCTION: CheckOutputKVARatingChanged 
// *
// *    DESCRIPTION: Reset EEPMAP and restart DSP when OutputKVARating Changed
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void CheckSystemChanged( void )
{
    if ( SystemChanged )
    {
        SystemChanged = 0;

          // Mask reset all eep, especially calibrate eep. if need reset, use rs255 or ate cmd 
//        // Reset EEPROM
//        uint16_t thisSection = EE_System_ID;
//
//        while ( thisSection < EE_Last_Section )
//        {
//            ReBootEepromSection( thisSection );
//            ++thisSection;
//            TSK_sleep(TSK_5_ms);
//        }

        //Import 500k fix code. Just in case
        SEM_reset(&PeriodicTaskSem, 0);
        // Reset DSP
        PreventWatchdogKick = 1;
    }

    if ( SystemUpdated )
    {
        SystemUpdated = 0;
        //This sleep doesn't matter, because DSP will reset soon.
        TSK_sleep(TSK_5_ms);

        // Reset DSP
        PreventWatchdogKick = 1;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: ValidOutputKVARating 
// *
// *    DESCRIPTION: Return if the requested OutputKVA is valid,don't need,will be check by service tool
// *
// *    ARGUMENTS: kva_rating: desired KVA in KVA*10.
// *
// *    RETURNS:
// *
// ***********************************************************************
bool CheckValidOutputKVARating( uint16_t kva_rating )
{
	return true;
}

// ***********************************************************************
// *
// *    FUNCTION: ValidSystemType
// *
// *    DESCRIPTION: Return if the requested System Type is valid, do not need,will be check by service tool
// *
// *    ARGUMENTS: kva_rating: desired KVA in KVA*10.
// *
// *    RETURNS: true
// *
// ***********************************************************************
bool CheckValidSystemTypeRating( uint16_t type )
{
    return true;
}

// ***********************************************************************
// *
// *    FUNCTION: SetRectHWCurrentLimit 
// *
// *    DESCRIPTION: Set different Rectifier HWCL at different OutNomVolts
// *
// *    ARGUMENTS:
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SetRectHWCurrentLimit( void )
{
   //do nothing
}

// ***********************************************************************
// *
// *    FUNCTION: SetBalancerHWCurrentLimit 
// *
// *    DESCRIPTION: Set different Rectifier HWCL when UPS on battery
// *
// *    ARGUMENTS:
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SetBalancerHWCurrentLimit( void )
{
    //do nothing
}

// ***********************************************************************
// *
// *    FUNCTION: SetBalancerHWCurrentLimit
// *
// *    DESCRIPTION: Set different Rectifier HWCL when UPS on battery
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
extern void ee_system_type( const EE_ID* ee, const uint16_t* data )
{
    CoefficientsIndex = 0;
    switch( ee->paramNum )
    {
        case PARAM_SystemType:
            if ( CheckValidSystemTypeRating( *data ) )
            {
                // copy to RAM
                SystemType.all = *data;

                switch(SystemType.bit.VoltageLevel)
                {
                    case HIGH_VOLTAGE:
                        switch(SystemType.bit.UPMKVALevel)
                        {
                            case UPMKVA_10K:
                            case UPMKVA_15K:
                            case UPMKVA_20K:
                                UPMSystem = HV_20K;
                                CoefficientsIndex = 0;
                                break;

                            case UPMKVA_25K:
                            case UPMKVA_30K:
                            case UPMKVA_27K:
                                UPMSystem = HV_30K;
                                CoefficientsIndex = 2;
                                break;
								
							case UPMKVA_40K:
								UPMSystem = HV_40K;
                                CoefficientsIndex = 1;
								break;

                            default:
                                UPMSystem = SystemInvalid;
                                break;
                        }
                        break;

                    default:
                        UPMSystem = SystemInvalid;
                        break;
                }

                // Detecting system update or system changed
                if( ( UPMSystem != PreUPMSystem )          &&
                    ( PreUPMSystem != SystemInitializing ) &&  // System has already been initialized before
                    ( UPMSystem != SystemInvalid ) )           // No action(No need to detecting system changed ) when system been changed to invalid system
                {
                    if( ( PreUPMSystem == SystemInvalid ) &&
                        PreUPMVersionLessThanV1_23() )         // detecting previous UPM version is v1.22 or older than v1.22
                    {
                        // It is only update firmware from old version to new common package version.
                        // No need to reset EEPROM, only reset DSP is enough.
                        SystemUpdated = 1;
                    }
                    else
                    {
                        // system changed from one valid system to another, need reset EEPROM and then reset DSP.
                        SystemChanged = 1;
                    }
                }
            }
            else
            {
                if(UPMSystem == SystemInitializing)
                {
                    UPMSystem = SystemInvalid;
                }
            }

            PreUPMSystem = UPMSystem;

            // Configuration error if system is unknown.
            if(UPMSystem == SystemInvalid)
            {
                EEStatusBits.bit.InvalidSystem = 1;
            }
            else
            {
                EEStatusBits.bit.InvalidSystem = 0;
            }
            break;

        default:
            break;
    }
}

void SetSystemType(uint16_t voltage_level, uint16_t kVA_level)
{
    SYSTEM_TYPE type = {0};
    type.bit.VoltageLevel = voltage_level;
    type.bit.UPMKVALevel = kVA_level;

    PutEepData(PARAM_SystemType, 1, &(type.all), TSK_1000_ms);    // Write newValue to eepAddress
}

bool PreUPMVersionLessThanV1_23(void)
{
    bool result = false;

    if ( Inverter.ValidInverterVoltage( OutNomVolts ) )
    {
    	if( EEVersion <= V1_22_EEMAP_VERSION_HV )
    	{
    	     result = true;
    	}
    }

    return result;
}
// ************************************************************************************
//  No more
// ************************************************************************************
