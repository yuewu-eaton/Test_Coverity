#ifndef _EEPROM_MAP_H
#define _EEPROM_MAP_H
// ********************************************************************************************************
// *            Eeprom_Map.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Eeprom_Map.h
// *
// *    DESCRIPTION: Equates for the EEPROM SECTION MAPPING
// *
// *    ORIGINATOR: Fred Tassitino
// *
// *    DATE: 05/20/2003
// *
// *    HISTORY: See CVS history
// *********************************************************************************************************

#include "Parameters.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// *********************************************************************************************************
// *        Current version of EEPROM map
// *********************************************************************************************************
#define CURRENT_EEMAP_VERSION_HV           35
#define CURRENT_EEMAP_REVISION              1

#define V1_22_EEMAP_VERSION_HV            9

#define MY_EE_ID                            0x1234          // means nothing

// other constants
    // EEP_Map.flags definitions, bit encoded
#define MIRROR1                             0x0001          // Set to force Mirror on Section, Primary Section
#define MIRROR2                             0x0002          // Set to force Mirror on Section, Secondary Section
#define NO_MIRROR                           0x0000          // Do not Mirror
#define NO_CHECKSUM                         0x0000          // Not protected by a Checksum
#define CHECKSUM                            0x0004          // Protected by a checksum
#define QUEUE                               0x0008          // History Queue, Battery Test Queue, etc..
#define NOT_USED_SECTION                    0x0000          // Not Used
#define SCRATCHPAD                          CHECKSUM        // Checksum Protect
#define NO_DEFAULTS                         0x0010
#define LAST_SECTION                        0x8000          // last Section Marker

#define MAX_EE_LENGTH                       4096            // max Block operation, 4096 words
#define MAX_SECTIONS                        64              // the eeprom can be divided into a maximum

//Jacob/20130813/Merge 120K
#define KVARATING_20K                       200				//Keming/20120814,add
#define KVARATING_30K                       300
#define KVARATING_40K                       400
#define KVARATING_80K                       800
#define KVARATING_100K                      1000
#define KVARATING_120K                      1200

//precharge type
#define FLYBACK         0
#define RESISTOR        1

// EEP_Map.flags definitions
#define MAX_DEF_TYPE    3  //max eep default value type: Hobbit 93PR T 20/30/40K

// *********************************************************************************************************
// *    EEPROM ACCESS DEFINITIONS - EEPROM is the LC25C640
// *********************************************************************************************************
#define MAX_EE_NUMBER                       4095

enum EEPROM_FUNCTIONS
{
    EE_NOTHING,
    EE_RAM,
    EE_CALIBRATION,
    EE_PROTECTIONS,
    EE_ABM_DATA,
    EE_BTR_DATA,
    EE_OUT_VOLTS,
    EE_OUT_FREQ,
    EE_AC_LIMITS,
    EE_OUT_POWER,
    EE_BYPASS_LIMITS,
    EE_UPDATE_PARALLEL,
    EE_UPDATE_RECTIFIER,
    EE_UPDATE_INVERTER,
    EE_UPDATE_OL,
    EE_RAM_FLOAT,
    EE_LOGIC_POWER,
    EE_BASE_SYNC,
    EE_BYPASS_SYNC,
    EE_SYSTEM_TYPE,
	EE_OUTPUT_SYNC,
	EE_UTILITY_SYNC,    
    // Keep this ORed bit at the end, new entries go above this one
    EE_PARALLEL = 0x1000u
};

// ********************************************************************************************************
// *            Structure definitions
// ********************************************************************************************************

typedef struct EE_SECTION
{
    uint16_t    length;           // number if words in this sections
    uint16_t    flags;            // Bits to alter operation, see below for definitions
}EE_SECTION;

typedef struct
{
    uint16_t    UPMKVALevel    :8;           // Bit 0~7
    uint16_t    Reserved       :6;           // Bit 8~13
    uint16_t    VoltageLevel   :2;           // Bit 14~15
}SYSTEM_TYPE_BITS;

// this is what allows you to access the bits as either bits or words
typedef union
{
	SYSTEM_TYPE_BITS bit;
    uint16_t         all;
} SYSTEM_TYPE;

#define VOLTAGE_LEVEL_OFFSET         14
#define VOLTAGE_LEVEL_MASK (0x3<<VOLTAGE_LEVEL_OFFSET)
#define UPMKVA_LEVEL_MASK 0xff
#define SYSTEM_TYPE_BIT_MASK (VOLTAGE_LEVEL_MASK|UPMKVA_LEVEL_MASK)

#define HIGH_VOLTAGE                     0x01
#define LOW_VOLTAGE                      0x00

#define UPMKVA_10K                       10
#define UPMKVA_15K                       15
#define UPMKVA_20K                       20
#define UPMKVA_30K                       30
#define UPMKVA_25K						 25
#define UPMKVA_27K                       27
#define UPMKVA_40K                       40
#define UPMKVA_80K                       80
#define UPMKVA_100K                      100
#define UPMKVA_120K                      120

enum UPM_SYSTEM_TYPE
{
    SystemInitializing,
	HV_40K,
    HV_20K,
    HV_30K,
    SystemInvalid
};

typedef struct
{
    uint16_t    eep_addr;           // the address in the eeprom device
    uint16_t    eep_DefaultValue[MAX_DEF_TYPE];   // initialization default value: 
    void*       ram_ptr;            // corresponding ram  address      
    uint16_t    eep_length;         // length of eep
    uint16_t    eep_functionIndex;  // the function to call when accessed: 
    parameter_t paramNum;           // parameter number
}EE_ID;
// ee_function def
typedef void (*EEP_PTR)( EE_ID* ee, uint16_t* data );

enum EE_Map_Sections
{
    EE_System_ID,
    EE_Unit_Setup,
    EE_Protection,          
    EE_Battery_Setup,
    EE_Calibration,
    EE_history_queue,
    EE_Battery_Test_queue,
    EE_Private_Setup,
    EE_Triggered_History,
    EE_Not_Used1,
    EE_Last_Section 
};  

// UNIT SETUP 1 bits
typedef struct
{
    uint16_t     AutoBypassEnable        :1;
    uint16_t     ATBUnconditional        :1;
    uint16_t     unused                  :14;
}st_Unit_Setup1_bits;

typedef union
{
	st_Unit_Setup1_bits bit;
    uint16_t           word;
}uUnit_Setup1AndBits;

// sync configuration bits
typedef struct
{
    uint16_t     EnableCanFailSyncToByp  :1;
    uint16_t     ForceSyncBaseInLineMode :1;
    uint16_t     unused                  :14;
}st_Sync_Config_bits;

typedef union
{
    st_Sync_Config_bits bit;
    uint16_t            word;
}uSyncConfigWordsAndBits; 

// unit configuration bits 
typedef struct
{
    uint16_t     EnableSurgeProtection   :1;
    uint16_t     TempCompensateEnable    :1;
    uint16_t     unused1    		 	 :2;
	
    uint16_t     ECTDTProtectEnable    	 :1;
	uint16_t     EnableSurgeToBattery    :1;
	uint16_t     DisableDCControl    	 :1;
	uint16_t     CAP_I_DETEnable		 :1;
	
	uint16_t     EnablePhaseOffset    	 :1;	
    uint16_t     unused2                 :7;
}st_Unit_Config_bits;

typedef union
{
    st_Unit_Config_bits bit;
    uint16_t            word;
}uUnitConfigWordsAndBits; 

// ********************************************************************************************************
// * EEPROM Status Bits
// ********************************************************************************************************
    // EEPROM Status Bit definitions -----------------------------------------------------
typedef struct
{
        // word 0
   uint16_t   Section0Fail:1;             // 0
   uint16_t   Section1Fail:1;             // 1
   uint16_t   Section2Fail:1;             // 2
   uint16_t   Section3Fail:1;             // 3

   uint16_t   Section4Fail:1;             // 4
   uint16_t   Section5Fail:1;             // 5
   uint16_t   Section6Fail:1;             // 6
   uint16_t   Section7Fail:1;             // 7

   uint16_t   Section8Fail:1;             // 8
   uint16_t   Section9Fail:1;             // 9
   uint16_t   Section10Fail:1;            // 10
   uint16_t   Section11Fail:1;            // 11

   uint16_t   Section12Fail:1;            // 12
   uint16_t   Section13Fail:1;            // 13
   uint16_t   Section14Fail:1;            // 14
   uint16_t   Section15Fail:1;            // 15

        // word 1
   uint16_t   EEDataInitialized:1;        // 16
   uint16_t   dummy17:1;                  // 17
   uint16_t   dummy18:1;                  // 18
   uint16_t   dummy19:1;                  // 19
   uint16_t   dummy20:1;                  // 20
   uint16_t   dummy21:1;                  // 21
   uint16_t   dummy22:1;                  // 22
   uint16_t   dummy23:1;                  // 23
   uint16_t   dummy24:1;                  // 24
   uint16_t   dummy25:1;                  // 25
   uint16_t   dummy26:1;                  // 26
   uint16_t   dummy27:1;                  // 27
   uint16_t   dummy28:1;                  // 28
   uint16_t   dummy29:1;                  // 29
   uint16_t   dummy30:1;                  // 30
   uint16_t   dummy31:1;                  // 31

        // word 2	//0x4480- 0100,0100,1000,0000
   uint16_t   ParallelSetupError:1;       // 32
   uint16_t   EepromVersionDowngrade:1;   // 33
   uint16_t   PLDVersionFail:1;           // 34  // not eeprom related but needed for configuration error nodebit
   uint16_t   BypassParallelErr:1;        // 35  // Observed bypass PIC when not expected
   
   uint16_t   UnexpectedParallelError:1;  // 36  // Observed PCAN packets when non-parallel config
   uint16_t   PrivateSetupMismatch:1;     // 37  // Private section values mismatch between boards
   uint16_t   ServiceRequired:1;          // 38  // Field service is required
   uint16_t   SetupRequired:1;            // 39  // EEPs were defaulted and verification is requried.

   uint16_t   HardwareIncompatible:1;     // 40  // Hardware revision is incompatible with current configuration.
   uint16_t   InvalidSystem:1;            // 41
   uint16_t   SystemTypeCheckFail:1;      // 42
   uint16_t   DecryptionFailed:1;         // 43  // Decryption failed when communicate with SHA204

   uint16_t   DeadBandConfFail:1;		  // 44
   uint16_t   CntlBoardVersionFail:1;     // 45
   uint16_t   BatterySetupRequired:1;     // 46
   uint16_t   dummy47:1;                  // 47

        // To add any more bits, another word
        // must be added to the word structure below.
} stEE_Status_Bits;

    // this is what allows you to access the bits as either bits or words
typedef union
{
   stEE_Status_Bits bit;
   uint16_t         w[ sizeof( stEE_Status_Bits ) ];
} uEEStatusWordsAndBits;

#define EE_STATUS_CHECKSUM_WORD         0
#define EE_STATUS_FLAGS_WORD            1
#define EE_STATUS_CONFIG_WORD           2

extern uEEStatusWordsAndBits EEStatusBits;

// ********************************************************************************************************
// *            Global data
// ********************************************************************************************************

extern EEP_PTR const Eep_Functions[];
extern const EE_ID eeprom[];

extern const EE_SECTION EE_Map[];

// section 0 ee data
extern uint16_t EEVersion;
extern uint16_t EERevision;
extern uint16_t MyUPSNumber;
extern uint16_t MyUPMNumber;
extern uint16_t MyHardwareNumber;
extern uint16_t ExtSyncEnabled;
extern uint16_t UPMCalEnabled;
extern uint16_t NumOfUPMs;
extern uint16_t MOBEnabled;
extern uint16_t DisableAutoID;
extern uint16_t ParallelForRedundancy;
extern uint16_t NumOfUPSs;
extern SYSTEM_TYPE SystemType;

// section 1 data
extern uint16_t BypassWithNormalCommand;
extern uint16_t EPOEnabled;
extern uint16_t OutNomVolts;
extern uint16_t OutNomFreq;
extern uint16_t BypVoltMaxLimit;
extern uint16_t BypVoltMinLimit;
extern uint16_t BypVoltMaxLimitFast;
extern uint16_t BypVoltMinLimitFast;
extern uint16_t BypVoltMinLimitFast_ESS;
extern uint16_t OutputPowerFactorRating;
extern uint16_t OutputkVARating;
extern uint16_t OutputkWRating;
extern uint16_t DisableImmBypassTransfer;
extern uint16_t ImmBypassTransferDelay;
extern uint16_t InputSyncEnabled;
extern uint16_t DisableTooManyBatTrans;
extern uint16_t ParaLoadShareErrorLimit;

extern uint16_t BatteryNotRequiredForOnline;
extern uint16_t EnableAutoStandby;
extern uint16_t EnableAutoRestart;
extern uint16_t AutoRestart;
extern uint16_t ESSInstalled;
extern uint16_t ESSEnabled;
extern uint16_t ESSMaxNumOutages;
extern uint16_t ESSOutageResetTime;
extern uint16_t ESSLockoutTime;
extern uint16_t STSWshortCurrentPercent;
extern uint16_t InternalMBSInstalled;
extern uint16_t EnableOpenLoop;
extern uint16_t ATBBypassCMDEnable;
extern uint16_t ATBEnabled;
extern uint16_t PowerConditionerMode;
extern uint16_t EEBalRelayTransferTime; 
extern uint16_t EERelL1TransferTime;    
extern uint16_t EEBattRelayTransferTime;
extern uint16_t EE_PLLMaxPhaseError;
extern uint16_t SetupRequired; // Always true after section reset
extern uint16_t BackfeedContactorInstalled;
extern uint16_t BatteryBreakerInstalled;
extern uint16_t EnableVFMode;
extern uint16_t VFModeLoadPercentMax;
extern uint16_t VFModeLoadPercentHysteresis;
extern uint16_t VFModeEntryTime;
extern uint16_t EE_BattChrgCurrentMax;
extern uint16_t ESSOutputSdLowTime;
extern uUnit_Setup1AndBits Unit_Setup1;
extern uSyncConfigWordsAndBits SyncConfig;
extern uUnitConfigWordsAndBits UnitConfig;
extern float  DCLinkStepMax;
extern uint16_t DeadTimeConfig;
extern float  RecDutyCompensate;
extern float  InvDutyCompensate;

extern uint16_t EnableDeadTimeCompensate;
extern uint16_t EnablePullChainChk;
extern uint16_t EEEPODelay;
extern uint16_t MinReqkVA;

extern uint16_t K1MaxTimeToOpen;
extern uint16_t BalMaxTimeToOpen;
extern uint16_t InputCapCompRef;
extern uint16_t PrechargeTime;
extern uint16_t PrechargeType;
extern uint16_t InvControlParaOption;
extern uint16_t EnableParalPwmSync;
extern uint16_t FrequencyConverterMode;

// section 2 data
extern int16_t PhaseAHeatsinkTempWarningLimit;
extern int16_t PhaseAHeatsinkTempTripLimit;
extern int16_t PhaseBHeatsinkTempWarningLimit;
extern int16_t PhaseBHeatsinkTempTripLimit;
extern int16_t BatHeatsinkTempWarningLimit;
extern int16_t BatHeatsinkTempTripLimit;
extern int16_t SCRHeatsinkTempWarningLimit;
extern int16_t SCRHeatsinkTempTripLimit;
extern int16_t InvCapHeatsinkTempWarningLimit;
extern int16_t InvCapHeatsinkTempTripLimit;
extern int16_t PhaseCHeatsinkTempWarningLimit;
extern int16_t FanFailTempPrtDec;
extern int16_t RecFanFailTempPrtDec;
extern int16_t BatFanFailTempPrtDec;
extern int16_t InvHeatsinkTempFanFull;
extern int16_t RecHeatsinkTempFanFull;
extern int16_t AutoECTEnable;
extern int16_t AmbientTempWarningLimit; 
extern int16_t ACPreChargeRailLimit;
extern int16_t DCPreChargeRailLimit;
extern uint16_t DisableSiteWiringFault;
extern float SiteWiringFaultHighLimit;
extern uint16_t DisableRevisionCheck;
extern uint16_t OptimizeInputPFEnabled;
extern uint16_t MinLoadForMeterDisplay;
extern uint16_t EnableDisChargeLink;
extern float DisChargeLinkVoltage;
extern uint16_t FanInputAmpsRMSLevel_1;
extern uint16_t FanInputAmpsRMSLevel_2;
extern uint16_t FanInputAmpsRMSLevel_3;
extern uint16_t FanInputAmpsRMSHysteresis;
extern uint16_t FuseFailLoadPercentLimit;
extern float UtilityLossLoadPercentLimit;
extern uint16_t AOLEnable;
extern float AolAmbTempLimit;
extern float AolAmbTempHyst_x10;
extern uint16_t WalkinStartPercentLoad;
extern uint16_t KeepAlive_Delay;

// section 3 data
extern uint16_t BatteryAhRating;
extern uint16_t BatteryTimeRating;
extern uint16_t CommissioningTest;
extern uint16_t CompensateRate;
extern uint16_t BattRelayOpenCurrent;

// section 7 data
extern uint16_t ServiceRequired;  // Defaults to false.  When set, unit requires field service prior to going online.

extern bool SystemChanged;
extern bool SystemUpdated;
extern uint16_t RelayTime;
extern uint16_t UPMSystem;
extern uint16_t PreUPMSystem;
extern uint16_t EnableTransferBatDCF;
extern int16_t DiffVoltInvOutRms_X10;
extern float  EEInputHighLimit;
extern float  EERecHighLimit;
extern float  EEDClinkDelta;
extern float  ESSChrgCurrentOffsetCal;

extern uint16_t CoefficientsIndex;

extern uint16_t NumOfModule;
extern uint16_t NewCtrlBoard;


// function prototypes
uint16_t GetEETableSize(void);
uint16_t GetNumEESections(void);
uint16_t Find_Eep_Num( const void* address );
bool     EepromAddressValid(uint16_t start_address, uint16_t num_words);
uint16_t FindEepromTableIndex(uint16_t eeprom_address);
uint16_t GetSectionNumber(uint16_t address);
uint16_t GetSectionAddress(uint16_t section_number);
EE_ID*   GetParameterEE( parameter_t param );

void CheckSystemChanged( void );
bool CheckValidOutputKVARating( uint16_t kva );
void SetBalancerHWCurrentLimit( void );
void SetRectHWCurrentLimit( void );
bool CheckValidSystemTypeRating( uint16_t type );
void SetSystemType(uint16_t voltage_level, uint16_t kVA_level);
bool PreUPMVersionLessThanV1_23(void);

inline uint16_t CheckChecksumProtected(uint16_t section_number)  
{   return (EE_Map[section_number].flags & CHECKSUM);   }
inline uint16_t GetSectionLength(uint16_t section_number)
{   return (EE_Map[section_number].length);             }
inline uint16_t EEP_InputSyncEnabled( void )
{   return InputSyncEnabled;                            } 
inline uint16_t EEP_ExtSyncEnabled( void )
{   return ExtSyncEnabled;                              }
inline bool EEP_IsInternalParallel( void )
{   return ( NumOfUPMs > 1 );                           } 
inline bool EEP_ParallelForRedundancy( void )
{   return ParallelForRedundancy;                       }  


#ifdef __cplusplus
}
#endif

// ********************************************************************************************************
// *            END OF Eeprom_Map.h
// ********************************************************************************************************
#endif
