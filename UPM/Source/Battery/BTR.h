#ifndef BTR_H_
#define BTR_H_
// ******************************************************************************************************
// *            BTR.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton  
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BTR.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: 
// *
// *    DATE: 2/8/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#include "Constants.h"
#include "algos.h"
#include "Eeprom_Map.h"
#include "StateTimer.h"


typedef struct
{
    uint16_t BatteryTest                : 1;
    uint16_t BatteryQuickTest           : 1;
    uint16_t BTAbortBypassNotAvailable  : 1;
    uint16_t BTAbortLoadChange          : 1;
    
    uint16_t BTDone                     : 1;
    uint16_t BTFailLowBattery           : 1;
    uint16_t BTMonitorBattTestFail      : 1;
    uint16_t UseHRDFullCapacityBTR      : 1;    

    uint16_t CustomDischargeCurve       : 1;
    uint16_t BattTestResultsReady       : 1;
    uint16_t BTRParaChanged             : 1;    // Jack/20150112
    uint16_t BTRInit                    : 1;
        
    uint16_t nibble3                    : 4;
} stBTRStatus;

typedef union
{
    stBTRStatus bit;
    uint16_t    all;
} uBTRStatus;

typedef struct
{
    float Power;
    float Voltage;   
    float Current;    
} BatteryTestInfo;

typedef struct
{
    BatteryTestInfo FiftyPercentTest;
    BatteryTestInfo OneHundredPercentTest;
} DualStageBatteryTestInfo;

typedef struct
{
     DualStageBatteryTestInfo   BatteryTest;
     float                      Resistance;
     float                      Health;   
} BatteryTestResultInfo;

// Pan/20120914 add begin
typedef struct      
{
    float Slope;
    float Intercept;
    float ScaleFactor;
}stBTRParameters;

typedef struct      
{
    float A0;
    float A1;
    float A2;
    float A3;
    float A4;
}stEffParameters;
// Pan/20120914 end

class BatteryTimeRemaining
{
    public:
        BatteryTimeRemaining( void ) 
        {
            BTRStatus.all = 0;
            ChargeTimePercent = 0;
            PercentOfCapacity = 0;
            ChargeVoltagePercent = 1.0;
            HRDBattTimeIntercept = 1.0;
            HRDBattTimeSlopeReciprocal = 1.0;
            LRDBattTimeIntercept = 1.0;
            LRDBattTimeSlopeReciprocal = 1.0;
            PreviousPower = 0;          // Pan/20120925 initial value;
        }
        ~BatteryTimeRemaining()
        {
        }
        
        stBTRStatus GetBTRStatus( void )
        {
            return  BTRStatus.bit;
        }
    public:
        void Init( void );
        void MonitorFullCapacityBTR( void );
        void CalculateBatPower( stEffParameters const * EffParaTablePtr );   // Pan/20121129 add
        void ComputeMixedBTR();         // Pan/20121011 add
        void StartDualStageBatteryTest( void );
        void MonitorStateOfCapacity( void );
        void FindBatteryTestResults( void );
        void CheckAlarms( void );
        void TestResultsReady( void )
        {
            BTRStatus.bit.BattTestResultsReady = 1;
        }

        void EEFunc_BTR( const EE_ID* ee, const uint16_t* data );

        
    public:
        float           BatteryTime;
        float           FullCapacityBTR;
        float           HRDFullCapacityBTR;
        float           LRDFullCapacityBTR;
        float           HRDBattTimeIntercept;
        float           HRDBattTimeSlopeReciprocal;
        double          HRDBattTimeExpConstant;
        float           LRDBattTimeIntercept;
        float           LRDBattTimeSlopeReciprocal;
        double          LRDBattTimeExpConstant;
        float           CustomDischargeIntercept;
        float           CustomDischargeSlopeReciprocal;
        double          CustomDischargeExpConstant;
        uint16_t        WattsPerCell15Min;
        uint16_t        WattsPerCell60Min;
        uint16_t        WattsPerCellUnits;  //Jack/20150108/add
        float           TestLoadLevel;
        float           BatteryTestFailVLevel;
        uBTRStatus      BTRStatus;
        
        float           StartChargeVoltage;
        float           ChargeVoltagePercent;
        float           ChargeCurrentPercent;
        float           ChargeTimePercent;
        float           PercentOfCapacity;
        float           OldPercentOfCapacity;
        float           TimeOnBattery;
        float           BatteryTimeLeft;
        
        // Pan/20120925 add variables for parallel battery BTR, begin
        float           SDTInternalExp;
        float           SDTExternalSlope;
        float           SDTExternalFactor;
        float           dSDTInternalExp;

        float           LDTInternalExp;
        float           LDTExternalExp;
        float           dLDTInternalExp;
        float           dLDTExternalExp;

        float           PreviousPower;
        
        //add for UPM debugger
        uint16_t        NumOfSDTIteration;
        uint16_t        NumOfLDTIteration;
        // Pan/20120925 end       
        
        // Pan/20121112 add for debugger begin
        float           BatEff;
        float           BatPower;
        float           UPSPower;
        // Pan/20121112 end
        
        BatteryTestResultInfo    CommisioningTestResults;
        BatteryTestResultInfo    LastBatteryTestResults;                
        StateTimer      BatteryTestTimer;
        StateTimer      BatteryTestFailTimer;

};

extern BatteryTimeRemaining BTR;

inline void ee_calc_BTR( EE_ID* ee, uint16_t* data )
{
    BTR.EEFunc_BTR( ee, data );
}

// ******************************************************************************************************
// *            End of BTR.h
// ******************************************************************************************************

#endif /*BTR_H_*/
