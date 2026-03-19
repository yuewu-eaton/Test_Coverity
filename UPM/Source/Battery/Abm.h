// ******************************************************************************************************
// *        ABM.H
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *                      ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *     FILE NAME:   ABM.H
// *
// *     DESCRIPTION: ABM public function prototypes
// *                  
// *     ORIGINATOR:  Tuomo Kaikkonen
// *
// *     DATE:        11/04/2003
// *
// *     HISTORY:     See HPO CVS history
// ******************************************************************************************************
#ifndef __ABM_H
#define __ABM_H
#include "BatteryStateControl.h"
// ******************************************************************************************************
// *        ABM State machine states
// ******************************************************************************************************
enum eAbmState
{
    ABM_RESET,                  
    ABM_CHARGE,                 
    ABM_FLOAT,                  
    ABM_REST,                   
    ABM_DISCHARGING            
};

#define ABM_TIMEBASE                uint32_t(10u)             // 10 times / second
#define ABM_CLK_15S                 uint32_t(15u * ABM_TIMEBASE)
#define ABM_CLK_20S                 uint32_t(20u * ABM_TIMEBASE)
#define ABM_CLK_30S                 uint32_t(30u * ABM_TIMEBASE)
#define START_CHRG_TIME             ABM_CLK_15S
#define FILTER_SETTLING_TIME        uint32_t(5u * ABM_TIMEBASE)
#define CT_ZERO_THRESHOLD           uint32_t(15u)
#define OCV_DISABLE_TIME            uint32_t(1800u * ABM_TIMEBASE)    // 30 min

#define FLOAT_T_EXT_MULTIPLIER      1.5                      // For calculating float time extension (1.5 * TotalChargeTime)
#define BATT_SUPP_TEST_MAXWAIT      uint32_t(60u)            // Maximum time to wait to be able to run the battery support test, seconds

// ******************************************************************************************************
// *        return statuses of ABM_Battery_Support_Test() function
// ******************************************************************************************************
#define ABM_BST_STARTED                 0
#define ABM_BST_RUNNING                 1
#define ABM_BST_CANNOT_RUN              2

#define BST_LOW_RATE_THRESH             uint32_t(900u)            // 15 min

// default times for battery test:
enum 
{
  ABM_BST_Disch_Default = 15
};

// ******************************************************************************************************
// *        ABM battery statuses (ABM_Get_Batt_Status)
// ******************************************************************************************************
#define UI_STATUS_NO_BATT               0       // for LCD
#define XCP_STATUS_CHARGE               1       // match XCP ABM status
#define XCP_STATUS_DISCHARGE            2
#define XCP_STATUS_FLOAT                3
#define XCP_STATUS_REST                 4
#define XCP_STATUS_UNKNOWN              5       // useful??

// *****************************************************************************
// * Types of Common Battery configurations
// *****************************************************************************
namespace {
    const uint16_t CommonBattery_Separate = 0;
    const uint16_t CommonBattery_Internal = 1;
    const uint16_t CommonBattery_External = 2;
}

// ******************************************************************************************************
// *        EEPROM VARIABLES
// ******************************************************************************************************
extern uint16_t NumberOfBatteryCells;     // number of cells connected in series
extern float    NumberOfBatteryStrings;   // number of parallel battery strings
extern uint16_t NumberOfExternalBatteryStrings;     // Pan/20120914 number of external parallel battery strings
extern uint16_t BattChrgTimeMax;          // maximum time in constant current charge
extern uint16_t BattFloatTime;            // ABM cycling float time
extern uint16_t BattMaxRestTime;          // maximum rest mode time, duration of rest mode
extern uint16_t BattMaxDischTime;         // max discharge time to reset ABM
extern uint16_t BattRestFailTime;         // battery OCV failure time in rest mode
extern uint16_t CommonBattery;            // nonzero for common battery configs
extern float LowRateTestThreshold;        // Battery test fail threshold for discharge time > 15 min
extern float HighRateTestThreshold;       // Battery test fail threshold for discharge time <= 15 min
extern float BattABMFloatVPC;      // charge voltage, the level where charge mode changes to float mode
extern float BattChrgCurrentMax;
extern uint16_t ABMDisabled;       //Jacob/20130823/add
extern float BattMaxchrgCurrentforESS;

extern float  BattFloatV_Comp;      // The temperature compensated limit where charge mode changes to float mode
extern float  BattVoltsperCell;     // Battery voltage in volts/cell
extern float  BattVoltsperCellBU;   // Redundant battery voltage measurement, V/cell
extern float  BattVoltsperCell1;
extern float  BattVoltsperCell2;
extern float  BattConstFloatVPC;
extern float ABMBattOVClear;
extern uint32_t total_Float_Time;     // 
extern uint16_t BattFloatTimeExt;     // ABM cycling float time extension; BattFloatTimeExt = 1.5 * TotalChargeTime
extern float CompensateVPC;
extern float Batt_Disconnect_VPC;    //EEP, min volt/cell

// ******************************************************************************************************
// *        FUNCTION PROTOTYPES
// ******************************************************************************************************

struct ABM_Status_Bits {
    // Always one of eAbmState
    uint16_t State              : 4;    
    
    uint16_t ChargerCmdOn       : 1;    
    uint16_t ChargerCmdOff      : 1;
    uint16_t OpportunityCharge  : 1;
    uint16_t LowLineChargerOff  : 1;
    
    // Momentary bit for a battery over-voltage alarm on this node.
    uint16_t BatteryOV          : 1;    
    uint16_t ABMEnable          : 1;     
    uint16_t BldInpChargerOff   : 1;    
    uint16_t Battery_Failure    : 1;    // An active battery failure condition is on

    uint16_t BST_Enable         : 1;
    uint16_t Enable_CT_Zero     : 1;
    uint16_t BTRInitDone        : 1;    
    uint16_t No_Battery         : 1;
    
    // Used to communicate that the battery DC OV alarm nodebit is set.  In
    // common battery, the slave UPM's use this status bit in lieu of directly
    // communicating the nodebit.
    uint16_t BatteryOVTrip      : 1;
    uint16_t DischargeLoadChanged   :1;  // Pan/20120130 add
    uint16_t BelowHEChargeLimit     :1;  // charge current is small, can go to HE
    uint16_t ManualChargerOff       :1;

    uint16_t BatterySetupRequired   :1;
    uint16_t unused                 :11;
    
    uint16_t word2;
    uint16_t /*unallocated*/: 12;
    
    uint16_t SequenceNumber: 4;
};

union uAbmStatus
{
    ABM_Status_Bits bit;
    uint16_t words[sizeof(ABM_Status_Bits)];
    
    void SetMasterBits(const uAbmStatus& master)
    {
        // Copy all bits from master except for the local-only bits
        words[0] = (words[0] & (1u << 8)) | (master.words[0] & ~(1u << 8));
        words[1] = master.words[1];
    }
};

class AbmActions
{
public:
    
    AbmActions();
    virtual ~AbmActions();

    // State machine interface
    virtual void Run( void ) = 0;
    
    // Command interface
    virtual void ChargerCmdOff() = 0;
    virtual void ChargerCmdOn() = 0;
    virtual void BldInpChargerOff(bool command) = 0;
    virtual void ResetAlarms() = 0;
    
    // Query interface
    virtual const uAbmStatus& GetStatus( void ) = 0;
    virtual eAbmState GetState() = 0;
    bool IsInternalCommonBattery( void )
    {   return ( CommonBattery == CommonBattery_Internal );     }

    // Set the battery OV timer time, in deciseconds
    void SetOVTime(uint16_t deciseconds);

    inline bool GetBatterySetupRequired(void)
    {
        return ABM_Status.bit.BatterySetupRequired;
    }

    inline void SetBatterySetupRequired(bool state)
    {
        ABM_Status.bit.BatterySetupRequired = state;
    }

    virtual void ClearChargerOffCmd( void)
    {
        ABM_Status.bit.ChargerCmdOff = 0;
        ABM_Status.bit.ManualChargerOff = 0;
    }

protected:
    // Default behavior common to both slaves and masters.  Inheriting base classes
    // should directly invoke these functions as the first thing they do.
    virtual void Charge_State( void );
    virtual void Reset_State( void );
    virtual void Float_State( void );
    virtual void Rest_State( void );
    virtual void Discharging_State( void );
    virtual void Transfer_State( eAbmState newState );
    bool BatteryOVSelf() const;
    
    uint32_t BatteryOVAlarmTimeout;
    uAbmStatus ABM_Status;
    StateTimer Timer1;

private:
    AbmActions(const AbmActions&);
    const AbmActions& operator=(const AbmActions&);
};


class AdvancedBatteryManager : public AbmActions
{
    public:
        AdvancedBatteryManager();
        virtual ~AdvancedBatteryManager();

    public:
        void Run( void );
        
        void EnableABM( void )
        {
            ABM_Status.bit.ABMEnable = 1;
        }
        
        virtual void ChargerCmdOn( void );
        
        virtual void ChargerCmdOff( void )
        {
            ABM_Status.bit.ChargerCmdOff = 1;
            ABM_Status.bit.ManualChargerOff = 1;
        }

        virtual void ResetAlarms( void );

        virtual void BldInpChargerOff( bool active )
        {   ABM_Status.bit.BldInpChargerOff = active;             }

        
        virtual const uAbmStatus& GetStatus( void )
        {
            return ABM_Status;
        }
        
        virtual eAbmState GetState( void )
        {
            return eAbmState(ABM_Status.bit.State);
        }

    protected:
        void Transfer_State( eAbmState newState );
        virtual void Charge_State( void );
        virtual void Reset_State( void );
        virtual void Float_State( void );
        virtual void Rest_State( void );
        virtual void Discharging_State( void );
        
        uint32_t Get_Total_State_Time( void );
        bool BatteryOV() const;
        bool CheckBatteryDischarge( void );

    public:
        eAbmState PreviousState;
     
    protected:

        StateTimer CurrentStateTime;     // time in the state after last state change
        StateTimer OCVDisableTimer;      // Disables rest mode OCV test
        float PreviousDischargeLoad;

        uint16_t ABM_Batt_Status;
        
    private:
        // ******************************************************************************************************
        // *        Local VARIABLES
        // ******************************************************************************************************
        uint16_t TotalChargeTime;      // charge time; how long charge mode lasted
        uint16_t TotalDischargeTime;   // cumulative discharge time (seconds); calculated internally by UPS
        uint32_t CumulativeStateTime;  // Cumulative state time for rest/float modes to keep track of time when interrupted
        uint16_t BSTCompletionPercent; // Percentage of completion of the Battery Support Test for displaying the status on UI
        int16_t  BattStartChrgTimer;   // delay after system normal before starting charger
        uint16_t BSTTestTime;          // 25% of expected run time in seconds
        
        StateTimer Timer2;
		StateTimer Timer3;
};

class SlaveAbm : public AbmActions
{
public:
    
    SlaveAbm();
    virtual ~SlaveAbm();

    // State machine interface
    virtual void Run( void );
    
    // Command interface
    virtual void ChargerCmdOff();
    virtual void ChargerCmdOn();
    virtual void BldInpChargerOff(bool command);
    virtual void ResetAlarms();
    
    // Query interface
    virtual const uAbmStatus& GetStatus( void );
    virtual eAbmState GetState();
    
protected:
    const uAbmStatus& masterStatus();
};


AbmActions& Abm();

// For debugging use only.  Non-debugger clients should go through the Abm()
// accessor function above.
extern AdvancedBatteryManager AbmMaster;

extern void CalTempCompensateVPC(uint16_t temperature);

// ******************************************************************************************************
// *        END OF ABM.H
// ******************************************************************************************************
#endif // __ABM_H
