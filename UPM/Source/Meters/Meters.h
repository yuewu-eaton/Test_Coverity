// ********************************************************************************************************
// *            ProcessAC.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *    Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: ProcessAC.h
// *
// *    DESCRIPTION: RMS and power measurements
// *
// *    ORIGINATOR: Jason Anderson
// *
// *    DATE: 2/19/2003
// *
// *    HISTORY: See Visual Source Safe history.
// ********************************************************************************************************
#ifndef _METERS_H
#define _METERS_H

// *********************************************************************************************************
// *        System Control structure and declaration
// *********************************************************************************************************
#include "Constants.h"
#include "Algos.h"

typedef struct
{
    float phA;
    float phB;
    float phC;
    float sum;
    
    inline void UpdateSum(void)
    {
        sum = phA + phB + phC;
    }
} stAC_Power;

/*
 * Divide an stAC_Power object as a three-vector quantity by a scalar.
 */
stAC_Power operator/(const stAC_Power& lhs, float divisor);

typedef struct
{
    float               FastFiltered;
    float               SlowFiltered;
    stSecondOrderIIRFP  Filter5ms;
    stSecondOrderIIRFP  Filter250ms;
} stDC_Meter;


typedef struct
{
    stThreePhase InputVoltageRMS;
    stThreePhase InverterVoltageRMS;
    stThreePhase OutputVoltageRMS;
    stThreePhase BypassVoltageRMS;
    stThreePhase BypassVoltage2RMS;

    stThreePhase InputVoltageLL_RMS;
    stThreePhase InverterVoltageLL_RMS;
    stThreePhase OutputVoltageLL_RMS;
    stThreePhase BypassVoltageLL_RMS;

    stThreePhase InputCurrentRMS;
    stThreePhase InverterCurrentRMS;
    stThreePhase BypassCurrentRMS;
    stThreePhase OutputCurrentRMS;

    stAC_Power   InputPower;
    stAC_Power   OutputPower;
    stAC_Power   InputVoltAmperes;
    stAC_Power   OutputVoltAmperes;
    stAC_Power   BypassPower;
    stAC_Power   BypassVoltAmperes;
    stAC_Power   PercentLoadWatts;
    stAC_Power   PercentLoadVA;
    stAC_Power   PercentLoad;

    float        InputPowerFactor;
    float        OutputPowerFactor;
    float        BypassPowerFactor;

    float        InputFrequency;
    float        OutputFrequency;
    float        BypassFrequency;
    float        InverterFrequency;

    float        BatteryVoltage;
    float        BatteryVoltagePos;
    float        BatteryVoltageNeg;
    float        BatteryVPC;
    float        BatteryCurrent;
    float        BatteryCurrentP_Sum;
    float        BatteryCurrentN_Sum;
    float        BatteryCurrentAvg;
    float        PercentBatteryRemaining;
    float        AmbientTemperature;
    float        DCLinkVoltage;

    float        CalculatedBTR;
    uint16_t     BatteryFloatTime;

    uint32_t     HourMeter;
    uint32_t     kWHMeter;
    
    // Rating meters
    float       InputVoltageRating;
    float       OutputVoltageRating;
    float       InputVoltageMinRating;
    float       InputVoltageMaxRating;
    float       InputFrequencyRating;
    float       OutputFrequencyRating;
    float       OutputPowerFactorRating;
    float       OutputkVARating;
    float       BatteryTimeRating;
    float       BatteryLoadRating;

} stScreenMeters;       

// DC voltages
extern volatile stDC_Meter DCLinkVoltagePositive;
extern volatile stDC_Meter DCLinkVoltageNegative;
extern volatile stDC_Meter BatteryVoltage;
extern volatile stDC_Meter BatteryVoltagePos;
extern volatile stDC_Meter BatteryVoltageChgPos;
extern volatile stDC_Meter BatteryVoltageChgNeg;
extern volatile stDC_Meter BatteryVoltageBU;
extern volatile stDC_Meter BatteryCurrentNeg;
extern volatile stDC_Meter BatteryCurrentPos;
extern volatile stDC_Meter BatteryCurrentPos_LegB;
extern volatile stDC_Meter BatteryCurrentNeg_LegB;
extern volatile stDC_Meter InverterDCphA;
extern volatile stDC_Meter InverterDCphB;
extern volatile stDC_Meter InverterDCphC;
extern volatile stDC_Meter InputNeutral;
extern float BatteryVoltageNeg_SlowFiltered;       


// global meters data
extern stScreenMeters ScreenMeters;

// ********************************************************************************************************
// * FUNCTION PROTOTYPES
// ********************************************************************************************************

void ProcessDC( volatile float dcRaw, stDC_Meter* ptrDc );

void ProcessACScreenMeters( void );

// ********************************************************************************************************
// *            END OF ProcessAC.h
// ********************************************************************************************************
#endif

