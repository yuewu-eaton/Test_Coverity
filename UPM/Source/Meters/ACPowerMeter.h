#ifndef _ACPOWERMETER_H
#define _ACPOWERMETER_H
// ******************************************************************************************************
// *            ACPowerMeter.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: ACPowerMeter.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 7/13/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#include "Constants.h"
#include "Meters.h"
#include "ACMeter.h"        // for filter coefficients

class ACPowerMeter
{
    public:
        ACPowerMeter();
        ~ACPowerMeter()
        {
        }

    private:
        // non-copyable, non-assignable
        ACPowerMeter( const ACPowerMeter& a );
        const ACPowerMeter& operator=( const ACPowerMeter& a );

    public:
        void ProcessPower( volatile const stThreePhase* voltage, volatile const stThreePhase* current );
        void CalculatePower( const stThreePhase& rmsVoltage, const stThreePhase& rmsCurrent);
        void ClearPower( void );
        void SetActivePower( const stAC_Power& ref);
        void DividePower(float nUpms);

        stAC_Power          ActivePower;
        stAC_Power          ActivePowerFiltered;
        stAC_Power          ReactivePower;
        stAC_Power          ReactivePowerFiltered;
        stAC_Power          TotalPower;
        stAC_Power          TotalPowerFiltered;
        
        stThreePhase        LastVoltage;

    protected:
        stThreePhase        ActivePowerSample;
		stFirstOrderIIRFP	PowerFilterPhaseA;
		stFirstOrderIIRFP	PowerFilterPhaseB;
		stFirstOrderIIRFP	PowerFilterPhaseC;
		stFirstOrderIIRFP	VARFilterPhaseA;
		stFirstOrderIIRFP	VARFilterPhaseB;
		stFirstOrderIIRFP	VARFilterPhaseC;
		stFirstOrderIIRFP	TotalFilterPhaseA;
		stFirstOrderIIRFP	TotalFilterPhaseB;
		stFirstOrderIIRFP	TotalFilterPhaseC;
        
        uint32_t            SampleCount;

};

extern ACPowerMeter InverterPower;
extern ACPowerMeter RectifierPower;
extern ACPowerMeter BypassPower;
extern ACPowerMeter LoadPower;


// ******************************************************************************************************
// *            End of ACPowerMeter.h
// ******************************************************************************************************
#endif // _ACPOWERMETER_H
