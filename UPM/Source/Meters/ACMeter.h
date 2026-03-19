// ******************************************************************************************************
// *            ACMeter.h
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
// *    FILE NAME: ACMeter.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 8/21/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _ACMETER_H
#define _ACMETER_H

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "Algos.h"

extern const stFirstOrderIIRFP RMSFilterCoefficients;

class ACMeter
{
    public:
        ACMeter( stThreePhase* source );
        ~ACMeter()
        {
        }

    private:
        // non-copyable, non-assignable
        ACMeter( const ACMeter& );
        const ACMeter& operator=( const ACMeter& )
        {
            return *this;
        }

    public:
        void ProcessRawAC( void );
        void CalculateRMS( void );

    public:
        stThreePhase        RawRMS;
        stThreePhase        FilteredRMS;

    protected:
        stThreePhase*       RawSourceData;
        stThreePhase        RawSumOfSquares;
		stFirstOrderIIRFP	PhaseAFilterTable;
		stFirstOrderIIRFP	PhaseBFilterTable;
		stFirstOrderIIRFP	PhaseCFilterTable;
        uint32_t            Count;
};

/*
 * Compute the direct sum of each phase of a three-phase variable.
 */
inline float sum(const stThreePhase& source)
{
	return source.phA + source.phB + source.phC;
}

extern ACMeter InverterVoltageRMS;
extern ACMeter InverterCurrentRMS;

extern ACMeter OutputVoltageRMS;
extern ACMeter OutputCurrentRMS;

extern ACMeter UtilityVoltageRMS;
extern ACMeter UtilityCurrentRMS;

extern ACMeter BypassVoltageRMS;
extern ACMeter BypassVoltage2RMS;
extern ACMeter BypassCurrentRMS;
extern ACMeter ChassisVoltageRMS;

extern stThreePhase OutputReactiveAmps;

extern void ProcessOutReactiveCurrent(void);
extern void CalculateOutReactiveCurrent(void);
#endif
// ******************************************************************************************************
// *            End of ACMeter.h
// ******************************************************************************************************
