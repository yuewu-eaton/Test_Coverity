// ******************************************************************************************************
// *            ACMeter.cpp
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
// *    FILE NAME: ACMeter.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 8/21/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "Adc.h"
#include "ACMeter.h"
#include "MCUState.h"
#include "CriticalSection.h"
#include "Algos.h"
#include "InverterControl.h"
#include <cmath>

using namespace std;

// Matlab generated 1st order filter. Butterworth is used because ringing step response is undesirable. Designed more for step 
// response than frequency response, intended to be like the 4095/4096 IIR previously used
// Assume 60Hz sample rate, pass frequency = 1.5Hz, stop frequency = 0.0065Hz, 1db passband ripple, 30dB stopband attenuation.
//1order LPF, pass frequency = 0~0.2Hz, stop frequency 0.2Hz/ -3db,  60Hz ripple about ~30dB attenuation
const stFirstOrderIIRFP RMSFilterCoefficients = { 0.010642605011813414, 1.0, -0.97871478997637318, 0 };

ACMeter InverterVoltageRMS( ( stThreePhase* )&RawAdcDataPtr->st.InverterVoltage );
ACMeter InverterCurrentRMS( ( stThreePhase* )&RawAdcDataPtr->st.InverterCurrent );

ACMeter OutputVoltageRMS( ( stThreePhase* )&RawAdcDataPtr->st.OutputVoltageRaw );
// Note: actually updated at 2.5 kHz.  The filter constants for this meter should
// be revisited if filtered RMS values are ever used.
ACMeter OutputCurrentRMS( &OutputCurrent);

ACMeter UtilityVoltageRMS( ( stThreePhase* )&RawAdcDataPtr->st.InputVoltage );
ACMeter UtilityCurrentRMS( ( stThreePhase* )&RawAdcDataPtr->st.InputCurrent );

ACMeter BypassVoltageRMS( ( stThreePhase* )&RawAdcDataPtr->st.BypassVoltage );
ACMeter BypassVoltage2RMS( ( stThreePhase* )&RawAdcDataPtr->st.BypassVoltage2 );
ACMeter BypassCurrentRMS( ( stThreePhase* )&RawAdcDataPtr->st.BypassCurrent );
ACMeter ChassisVoltageRMS( ( stThreePhase* )&RawAdcDataPtr->st.ChassisVoltage );

stThreePhase OutputReactiveAmps;

// ********************************************************************
// * FILE VARIABLES
// ********************************************************************
static uint16_t SplCount;
static stThreePhase OutputReactiveSum;                  // temporary calculation

ACMeter::ACMeter( stThreePhase* source )
{
    RawSourceData = source;
    RawSumOfSquares.phA = 0;
    RawSumOfSquares.phB = 0;
    RawSumOfSquares.phC = 0;
    RawRMS.phA = 0;
    RawRMS.phB = 0;
    RawRMS.phC = 0;
    FilteredRMS.phA = 0;
    FilteredRMS.phB = 0;
    FilteredRMS.phC = 0;
    Count = 0;

    PhaseAFilterTable = RMSFilterCoefficients;
    PhaseBFilterTable = RMSFilterCoefficients;
    PhaseCFilterTable = RMSFilterCoefficients;
}

// ***********************************************************************
// *
// *    FUNCTION: ProcessRawAC 
// *
// *    DESCRIPTION: Updates the sum of squares
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void ACMeter::ProcessRawAC( void )
{
    // Note: It is the caller's responsibility to check if these should be run!
    
    RawSumOfSquares.phA += ( RawSourceData->phA * RawSourceData->phA );
    RawSumOfSquares.phB += ( RawSourceData->phB * RawSourceData->phB );
    RawSumOfSquares.phC += ( RawSourceData->phC * RawSourceData->phC );
    ++Count;
}

// ***********************************************************************
// *
// *    FUNCTION: CalculateRMS 
// *
// *    DESCRIPTION: Calculates RMS from last sum of squares and sample count
// *                 Runs filter on RMS for metering purposes.
// *                 This is intended to be called once per line cycle
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ACMeter::CalculateRMS( void )
{
    if ( 0 != Count )
    {
        stThreePhase tempSumOfSquares;
        uint32_t     tempCount;
        
        {
            // copy sum of squares data to temp and zero sum of squares
            CriticalSection enter( IER_DMA_ONLY );
            
            tempSumOfSquares.phA = RawSumOfSquares.phA;
            tempSumOfSquares.phB = RawSumOfSquares.phB;
            tempSumOfSquares.phC = RawSumOfSquares.phC;
            tempCount = Count;
            
            RawSumOfSquares.phA = 0;
            RawSumOfSquares.phB = 0;
            RawSumOfSquares.phC = 0;
            Count = 0;
        }
        
        // divide by number of samples and take the square root
        RawRMS.phA = sqrt( tempSumOfSquares.phA / float( tempCount ) );
        RawRMS.phB = sqrt( tempSumOfSquares.phB / float( tempCount ) );        
        RawRMS.phC = sqrt( tempSumOfSquares.phC / float( tempCount ) );

		FilteredRMS.phA = FirstOrderIIRFP( RawRMS.phA, &PhaseAFilterTable );
		FilteredRMS.phB = FirstOrderIIRFP( RawRMS.phB, &PhaseBFilterTable );
		FilteredRMS.phC = FirstOrderIIRFP( RawRMS.phC, &PhaseCFilterTable );		 
                
    }
}

// ***********************************************************************
// *    FUNCTION:       ProcessOutReactiveCurrent
// *    DESCRIPTION:    Updates the sum of Reactive Current
// *    ARGUMENTS:      none
// *    RETURNS:        nothing
// ***********************************************************************
void ProcessOutReactiveCurrent(void)
{
    float cos_th = 0.0f;
    float cos_thMINUS120 = 0.0f;
    float cos_thPLUS120 = 0.0f;
    
    cos_3_phase(Inverter.GetAngle(), &cos_th, &cos_thMINUS120, &cos_thPLUS120);
    OutputReactiveSum.phA += RawAdcDataPtr->st.InverterCurrent.phA * cos_th;
    OutputReactiveSum.phB += RawAdcDataPtr->st.InverterCurrent.phB * cos_thMINUS120;
    OutputReactiveSum.phC += RawAdcDataPtr->st.InverterCurrent.phC * cos_thPLUS120;
    SplCount++;
}

// ***********************************************************************
// *    FUNCTION:       CalculateOutReactiveCurrent 
// *    DESCRIPTION:    Calculates output reactive current of last line cyale
// *                    from the sum of reactive current
// *    ARGUMENTS:      None
// *    RETURNS:        None
// ***********************************************************************
void CalculateOutReactiveCurrent(void)
{
    if( 0!= SplCount )
    {
        stThreePhase OutputReactiveAmpstemp;
        uint16_t countTemp = SplCount;
        
        {
            CriticalSection enter( IER_DMA_ONLY );
            SplCount = 0;
            OutputReactiveAmpstemp.phA = OutputReactiveSum.phA;
            OutputReactiveAmpstemp.phB = OutputReactiveSum.phB;
            OutputReactiveAmpstemp.phC = OutputReactiveSum.phC;
            OutputReactiveSum.phA = 0.0f;   // zero react values
            OutputReactiveSum.phB = 0.0f;   // zero react values
            OutputReactiveSum.phC = 0.0f;   // zero react values
        }
        OutputReactiveAmps.phA = OutputReactiveAmpstemp.phA/float(countTemp);
        OutputReactiveAmps.phB = OutputReactiveAmpstemp.phB/float(countTemp);
        OutputReactiveAmps.phC = OutputReactiveAmpstemp.phC/float(countTemp);
    }
}

// ******************************************************************************************************
// *            End of ACMeter.cpp
// ******************************************************************************************************
