// ******************************************************************************************************
// *            ACPowerMeter.cpp
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
// *    FILE NAME: ACPowerMeter.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 7/13/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "Constants.h"
#include "ACPowerMeter.h"
#include "C28x_FPU_FastRTS.h"
#include "MCUState.h"

using namespace std;

ACPowerMeter InverterPower;
ACPowerMeter RectifierPower;
ACPowerMeter BypassPower;
ACPowerMeter LoadPower;

// ***********************************************************************
// *
// *    FUNCTION: ProcessPower  
// *
// *    DESCRIPTION: Integrates product of voltage and current 
// *
// *    ARGUMENTS: pointers to voltage and current
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void ACPowerMeter::ProcessPower( volatile const stThreePhase* voltage, volatile const stThreePhase* current )
{
    if ( MCUStateMachine.GetADCReadyFlag() )
    {
    	LastVoltage.phA = voltage->phA;
    	LastVoltage.phB = voltage->phB;
    	LastVoltage.phC = voltage->phC;
        ActivePowerSample.phA += ( LastVoltage.phA * current->phA );
        ActivePowerSample.phB += ( LastVoltage.phB * current->phB );
        ActivePowerSample.phC += ( LastVoltage.phC * current->phC );
        ++SampleCount;    
    }
    
}

// ***********************************************************************
// *
// *    FUNCTION: CalculatePower 
// *
// *    DESCRIPTION: Calculates power from the integrated samples. Should be called
// *                 at the zero cross.
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none
// *
// ***********************************************************************
void ACPowerMeter::CalculatePower( const stThreePhase& rmsVoltage, const stThreePhase& rmsCurrent)
{
    stThreePhase activeSums;
    uint32_t     lastcount;
    
    if ( 0 != SampleCount )
    {
        {
            // copy sum of squares data to temp and zero sum of squares
            CriticalSection enter( IER_DMA_ONLY );
            
            activeSums.phA = ActivePowerSample.phA;
            activeSums.phB = ActivePowerSample.phB;
            activeSums.phC = ActivePowerSample.phC;
            lastcount = SampleCount;
            
            ActivePowerSample.phA = 0;
            ActivePowerSample.phB = 0;
            ActivePowerSample.phC = 0;
            SampleCount = 0;
        }     
        
        // calculate watts (resistive power)
        ActivePower.phA = activeSums.phA / float( lastcount );
        ActivePower.phB = activeSums.phB / float( lastcount );
        ActivePower.phC = activeSums.phC / float( lastcount );
              
        // calculate VA
        TotalPower.phA = rmsVoltage.phA * rmsCurrent.phA;
        TotalPower.phB = rmsVoltage.phB * rmsCurrent.phB;
        TotalPower.phC = rmsVoltage.phC * rmsCurrent.phC;
               
        // calculate VARs, may be useful for parallel autocal. Maybe
        if ( TotalPower.phA > ActivePower.phA )
        {
            ReactivePower.phA = sqrt( ( TotalPower.phA * TotalPower.phA ) - ( ActivePower.phA * ActivePower.phA ) );
        }
        else
        {
            ReactivePower.phA = 0;
        }
        
        if ( TotalPower.phB > ActivePower.phB )
        {
            ReactivePower.phB = sqrt( ( TotalPower.phB * TotalPower.phB ) - ( ActivePower.phB * ActivePower.phB ) );
        }
        else
        {
            ReactivePower.phB = 0;
        }
        
        if ( TotalPower.phC > ActivePower.phC )
        {
            ReactivePower.phC = sqrt( ( TotalPower.phC * TotalPower.phC ) - ( ActivePower.phC * ActivePower.phC ) );
        }
        else
        {
            ReactivePower.phC = 0;
        }
        
        // run filter
		ReactivePowerFiltered.phA = FirstOrderIIRFP( ReactivePower.phA, &VARFilterPhaseA );
		ReactivePowerFiltered.phB = FirstOrderIIRFP( ReactivePower.phB, &VARFilterPhaseB );
		ReactivePowerFiltered.phC = FirstOrderIIRFP( ReactivePower.phC, &VARFilterPhaseC );

		ActivePowerFiltered.phA = FirstOrderIIRFP( ActivePower.phA, &PowerFilterPhaseA );
		ActivePowerFiltered.phB = FirstOrderIIRFP( ActivePower.phB, &PowerFilterPhaseB );
		ActivePowerFiltered.phC = FirstOrderIIRFP( ActivePower.phC, &PowerFilterPhaseC );

		TotalPowerFiltered.phA = FirstOrderIIRFP( TotalPower.phA, &TotalFilterPhaseA );
		TotalPowerFiltered.phB = FirstOrderIIRFP( TotalPower.phB, &TotalFilterPhaseB );
		TotalPowerFiltered.phC = FirstOrderIIRFP( TotalPower.phC, &TotalFilterPhaseC );
        
        ActivePower.UpdateSum();
        ActivePowerFiltered.UpdateSum();
        ReactivePower.UpdateSum();
        ReactivePowerFiltered.UpdateSum();
        TotalPower.UpdateSum();
        TotalPowerFiltered.UpdateSum();
    }    
}

// ***********************************************************************
// *
// *    FUNCTION: ClearPower 
// *
// *    DESCRIPTION: Zeroes out the power meters
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void ACPowerMeter::ClearPower( void )
{
    ActivePower.phA = 0;
    ActivePower.phB = 0;
    ActivePower.phC = 0;
    ActivePower.sum = 0;
    
    ActivePowerFiltered.phA = 0;
    ActivePowerFiltered.phB = 0;
    ActivePowerFiltered.phC = 0;
    ActivePowerFiltered.sum = 0;
    
    ReactivePower.phA = 0;
    ReactivePower.phB = 0;
    ReactivePower.phC = 0;
    ReactivePower.sum = 0;
    
    ReactivePowerFiltered.phA = 0;
    ReactivePowerFiltered.phB = 0;
    ReactivePowerFiltered.phC = 0;
    ReactivePowerFiltered.sum = 0;
    
    TotalPower.phA = 0;
    TotalPower.phB = 0;
    TotalPower.phC = 0;
    TotalPower.sum = 0;
    
    TotalPowerFiltered.phA = 0;
    TotalPowerFiltered.phB = 0;
    TotalPowerFiltered.phC = 0;
    TotalPowerFiltered.sum = 0;
    
    PowerFilterPhaseA.X1 = 0;
    PowerFilterPhaseB.X1 = 0;
    PowerFilterPhaseC.X1 = 0;
    
    VARFilterPhaseA.X1 = 0;
    VARFilterPhaseB.X1 = 0;
    VARFilterPhaseC.X1 = 0;
    
    TotalFilterPhaseA.X1 = 0;
    TotalFilterPhaseB.X1 = 0;
    TotalFilterPhaseC.X1 = 0;
    
    CriticalSection enter( IER_DMA_ONLY );
    SampleCount = 0;
    ActivePowerSample.phA = 0;
    ActivePowerSample.phB = 0;
    ActivePowerSample.phC = 0;
    
    LastVoltage.phA = 0;
    LastVoltage.phB = 0;
    LastVoltage.phC = 0;
}

// ***********************************************************************
// *
// *    FUNCTION: DividePower 
// *
// *    DESCRIPTION: Divides the power meters by an integer N
// *
// *    ARGUMENTS: nUpms the number of UPM's to divide by
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void
ACPowerMeter::DividePower(float nUpms)
{
	float factor = 1.0f/nUpms;
	
	{
		CriticalSection enter( IER_DMA_ONLY );
	    ActivePowerSample.phA *= factor;
    	ActivePowerSample.phB *= factor;
    	ActivePowerSample.phC *= factor;
	}
	
	float *meters[] = {
	    &ActivePower.phA,
	    &ActivePower.phB,
	    &ActivePower.phC,
	    &ActivePowerFiltered.phA,
	    &ActivePowerFiltered.phB,
	    &ActivePowerFiltered.phC,
	    &ReactivePower.phA,
	    &ReactivePower.phB,
	    &ReactivePower.phC,
	    &ReactivePowerFiltered.phA,
	    &ReactivePowerFiltered.phB,
	    &ReactivePowerFiltered.phC,
	    &TotalPower.phA,
	    &TotalPower.phB,
	    &TotalPower.phC,
	    &TotalPowerFiltered.phA,
	    &TotalPowerFiltered.phB,
	    &TotalPowerFiltered.phC,
	    &PowerFilterPhaseA.X1,
	    &PowerFilterPhaseB.X1,
	    &PowerFilterPhaseC.X1,
	    &VARFilterPhaseA.X1,
	    &VARFilterPhaseB.X1,
	    &VARFilterPhaseC.X1,
	    &TotalFilterPhaseA.X1,
	    &TotalFilterPhaseB.X1,
	    &TotalFilterPhaseC.X1,
	};
	for (unsigned i = 0; i < sizeof(meters) / sizeof(meters[0]); ++i)
	{
		*meters[i] *= factor;
	}
	
    ActivePower.UpdateSum();
    ActivePowerFiltered.UpdateSum();
    ReactivePower.UpdateSum();
    ReactivePowerFiltered.UpdateSum();
    TotalPower.UpdateSum();
    TotalPowerFiltered.UpdateSum();
}

// ***********************************************************************
// *
// *    FUNCTION: SetActivePower 
// *
// *    DESCRIPTION: Set the active power meters to the provided value, and
// *                 back-initialize the filters to the same.
// *
// *    ARGUMENTS: ref The new power value.
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void ACPowerMeter::SetActivePower( const stAC_Power& ref)
{
	ActivePower = ref;
	ActivePowerFiltered = ref;
	FirstOrderIIRFPBackInitialize(ActivePower.phA, &PowerFilterPhaseA);
	FirstOrderIIRFPBackInitialize(ActivePower.phB, &PowerFilterPhaseB);
	FirstOrderIIRFPBackInitialize(ActivePower.phC, &PowerFilterPhaseC);
}

ACPowerMeter::ACPowerMeter()
{
    PowerFilterPhaseA = RMSFilterCoefficients;
    PowerFilterPhaseB = RMSFilterCoefficients;
    PowerFilterPhaseC = RMSFilterCoefficients;
    VARFilterPhaseA = RMSFilterCoefficients;
    VARFilterPhaseB = RMSFilterCoefficients;
    VARFilterPhaseC = RMSFilterCoefficients;
    TotalFilterPhaseA = RMSFilterCoefficients;
    TotalFilterPhaseB = RMSFilterCoefficients;
    TotalFilterPhaseC = RMSFilterCoefficients;
    
    ClearPower();
}

// ******************************************************************************************************
// *            End of ACPowerMeter.cpp
// ******************************************************************************************************
