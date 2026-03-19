// ******************************************************************************************************
// *            DQPhaseLockLoop.cpp
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
// *    FILE NAME: DQPhaseLockLoop.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 4/27/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cmath>
#include "Constants.h"
#include "DQPhaseLockLoop.h"
#include "C28x_FPU_FastRTS.h"
#include "Adc.h"
#include "MCUState.h"
#include "DebuggerBlocks.h"

using namespace std;

// global PLLs
DQPhaseLockLoop BypassPLL( SLEW_RATE_20_HZ, &RawAdcDataPtr->st.BypassVoltage, ( ( float(5.0) * PI ) / 180.0 ) ); //5 degree default
DQPhaseLockLoop OutputPLL( SLEW_RATE_LUDICROUS, &RawAdcDataPtr->st.OutputVoltageRaw, ( ( float(5.0) * PI ) / 180.0 ) );


const stFirstOrderIIRFP SlewRateGains[] =
{
	//B0			B1		A1	  X1
	    { 0.0010125,    -0.9995, -1.0, 0 },                   // ~10Hz/s
	    { 0.0028275,    -0.983264,  -1.0, 0 },                // ~20Hz/s--
	    { 0.0070684,    -0.983347,  -1.0, 0 }                 // ~They've gone plaid...
};

float utilityPhaseError = 0;
float utilityPhaseErrorLimit = 0;
// ***********************************************************************
// *
// *    FUNCTION: RunPLLFast  
// *
// *    DESCRIPTION: Updates source angle, calculates and applies phase correction
// *                 fast call from HWI, 2551Hz rate 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void DQPhaseLockLoop::RunPLLFast( void )
{
    if ( MCUStateMachine.GetADCReadyFlag() )
    {
        // copy volatile source data
        std::memcpy( &SourceVoltage, (const void*)SyncSource, sizeof( stThreePhase ) );
    
        // run abc-dq0 transform
        abc_to_dq0( &SourceVoltage, &SourceDQOData, SineRef.Angle );    
		SourceDQODataSlowSd = FirstOrderIIRFP( SourceDQOData.Sd, &SdFilter );

        // normalize data
        SourceDQONorm.Sd = SourceDQOData.Sd * SourceNormalizationFactor;
        SourceDQONorm.Sq = SourceDQOData.Sq * SourceNormalizationFactor;
        SourceDQONorm.S0 = SourceDQOData.S0 * SourceNormalizationFactor;
    
        // nominal data for UV/OV detection
        SourceNominalDQO.Sd = SourceDQOData.Sd * NominalNormalizationFactor;
        SourceNominalDQO.Sq = SourceDQOData.Sq * NominalNormalizationFactor;
        SourceNominalDQO.S0 = SourceDQOData.S0 * NominalNormalizationFactor;
    
        // run compensator, Sq is error 
        SineRef.AngleCorrection = FirstOrderIIRFP( SourceDQONorm.Sq, &PllCompensator ); 
    
        // integrate the phase correction for frequency measurement later
        PhaseIntegral += SineRef.AngleCorrection;

        // using small angle approximation sin(x) ~= x which implies qNorm ~= phase angle
        // if phase angle is small, valid since that's what we are looking for, small angle
        PhaseError = SecondOrderIIRFP( SourceDQONorm.Sq, &PhaseErrorFilter );

        utilityPhaseError = fabs( PhaseError );
        utilityPhaseErrorLimit = PLLMaxPhaseError;

        if ( SourceNormalizationFactor < MinNormMagnitude )
        {
            // check for phase lock, error < 5deg
            if ( fabs( PhaseError ) < PLLMaxPhaseError )
            {
                Locked.Debounce( true );
            }
            else
            {
                Locked.Debounce( false );
            }
        }
        else
        {
            Locked.Debounce( false );
        }
        
        InnerLoopCount++;
    }          
}

// ***********************************************************************
// *
// *    FUNCTION: PLLPeriodicFunction  
// *
// *    DESCRIPTION: updates norm factor and frequency. Called every
// *                 20ms if you haven't figured that out already 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void DQPhaseLockLoop::PLLPeriodicFunction( void )
{
    if ( MCUStateMachine.GetMetersReadyFlag() )
    {
        float tempPhaseIntegral;
        uint16_t tempCount;

        // get the integrated phase, and number of samples
        {
            CriticalSection enter( IER_DMA_ONLY );
            tempPhaseIntegral = PhaseIntegral;
            PhaseIntegral = 0;
            tempCount = InnerLoopCount;
            InnerLoopCount = 0;
        }

        // PLL output applied 8x since angle is update 18kHz, PLL runs 2250Hz
      //  tempPhaseIntegral = 8.0 * tempPhaseIntegral; 	//8=18k/2.25k
        // PLL output applied 8x since angle is update 13kHz, PLL runs 2166Hz
        tempPhaseIntegral = 6.0 * tempPhaseIntegral; 	//6=13k/2.166k
//	        tempPhaseIntegral = 12.0 * tempPhaseIntegral; 	//6=13k/2.166k

        // normalization factor is 1 / magnitude. Assuming no zero-sequence,
        // source magnitude = sqrt( Sd^2 + Sq^2 )
        float tempMag = SourceDQOData.Sd * SourceDQOData.Sd;
        tempMag += ( SourceDQOData.Sq * SourceDQOData.Sq );
        if ( tempMag > 0 )
        {
            tempMag = isqrt( tempMag );
        }
        else
        {
            tempMag = 1.0;
        }        
    
        
        if ( tempMag > MinNormMagnitude )
        {
            // signal too low, zero out norm factor to 'disable' PLL and zero out frequency
            tempMag = 0;
            SineRef.UpdateFrequency( 0.0f );
            Locked.SetState( false );
            PhaseRotationError = false;
        
            CriticalSection enter( IER_DMA_ONLY );
            PllCompensator.X1 = 0;
        }
        else
        {    
            // phase integral represents the total phase adjustment made off 55Hz sine the last time in radians, 
            // to change to frequency, need to multiply by 2250 / count (our sample frequency) and divide by 2pi (radians/sec to Hz)
            // 2250/2pi = 358.099      //Delta_f =Delta_W/2pi =Theta_step*2.25k/(2pi)
        	// 2166/2pi = 344.90
            float measuredFrequency = DQPLLBaseFrequency + ( tempPhaseIntegral * ( 344.90 / float(tempCount) )  );
			
            // update measurement data only
            SineRef.UpdateFrequency( measuredFrequency );
        
            // check phase rotation
            if ( Locked.GetState() )
            {
            	if(false == PhaseRotationError )
            	{
               		if ( SineRef.Frequency < -5 )
            		{
            			if( ++PhaseRotationErrorCnt > 3)// fix jira APACTS-380, add filter
            			{
            				PhaseRotationErrorCnt = 0;
            				PhaseRotationError = true;
            			} 
						else//clear PLL, re-lock.fix jira APACTS-380
						{
                			SineRef.UpdateFrequency( 0.0f );
                	    	Locked.SetState( false );
                	    	CriticalSection enter( IER_DMA_ONLY );
                	    	PllCompensator.X1 = 0;
						}           			
            		}
            		else
            		{
						PhaseRotationErrorCnt = 0;
					}
				}
				else
				{
            		if ( SineRef.Frequency > 5.0f )
            		{
            			if( ++PhaseRotationErrorCnt >= 10)
            			{
            				PhaseRotationErrorCnt = 0;
            				PhaseRotationError = false;
            			}
            		}
            		else
            		{
            			PhaseRotationErrorCnt = 0;
            		}					
				}  
            }
        }  
        // load
        SourceNormalizationFactor = tempMag;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: SetNominalSourceVoltage 
// *
// *    DESCRIPTION: Used to set Vnom for normalization purposes, typically
// *                 called when somebody writes the EEPROM 
// *
// *    ARGUMENTS: int16_t voltage in rms volts * 10
// *
// *    RETURNS: 
// *
// ***********************************************************************
void DQPhaseLockLoop::SetNominalSourceVoltage( int16_t voltage )
{
    // convert fixed point *10 to float
    float temp = (float)voltage / 10.0;
    // rms to magnitude
    temp *= SQRT_2;
    // invert
    temp = 1.0 / temp;
    // load

    NominalNormalizationFactor = temp;
}
// ***********************************************************************
// *
// *    FUNCTION: IsDQGoodForSync 
// *
// *    DESCRIPTION: Used to check bypass good for sync
// *
// *    ARGUMENTS: DQ
// *
// *    RETURNS: 
// *
// ***********************************************************************
bool DQPhaseLockLoop::IsDQGoodForSync(void)
{
  	 return ( Locked.GetState() && ( SourceNominalDQO.Sd > PLLMinDq ) ); 
}
// ******************************************************************************************************
// *            End of PhaseLockLoop.cpp
// ******************************************************************************************************
