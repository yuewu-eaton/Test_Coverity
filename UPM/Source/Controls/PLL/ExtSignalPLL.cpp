// ******************************************************************************************************
// *            ExtSignalPLL.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2011 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: ExtSignalPLL.cpp
// *
// *    DESCRIPTION: Locks sine ref to ext sync in
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/28/2011
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ExtSignalPLL.h"
#include "CriticalSection.h"
#include "BypassInterface.h"

ExtSignalPLL ExtSignalSync;

// debug
float BadFrequency = 0.0f;

// ***********************************************************************
// *
// *    FUNCTION: RunPLLFast  
// *
// *    DESCRIPTION: Updates angle, checks for sync in recieved 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void ExtSignalPLL::RunPLLFast()
{
    const float FrequencyGain = 0.015f;
    float MinFrequencyLimit = 0.6*OutNomFreq;
    float MaxFrequencyLimit = 1.4*OutNomFreq;
    
    if ( TimeStamp != 0 )
    {
        uint32_t tempTimeStamp;

        {
            CriticalSection enter;
            
            TempPhaseError = PhaseError;
            PhaseError = 0;
            tempTimeStamp = TimeStamp;
            TimeStamp = 0;
        }

        TargetFrequency = float( CPUFrequency ) / float( tempTimeStamp );

        // re-enable eCap2, clearing any pending interrupt flags first
        ECap2Regs.ECCLR.all = 0xff;
        ECap2Regs.ECEINT.bit.CEVT1 = 1;
        
        // limit check
        if ( ( TargetFrequency > MinFrequencyLimit ) &&
             ( TargetFrequency < MaxFrequencyLimit ) )
        {

            // get frequency error
            float frequencyCorrection = ( TargetFrequency - SineRef.Frequency ) * FrequencyGain;

            SineRef.UpdateFrequencyAndAngleStep( SineRef.Frequency + frequencyCorrection );
        
            SineRef.AngleCorrection = TempPhaseError * PhaseGain;
        }
        else
        {
            BadFrequency = TargetFrequency;
        }
    }
}

// ******************************************************************************************************
// *            End of ExtSignalPLL.cpp
// ******************************************************************************************************
