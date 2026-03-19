// ******************************************************************************************************
// *            DQPhaseLockLoop.h
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
// *    FILE NAME: DQPhaseLockLoop.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 4/27/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _PHASELOCKLOOP_H
#define _PHASELOCKLOOP_H

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "algos.h"
#include "SineReference.h"
#include "FilteredBit.h"
#include "NB_Config.h"

enum dq0_slew_rate_t
{
    SLEW_RATE_10_HZ = 0,            // 20kHz update rate
    SLEW_RATE_20_HZ,                // 20kHz update rate
    SLEW_RATE_LUDICROUS,            // 20kHz update rate
    MAX_SLEW_RATE
};

namespace
{
    const float PLLTenDegrees           = ( ( 10.0 * PI ) / 180.0 );        // 10 degrees
    const float DQPLLBaseFrequency      = 55.0;                             // 55Hz base frequency
    const float MinNormMagnitude        = 0.025;                            // check for low or missing signal, 0.025 = ~30V in 120V unit or ~60V in 230V unit
    const stSecondOrderIIRFP PhaseErrorFilterCoefficients = { 0.19926, -1.75727, 1.0, -1.74706, 0.79543, 0, 0 };
    const stFirstOrderIIRFP SdFilterCoefficients = { 0.071833, -0.06127, -0.9326, 0 };
}

extern const stFirstOrderIIRFP SlewRateGains[];


class DQPhaseLockLoop
{
    public:
        DQPhaseLockLoop( dq0_slew_rate_t slewRate, volatile const stThreePhase* source, float PLLPhaseError )
            : Locked( PASS_32, PASS_4 ), SineRef( NOM_FREQUENCY_55HZ, PLL_Ts ) 
        {
            // initialize PLL gains
            if ( slewRate < MAX_SLEW_RATE )
            {
                PllCompensator = SlewRateGains[ uint16_t(slewRate) ];
            }
            else
            {
                // default to 1Hz/sec if invalid
                PllCompensator = SlewRateGains[0];
            }
            
            // set source
            SyncSource = source; 
            
            // initialize data
            PhaseError = 0;
            PhaseErrorOffset = 0;
            LoadSharePhaseCal = 0;
            PhaseIntegral = 0;
            SourceNormalizationFactor = 1.0 / 325.22; // initialize to 230         
            NominalNormalizationFactor = 1.0 / 325.22; 
            InnerLoopCount = 0;
            PhaseRotationError = false;  
            PhaseErrorFilter = PhaseErrorFilterCoefficients;    
            SdFilter = SdFilterCoefficients;						
            PLLMaxPhaseError = PLLPhaseError;
			PhaseRotationErrorCnt = 0;
        }
        ~DQPhaseLockLoop()
        {
        }

    public:
        void RunPLLFast( void );
        void PLLPeriodicFunction( void );
        void SetNominalSourceVoltage( int16_t voltage );
		bool IsDQGoodForSync(void);

        float GetOffset( void ) {                            
            return PhaseErrorOffset; 
        }                            
        bool IsPhaseLocked( void ) {
            return Locked.GetState();
        }
        bool IsSourcePresent( void ) {
            return ( Locked.GetState() && ( SourceNominalDQO.Sd > 0.47 ) ); //change 0.5 to 0.47 for PE100200-296
        }
        bool IsPhaseRotationError( void ) {
            return PhaseRotationError;
        }
        float GetSourceNormFactor( void ) {
            return SourceNormalizationFactor;
        }
        float GetNominalNormFactor( void ) {
            return NominalNormalizationFactor;
        }
        float GetPhaseError( void ) {
            return PhaseError;
        }
        float GetFrequency( void ) {
            return SineRef.Frequency;
        }
        float GetAngleStep(void) const
        {
            return SineRef.GetAngleStep();
        }
        
// public data
    public:
        stPark                      SourceNominalDQO;       // DQ0 data, normalized to nominal voltage, if applicable
                                                            // used for UV/OV detection
        stPark                      SourceDQOData;          // useful for fast measurement info
        SineReference               SineRef;
        float                       DBPhaseError;
        float                       SourceDQODataSlowSd;    // for dc link reference calculation		
        float                       PLLMaxPhaseError;
		float						PLLMinDq;
		uint16_t  					PhaseRotationErrorCnt;
    protected:
        FilteredBit                 Locked;
        uint16_t                    InnerLoopCount;
        bool                        PhaseRotationError;
        float                       PhaseError;
        float                       PhaseErrorOffset;
        float                       LoadSharePhaseCal;
        float                       PhaseIntegral;
        float                       SourceNormalizationFactor;
        float                       NominalNormalizationFactor;
        stThreePhase                SourceVoltage;          // 3p sync source
        stPark                      SourceDQONorm;          // DQ0 data, normalized to the measured magnitude
        stFirstOrderIIRFP			PllCompensator;
		stFirstOrderIIRFP			SdFilter;		
        stSecondOrderIIRFP          PhaseErrorFilter;
        volatile const stThreePhase*      SyncSource;

};

// global PLLs
extern DQPhaseLockLoop BypassPLL;
extern DQPhaseLockLoop OutputPLL;

inline void ee_update_bypass_sync( EE_ID* ee, uint16_t* data )
{
    BypassPLL.SineRef.EEFunc_Sync( ee, data );    
}

inline void ee_update_output_sync( EE_ID* ee, uint16_t* data )
{
    OutputPLL.SineRef.EEFunc_Sync( ee, data );    
}

#endif

// ******************************************************************************************************
// *            End of DQPhaseLockLoop.h
// ******************************************************************************************************
