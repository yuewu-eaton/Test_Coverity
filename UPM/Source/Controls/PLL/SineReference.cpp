// ******************************************************************************************************
// *            SineReference.cpp
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
// *    FILE NAME: SineReference.cpp
// *
// *    DESCRIPTION: Basic sine reference
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
#include "SineReference.h"
#include "Constants.h"
#include <cmath>
#include "DQPhaseLockLoop.h"

using namespace std;

// base dummy sine ref
BaseSineReference BaseSineRef;

float PLL_Ts = PLL_Ts_HV;
extern float LoadSharePhaseCalByp;

// ***********************************************************************
// *
// *    FUNCTION: SineReference_init 
// *
// *    DESCRIPTION: initializes SineReference data to known default 
// *
// *    ARGUMENTS: Ts - this is the rate that the angle is updated, NOT the rate
// *                    at which the control runs
// *
// *    RETURNS:
// *
// ***********************************************************************
void SineReference::InitFrequencyAndAngleStep( float Ts )
{
    Angle = 0.0f;
    AngleCorrection = 0.0f;
    AngleStepScale = ( 2 * PI ) * Ts;
    SetBaseFrequency( InitFrequency );
    UpdateFrequencyAndAngleStep(BaseFrequency);
    PhaseErrorOffset = 0.0f;
    LoadSharePhaseCal = 0.0f;
}

// ***********************************************************************
// *
// *    FUNCTION: UpdateAngle 
// *
// *    DESCRIPTION: Increments angle by stepsize, wraps from PI to -PI
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void SineReference::UpdateAngle( void )
{               
    Angle += ( AngleStep + AngleCorrection );

    if ( Angle > PI )
    {
        Angle -= ( 2.0 * PI );
    }
    /*the below sentence of else is added in order to fix issue:phase rotate,and ups run in battery mode for more than 30minute,
    //then ups will lock at battery mode and will not restore to online mode*/
	else if( Angle < -PI )
	{
		 Angle += ( 2.0 * PI );
	}

}

// ***********************************************************************
// *
// *    FUNCTION: SetFrequency 
// *
// *    DESCRIPTION: Sets anglestep to new frequency 
// *
// *    ARGUMENTS: frequency, only 50 or 60 allowed
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SineReference::SetBaseFrequency( nom_frequency_t frequency )
{
    switch ( frequency )
    {
        case NOM_FREQUENCY_50HZ:
            BaseFrequency = 50.0f;
            break;
            
        case NOM_FREQUENCY_55HZ:        
            BaseFrequency = 55.0f;
            break;

        default:
            BaseFrequency = 60.0f;
            break;
    }

    AngleStep = BaseFrequency * AngleStepScale;
    Frequency = BaseFrequency;
}

// ***********************************************************************
// *
// *    FUNCTION: EEFunc_Sync 
// *
// *    DESCRIPTION: ye olde EEP func
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SineReference::EEFunc_Sync( const EE_ID* ee, const uint16_t* data )
{
    const float MaxPhaseErrorOffset = 20.0f;        // limit to 20 degrees max
    float temp;

    switch ( ee->paramNum )
    {
        case PARAM_BypassPhaseOffset:
        case PARAM_UtilityPhaseOffset:
        case PARAM_OutputPhaseOffset:
        case PARAM_BasePhaseOffset:
            temp = float( int16_t( *data ) ) / 100.0f;
            if ( fabs( temp ) > MaxPhaseErrorOffset )
            {
                if ( temp > 0.0f )
                {
                    temp = MaxPhaseErrorOffset;
                }
                else
                {
                    temp = -MaxPhaseErrorOffset;
                }               
            }

            PhaseErrorOffset = temp * DegreesToRadians;
            
            break;

        case PARAM_BypassLoadSharePhaseCal:
			LoadSharePhaseCal = float( int16_t( *data ) ) / 32767.0f;
			LoadSharePhaseCalByp = LoadSharePhaseCal;
			break;
			
        case PARAM_UtilityLoadSharePhaseCal:
        case PARAM_OutputLoadSharePhaseCal:
        case PARAM_BaseLoadSharePhaseCal:
            LoadSharePhaseCal = float( int16_t( *data ) ) / 32767.0f;
            break;

        case PARAM_BypassPLLMaxPhaseError:
            BypassPLL.PLLMaxPhaseError = ( ( float(*data) * PI ) / 180.0 );
            EE_PLLMaxPhaseError = *data;
            break;
        case Param_OutputPLLMaxPhaseError:
            OutputPLL.PLLMaxPhaseError = ( ( float(*data) * PI ) / 180.0 );
            break;
        case PARAM_BypassPLLMinDQ:
        	BypassPLL.PLLMinDq =  float(*data) * 0.01f;
        	break;
        default:
            break;
    }
}

// ******************************************************************************************************
// *            End of SineReference.cpp
// ******************************************************************************************************
