// ******************************************************************************************************
// *            SineReference.h
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
// *    FILE NAME: SineReference.h
// *
// *    DESCRIPTION: Basic sine reference
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/28/2011
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _SINE_REFERENCE_H
#define _SINE_REFERENCE_H

#include "Eeprom_Map.h"
#include "Constants.h"

namespace                            
{                                    
    const float PI                  = 3.1415926536;
    const float DegreesToRadians    = ( PI / 180.0 );
    const float TwoThirdsPI         = ( 2.0f * PI ) / 3.0f;
    const float OneThirdPI          = ( 1.0f * PI ) / 3.0f;
}                                    

const float PLL_Ts_HV          = ( 2.0f / float( PWMFrequency ) ); //13KHz
extern float PLL_Ts;       
                                     
enum nom_frequency_t                 
{                                    
    NOM_FREQUENCY_50HZ,
    NOM_FREQUENCY_55HZ,              
    NOM_FREQUENCY_60HZ               
};                                   

class SineReference
{
    public:
        SineReference( nom_frequency_t frequency, float Ts )
        {
            InitFrequency = frequency;
        }

        ~SineReference()
        {
        }

        const SineReference& operator=( const SineReference& )
        {
            return *this;
        }

        void InitFrequencyAndAngleStep( float Ts );
        void UpdateAngle( void );
        void SetBaseFrequency( nom_frequency_t frequency );
        void EEFunc_Sync( const EE_ID* ee, const uint16_t* data );

        void UpdateFrequency( float newFreq ) {
            Frequency = newFreq;
        }
        void UpdateFrequencyAndAngleStep( float newFreq ) {
            Frequency = newFreq;
            AngleStep = newFreq * AngleStepScale;
        }
        void UpdateFrequencyAndAngle( float newFreq, float angle ) {
            Frequency = newFreq;
            AngleStep = newFreq * AngleStepScale;
            Angle = angle;
        }
        float GetBaseFrequency( void ) {   
            return BaseFrequency;           
        }
        float GetPhaseCal( void ) {
            return LoadSharePhaseCal;
        }
        float GetOffset( void ) {
            return PhaseErrorOffset;
        }

        float GetAngleStep(void) const
        {
            return (AngleStep + AngleCorrection);
        }

    public:
        float Angle;
        float AngleCorrection;      
        float Frequency;            // this is actual frequency 
		float LoadSharePhaseCal;
		
    protected:
        nom_frequency_t InitFrequency;        // this is initial value of Frequency
        float PhaseErrorOffset;
        float AngleStep;
        float BaseFrequency;        // this is configured frequency, i.e. 50 or 60Hz
        float AngleStepScale;

};

class BaseSineReference : public SineReference
{
    public:
        BaseSineReference() : SineReference( NOM_FREQUENCY_60HZ, PLL_Ts )
        {
        }

        ~BaseSineReference()
        {
        }

    private:
        BaseSineReference(const BaseSineReference& other);
        const BaseSineReference& operator=( const BaseSineReference& );
};

extern BaseSineReference BaseSineRef;

#endif
// ******************************************************************************************************
// *            End of SineReference.h
// ******************************************************************************************************
