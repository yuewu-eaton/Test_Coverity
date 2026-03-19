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
#ifndef _EXT_SIGNAL_PLL_H
#define _EXT_SIGNAL_PLL_H

#include "SineReference.h"
#include "Eeprom_Map.h"


class ExtSignalPLL
{
    public:
        ExtSignalPLL() : SineRef( NOM_FREQUENCY_60HZ, PLL_Ts )       
        {
            PhaseError = 0;
            TimeStamp = 0;
            TempPhaseError = 0;
            PhaseGain = -0.0025;
        }

        ~ExtSignalPLL()
        {
        }

    private:
        ExtSignalPLL( const ExtSignalPLL& );
        const ExtSignalPLL& operator=( const ExtSignalPLL& );

    public:
        void RunPLLFast( void );
        void ExtSyncReceived( uint32_t timestamp )
        {
            PhaseError = SineRef.Angle;
            TimeStamp = timestamp;
        }

    public:
        SineReference   SineRef;
        float           TargetFrequency;
        float           TempPhaseError;
        float           PhaseGain;


    protected:
        float           PhaseError;
        uint32_t        TimeStamp;

};

extern ExtSignalPLL ExtSignalSync;

inline void ee_update_base_sync( EE_ID* ee, uint16_t* data )
{
    ExtSignalSync.SineRef.EEFunc_Sync( ee, data );    
}


#endif
// ******************************************************************************************************
// *            End of ExtSignalPLL.cpp
// ******************************************************************************************************
