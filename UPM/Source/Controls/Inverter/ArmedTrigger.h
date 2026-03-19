// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: ArmedTrigger.h
// *
// *    DESCRIPTION: Defines an object for Arm-Trigger-Clear operation.
// *
// *    HISTORY: See SVN history for author and revision history.
// *********************************************************************************************************
#ifndef _ARMED_TRIGGER_H
#define _ARMED_TRIGGER_H

#include "Constants.h"

enum trigger_cause_t
{
    TRIGGER_CLEAR,
    INVERTER_ZERO_CROSSING,
    BYPASS_ZERO_CROSSING,
    UTILITY_ZERO_CROSSING
};

class ArmedTrigger
{
    public:
        ArmedTrigger( void ) 
        {
            lock = TRIGGER_CLEAR;
            fire = false;
        }
        ~ArmedTrigger()
        {
        }
        
        void Arm ( trigger_cause_t key )
        {
            lock = key;
            fire = false;
        }
        
        void Disarm ( void )
        {
            lock = TRIGGER_CLEAR;
            fire = false;
        }
        
        bool Check ( void )
        {
            if ( fire )
            {
                fire = false;
                return true;
            }
            else
            {
                return false;
            }
        }
        
        void Trigger ( trigger_cause_t key )
        {
            if ( (key != TRIGGER_CLEAR ) && ( lock == key ) )
            {
                key = TRIGGER_CLEAR;
                fire = true;
            }
        }
        
    private:
        bool fire;
        trigger_cause_t lock;
};

#endif
// ******************************************************************************************************
// *            End of ArmedTrigger.h
// ******************************************************************************************************
