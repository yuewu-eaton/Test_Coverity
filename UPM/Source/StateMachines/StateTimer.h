// ******************************************************************************************************
// *            StateTimer.h
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
// *    FILE NAME: StateTimer.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 5/30/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _STATE_TIMER_H
#define _STATE_TIMER_H

class StateTimer
{
    public:
        StateTimer() 
        	: timer(0)
        {
        }
        
        /*
         * Allow a timer to be initialized in the timed out state or an
         * intermeddiate state.
         * 
         * Postcondition: CheckTimeout may not be called with a value smaller
         * this one until after the timer has been cleared.
         */
        explicit StateTimer(uint32_t initial) 
        	: timer(initial) 
        {
        }
        
        ~StateTimer()
        {
        }

    public:
        bool CheckTimeout( const uint32_t timeout );
        void ClearTimer( void )
        {
            timer = 0;
        }
        
        uint32_t TimerValue( void )
        {
            return timer;
        }
        
        void IncTimer( void )
        {
            timer++;
        }
    protected:
        uint32_t timer;
};

#endif
// ******************************************************************************************************
// *            End of StateTimer.h
// ******************************************************************************************************
