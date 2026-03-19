// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: IdleFuncs.c
// *
// *    DESCRIPTION: Idle Task and Functions
// *
// *    HISTORY: See SVN history for author and revision history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "IdleFuncs.h"
#include <stdint.h>

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************
uint32_t IdleCount = 0;

const uint32_t IdleLoopns = 950;        // EXPERIMENTALLY found idle loop time, in ns

// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ****************************************************************************
// *
// *  Function:    IdleTime( void )
// *
// *  Purpose :    This function is called everytime the BIOS calls IDLE, and
// *                 increments the idle timer.
// *
// ****************************************************************************
void IdleTime( void )
{
    IdleCount++;
}

// ****************************************************************************
// *
// *  Function:    CalcPercentIdle( uint32_t interval_us )
// *
// *  Purpose :    This function is calculates the percentage of time
// *                 the processor is idle.
// *
// ****************************************************************************
float CalcPercentIdle( uint32_t interval_us )
{
    // Idle loop count * idle loop time in ns, divide by 1000 for us
    float idlePercent = (float)( IdleCount * IdleLoopns ) / (float)1000.0;
    
    // divide by interval us
    idlePercent = idlePercent / (float)interval_us;

    // clear Idle count
    IdleCount = 0;
    
    return idlePercent; 
}

// ******************************************************************************************************
// *    End of IdleFuncs.c
// ******************************************************************************************************


