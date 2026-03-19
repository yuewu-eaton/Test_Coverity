// ********************************************************************************************************
// *            FilteredBit.cpp
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 EATON
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME:   FilteredBit.cpp
// *
// *    DESCRIPTION: Filtering (debouncing) bits functions for places where a separate nodebit is not needed/wanted.
// *
// *    ORIGINATORS: Tuomo Kaikkonen
// *
// *    DATE:        2010/07/23
// *
// *    HISTORY:     See revision control system's history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "FilteredBit.h"


// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************


// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************


// ********************************************************************************************************
// * Function:     void FilteredBit( uint16_t activeCount, uint16_t inactiveCount, eSticky sticky ) 
// *
// * Purpose:      Initializes the FilterBit
// *
// * Parms Passed: activeCount     the count to activate the filterbit
// *               inactiveCount   the count to inactivate the filterbit
// *               sticky          true if the FilterBit is sticky (i.e. only inactivates using SetState)
// * Returns:      Nothing
// *
// * Description:  
// *
// ********************************************************************************************************
FilteredBit::FilteredBit( uint16_t activeCount, uint16_t inactiveCount, eSticky sticky ):
    ActiveCount   ( activeCount ),
    InactiveCount ( inactiveCount ),
    Sticky        ( sticky ),
    CurrentCount  ( 0 )
{
    SetState( false );
}

// ********************************************************************************************************
// * Function:     void FilteredBit( upm_nb_id_t nbNum, eSticky sticky ) 
// *
// * Purpose:      Initializes the FilterBit
// *
// * Parms Passed: nbNum           The nodebit number from which the active and inactive
// *                               count are taken.
// *               sticky          true if the FilterBit is sticky (i.e. only inactivates using SetState)
// * Returns:      Nothing
// *
// * Description:  
// *
// ********************************************************************************************************
FilteredBit::FilteredBit(upm_nb_id_t nbNum, eSticky sticky)
    : ActiveCount(NB_Cfg_Flash[nbNum].bit.ActiveCount)
    , InactiveCount(NB_Cfg_Flash[nbNum].bit.InactiveCount)
    , Sticky (sticky)
    , CurrentCount( 0 )
{
}

// ********************************************************************************************************
// * Function:     bool GetState( void ) 
// *
// * Purpose:      Check the current state of the FilterBit
// *
// * Parms Passed: none
// * Returns:      The current state.
// *
// ********************************************************************************************************
bool FilteredBit::GetState( void ) 
{ 
    return Status; 
}


// ********************************************************************************************************
// * Function:     bool SetState(bool state) 
// *
// * Purpose:      Sets the FilterBit to a known state without debouncing
// *
// * Parms Passed: state   true/false, the wanted state.
// * Returns:      state
// *
// ********************************************************************************************************
bool FilteredBit::SetState(bool state) 
{        
    CurrentCount = 0; 
    Status = state;
    
    return Status;
}


// ********************************************************************************************************
// * Function:     bool FilteredBit::Debounce( bool input )
// *
// * Purpose:      Debounce the FilterBit
// *
// * Parms Passed: input    true/false, the input value
// * Returns:      state after debounce
// *
// * Description:  If input is opposite than current state, counter is incremented.
// *               If ActiveCount/InactiveCount is reached, state is changed.
// *
// *               If input is the same as current state, counter is decremented.
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
bool FilteredBit::Debounce( bool input )
{
    uint16_t limit;

    if ( Status )
    {
        if ( Sticky )
        {
                // non-standard return statement in the middle of the method. sorry.
            return Status;
        }
        limit = InactiveCount;
    }
    else
    {
        limit = ActiveCount;
    }

    if ( Status != input )
    {
        ++CurrentCount;
        if ( CurrentCount >= limit )
        {
            Status = input;
            CurrentCount = 0;
        }
    }
    else
    {
        if ( CurrentCount )
        {
            --CurrentCount;
        }        
    }

    return Status;
}

// ********************************************************************************************************
// * Function:     bool FilteredBit::Debounce_Hysterisis( bool set, bool clear )
// *
// * Purpose:      Debounce the FilterBit
// *
// * Parms Passed: set    true/false, whether to debounce the FB true
// *               clear  true/false, whether to debounce the FB false
// * Returns:      state after debounce
// *
// * Description:  Utility function for using FilteredBit with hysterisis.
// *
// ********************************************************************************************************
bool FilteredBit::Debounce_Hysterisis( bool set, bool clear )
{
    if (set)
    {
        Debounce(true);
    }    
    
    if (clear)
    {
        Debounce(false);
    }
    
    return Status;
}
// *********************************************************************************************************
// *        end of FilteredBit.c
// *********************************************************************************************************
