#ifndef _FILTEREDBIT_H
#define _FILTEREDBIT_H

// ********************************************************************************************************
// *            FilteredBit.h
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
// *    FILE NAME:   FilteredBit.h
// *
// *    DESCRIPTION: Structure and Prototypes for filtering (debouncing only) bits.
// * 
// *    ORIGINATORS: Tuomo Kaikkonen
// *
// *    DATE:        2010/07/23
// *
// *    HISTORY:     See revision control system's history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES  (included files must have #ifndef protection)
// *********************************************************************************************************
#include "Constants.h"
#include "NB_Config.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************
enum eSticky
{
    NOT_STICKY,
    IS_STICKY
};

// *********************************************************************************************************
// *        Class Definition
// *********************************************************************************************************
class FilteredBit
{
public:
    FilteredBit(uint16_t activeCount, uint16_t inactiveCount, eSticky sticky = NOT_STICKY);
    FilteredBit(upm_nb_id_t nbNum, eSticky sticky = NOT_STICKY);
    ~FilteredBit() {}

    bool GetState( void );
    bool SetState( bool state );
    bool Debounce( bool input );
    bool Debounce_Hysterisis( bool set, bool clear );

protected:
    uint16_t ActiveCount;    // these 3 could be const, and it compiles, but does not run...
    uint16_t InactiveCount;
    eSticky  Sticky;

    uint16_t CurrentCount;
    bool     Status;
    
};


// ********************************************************************************************************
// *            END OF FilteredBit.h
// ********************************************************************************************************
#endif

