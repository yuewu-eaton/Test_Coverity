// ********************************************************************************************************
// *            NB_Funcs.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton Powerware
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: NB_Funcs.c
// *
// *    DESCRIPTION: Nodebit Functions.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/10/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "MCUState.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "XCPDefs.h"
#include "Rtc.h"
#include "HQ_Funcs.h"
#include "Abm.h"
#include "BatteryStateControl.h"
#include "BypassInterface.h"
#include "IOexpansion.h"
#include "ParallelCan.h"

#include <limits>

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
uint16_t NumActEvents;
uint16_t NumActAlarms;
uint16_t NumActNotices;
st_NB_Status NB_Status[ UPM_NB_NUMBER_OF_SUPPORTED_EVENTS ];
// ********************************************************************************************************
// * MODULE-PRIVATE VARIABLES
// ********************************************************************************************************
namespace {
    bool TestMode;
    /* Nodebits are periodically transmitted over CAN to the other modules. The CAN
     * transmission packet gathers up all of the nodebit defn's as a large bitfield.
     * Rather than re-compute this bitfield every time the packet is sent, cache it
     * here. This array shall be kept synchronized with the values if NB_Stats.StatusBit below.
     */
    uint16_t NB_Status_Bits[UPM_NB_NUMBER_OF_SUPPORTED_EVENTS / 16 + 1];

} // !namespace (anon)


// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

namespace {
    void NB_Status_Bit_Set(uint16_t nbId);
    void NB_Status_Bit_Clear(uint16_t nbId);
} // !namespace (anon)

// ********************************************************************************************************
// *
// * Function: NB_GetNodebitBits
// *
// * Purpose: Returns status of Nodebits for CAN
// *
// * Parms Passed   :   bits[in] An array of at least 8 uint16_t words that will receive the current list
// *                    of active alarms. The array shall be zero-initialized prior to calling this
// *                    function, since it will only write those portions of the array that are actually
// *                    allocated by the firmware.
// * 
// * Returns        :   nothing.
// *
// * Description    :   Sets the bits of a provided array to send over the CAN.
// *  Does not fill in over 12 words due to the CAN packet code.
// *
// ********************************************************************************************************
void NB_GetNodebitBits(uint16_t* const bits)
{
    CriticalSection enter;
    // Pan/20121012 delete
    /*
    for (size_t i = 0; i < sizeof(NB_Status_Bits) / sizeof(NB_Status_Bits[0]) && i < 8; i++) {
        bits[i] = NB_Status_Bits[i];
    }*/
    // Pan/20121012 add, max events is 191, we can fill in less than 12 words 
    for(size_t i = 0; i < sizeof(NB_Status_Bits) / sizeof(NB_Status_Bits[0]) && i < 12; i++) {
        bits[i] = NB_Status_Bits[i];
    }
}

// ********************************************************************************************************
// *
// * Function: NB_Init
// *
// * Purpose: Initializes Nodebit structures prior to first use
// *
// * Parms Passed   :   None
// * 
// * Returns        :   nothing.
// *
// * Description    :   This function first zeros out the entire NB_Status area to clear all nodebits.
// *                    and initializes debounce counts to 0                                 
// ********************************************************************************************************
void NB_Init(void)
{
    TestMode = false;
    for (size_t i = 0; i < sizeof(NB_Status_Bits) / sizeof(NB_Status_Bits[0]); i++) {
        NB_Status_Bits[i] = 0;
    }
    for (size_t i = 0; i < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS; i++ )
    {
        NB_Status[i].StatusBit = false;            // All nodebits are Clear, 
        NB_Status[i].NB_Cnt = 0;               // Count is 0
    }
}       // end of NB_Init

// ********************************************************************************************************
// *
// * Function: NB_GetNodebit
// *
// * Purpose: Returns current state of the Nodebit: Active or Inactive
// *
// * Parms Passed   :   nbNum       : number of the nodebit being read
// * 
// * Returns        :   0 if nodebit is inactive, 1 if active
// *
// * Description    :   Returns status (true/false) of nodebit. Returns false if no such bit.
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
uint16_t NB_GetNodebit( uint16_t nbNum )
{
    // Optimization opportunity
    uint16_t signalOut = false;                                   // start out invalid
    
    if ( nbNum < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS ) 
    {
        signalOut = NB_Status[nbNum].StatusBit;             
    }

    return signalOut;
}   // end of NB_GetNodebit

// ********************************************************************************************************
// *
// * Function: NB_Force
// *
// * Purpose: Forces the nodebit to the given state. Additionally, enters Test Mode.
// * 
// *
// * Parms Passed   :   nbNum       : number of the nodebit being triggered
// *                    signalIn    : Signal to be set: 0 if NB should be cleared, >0 if it should be set.
// *                    nbData      : Data to be associated with the nodebit. Default: 0.
// * 
// * Returns        :   signalIn
// *
// * Description    :   The DebounceAndQue function uses parts of the NB_Config structure to know what to do:
// *                If this is the first transition into Test Mode, then all of the nodebits are forced 
// *                inactive prior to excersizing nbNum. While in Test Mode, none of the nodebit signals 
// *                will be activated by NB_DebounceAndQueue(), but that function will return the forced 
// *                state of the nodebit. If the state transitions, then queue for the event log.
// *                                  
// *
// ********************************************************************************************************
uint16_t NB_Force(uint16_t nbNum, uint16_t signalIn, uint16_t nbData)
{   
    if (!TestMode) 
    {
        NB_Init();
        TestMode = true;
    }
    
    return NB_SetNodebit( nbNum, signalIn, nbData, true );
}

// ********************************************************************************************************
// *
// * Function: NB_LogStateChange
// *
// * Purpose: Logs a state machine change into the history log without affecting
// *          the nodebit system.
// * 
// *
// * Parms Passed   :   machine     : State machine being triggered.
// *                    state       : Numeric state that the machine is entering.
// * 
// * Returns        :   Nothing
// *
// * Description    :   Queues up a log entry in the active direction without
// *                    touching any nodebits.  The logged state will correspond
// *                    to the passed value + 1 to ensure that it gets printed
// *                    in a dump of the history log.
// *
// ********************************************************************************************************
#pragma CODE_SECTION("ramfuncs")
void NB_LogStateChange(upm_state_nb_t machine, uint16_t state)
{
    if( !HistoryQueue.IsQueueReady() ) return;
    
    uint16_t nbNum(machine); 
    HistoryQueue.QueueEvent(nbNum | HQ_EVENT_IS_TRUE, true, state);
}

// ********************************************************************************************************
// *
// * Function: NB_SetNodebit
// *
// * Purpose: Sets the nodebit to the given state.
// * 
// *
// * Parms Passed   :   nbNum       : number of the nodebit being triggered
// *                    signalIn    : Signal to be set: 0 if NB should be cleared, >0 if it should be set.
// *                    nbData      : Data to be associated with the nodebit. Default: 0.
// *                    force       : Override test mode and force the nodebit on / off (Default: 0)
// * 
// * Returns        :   signalIn
// *
// * Description    :   Overrides normal debouncing and sets/resets nodebit immediately
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
uint16_t NB_SetNodebit(uint16_t nbNum, uint16_t signalIn, uint16_t nbData, uint16_t force)
{
    if( !HistoryQueue.IsQueueReady() ) return false;
        
    if ( nbNum < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS )
    {
        st_NB_Status*    nb    = &NB_Status[ nbNum ];
        const st_NB_Cfg* nbCfg = &NB_Cfg_Flash[ nbNum ];
        if (TestMode && !force) 
        {
            // Report current status, but otherwise ignore the request to set the NB.
            return nb->StatusBit;
        }
        if (!signalIn && nb->StatusBit)
        {
            // States differ and are clearing the event
            NB_Status_Bit_Clear(nbNum);
            nb->StatusBit = false;
            if (nbCfg->bit.QueueControl & QUE_INACTIVE)
            {
                HistoryQueue.QueueEvent( 
                    ( nbNum | HQ_EVENT_IS_FALSE ),
                    (nbCfg->bit.QueueControl & QUE_LOCAL_ONLY) == QUE_LOCAL_ONLY, 
                    nbData );   // EVENT - Que HI to LOW event
            }
        } 
        else if (signalIn && !nb->StatusBit)
        {
            // States differ and are setting the event
            NB_Status_Bit_Set(nbNum);
            nb->StatusBit = true;
            if (nbCfg->bit.QueueControl & QUE_ACTIVE)
            {
                HistoryQueue.QueueEvent( 
                    ( nbNum | HQ_EVENT_IS_TRUE ),
                    (nbCfg->bit.QueueControl & QUE_LOCAL_ONLY) == QUE_LOCAL_ONLY, 
                    nbData );   // EVENT - Que HI to LOW event
            }   
        }
        /*
        else {
            signalIn == nb->StatusBit, and therefore nothing needs to be done.
        } */
        return nb->StatusBit;
    }    
    else
    {
        return false;                           // no such nodebit, so must be false
    }    
}

// ********************************************************************************************************
// *
// * Function: NB_DebounceAndQue
// *
// * Purpose: Debounces Input signal (NB_input) as specified by parameters in NB_Cfg
// *
// * Parms Passed   :   nbNum       : number of the nodebit being debounced.
// *                    signalIn    : Signal to be debounced: 0 if currently Low, >0 if currently High
// * 
// * Returns        :   0 if debounced signal is inactive after this debounce call, 1 if active
// *
// * Description    :   The DebounceAndQue function uses parts of the NB_Config structure to know what to do:
// *                    If signal is active, count up, if greater than debounce count defined in NB_Config array,
// *                    NB = true, queue for event log
// *                    If signal is in-active, count down.
// *                                  
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
uint16_t NB_DebounceAndQue( uint16_t nbNum, uint16_t signalIn, uint16_t nbData )
{
    if( !HistoryQueue.IsQueueReady() ) return false;
    
    if ( nbNum < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS )
    {
        st_NB_Status*    nb    = &NB_Status[ nbNum ];
        const st_NB_Cfg* nbCfg = &NB_Cfg_Flash[ nbNum ];
        if (TestMode) {
            // Report current status, but otherwise ignore the request to set the NB.
            return nb->StatusBit;
        }

        if ( !signalIn )
        {
            if ( nb->StatusBit )                                // signalIn = 0 AND signalOut = 1,
            {
                nb->NB_Cnt++;
                if ( nb->NB_Cnt >= nbCfg->bit.InactiveCount )
                {
                    nb->NB_Cnt = 0;                             // Reset Counter - EVENT
                    nb->StatusBit = false;
                    NB_Status_Bit_Clear(nbNum);
                    if (nbCfg->bit.QueueControl & QUE_INACTIVE)
                    {
                        HistoryQueue.QueueEvent( 
                            ( nbNum | HQ_EVENT_IS_FALSE ),
                            (nbCfg->bit.QueueControl & QUE_LOCAL_ONLY) == QUE_LOCAL_ONLY, 
                            nbData );   // EVENT - Que HI to LOW event
                    }
                }
            }
            else
            {
                nb->NB_Cnt = 0;
            }    
        } 
        else    // else signalIn = '1'
        {
            if ( !nb->StatusBit )                               // Else, if signalIn = 1 AND signalOut = 0,
            {
                nb->NB_Cnt++;                                   
                if ( nb->NB_Cnt >= nbCfg->bit.ActiveCount )
                {
                    nb->NB_Cnt = 0;
                    nb->StatusBit = true;
                    NB_Status_Bit_Set(nbNum);
                    if (nbCfg->bit.QueueControl & QUE_ACTIVE)
                    {
                        HistoryQueue.QueueEvent( 
                            ( nbNum | HQ_EVENT_IS_TRUE ), 
                            ( nbCfg->bit.QueueControl & QUE_LOCAL_ONLY) == QUE_LOCAL_ONLY,
                            nbData );    // EVENT - Que Low to Hi event
                    }
                } 
            }
            else
            {
                nb->NB_Cnt = 0;
            }    
        }           // end of 'else "signalIn = 1"
        
        return nb->StatusBit;
    }    
    else
    {
        return false;                           // no such nodebit, so must be false
    }    
}       // end of NB_DebounceAndQue

// ********************************************************************************************************
// *
// * Function: NB_DebounceAndQue_Hysterisis
// *
// * Purpose: Debounces Input signal (NB_input) as specified by parameters in NB_Cfg
// *
// * Parms Passed   :   nbId        : number of the nodebit being debounced.
// *                    set         : Pass the comparison result of whether to set this nodebit
// *                    clear       : Pass the comparison result of whether to clear this nodebit
// * 
// * Returns        :   false if debounced signal is inactive after this debounce call, true if active
// *
// * Description    :   This code simplifies NB checks where hysterisis is involved. Instead of many
// *     structural checks in the code, you can use it as such:
// *     
// *     NB_DebounceAndQue_Hysterisis( NODEBIT_ACOV,
// *        signal > ACOV_Limit,
// *        signal < (ACOV_Limit - Hysterisis) );
// *     
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
uint16_t NB_DebounceAndQue_Hysterisis( uint16_t nbId, bool set, bool clear, uint16_t nbData )
{
    if( !HistoryQueue.IsQueueReady() ) return false;
    
    if (set)
    {
        NB_DebounceAndQue(nbId, true, nbData);
    }
    
    if (clear)
    {
        NB_DebounceAndQue(nbId, false);
    }
    
    return NB_GetNodebit( nbId );
}

// ********************************************************************************************************
// *
// * Function: NB_Parse
// *
// * Purpose:  Parses nodebits and updates relay/phone/alarmled status
// *
// ********************************************************************************************************
void NB_Parse( void )
{
    uint16_t numEvents = 0;
    uint16_t numNotices = 0;
    uint16_t numAlarms= 0;
    
    for ( uint16_t idx = 0; idx < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS; idx++ )
    {
        if ( NB_Status[ idx ].StatusBit )
        {
            // count events, notices and alarms. Why I don't know. I reckon the display
            // will need this someday.
            if ( NB_Cfg_Flash[ idx ].bit.XCPAlarmLevel < EVENT_LEVEL_NOTICE )
            {
                numEvents++;   
            }
            else if ( NB_Cfg_Flash[ idx ].bit.XCPAlarmLevel < EVENT_LEVEL_ALARM )
            {
                numNotices++;
            }
            else
            {
                numAlarms++;
            }
        }
    }
    
    NumActEvents = numEvents;
    NumActNotices = numNotices;
    NumActAlarms = numAlarms;
}


// ********************************************************************************************************
// *
// * Function: NB_GetNumEvents( void )
// *
// * Purpose:  returns the number of active events
// *
// ********************************************************************************************************
uint16_t NB_GetNumEvents(void)
{
    return NumActEvents;
}
// ********************************************************************************************************
// *
// * Function: NB_GetNumAlarms( void )
// *
// * Purpose:  returns the number of active alarms
// *
// ********************************************************************************************************
uint16_t NB_GetNumAlarms(void)
{
    return NumActAlarms;
}

// ********************************************************************************************************
// *
// * Function: NB_GetNumNotices( void )
// *
// * Purpose:  returns the number of active notices
// *
// ********************************************************************************************************
uint16_t NB_GetNumNotices(void)
{
    return NumActNotices;
}


// ********************************************************************************************************
// *
// * Function:     void ResetStickyAlarms( void )
// * 
// * Purpose:      Clears "sticky" alarms and bits and front panel acknowledge/quit alarms is pressed.
// *
// ********************************************************************************************************
void ResetStickyAlarms( void )
{
    const uint16_t sentinnel = std::numeric_limits<uint16_t>::max();
    const uint16_t stickyAlarm[] = {
        UPM_NB_STATIC_SWITCH_SHORT,
        UPM_NB_TOO_MANY_INVERTER_TRANSFERS,
        UPM_NB_INVERTER_CONTACTOR_FAILURE,
        UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP,
		UPM_NB_ABNORMAL_EXIT_ECT_MODE,
		UPM_NB_INVERTER_STARTUP_FAILURE,
		UPM_NB_MOB_FAILURE,
        UPM_NB_SELECTIVE_TRIP_OF_MODULE,
        UPM_NB_BATTERY_BREAKER_FAILURE,     // Pan/20121012 add
        UPM_NB_OUTPUT_SHORT_CIRCUIT,
        sentinnel
    };
    
    MCUStateMachine.ResetECTAlarm();
    MCUStateMachine.SetStaticSwitchShort( false );
    
    for (uint16_t i = 0; stickyAlarm[i] != sentinnel; ++i)
    {
        NB_SetNodebit(stickyAlarm[i], false);
    }
    
    ParallelCan.ResetSticky();
    ClearLatchedEPO();
    Rectifier.ResetSiteWiringFault();
}

// ********************************************************************************************************
// *
// * Function:     void ResetAlarms( void )
// * 
// * Purpose:      Clears alarms and bits when any command is pressed.  Intended for non-saftey alarms.
// *
// ********************************************************************************************************
void ResetAlarms( void )
{
    const uint16_t sentinnel = std::numeric_limits<uint16_t>::max();
    const uint16_t stickyAlarm[] = {
        UPM_NB_RECTIFIER_FAILED,
        UPM_NB_FUSE_FAILURE,
        // UPM_NB_BACKFEED_CONTACTOR_FAILURE,
        // The following are debounced normally at some periodic rate, but 
        // are placed here so that users can override the debounce rate
        UPM_NB_INVERTER_OVERTEMPERATURE_TRIP,
        UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP,
        UPM_NB_PM_OVERTEMPERATURE_TRIP,
        // The following are debounced per inverter period, triggered by the
        // zero crossing SWI. They are not normally sticky, but may stick anyway
        // if the zero crossing SWI is not being triggered.
        UPM_NB_INVERTER_OUTPUT_OVER_CURRENT,
        UPM_NB_RECTIFIER_INPUT_OVER_CURRENT,
        UPM_NB_BATTERY_CURRENT_LIMIT,
        UPM_NB_OUTPUT_SHORT_CIRCUIT,
        UPM_NB_CHECK_PRECHARGE,
        UPM_NB_DC_LINK_UNDER_VOLTAGE,
        UPM_NB_INVERTER_STARTUP_FAILURE,
        sentinnel
    };
    
    for (uint16_t i = 0; stickyAlarm[i] != sentinnel; ++i)
    {
        NB_SetNodebit(stickyAlarm[i], false);
    }

    MCUStateMachine.ResetSticky();
    BypassState().ResetSticky();
    
    Abm().ResetAlarms();
    BatteryConverter.ClearBatteryAlarms();
}

void NB_Resume( void )
{
    TestMode = false;
}

namespace {
// Local Function Definitions

// ********************************************************************************************************
// *
// * Function: void NB_Status_Bit_Set(uint16_t nbId)
// * 
// * Purpose:  Sets the status bit associated with a particular nodebit index
// * Precondition: nbId < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS
// * 
// * Note: Placed in ranfuncs due to its disable/enable of interrupts
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void NB_Status_Bit_Set(uint16_t nbId)
{
    size_t word = nbId >> 4;
    uint16_t mask = 1 << (nbId & 0xf);
    
    CriticalSection enter;
    NB_Status_Bits[word] |= mask;
}

// ********************************************************************************************************
// *
// * Function: void NB_Status_Bit_Set(uint16_t nbId)
// * 
// * Purpose:  Clears the status bit associated with a particular nodebit index
// * Precondition: nbId < UPM_NB_NUMBER_OF_SUPPORTED_EVENTS
// *
// * Note: Placed in ranfuncs due to its disable/enable of interrupts
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void NB_Status_Bit_Clear(uint16_t nbId)
{
    size_t word = nbId >> 4;
    uint16_t mask = ~(1 << (nbId & 0xf));
    
    CriticalSection enter;
    NB_Status_Bits[word] &= mask;
}

} // !namespace (anon)
