#ifndef _NB_CONFIG_H
#define _NB_CONFIG_H

// ********************************************************************************************************
// *            NB_Config.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: NB_Config.h
// *
// *    DESCRIPTION: Defines the Nodebits which can be debounced and queued.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/16/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        DEFINES
// *********************************************************************************************************

#include "UPM_NB_Ids.h"

/*
 * The following nodebits are private to the UPM.  They will never be
 * transmitted to the CSB.
 */
enum upm_state_nb_t
{
    UPM_NB_MCU_STATE_CHANGED = UPM_NB_MAXIMUM_EVENT_IDX + 1,
    UPM_NB_BYPASS_STATE_CHANGED,
    UPM_NB_RECTIFIER_STATE_CHANGED,
    UPM_NB_BATTERY_STATE_CHANGED,
    UPM_NB_SYNC_STATE_CHANGED,
    
    UPM_NB_PRIVATE_SUPPORTED_EVENTS
};

// Defines for ActiveCount and InactiveCount - Activate and Deactivate Rate defines
// these can be anything now, not only powers of 2
#define PASS_1          1           // Nodebit transitions to other state after 1 pass
#define PASS_2          2           // Nodebit transitions to other state after 2 passes
#define PASS_3          3           // Nodebit transitions to other state after 3 passes
#define PASS_4          4           // Nodebit transitions to other state after 4 passes
#define PASS_5          5           // Nodebit transitions to other state after 5 passes
#define PASS_6          6           // Nodebit transitions to other state after 6 passes
#define PASS_8          8           // Nodebit transitions to other state after 8 passes
#define PASS_10         10          // Nodebit transitions to other state after 10 passes
#define PASS_11         11          // Nodebit transitions to other state after 11 passes
#define PASS_16         16          // Nodebit transitions to other state after 16 passes
#define PASS_20         20          // Nodebit transitions to other state after 20 passe
#define PASS_22         22          // Nodebit transitions to other state after 32 passes
#define PASS_32         32          // Nodebit transitions to other state after 32 passes
#define PASS_64         64          // Nodebit transitions to other state after 64 passes
#define PASS_128        128         // Nodebit transitions to other state after 128 passes
#define PASS_200        200         // Nodebit transitions to other state after 200 passes
#define PASS_250        250         // Nodebit transitions to other state after 250 passes
#define PASS_300        300         // Nodebit transitions to other state after 300 passes
#define PASS_319        319         // Nodebit transitions to other state after 319 passes
#define PASS_400        400         // Nodebit transitions to other state after 400 passes
#define PASS_638        638         // Nodebit transitions to other state after 638 passes
#define PASS_1275       1275        // Nodebit transitions to other state after 1275 passes
#define PASS_2551       2551        // Nodebit transitions to other state after 2551 passes
#define PASS_5102       5102        // Nodebit transitions to other state after 5102 passes
#define PASS_1000       1000        // Nodebit transitions to other state after 1000 passes
#define PASS_6000       6000        // Nodebit transitions to other state after 6000 passes
#define PASS_9000       9000        // Nodebit transitions to other state after 9000 passes
#define PASS_12000      12000
#define PASS_36000      36000
#define PASS_51         51          // 10ms @ 5.1kHz
#define PASS_15306      15306       // 3s @ 5.1kHz

// Defines for QueueControl
#define QUE_NONE        0           // Do not queue
#define QUE_ACTIVE      1           // queue Active transitions only
#define QUE_INACTIVE    2           // queue Inactive transitions only
#define QUE_ALL         3           // queue all transitions
#define QUE_LOCAL_ONLY  4           // queue locally only, bitwise ORed with one of the above

// NB_Cfg structures and unions
struct st_NB_Cfg_Bits {
    uint16_t ActiveCount;             // Debounce count, activate
    uint16_t InactiveCount;           // Debounce count, de-activate

// the following 3 fields are used by the history queue only    
    uint16_t XCPAlarmNumber;          // XCP alarm number
    uint16_t QueueControl;            // Active/Inactive/all/none
    uint16_t XCPAlarmLevel;           // XCP Alarm Level
};

struct st_NB_Cfg_Words {            // The word method of looking at above structure
    uint16_t  w[ sizeof( st_NB_Cfg_Bits ) ];
};

typedef union  {                   // Union to allow accessing the NB_Config data either by bitfields or words
    struct  st_NB_Cfg_Bits  bit;
    struct  st_NB_Cfg_Words word;
} st_NB_Cfg;


// *********************************************************************************************************
// *        Public Data
// *********************************************************************************************************
extern const st_NB_Cfg      NB_Cfg_Flash[];


// *********************************************************************************************************
// *        Function Prototypes
// *********************************************************************************************************
uint16_t GetNumDefinedNB();
uint16_t GetNodeBitArrayIndex( uint16_t nbNum );

enum system_test_id_t {
    SYSTEM_TEST_ECT = 1,
    SYSTEM_TEST_AUTOCAL = 2
};

// ********************************************************************************************************
// *        END OF NB_CONFIG.h
// ********************************************************************************************************
#endif
