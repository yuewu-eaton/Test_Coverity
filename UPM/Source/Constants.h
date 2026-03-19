// ******************************************************************************************************
// *            Constants.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: Constants.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/19/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _CONSTANTS_H
#define _CONSTANTS_H

#include <stdint.h>

#define CPUFrequency            ( 150000000UL )
#define PWMFrequency            ( 26000UL )
#define ADCFrequency            ( 104000UL ) 
#define BoostFrequency          ( 18000UL )
#define ChargeFrequency         ( 18000UL )
#define NumberOfBoostLegs       ( 1UL )

#define SQRT_2                  ( (float)1.414213562 )
#define SQRT_3                  ( (float)1.732050808 )

#define OS_PRD_TICK_MS          1

#define TSK_5_ms                ( 5    / OS_PRD_TICK_MS )
#define TSK_10_ms               ( 10   / OS_PRD_TICK_MS )
#define TSK_20_ms               ( 20   / OS_PRD_TICK_MS )
#define TSK_50_ms               ( 50   / OS_PRD_TICK_MS )
#define TSK_100_ms              ( 100  / OS_PRD_TICK_MS )
#define TSK_200_ms              ( 200  / OS_PRD_TICK_MS )
#define TSK_1000_ms             ( 1000 / OS_PRD_TICK_MS )

// from stdint.h, copied here for reference
//     typedef          int         int16_t;
//     typedef unsigned int         uint16_t;
//     typedef          long        int32_t;
//     typedef unsigned long        uint32_t;
//     typedef          long long   int64_t;
//     typedef unsigned long long   uint64_t;

typedef struct
{
    float phA;
    float phB;
    float phC;
} stThreePhase;

#define BIT_0                   ( (uint32_t)( 1UL ) )
#define BIT_1                   ( (uint32_t)( 1UL << 1 ) )
#define BIT_2                   ( (uint32_t)( 1UL << 2 ) )
#define BIT_3                   ( (uint32_t)( 1UL << 3 ) )
#define BIT_4                   ( (uint32_t)( 1UL << 4 ) )
#define BIT_5                   ( (uint32_t)( 1UL << 5 ) )
#define BIT_6                   ( (uint32_t)( 1UL << 6 ) )
#define BIT_7                   ( (uint32_t)( 1UL << 7 ) )
#define BIT_8                   ( (uint32_t)( 1UL << 8 ) )
#define BIT_9                   ( (uint32_t)( 1UL << 9 ) )
#define BIT_10                  ( (uint32_t)( 1UL << 10 ) )
#define BIT_11                  ( (uint32_t)( 1UL << 11 ) )
#define BIT_12                  ( (uint32_t)( 1UL << 12 ) )
#define BIT_13                  ( (uint32_t)( 1UL << 13 ) )
#define BIT_14                  ( (uint32_t)( 1UL << 14 ) )
#define BIT_15                  ( (uint32_t)( 1UL << 15 ) )

#define BIT_16                  ( (uint32_t)( BIT_0 << 16 ) )
#define BIT_17                  ( (uint32_t)( BIT_1 << 16 ) )
#define BIT_18                  ( (uint32_t)( BIT_2 << 16 ) )
#define BIT_19                  ( (uint32_t)( BIT_3 << 16 ) )
#define BIT_20                  ( (uint32_t)( BIT_4 << 16 ) )
#define BIT_21                  ( (uint32_t)( BIT_5 << 16 ) )
#define BIT_22                  ( (uint32_t)( BIT_6 << 16 ) )
#define BIT_23                  ( (uint32_t)( BIT_7 << 16 ) )
#define BIT_24                  ( (uint32_t)( BIT_8 << 16 ) )
#define BIT_25                  ( (uint32_t)( BIT_9 << 16 ) )
#define BIT_26                  ( (uint32_t)( BIT_10 << 16 ) )
#define BIT_27                  ( (uint32_t)( BIT_11 << 16 ) )
#define BIT_28                  ( (uint32_t)( BIT_12 << 16 ) )
#define BIT_29                  ( (uint32_t)( BIT_13 << 16 ) )
#define BIT_30                  ( (uint32_t)( BIT_14 << 16 ) )
#define BIT_31                  ( (uint32_t)( BIT_15 << 16 ) )

// ill-defined board ID
#define BOARD_ID_KENGAT_TEST                ( (uint16_t)( 0 ) )
#define BOARD_ID_PANDA_30LV_ESSENTIAL       ( (uint16_t)( 15 ) )
                                
#endif
// ******************************************************************************************************
// *            End of Constants.h
// ******************************************************************************************************
