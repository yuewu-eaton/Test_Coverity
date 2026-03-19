// ********************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporaton, ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// *    FILE NAME: NB_Funcs.h
// *
// *    DESCRIPTION: Structure and Prototypes for all public Nodebit functions.
// *
// *    HISTORY: See SVN revision history.
// *********************************************************************************************************
#ifndef _NB_FUNCS_H
#define _NB_FUNCS_H

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "NB_Config.h"

// *********************************************************************************************************
// *        Public Data
// *********************************************************************************************************

// *********************************************************************************************************
// *        Function Prototypes
// *********************************************************************************************************
extern "C" { 
    void NB_Init( void );
}

typedef struct  
{
    bool  StatusBit;                  // Nodebit Status bit
    int16_t   NB_Cnt;                     // debounce counter
} st_NB_Status;

extern st_NB_Status NB_Status[ UPM_NB_NUMBER_OF_SUPPORTED_EVENTS ];

inline bool NB_GetNodebit(upm_nb_id_t nbNum)
{
    return NB_Status[nbNum].StatusBit;
}

uint16_t NB_GetNodebit( uint16_t nbNum );
uint16_t NB_DebounceAndQue( uint16_t nbNum, uint16_t signalIn, uint16_t nbData = 0 );
uint16_t NB_Force( uint16_t nbNum, uint16_t signalIn, uint16_t nbData = 0 );
uint16_t NB_SetNodebit( uint16_t nbNum, uint16_t signalIn, uint16_t nbData = 0, uint16_t force = 0 );
void NB_Parse( void );
uint16_t NB_GetNumEvents( void );
uint16_t NB_GetNumAlarms( void );
uint16_t NB_GetNumNotices( void );
void NB_GetNodebitBits( uint16_t* bits );
void ResetStickyAlarms( void );
void ResetAlarms( void );
void NB_Resume( void );
void NB_LogStateChange(upm_state_nb_t machine, uint16_t state);
uint16_t NB_DebounceAndQue_Hysterisis( uint16_t nbId, bool set, bool clear, uint16_t nbData = 0 );

// ********************************************************************************************************
// *            END OF NB_Funcs.h
// ********************************************************************************************************
#endif

