#ifndef _DEBUGGERBLOCKS_H
#define _DEBUGGERBLOCKS_H
// ********************************************************************************************************
// *            DebuggerBlocks.c
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
// *    FILE NAME: DebuggerBlocks.c
// *
// *    DESCRIPTION: Blocks array for the debugger.
// *
// *    ORIGINATOR: Jonathan Rodriguez
// *
// *    DATE: 1/8/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// ********************************************************************************************************
// *    Extern for BLOCK ARRAY
// ********************************************************************************************************
typedef enum
{
    NONE = 0,    
    DEC0,    
    DEC1,    
    DEC2,
    DEC3,
    DEC4,       // 4 places is all sprintf will do
    HEX
} eBlockType;


typedef int16_t  (*FUNC_TYPE_INT_PTR)  ( void );  // FuncTypeINT
typedef uint16_t (*FUNC_TYPE_UINT_PTR) ( void );  // FuncTypeUINT
typedef int32_t  (*FUNC_TYPE_LONG_PTR) ( void );  // FuncTypeLONG
typedef uint32_t (*FUNC_TYPE_ULONG_PTR)( void );  // FuncTypeULONG
typedef float    (*FUNC_TYPE_FLOAT_PTR)( void );  // FuncTypeFLOAT

typedef enum
{
    MemTypeNONE = 0,
    MemTypeINT,
    MemTypeUINT,
    MemTypeLONG,
    MemTypeULONG,
    MemTypeFLOAT,
    MemTypeEEPROM,

    FuncTypeINT,    // NOTE: Only functions in flash memory supported for now. 
    FuncTypeUINT,   // Can not use member functions directly from blocks, must be wrapped.
    FuncTypeLONG,
    FuncTypeULONG,
    FuncTypeFLOAT
} eMemType;

typedef struct Block 
{
    eBlockType  type;
    eMemType    memtype;            
    void*       data;
} stBlock;

extern const stBlock BlockArray[];
extern const char* const BlocksStrings[];

extern const uint16_t BlockArraySize;

extern uint16_t DebugWord[12];
extern float DebugFloat[6];
extern float DebugPara[50][4];

// ********************************************************************************************************
// *    Externs for BLOCK ARRAY
// ********************************************************************************************************

#define BLOCKS_MCU_START            0
#define BLOCKS_MCU_END              35
#define BLOCKS_INVERTER_START       36
#define BLOCKS_INVERTER_END         71
#define BLOCKS_RECTIFIER_START      72
#define BLOCKS_RECTIFIER_END        107
#define BLOCKS_SYSTEM_START         108
#define BLOCKS_SYSTEM_END           143
#define BLOCKS_CHARGER_START        144
#define BLOCKS_CHARGER_END          179
#define BLOCKS_METERS1_START        180
#define BLOCKS_METERS1_END          215
#define BLOCKS_METERS2_START        216
#define BLOCKS_METERS2_END          251
#define BLOCKS_PARALLEL_START       252
#define BLOCKS_PARALLEL_END         287
#define BLOCKS_1_START              288
#define BLOCKS_1_END                288+35
#define BLOCKS_2_START              324
#define BLOCKS_2_END                324+35
#define BLOCKS_3_START              360
#define BLOCKS_3_END                395
#define BLOCKS_4_START              396
#define BLOCKS_4_END                431

// ********************************************************************************************************
// *            END OF DebuggerBlocks.h
// ********************************************************************************************************
#endif
