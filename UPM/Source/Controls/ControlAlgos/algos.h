// ********************************************************************
// *            algos.h
// ********************************************************************
// ********************************************************************
// * 
// * THIS INFORMATION IS PROPRIETARY TO INVENSYS Powerware Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 1999, 2000, 2001, 2002, 2003 Invensys Powerware                        
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   algos.h
// *                                                                      
// *     DESCRIPTION: C-header for for assembly language algorithms
// *                  
// *     ORIGINATOR:  Kevin VanEyll                                         
// *                                                                      
// *     DATE:        3/12/2004                                            
// *                                                                      
// *     HISTORY:     See visual source safe history                                                    
// ********************************************************************
#ifndef _ALGOS_H
#define _ALGOS_H

#include "Constants.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _stSecondOrderIIR {
    int32_t B0;
    int32_t X1;
    int32_t A1;
    int32_t B1;
    int32_t X2;
    int32_t A2;
    int32_t B2;
} stSecondOrderIIR;

// floating point version
// floating point version
typedef struct _stSecondOrderIIRFP {
    float   B0;
    float   B1;
    float   B2;
    float   A1;
    float   A2;
    float   X1;
    float   X2;
} stSecondOrderIIRFP;

//20171227 add for first order iir, for improve ram use
typedef struct _stFirstOrderIIRFP {
    float   B0;
    float   B1;
    float   A1;
    float   X1;
} stFirstOrderIIRFP;

//
typedef struct _stPark
{
    float   Sd;
    float   Sq;
    float   S0;
} stPark;

typedef struct _stClarke
{
    float   alpha;
    float   beta;
    float   gamma;
} stClarke;

union CombinedVector
{
    stClarke        aby;
    stThreePhase    abc;
};

//
typedef struct tagstVectorDQ0
{
    float   Sd;
    float   Sq;
    float   S0;
} stVectorDQ0;

typedef struct tagstVectorABY
{
    float alpha;
    float beta;
    float gamma;
} stVectorABY;
// ********************************************************************
// *            FUNCTION PROTOTYPES
// ********************************************************************
int32_t SecondOrderIIR( int32_t input, stSecondOrderIIR* table );
float SecondOrderIIRFP( float input, stSecondOrderIIRFP* table );
float FirstOrderIIRFP( float input, stFirstOrderIIRFP* table );
void FirstOrderIIRFPBackInitialize( float output, stFirstOrderIIRFP* table );
void  abc_to_dq0( stThreePhase* const ph,  stPark* const dq, float angle );
void  dq0_to_abc( stPark* dq, stThreePhase* ph, float angle );
void cos_3_phase(float radians, float* cos_th, float* cos_thMINUS120, float* cos_thPLUS120);
void sin_3_phase(float radians, float* sin_th, float* sin_thMINUS120, float* sin_thPLUS120);

#ifdef __cplusplus
}
#endif

#endif
// ********************************************************************
// *            END OF algos.h   
// ********************************************************************
