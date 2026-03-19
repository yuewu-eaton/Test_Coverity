// ******************************************************************************************************
// *            ControlBlocks.c
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
// *    FILE NAME: ControlBlocks.c
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 3/29/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "Algos.h"
#include "math.h"
#include "C28x_FPU_FastRTS.h"

// since this is a C file, can't use PhaseLockLoop namespace
#define PI                      ( (float)3.14159264 )
#define DEGREES_120             ( (float)( ( 2.0 * PI ) / 3.0 ) )
#define SINE_120                ( (float)0.866025404f )
#define COSINE_120              ( (float)-0.5f )
#define COSINE_MINUS_120        ( (float)-0.5f )
#define SINE_MINUS_120          ( -SINE_120 )
static void ComplexMultiply(float* sin, float* cos,
	float lhs_sin, float lhs_cos,
	float rhs_sin, float rhs_cos);
	
/*
 * Rotate one sin/cos pair into another by a fixed angular offset using basic
 * trigonometry rules for sin(alpha + beta) and cos(alpha + beta).
 */
#pragma CODE_SECTION(ComplexMultiply, "ramfuncs")
static void ComplexMultiply(float* sin, float* cos,
	float lhs_sin, float lhs_cos,
	float rhs_sin, float rhs_cos)
{
	*cos = lhs_cos*rhs_cos - lhs_sin*rhs_sin;
	*sin = lhs_cos*rhs_sin + rhs_cos*lhs_sin;
}

// *********************************************************************
// 
//    Function Name   : _IIR_Second_OrderFP;
// 
//    Function Description : Second order IIR transfer function. Normalized form:
//             H(z) = B0 * ((1 + B1*Z^-1 + B2*Z^-2)/(1 - A1*Z^-1 - A2*Z^-2)). Implemented in 
//             canonical direct form II:
// 
//             x[n]--->C1-->+[ ]------->------>+[ ]---->y[n]
//                            -        \         +
//                            \      [1/Z]       \
//                            \        \         \
//                            \        \         \
//                           [ ]<--A1------B1-->[ ]
//                            -        \         +
//                            \      [1/Z]       \
//                            \        \         \
//                            \        \         \
//                           [ ]<--A2------B2-->[+]
// 
//            y[n] = C1x[n] - A1X1 - A2X2 + B1X1 + B2X2
//            X1   = C1x[n] - A1X1 - A2X2
//            X2   = X1
//                              
// *********************************************************************
#pragma CODE_SECTION( SecondOrderIIRFP, "ramfuncs" )
float SecondOrderIIRFP( float input, stSecondOrderIIRFP* table )
{
    float yn;
    float temp;
    
    yn = input * table->B0;
    yn -= ( table->A1 * table->X1 );
    yn -= ( table->A2 * table->X2 );
    temp = yn;
    yn += ( table->B1 * table->X1 );
    yn += ( table->B2 * table->X2 );
    table->X2 = table->X1;
    table->X1 = temp;
    
    return yn;
}

/* 
 * Perform a first-order IIR filter over floating-point data. Optimized for the
 * case where A2 and B2 are equal to zero.
 */
#pragma CODE_SECTION( FirstOrderIIRFP, "ramfuncs" )
float FirstOrderIIRFP( float input, stFirstOrderIIRFP* table )
{
    float yn;
    float temp;
    
    yn = input * table->B0;
    yn -= ( table->A1 * table->X1 );
    temp = yn;
    yn += ( table->B1 * table->X1 );
    table->X1 = temp;
    
    return yn;
}

// ***********************************************************************
// *
// *    FUNCTION: FirstOrderIIRFPBackInitialize 
// *
// *    DESCRIPTION: Back-initializes a first-order IIRFP structure to a
// *                 predetermined constant output.
// *
// *    ARGUMENTS: output - The value to be initialized to
// *               table - pointer to dqo structure for result
// * 
// *    NOTES:
// *          1. This form of the back-initialization is only applicable for
// *             first-order structures; ie, those whose B2 and A2 are zero.
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
void FirstOrderIIRFPBackInitialize( float output, stFirstOrderIIRFP* table )
{
	table->X1 = output*(1.0f - table->B0) / (table->B1 - table->A1);
}

// ***********************************************************************
// *
// *    FUNCTION: abc_to_dq0 
// *
// *    DESCRIPTION: transforms 3phase abc values to dqo reference
// *
// *    ARGUMENTS: ph - pointer to 3phase quantities to be transformed
// *               dq - pointer to dqo structure for result
// *               angle - angle in radians
// * 
// *    NOTES:
// *          1. There is some controversy about what is 'd' and what is 'q',
// *             certain texts have them opposite. This function is exactly the
// *             same as Matlabs's abc-dqo block: d is active, q is reactive.
// *
// *          Sd = (2/3)*[ Asin(angle) + Bsin(angle - 120) + Csin(angle + 120) ]
// *          Sq = (2/3)*[ Acos(angle) + Bcos(angle - 120) + Ccos(angle + 120) ]
// *          S0 = (1/3)*[ A + B + C ]
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
#pragma CODE_SECTION( abc_to_dq0, "ramfuncs" )
void  abc_to_dq0( stThreePhase* const ph, stPark* const dq, float angle )
{
    const float DQScale = ( 2.0 / 3.0 );
    const float ZeroScale = ( 1.0 / 3.0 );
    float base_sin = 0.0f;
    float base_cos = 0.0f;
    float tempsin = 0.0f;
    float tempcos = 0.0f;

// done is a somewhat strange manner to utilize the sincos function
// of the RTS library
    // get sin and cos of theta
    sincos( angle, &base_sin, &base_cos );
    dq->Sd = ph->phA * base_sin;
    dq->Sq = ph->phA * base_cos;

    // theta - 120 degrees
    // sincos( ( angle - DEGREES_120 ), &tempsin, &tempcos );
    ComplexMultiply(&tempsin, &tempcos, SINE_MINUS_120, COSINE_MINUS_120, base_sin, base_cos);
    dq->Sd += ( ph->phB * tempsin );
    dq->Sq += ( ph->phB * tempcos );
    
    // theta + 120 degrees
    // sincos( ( angle + DEGREES_120 ), &tempsin, &tempcos );
    ComplexMultiply(&tempsin, &tempcos, SINE_120, COSINE_120, base_sin, base_cos);
    dq->Sd += ( ph->phC * tempsin );
    dq->Sq += ( ph->phC * tempcos );
    
    // * 2/3
    dq->Sd *= DQScale;
    dq->Sq *= DQScale;   
    
    dq->S0  = ph->phA + ph->phB;
    dq->S0 += ph->phC;
    dq->S0 *= ZeroScale;

}

// ***********************************************************************
// *
// *    FUNCTION: dq0_to_abc 
// *
// *    DESCRIPTION: dqo reference to abc 3phase quantities
// *
// *    ARGUMENTS: ph - pointer to 3phase quantities to be transformed
// *               dq - pointer to dqo structure for result
// *               angle - angle in radians
// * 
// *    NOTES:
// *          1. There is some controversy about what is 'd' and what is 'q',
// *             certain texts have them opposite. This function is exactly the
// *             same as Matlabs's dq0_to_abc block: d is active, q is reactive.
// *
// *          pA = Sd*sin(angle) + Sq*cos(angle) + S0
// *          pB = Sd*sin(angle-120) + Sq*cos(angle-120) + S0 
// *          pC = Sd*sin(angle+120) + Sq*cos(angle+12) + S0
// *
// *    RETURNS: nothing
// *
// ***********************************************************************
#pragma CODE_SECTION( dq0_to_abc, "ramfuncs" )
void  dq0_to_abc( stPark* dq, stThreePhase* ph, float angle )
{
    float tempsin = 0.0f;
    float tempcos = 0.0f;

    sincos( angle, &tempsin, &tempcos );
    ph->phA =  dq->Sd * tempsin;
    ph->phA += ( dq->Sq * tempcos );
    ph->phA += dq->S0;
    
    sincos( ( angle - DEGREES_120 ), &tempsin, &tempcos );
    ph->phB =  dq->Sd * tempsin;
    ph->phB += ( dq->Sq * tempcos );
    ph->phB += dq->S0;
    
    sincos( ( angle + DEGREES_120 ), &tempsin, &tempcos );
    ph->phC =  dq->Sd * tempsin;
    ph->phC += ( dq->Sq * tempcos );
    ph->phC += dq->S0;
}


// ********************************************************************************************************
// *
// * Function       :   cos_3_phase
// *
// * Purpose        :   function to computer cosine of an angle, angle-120, and angle + 120 in an efficient way
// *
// *
// * Parms Passed   :   radians:     	angle in radians
// *                    cos_th:      	pointer to the cos(th) value
// *                    cos_thMINUS120: pointer to the cos(th-120) value
// *                    cos_thPLUS120: 	pointer to the cos(th+120) value
// *
// * Outputs        :   side effect on cos_th, cos_thMINUS120, cos_thPLUS120
// *
// * Returns        :   nothing
// *
// ********************************************************************************************************

void cos_3_phase(float radians, float* cos_th, float* cos_thMINUS120, float* cos_thPLUS120)
{
    float sin_th = 0.0f;
    float cos_thXcos_120 = 0.0f;
    float sin_thXsin_120 = 0.0f;
    
    sincos(radians, &sin_th, cos_th); // populate sin_th and cos_th
    
    cos_thXcos_120 = (*cos_th) * COSINE_120;
    sin_thXsin_120 = sin_th    * SINE_120;
   
    *cos_thMINUS120 = cos_thXcos_120 + sin_thXsin_120; // cos(th-120) = cos(th)cos(120) + sin(th)sin(120)
    *cos_thPLUS120  = cos_thXcos_120 - sin_thXsin_120; // cos(th+120) = cos(th)cos(120) - sin(th)sin(120)
}


// ********************************************************************************************************
// *
// * Function       :   sin_3_phase
// *
// * Purpose        :   function to compute sine of an angle, angle-120, and angle + 120 in an efficient way
// *
// *
// * Parms Passed   :   radians:     	angle in radians
// *                    sin_th:      	pointer to the sin(th) value
// *                    sin_thMINUS120: pointer to the sin(th-120) value
// *                    sin_thPLUS120: 	pointer to the sin(th+120) value
// *
// * Outputs        :   side effect on sin_th, sin_thMINUS120, sin_thPLUS120
// *
// * Returns        :   nothing
// *
// ********************************************************************************************************

void sin_3_phase(float radians, float* sin_th, float* sin_thMINUS120, float* sin_thPLUS120)
{
    float cos_th = 0.0f;
    float sin_thXcos_120 = 0.0f;
    float cos_thXsin_120 = 0.0f;
    
    sincos(radians, sin_th, &cos_th); // populate sin_th and cos_th
    
    sin_thXcos_120 = (*sin_th) * COSINE_120;
    cos_thXsin_120 = cos_th    * SINE_120;
    
    *sin_thMINUS120 = sin_thXcos_120 - cos_thXsin_120; // sin(th-120) = sin(th)cos(120) - cos(th)sin(120)
    *sin_thPLUS120  = sin_thXcos_120 + cos_thXsin_120; // sin(th+120) = sin(th)cos(120) + cos(th)sin(120)
}

// ******************************************************************************************************
// *            End of ControlBlocks.c
// ******************************************************************************************************
