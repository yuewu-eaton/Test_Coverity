// ******************************************************************************************************
// *            ControlFuncs.cpp
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
#include "Constants.h"
#include "Algos.h"
#include "complex.h"

// ***********************************************************************
// *
// *    FUNCTION:  copyIIRGainsOnly
// *
// *    DESCRIPTION:  Copies the IIRFP filter gains from src to dst atomically.
// *
// *    ARGUMENTS: dst [out,nonnull] The IIRFP to receive the gains
// *               src [in]          The IIRFP to copy said gains from
// *
// *    RETURNS: Nothing
// *
// ***********************************************************************
void copyIIRGainsOnly(stSecondOrderIIRFP* dst, stSecondOrderIIRFP const & src)
{
    //CriticalSection enter(IER_DMA_ONLY);
    dst->A1 = src.A1;
    dst->A2 = src.A2;
    dst->B0 = src.B0;
    dst->B1 = src.B1;
    dst->B2 = src.B2;
}

// ***********************************************************************
// *
// *    FUNCTION:  matchedZTransform
// *
// *    DESCRIPTION:  Computes the second-order IIR coefficients for a complex filter using
// *                  the matched Z-transform.
// *
// *    ARGUMENTS: ret [out,nonnull] A second-order IIRFP structure which will receive the gains.
// *               fline [in] The line frequency, in radians/angle-step.
// *               fpole [in] The frequency and damping ratio of the target pole, as a multiple of the line frequency.  Either
// *                     of the conjugates may be passed.
// *               fzero [in] The frequency and damping ratio of the target zero, as a multiple of the line frequency.
// *                     Either of the conjugates may be passed.
// *               dcGain [in] The desired DC gain of the resulting filter structure.
// *
// *    RETURNS:  It's results though the non-null pointer 'ret'
// *
// ***********************************************************************
void matchedZTransform(stSecondOrderIIRFP* ret, float fline, complex<float> fpole, complex<float> fzero, float dcGain)
{
    complex<float> p = exp(fpole*fline);
    complex<float> z = exp(fzero*fline);
    ret->B1 = -2*z.real();
    ret->B2 = norm(z);
    ret->A1 = -2*p.real();
    ret->A2 = norm(p);
    
    ret->B0 = dcGain*(1 + ret->A1 + ret->A2)/(1 + ret->B1 + ret->B2);
}

// ***********************************************************************
// *
// *    FUNCTION:  phaseAdjustableResonantCompensator
// *
// *    DESCRIPTION:  Computes the complex pole and zero locations for a phase-adjustable resonant compensator
// *                  See "Classical Control Revisited: Variations on a Theme" by Messner for details.
// *
// *    ARGUMENTS: fpole [out,nonnull] The complex pole location, in the S-plane
// *               fzero [out,nonnull] The complex zero location, in the S-plane
// *               frequency [in] The desired resonant peak frequency, in radians/second
// *               phase [in] The desired phase of the filter at @param frequency, in radians
// *               inversePeakGain [in] 1/ the desired peak gain of the filter (approximate).
// *
// *    RETURNS:  It's results though pointers fpole and fzero
// *
// ***********************************************************************
void phaseAdjustableResonantCompensator(complex<float>* fpole, complex<float>* fzero, float frequency, float phase, float inversePeakGain)
{
    const float damping_zero = 0.09f;
    float damping_pole = damping_zero * inversePeakGain;
    float tanPhase = tan(phase);
    float zAdjust = damping_zero*tanPhase + sqrt(damping_zero*damping_zero*tanPhase*tanPhase + 1);
    float freq_zero = zAdjust*frequency;
    *fpole = complex<float>(-frequency*damping_pole, frequency*sqrt(1 - damping_pole*damping_pole));
    *fzero = complex<float>(-freq_zero*damping_zero, freq_zero*sqrt(1 - damping_zero*damping_zero));
}

// ******************************************************************************************************
// *            End of ControlBlocks.c
// ******************************************************************************************************
