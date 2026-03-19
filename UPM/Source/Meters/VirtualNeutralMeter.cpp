// *****************************************************************************
// *            VirtualNeutralMeter.cpp
// *****************************************************************************
// *****************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// *****************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// *****************************************************************************
// *****************************************************************************
// *    FILE NAME: VirtualNeutralMeter.h
// *
// *    DESCRIPTION: Provide a filtered virtual Neutral meter constructed from
// * 		the three-phase measurements of the individual source phases.
// *
// *    ORIGINATORS: Jonathan Brandmeyer
// *
// *    DATE: 10/25/2010
// *
// *    HISTORY: See Subversion history
// *****************************************************************************

#include "DSP28x_Project.h"
#include "Constants.h"
#include "Adc.h"
#include "MCUState.h"
#include "ACMeter.h"
#include "CriticalSection.h"
#include "VirtualNeutralMeter.h"
#include "RectifierStateControl.h"
#include <cmath>

namespace {
/*
 * Fsamp = 1250 # 5ms periodic
 * Fnyq = Fsamp/2
 * Fpass = 2 # don't make it too slow
 * Fcut = 45 # definitely cut off frequencies higher than this
 * 
 * # Compute filter order and normalized freq, s.t. max 3db ripple in passband
 * # and at least 20 db attenuation in the stopband
 * [n,Wn] = buttord(Fpass/Fnyq, Fcut/Fnyq, 3, 20)
 * # [n,Wn] = 1, 0.0032076
 * # Compute s-domain poles and zeros
 * [z, p] = butter(n, Wn, 's')
 * # Compute z-domain poles and zeros
 * [z_z, p_z, k_z] = butter(n, Wn)
 * # Transform z-domain into second-order-sections
 * [sos,g] = zp2sos(z_z, p_z, k_z)
 * # sos = [1 1 0 1 -0.98997 0]
 * # g = 0.0050133
 * # produce bode plot to verify
 * bode( tf(z, p))
 * 
 * Since g is the common gain, and our impl is normalized with B0 at that gain,
 * divide through BN by sos[0] (a nop since sos[0] == 1.0).
 */
const stFirstOrderIIRFP IIRNeutralGains = { 5.0133e-3, 1.0, -0.98997, 0 };
} // !namespace (anon)

// const-cast away the volatility
//	VirtualNeutralMeter VirtualInputNeutralA(
//		const_cast<const float&>(RawAdcDataPtr->st.InputVoltage.phA));
//	VirtualNeutralMeter VirtualInputNeutralB(
//		const_cast<const float&>(RawAdcDataPtr->st.InputVoltage.phB));
//	VirtualNeutralMeter VirtualInputNeutralC(
//		const_cast<const float&>(RawAdcDataPtr->st.InputVoltage.phC));
//	DirectNeutralMeter DirectInputNeutral(
//		const_cast<const float&>(RawAdcDataPtr->st.ChassisVoltage));
//	DirectNeutralMeter DirectBypassNeutral(
//		const_cast<const float&>(RawAdcDataPtr->st.AMBTemperature));
//	DirectNeutralMeter DirectInputVoltSum(
//	    const_cast<const float&>(Rectifier.UtilityPLL.SourceDQOData.S0));

VirtualNeutralMeter::VirtualNeutralMeter( const float& source)
	: FilteredNeutral(0)
	, RawNeutral(0)
	, RawSourceData(&source)
	, FilterTable(IIRNeutralGains)
{
}

VirtualNeutralMeter::VirtualNeutralMeter( const VirtualNeutralMeter& other)
	: FilteredNeutral(other.FilteredNeutral)
	, RawNeutral(other.RawNeutral)
	, RawSourceData(other.RawSourceData)
	, FilterTable(other.FilterTable)
{
}

#pragma CODE_SECTION("ramfuncs")
void VirtualNeutralMeter::ProcessRawAC(void)
{
	if (MCUStateMachine.GetADCReadyFlag())
	{
		RawNeutral = *RawSourceData;
		FilteredNeutral = FirstOrderIIRFP(RawNeutral, &FilterTable);
	}
}



#pragma CODE_SECTION("ramfuncs")
void DirectNeutralMeter::ProcessRawDC(void)
{
	if (MCUStateMachine.GetADCReadyFlag())
	{
		RawNeutral = *RawSourceData;
		FilteredNeutral = FirstOrderIIRFP(std::abs(RawNeutral), &FilterTable);
	}
}


