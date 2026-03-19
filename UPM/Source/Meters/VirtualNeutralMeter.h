// *****************************************************************************
// *            VirtualNeutralMeter.h
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

#ifndef VIRTUALNEUTRALMETER_H_
#define VIRTUALNEUTRALMETER_H_

#include "Algos.h"

class VirtualNeutralMeter
{
public:
	VirtualNeutralMeter( const float& source);
	~VirtualNeutralMeter() {}
	
protected:
	friend class Debugger;
	VirtualNeutralMeter(const VirtualNeutralMeter& other);

public:
	void ProcessRawAC(void);
	float FilteredNeutral;
	
protected:
	float RawNeutral;
	const float* RawSourceData;
	stFirstOrderIIRFP FilterTable;
};

class DirectNeutralMeter : public VirtualNeutralMeter
{
protected:
	DirectNeutralMeter(const DirectNeutralMeter& other)
		: VirtualNeutralMeter(other)
	{}
	
	friend class Debugger;
	
public:
	DirectNeutralMeter( const float& source)
		: VirtualNeutralMeter(source)
	{}
	
	~DirectNeutralMeter() {}
	
	void ProcessRawDC(void);
};

//	extern VirtualNeutralMeter VirtualInputNeutralA;
//	extern VirtualNeutralMeter VirtualInputNeutralB;
//	extern VirtualNeutralMeter VirtualInputNeutralC;
//	extern DirectNeutralMeter DirectInputNeutral;
//	extern DirectNeutralMeter DirectBypassNeutral;
//	extern DirectNeutralMeter DirectInputVoltSum;
#endif /*VIRTUALNEUTRALMETER_H_*/
