// *****************************************************************************
// *        ABM.C
// *****************************************************************************
// *****************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON
// *
// *****************************************************************************
// *
// *  Copyright (c) 2010 EATON
// *                      ALL RIGHTS RESERVED
// *
// *****************************************************************************
// *****************************************************************************
// *     FILE NAME:   ABM.C
// *
// *     DESCRIPTION: ABM (Advanced Battery Management) charging cycles
// *                  implementation
// *
// *     ORIGINATOR:  Tuomo Kaikkonen
// *
// *     DATE:        02/19/2004
// *
// *     HISTORY:     See HPO CVS history
// *****************************************************************************

// *****************************************************************************
// *        INCLUDE FILES
// *****************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Abm.h"
#include "Eeprom_Map.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include "Meters.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "RectifierStateControl.h"
#include "Btr.h"
#include "ParallelCan.h"


SlaveAbm::SlaveAbm()
{
}

SlaveAbm::~SlaveAbm()
{
}

void
SlaveAbm::Run()
{	
	ABM_Status.SetMasterBits(masterStatus());
	ABM_Status.bit.BatteryOV = BatteryOVSelf();
	
	AbmActions::Run();
	// Slave this node's alarm to the master.
	NB_SetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE, masterStatus().bit.BatteryOVTrip);

	AbmActions::Transfer_State(GetState());//fix APACFA-181:common battery NMC display issue 
	// Choose actions fully slaved to the master state machine
	switch (GetState())
	{
		case ABM_RESET:
			Reset_State();
			break;
		
		case ABM_CHARGE:
			Charge_State();
			break;
		
		case ABM_FLOAT:
			Float_State();
			break;
		
		case ABM_REST:
			Rest_State();
			break;
		
		case ABM_DISCHARGING:
			Discharging_State();
			break;
		
		default:
			break;
	}
}

void
SlaveAbm::ChargerCmdOff()
{
	// Ignore.  Allow the master to handle this command.
}

void
SlaveAbm::ChargerCmdOn()
{
	// Ignore.  Allow the master to handle this command.
}
	
void
SlaveAbm::BldInpChargerOff(bool action)
{
	// Preemptively disable the charger.  A building input charger off command
	// is frequently linked to H2 gas and/or temperature alarms in external
	// battery applications.  Note that the ABM master should change states
	// soon.
	if (action)
	{
		BatteryConverter.ChargerCmdOff();
	}
}

void
SlaveAbm::ResetAlarms()
{
    BatteryConverter.BatTestAbandonReason = 0;
    NB_SetNodebit(UPM_NB_BATTERY_DC_OVER_VOLTAGE, false);
}

const uAbmStatus&
SlaveAbm::masterStatus()
{
	switch (CommonBattery)
	{
		case CommonBattery_Separate:
		default:
			// This really shouldn't happen.
			return ParallelCan.MyUpmData().AbmStatus;
		
		case CommonBattery_Internal:
			return ParallelCan.UpmData[MyUPSNumber][0].AbmStatus;
		
		case CommonBattery_External:
			return ParallelCan.UpmData[0][0].AbmStatus;
	}
}

const uAbmStatus&
SlaveAbm::GetStatus()
{
	return ABM_Status;
}

eAbmState
SlaveAbm::GetState()
{
	return eAbmState(GetStatus().bit.State);
}
