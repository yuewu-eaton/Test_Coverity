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
#include "Version.h"

float  BattVoltsperCell;     // Battery voltage in volts/cell
float  BattVoltsperCellBU;   // Redundant battery voltage measurement, V/cell
float  BattVoltsperCell1;
float  BattVoltsperCell2;
float  BattFloatV_Comp;      // The temperature compensated limit where charge mode changes to float mode
float  Batt_Disconnect_VPC = 1.574f;  // EEP, min volt/cell

#define DEFAULT_BATTERY_OV_TIME (30u*10)
#define BATTERY_OV_DEBOUNCE 2u

AbmActions::AbmActions()
	: BatteryOVAlarmTimeout(DEFAULT_BATTERY_OV_TIME)
{
    ABM_Status.words[0] = 0;
    ABM_Status.words[1] = 0;
	Timer1.ClearTimer();
}

AbmActions::~AbmActions()
{
}

void
AbmActions::Run( void )
{
	NB_SetNodebit( UPM_NB_COMMON_BATTERY, CommonBattery != CommonBattery_Separate);
	if (MCUStateMachine.GetMetersReadyFlag())
	{
		// calculate battery cell voltage, Volts/cell
	    BattVoltsperCell = BatteryVoltage.SlowFiltered / (float)NumberOfBatteryCells;
	    BattVoltsperCellBU = BatteryVoltageBU.SlowFiltered / (float)NumberOfBatteryCells;

		if((InterfaceBoardRevID == CONST_InterfaceBrdRev_ID_P0) || 
			(InterfaceBoardRevID == CONST_InterfaceBrdRev_ID_P1))	//Board P0/1, no Vbat+ sample
		{
			if(!NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_OPEN )) //for hobbit-72, no bat+- sample
			{
				BattVoltsperCell1 = 2.0 * BatteryVoltageChgPos.SlowFiltered / (float)NumberOfBatteryCells;
				BattVoltsperCell2 = -2.0 * BatteryVoltageChgNeg.SlowFiltered / (float)NumberOfBatteryCells;
			}
			else
			{
				BattVoltsperCell1 = BattVoltsperCell;
				BattVoltsperCell2 = BattVoltsperCell;	
			}
		}
		else														//Board P2 or more, with Vbat+
		{
			BattVoltsperCell1 = 2.0 * BatteryVoltagePos.SlowFiltered / (float)NumberOfBatteryCells;
			BattVoltsperCell2 = -2.0 * BatteryVoltageNeg_SlowFiltered / (float)NumberOfBatteryCells;
		}

        BattFloatV_Comp = BattConstFloatVPC * (float)NumberOfBatteryCells;
	}
}

/*
 * Function:   bool BatteryOV( void )
 * Purpose:     Determine if this node's battery pack has an over-voltage alarm
 *              active.
 * 
 * Description: Return true if any charger that is connected to this battery
 *              pack is observing an over-voltage alarm.  False otherwise.
 * 
 */
bool 
AbmActions::BatteryOVSelf() const
{
	return ((BattVoltsperCell >= BatteryOVLevelVPC)            ||
		(BattVoltsperCell1 >= BatteryOVLevelVPC)               ||
		(BattVoltsperCell2 >= BatteryOVLevelVPC)               ||
		BatteryConverter.GetBatteryStateStatus().bit.ChargerFailOV );
}

void
AbmActions::Transfer_State( eAbmState newState )
{
	NB_DebounceAndQue( UPM_NB_ABM_CHARGE_MODE, newState == ABM_CHARGE );
	NB_DebounceAndQue( UPM_NB_ABM_FLOAT_MODE,  newState == ABM_FLOAT  );
	NB_DebounceAndQue( UPM_NB_ABM_REST_MODE,   newState == ABM_REST   );
	NB_DebounceAndQue( UPM_NB_ABM_OFF,         newState == ABM_RESET  );
	
	Timer1.ClearTimer();
}

void
AbmActions::SetOVTime(uint16_t deciseconds)
{
	BatteryOVAlarmTimeout = deciseconds;
}

void
AbmActions::Charge_State( void )
{
    BatteryConverter.SetChargeVoltage( ( BattABMFloatVPC - CompensateVPC) * (float)NumberOfBatteryCells + 5.0 );
    BatteryConverter.ChargerCmdOn();
}

void
AbmActions::Reset_State( void )
{
    BatteryConverter.SetChargeVoltage( ( BattABMFloatVPC - CompensateVPC) * (float)NumberOfBatteryCells + 5.0 );
	BatteryConverter.ChargerCmdOff();
}

void
AbmActions::Float_State( void )
{
	if ( ABMDisabled )
    {
        BatteryConverter.SetChargeVoltage( ( BattConstFloatVPC - CompensateVPC ) * (float)NumberOfBatteryCells );
    }
    else
    {
        BatteryConverter.SetChargeVoltage( ( BattABMFloatVPC - CompensateVPC ) * (float)NumberOfBatteryCells );
    }
	if (Timer1.CheckTimeout(START_CHRG_TIME))
	{
		BatteryConverter.ChargerCmdOn();
	}
}

void
AbmActions::Rest_State( void )
{
	BatteryConverter.ChargerCmdOff();
}

void
AbmActions::Discharging_State( void )
{
	BatteryConverter.ChargerCmdOff();
}
