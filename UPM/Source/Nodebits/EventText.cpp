// ******************************************************************************************************
// *            EventText.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: EventText.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 4/23/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "UPM_NB_Ids.h"
#include "Xcpdefs.h"

struct stEventText
{
    uint16_t num;
    const char* text;
};

const stEventText EventTextEnglish[] =
{
    // Alarm Text for: UPM_NB_INVERTER_AC_UNDER_VOLTAGE
    { INVERTER_AC_UNDER_VOLTAGE,
        "Inverter AC Under Voltage"
    },

    // Alarm Text for: UPM_NB_BYPASS_AC_OVER_VOLTAGE
    { BYPASS_AC_OVER_VOLTAGE,
        "Bypass AC Over Voltage"
    },

    // Alarm Text for: UPM_NB_BYPASS_AC_UNDER_VOLTAGE
    { BYPASS_AC_UNDER_VOLTAGE,
        "Bypass AC Under Voltage"
    },

    // Alarm Text for: UPM_NB_BYPASS_UNDER_OVER_FREQUENCY
    { BYPASS_UNDER_OVER_FREQUENCY,
        "Bypass Under/Over Frequency"
    },

    // Alarm Text for: UPM_NB_INPUT_AC_OVER_VOLTAGE
    { INPUT_AC_OVER_VOLTAGE,
        "Input AC Over Voltage"
    },

    // Alarm Text for: UPM_NB_INPUT_AC_UNDER_VOLTAGE
    { INPUT_AC_UNDER_VOLTAGE,
        "Input AC Under Voltage"
    },

    // Alarm Text for: UPM_NB_INPUT_UNDER_OVER_FREQUENCY
    { INPUT_UNDER_OVER_FREQUENCY,
        "Input Under/Over Frequency"
    },

    // Alarm Text for: UPM_NB_OUTPUT_AC_OVER_VOLTAGE
    { OUTPUT_AC_OVER_VOLTAGE,
        "Output AC Over Voltage"
    },

    // Alarm Text for: UPM_NB_OUTPUT_AC_UNDER_VOLTAGE
    { OUTPUT_AC_UNDER_VOLTAGE,
        "Output AC Under Voltage"
    },

    // Alarm Text for: UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY
    { OUTPUT_UNDER_OVER_FREQUENCY,
        "Output Under/Over Frequency"
    },

    // Alarm Text for: UPM_NB_REMOTE_EMERGENCY_POWER_OFF
    { REMOTE_EMERGENCY_POWER_OFF,
        "Remote Emergency Power Off"
    },

    // Alarm Text for: UPM_NB_INVERTER_OVERTEMPERATURE
    { INVERTER_OVER_TEMPERATURE,
        "Inverter Over Temperature"
    },

    // Alarm Text for: UPM_NB_OUTPUT_OVERLOAD
    { OUTPUT_OVERLOAD,
        "Output Overload"
    },

    // Alarm Text for: UPM_NB_RECTIFIER_INPUT_OVER_CURRENT
    { RECTIFIER_INPUT_OVER_CURRENT,
        "Rectifier Input Over Current"
    },

    // Alarm Text for: UPM_NB_INVERTER_OUTPUT_OVER_CURRENT
    { INVERTER_OUTPUT_OVER_CURRENT,
        "Inverter Output Over Current"
    },

    // Alarm Text for: UPM_NB_DC_LINK_OVER_VOLTAGE
    { DC_LINK_OVER_VOLTAGE,
        "DC Link Over Voltage"
    },

    // Alarm Text for: UPM_NB_DC_LINK_UNDER_VOLTAGE
    { DC_LINK_UNDER_VOLTAGE,
        "DC Link Under Voltage"
    },

    // Alarm Text for: UPM_NB_STATIC_SWITCH_FAILURE
    { STATIC_SWITCH_FAILURE,
        "Static Switch Failure"
    },

    // Alarm Text for: UPM_NB_BATTERY_CURRENT_LIMIT
    { BATTERY_CURRENT_LIMIT,
        "Battery Over Current"
    },

    // Alarm Text for: UPM_NB_NON_VOLATILE_RAM_FAILURE
    { EEPROM_FAULT,
        "EEPROM Checksum Failure"
    },

    // Alarm Text for: UPM_NB_SHUTDOWN_IMMINENT
    { SHUTDOWN_IMMINENT,
        "Shutdown Imminent"
    },

    // Alarm Text for: UPM_NB_BATTERY_LOW
    { BATTERY_LOW,
        "Low Battery Warning"
    },

    // Alarm Text for: UPM_NB_OUTPUT_SHORT_CIRCUIT
    { OUTPUT_SHORT_CIRCUIT,
        "Output Short Circuit"
    },

    // Alarm Text for: UPM_NB_UTILITY_NOT_PRESENT
    { UTILITY_NOT_PRESENT,
        "Utility Not Present"
    },

    // Alarm Text for: UPM_NB_BATTERY_DC_OVER_VOLTAGE
    { BATTERY_DC_OVER_VOLTAGE,
        "Battery DC Over Voltage"
    },

    // Alarm Text for: UPM_NB_POWER_SUPPLY_FAILURE
    { POWER_SUPPLY_FAILURE,
        "Power Supply Failure"
    },
        
    // Alarm Text for: UPM_NB_TO_BYPASS_COMMAND
    { TO_BYPASS_COMMAND,
        "To Bypass Command"
    },

    // Alarm Text for: UPM_NB_BYPASS_NOT_AVAILABLE
    { BYPASS_NOT_AVAILABLE,
        "Bypass Not Available"
    },

    // Alarm Text for: UPM_NB_BATTERY_CONTACTOR_OPEN
    { BATTERY_CONTACTOR_OPEN,
        "Battery Relay Open"
    },

    // Alarm Text for: UPM_NB_RECTIFIER_PHASE_ROTATION
    { RECTIFIER_PHASE_ROTATION,
        "Rectifier Phase Rotation"
    },

    // Alarm Text for: UPM_NB_BYPASS_PHASE_ROTATION
    { BYPASS_PHASE_ROTATION,
        "Bypass Phase Rotation"
    },

    // Alarm Text for: UPM_NB_BYPASS_PHASE_ROTATION
    { OUTPUT_PHASE_ROTATION,
        "Output Phase Rotation"
    },

    { LOSS_OF_SYNC_BUS,
        "Loss of Sync Bus"
    },

    // Alarm Text for: UPM_NB_CONTROL_POWER_ON
    { CONTROL_POWER_ON,
        "Control Power On"
    },

    // Alarm Text for: UPM_NB_INVERTER_ON
    { INVERTER_ON,
        "Inverter On"
    },

    // Alarm Text for: UPM_NB_CHARGER_ON
    { CHARGER_ON,
        "Charger On"
    },

    // Alarm Text for: UPM_NB_INTERNAL_MBS_ACTIVE
    { INTERNAL_MBS_ACTIVE,
        "Internal MBS Active"
    },

    // Alarm Text for: UPM_NB_MBS_CLOSED
    { MBS_CLOSED,
        "MBS Closed"
    },

    // Alarm Text for: UPM_NB_BATTERY_NEEDS_SERVICE
    { BATTERY_NEEDS_SERVICE,
        "Service Battery"
    },

    // Alarm Text for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_A
    { LEVEL_2_OVERLOAD_PHASE_A,
        "L1 Overload"
    },

    // Alarm Text for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_B
    { LEVEL_2_OVERLOAD_PHASE_B,
        "L2 Overload"
    },

    // Alarm Text for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_C
    { LEVEL_2_OVERLOAD_PHASE_C,
        "L3 Overload"
    },

    // Alarm Text for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_A
    { LEVEL_3_OVERLOAD_PHASE_A,
        "L1 Overload (High Level)"
    },

    // Alarm Text for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_B
    { LEVEL_3_OVERLOAD_PHASE_B,
        "L2 Overload (High Level)"
    },

    // Alarm Text for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_C
    { LEVEL_3_OVERLOAD_PHASE_C,
        "L3 Overload (High Level)"
    },

    // Alarm Text for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_A
    { LEVEL_4_OVERLOAD_PHASE_A,
        "L1 Overload (Extreme Level)"
    },

    // Alarm Text for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_B
    { LEVEL_4_OVERLOAD_PHASE_B,
        "L2 Overload (Extreme Level)"
    },

    // Alarm Text for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_C
    { LEVEL_4_OVERLOAD_PHASE_C,
        "L3 Overload (Extreme Level)"
    },

    // Alarm Text for: UPM_NB_UPS_ON_BATTERY
    { UPS_ON_BATTERY,
        "UPS On Battery"
    },

    // Alarm Text for: UPM_NB_UPS_ON_BYPASS
    { UPS_ON_BYPASS,
        "UPS On Bypass"
    },

    // Alarm Text for: UPM_NB_LOAD_POWER_OFF
    { LOAD_POWER_STATUS,
        "Load Power Off"
    },

    // Alarm Text for: UPM_NB_UPS_ON_COMMAND
    { UPS_ON_COMMAND,
        "UPS On Command"
    },

    // Alarm Text for: UPM_NB_UPS_OFF_COMMAND
    { LOAD_OFF_COMMAND,
        "UPS Off Command"
    },

    // Alarm Text for: UPM_NB_LOW_BATTERY_SHUTDOWN
    { LOW_BATTERY_SHUTDOWN,
        "Low Battery Shutdown"
    },

    // Alarm Text for: UPM_NB_SOFTWARE_INCOMPATIBILITY_DETECTED
    { SOFTWARE_INCOMPATIBILITY_DETECTED,
        "Software Incompatibility Detected"
    },

    // Alarm Text for: UPM_NB_BATTERY_TEST_FAILED
    { BATTERY_TEST_FAILED,
        "Battery Test Failed"
    },

    // Alarm Text for: UPM_NB_FUSE_FAILURE
    { FUSE_FAILURE,
        "Fuse Failure"
    },

    // Alarm Text for: UPM_NB_FAN_FAILURE
    { FAN_FAILURE,
        "Fan Failure"
    },

    // Alarm Text for: UPM_NB_SITE_WIRING_FAULT
    { SITE_WIRING_FAULT,
        "Site Wiring Fault"
    },

    // Alarm Text for: UPM_NB_BACKFEED_CONTACTOR_FAILURE
    { BACKFEED_CONTACTOR_FAILURE,
        "Backfeed Relay Failure"
    },

    // Alarm Text for: UPM_NB_BATTERIES_DISCONNECTED
    { BATTERIES_DISCONNECTED,
        "Batteries Disconnected"
    },

    // Alarm Text for: UPM_NB_AMBIENT_OVERTEMPERATURE
    { AMBIENT_OVERTEMPERATURE,
        "Ambient Over Temperature"
    },

    // Alarm Text for: UPM_NB_SELECTIVE_TRIP_OF_MODULE
    { SELECTIVE_TRIP_OF_MODULE,
        "UPM Selective Trip"
    },

    // Alarm Text for: UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP
    { ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP,
        "Abnormal Output Voltage Detected"
    },

    // Alarm Text for: UPM_NB_RECTIFIER_OVERTEMPERATURE
    { RECTIFIER_OVERTEMPERATURE,
        "Rectifier Over Temperature"
    },

    // Alarm Text for: UPM_NB_CONFIGURATION_ERROR
    { CONFIGURATION_ERROR,
        "Configuration Error"
    },

    // Alarm Text for: UPM_NB_REDUNDANCY_LOSS_DUE_TO_OVERLOAD
    { REDUNDANCY_LOSS_DUE_TO_OVERLOAD,
        "System Not Redundant"
    },

    // Alarm Text for: UPM_NB_IN_HIGH_EFFICIENCY_MODE
    { IN_HIGH_EFFICIENCY_MODE,
        "In High Efficiency Mode"
    },

    // Alarm Text for: UPM_NB_UPS_ON_NORMAL
    { UPS_ON_NORMAL,
        "UPS System Normal"
    },

    // Alarm Text for: UPM_NB_INVALID_BOARD_ID
    { INVALID_BOARD_ID,
        "Invalid Board ID"
    },

    // Alarm Text for: UPM_NB_CHECK_PRECHARGE
    { PRECHARGE_FAILED,
        "Precharge failed"
    },

    // Alarm Text for: UPM_NB_RECTIFIER_ON
    { RECTIFIER_STATUS,
        "Rectifier On"
    },

    // Alarm Text for: UPM_NB_UPS_ON_GENERATOR
    { UPS_ON_GENERATOR,
        "UPS On Generator"
    },

    // Alarm Text for: UPM_NB_BATTERY_TEST_IN_PROGRESS
    { BATTERY_TEST_IN_PROGRESS,
        "Battery Test In Progress"
    },

    // Alarm Text for: UPM_NB_CLOCK_SET_DONE
    { CLOCK_SET_DONE,
        "Clock Set Done"
    },

    // Alarm Text for: UPM_NB_IN_APM_MODE
    { IN_APM_MODE,
        "VMMS On"
    },

    // Alarm Text for: UPM_NB_ABM_CHARGE_MODE
    { ABM_CHARGE_MODE,
        "ABM State Charging"
    },

    // Alarm Text for: UPM_NB_ABM_FLOAT_MODE
    { ABM_FLOAT_MODE,
        "ABM State Floating"
    },

    // Alarm Text for: UPM_NB_ABM_REST_MODE
    { ABM_REST_MODE,
        "ABM State Resting"
    },

    // Alarm Text for: UPM_NB_ABM_OFF
    { ABM_STATE_OFF,
        "ABM State OFF"
    },

    // Alarm Text for: UPM_NB_INVERTER_CONTACTOR_CLOSED
    { INVERTER_CONTACTOR_CLOSED,
        "Inverter Relay Closed"
    },

    // Alarm Text for: UPM_NB_TOO_MANY_INVERTER_TRANSFERS
    { TOO_MANY_INVERTER_TRANSFERS,
        "Too many Inv Transfers"
    },

    // Alarm Text for: UPM_NB_STATIC_SWITCH_SHORT
    { STATIC_SWITCH_SHORT,
        "Static Switch Short"
    },

    // Alarm Text for: UPM_NB_PULL_CHAIN
    { PULL_CHAIN,
        "Pull Chain"
    },

    // Alarm Text for: UPM_NB_CHECK_PULL_CHAIN
    { CHECK_PULL_CHAIN,
        "Check Pull Chain"
    },

    // Alarm Text for: UPM_NB_NORMAL_MODE_COMMAND
    { NORMAL_COMMAND,
        "Normal Mode Command"
    },
    
    // Alarm Text for: UPM_NB_RECTIFIER_FAILED
    { RECTIFIER_FAILED,
        "Rectifier Failed to Start"
    },

    // Alarm Text for: UPM_NB_INVERTER_CONTACTOR_FAILURE
    { CHECK_INVERTER_SWITCHGEAR,
        "Inverter Switchgear Failed"
    },

    // Alarm Text for: UPM_NB_AUTOMATIC_STARTUP_PENDING
    { AUTOMATIC_STARTUP_PENDING,
        "Automatic Startup Pending"
    },
    
    // Alarm Text for: UPM_NB_CHARGER_ON_COMMAND
    { CHARGER_ON_COMMAND,
        "Charger On Command"
    },
    
    // Alarm Text for: UPM_NB_CHARGER_OFF_COMMAND
    { CHARGER_OFF_COMMAND,
        "Charger Off Command"
    },

    // Alarm Text for: UPM_NB_CHARGER_FAILURE
    { CHARGER_FAILURE,
        "Charger Failure"
    },
    
    // Alarm Text for: UPM_NB_BATTERY_CONTACTOR_FAIL
    { BATTERY_CONTACTOR_FAIL,
        "Battery Relay Failure"
    },

    // Alarm Text for: UPM_NB_OUTPUT_OVERLOAD_TRIP
    { OUTPUT_OVERLOAD_TRIP,
        "Output Overload Trip"
    },

    // Alarm Text for: UPM_NB_POWER_SUPPLY_5_VOLT_FAULT
    { POWER_SUPPLY_5_VOLT_FAULT,
        "Power Supply 5v Fault"
    },
    
    // Alarm Text for: UPM_NB_POWER_SUPPLY_15_VOLT_FAULT
    { POWER_SUPPLY_15_VOLT_FAULT,
        "Power Supply 15v Fault"
    },

    // Alarm Text for: UPM_NB_ECO_ENABLE
    { ECO_ENABLE,
        "ECO Mode Enabled"
    },
    { MCU_STATE_CHANGE,
        "MCU State Changed"
    },
    { BYPASS_STATE_CHANGE,
        "Bypass State Changed"
    },
    { RECTIFIER_STATE_CHANGE,
        "Rectifier State Changed"
    },
    { BATTERY_STATE_CHANGE,
        "Battery State Changed"
    },
    { INPUT_SYNC_OUT_OF_RANGE,
        "Input Sync Out Of Range"
    },
    { INVERTER_STARTUP_FAILURE,
        "Inverter Startup Failure"
    },
    { IN_EASY_CAPACITY_TEST_MODE,
        "Easy Capacity Test"
    },
    { I2C_BUS_FAIL,
        "I2C Bus Failure"
    },
    // UPM_NB_RECTIFIER_SWITCHGEAR_OPEN
    { RECTIFIER_SWITCHGEAR_OPEN,
        "Rectifier Switchgear Open"
    },
    // UPM_NB_ECT_MODE_COMMAND
    { ECT_COMMAND,
        "Easy Capacity Test Command"
    },
    // UPM_NB_ABNORMAL_EXIT_ECT_MODE
    { ABNORMAL_EXIT_ECT_MODE,
        "ECT Failure"
    },
    // UPM_NB_PARALLEL_SETUP_FAIL
    { PARALLEL_SETUP_FAIL,
        "Parallel Setup Failure"
    },
    // UPM_NB_SYSTEM_TEST_IN_PROGRESS
    { SYSTEM_TEST_IN_PROGRESS,
        "System Test in Progress"
    },
    // UPM_NB_SYNC_STATE_CHANGED
    { SYNC_STATE_CHANGE,
        "Sync State Changed"
    },
    // UPM_NB_PARALLEL_CAN_ERROR
    { PARALLEL_BOARD_FAILURE,
        "Parallel CAN Failure"
    },
    // UPM_NB_MOB_OPEN
    { MOB_OPEN,
        "MOB Open"
    },
    // UPM_NB_MOB_FAILURE
    { MOB_FAILURE,
        "MOB Failure"
    },
    // UPM_NB_EXTERNAL_PARALLEL
    {
        EXTERNAL_PARALLEL,
        "External Parallel System"
    },
    // UPM_NB_SYSTEM_IS_REDUNDANT
    {
        SYSTEM_IS_REDUNDANT,
        "System is Redundant"
    },
    // UPM_NB_DRIVER_FAULT
    {
        DRIVER_FAULT,
        "Driver Voltage Fault"
    },
    // Alarm Text for: UPM_NB_PM_OVERTEMPERATURE
    { PM_OVER_TEMPERATURE,
        "PM Over Temperature Warning"
    },
    // Alarm Text for: UPM_NB_PM_OVERTEMPERATURE_TRIP
	{ PM_OVER_TEMPERATURE_TRIP,
	   "PM Over Temperature Trip"
	},
    // Alarm Text for: UPM_NB_BALANCER_LOOP_OVER_CURRENT
    { BALANCER_LOOP_OVER_CURRENT,
        "Balancer Loop Over Current"
    },
    // Alarm Text for: UPM_NB_BATTERY_BREAKER_FAILURE
    { BATTERY_BREAKER_FAILURE,
        "Battery Breaker Failure"
    },
    // Alarm Text for: UPM_NB_TOO_MANY_BATTERY_TRANSFERS
    { TOO_MANY_BATTERY_TRANSFERS,
        "Too Many Battery Transfers"
    },

    //Alarm Test for: UPM_NB_SPI_BUS_FAILURE
    {SPI_BUS_FAILURE,
    	"SPI Bus Failure"
    },

    //Alarm Test for: PARALLEL_LOAD_SHARE_ERROR
    {PARALLEL_LOAD_SHARE_ERROR,
    	"Parallel Load Share Error"
    },
    
    //Alarm Test for: UPM_NB_HE_NOT_AVAILABLE
    {HE_NOT_AVAILABLE,
        "High Efficiency Not Available"
    },
    //Alarm Test for: UPM_NB_INPUT_VOLTAGE_ABNORMAL
	{INPUT_ABNORMAL,
        "Input Voltage Abnormal"
	},
	// UPM_NB_BATTERY_STARTUP_FAILURE
	{ BATTERY_STARTUP_FAILURE,
		"Battery Startup Failure"
	},
	//Alarm Test for: UPM_NB_PLD_CONFIG_FAIL
	{PLD_CONFIG_FAIL,
		"PLD DeadTime Config Fail"
	},
	//Alarm Test for: UPM_NB_DISCHARGE_DC_LINK
	{DISCHARGE_DCLINK,
        "Discharge DC Link"
	},

    //Alarm Test for: UPM_NB_REC_PWM_OFF
    {REC_PWM_OFF,
        "Rectifier PWM Off"
    },

    //Alarm Test for: UPM_NB_ADAPTIVE_OVERLOAD_CAPACITY_OFF
    {ADAPTIVE_OVERLOAD_CAPACITY_OFF,
        "ADAPTIVE OVERLOAD Off"
    },

    //Alarm Test for: UPM_NB_STS_OVER_TEMPERATURE
    {STS_OVER_TEMPERATURE,
        "STS Over Temperature Warning"
    },

	//Alarm Test for: UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R
	{ RECTIFIER_SWITCHGEAR_OPEN_R,
		"Rectifier Switchgear R Open"
	},

	//Alarm Test for: UPM_NB_BAT_LEGB_RELAY_OPEN
	{ BAT_LEGB_RELAY_OPEN,
		"Bat LegB Relay Open"
	},		

	//Alarm Test for: UPM_NB_LOSS_OF_PWM_SYNC
	{ LOSS_OF_PWM_SYNC,
		"Loss of PWM Sync"
	},		

    // Alarm Test for: UPM_NB_BACKFEED_RELAY_CLOSE
    {
        BACKFEED_RELAY_CLOSE,
        "Backfeed Switchgear Closed"
    },

};

const uint16_t EventTextTableSize = sizeof( EventTextEnglish ) / sizeof( EventTextEnglish[0] );

const char* GetEventText( uint16_t strNum )
{
    const char* textPtr = NULL;

    for ( uint16_t idx = 0; idx < EventTextTableSize; idx++ )
    {
        if ( EventTextEnglish[ idx ].num == strNum )
        {
            textPtr = (const char*)EventTextEnglish[ idx ].text;
            break;
        }
    }

    return textPtr;
}

// ******************************************************************************************************
// *            End of EventText.cpp
// ******************************************************************************************************
