// ********************************************************************************************************
// *            NB_Config.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO INVENSYS Powerware Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Invensys Powerware
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: NB_Config.c
// *
// *    DESCRIPTION: Creates and defines the config and status structures required to use nodebits
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/16/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "NB_Config.h"
#include "XCPDefs.h"

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************
// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * AlarmSetup Table
// ********************************************************************************************************

const st_NB_Cfg NB_Cfg_Flash[UPM_NB_PRIVATE_SUPPORTED_EVENTS] = // Nodebit Configuration table in Flash
{
    // Nodebit Configuration for: UPM_NB_INVERTER_AC_UNDER_VOLTAGE       // 0
    { PASS_4, PASS_10, INVERTER_AC_UNDER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_BYPASS_AC_OVER_VOLTAGE          // 1
    { PASS_11, PASS_5102, BYPASS_AC_OVER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_BYPASS_AC_UNDER_VOLTAGE         // 2
    { PASS_11, PASS_5102, BYPASS_AC_UNDER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_BYPASS_UNDER_OVER_FREQUENCY     // 3
    { PASS_2, PASS_4, BYPASS_UNDER_OVER_FREQUENCY, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_INPUT_AC_OVER_VOLTAGE           // 4
    { PASS_32, PASS_200, INPUT_AC_OVER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_INPUT_AC_UNDER_VOLTAGE          // 5
    { PASS_4, PASS_200, INPUT_AC_UNDER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_INPUT_UNDER_OVER_FREQUENCY      // 6
    { PASS_8, PASS_64, INPUT_UNDER_OVER_FREQUENCY, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_OUTPUT_AC_OVER_VOLTAGE          // 7
    { PASS_8, PASS_64, OUTPUT_AC_OVER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_OUTPUT_AC_UNDER_VOLTAGE         // 8
    { PASS_3, PASS_64, OUTPUT_AC_UNDER_VOLTAGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY     // 9
    { PASS_3, PASS_64, OUTPUT_UNDER_OVER_FREQUENCY, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_REMOTE_EMERGENCY_POWER_OFF      // 10
    { PASS_1, PASS_1, REMOTE_EMERGENCY_POWER_OFF, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_INVERTER_OVERTEMPERATURE        // 11
    { PASS_16, PASS_64, INVERTER_OVER_TEMPERATURE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_OUTPUT_OVERLOAD                 // 12
    { PASS_16, PASS_16, OUTPUT_OVERLOAD, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_INPUT_OVER_CURRENT    // 13
    { PASS_1, PASS_1, RECTIFIER_INPUT_OVER_CURRENT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_INVERTER_OUTPUT_OVER_CURRENT    // 14
    { PASS_1, PASS_64, INVERTER_OUTPUT_OVER_CURRENT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_DC_LINK_OVER_VOLTAGE            // 15
    { PASS_2, PASS_5102, DC_LINK_OVER_VOLTAGE, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_DC_LINK_UNDER_VOLTAGE           // 16
    { PASS_1, PASS_5102, DC_LINK_UNDER_VOLTAGE, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_STATIC_SWITCH_FAILURE           // 17
    { PASS_8, PASS_64, STATIC_SWITCH_FAILURE, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_BATTERY_CURRENT_LIMIT           // 18
    { PASS_1, PASS_64, BATTERY_CURRENT_LIMIT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_NON_VOLATILE_RAM_FAILURE        // 19
    { PASS_1, PASS_1, EEPROM_FAULT, QUE_ALL, USR_AL, },    
    // Nodebit Configuration for: UPM_NB_SHUTDOWN_IMMINENT               // 20
    { PASS_1, PASS_1, SHUTDOWN_IMMINENT, QUE_ALL, USR_AL, },    
    // Nodebit Configuration for: UPM_NB_BATTERY_LOW                     // 21
    { PASS_16, PASS_16, BATTERY_LOW, QUE_ALL, USR_AL, },    
    // Nodebit Configuration for: UPM_NB_OUTPUT_SHORT_CIRCUIT            // 22
    { PASS_1, PASS_1, OUTPUT_SHORT_CIRCUIT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_UTILITY_NOT_PRESENT             // 23
    { PASS_8, PASS_2551, UTILITY_NOT_PRESENT, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_BATTERY_DC_OVER_VOLTAGE         // 24
    { PASS_1, PASS_1, BATTERY_DC_OVER_VOLTAGE, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_POWER_SUPPLY_FAILURE            // 25
    { PASS_1, PASS_10, POWER_SUPPLY_FAILURE, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_TO_BYPASS_COMMAND               // 26
    { PASS_1, PASS_1, TO_BYPASS_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },    
    // Nodebit Configuration for: UPM_NB_BYPASS_NOT_AVAILABLE            // 27
    { PASS_1, PASS_1, BYPASS_NOT_AVAILABLE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_PHASE_ROTATION        // 28
    { PASS_16, PASS_16, RECTIFIER_PHASE_ROTATION, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_BYPASS_PHASE_ROTATION           // 29
    { PASS_32, PASS_16, BYPASS_PHASE_ROTATION, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_CONTROL_POWER_ON                // 30
    { PASS_1, PASS_1, CONTROL_POWER_ON, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_INVERTER_ON                     // 31
    { PASS_1, PASS_1, INVERTER_ON, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_CHARGER_ON                      // 32
    { PASS_1, PASS_1, CHARGER_ON, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_INTERNAL_MBS_ACTIVE             // 33
    { PASS_1, PASS_1, INTERNAL_MBS_ACTIVE, QUE_ALL, NOTC1 },    
    // Nodebit Configuration for: UPM_NB_BATTERY_NEEDS_SERVICE           // 34
    { PASS_8, PASS_8, BATTERY_NEEDS_SERVICE, QUE_ALL, SVC_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_A        // 35
    { PASS_1, PASS_8, LEVEL_2_OVERLOAD_PHASE_A, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_B        // 36
    { PASS_1, PASS_8, LEVEL_2_OVERLOAD_PHASE_B, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_2_OVERLOAD_PHASE_C        // 37
    { PASS_1, PASS_8, LEVEL_2_OVERLOAD_PHASE_C, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_A        // 38
    { PASS_8, PASS_4, LEVEL_3_OVERLOAD_PHASE_A, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_B        // 39
    { PASS_8, PASS_4, LEVEL_3_OVERLOAD_PHASE_B, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_3_OVERLOAD_PHASE_C        // 40
    { PASS_8, PASS_4, LEVEL_3_OVERLOAD_PHASE_C, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_A        // 41
    { PASS_8, PASS_4, LEVEL_4_OVERLOAD_PHASE_A, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_B        // 42
    { PASS_8, PASS_4, LEVEL_4_OVERLOAD_PHASE_B, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_LEVEL_4_OVERLOAD_PHASE_C        // 43
    { PASS_8, PASS_4, LEVEL_4_OVERLOAD_PHASE_C, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_UPS_ON_BATTERY                  // 44
    { PASS_1, PASS_1, UPS_ON_BATTERY, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_UPS_ON_BYPASS                   // 45
    { PASS_1, PASS_1, UPS_ON_BYPASS, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_LOAD_POWER_OFF                  // 46
    { PASS_1, PASS_1, LOAD_POWER_STATUS, QUE_ACTIVE | QUE_LOCAL_ONLY, USR_ST },
    // Nodebit Configuration for: UPM_NB_UPS_ON_COMMAND                  // 47
    { PASS_1, PASS_1, UPS_ON_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // Nodebit Configuration for: UPM_NB_UPS_OFF_COMMAND                 // 48
    { PASS_1, PASS_1, LOAD_OFF_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // Nodebit Configuration for: UPM_NB_LOW_BATTERY_SHUTDOWN            // 49
    { PASS_1, PASS_1, LOW_BATTERY_SHUTDOWN, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_SOFTWARE_INCOMPATIBILITY_DETECTED  // 50
    { PASS_1, PASS_1, SOFTWARE_INCOMPATIBILITY_DETECTED, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_BATTERY_TEST_FAILED             // 51
    { PASS_1, PASS_1, BATTERY_TEST_FAILED, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_FUSE_FAILURE                    // 52
    { PASS_32, PASS_1, FUSE_FAILURE, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_FAN_FAILURE                     // 53
    { PASS_22, PASS_22, FAN_FAILURE, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_SITE_WIRING_FAULT               // 54
    { PASS_5, PASS_36000, SITE_WIRING_FAULT, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_BACKFEED_CONTACTOR_FAILURE      // 55
    { PASS_64, PASS_64, BACKFEED_CONTACTOR_FAILURE, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_BATTERIES_DISCONNECTED          // 56
    { PASS_1, PASS_1, BATTERIES_DISCONNECTED, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_AMBIENT_OVERTEMPERATURE         // 57
    { PASS_16, PASS_16, AMBIENT_OVERTEMPERATURE, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_SELECTIVE_TRIP_OF_MODULE        // 58
    { PASS_4, PASS_1, SELECTIVE_TRIP_OF_MODULE, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP  // 59
    { PASS_8, PASS_8, ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_RECTIFIER_OVERTEMPERATURE       // 60
    { PASS_16, PASS_64, RECTIFIER_OVERTEMPERATURE, QUE_ALL, NOTC1 },    
    // Nodebit Configuration for: UPM_NB_CONFIGURATION_ERROR             // 61
    { PASS_1, PASS_1, CONFIGURATION_ERROR, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_REDUNDANCY_LOSS_DUE_TO_OVERLOAD // 62
    { PASS_8, PASS_8, REDUNDANCY_LOSS_DUE_TO_OVERLOAD, QUE_ALL, NOTC1 },    
    // Nodebit Configuration for: UPM_NB_IN_HIGH_EFFICIENCY_MODE         // 63
    { PASS_1, PASS_1, IN_HIGH_EFFICIENCY_MODE, QUE_ALL, USR_ST },    
    // Nodebit Configuration for: UPM_NB_UPS_ON_NORMAL                   // 64
    { PASS_1, PASS_1, UPS_ON_NORMAL, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_INVALID_BOARD_ID                // 65
    { PASS_1, PASS_1, INVALID_BOARD_ID, QUE_ALL, USR_AL },    
    // Nodebit Configuration for: UPM_NB_CHECK_PRECHARGE                 // 66
    { PASS_1, PASS_1, PRECHARGE_FAILED, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_ON                    // 67
    { PASS_1, PASS_1, RECTIFIER_STATUS, QUE_ALL, USR_ST },    
    // Nodebit Configuration for: UPM_NB_UPS_ON_GENERATOR                // 68
    { PASS_1, PASS_1, UPS_ON_GENERATOR, QUE_ALL, NOTC1 },    

    // Nodebit Configuration for: UPM_NB_BATTERY_TEST_IN_PROGRESS        // 69
    { PASS_1, PASS_1, BATTERY_TEST_IN_PROGRESS, QUE_ALL, USR_ST },    

    // Nodebit Configuration for: UPM_NB_CLOCK_SET_DONE                  // 70
    { PASS_1, PASS_1, CLOCK_SET_DONE, QUE_ACTIVE, USR_ST },
    // Nodebit Configuration for: UPM_NB_IN_APM_MODE                     // 71
    { PASS_1, PASS_1, IN_APM_MODE, QUE_NONE, USR_ST }, 
    // Nodebit Configuration for: UPM_NB_ABM_CHARGE_MODE                 // 72
    { PASS_1, PASS_1, ABM_CHARGE_MODE, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_ABM_FLOAT_MODE                  // 73
    { PASS_1, PASS_1, ABM_FLOAT_MODE, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_ABM_REST_MODE                   // 74
    { PASS_1, PASS_1, ABM_REST_MODE, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_ABM_OFF                         // 75
    { PASS_1, PASS_1, ABM_STATE_OFF, QUE_ACTIVE, USR_ST },    
    // Nodebit Configuration for : UPM_NB_OUTPUT_HOT                     // 76
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },                                        
    // Nodebit Configuration for: UPM_NB_BYPASS_HOT                      // 77
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },    
    // Nodebit Configuration for: UPM_NB_BATTERY_DISCHARGING             // 78
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },
    // Nodebit Configuration for: UPM_NB_INVERTER_CONTACTOR_CLOSED       // 79
    { PASS_51, PASS_51, INVERTER_CONTACTOR_CLOSED, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_MIS_CLOSED                      // 80
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },
    // Nodebit Configuration for: UPM_NB_MIS_INSTALLED                   // 81
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST }, 
    // Nodebit Configuration for: UPM_NB_BYPASS_INSTALLED                // 82
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },
    // Nodebit Configuration for: UPM_NB_INTERNAL_MBS_INSTALLED          // 83
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },  
    // Nodebit Configuration for: UPM_NB_BATTERY_INSTALLED               // 84
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },   
    // Nodebit Configuration for: UPM_NB_PARALLEL_CAN_ERROR              // 85
    { PASS_1, PASS_1000, PARALLEL_BOARD_FAILURE, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_CHARGER_OFF_COMMAND             // 86
    { PASS_1, PASS_1, CHARGER_OFF_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // Nodebit Configuration for: UPM_NB_BATTERY_CONTACTOR_OPEN          // 87
    { PASS_1, PASS_1, BATTERY_CONTACTOR_OPEN, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_ECO_INSTALLED                   // 88
    { PASS_1, PASS_1, 0, QUE_NONE, USR_ST },
    // Nodebit Configuration for: UPM_NB_ECO_ENABLE,                     // 89
    { PASS_1, PASS_1, ECO_ENABLE, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_COMMON_BATTERY,                 // 90
    { PASS_1, PASS_1, 0, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_TOO_MANY_INVERTER_TRANSFERS     // 91
    { PASS_1, PASS_1, TOO_MANY_INVERTER_TRANSFERS, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_OUTPUT_PHASE_ROTATION           // 92
    { PASS_1, PASS_1, OUTPUT_PHASE_ROTATION, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_STATIC_SWITCH_SHORT             // 93
    { PASS_16, PASS_4, STATIC_SWITCH_SHORT, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_PULL_CHAIN                      // 94
    { PASS_8, PASS_4, PULL_CHAIN, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_CHECK_PULL_CHAIN                // 95
    { PASS_2551, PASS_1, CHECK_PULL_CHAIN, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_NORMAL_MODE_COMMAND             // 96
    { PASS_1, PASS_1, NORMAL_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_FAILED                // 97
    { PASS_1, PASS_1, RECTIFIER_FAILED, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_INVERTER_CONTACTOR_FAILURE      // 98
    { PASS_15306, PASS_1, CHECK_INVERTER_SWITCHGEAR, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_INVERTER_OVERTEMPERATURE_TRIP   // 99
    { PASS_2, PASS_6000, INVERTER_OVER_TEMPERATURE, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP  // 100
    { PASS_2, PASS_6000, RECTIFIER_OVERTEMPERATURE, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_AUTOMATIC_STARTUP_PENDING       // 101
    { PASS_1, PASS_1, AUTOMATIC_STARTUP_PENDING, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_CHARGER_ON_COMMAND              // 102
    { PASS_1, PASS_1, CHARGER_ON_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // UPM_NB_CHARGER_FAILURE                                            // 103
    { PASS_1, PASS_1, CHARGER_FAILURE, QUE_ACTIVE, USR_AL },
    // UPM_NB_BATTERY_CONTACTOR_FAIL                                     // 104
    { PASS_10, PASS_10, BATTERY_CONTACTOR_FAIL, QUE_ACTIVE, USR_AL },
    // Nodebit Configuration for: UPM_NB_OUTPUT_OVERLOAD_TRIP            // 105
    { PASS_1, PASS_32, OUTPUT_OVERLOAD_TRIP, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_POWER_SUPPLY_5_VOLT_FAULT       // 106
    { PASS_250, PASS_10, POWER_SUPPLY_5_VOLT_FAULT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_POWER_SUPPLY_15_VOLT_FAULT      // 107
    { PASS_250, PASS_10, POWER_SUPPLY_15_VOLT_FAULT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_INPUT_SYNC_OUT_OF_RANGE         // 108
    { PASS_3, PASS_20, INPUT_SYNC_OUT_OF_RANGE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_INVERTER_STARTUP_FAILURE        // 109
    { PASS_1, PASS_1, INVERTER_STARTUP_FAILURE, QUE_ACTIVE, USR_AL },
    // Nodebit Configuration for: UPM_NB_IN_EASY_CAPACITY_TEST_MODE      // 110
    { PASS_1, PASS_1, IN_EASY_CAPACITY_TEST_MODE, QUE_ACTIVE, USR_ST },
    // Nodebit Configuration for: UPM_NB_I2C_FAIL                        // 111
    { PASS_1, PASS_5, I2C_BUS_FAIL, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_SWITCHGEAR_OPEN       // 112
    { PASS_1, PASS_1, RECTIFIER_SWITCHGEAR_OPEN, QUE_ALL, USR_ST },
    // Nodebit Configuration for: UPM_NB_ECT_MODE_COMMAND                // 113
    { PASS_1, PASS_1, ECT_COMMAND, QUE_ACTIVE | QUE_LOCAL_ONLY, COMD },
    // Nodebit Configuration for: UPM_NB_ABNORMAL_EXIT_ECT_MODE          // 114
    { PASS_1, PASS_1, ABNORMAL_EXIT_ECT_MODE, QUE_ACTIVE, USR_AL },
    // Nodebit Configuration for : UPM_NB_PARALLEL_SETUP_FAIL            // 115
    { PASS_1, PASS_5, PARALLEL_SETUP_FAIL, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_SYSTEM_TEST_IN_PROGRESS         // 116
    { PASS_1, PASS_1, SYSTEM_TEST_IN_PROGRESS, QUE_ALL, USR_ST },
        
    // Directions: To add a single new nodebit configuration, delete the next line
    // and insert the new configuration in its place.  The starting addrss for
    // private nodebits should not change.
    // Nodebit Configuration for : UPM_NB_LOSS_OF_SYNC_BUS               // 117
    { PASS_1, PASS_16, LOSS_OF_SYNC_BUS, QUE_ALL, USR_AL },
    // Nodebit Configuration for : UPM_NB_MBS_CLOSED                     // 118
    { PASS_1, PASS_1,  MBS_CLOSED,       QUE_ALL, NOTC1  },
    // Nodebit Configuration for : UPM_NB_MOB_OPEN                       // 119
    { PASS_1, PASS_1,  MOB_OPEN,         QUE_ALL, NOTC1  },
    // Nodebit Configuration for : UPM_NB_MOB_FAILURE                    // 120
    { PASS_1, PASS_1,  MOB_FAILURE,      QUE_ALL, NOTC1  },
    // Nodebit Configuration for : UPM_NB_EXTERNAL_PARALLEL              // 121
    { PASS_2, PASS_1,  EXTERNAL_PARALLEL,QUE_ALL, USR_ST },
    // Nodebit Configuration for : UPM_NB_SYSTEM_IS_REDUNDANT            // 122
    { PASS_1, PASS_1, SYSTEM_IS_REDUNDANT, QUE_NONE, USR_ST},
    // Nodebit Configuration for : UPM_NB_DRIVER_FAULT                   // 123
    { PASS_250, PASS_10, DRIVER_FAULT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_PM_OVERTEMPERATURE              // 124
    { PASS_16, PASS_64, PM_OVER_TEMPERATURE, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_PM_OVERTEMPERATURE_TRIP         // 125
    { PASS_2, PASS_6000, PM_OVER_TEMPERATURE_TRIP, QUE_ALL, SVC_AL },
    // Nodebit Configuration for: UPM_NB_BALANCER_LOOP_OVER_CURRENT      // 126
    { PASS_1, PASS_1, BALANCER_LOOP_OVER_CURRENT, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_BATTERY_BREAKER_FAILURE         // 127
    { PASS_1, PASS_1, BATTERY_BREAKER_FAILURE, QUE_ALL, USR_AL },
    // Nodebit Configuration for: UPM_NB_TOO_MANY_BATTERY_TRANSFERS      // 128
    { PASS_1, PASS_1, TOO_MANY_BATTERY_TRANSFERS, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_SPI_BUS_FAILURE	             // 129
    { PASS_1, PASS_5, SPI_BUS_FAILURE, QUE_ALL, USR_ST},
    // Nodebit Configuration for: UPM_NB_PARALLEL_LOAD_SHARE_ERROR       // 130
    { PASS_32, PASS_10, PARALLEL_LOAD_SHARE_ERROR, QUE_ALL, USR_AL},
    // Nodebit Configuration for: UPM_NB_HE_NOT_AVAILABLE                // 131
    { PASS_1, PASS_2, HE_NOT_AVAILABLE, QUE_ALL, NOTC1 },                   
    // Nodebit Configuration for: UPM_NB_INPUT_VOLTAGE_ABNORMAL          // 132
    { PASS_2, PASS_36000, INPUT_ABNORMAL, QUE_ALL, NOTC1 },
   	// Nodebit Configuration for: UPM_NB_BATTERY_STARTUP_FAILURE         // 133
	{ PASS_1, PASS_1, BATTERY_STARTUP_FAILURE, QUE_ALL, SVC_AL},		  
     // Nodebit Configuration for: UPM_NB_PLD_CONFIG_FAIL        		 // 134
    { PASS_1, PASS_1, PLD_CONFIG_FAIL, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },
    // Nodebit Configuration for: UPM_NB_DISCHARGE_DC_LINK               // 135
    { PASS_1, PASS_1, DISCHARGE_DCLINK, QUE_ALL, NOTC1 },         
    {0, },                                                               // 136 Reserved for 3_Level
    {0, },                                                               // 137 Reserved for 3_Level
    {0, },                                                               // 138 Reserved for 3_Level
    {0, },                                                               // 139 Reserved for 3_Level
    {0, },                                                               // 140 Reserved for 3_Level
    {0, },                                                               // 141 Reserved for 3_Level 400-500
    {0, },                                                               // 142 Reserved for 3_Level 400-500
    {0, },                                                               // 143 Reserved for 3_Level 400-500
    // Nodebit Configuration for: UPM_NB_REC_PWM_OFF                     // 144
    { PASS_1, PASS_1, REC_PWM_OFF, QUE_ALL | QUE_LOCAL_ONLY, NOTC1 },
    // Nodebit Configuration for: UPM_NB_ADAPTIVE_OVERLOAD_CAPACITY_OFF  // 145
    { PASS_1, PASS_1, ADAPTIVE_OVERLOAD_CAPACITY_OFF, QUE_ALL, NOTC1 },
    // Nodebit Configuration for: UPM_NB_STS_OVER_TEMPERATURE            // 146
    { PASS_16, PASS_64, STS_OVER_TEMPERATURE, QUE_ALL, NOTC1  },
    // Nodebit Configuration for: UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R     // 147 for jira hobbit-113, nobbit only local
    { PASS_1, PASS_1, RECTIFIER_SWITCHGEAR_OPEN_R, QUE_ALL|QUE_LOCAL_ONLY, USR_ST  },
    // Nodebit Configuration for: UPM_NB_BAT_LEGB_RELAY_OPEN             // 148 for jira hobbit-113,nobbit same as bat relay	
    { PASS_1, PASS_1, BAT_LEGB_RELAY_OPEN, QUE_ALL|QUE_LOCAL_ONLY, USR_ST  },
    // Nodebit Configuration for: UPM_NB_LOSS_OF_PWM_SYNC                // 149
    { PASS_10, PASS_16, LOSS_OF_PWM_SYNC, QUE_ALL, USR_ST  },
    { PASS_1,  PASS_1,  BACKFEED_RELAY_CLOSE, QUE_NONE, USR_ST  },       // 150
    {0, },  // 151
    {0, },  // 152
    {0, },  // 153
    {0, },  // 154
    {0, },  // 155
    {0, },  // 156
    {0, },  // 157
    {0, },  // 158
    {0, },  // 159
    {0, },  // 160
    {0, },  // 161
    {0, },  // 162
    {0, },  // 163
    {0, },  // 164
    {0, },  // 165
    {0, },  // 166
    {0, },  // 167
    {0, },  // 168
    {0, },  // 169
    {0, },  // 170
    {0, },  // 171
    {0, },  // 172
    {0, },  // 173
    {0, },  // 174
    {0, },  // 175
    {0, },  // 176
    {0, },  // 177
    {0, },  // 178
    {0, },  // 179
    {0, },  // 180
    {0, },  // 181
    {0, },  // 182
    {0, },  // 183
    {0, },  // 184
    {0, },  // 185
    {0, },  // 186
    {0, },  // 187
    {0, },  // 188
    {0, },  // 189
    {0, },  // 190
    {0, },  // 191
    
    ///////////////  Private nodebits begin here.
    { PASS_1, PASS_1, MCU_STATE_CHANGE, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },
    { PASS_1, PASS_1, BYPASS_STATE_CHANGE, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },
    { PASS_1, PASS_1, RECTIFIER_STATE_CHANGE, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },
    { PASS_1, PASS_1, BATTERY_STATE_CHANGE, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },
    { PASS_1, PASS_1, SYNC_STATE_CHANGE, QUE_ACTIVE | QUE_LOCAL_ONLY, NOTC1 },

};  // end of NB_Cfg_Flash definition


const uint16_t NumberOfDefinedNodebits = sizeof( NB_Cfg_Flash ) / sizeof( st_NB_Cfg );

uint16_t GetNumDefinedNB()
{
    return NumberOfDefinedNodebits;
}

// ***********************************************************************
// *
// *    FUNCTION: GetNodeBitArrayIndex 
// *
// *    DESCRIPTION: Function to search the NB config table by XCP alarm number 
// *
// *    ARGUMENTS: XCP alarm number
// *
// *    RETURNS: Array index, returns NumberOfDefinedNodebits is alarm is not found 
// *
// ***********************************************************************
uint16_t GetNodeBitArrayIndex( uint16_t nbNum )
{
    uint16_t idx;
    
    for ( idx = 0; idx < NumberOfDefinedNodebits; idx++ )
    {
        if ( NB_Cfg_Flash[ idx ].bit.XCPAlarmNumber == nbNum )
        {
            break;
        }
    } 
    
    return idx;           
}
