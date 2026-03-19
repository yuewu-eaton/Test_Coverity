#ifndef PANDA_UPM_NB_IDS_H
#define PANDA_UPM_NB_IDS_H

// ********************************************************************************************************
// *            NB_Config.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: UPM_NB_Ids.h
// *
// *    DESCRIPTION: Defines the Nodebits ID numbers that can be debounced and queued. Each of these
// *        Nodebits is fully controlled by the UPM.
// *        WARNING FOR PANDA: Changes in this file must be exactly synchronized with the Panda CSB
// *        project.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/16/2003
// *
// *    HISTORY: See Subversion history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        DEFINES
// *********************************************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

// This table defines the supported events. Difference from 2812 projects
// Call DebounceAndQueu with these numbers, NOT the XCP alarm number!!!!
typedef enum _upm_nb_id_t
{
    UPM_NB_EVENT_BASE,
    // 0 - 9
    UPM_NB_INVERTER_AC_UNDER_VOLTAGE = UPM_NB_EVENT_BASE,   // 0
    UPM_NB_BYPASS_AC_OVER_VOLTAGE,                          // 1
    UPM_NB_BYPASS_AC_UNDER_VOLTAGE,                         // 2
    UPM_NB_BYPASS_UNDER_OVER_FREQUENCY,                     // 3
    UPM_NB_INPUT_AC_OVER_VOLTAGE,                           // 4
    UPM_NB_INPUT_AC_UNDER_VOLTAGE,                          // 5
    UPM_NB_INPUT_UNDER_OVER_FREQUENCY,                      // 6
    UPM_NB_OUTPUT_AC_OVER_VOLTAGE,                          // 7
    UPM_NB_OUTPUT_AC_UNDER_VOLTAGE,                         // 8
    UPM_NB_OUTPUT_UNDER_OVER_FREQUENCY,                     // 9
    // 10 - 19
    UPM_NB_REMOTE_EMERGENCY_POWER_OFF,                      // 10
    UPM_NB_INVERTER_OVERTEMPERATURE,                        // 11
    UPM_NB_OUTPUT_OVERLOAD,                                 // 12
    UPM_NB_RECTIFIER_INPUT_OVER_CURRENT,                    // 13
    UPM_NB_INVERTER_OUTPUT_OVER_CURRENT,                    // 14
    UPM_NB_DC_LINK_OVER_VOLTAGE,                            // 15
    UPM_NB_DC_LINK_UNDER_VOLTAGE,                           // 16
    UPM_NB_STATIC_SWITCH_FAILURE,                           // 17
    UPM_NB_BATTERY_CURRENT_LIMIT,                           // 18
    UPM_NB_NON_VOLATILE_RAM_FAILURE,                        // 19
    // 20 - 29
    UPM_NB_SHUTDOWN_IMMINENT,                               // 20
    UPM_NB_BATTERY_LOW,                                     // 21
    UPM_NB_OUTPUT_SHORT_CIRCUIT,                            // 22
    UPM_NB_UTILITY_NOT_PRESENT,                             // 23
    UPM_NB_BATTERY_DC_OVER_VOLTAGE,                         // 24
    UPM_NB_POWER_SUPPLY_FAILURE,                            // 25
    UPM_NB_TO_BYPASS_COMMAND,                               // 26
    UPM_NB_BYPASS_NOT_AVAILABLE,                            // 27
    UPM_NB_RECTIFIER_PHASE_ROTATION,                        // 28
    UPM_NB_BYPASS_PHASE_ROTATION,                           // 29
    // 30 - 39
    UPM_NB_CONTROL_POWER_ON,                                // 30
    UPM_NB_INVERTER_ON,                                     // 31
    UPM_NB_CHARGER_ON,                                      // 32
    UPM_NB_INTERNAL_MBS_ACTIVE,                             // 33
    UPM_NB_BATTERY_NEEDS_SERVICE,                           // 34
    UPM_NB_LEVEL_2_OVERLOAD_PHASE_A,                        // 35
    UPM_NB_LEVEL_2_OVERLOAD_PHASE_B,                        // 36
    UPM_NB_LEVEL_2_OVERLOAD_PHASE_C,                        // 37
    UPM_NB_LEVEL_3_OVERLOAD_PHASE_A,                        // 38
    UPM_NB_LEVEL_3_OVERLOAD_PHASE_B,                        // 39
    // 40 - 49
    UPM_NB_LEVEL_3_OVERLOAD_PHASE_C,                        // 40
    UPM_NB_LEVEL_4_OVERLOAD_PHASE_A,                        // 41
    UPM_NB_LEVEL_4_OVERLOAD_PHASE_B,                        // 42
    UPM_NB_LEVEL_4_OVERLOAD_PHASE_C,                        // 43
    UPM_NB_UPS_ON_BATTERY,                                  // 44 
    UPM_NB_UPS_ON_BYPASS,                                   // 45
    UPM_NB_LOAD_POWER_OFF,                                  // 46
    UPM_NB_UPS_ON_COMMAND,                                  // 47
    UPM_NB_UPS_OFF_COMMAND,                                 // 48
    UPM_NB_LOW_BATTERY_SHUTDOWN,                            // 49
    // 50 - 59
    UPM_NB_SOFTWARE_INCOMPATIBILITY_DETECTED,               // 50
    UPM_NB_BATTERY_TEST_FAILED,                             // 51
    UPM_NB_FUSE_FAILURE,                                    // 52
    UPM_NB_FAN_FAILURE,                                     // 53
    UPM_NB_SITE_WIRING_FAULT,                               // 54
    UPM_NB_BACKFEED_CONTACTOR_FAILURE,                      // 55
    UPM_NB_BATTERIES_DISCONNECTED,                          // 56
    UPM_NB_AMBIENT_OVERTEMPERATURE,                         // 57
    UPM_NB_SELECTIVE_TRIP_OF_MODULE,                        // 58
    UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP,              // 59
    // 60 - 69
    UPM_NB_RECTIFIER_OVERTEMPERATURE,                       // 60
    UPM_NB_CONFIGURATION_ERROR,                             // 61
    UPM_NB_REDUNDANCY_LOSS_DUE_TO_OVERLOAD,                 // 62
    UPM_NB_IN_HIGH_EFFICIENCY_MODE,                         // 63
    UPM_NB_UPS_ON_NORMAL,                                   // 64
    UPM_NB_INVALID_BOARD_ID,                                // 65
    UPM_NB_CHECK_PRECHARGE,                                 // 66
    UPM_NB_RECTIFIER_ON,                                    // 67
    UPM_NB_UPS_ON_GENERATOR,                                // 68
    UPM_NB_BATTERY_TEST_IN_PROGRESS,                        // 69
    // 70 - 79
    UPM_NB_CLOCK_SET_DONE,                                  // 70
    UPM_NB_IN_APM_MODE,                                     // 71
    UPM_NB_ABM_CHARGE_MODE,                                 // 72
    UPM_NB_ABM_FLOAT_MODE,                                  // 73
    UPM_NB_ABM_REST_MODE,                                   // 74
    UPM_NB_ABM_OFF,                                         // 75
    // The following additional nodebits are required per the CSB FRS
    // See Panda Essential CSB FRS, section 4.11.4.1  
    UPM_NB_OUTPUT_HOT,                                      // 76
    UPM_NB_BYPASS_HOT,                                      // 77
    UPM_NB_BATTERY_DISCHARGING,                             // 78
    UPM_NB_INVERTER_CONTACTOR_CLOSED,                       // 79
    // 80 - 89
    UPM_NB_MIS_CLOSED,                                      // 80
    UPM_NB_MIS_INSTALLED,                                   // 81
    UPM_NB_BYPASS_INSTALLED,                                // 82
    UPM_NB_INTERNAL_MBS_INSTALLED,                          // 83
    UPM_NB_BATTERY_INSTALLED,                               // 84
    UPM_NB_PARALLEL_CAN_ERROR,                              // 85
    UPM_NB_CHARGER_OFF_COMMAND,                             // 86
    UPM_NB_BATTERY_CONTACTOR_OPEN,                          // 87
    // ECO mode installed.
    UPM_NB_ECO_INSTALLED,                                   // 88
    // ECO mode enabled.
    UPM_NB_ECO_ENABLE,                                      // 89
    // 90 - 99
    // Common battery
    UPM_NB_COMMON_BATTERY,                                  // 90
    UPM_NB_TOO_MANY_INVERTER_TRANSFERS,                     // 91
    UPM_NB_OUTPUT_PHASE_ROTATION,                           // 92
    UPM_NB_STATIC_SWITCH_SHORT,                             // 93
    UPM_NB_PULL_CHAIN,                                      // 94
    UPM_NB_CHECK_PULL_CHAIN,                                // 95
    UPM_NB_NORMAL_MODE_COMMAND,                             // 96
    // Using this for now to indicate that rectifier failed 3 startup attempts
    UPM_NB_RECTIFIER_FAILED,                                // 97
    UPM_NB_INVERTER_CONTACTOR_FAILURE,                      // 98
    UPM_NB_INVERTER_OVERTEMPERATURE_TRIP,                   // 99
    // 100 - 109
    UPM_NB_RECTIFIER_OVERTEMPERATURE_TRIP,                  // 100
    UPM_NB_AUTOMATIC_STARTUP_PENDING,                       // 101
    UPM_NB_CHARGER_ON_COMMAND,                              // 102
    UPM_NB_CHARGER_FAILURE,                                 // 103
    UPM_NB_BATTERY_CONTACTOR_FAIL,                          // 104
    UPM_NB_OUTPUT_OVERLOAD_TRIP,                            // 105
    UPM_NB_POWER_SUPPLY_5_VOLT_FAULT,                       // 106
    UPM_NB_POWER_SUPPLY_15_VOLT_FAULT,                      // 107
    UPM_NB_INPUT_SYNC_OUT_OF_RANGE,                         // 108
    UPM_NB_INVERTER_STARTUP_FAILURE,                        // 109
    // 110 - 119
    UPM_NB_IN_EASY_CAPACITY_TEST_MODE,                      // 110
    UPM_NB_I2C_FAIL,                                        // 111
    UPM_NB_RECTIFIER_SWITCHGEAR_OPEN,                       // 112
    UPM_NB_ECT_MODE_COMMAND,                                // 113
    UPM_NB_ABNORMAL_EXIT_ECT_MODE,                          // 114
    UPM_NB_PARALLEL_SETUP_FAIL,                             // 115
    UPM_NB_SYSTEM_TEST_IN_PROGRESS,                         // 116
    UPM_NB_LOSS_OF_SYNC_BUS,                                // 117
    UPM_NB_MBS_CLOSED,                                      // 118
    UPM_NB_MOB_OPEN,                                        // 119
    // 120 - 129
    UPM_NB_MOB_FAILURE,                                     // 120
    UPM_NB_EXTERNAL_PARALLEL,                               // 121
    UPM_NB_SYSTEM_IS_REDUNDANT,                             // 122
    UPM_NB_DRIVER_FAULT,                                    // 123
    UPM_NB_PM_OVERTEMPERATURE,                              // 124
    UPM_NB_PM_OVERTEMPERATURE_TRIP,                         // 125
    UPM_NB_BALANCER_LOOP_OVER_CURRENT,                      // 126
    UPM_NB_BATTERY_BREAKER_FAILURE,                         // 127
    UPM_NB_TOO_MANY_BATTERY_TRANSFERS,                      // 128
    UPM_NB_SPI_BUS_FAILURE,                                 // 129
    // 130 - 139
    UPM_NB_PARALLEL_LOAD_SHARE_ERROR,                       // 130
    UPM_NB_HE_NOT_AVAILABLE,                                // 131
    UPM_NB_INPUT_VOLTAGE_ABNORMAL,                          // 132
    UPM_NB_BATTERY_STARTUP_FAILURE,                         // 133
    UPM_NB_PLD_CONFIG_FAIL,                                 // 134
    UPM_NB_DISCHARGE_DC_LINK,                               // 135
    UPM_NB_CLOSE_BATTERY_BREAKER,                           // 136 Reserved for 3_Level 100-500
    UPM_NB_BATTERY_BREAKFB_OPEN,                            // 137 Reserved for 3_Level 100-500
    UPM_NB_CAP_OVERTEMPERATURE,                             // 138 Reserved for 3_Level 100-500
    UPM_NB_BATTERY_BREAKER_TRIP,                            // 139 Reserved for 3_Level 100-500
    // 140 - 149
    UPM_NB_ESS_INVERTER_GAPPING,                            // 140 Reserved for 3_Level 100-500
    UPM_NB_DC_LINK_FUSE_FAILURE,                            // 141 Reserved for 3_Level 400-500
    UPM_NB_LOAD_SYNC_NOT_AVAILABLE,                         // 142 Reserved for 3_Level 400-500
    UPM_NB_LOAD_SYNC_NOT_MATCH_BYPASS,                      // 143 Reserved for 3_Level 400-500
    UPM_NB_REC_PWM_OFF,                                     // 144
    UPM_NB_ADAPTIVE_OVERLOAD_CAPACITY_OFF,                  // 145
    UPM_NB_STS_OVER_TEMPERATURE,                            // 146
    // New for jira hobbit-113 bat low back explode
    UPM_NB_RECTIFIER_SWITCHGEAR_OPEN_R,                     // 147
    UPM_NB_BAT_LEGB_RELAY_OPEN,                             // 148
    UPM_NB_LOSS_OF_PWM_SYNC,                                // 149
    // 150-152
    UPM_NB_BACKFEED_RELAY_CLOSE,                            // 150

    // New nodebits should be inserted above this line.
    UPM_NB_NUMBER_OF_SUPPORTED_EVENTS,

    // THE FOLLOWING NODEBITS ARE ALTERNATE NAMES FOR ALARMS DEFINED
    // ABOVE. They are provided for the convenience of the CSB FRS only
    UPM_NB_CONTROL_POWER_OFF = UPM_NB_POWER_SUPPLY_FAILURE,
    // Do not allow UPM_NB_NUMBER_OF_SUPPORTED_EVENTS to exceed this value
    //UPM_NB_MAXIMUM_EVENT_IDX = 127
    UPM_NB_MAXIMUM_EVENT_IDX = 191
} upm_nb_id_t;

#ifdef __cplusplus
}
#endif

#endif
