// ********************************************************************************************************
// *            XCPDefs.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton Corporation
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: XCPDefs.h
// *
// *    DESCRIPTION: Nodebit# Defines. Also includes all other header files required to do any nodebit 
// *                processing.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/21/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************
#ifndef _XCPDEFS_H
#define _XCPDEFS_H


// ********************************************************************************************************
// *        DEFINES - XCP alarm numbers
// ********************************************************************************************************
#define INVERTER_AC_OVER_VOLTAGE                        0  //
#define INVERTER_AC_UNDER_VOLTAGE                       1  //
#define INVERTER_UNDER_OVER_FREQUENCY                   2  //
#define BYPASS_AC_OVER_VOLTAGE                          3  //
#define BYPASS_AC_UNDER_VOLTAGE                         4  //
#define BYPASS_UNDER_OVER_FREQUENCY                     5  //
#define INPUT_AC_OVER_VOLTAGE                           6  //
#define INPUT_AC_UNDER_VOLTAGE                          7  //
#define INPUT_UNDER_OVER_FREQUENCY                      8  //
#define OUTPUT_AC_OVER_VOLTAGE                          9  //
#define OUTPUT_AC_UNDER_VOLTAGE                         10  //
#define OUTPUT_UNDER_OVER_FREQUENCY                     11  //
#define REMOTE_EMERGENCY_POWER_OFF                      12  //
#define REMOTE_GO_TO_BYPASS                             13  //
#define BUILDING_ALARM_6                                14  //
#define BUILDING_ALARM_5                                15  //
#define BUILDING_ALARM_4                                16  //
#define BUILDING_ALARM_3                                17  //
#define BUILDING_ALARM_2                                18  //
#define BUILDING_ALARM_1                                19  //
#define STATIC_SWITCH_OVER_TEMPERATURE                  20  //  
#define INVERTER_BATT_OT_SHUTDOWN_LVL                   21  //
#define CHARGER_TRIPPED                                 22  //  
#define CHARGER_OVER_VOLTAGE_OR_CURRENT                 23  //
#define INVERTER_OVER_TEMPERATURE                       24  //
#define OUTPUT_OVERLOAD                                 25  //
#define RECTIFIER_INPUT_OVER_CURRENT                    26  //
#define INVERTER_OUTPUT_OVER_CURRENT                    27  //  
#define DC_LINK_OVER_VOLTAGE                            28  //
#define DC_LINK_UNDER_VOLTAGE                           29  //
#define RECTIFIER_FAILED                                30  //
#define CHECK_INVERTER                                  31  //
#define BATTERY_CONTACTOR_FAIL                          32  //
#define BYPASS_BREAKER_FAIL                             33  //
#define CHARGER_FAILURE                                 34  //  
#define RAMP_UP_FAILED                                  35  //  
#define STATIC_SWITCH_FAILURE                           36  //
#define ANALOG_BOARD_A_D_REFERENCE_FAIL                 37  //  
#define BYPASS_UNCALIBRATED                             38  //  
#define RECTIFIER_UNCALIBRATED                          39  //  
#define OUTPUT_UNCALIBRATED                             40  //  
#define INVERTER_UNCALIBRATED                           41  //  
#define DC_VOLTAGE_UNCALIBRATED                         42  //  
#define OUTPUT_CURRENT_UNCALIBRATED                     43  //  
#define RECTIFIER_CURRENT_UNCALIBRATED                  44  //  
#define BATTERY_CURRENT_UNCALIBRATED                    45  //  
#define INVERTER_ON_OFF_STAT_FAILURE                    46  //  
#define BATTERY_CURRENT_LIMIT                           47  //
#define INVERTER_STARTUP_FAILURE                        48  //  
#define ANALOG_BOARD_AD_STAT_FAILURE                    49  //  
#define CB4_FAILURE                                     50  //
#define BATTERY_GROUND_FAULT                            51  //  
#define WAITING_FOR_CHARGER_SYNC                        52  //  
#define EEPROM_FAULT                                    53  //
#define ANALOG_BOARD_AD_TIME_OUT                        54  //  
#define SHUTDOWN_IMMINENT                               55  //
#define BATTERY_LOW                                     56  //
#define UTILITY_OUT_OF_LIMITS                           57  //
#define OUTPUT_SHORT_CIRCUIT                            58  //  
#define UTILITY_NOT_PRESENT                             59  //  
#define FULL_TIME_CHARGING                              60  //  
#define FAST_BYPASS_COMMAND                             61  //  
#define AD_ERROR                                        62  //  
#define INTERNAL_COMMUNICATION_FAILURE                  63  //
#define RECTIFIER_FAILED_SELF_TEST                      64  //  
#define RECTIFIER_EEPROM_FAILURE                        65  //  
#define RECTIFIER_EPROM_FAILURE                         66  //  
#define INPUT_LINE_VOLTAGE_LOSS                         67  //  
#define BATTERY_DC_OVER_VOLTAGE                         68  //
#define POWER_SUPPLY_OVER_TEMPERATURE                   69  //  
#define POWER_SUPPLY_FAILURE                            70  //
#define POWER_SUPPLY_5_VOLT_FAULT                       71  //
#define RECT_5VOLT_LOSS                                 71  //
#define INV_5VOLT_LOSS                                  71  //
#define POWER_SUPPLY_15_VOLT_FAULT                      72  //
#define HEATSINK_OVER_TEMPERATURE                       73  //
#define HEATSINK_TEMPERATURE_SENSOR_FAILED              74  //
#define RECTIFIER_CURRENT_OVER_125PERCENT               75  //  
#define RECTIFIER_TRIPPED                               76  //
#define RECTIFIER_POWER_CAPACITOR_FAULT                 77  //  
#define INVERTER_PROGRAM_STACK_ERROR                    78  //  
#define INVERTER_CONTROL_BOARD_FAILED_SELF_TEST         79  //  
#define INVERTER_AD_CONVERTER_SELF_TEST_FAILED          80  //  
#define INVERTER_RAM_SELF_TEST_FAILURE                  81  //  
#define NONVOLATILE_MEMORY_CHECKSUM_FAILURE             82  //  
#define PROGRAM_CHECKSUM_FAILURE                        83  //  
#define INVERTER_CPU_SELF_TEST_FAILED                   84  //  
#define NETWORK_NOT_RESPONDING                          85  //  
#define FRONT_PANEL_SELF_TEST_FAILURE                   86  //  
#define NODE_EEPROM_VERIFICATION_ERROR                  87  //  
#define OUTPUT_AC_OVER_VOLTAGE_TEST_FAILED              88  //  
#define OUTPUT_DC_OVER_VOLTAGE                          89  //  
#define INPUT_PHASE_ROTATION_ERROR                      90  //  
#define INVERTER_RAMP_UP_TEST_FAILED                    91  //  
#define INVERTER_OFF_COMMAND                            92  //  
#define INVERTER_ON_COMMAND                             93  //  
#define TO_BYPASS_COMMAND                               94  //
#define FROM_BYPASS_COMMAND                             95  //  
#define NORMAL_COMMAND                                  96  //
#define EMERGENCY_SHUTDOWN_COMMAND                      97  //  
#define SETUP_SWITCH_OPEN                               98  //  
#define INVERTER_AC_OVER_VOLTAGE_INTERRUPT              99  //  
#define INVERTER_AC_UNDER_VOLTAGE_INTERRUPT             100  // 
#define ABSOLUTE_DCOV_ACOV_INTERRUPT                    101  // 
#define INVERTER_AMPS_LIMIT                             102  //
#define PHASE_B_CURRENT_LIMIT                           103  //
#define PHASE_C_CURRENT_LIMIT                           104  //
#define BYPASS_NOT_AVAILABLE                            105  //
#define RECTIFIER_SWITCHGEAR_OPEN                       106  // 
#define BATTERY_SWITCHGEAR_OPEN                         107  //
#define INVERTER_SWITCHGEAR_OPEN                        108  //
#define BYPASS_SWITCHGEAR_OPEN                          109  // 
#define INVERTER_BOARD_ACOV_INTERRUPT_TEST_FAILED       110  // 
#define INV_OT_SHUTDOWN                                 111  //
#define INVERTER_BOARD_ACUV_INTERRUPT_TEST_FAIL         112  // 
#define INVERTER_VOLTAGE_FEEDBACK_ERROR                 113  // 
#define DC_UNDER_VOLTAGE_TIMEOUT                        114  // 
#define AC_UNDER_VOLTAGE_TIMEOUT                        115  // 
#define DC_UNDER_VOLTAGE_WHILE_CHARGER_IS_FULL_ON       116  // 
#define INVERTER_VOLTAGE_BIAS_ERROR                     117  // 
#define RECTIFIER_PHASE_ROTATION                        118  //
#define BYPASS_PHASE_ROTATION                           119  //
#define SYSTEM_INTERFACE_BOARD_FAILURE                  120  // 
#define PARALLEL_BOARD_FAILURE                          121  // 
#define LOSS_OF_LOAD_SHARING_PHASE_A                    122  // 
#define LOSS_OF_LOAD_SHARING_PHASE_B                    123  // 
#define LOSS_OF_LOAD_SHARING_PHASE_C                    124  // 
#define DC_OVER_VOLTAGE_TIMEOUT                         125  // 
#define BATTERY_TOTALLY_DISCHARGED                      126  //
#define INVERTER_PHASE_BIAS_ERROR                       127  // 
#define INVERTER_VOLTAGE_BIAS_ERROR_Num2                128  //  Duplicate of #117 _ WHY???
#define DC_LINK_BLEED_COMPLETE                          129  // 
#define LARGE_CHARGER_INPUT_CURRENT                     130  // 
#define INVERTER_VOLTAGE_TOO_LOW_FOR_RAMP_LEVEL         131  // 
#define LOSS_OF_REDUNDANCY                              132  //
#define LOSS_OF_SYNC_BUS                                133  // 
#define RECTIFIER_BREAKER_SHUNT_TRIPPED                 134  // 
#define LOSS_OF_CHARGER_SYNC                            135  // 
#define INVERTER_LOW_LEVEL_TEST_TIMEOUT                 136  // 
#define OUTPUT_BREAKER_OPEN                             137  //
#define CONTROL_POWER_ON                                138  //
#define INVERTER_ON                                     139  //
#define CHARGER_ON                                      140  //
#define BYPASS_ON                                       141  // 
#define BYPASS_POWER_LOSS                               142  // 
#define INTERNAL_MBS_ACTIVE                             143  //
#define BYPASS_MANUAL_TURN_OFF                          144  // 
#define INVERTER_BLEEDING_DC_LINK_VOLTAGE               145  // 
#define CPU_ISR_ERROR                                   146  // 
#define SYSTEM_ISR_RESTART                              147  // 
#define PARALLEL_DC                                     148  // 
#define BATTERY_NEEDS_SERVICE                           149  //
#define BATTERY_CHARGING                                150  // 
#define BATTERY_NOT_CHARGED                             151  // 
#define DISABLED_BATTERY_TIME                           152  // 
#define SERIES_7000_ENABLE                              153  // 
#define OTHER_UPS_ON                                    154  // 
#define PARALLEL_INV                                    155  // 
#define UPS_IN_PARALLEL                                 156  // 
#define OUTPUT_BREAKER_FAIL                             157  // 
#define CONTROL_POWER_OFF                               158  // 
#define LEVEL_2_OVERLOAD_PHASE_A                        159  //
#define LEVEL_2_OVERLOAD_PHASE_B                        160  //
#define LEVEL_2_OVERLOAD_PHASE_C                        161  //
#define LEVEL_3_OVERLOAD_PHASE_A                        162  //
#define LEVEL_3_OVERLOAD_PHASE_B                        163  //
#define LEVEL_3_OVERLOAD_PHASE_C                        164  //
#define LEVEL_4_OVERLOAD_PHASE_A                        165  //
#define LEVEL_4_OVERLOAD_PHASE_B                        166  //
#define LEVEL_4_OVERLOAD_PHASE_C                        167  //
#define UPS_ON_BATTERY                                  168  //
#define UPS_ON_BYPASS                                   169  //
#define LOAD_POWER_STATUS                               170  //
#define LOAD_ON_INVERTER                                171  // 
#define UPS_ON_COMMAND                                  172  //
#define LOAD_OFF_COMMAND                                173  //
#define LOW_BATTERY_SHUTDOWN                            174  //
#define AUTO_ON_ENABLED                                 175  // 
#define SOFTWARE_INCOMPATIBILITY_DETECTED               176  //
#define INV_TEMP_SENSOR_FAIL                            177  //
#define DC_START_OCCURRED                               178  // 
#define IN_PARALLEL_OPERATION                           179  // 
#define SYNCING_TO_BYPASS                               180  // 
#define RAMPING_UPS_UP                                  181  // 
#define INVERTER_ON_DELAY                               182  // 
#define CHARGER_ON_DELAY                                183  // 
#define WAITING_FOR_UTIL_INPUT                          184  // 
#define CLOSE_BYPASS_BREAKER                            185  // 
#define EMERGENCY_TRANSFER_TO_BYPASS                    186  //
#define SYNCING_TO_OUTPUT                               187  // 
#define BYPASS_FAILURE                                  188  //
#define AUTO_OFF_COMMAND_EXECUTED                       189  //
#define AUTO_ON_COMMAND_EXECUTED                        190  //
#define BATTERY_TEST_FAILED                             191  //
#define FUSE_FAILURE                                    192  //
#define FAN_FAILURE                                     193  //
#define SITE_WIRING_FAULT                               194  //
#define BACKFEED_CONTACTOR_FAILURE                      195  // 
#define ON_BUCK_VOLTAGE_REDUCER                         196  // 
#define ON_BOOST_VOLTAGE_STEP_UP                        197  // 
#define ON_DOUBLE_BOOST_VOLTAGE_STEP_UP                 198  // 
#define BATTERIES_DISCONNECTED                          199  //
#define UPS_CABINET_OVERTEMPERATURE                     200  // 
#define TRANSFORMER_OVERTEMPERATURE                     201  // 
#define AMBIENT_UNDERTEMPERATURE                        202  // 
#define AMBIENT_OVERTEMPERATURE                         203  // 
#define CABINET_DOOR_OPEN                               204  // 
#define CABINET_DOOR_OPEN_WITH_VOLTAGE_PRESENT          205  // 
#define AUTOMATIC_SHUTDOWN_PENDING                      206  //
#define TAP_SWITCHING_RELAY_FAILURE                     207  // 
#define UNABLE_TO_CHARGE_BATTERIES                      208  // 
#define STARTUP_FAILED_CHECK_EPO_RESET                  209  // 
#define AUTOMATIC_STARTUP_PENDING                       210  //
#define MODEM_FAILED                                    211  //
#define INCOMING_MODEM_CALL_STARTED                     212  //
#define CALLING_PHONE                                   213  //
#define MODEM_CONNECTION_ESTABLISHED                    214  //
#define MODEM_CALL_COMPLETED_SUCCESSFULLY               215  //
#define MODEM_CALL_COMPLETION_FAILED                    216  //
#define INPUT_BREAKER_FAILED                            217  //
#define SYSTEM_INITIALIZATION_IN_PROGRESS               218  // 
#define AUTOCALIBRATION_FAILED                          219  // 
#define SELECTIVE_TRIP_OF_MODULE                        220  //
#define INVERTER_OUTPUT_FAILURE                         221  //
#define ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP              222  // 
#define RECTIFIER_OVERTEMPERATURE                       223  //
#define CONFIGURATION_ERROR                             224  //
#define REDUNDANCY_LOSS_DUE_TO_OVERLOAD                 225  // 
#define ON_ALTERNATE_AC_SOURCE                          226  // 
#define IN_HIGH_EFFICIENCY_MODE                         227  // 
#define SUMMARY_NOTICE                                  228  //
#define SUMMARY_ALARM                                   229  //
#define ALTERNATE_POWER_SOURCE_NOT_AVAILABLE            230  // 
#define CURRENT_BALANCE_FAILURE                         231  // 
#define CHECK_AIR_FILTER                                232  // 
#define SUBSYSTEM_NOTICE_ACTIVE                         233  // 
#define SUBSYSTEM_ALARM_ACTIVE                          234  // 
#define CHARGER_ON_COMMAND                              235  //
#define CHARGER_OFF_COMMAND                             236  //
#define UPS_ON_NORMAL                                   237  //
#define INVERTER_PHASE_ROTATION                         238  //
#define UPS_OFF_COMMAND                                 239  //
#define EXTERNAL_COMMUNICATION_FAILURE                  240  //
#define INVALID_BOARD_ID                                241  //
#define CHECK_INVERTER_SWITCHGEAR                       242  //
#define OUTPUT_KW_OVERLOAD                              243  //
#define PRECHARGE_FAILED                                244  //
#define RECTIFIER_STATUS                                245  //
#define NEUTRAL_AMPS_LIMIT                              246  //
#define CLOSE_BATTERY_BREAKER                           247  //
#define UPS_ON_GENERATOR                                248  //
#define RECTIFIER_NEUTRAL_OT_SHUTDOWN_LVL               249  //
#define RECT_TEMP_SENSOR_FAIL                           250  //
#define INVERTER_CONTACTOR_OPEN                         251  // 
#define RECTIFIER_CONTACTOR_OPEN                        252  //
#define BYPASS_BREAKER_OPEN                             253  //
#define BATTERY_CONTACTOR_OPEN                          254  //
#define BACKFEED_SWITCHGEAR                             255  // 
#define BATTERY_TEST_IN_PROGRESS                        256  //
#define SYSTEM_TEST_IN_PROGRESS                         257  //
#define TEST_ABORTED                                    258  //
#define RECTIFIER_PHASE_A_CURRENT_LIMIT                 259  //
#define RECTIFIER_PHASE_B_CURRENT_LIMIT                 260  //
#define RECTIFIER_PHASE_C_CURRENT_LIMIT                 261  //
#define NOT_ENOUGH_UPMS                                 262  //
#define MODEM_CALL_ABORTED                              263  //
#define NOT_DEFINED                                     264  //
#define HQ_MISSING_SEQ_NUM                              265
#define NB_266                                          266
#define NB_267                                          267
#define LEVEL_1_OVERLOAD_N                              268  //
#define LEVEL_2_OVERLOAD_N                              269  //
#define LEVEL_3_OVERLOAD_N                              270  //
#define NB_271                                          271
#define NB_272                                          272
#define NB_273                                          273
#define NB_274                                          274
#define NB_275                                          275
#define NB_276                                          276
#define EEP_SETUP_ERROR                                 277
#define OUTPUT_PHASE_ROTATION                           278
#define NB_279                                          279
#define TOO_MANY_RESTARTS                               280
#define UPM_SERVICE_MODE                                281
#define NB_282                                          282
#define NB_283                                          283
#define NB_284                                          284
#define NB_285                                          285
#define NB_286                                          286
#define NB_287                                          287
#define NB_288                                          288
#define NB_289                                          289
#define NB_290                                          290
#define NB_291                                          291
#define NB_292                                          292
#define NB_293                                          293
#define NB_294                                          294
#define NB_295                                          295
#define NB_296                                          296
#define NB_297                                          297
#define BYPASS_1PH_AC_UNDER_VOLTAGE                     298
#define DEVELOPMENT_CODE                                299
#define CHECK_MCU                                       300
#define RESETTING_EEPROM                                301
#define EEPS_CONFIGURING                                302
#define PULL_CHAIN                                      303
#define CHECK_PULL_CHAIN                                304
#define MCU_NOT_RESPONDING                              305
#define PMF_NOT_RESPONDING                              306
#define CSB_NOT_RESPONDING                              307
#define LCD_NOT_RESPONDING                              308
#define CAN_BRIDGE_NOT_RESPONDING                       309
#define NOT_ENOUGH_BYPASS_CAPACITY                      310
#define PARALLEL_SYSTEM_OVERLOAD                        311
#define NOT_SYNCHRONIZED                                312
#define BYPASS_SYNC_DISABLED                            313
#define ABNORMAL_OUTPUT_VOLTAGE                         314
#define PARALLEL_SETUP_ERROR                            315
#define CLOCK_SET_DONE                                  316
#define CHECK_CB4_FEEDBACK_CABLE                        317
#define DUAL_CAN_FAILURE                                318
#define BATTERY_START_MODE                              319
#define RECTIFIER_CURRENT_LIMIT                         320
#define INVERTER_CURRENT_LIMIT                          321
#define LOSS_OF_INHERENT_REDUNDANCY                     322
#define IN_APM_MODE                                     323
#define ALARM_TEST_MODE                                 324
#define ALTERNATE_SYNC                                  325
#define EXTERNAL_COMMUNICATION_EVENT                    326
#define MBS_CLOSED                                      327
#define MOB_OPEN                                        328
#define MOB_FAILURE                                     329
#define EXTERNAL_PARALLEL                               330
#define SYSTEM_IS_REDUNDANT                             331
#define DRIVER_FAULT                                    332
#define PM_OVER_TEMPERATURE                             333
#define BALANCER_LOOP_OVER_CURRENT                      334
#define BATTERY_BREAKER_FAILURE                         335
#define TOO_MANY_BATTERY_TRANSFERS                      336
#define SPI_BUS_FAILURE                                 337
#define PARALLEL_LOAD_SHARE_ERROR                       338
#define HE_NOT_AVAILABLE                                339
#define INPUT_ABNORMAL                                  340
#define BATTERY_STARTUP_FAILURE                         341
#define PLD_CONFIG_FAIL                          		342
#define DISCHARGE_DCLINK                                343
#define BATTERYBRK_FEEDBACK_CLOSE						344
#define CAPACITOR_OVER_TEMPERATURE						345
#define BATTERYBRK_TRIP									346
#define ESS_INVERTER_GAPPING							347
#define DC_LINK_FUSE_FAILURE					     	348
#define PM_OVER_TEMPERATURE_TRIP                        349
#define REC_PWM_OFF										350
#define ADAPTIVE_OVERLOAD_CAPACITY_OFF                  351
#define STS_OVER_TEMPERATURE                            352
#define RECTIFIER_SWITCHGEAR_OPEN_R						353
#define BAT_LEGB_RELAY_OPEN								354
#define LOSS_OF_PWM_SYNC                                355
#define BACKFEED_RELAY_CLOSE                            356

#define ABM_CHARGE_MODE                                 1600
#define ABM_FLOAT_MODE                                  1601
#define ABM_REST_MODE                                   1602
#define ABM_STATE_OFF                                   1603
#define OUTPUT_HOT                                      1604
#define BYPASS_HOT                                      1605
#define BATTERY_DISCHARGING                             1606
#define INVERTER_CONTACTOR_CLOSED                       1607
#define MIS_CLOSED                                      1608
#define MIS_INSTALLED                                   1609
#define BYPASS_INSTALLED                                1610
#define MAINTENANCE_BYPASS_INSTALLED                    1611
#define BATTERY_INSTALLED                               1612
#define PARALLEL_CAN_ERROR                              1613
#define TOO_MANY_INVERTER_TRANSFERS                     1614
#define STATIC_SWITCH_SHORT                             1615
#define OUTPUT_OVERLOAD_TRIP                            1616
#define ECO_ENABLE                                      1617

#define MCU_STATE_CHANGE                                2000
#define BYPASS_STATE_CHANGE                             2001
#define RECTIFIER_STATE_CHANGE                          2002
#define BATTERY_STATE_CHANGE                            2003
#define INPUT_SYNC_OUT_OF_RANGE                         2004
#define IN_EASY_CAPACITY_TEST_MODE                      2005
#define I2C_BUS_FAIL                                    2006
#define ECT_COMMAND                                     2007
#define ABNORMAL_EXIT_ECT_MODE                          2008
//#define INTERNAL_SYNC_FAIL                              2009
#define PARALLEL_SETUP_FAIL                             2010
//#define PARALLEL_SYNC_FAIL                              2011
#define SYNC_STATE_CHANGE                               2012

#define EVENT_LEVEL_NONE                                0
#define EVENT_LEVEL_EVENT                               32
#define EVENT_LEVEL_NOTICE                              35
#define EVENT_LEVEL_ALARM                               41

// Defines for NB_Cfg_AlmLvl - Alarm Level
// Note: Alarm Level directly maps to XCP Action Level (See section 5.4 of XCP Specification).
//          ActionLevel = NB_Cfg_XCPAlarmLevel + 0x1F
#define USR_ST                                          32           // User oriented Status                 - Maps to XCP Action Level 32
#define SVC_ST                                          33           // Service oriented Status              - Maps to XCP Action Level 33
#define COMD                                            34           // UPS Command (On/Off, etc)            - Maps to XCP Action Level 34
#define NOTC1                                           35           // Notice level 1, Information Only     - Maps to XCP Action Level 35
#define NOTC2                                           36           // Notice level 2, User Instruction     - Maps to XCP Action Level 36
#define NOTC3                                           37           // Notice level 3, Protection Level Downgraded  - XCP Action Level 37
#define NOTC4                                           38           // Notice level 4, Investigate UPS      - Maps to XCP Action Level 38
#define NOTC5                                           39           // Notive level 5, Schedule Service     - Maps to XCP Action Level 39
//                                                      40           // Notice level 6, Spare                - Maps to XCP Action Level 40
#define USR_AL                                          41           // User Alarm, user Action required     - Maps to XCP Action Level 41
//                                                      42           // Alarm Level 2, Spare                 - Maps to XCP Action Level 42
#define SHT_AL                                          43           // Shutdown is imminent Alarm           - Maps to XCP Action Level 43
#define SCD_AL                                          44           // Schedule Service Alarm               - Maps to XCP Action Level 44
#define SVC_IN                                          45           // Service oriented Alarm               - Maps to XCP Action Level 45
#define SVC_AL                                          46           // Service of the UPS is required       - Maps to XCP Action Level 46



// ********************************************************************************************************
// *        END OF XCPDefs.h
// ********************************************************************************************************
#endif
