// ********************************************************************************************************
// *            Alarms.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Alarms.h
// *
// *    DESCRIPTION: Contains monitoring of aux voltages, temperatures, etc.
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *    DATE: 10/8/2003
// *
// *    HISTORY: See CVS history.
// *********************************************************************************************************
#ifndef ALARMS_H
#define ALARMS_H

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************

// *********************************************************************************************************
// *        DEFINES
// *********************************************************************************************************
#define FUSE_RECT_PH_A      (0x0001)
#define FUSE_RECT_PH_B      (0x0002)
#define FUSE_RECT_PH_C      (0x0004)
#define FUSE_BATT_POS       (0x0010)
#define FUSE_BATT_NEG       (0x0020)
#define FUSE_INV_PH_A       (0x0100)
#define FUSE_INV_PH_B       (0x0200)
#define FUSE_INV_PH_C       (0x0400)

#define FuseFailMaxRestartTimes  1
#define FUSE_WAIT_1_HOUR         36000L
// *********************************************************************************************************
// *        Global Variables
// *********************************************************************************************************
extern uint16_t MaxInverterHWCLCycles;      // EE Var
extern uint16_t MaxRectifierHWCLCycles;     // EE Var
extern uint16_t MaxBatteryHWCLCycles;       // EE Var

extern float AbsDCOVSet;                    // EE var
extern float AbsDCUVSet;                    // EE var

extern bool AdaptOverloadCapacityOff;

extern uint16_t InverterCLCount;

//debugger variables
/*
extern uint16_t MaxInverterCLCount;
extern uint16_t MaxRectifierCLCount;
extern uint16_t MaxBatteryCLCount;

extern uint16_t InvTempWarningRT;
extern uint16_t RecTempWarningRT;
extern uint16_t BatTempWarningRT;

extern uint16_t InvTempTripRT;
extern uint16_t RecTempTripRT;
extern uint16_t BatTempTripRT;
*/
extern uint16_t BatteryCLCount;
extern bool AteMaskAlarm5VforRsEep;

// ********************************************************************************************************
// *        FUNCTION PROTOTYPES
// ********************************************************************************************************
void CheckAlarmsFast( void );
void CheckAlarms20ms( void );
void CheckTemperatureAlarms( void );
void CheckFuseFailures( void );
void InitAlarms( void );
void FastAuxPowerShutdown( void ); 
void CheckCurrentLimits( void );
void BatteryCLReset( void );
void CheckEpoAlarm( void );
void CheckSiteWiring( void );
void CheckAdapOverloadCapacity( void);
bool isFastAuxPowerShutdown( void );
                  
// ********************************************************************************************************
// *            END OF Alarms.h
// ********************************************************************************************************
#endif



