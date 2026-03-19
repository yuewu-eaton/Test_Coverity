#ifndef ALARMS_AC_H
#define ALARMS_AC_H
// ********************************************************************************************************
// *            Alarms_AC.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Alarms_AC.h
// *
// *    DESCRIPTION: Contains monitoring of mcu status bits for all ac alarms
// *
// *
// *    ORIGINATOR: Pasi Pulkkinen
// *
// *    DATE: 17.12.2003
// *
// *    HISTORY: See CVS history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "Eeprom_Map.h"
#include "FilteredBit.h"
 
// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

//input abnormal error code
enum
{
	NOERROR,
	SEL_TRIP,
    INPUT_HIGH
};

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
extern uint16_t OverloadTime1;            // EE var
extern uint16_t OverloadTime2;            // EE var
extern uint16_t OverloadTime3;            // EE var
extern uint16_t OverloadTime4;            // EE var
extern uint16_t OpACUVActiveTime;         // EE var
extern uint16_t AdapOverloadTime1;        // EE var
extern uint16_t AdapOverloadTime2;        // EE var
extern uint16_t AdapOverloadTime3;        // EE var
extern uint16_t AdapOverloadTime4;        // EE var
extern int16_t OverloadLevel1;            // EE var    
extern int16_t OverloadLevel2;            // EE var
extern uint16_t OutRMSOVDelayCount;
extern uint16_t InstantVoltageState;
extern float UtilityMaxRMSLimit;
extern float UtilityMinRMSLimit;
extern float NegPowerLimit;
extern float NegPowerLimNorm;

extern FilteredBit BypassRMSOV;
extern FilteredBit BypassRMSUV;
extern FilteredBit Bypass25RMSUV;
extern FilteredBit Output25RMSUV;
extern FilteredBit OutVolRMS_NotPresent;

// ********************************************************************************************************
// * FUNCTION PROTOTYPES
// ********************************************************************************************************

void CheckFasterACAlarms(void);
void CheckFastACAlarms(void);
void CheckMediumACAlarms(void);
void CheckSlowACAlarms(void);

void CheckOverloadMedium(void);
void CheckSTSWshort(void);
void CheckInstantVoltage(void);
void ee_update_overload_limits( const EE_ID* ee, const uint16_t* data );

extern uint16_t AcuvCheck(float VoltageSd, float UnderLimit, uint16_t ActiveTime, uint16_t *Counter);
// ********************************************************************************************************
// * DEFINED VALUES
// ********************************************************************************************************

// ********************************************************************************************************
// *            END OF Alarms_AC.h
// ********************************************************************************************************
#endif



