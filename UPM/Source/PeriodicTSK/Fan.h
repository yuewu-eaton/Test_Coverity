// ********************************************************************
// *            Fan.h
// ********************************************************************
// ********************************************************************
// * 
// * This information if proprietary to Eaton Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 2005 Eaton Corporation                       
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   Fan.h
// *                                                                      
// *     DESCRIPTION: 
// *                  
// *     ORIGINATOR:  Jun Zhang                                         
// *                                                                      
// *     DATE:        05/04/2010                                            
// *                                                                      
// *     HISTORY:                                                         
// ********************************************************************
#ifndef _FAN_H
#define _FAN_H

#include <stdint.h>
#include "Meters.h"

// ********************************************************************
// *            Global data declarations 
// ********************************************************************
extern uint16_t FanSpeed;
extern uint16_t FanSpeedNew;
extern uint16_t FanMaxSpeed;
extern uint16_t FanFailErrorLimit;
extern uint16_t FanStatus;
extern uint16_t FanLoad;

extern uint16_t FanA_Speed_Temp;
extern uint16_t FanB_Speed_Temp;
extern uint16_t FanC_Speed_Temp;

extern uint16_t FanA_Speed_Final;
extern uint16_t FanB_Speed_Final;
extern uint16_t FanC_Speed_Final;

extern uint16_t FanTestSpeedUPM;
extern uint16_t FanTestSpeedSTS;
extern uint16_t FanTestSpeedUpmEnable;
extern uint16_t FanTestSpeedSTSEnable;
extern uint16_t STSFanSpeed;
extern uint16_t STSFanDutyToPLD;
extern uint16_t FanDutyToPLD;

class FanState
{
    private:
        // Declare these prototypes to be private to force the class to be non-
        // copyable.  See also boost::noncopyable.
        FanState(const FanState& other);
        const FanState& operator=(const FanState& other);
    
    public:
        explicit FanState( uint16_t num )
        {
            FanNumber = num;
            FanFailCount = 0;
        }
        ~FanState(){}

        void ClearCount( void )
        {
            FanFailCount = 0;
        }
        
    private:
        uint16_t FanFailCount;
        uint16_t FanNumber;
};

// ********************************************************************
// *            Function prototypes
// ********************************************************************
extern void FanHv20_40KControl(void);
extern void STSFanControl( const float LoadPercent, const float UpmAmbTemp );
//extern void SimpleFanControl( void );
extern void STSFanStateCheck( void );
extern void FanSpeedCalculate(void);
extern void FanSpeedCheck_pld(void);
extern void FanTestWithSpeed( const uint16_t Testfan, const uint16_t SpeedPercent);
extern void FanTestWithLoad(const uint16_t TestEnable, const uint16_t LoadPercent);

#endif
// ********************************************************************
// *            End of Fan.h   
// ********************************************************************
