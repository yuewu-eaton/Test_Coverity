#ifndef _FCTSTATE_H
#define _FCTSTATE_H
// ********************************************************************************************************
// *                 FCTState.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2014 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME:   FCTState.h
// *
// *    DESCRIPTION:                   
// *
// *    ORIGINATOR:  Pasi Pulkkinen, Tuomo Kaikkonen
// *
// *    DATE:        2010/06/22
// *
// *    HISTORY:     See SVN history
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "Eeprom_Map.h"
#include "StateTimer.h"

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************

/*** DEFINE THE MACHINE STATES ****/

enum eFCTState
{
    FCT_IDLE_STATE               =           0,
    FCT_CHG_TEST_STATE           =           1,
    FCT_INV_TEST_STATE           =           2,
    FCT_BOOST_TEST_STATE         =           3,
    FCT_RECTIFIER_TEST_STATE     =           4,
    FCT_LINEPMDISCHARGE_STATE    =           5
};
enum eFCTResult
{
    FCT_NOT_START         =   0,
    FCT_PASS              =   1,
    FCT_IN_PROGRESS       =   2,
    FCT_TEST_FAILED       =   3,
    FCT_DISCHARGE_FAILED  =   4
};

// Time constants, based on 196us task.

// Alarm values for Abnormal output voltage

// FCT boost soft start definition
#define STATE_ERROR            0x1111
#define STATE_IN_PROGRESS      0x2222
#define STATE_SUCCESS          0x3333


// ****************************************************************
// FCT Status Bit definitions
// ****************************************************************
// Alex/ 20131119  add for FCT begin..
struct stFCTstruct
{
    //  Word 0
    uint16_t ChargerTestFail       :    1;      // word0 bit 0
    uint16_t InverterTestFail      :    1;      // bit 1
    uint16_t BoostTestFail         :    1;      // bit 2
    uint16_t RectifierTestFail     :    1;      // bit 3

    uint16_t ChargerControlOn      :    1;      // bit 4
    uint16_t InverterControlOn     :    1;      // bit 5
    uint16_t BoostControlOn        :    1;      // bit 6
    uint16_t RectifierControlOn    :    1;      // bit 7

    uint16_t ChargerTestCmd        :    1;      // bit 8
    uint16_t InverterTestCmd       :    1;      // bit 9
    uint16_t BoostTestCmd          :    1;      // bit 10
    uint16_t RectifierTestCmd      :    1;      // bit 11

    uint16_t CancelTestCmd         :    1;      // bit 12
    uint16_t LineShortTest             :    1;      // bit 13
    uint16_t LineDischargeCmd      :    1;      // bit 14
    uint16_t LinePMDischargeOn     :    1;      // bit 15
    
    // Word 1
    uint16_t FCTModeOn             :    1;      // word1 bit 0
    uint16_t Reserved              :    15;     // bit 1~15
};

union unFCTunion
{
    stFCTstruct bit;
    uint16_t    word[sizeof(stFCTstruct)];
};

// Alex add ..end

// ********************************************************************************************************
// * FCT STATE MACHINE CLASS DEFINITION
// ********************************************************************************************************
//Alex /20131119 add for FCT begin..
class FCTStateControl
{
private:
    void FCT_IdleState                          (void);
    void FCT_ChargerState                       (void);
    void FCT_InverterState                      (void);
    void FCT_BoostState                         (void);
    void FCT_RectifierState                     (void);
    void FCT_LinePMDischargeState               (void);

    inline bool FCT_RecVoltageCheck                    (float volt);
    inline bool FCT_DcLinkCheck_BothHigherThan         (float volt);
    inline bool FCT_DcLinkCheck_AllHigherThan          (float volt);
    inline bool FCT_DcLinkCheck_BothLowerThan          (float volt);
    inline bool FCT_DcLinkCheck_AllLowerThan           (float volt);
    inline bool FCT_DcLinkMarginCheck_LowerThan        (float volt);
    inline bool FCT_Battery2Check_HigherThan           (float volt);
    inline bool FCT_Battery2Check_LowerThan            (float volt);
    inline bool FCT_InverterVoltageCheck_AllHigherThan (float volt);
    inline bool FCT_InverterVoltageCheck_AllLowerThan  (float volt);
    inline bool FCT_InverterVoltageCheck_AnyHigherThan (float volt);
    inline bool FCT_InverterVoltageCheck_AnyLowerThan  (float volt);
    inline bool FCT_INVCurrent_Check_AnyHigherThan  (float current);
    inline bool FCT_RecCurrent_Check_AnyHigherThan  (float current);
    
    uint16_t    FCT_DcLink_Softstart                   (float volt);
    //float PreRMSVoltageNormFactor;
    bool  FCT_ReceriveExitCmd;

protected:
    

public:
    eFCTState  FCTRunState;
    unFCTunion FCT_Status;
    uint16_t   FCT_Time;
    uint16_t   FCT_Phase;
    uint16_t   FCT_Error;
    float      FCT_DcLinkRef;
    eFCTResult FCT_Result;
    uint16_t   FCT_TimeRunInSec;
    uint16_t   EPOEnabledStore; // Record EPODisabled status
    bool       BoardID_Warning;

    StateTimer FCT_Timer1;
    StateTimer FCT_Timer2;
    StateTimer FCT_Timer3;
    StateTimer FCT_Timer4;

    FCTStateControl();
    ~FCTStateControl(){};

    void RunStateMachine( void );
    void ClearAllCmd( void );
    void ClearResult( void );

    eFCTState GetFCTState( void )
    {
        return( FCTRunState );
    }
    eFCTResult GetFCTResult( void )
    {
        return( FCT_Result );
    }

    void EnableFCT( void );
    void DisableFCT( void );

    const unFCTunion& GetFCTStatus( void )
    {
        return( FCT_Status );
    }

    uint16_t GetFCTTime( void )
    {
        return( FCT_Time );
    }

    bool GetFCTMode( void )
    {
        return( FCT_Status.bit.FCTModeOn );
    }
};

extern FCTStateControl FCTStateMachine;

// ********************************************************************************************************
// *                 END OF FCTState.h
// ********************************************************************************************************
#endif

