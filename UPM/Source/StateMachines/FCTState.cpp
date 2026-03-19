// ******************************************************************************************************
// *                 FCTState.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2015 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME:   FCTState.cpp
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR:
// *
// *
// *    DATE:        2015/06/11
// *
// *    HISTORY:     See SVN history.
// ******************************************************************************************************


// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Abm.h"
#include "Constants.h"
#include "Debugger.h"
#include "FCTState.h"
#include "Spi_Task.h"
#include "RectifierStateControl.h"
#include "Eeprom_Map.h"
#include "InvSync.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "Meters.h"
#include "IOexpansion.h"
#include "InverterControl.h"
#include "F28335Port.h"
#include "StateTimer.h"
#include "BootloaderAPI.h"
#include "math.h"
#include "ACMeter.h"
#include "HQ_Funcs.h"
#include "History.h"
#include "Version.h"
#include "InternalCan.h"
#include "Alarms_AC.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include <cmath>


// ********************************************************************************************************
// * EXTERNAL DECLARATIONS
// ********************************************************************************************************
extern uRawAdcData RawAdcData;
// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
FCTStateControl FCTStateMachine;

// ********************************************************************************************************
// *
// * Function:     Class Constructor
// *
// * Purpose:      Initialize things
// *
// ********************************************************************************************************
FCTStateControl::FCTStateControl()
{
    FCTRunState = FCT_IDLE_STATE;
    FCT_Status.word[0] = 0;
    FCT_Status.word[1] = 0;
    FCT_ReceriveExitCmd = false;
    FCT_Time = 10;
    FCT_Phase = 0;
    FCT_Error = 0;
    FCT_DcLinkRef = 0;
    EPOEnabledStore = 0;
    FCT_Result = FCT_NOT_START;
    FCT_TimeRunInSec = 0;
    FCT_Timer1.ClearTimer();
    FCT_Timer2.ClearTimer();
    FCT_Timer3.ClearTimer();
    FCT_Timer4.ClearTimer();
    BoardID_Warning = true;
}

// ********************************************************************************************************
// *
// * Function: RunStateMachine(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Run state machine of FCT
// *
// ********************************************************************************************************
void FCTStateControl::RunStateMachine(void)
{
    switch(FCTRunState)
    {
        case FCT_IDLE_STATE :
             FCT_IdleState();
             break;

        case FCT_CHG_TEST_STATE :
             FCT_ChargerState();
             break;

        case FCT_INV_TEST_STATE :
             FCT_InverterState();
             break;

        case FCT_BOOST_TEST_STATE :
             FCT_BoostState();
             break;

        case FCT_RECTIFIER_TEST_STATE :
             FCT_RectifierState();
             break;

        case FCT_LINEPMDISCHARGE_STATE :
             FCT_LinePMDischargeState();
             break;

        default:
             FCTRunState = FCT_IDLE_STATE;
             break;
    }
}

// ********************************************************************************************************
// *
// * Function: FCT_IdleState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Reset everything, 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_IdleState(void)
{
    BatteryConverter.BoostOff();
    BatteryConverter.ChargerOff();
    Rectifier.RectifierOff();
    Inverter.Off();
    PreChargeOff();
    FCT_Timer1.ClearTimer();
    FCT_Timer2.ClearTimer();
    FCT_Timer3.ClearTimer();
    FCT_Timer4.ClearTimer();
    FCTRunState = FCT_IDLE_STATE;

    //DSPOutRegister.GpoB.bit.BalancerRelay = 1;
    //DSPOutRegister.GpoA.bit.NeutralRelay = 1;

    FCT_Status.bit.ChargerControlOn = 0;
    FCT_Status.bit.InverterControlOn = 0;
    FCT_Status.bit.BoostControlOn = 0;
    FCT_Status.bit.RectifierControlOn = 0;
    FCT_Status.bit.CancelTestCmd = 0;
    FCT_Status.bit.LinePMDischargeOn = 0;

    if( FCT_Status.bit.ChargerTestCmd  && !FCT_Status.bit.ChargerTestFail )
    {
        FCT_Status.bit.ChargerTestCmd = 0;
        FCTRunState = FCT_CHG_TEST_STATE;
    }
    else if( FCT_Status.bit.InverterTestCmd && !FCT_Status.bit.InverterTestFail )
    {
        FCT_Status.bit.InverterTestCmd = 0;
        FCTRunState = FCT_INV_TEST_STATE;
    }
    else if( FCT_Status.bit.BoostTestCmd && !FCT_Status.bit.BoostTestFail )
    {
        FCT_Status.bit.BoostTestCmd = 0;
        FCTRunState = FCT_BOOST_TEST_STATE;
    }
    else if( FCT_Status.bit.RectifierTestCmd && !FCT_Status.bit.RectifierTestFail )
    {
        FCT_Status.bit.RectifierTestCmd = 0;
        FCTRunState = FCT_RECTIFIER_TEST_STATE;
    }
    else if( FCT_Status.bit.LineDischargeCmd && !FCT_Status.bit.RectifierTestFail) // Use rectifier to discharge line PM Bus voltage
    {
        FCT_Status.bit.LineDischargeCmd = 0;
        FCTRunState = FCT_LINEPMDISCHARGE_STATE;
    }

    if ( FCTRunState != FCT_IDLE_STATE )
    {
        FCT_Result = FCT_IN_PROGRESS;
    }
    
    if( FCT_Status.bit.ChargerTestFail    ||
        FCT_Status.bit.InverterTestFail   ||
        FCT_Status.bit.BoostTestFail      ||
        FCT_Status.bit.RectifierTestFail  )
    {
        FCT_Result = FCT_TEST_FAILED;
    }
    
    if( FCT_ReceriveExitCmd == true )
    {
        FCT_ReceriveExitCmd = false;
        FCT_Status.bit.FCTModeOn = 0;
        EPOEnabled = EPOEnabledStore;
    }
}

// ********************************************************************************************************
// *
// * Function: FCT_ChargerState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_ChargerState(void)
{
    if( !FCT_Status.bit.ChargerControlOn )
    {
        if( !FCT_Status.bit.CancelTestCmd         &&
            FCT_Battery2Check_LowerThan(50.0)     && //Voltage Between capacitance should lower than 50V
            FCT_DcLinkCheck_BothHigherThan(255.0) && //Voltage between DCBus+(-) and N should be 300V(+-15%)
            FCT_DcLinkCheck_BothLowerThan(345.0) )
        {
            FCT_Timer2.ClearTimer();
            BatteryConverter.ChargerFixedDutyOn();  //FCTTODO: fix duty 0.01
            FCT_Status.bit.ChargerControlOn = 1;

        }
        else // CancelTestCmd or Voltage outof 300V(+-15%)
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0001;
            }
            if( !FCT_DcLinkCheck_BothHigherThan(255.0) )
            {
                FCT_Error |= 0x0002;
            }
            if( !FCT_DcLinkCheck_BothLowerThan(345.0) )
            {
                FCT_Error |= 0x0004;
            }
            if( !FCT_Battery2Check_LowerThan(50.0) )
            {
                FCT_Error |= 0x0008;
            }

            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            BatteryConverter.ChargerFixedDutyOff();
            FCT_Status.bit.ChargerTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.ChargerControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
    else // ChargerControlOn
    {
        if( !FCT_Status.bit.CancelTestCmd         &&
            FCT_DcLinkCheck_BothHigherThan(255.0) && //Voltage between DCBus+(-) and N should be 300V(+-15%)
            FCT_DcLinkCheck_BothLowerThan(345.0) )
        {
            FCT_Timer1.ClearTimer();
            FCT_Timer2.IncTimer();
            FCT_TimeRunInSec = ( FCT_Timer2.TimerValue() / ONE_SECOND );
            if( FCT_Timer2.TimerValue() >= ONE_SECOND * FCT_Time )
            {
                if( FCT_Battery2Check_HigherThan(480.0) && // Charge voltage between 600V(+-20%)
                    FCT_Battery2Check_LowerThan(720.0) )
                {
                    FCT_Status.bit.ChargerTestFail = 0;
                    FCT_Result = FCT_PASS;
                }
                else
                {
                    FCT_Status.bit.ChargerTestFail = 1;
                    FCT_Error |= 0x4000;
                }
                BatteryConverter.ChargerOff();
                FCT_Timer2.ClearTimer();
                FCT_TimeRunInSec = 0;
                FCT_Status.bit.CancelTestCmd = 0;
                FCT_Status.bit.ChargerControlOn = 0;
                FCTRunState = FCT_IDLE_STATE;
            }
        }
        else // CancelTestCmd or Voltage outof 300V(+-15%)
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0100;
            }
            if( !FCT_DcLinkCheck_BothHigherThan(255.0) )
            {
                FCT_Error |= 0x0200;
            }
            if( !FCT_DcLinkCheck_BothLowerThan(345.0) )
            {
                FCT_Error |= 0x0400;
            }
            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            BatteryConverter.ChargerOff();
            FCT_TimeRunInSec = 0;
            FCT_Status.bit.ChargerTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.ChargerControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
}

// ********************************************************************************************************
// *
// * Function: FCT_BoostState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Boost 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_BoostState(void)
{
    if( !FCT_Status.bit.BoostControlOn )
    {
        if( !FCT_Status.bit.CancelTestCmd           &&
            FCT_DcLinkCheck_AllHigherThan( 240.0 )  && // Voltage should be greater than 300V (-20%)
            FCT_DcLinkCheck_AllLowerThan( 800.0 )   &&
            FCT_DcLinkMarginCheck_LowerThan( 100.0 ) ) // Voltage error between Bus+ and Bus- should lower than 100V
        {
            FCT_Timer2.ClearTimer();
            FCT_Timer1.IncTimer();
            if( FCT_Timer1.TimerValue() == WAIT_100_MS )
            {
                FCT_DcLinkRef = 300.0;
                BatteryConverter.SetDCLinkVoltage(FCT_DcLinkRef);
                BatteryConverter.BoostOn();
            }
            else if( ( FCT_Timer1.TimerValue() > WAIT_100_MS ) &&
                     ( FCT_Timer1.TimerValue() < ONE_SECOND * 20 ) )
            {
                uint16_t state = FCT_DcLink_Softstart(450.0);   //FCTTODO, dc link rise speed
                if( state == STATE_SUCCESS )
                {
                    FCT_Status.bit.BoostControlOn = 1;
                    FCT_Timer1.ClearTimer();
                }
            }
            else if( FCT_Timer1.TimerValue() >= ONE_SECOND * 20 )  // timeout
            {
                BatteryConverter.BoostOff();
                FCT_Status.bit.CancelTestCmd = 1;
                FCT_Timer1.ClearTimer();
                FCT_Error |= 0x8000;
            }
        }
        else //CancelTestCmd or BAT DClink voltage out of range
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0001;
            }
            if( !FCT_DcLinkCheck_AllHigherThan( 240.0 ) )
            {
                FCT_Error |= 0x0002;
            }
            if( !FCT_DcLinkCheck_AllLowerThan( 800.0 ) )
            {
                FCT_Error |= 0x0004;
            }
            if( !FCT_DcLinkMarginCheck_LowerThan( 100.0 ) )
            {
                FCT_Error |= 0x0010;
            }
            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            FCT_Status.bit.BoostTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.BoostControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
    else //BoostControlOn
    {
        if( !FCT_Status.bit.CancelTestCmd           &&
            FCT_DcLinkCheck_AllLowerThan( 800.0 )	&&
            FCT_DcLinkMarginCheck_LowerThan( 100.0 ) ) // Margin Voltage of Bus+ and Bus- should lower than 100V
        {
            FCT_Timer1.ClearTimer();
            FCT_Timer2.IncTimer();
            FCT_TimeRunInSec = ( FCT_Timer2.TimerValue() / ONE_SECOND );
            if( FCT_Timer2.TimerValue() == ONE_SECOND * FCT_Time )
            {
                if( FCT_DcLinkCheck_AllHigherThan( 382.0 ) && // Voltage between Bus+ and Bus- should be 450V(+-15%)
                    FCT_DcLinkCheck_AllLowerThan( 517.0 ) )
                {
                    FCT_Status.bit.BoostTestFail = 0;
                    FCT_Result = FCT_PASS;
                }
                else
                {
                    FCT_Status.bit.BoostTestFail = 1;
                    FCT_Error |= 0x4000;
                }
                BatteryConverter.BoostOff();
                FCT_Timer2.ClearTimer();
                FCT_TimeRunInSec = 0;
                FCT_Status.bit.CancelTestCmd = 0;
                FCT_Status.bit.BoostControlOn = 0;
                FCTRunState = FCT_IDLE_STATE;
            }
        }
        else // Error status
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0100;
            }
            if( !FCT_DcLinkCheck_AllLowerThan( 800.0 ))
            {
                FCT_Error |= 0x0200;
            }
            if( !FCT_DcLinkMarginCheck_LowerThan( 100.0 ) )
            {
                FCT_Error |= 0x0800;
            }
            BatteryConverter.BoostOff();
            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            FCT_TimeRunInSec = 0;
            FCT_Status.bit.BoostTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.BoostControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: inverter on with RMS loop only 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_InverterState(void)
{
    if( !FCT_Status.bit.InverterControlOn )
    {
        if( !FCT_Status.bit.CancelTestCmd                &&     //FCTTODO
            ( ( FCT_DcLinkCheck_BothHigherThan( 42.5 )  && FCT_DcLinkCheck_BothLowerThan( 57.5 ) ) ||
              ( FCT_DcLinkCheck_BothHigherThan( 255.0 ) && FCT_DcLinkCheck_BothLowerThan( 345.0 ) ) )  )
        {
            FCT_Timer2.ClearTimer();
            FCT_Timer1.IncTimer();
            if( FCT_Timer1.TimerValue() == WAIT_500_MS )
            {
                FCT_Timer1.ClearTimer();
                //PreRMSVoltageNormFactor = Inverter.RMSVoltageNormFactor;
                //Inverter.RMSVoltageNormFactor = 1.0 / 180.0; // Set Output voltage RMS to 180V
                Inverter.OnOpenLoop();
                Inverter.DisableRMS();      //Disable RMS loop
                FCT_Status.bit.InverterControlOn = 1;
            }
        }
        else
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0001;
            }
            if( !FCT_DcLinkCheck_BothHigherThan( 255.0 ) )
            {
                FCT_Error |= 0x0002;
            }
            if( !FCT_DcLinkCheck_BothLowerThan( 345.0 ) )
            {
                FCT_Error |= 0x0004;
            }
            if( !FCT_DcLinkCheck_BothHigherThan( 42.5 ) )   //FCTTODO
            {
                FCT_Error |= 0x0008;
            }
            if( !FCT_DcLinkCheck_BothLowerThan( 57.5 ) )
            {
            	FCT_Error |= 0x0010;
            }

            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            Inverter.Off();
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.InverterControlOn = 0;
            FCT_Status.bit.InverterTestFail = 1;
            FCTRunState = FCT_IDLE_STATE;

        }
    }
    else
    {
        if( !FCT_Status.bit.CancelTestCmd )
        {
            FCT_Timer1.ClearTimer();
            FCT_Timer2.IncTimer();
            FCT_TimeRunInSec = ( FCT_Timer2.TimerValue() / ONE_SECOND );
            if( FCT_Timer2.TimerValue() == ONE_SECOND * ( FCT_Time - 1 ) ) //Check the result one second before end for current calculate
            {
                if( FCT_Status.bit.LineShortTest ) // Inverter test is short test mode
                {
                    if( FCT_INVCurrent_Check_AnyHigherThan( 10.0 ) ) //In ouput short test ,  Current RMS should greater than 15A
                    {
                        FCT_Status.bit.InverterTestFail = 0;
                        FCT_Result = FCT_PASS;
                    }
                    else
                    {
                        FCT_Error |= 0x4000;
                        FCT_Status.bit.InverterTestFail = 1;
                    }
                }
                else // Inverter test is normal test mode
                {
                    if( FCT_InverterVoltageCheck_AnyHigherThan( 153.0 ) &&  // The test phase voltage RMS should be 180(+-15%)
                        FCT_InverterVoltageCheck_AllLowerThan( 207.0 ) )
                    {
                        FCT_Status.bit.InverterTestFail = 0;
                        FCT_Result = FCT_PASS;
                    }
                    else
                    {
                        FCT_Error |= 0x4000;
                        FCT_Status.bit.InverterTestFail = 1;
                    }
                }
            }
            else if( FCT_Timer2.TimerValue() >= ONE_SECOND * FCT_Time )
            {
                FCT_Timer2.ClearTimer();
                Inverter.Off();
                FCT_TimeRunInSec = 0;
                FCT_Status.bit.CancelTestCmd = 0;
                FCT_Status.bit.InverterControlOn = 0;
                FCTRunState = FCT_IDLE_STATE;
            }
        }
        else //CancelTest
        {
            // Record exit reason
            FCT_Error |= 0x0100;

            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            Inverter.Off();
            FCT_TimeRunInSec = 0;
            FCT_Status.bit.InverterTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.InverterControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
}

// ********************************************************************************************************
// *
// * Function: FCT_RectifierState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Rectifier is working like inverter, only the PWM is for rec IGBT 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_RectifierState(void)  //Rectifier test should use inverter output circuit
{

    if( !FCT_Status.bit.RectifierControlOn )
    {
        if( !FCT_Status.bit.CancelTestCmd           &&      //FCTTODO
            ( ( FCT_DcLinkCheck_BothHigherThan( 42.5 )  && FCT_DcLinkCheck_BothLowerThan( 57.5 ) ) ||
              ( FCT_DcLinkCheck_BothHigherThan( 255.0 ) && FCT_DcLinkCheck_BothLowerThan( 345.0 ) ) )
          )
        {
            FCT_Timer2.ClearTimer();
            FCT_Timer1.IncTimer();
            
            if( FCT_Timer1.TimerValue() == WAIT_500_MS )
            {
                FCT_Timer1.ClearTimer();
                Inverter.OnOpenLoop_FCT(); // No RMS
                FCT_Status.bit.RectifierControlOn = 1;
            }
        }
        else
        {
            // Record exit reason
            if( FCT_Status.bit.CancelTestCmd )
            {
                FCT_Error |= 0x0001;
            }
            if( !FCT_DcLinkCheck_BothHigherThan( 255.0 ) )
            {
                FCT_Error |= 0x0002;
            }
            if( !FCT_DcLinkCheck_BothLowerThan( 345.0 ) )
            {
                FCT_Error |= 0x0004;
            }
            if( !FCT_DcLinkCheck_BothHigherThan( 42.5 ) )
            {
                FCT_Error |= 0x0008;
            }
            if( !FCT_DcLinkCheck_BothLowerThan( 57.5 ) )
            {
                FCT_Error |= 0x0010;
            }
            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            Inverter.Off_FCT();
            FCT_Status.bit.RectifierTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.RectifierControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
    else
    {
        if( !FCT_Status.bit.CancelTestCmd )
        {
            FCT_Timer1.ClearTimer();
            FCT_Timer2.IncTimer();
            FCT_TimeRunInSec = ( FCT_Timer2.TimerValue() / ONE_SECOND );
            if( FCT_Timer2.TimerValue() == ONE_SECOND * ( FCT_Time - 1 ) ) //Check the result one second before end for current calculate
            {
                if( FCT_Status.bit.LineShortTest ) // Rectify test is short test mode
                {
                    if( FCT_RecCurrent_Check_AnyHigherThan( 10.0 ) ) //In ouput short test ,  Current RMS should greater than 15A
                    {
                        FCT_Status.bit.RectifierTestFail = 0;
                        FCT_Result = FCT_PASS;
                    }
                    else
                    {
                        FCT_Error |= 0x4000;
                        FCT_Status.bit.RectifierTestFail = 1;
                    }
                }
                else
                {
                    if( FCT_RecVoltageCheck( 180 ) )    // nomial voltage is 180V, ¡À15%
                    {
                        FCT_Status.bit.RectifierTestFail = 0;
                        FCT_Result = FCT_PASS;
                    }
                    else
                    {
                        FCT_Error |= 0x4000;
                        FCT_Status.bit.RectifierTestFail = 1;
                    }
                }
            }
            else if( FCT_Timer2.TimerValue() >= ONE_SECOND * FCT_Time )
            {
                FCT_Timer2.ClearTimer();
                Inverter.Off_FCT();
                FCT_TimeRunInSec = 0;
                FCT_Status.bit.CancelTestCmd = 0;
                FCT_Status.bit.RectifierControlOn = 0;
                FCTRunState = FCT_IDLE_STATE;
            }
        }
        else //CancelTest
        {
             // Record exit reason
            FCT_Error |= 0x0100;

            FCT_Timer1.ClearTimer();
            FCT_Timer2.ClearTimer();
            Inverter.Off_FCT();
            FCT_TimeRunInSec = 0;
            FCT_Status.bit.RectifierTestFail = 1;
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Status.bit.RectifierControlOn = 0;
            FCTRunState = FCT_IDLE_STATE;
        }
     }
}

// ********************************************************************************************************
// *
// * Function: FCT_LinePMDischargeState(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: discharge DC link 
// *
// ********************************************************************************************************
void FCTStateControl::FCT_LinePMDischargeState(void)
{
    if( !FCT_Status.bit.LinePMDischargeOn ) //Check Enter condition
    {
        if( !FCT_DcLinkCheck_BothLowerThan( 50.0 ) ) //  Any Higher than 50V
        {
            FCT_Timer2.ClearTimer();
            FCT_Timer1.IncTimer();
            
            if( FCT_Timer1.TimerValue() == WAIT_500_MS )
            {
                FCT_Timer1.ClearTimer();
                Inverter.OnOpenLoop_FCT(); // No RMS
                FCT_Status.bit.LinePMDischargeOn = 1;
            }
        }
        else
        {
            if( FCT_DcLinkCheck_BothLowerThan( 50.0 ) )
            {
                FCT_Error |= 0x0002;
            }
            FCT_Status.bit.LinePMDischargeOn = 0;
            Inverter.Off_FCT();
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Result = FCT_PASS;
            FCTRunState = FCT_IDLE_STATE;
        }
    }
    else //Discharge on
    {
        if( !FCT_Status.bit.CancelTestCmd )
        {
            FCT_Timer2.IncTimer();
            FCT_TimeRunInSec = ( FCT_Timer2.TimerValue() / ONE_SECOND );
            if( FCT_Timer2.TimerValue() >= ONE_SECOND * FCT_Time ) //Discharge for set seconds
            {
                FCT_Timer2.ClearTimer();
                if( FCT_DcLinkCheck_BothLowerThan( 50.0 ) )
                {
                    FCT_Result = FCT_PASS;
                }
                else
                {
                    FCT_Error |= 0x4000;
                    FCT_Result = FCT_DISCHARGE_FAILED;
                }
                FCT_TimeRunInSec = 0;
                FCT_Status.bit.LinePMDischargeOn = 0;
                Inverter.Off_FCT();
                FCTRunState = FCT_IDLE_STATE;
            }
        }
        else  // CancelTestCmd
        {
            // Record exit reason
            FCT_Error |= 0x0100;
            
            FCT_TimeRunInSec = 0;
            FCT_Status.bit.LinePMDischargeOn = 0;
            Inverter.Off_FCT();
            FCT_Status.bit.CancelTestCmd = 0;
            FCT_Result = FCT_DISCHARGE_FAILED;
            FCTRunState = FCT_IDLE_STATE;        
        }
    }
}

// ********************************************************************************************************
// *
// * Function: EnableFCT(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: enable FCT mode and clear all bits, timers 
// *
// ********************************************************************************************************
void FCTStateControl::EnableFCT( void )
{
    FCT_Status.bit.FCTModeOn = 1;
    MCUStateMachine.ClearAllCommands();
    FCTRunState = FCT_IDLE_STATE;
    //PreRMSVoltageNormFactor = 0;
    FCT_Time = 10;
    FCT_Phase = 0;
    FCT_Error = 0;
    FCT_DcLinkRef = 0;
    FCT_TimeRunInSec = 0;
    ClearAllCmd();
    ClearResult();
    EPOEnabledStore = EPOEnabled;
    EPOEnabled = 0; // Disable EPO function when in FCT mode
    
    // close all relays in advance, input, output, battery voltage should all be zero.
    DSPOutRegister.GpoB.bit.InputRelay_L1 = 1;
    DSPOutRegister.GpoB.bit.InputRelay_L23 = 1;
    MCUStateMachine.SetInverterRelay(RELAY_CLOSED);
    BatteryConverter.SetBatteryRelay( RELAY_CLOSED );
}

// ********************************************************************************************************
// *
// * Function: DisableFCT(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: disable FCT, clear all bits and timer 
// *
// ********************************************************************************************************
void FCTStateControl::DisableFCT( void )
{
    FCT_ReceriveExitCmd = true;
    //PreRMSVoltageNormFactor = 0;
    FCT_Time = 10;
    FCT_Phase = 0;
    FCT_Error = 0;
    FCT_DcLinkRef = 0;
    FCT_TimeRunInSec = 0;
    ClearAllCmd();
    ClearResult();
    FCTRunState = FCT_IDLE_STATE;
    
    //exit FCT, open all relay
    DSPOutRegister.GpoB.bit.InputRelay_L1 = 0;
    DSPOutRegister.GpoB.bit.InputRelay_L23 = 0;
    MCUStateMachine.SetInverterRelay(RELAY_OPEN);
    BatteryConverter.SetBatteryRelay( RELAY_OPEN );
}

// ********************************************************************************************************
// *
// * Function: ClearAllCmd(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: clear all fct commands 
// *
// ********************************************************************************************************
void FCTStateControl::ClearAllCmd(void)
{
    FCT_Status.bit.ChargerTestCmd = 0;
    FCT_Status.bit.InverterTestCmd = 0;
    FCT_Status.bit.BoostTestCmd = 0;
    FCT_Status.bit.RectifierTestCmd = 0;
    FCT_Status.bit.LineDischargeCmd = 0;
}

// ********************************************************************************************************
// *
// * Function: ClearResult(void);
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: clear all fct results 
// *
// ********************************************************************************************************
void FCTStateControl::ClearResult(void)
{
    FCT_Status.bit.BoostTestFail = 0;
    FCT_Status.bit.ChargerTestFail = 0;
    FCT_Status.bit.InverterTestFail = 0;
    FCT_Status.bit.RectifierTestFail = 0;
    FCT_Result = FCT_NOT_START;
    FCT_Error = 0;
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLinkCheck_BothHigherThan;
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if both positive and negative link voltage are higher than level
// *
// * Description: check DC link voltage
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_DcLinkCheck_BothHigherThan(float volt)
{
    bool state = false;

    if( ( DCLinkVoltagePositive.FastFiltered > volt ) &&
        ( DCLinkVoltageNegative.FastFiltered < -volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLinkCheck_AllHigherThan
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if sum of ¡Àlink voltage > level
// *
// * Description: check DC link voltage
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_DcLinkCheck_AllHigherThan(float volt)
{
    bool state = false;

    if( ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered ) > volt )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLinkCheck_BothLowerThan;
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if both positive and negative link voltage are lower than level
// *
// * Description: check DC link voltage 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_DcLinkCheck_BothLowerThan(float volt)
{
    bool state = false;

    if( ( DCLinkVoltagePositive.FastFiltered < volt ) &&
        ( DCLinkVoltageNegative.FastFiltered > -volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLinkCheck_AllLowerThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if sum of ¡Àlink voltage < level
// *
// * Description: check DC link voltage 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_DcLinkCheck_AllLowerThan(float volt)
{
    bool state = false;

    if( ( DCLinkVoltagePositive.FastFiltered - DCLinkVoltageNegative.FastFiltered ) < volt )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLinkMarginCheck_LowerThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if abs( ¡À link unbalance error ) < limit 
// *
// * Description: disable FCT, clear all bits and timer 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_DcLinkMarginCheck_LowerThan(float volt)
{
    bool state = false;

    if( ( DCLinkVoltagePositive.FastFiltered + DCLinkVoltageNegative.FastFiltered < volt ) &&
        ( DCLinkVoltagePositive.FastFiltered + DCLinkVoltageNegative.FastFiltered > -volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_Battery2Check_HigherThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if battery sample value > low limit
// *
// * Description: verify if battery voltage sample value is in normal range
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_Battery2Check_HigherThan(float volt)
{
    bool state = false;

//	    if( RawAdcDataPtr->st.BatteryVoltageChgNeg > volt )
	if((RawAdcDataPtr->st.BatteryVoltageChgPos - RawAdcDataPtr->st.BatteryVoltageChgNeg) > volt )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_Battery2Check_LowerThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if battery sample value < high limit
// *
// * Description: verify if battery voltage sample value is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_Battery2Check_LowerThan(float volt)
{
    bool state = false;

    if((RawAdcDataPtr->st.BatteryVoltageChgPos - RawAdcDataPtr->st.BatteryVoltageChgNeg) < volt )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterVoltageCheck_AllHigherThan(void);
// *
// * Parms Passed   :   volt  --       nomial voltage
// *                    limit --       percent of error
// *
// * Returns        :   true if all 3 phase voltage are in range
// *
// * Description: verify if 3 phase voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_RecVoltageCheck(float volt)
{
    bool state = false;
    const float limit = 0.15;
    float highlimit = volt *( 1 + limit );
    float lowlimit = volt *( 1 - limit );
    
    if( ( volt > 0 ) && ( limit > 0 ) && ( limit < 1 ) )
    {

        if( ( UtilityVoltageRMS.FilteredRMS.phA > lowlimit ) &&
            ( UtilityVoltageRMS.FilteredRMS.phB > lowlimit ) &&
            ( UtilityVoltageRMS.FilteredRMS.phC > lowlimit ) &&
            ( UtilityVoltageRMS.FilteredRMS.phA < highlimit ) &&
            ( UtilityVoltageRMS.FilteredRMS.phB < highlimit ) &&
            ( UtilityVoltageRMS.FilteredRMS.phC < highlimit ) )
        {
            state = true;
        }
    }
    
    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterVoltageCheck_AllHigherThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if all inv 3 phase > low limit
// *
// * Description: verify if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_InverterVoltageCheck_AllHigherThan(float volt)
{
    bool state = false;

    if( ( InverterVoltageRMS.FilteredRMS.phA > volt ) &&
        ( InverterVoltageRMS.FilteredRMS.phB > volt ) &&
        ( InverterVoltageRMS.FilteredRMS.phC > volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterVoltageCheck_AllLowerThan(void);
// *
// * Parms Passed   :   volt
// * 
// * Returns        :   true if battery sample value < high limit
// *
// * Description: check if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_InverterVoltageCheck_AllLowerThan(float volt)
{
    bool state = false;

    if( ( InverterVoltageRMS.FilteredRMS.phA < volt ) &&
        ( InverterVoltageRMS.FilteredRMS.phB < volt ) &&
        ( InverterVoltageRMS.FilteredRMS.phC < volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterVoltageCheck_AnyHigherThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   true if any phase inv voltage is too high
// *
// * Description: check if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_InverterVoltageCheck_AnyHigherThan(float volt)
{
    bool state = false;

    if( ( InverterVoltageRMS.FilteredRMS.phA > volt ) ||
        ( InverterVoltageRMS.FilteredRMS.phB > volt ) ||
        ( InverterVoltageRMS.FilteredRMS.phC > volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_InverterVoltageCheck_AnyLowerThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   True if any phase inv voltage is too low
// *
// * Description: check if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_InverterVoltageCheck_AnyLowerThan(float volt)
{
    bool state = false;

    if( ( InverterVoltageRMS.FilteredRMS.phA < volt ) ||
        ( InverterVoltageRMS.FilteredRMS.phB < volt ) ||
        ( InverterVoltageRMS.FilteredRMS.phC < volt ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_INVCurrent_Check_AnyHigherThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   True if any phase inv voltage is too high
// *
// * Description: check if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_INVCurrent_Check_AnyHigherThan(float current)
{
    bool state = false;

    if( ( OutputCurrentRMS.FilteredRMS.phA > current ) ||
        ( OutputCurrentRMS.FilteredRMS.phB > current ) ||
        ( OutputCurrentRMS.FilteredRMS.phC > current ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_INVCurrent_Check_AnyHigherThan(void);
// *
// * Parms Passed   :   volt
// *
// * Returns        :   True if any phase inv voltage is too high
// *
// * Description: check if inverter voltage is in normal range 
// *
// ********************************************************************************************************
inline bool FCTStateControl::FCT_RecCurrent_Check_AnyHigherThan(float current)
{
    bool state = false;

    if( ( UtilityCurrentRMS.FilteredRMS.phA > current ) ||
        ( UtilityCurrentRMS.FilteredRMS.phB > current ) ||
        ( UtilityCurrentRMS.FilteredRMS.phC > current ) )
    {
        state = true;
    }

    return( state );
}

// ********************************************************************************************************
// *
// * Function: FCT_DcLink_Softstart(void);
// *
// * Parms Passed   :   target_volt
// *
// * Returns        :   soft start result -- STATE_IN_PROGRESS means DC link not reach target yet
// *                                         STATE_SUCCESS measn DC link reaches target
// *
// * Description: increase dc link ref slowly to target voltage
// *
// ********************************************************************************************************
uint16_t FCTStateControl::FCT_DcLink_Softstart( float target_volt )
{
    FCT_Timer3.IncTimer();
    
    if( FCT_Timer3.TimerValue() == WAIT_100_MS ) // Increase 10 per second until target_volt
    {
        FCT_Timer3.ClearTimer();

        if( FCT_DcLinkRef < target_volt )
        {
            FCT_DcLinkRef += 1.0;
            BatteryConverter.SetDCLinkVoltage( FCT_DcLinkRef );
        }
        else
        {
            BatteryConverter.SetDCLinkVoltage( target_volt );
            return( STATE_SUCCESS );
        }
    }

    return( STATE_IN_PROGRESS );
}
