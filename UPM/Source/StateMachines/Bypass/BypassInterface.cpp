// ******************************************************************************************************
// *            BypassInterface.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// *    THIS INFORMATION IS PROPRIETARY TO EATON CORPORATION
// *
// *    Copyright (c) 2010 Eaton Corporation, ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BypassInterface.c
// *
// *    DESCRIPTION: Static functions of the bypass interface.
// *
// *    HISTORY: See SVN history for author and revision history.
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "BypassInterface.h"
#include "CriticalSection.h"
#include "Alarms_AC.h"
#include "NB_Funcs.h"
#include "InvSync.h"
#include "MCUState.h"

// *********************************************************************************************************
// *        CONSTANTS AND GLOBALS
// *********************************************************************************************************
uint16_t BypACUVActiveTime;    // EE var
// *********************************************************************************************************
// *        FUNCTIONS
// *********************************************************************************************************

// ***********************************************************************
// *
// *    FUNCTION: CheckBypassVoltageAlarms 
// *
// *    DESCRIPTION: Checks fast bypass ACOV/ACUV, placed in this file
// *                 due to inline restrictions. run at 2.5kHz
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void BypassInterface::CheckBypassVoltageAlarms( void )
{
    const float BypassVoltageHysteresis = 0.025;            // 2.5%
    static uint16_t BypACUVCounter = 0;
    
    // fast bypass ACOV check   AC_AlarmsStatus.bit.BypassRMSOV
    NB_DebounceAndQue_Hysterisis( UPM_NB_BYPASS_AC_OVER_VOLTAGE,
                ( BypassPLL.SourceNominalDQO.Sd > GetBypassDQHighLimit() || BypassRMSOV.GetState()),
                ( BypassPLL.SourceNominalDQO.Sd < ( GetBypassDQHighLimit() - BypassVoltageHysteresis ) && !BypassRMSOV.GetState())
    );
    
    // fast bypass ACUV check
    if( ESSEnabled && NB_GetNodebit( UPM_NB_ECO_INSTALLED ) )
    {
    	NB_DebounceAndQue_Hysterisis( UPM_NB_BYPASS_AC_UNDER_VOLTAGE,
					( BypassPLL.SourceNominalDQO.Sd < GetBypassDQLowLimit_ESS()  || BypassRMSUV.GetState() ),
					( BypassPLL.SourceNominalDQO.Sd > ( GetBypassDQLowLimit_ESS() + BypassVoltageHysteresis ) && !BypassRMSUV.GetState() )
		);
    }
    else
    {
    	if( !NB_GetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE ) )
    	{
    		if( AcuvCheck( BypassPLL.SourceNominalDQO.Sd, GetBypassDQLowLimit(), BypACUVActiveTime, &BypACUVCounter ) ||
    			BypassRMSUV.GetState() )
    		{
    			NB_SetNodebit( UPM_NB_BYPASS_AC_UNDER_VOLTAGE, true );
    		}
    	}
    	else
    	{
    		BypACUVCounter = 0;
    		if( BypassPLL.SourceNominalDQO.Sd > ( GetBypassDQLowLimit() + BypassVoltageHysteresis ) && !BypassRMSUV.GetState() )
			{
				NB_DebounceAndQue( UPM_NB_BYPASS_AC_UNDER_VOLTAGE, false );
			}
    	}
    }

    if(BypassPLL.SourceNominalDQO.Sd < 0.47)
    {
        BypassDQLoss.Debounce(true);
    }
    else
    {
        BypassDQLoss.Debounce(false);
    }
}

// ***********************************************************************
// *
// *    FUNCTION: EECalcBypassLimits 
// *
// *    DESCRIPTION: ee function, updates bypass params from ee data
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void BypassInterface::EECalcBypassLimits( const EE_ID* ee, const uint16_t* data )
{
	CriticalSection enter;
	
    switch ( ee->paramNum )
    {
        case PARAM_BypACOVLevel:  
            BypassRMSVoltageHighLimit = 1.0 + ( (float)(*data) / 100.0 );
            BypassDQVoltageHighLimit = BypassRMSVoltageHighLimit + ( (float)BypVoltMaxLimitFast / 100.0 );
            BypVoltMaxLimit = *data;
            break;
         case PARAM_BypACOVFastLevel:
            BypassDQVoltageHighLimit = BypassRMSVoltageHighLimit + ( (float)(*data) / 100.0 );
            BypVoltMaxLimitFast = *data;
        	break;
        case PARAM_BypACUVLevel:    
            BypassRMSVoltageLowLimit = 1.0 - ( (float)(*data)/ 100.0 );
            BypassDQVoltageLowLimit = BypassRMSVoltageLowLimit - ( (float)BypVoltMinLimitFast/ 100.0 );
            BypassDQVoltageLowLimit_ESS = BypassRMSVoltageLowLimit - ( (float)BypVoltMinLimitFast_ESS/ 100.0 );
            BypVoltMinLimit = *data;
            break;
		case PARAM_BypACUVFastLevel:
			BypassDQVoltageLowLimit = BypassRMSVoltageLowLimit - ( (float)(*data)/ 100.0 );
			BypVoltMinLimitFast = *data;
			break;
        case PARAM_InvSyncLim:
            BypFreqDeviation = *data;
            break;    

        case PARAM_ATBFixedDelay:
            if( (*data) >= 10 )
            {
                //atb_gap_cycles = data*0.5102.
                MCUStateMachine.atb_gap_cycles = (uint32_t)(*data)*ADCFrequency/(2*10*17600L);
            }
            else
            {
                //atb_gap_cycles = 10 * 0.5102 = 5. Thus: gap = 5*196us = 0.98ms
                MCUStateMachine.atb_gap_cycles = (uint32_t)(10)*ADCFrequency/(2*10*17600L);
            }
            break;

        case PARAM_BypACUVActiveTime:
        	BypACUVActiveTime = (uint16_t)( (*data) * 2.5 );
			break;

        case PARAM_ESSBypACUVFastLevel:
			BypassDQVoltageLowLimit_ESS = BypassRMSVoltageLowLimit - ( (float)(*data)/ 100.0 );
			BypVoltMinLimitFast_ESS = *data;
			break;

        default:
            break;
    }
}

float   BypassInterface::BypassDQVoltageHighLimit = 0;
float   BypassInterface::BypassDQVoltageLowLimit = 0;
float   BypassInterface::BypassDQVoltageLowLimit_ESS = 0;
float   BypassInterface::BypassRMSVoltageHighLimit = 0;
float   BypassInterface::BypassRMSVoltageLowLimit = 0;
int16_t BypassInterface::BypFreqDeviation = 0;

// ******************************************************************************************************
// *            End of BypassInterface.cpp
// ******************************************************************************************************
