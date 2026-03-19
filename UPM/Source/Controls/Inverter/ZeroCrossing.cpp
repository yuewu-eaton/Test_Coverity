// ********************************************************************
// *            SWI_functions.c
// ********************************************************************
// ********************************************************************
// * 
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   SWI_functions.c
// *                                                                      
// *     DESCRIPTION: SWIs that are not posted from periodic SWI
// *                  
// *                  
// *     ORIGINATOR:  Kevin VanEyll                                         
// *                                                                      
// *     DATE:        5/7/2004                                            
// *                                                                      
// *     HISTORY:     See CVS history                                                    
// ********************************************************************

// ********************************************************************
// *            INCLUDE FILES                                                                           
// ********************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ProcessAdc.h"
#include "InverterControl.h"
#include "adc.h"
#include "Alarms.h"
#include "F28335Port.h"
#include "ACPowerMeter.h"
#include "ProcessAdc.h"
#include "ACMeter.h"
#include "BypassInterface.h"
#include "MCUState.h"
#include "RectifierStateControl.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
#include <math.h>

// ********************************************************************
// *            LOCAL FUNCTION PROTOTYPES
// ********************************************************************
extern "C" {
    void InverterZeroCrossingSWI( void );
    void UtilityZeroCrossingSWI( void );
    void BypassZeroCrossingSWI( void );
} 


// ********************************************************************
// *            GLOBAL/PUBLIC DATA
// ********************************************************************
extern uint16_t  THDiStartFlag;
extern uint16_t  THDiCount;	
extern uint16_t  LightloadFlag;
extern float  KvfwRec; 
extern float DynamicLoadPercentRec;
extern float DynamicLoadPercent;
extern uint16_t  FlagUnblanceLoad;

// ********************************************************************
// *            LOCAL/PRIVATE DATA
// ********************************************************************

// ********************************************************************
// *            Local prototypes
// ********************************************************************

// ********************************************************************
// *                                                                                                                     
// * Function:  InverterZeroCrossingSWI() 
// *
// * Purpose:   Inverter zero cross functions
// *                    
// *                    
// * Parms Passed:  None
// * Returns:       Nothing                                                                          
// *                                                                                                                     
// *
// * Exceptions/Limitations: 
// ********************************************************************
void InverterZeroCrossingSWI(void)
{
    if ( CaptureControl.Start )
    {
        CaptureControl.Start = false;
        CaptureControl.Trigger = true;
    }   
    
    CheckCurrentLimits();
    
    InverterVoltageRMS.CalculateRMS();
    InverterCurrentRMS.CalculateRMS();
    
    OutputVoltageRMS.CalculateRMS();
    OutputCurrentRMS.CalculateRMS();
    
    Inverter.InverterLineCycleTask();
    CalculateOutReactiveCurrent();
    
    if ( Inverter.GetStatus().bit.InverterOn )
    {
        InverterPower.CalculatePower( InverterVoltageRMS.RawRMS, InverterCurrentRMS.RawRMS );
    }
    else
    {
        InverterPower.ClearPower();
    }
    
    
    switch ( MCUStateMachine.GetState() )
    {
        case ONLINE_STATE:
        case EASY_CAPACITY_TEST_STATE:
        case ESS_MODE_STATE:
            LoadPower.CalculatePower( OutputVoltageRMS.RawRMS, OutputCurrentRMS.RawRMS );
            break;

        case BYPASS_STATE:
        case BYPASS_STANDBY_STATE:
            LoadPower.CalculatePower( OutputVoltageRMS.RawRMS, BypassCurrentRMS.RawRMS );
            break;
			
        default:
            LoadPower.ClearPower();
            break;
    }
    
    //send output frequency of one UPS, all ups freeze at this frequency when CAN fail.
    if ( ParallelCan.ParallelStatus.bit.Master &&
         ( !NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR) ) &&
		 ( !ParallelCan.ParGlobalOrData.SyncStatus.bit.CanBusFailed) &&
		 ( OutputPLL.IsSourcePresent() ) )
    {
        static float freq_filter = OutputPLL.SineRef.Frequency;
        uint16_t temp_frequency;
        
        freq_filter = 0.99 * freq_filter + 0.01 * OutputPLL.SineRef.Frequency;
        temp_frequency = (uint16_t)(freq_filter * 1000.0 + 0.5);
        ParallelCan.TransmitBroadcastPacket(pcan::master_output_freq, temp_frequency, 0, 0, 0, true);
        ParallelCan.MasterOutputFrequency = float(temp_frequency) / 1000.0;
    }
}       // end of InverterZeroCrossingSWI()

// ********************************************************************
// *                                                                                                                     
// * Function:  UtilityZeroCrossingSWI() 
// *
// * Purpose:   Utility zero cross functions
// *                    
// *                    
// * Parms Passed:  None
// * Returns:       Nothing                                                                          
// *                                                                                                                     
// *
// * Exceptions/Limitations: 
// ********************************************************************
void UtilityZeroCrossingSWI( void )
{
    UtilityVoltageRMS.CalculateRMS();
    UtilityCurrentRMS.CalculateRMS();
    
    if ( Rectifier.GetStatus().bit.RectifierOnNormal )
    {
        RectifierPower.CalculatePower( UtilityVoltageRMS.RawRMS, UtilityCurrentRMS.RawRMS );
    }
    else
    {
        RectifierPower.ClearPower();
    }        

	DynamicLoadPercent = ((LoadPower.TotalPower.phA+LoadPower.TotalPower.phB+LoadPower.TotalPower.phC) / float(OutputkVARating));	//1cnt->1%

	//Rec Dynamic Load Check
	DynamicLoadPercentRec = (RectifierPower.TotalPower.phA+RectifierPower.TotalPower.phB+RectifierPower.TotalPower.phC) / (float(OutputkVARating)*1.05f);	//1cnt->1%

	if(THDiStartFlag == 1)
	{
		if(DynamicLoadPercentRec < float(40.0f))
		{
			THDiStartFlag = 0;
		}
	}
	else
	{
		if(DynamicLoadPercentRec > float(45.0f))
		{
			THDiCount++;
			if(THDiCount >=2)	
			{
				THDiCount = 0;
				THDiStartFlag = 1;
			}			
		}
		else
		{
			THDiCount = 0;
		}
	}		
	//end		

	//Rec Unbalance Load Check
	float temp = 0.0f;
	temp = float(OutputkVARating)*10.0f;		//OutputkVARating*100 * 10%
	if((fabs(LoadPower.TotalPower.phA-LoadPower.TotalPower.phB) > temp) ||			
		(fabs(LoadPower.TotalPower.phB-LoadPower.TotalPower.phC) > temp) ||
		(fabs(LoadPower.TotalPower.phA-LoadPower.TotalPower.phC) > temp))
	{
		FlagUnblanceLoad = true;
	}
	else
	{
		temp = float(OutputkVARating)*7.0f;		//OutputkVARating*100 * 7%
		if((fabs(LoadPower.TotalPower.phA-LoadPower.TotalPower.phB) < temp) &&			
			(fabs(LoadPower.TotalPower.phB-LoadPower.TotalPower.phC) < temp) &&
			(fabs(LoadPower.TotalPower.phA-LoadPower.TotalPower.phC) < temp))
		{
			FlagUnblanceLoad = false;
		}
	}


	//end
}       // end of UtilityZeroCrossingSWI()

// ********************************************************************
// *                                                                                                                     
// * Function:  BypassZeroCrossingSWI() 
// *
// * Purpose:   Bypass zero cross functions
// *                    
// *                    
// * Parms Passed:  None
// * Returns:       Nothing                                                                          
// *                                                                                                                     
// *
// * Exceptions/Limitations: 
// ********************************************************************
void BypassZeroCrossingSWI( void )
{
    BypassVoltageRMS.CalculateRMS();
    BypassVoltage2RMS.CalculateRMS();
    BypassCurrentRMS.CalculateRMS();
    ChassisVoltageRMS.CalculateRMS();
    
    if ( BYPASS_FIRE_STATE == BypassState().GetMonitoredBypassState() )
    {
        BypassPower.CalculatePower( BypassVoltageRMS.RawRMS, BypassCurrentRMS.RawRMS );
    }
    else
    {
        BypassPower.ClearPower();
    }                
    
}       // end of BypassZeroCrossingSWI()

// ********************************************************************
// *            END OF SWI_functions.c   
// ********************************************************************
