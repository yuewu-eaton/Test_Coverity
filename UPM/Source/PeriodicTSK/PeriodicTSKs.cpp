// ********************************************************************
// *            PeriodicTaskFuncs.c
// ********************************************************************
// ********************************************************************
// * 
// * This information is proprietry to Eaton Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 2005 Eaton Corporation                       
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   PeriodicTaskFuncs.c
// *                                                                      
// *     DESCRIPTION: 
// *                  
// *     ORIGINATOR:  Kevin VanEyll                                         
// *                                                                      
// *     DATE:        7/15/2005                                            
// *                                                                      
// *     HISTORY:     See CVS history                                                    
// ********************************************************************

// ********************************************************************
// *            Include files                                                                           
// ********************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "Adc.h"
#include "Meters.h"
#include "BypassInterface.h"
#include "Rtc.h"
#include "IOexpansion.h"      
#include "Alarms.h"           
#include "DQPhaseLockLoop.h"
#include "Alarms_AC.h"
#include "Fan.h"
#include "InverterControl.h"
#include "Rtc.h"              
#include "Spi_Driver.h"       
#include "Spi_Task.h"         
#include "Thermal.h"          
#include "MCUState.h"
#include "NB_Config.h"
#include "NB_Funcs.h"
#include "Abm.h"
#include "ParallelCan.h"
#include "RectifierStateControl.h"
#include "BatteryStateControl.h"
#include "IdleFuncs.h"
#include "InternalCan.h"
#include "BTR.h"
#include "InvSync.h"
#include "RectifierControl.h"
#include "Eeprom_map.h"
#include "EncryptionVry.h"
#include "I2c_Driver.h"
#include "debugger.h"
#include "PeriodicTSKs.h"
#include "FCTState.h"
#include <string.h>
#include <algorithm>

// Global data for debugger
float PortStackPercent = 0.0f;
float SPIControlStackPercent = 0.0f;
float HistQueStackPercent = 0.0f;
float PeriodicTaskPercent = 0.0f;
float HWIStackPercent = 0.0f;
float PCanStackPercent = 0.0f;
float ICanRxStackPercent = 0.0f;
float HeapPercent = 0.0f;
float PercentIdle = 0.0f;
extern float FuncRunTimeUs[6];
extern bool PWMSyncReceived;

// Error counters for periodic task overruns.
uint16_t Period5msOverruns = 0;
uint16_t Period20msOverruns = 0;
uint16_t Period100msOverruns = 0;
uint16_t Period500msOverruns = 0;
uint16_t Period1sOverruns = 0;
uint16_t Period20sOverruns = 0;
uint16_t Period1minOverruns = 0;

// ********************************************************************
// *            Local prototypes
// ********************************************************************
void TSK_5msFunctions( void );
void TSK_20msFunctions( void );
void TSK_100msFunctions( void );
void TSK_500msFunctions( void );
void TSK_1sFunctions( void );
void TSK_20sFunctions( void );
void TSK_1minFunctions( void );
void CheckStacks( void );
float CalcGlobalStackPercent( void );
void DebounceCSBNodebits( void );
void CheckSystemType( void );
void EncryptionVry( void );
void CheckVFModeReady(void);

#define PRD_TICK        5
#define VF_MODE_UPDATE_FREQ  2  //1s/500ms = 2
// ********************************************************************
// *            Local/private data
// ********************************************************************
extern SEM_Obj PeriodicTaskSem;

uint32_t LastCPUTimer = 0;


// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_PeriodicTask() 
// *
// * Purpose:   Keeps track of and calls periodic tasks
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: Posted from SWI every 20ms. Calls 20ms, 100ms, 500ms
// *              1s and 1min task functions as required.
// *                    
// ********************************************************************
extern "C" {
void TSK_PeriodicTask(void)
{
    uint16_t TSK_IntervalCounter = 0;
    uint16_t TSK_20sEarlyCounts = 0;
    
    enum intervals
    {
        INTERVAL_20MS  = 20/PRD_TICK,
        INTERVAL_100MS = 100/PRD_TICK,
        INTERVAL_500MS = 500/PRD_TICK,
        INTERVAL_1SEC  = 1000/PRD_TICK,
        INTERVAL_20SEC = 20000/PRD_TICK,
        INTERVAL_1MIN  = 60000/PRD_TICK
    };
    
    // Phase shift these tasks such that they do not get executed
    // in the same time slice as the 20ms task, or with each other.
    // This spreads out the load somewhat to ensure that we aren't trying to
    // execute everything in the same time slice.
    // The one and 20 second tasks are so short that they may safely be
    // scheduled with each other.
    enum phases {
        PHASE_20MS = 0,
        PHASE_100MS = 1,
        PHASE_500MS = 2,
        PHASE_1SEC = 3,
        PHASE_20SEC = PHASE_1SEC
    };

    for ( ; ; )
    {
        if ( SEM_pend( &PeriodicTaskSem, SYS_FOREVER ) )
        {
            TSK_IntervalCounter++;

            // run every time
            TSK_5msFunctions();

            if (SEM_count(&PeriodicTaskSem) != 0)
            {
                Period5msOverruns++;
            }
                // run every 4th pass at 5ms
            if ((TSK_IntervalCounter % INTERVAL_20MS) == PHASE_20MS)
            {
                TSK_20msFunctions();
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period20msOverruns++;
                }
            }

            if ((TSK_IntervalCounter % INTERVAL_100MS) == PHASE_100MS)
            {
                // every 20th pass at 5ms
                TSK_100msFunctions();
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period100msOverruns++;
                }
            }

            if ((TSK_IntervalCounter % INTERVAL_500MS) == PHASE_500MS)
            {
                // every 100th pass at 5ms
                TSK_500msFunctions();
                if (TSK_20sEarlyCounts < 5)
                {
                    // Run the 20s tasks every 500 ms at startup, to avoid a
                    // potential race condition between the CSB and UPM startup
                    TSK_20sEarlyCounts++;
                    TSK_20sFunctions();
                }
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period500msOverruns++;
                }
            }

            if ((TSK_IntervalCounter % INTERVAL_1SEC) == PHASE_1SEC)
            {
                // every 200th pass at 5ms
                TSK_1sFunctions();
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period1sOverruns++;
                }
            }
            
            if ((TSK_IntervalCounter % INTERVAL_20SEC) == PHASE_20SEC)
            {
                // every 4000th pass at 5ms
                TSK_20sFunctions();
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period20sOverruns++;
                }
            }

            if (TSK_IntervalCounter >= INTERVAL_1MIN)
            {
                // 12000th pass, reset counter
                TSK_1minFunctions();
                TSK_IntervalCounter = 0;
                if (SEM_count(&PeriodicTaskSem) != 0)
                {
                    Period1minOverruns++;
                }
            }
            
        }

    }
}
}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_5msFunctions() 
// *
// * Purpose:   Tasks to perform every 5ms
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ********************************************************************
void TSK_5msFunctions(void)
{
    RTC_IncSysTime();
    ParallelSystem5Msec();
    CheckOverloadMedium();
    MCUStateMachine.Run5Msec();
    InternalCan.DrainTxFifo();
//		LoadshareThetaCompen();

    if (AOLEnable)
    {
        CheckAdapOverloadCapacity();
    }
    else
    {
        AdaptOverloadCapacityOff = true;
        NB_DebounceAndQue ( UPM_NB_ADAPTIVE_OVERLOAD_CAPACITY_OFF, false, 0 );
    }
}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_20msFunctions() 
// *
// * Purpose:   Tasks to perform every 20ms
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ********************************************************************
void TSK_20msFunctions(void)
{
    Rectifier.Rectifier20msTask();
    CheckMediumACAlarms();
    CheckAlarms20ms();
    ParallelSystem20Msec();
    
    //Jacob/20130814/Merge 120K
    CheckSystemChanged();
//	    CheckSiteWiring();
	//Add Bat boost, 20ms vbus_target loop 
	BatteryConverter.BatConverter20msTask();
	//Slowly reduce the drop power so that the actual power take effect.
	if( InitDropPower < 55.0f )
	{
		InitDropPower = 0.0f;
	}
	else
	{	
		InitDropPower -= 50.0f;
	} 

}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_100msFunctions() 
// *
// * Purpose:   Tasks to perform every 100ms
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ******************************************************************** 
void TSK_100msFunctions(void)
{
    if( !FrequencyConverterMode )
    {
        BypassState().Task100ms();
    }

    CalTempCompensateVPC( ParallelCan.MaxBatTempInCommon );
    Abm().Run();
    CalculateThermal();
    CheckSlowACAlarms();
    ParallelSystem100Msec();
    MCUStateMachine.Run100Msec();
    CheckTemperatureAlarms();
    CheckFuseFailures();
    Rectifier.Rectifier100msTask();
    FanSpeedCheck_pld();  // include PM fan speed and STS fan state check
    //Make it more clear about the decryption status
    if(EEStatusBits.bit.DecryptionFailed)
    {
        ToggleLED();
    }
	
    if(MCUStateMachine.GetState() != INITIALIZATION_STATE)
    {
		if(!EnableParalPwmSync)		//no paral pwm sync, only inner pwm sync
		{
			if(MyUPMNumber != 0)	//
			{
				NB_DebounceAndQue( UPM_NB_LOSS_OF_PWM_SYNC , !PWMSyncReceived );
				PWMSyncReceived = false;
			}
			else
			{
				NB_DebounceAndQue( UPM_NB_LOSS_OF_PWM_SYNC , false);
			}
		}
		else
		{
			//master no detect
			if(!ParallelCan.ParallelStatus.bit.Master)
			{
		    	NB_DebounceAndQue( UPM_NB_LOSS_OF_PWM_SYNC , !PWMSyncReceived );
		    	PWMSyncReceived = false;
			}
			else
			{
		    	NB_DebounceAndQue( UPM_NB_LOSS_OF_PWM_SYNC , false);
			}
		}
    }	
}

void CopyTime(uint16_t *data)
{
    memcpy(RTC_BcdTime.w, data, 7);
}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_500msFunctions() 
// *
// * Purpose:   Tasks to perform every 500ms
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ********************************************************************
void TSK_500msFunctions(void)
{
    EncryptionVry();
    if(!EEStatusBits.bit.DecryptionFailed)
    {
        ToggleLED();
    }
    if( NB_GetNodebit( UPM_NB_BATTERY_INSTALLED ) )
    {
    	BatteryConverter.DebounceCharger();
    }
    ProcessACScreenMeters();
    InternalCan.SendUPMMeters();
    BTR.CheckAlarms();
    InternalCan.SendUPMNodebits();
    ParallelSystem500Msec();
    InternalCan.SendUpsNumber();
}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_1sFunctions() 
// *
// * Purpose:   Tasks to perform every 1s
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ******************************************************************** 
void TSK_1sFunctions(void)
{   
    float maxSinglePhaseLoad = std::max( ScreenMeters.PercentLoad.phA, std::max( ScreenMeters.PercentLoad.phB, ScreenMeters.PercentLoad.phC ) );
    uint16_t OutputKVA = OutputkVARating * NumOfModule;
    CheckStacks();

    if ( !NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT ) )
    {
        NB_DebounceAndQue( UPM_NB_RECTIFIER_PHASE_ROTATION, Rectifier.UtilityPLL.IsPhaseRotationError() );
    }
    else
    {
        NB_DebounceAndQue( UPM_NB_RECTIFIER_PHASE_ROTATION, false );
    }
    
    if ( EEStatusBits.bit.EEDataInitialized )
    {
        EEStatusBits.bit.ServiceRequired = ServiceRequired != 0;
        EEStatusBits.bit.SetupRequired = SetupRequired != 0;
        
        // Alarm attempts to put revision 5 hardware in internal or external parallel
//	        if( ( ExpansionInputReg.bit.RevID == Rev5 ) &&
//	              !DisableRevisionCheck                  &&
//	              ( ( ParallelCan.TotalNumberOfUPMs > 1 ) ||
//	                (NumOfUPMs > 1) ) )                   //    ||
//	//            ( !DSPInRegister.GpiC.bit.VersionID ) ) // fix APACTS-594
//	        {
//	        	EEStatusBits.bit.HardwareIncompatible = 1;
//	        }
//	        else
//	        {
//	        	EEStatusBits.bit.HardwareIncompatible = 0;
//	        }

        if ( Abm().GetBatterySetupRequired() &&
             !PowerConditionerMode )
        {
            EEStatusBits.bit.BatterySetupRequired = true;
        }
        else
        {
            EEStatusBits.bit.BatterySetupRequired = false;
        }
        
        // Limit rectifier and inverter hardware current limits to 200 for revision 5 hardware
        // Done periodically instead of in the update fuction because eeprom writes during startup
        // don't work right now
        // TODO: Move this back to the ee update funcs.  Make sure it works on power on reset.
//	        static uint16_t rev5Limit = 200;
//	        if((ExpansionInputReg.bit.RevID == Rev5) &&
//	           !DisableRevisionCheck )
//	        {
//	        	if( RectHWCurrentLimit > 200 )
//	        	{
//	        		PutEepData( PARAM_Rectifier_Current_Limit_Set, 1, &rev5Limit, 0 );
//	        	}
//	        	
//	        	if( InvHWCurrentLimit > 200 )
//	        	{
//	        		PutEepData( PARAM_Inverter_Current_Limit_Set, 1, &rev5Limit, 0 );
//	        	}
//	        }
        
        if ( 0 != EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] )
        {
            // checksum bad
            NB_DebounceAndQue( UPM_NB_NON_VOLATILE_RAM_FAILURE, true, EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ] );
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_NON_VOLATILE_RAM_FAILURE, false );
        }

        CheckSystemType();   // Check if CSB has finished system type verification.
    
        if (( 0 != EEStatusBits.w[ EE_STATUS_CONFIG_WORD ] )&&(FCTStateMachine.BoardID_Warning == true))
        {
            // configuration error
            NB_DebounceAndQue( UPM_NB_CONFIGURATION_ERROR, true, EEStatusBits.w[ EE_STATUS_CONFIG_WORD ] );
        }
        else
        {
            NB_DebounceAndQue( UPM_NB_CONFIGURATION_ERROR, false );
        }
    }
    BTR.MonitorFullCapacityBTR();
    BTR.MonitorStateOfCapacity();
    
    DebounceCSBNodebits();     

    FanSpeedCalculate();
    FanHv20_40KControl();
	if( OutputKVA != 200 && OutputKVA != 300 && OutputKVA != 400 && OutputKVA != 600 && OutputKVA != 800)
	{
		STSFanControl( maxSinglePhaseLoad, AMB_Thermal_C );
	}

    ParallelSystem1000Msec(); 
                  
    MCUStateMachine.Run1Sec();
}

// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_20sFunctions() 
// *
// * Purpose:   Tasks to perform every 20s, and every 500 ms for the first two
// *            seconds after startup.
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ******************************************************************** 
void TSK_20sFunctions(void)
{

    InternalCan.SendCanId();
}


// ********************************************************************
// *                                                                                                                     
// * Function:  TSK_1minFunctions() 
// *
// * Purpose:   Tasks to perform every 1min
// *                    
// * Parms Passed:  Nothing
// *
// * Returns:   Nothing                                                                           
// *
// * Description: 
// *                    
// ******************************************************************** 
void TSK_1minFunctions(void)
{
    static uint16_t Timer1 = 0;
    static uint16_t Timer2 = 0;

    FuncRunTimeUs[eAdcIsr] = 0.0;
    FuncRunTimeUs[eReverse1] = 0.0;
    FuncRunTimeUs[eReverse2] = 0.0;
    FuncRunTimeUs[eReverse3] = 0.0;
    FuncRunTimeUs[eReverse4] = 0.0;
    FuncRunTimeUs[eMax] = 0.0;
    // 1 hour 
    if(++Timer1 >= 60)
    {
        Timer1 = 0;
    }
}

// ********************************************************************************************************
// *
// * Function: ChkStacks(void);
// *
// * Purpose: To calculate how much percentage of the stack has been used
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Puts maximum of stack used percentage wise into a global variable.
// *
// ********************************************************************************************************
// BIOS defines heap1 using assembly in Panda28335cfg.h28, reference for .cpp here
extern int16_t heap1;

void CheckStacks( void )
{
    TSK_Stat statisticBuffer = { 0 };
    float tempPercentage = 0.0f;
    
    uint32_t thisInterval = LastCPUTimer - ReadCpuTimer0Counter();
    LastCPUTimer = ReadCpuTimer0Counter();
    
    PercentIdle = CalcPercentIdle( ( thisInterval / 10 ) );
    
    // Ourself
    TSK_stat( &TSK_Periodic, &statisticBuffer );
    PeriodicTaskPercent = (float)statisticBuffer.used / (float)statisticBuffer.attrs.stacksize;
    
    // PortHandler
    TSK_stat( &TSK_PortHandler, &statisticBuffer );
    PortStackPercent = (float)statisticBuffer.used / (float)statisticBuffer.attrs.stacksize;
    
    // SPIControlStackPercent
    TSK_stat( &TSK_spi, &statisticBuffer );
    SPIControlStackPercent = (float)statisticBuffer.used / (float)statisticBuffer.attrs.stacksize;
    
    // HistQueStackPercent
    TSK_stat( &TSK_HistoryQueue, &statisticBuffer );
    HistQueStackPercent = (float)statisticBuffer.used / (float)statisticBuffer.attrs.stacksize;
    
    // PCanStackPercent
    TSK_stat( &TSK_ParallelCAN, &statisticBuffer );
    PCanStackPercent = (float)statisticBuffer.used / (float)statisticBuffer.attrs.stacksize; 
    
    // C9 handler stack
    TSK_stat( &TSK_ICanRx, &statisticBuffer );
    ICanRxStackPercent = float(statisticBuffer.used) / statisticBuffer.attrs.stacksize;
    
    // not going to bother with idle
    
    // percentage of heap used
    MEM_Stat memBuffer = {0, 0, 0, 0};
    MEM_stat( heap1, &memBuffer ); //This API only gets current used size
    tempPercentage = (float)memBuffer.used / (float)memBuffer.size;
    //Get the max percentage ever since
    if(tempPercentage > HeapPercent)
    {
        HeapPercent = tempPercentage;
    }
    
    // finally, there seems to be no BIOS function to check the global stack. At least I can't find one if there is. So doing it 
    // the old fashioned way
    HWIStackPercent = CalcGlobalStackPercent();
}

// ********************************************************************************************************
// *
// * Function: CalcGlobalStackPercent(void);
// *
// * Purpose: To calculate how much percentage of the system stack has been used
// *
// * Parms Passed   :   Nothing
// * Returns        :   stack percentage used
// *
// * Description: 
// *
// ********************************************************************************************************
extern void* HWI_STKBOTTOM;
extern void* HWI_STKTOP; 
          
float CalcGlobalStackPercent( void )
{
    const uint16_t TaskSeedToken = 0xbeef;
    
    uint32_t  stackSize = (uint32_t)(&HWI_STKTOP) - (uint32_t)(&HWI_STKBOTTOM);
    uint16_t* stkPtr = (uint16_t*)(&HWI_STKBOTTOM);
    uint16_t  used = 0;
    uint16_t  tokens = 0;
    
    for ( uint16_t idx = 0; idx < stackSize; idx++ )
    { 
        if ( *stkPtr != TaskSeedToken )
        {
            ++used;
            // reset tokens, in case a used stack var happens to contain the token
            used += tokens;
            tokens = 0;
        }
        else
        {
            ++tokens;
            // safe to say there won't be 3 in row
            if ( tokens > 2 )
            {
                break;
            }    
        }    
        ++stkPtr;
    }    
    
    return ( (float)used / (float)stackSize );    
}

// ***********************************************************************
// *
// *    FUNCTION: DebounceCSBNodebits 
// *
// *    DESCRIPTION: Special nodebits ala Fred
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void DebounceCSBNodebits( void )
{
    // UPM_NB_OUTPUT_HOT
    NB_DebounceAndQue( UPM_NB_OUTPUT_HOT, OutputPLL.IsSourcePresent() );
    // UPM_NB_BYPASS_HOT
    NB_DebounceAndQue( UPM_NB_BYPASS_HOT, BypassPLL.IsSourcePresent() );
    // UPM_NB_BATTERY_DISCHARGING
    if ( BATTERY_ON_BOOST_STATE == BatteryConverter.GetBatteryState() )
    {
        NB_DebounceAndQue( UPM_NB_BATTERY_DISCHARGING, true );
    }
    else
    {
        NB_DebounceAndQue( UPM_NB_BATTERY_DISCHARGING, false );
    }        
    // UPM_NB_MIS_CLOSED 
    NB_DebounceAndQue( UPM_NB_MIS_CLOSED, false );
    // UPM_NB_MIS_INSTALLED
    NB_DebounceAndQue( UPM_NB_MIS_INSTALLED, false );
    // UPM_NB_BYPASS_INSTALLED
    NB_DebounceAndQue( UPM_NB_BYPASS_INSTALLED, true );
    // UPM_NB_INTERNAL_MBS_INSTALLED
    NB_DebounceAndQue( UPM_NB_INTERNAL_MBS_INSTALLED, InternalMBSInstalled );
    // UPM_NB_BATTERY_INSTALLED means power conditioner mode is NOT on
    NB_DebounceAndQue( UPM_NB_BATTERY_INSTALLED, !PowerConditionerMode );
    // UPM_NB_ECO_INSTALLED
    NB_DebounceAndQue( UPM_NB_ECO_INSTALLED, ESSInstalled );
}

// ********************************************************************
// *
// * Function:  WriteDSPOutputs_TSK( void ) ;
// *
// * Purpose:   Write RAM copy of GPIO output registers to actual ports, from
// *            any OS context.
// *
// * Parms Passed   : None
// * Returns    : Nothing
// *
// *
// * Description:
// *
// ********************************************************************
void WriteDSPOutputs_TSK(void)
{
    CriticalSection enter;
    WriteDSPOutputs_ISR();
}

// ********************************************************************
// *
// * Function:  CheckSystemType( void ) ;
// *
// * Purpose:   Check if CSB has finished system type verification
// *
// * Parms Passed   : None
// * Returns    : Nothing
// *
// *
// * Description:
// *
// ********************************************************************
void CheckSystemType(void)
{
	static uint16_t cunt = 0;

	if(cunt++ >= 10)   // detect system type after system power on for 10 seconds
	{
		cunt = 10;

		// after the time out, the configure error alarm will be active if the checking result failed;
        if (InternalCan.GetSystemTypeCheckResult() == false)
        {
//	            EEStatusBits.bit.SystemTypeCheckFail = 1;		//temp mask
        }
        else
        {
            EEStatusBits.bit.SystemTypeCheckFail = 0;
        }
	}
}
// ********************************************************************

// ********************************************************************
// *
// * Function:  EncryptionVry( void ) ;
// *
// * Purpose:   Encryption function
// *
// * Parms Passed   : None
// * Returns    : Nothing
// *
// *
// * Description:
// *
// ********************************************************************
void EncryptionVry( void )
{
    static uint16_t EncrypWatchDog_Count = 0;
    if( (Encrypt_Verify_204.Verify_Time < VERIFY_MAX_TIME) &&
        (EncrypWatchDog_Count % 3 == 0))                    // Watch Dog time for ATSHA204 which is 1.5s typically.       
    {
        uint16_t Delay_Time = 0;

        while((I2c_Operation != cOperation_Null) && (Delay_Time < 5))
        {
            TSK_sleep(1);
            Delay_Time++;
        }
        if( I2c_Operation == cOperation_Null )
        {
            uint16_t ret_code = SHA204_GEN_FAIL;
            Encrypt_Verify_204.I2C_Block = true;
            ret_code = Encrypt_Verify_204.Configure_204();
            
            if(ret_code == SHA204_SUCCESS)
            {
                EEStatusBits.bit.DecryptionFailed = 0;
                Encrypt_Verify_204.Verify_Time = VERIFY_MAX_TIME;     //Stop Verify process
            }
            else
            {
                EEStatusBits.bit.DecryptionFailed = 1;		//enable decrypt
                Encrypt_Verify_204.Verify_Time++;
                if(ret_code == SHA204_DATA_FAIL)
                {
                    Encrypt_Verify_204.Verify_Time = VERIFY_MAX_TIME; //Stop Verify process
                }
            }
            Encrypt_Verify_204.I2C_Block = false;
        }
    }

    EncrypWatchDog_Count++;
}


//*********************************************************************
//Function: void CheckVFModeReady(void)
//Purpose:  Update the VF mode ready bit 
//Parms Passed   : None
//Returns    : Nothing
//Dsscription: when The UPS is online or ESS and the load is lower than
//             55% and the power module is not ower temperature, the PWM
//             will change from 10K to 20K. otherwise back to 10K
//*********************************************************************
void CheckVFModeReady(void)
{
    static uint16_t enterVFModeCounter = 0;
    if( EnableVFMode                                           &&
        ( MCUStateMachine.GetState() == ONLINE_STATE )         &&
        ( Rectifier.GetState() == RECTIFIER_NORMAL_STATE )     &&
        !NB_GetNodebit(UPM_NB_PM_OVERTEMPERATURE)              &&
        !NB_GetNodebit(UPM_NB_PM_OVERTEMPERATURE_TRIP)         &&
        (ScreenMeters.PercentLoad.phA < VFModeLoadPercentMax ) &&
        (ScreenMeters.PercentLoad.phB < VFModeLoadPercentMax ) &&
        (ScreenMeters.PercentLoad.phC < VFModeLoadPercentMax ) &&
        !NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR) )
    {
        ++enterVFModeCounter;
        if( enterVFModeCounter > (VF_MODE_UPDATE_FREQ * VFModeEntryTime) )
        {
            enterVFModeCounter = VF_MODE_UPDATE_FREQ * VFModeEntryTime;
            MCUStateMachine.SetVFModeReady(true);
        }
    }
    else
    {
        enterVFModeCounter = 0;
    }
    
    if( !EnableVFMode                                       ||
        ( MCUStateMachine.GetState() != ONLINE_STATE )      ||
        ( Rectifier.GetState() != RECTIFIER_NORMAL_STATE )  ||
        NB_GetNodebit(UPM_NB_PM_OVERTEMPERATURE)            ||
        NB_GetNodebit(UPM_NB_PM_OVERTEMPERATURE_TRIP)       ||
        ( ScreenMeters.PercentLoad.phA > (VFModeLoadPercentMax + VFModeLoadPercentHysteresis) ) ||
        ( ScreenMeters.PercentLoad.phB > (VFModeLoadPercentMax + VFModeLoadPercentHysteresis) ) ||
        ( ScreenMeters.PercentLoad.phC > (VFModeLoadPercentMax + VFModeLoadPercentHysteresis) ) ||
        NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR) )
    {
        enterVFModeCounter = 0;
        MCUStateMachine.SetVFModeReady(false);
    }
}
// ********************************************************************
// *            End of PeriodicTaskFuncs.c   
// ********************************************************************
