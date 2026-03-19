// ******************************************************************************************************
// *            end of AutoCal.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: end of AutoCal.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 12/10/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "AutoCal.h"
#include "ParallelCanIds.h"
#include "DQPhaseLockLoop.h"
#include "InverterControl.h"
#include "MCUState.h"
#include "StateTimer.h"
#include "NB_Config.h"
#include "Spi_Task.h"
#include "ExtSignalPLL.h"
#include "ParallelCan.h" 

#include "DebuggerBlocks.h"

AutoCalControl AutoCal;

namespace {

const stSecondOrderIIRFP    LoadShareCalCoefficients = { 0.075, -0.85, 0, -0.9995, 0, 0, 0 };
const stSecondOrderIIRFP    VoltageBalanceCoefficients = { 2.0e-6, -0.50, 0, -0.95, 0, 0, 0 }; 	//4.1e-8 <-> 0.2V(15kW)


} // !namespace (anon) 

// ***********************************************************************
// *
// *    FUNCTION: RunAutoCal 
// *
// *    DESCRIPTION: Autocal state machine
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void AutoCalControl::RunAutoCal( void )
{
    // send cal meters if in autocal mode
    if ( ParallelCan.GetState() != CanDriver::Initialization )
    {
    	// This is for detecting CAN down in external parallel system.  This is sent every 20ms 
    	// so that all UPMs slew to base at this resolution.
    	ParallelCan.TransmitBroadcastPacket( pcan::inv_cal_meters0, (uint16_t*)&ParallelCan.MyCalData(), 4, true ); 
    }

    switch ( CalState )
    {
        //
        // waiting for startup command, master from debugger/CSB, slave UPMS follow master
        //
        case AUTOCAL_INIT:                      
            // clear status
            AutoCalStatus.all = 0;

            //    mstr                   slave
            if ( Start || ParallelCan.ParGlobalOrData.SyncStatus.bit.AutoCalRun )
            {
                if ( CheckAutoCalOK() )         // check abort conditins
                {
                    if ( ParallelCan.UPMNumber() == 0 )
                    {
                        // master sets bits for slaves to start their cal
                        AutoCalStatus.bit.AutoCalRun = 1;
                    }
                    NB_SetNodebit(UPM_NB_SYSTEM_TEST_IN_PROGRESS, true, SYSTEM_TEST_AUTOCAL);

                    CalState = AUTOCAL_LOADSHARE_BYPASS_START;
                    AutoCalTimer1.ClearTimer();
                    AutoCalTimer2.ClearTimer();

                    // reset cal data
                    EE_ID* ee = GetParameterEE( PARAM_BypassLoadSharePhaseCal );
                    PutEepData( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), 0 );
                    ee = GetParameterEE( PARAM_BaseLoadSharePhaseCal );
                    PutEepData( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), 0 );
                    ee = GetParameterEE( PARAM_LoadShareVoltageCalA );
                    PutEepData( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), 0 );
                    ee = GetParameterEE( PARAM_LoadShareVoltageCalB );
                    PutEepData( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), 0 );
                    ee = GetParameterEE( PARAM_LoadShareVoltageCalC );
                    PutEepData( ee->eep_addr, ee->eep_length, &(ee->eep_DefaultValue[CoefficientsIndex]), 0 );
                }

                Start = false;
            }
            break;

        //
        // waiting for everybody to get into the same mode, and for meters to start coming in
        // 
        case AUTOCAL_LOADSHARE_BYPASS_START:
            AutoCalStatus.bit.AutoCalBypass = 1;

            if ( ParallelCan.ParGlobalAndData.SyncStatus.bit.AutoCalBypass )
            {
                // everybody is ready, start the cal phase
                CalState = AUTOCAL_LOADSHARE_BYPASS;
                GPControl = LoadShareCalCoefficients;
                VarControlA = VoltageBalanceCoefficients;
                VarControlB = VoltageBalanceCoefficients;
                VarControlC = VoltageBalanceCoefficients;
				
                VarControlA.B0= VoltageBalanceCoefficients.B0;
                VarControlB.B0= VoltageBalanceCoefficients.B0;
                VarControlC.B0= VoltageBalanceCoefficients.B0;
                
                AutoCalTimer1.ClearTimer();
                AutoCalTimer2.ClearTimer();
            }
            else
            {
                if ( AutoCalTimer1.CheckTimeout( AutoCal5s ) )
                {
                    // for some reason, somebody declined the invitation to dance, reset
                    AutoCalReset();
                }
            }
            break;

        //
        // adjust bypass phase cal to equalize UPM real power, sync to bypass mode
        // 
        case AUTOCAL_LOADSHARE_BYPASS:
            if ( !CheckAutoCalOK() )
            {
                // abort
                AutoCalReset();
            } 
            else
            {
                // run offset calc delayed for meter settling and phase delay
                if ( AutoCalTimer1.CheckTimeout( AutoCal500ms ) )
                {
                    AutoCalTimer1.ClearTimer();
                    
                    float offset[4];
                    RunLoadShareCal( offset );
                    
                    // call the ee functions to update the cal factors
                    EE_ID* ee = GetParameterEE( PARAM_BypassLoadSharePhaseCal );
                    // float to int for ee write
//	                    EE_Data = int16_t( offset[0] * 32767.0f );
//	                    BypassPLL.SineRef.EEFunc_Sync( ee, (uint16_t*)&EE_Data);	//wombat no phase cal

                    ee = GetParameterEE( PARAM_LoadShareVoltageCalA );
                    EE_Data = int16_t( offset[1] * 32767.0f);
                    Inverter.EEFunc_Inverter(ee, (uint16_t*)&EE_Data);
                    
                    ee = GetParameterEE( PARAM_LoadShareVoltageCalB );
                    EE_Data = int16_t( offset[2] * 32767.0f);
                    Inverter.EEFunc_Inverter(ee, (uint16_t*)&EE_Data);
                    
                    ee = GetParameterEE( PARAM_LoadShareVoltageCalC );
                    EE_Data = int16_t( offset[3] * 32767.0f);
                    Inverter.EEFunc_Inverter(ee, (uint16_t*)&EE_Data);
					
                }

                // unsync cal next
                if ( AutoCalTimer2.CheckTimeout( AutoCal60s ) 
                    || !ParallelCan.ParGlobalAndData.SyncStatus.bit.AutoCalBypass )
                {
                    CalState = AUTOCAL_LOADSHARE_BYPASS_SAVE;
                    PhaseState = 0;
                    AutoCalTimer1.ClearTimer();
                    AutoCalTimer2.ClearTimer();
                }
            }
            break;
        
        case AUTOCAL_LOADSHARE_BYPASS_SAVE:
            // Save accumulated data.
            if ( AutoCalTimer1.CheckTimeout( AutoCal100ms ) ) 
            {
                EE_ID* ee;

                switch ( PhaseState )
                {
                    case 0:
//	                        ee = GetParameterEE( PARAM_BypassLoadSharePhaseCal );
//	                        EE_Data = int16_t( BypassPLL.SineRef.GetPhaseCal() * 32767.0f);
//	                        PutEepData(ee->eep_addr, 1, (uint16_t*)&EE_Data, 0);
                        PhaseState++;
                        break;

                    case 1:
                        ee = GetParameterEE( PARAM_LoadShareVoltageCalA );
                        EE_Data = int16_t( Inverter.GetLoadShareVoltageCal().phA * 32767.0f);
                        PutEepData(ee->eep_addr, 1, (uint16_t*)&EE_Data, 0);
                        PhaseState++;
                        break;
                    
                    case 2:
                        ee = GetParameterEE( PARAM_LoadShareVoltageCalB );
                        EE_Data = int16_t( Inverter.GetLoadShareVoltageCal().phB * 32767.0f);
                        PutEepData(ee->eep_addr, 1, (uint16_t*)&EE_Data, 0);
                        PhaseState++;
                        break;

                    case 3:
                        ee = GetParameterEE( PARAM_LoadShareVoltageCalC );
                        EE_Data = int16_t( Inverter.GetLoadShareVoltageCal().phC * 32767.0f);
                        PutEepData(ee->eep_addr, 1, (uint16_t*)&EE_Data, 0);
                        PhaseState = 0;
                        CalState = AUTOCAL_LOADSHARE_BASE_START;
                        break;

                    default:
                        PhaseState = 0;
                        break;
                }
                
                AutoCalTimer1.ClearTimer();    
            }   
            break;
        
        //
        // same as before, waiting for everybody to be in the same mode
        // 
        case AUTOCAL_LOADSHARE_BASE_START:
            if ( EEP_ExtSyncEnabled() && !ParallelCan.UPSIsParallel() )
            {
                AutoCalStatus.bit.AutoCalUnsync = 1;
                AutoCalStatus.bit.AutoCalBypass = 0;

                if ( ParallelCan.ParGlobalAndData.SyncStatus.bit.AutoCalUnsync )
                {
                    CalState = AUTOCAL_LOADSHARE_BASE;
                    GPControl = LoadShareCalCoefficients;
                    VarControlA = VoltageBalanceCoefficients;
                    VarControlB = VoltageBalanceCoefficients;
                    VarControlC = VoltageBalanceCoefficients;

                    AutoCalTimer1.ClearTimer();
                    AutoCalTimer2.ClearTimer();
                }
                else
                {
                    if ( AutoCalTimer1.CheckTimeout( AutoCal5s ) )
                    {
                        // same as last time
                        AutoCalReset();     
                    }
                }
            }
            else
            {
                AutoCalStatus.all = 0;
                AutoCalTimer1.ClearTimer();
                AutoCalTimer2.ClearTimer();
                CalState = AUTOCAL_DONE;
            }
            break;

        //
        // running un-sync(base) mode, voltage is assumed calibrated in bypass mode, only do phase to compensate prop
        // delay of PARA_SYNC circuits
        //
        case AUTOCAL_LOADSHARE_BASE:
            if ( !CheckAutoCalOK() )
            {
                AutoCalReset();
            } 
            else
            {
                // same as before
                if ( AutoCalTimer1.CheckTimeout( AutoCal500ms ) )
                {
                    AutoCalTimer1.ClearTimer();
                    
                    float offset[4];
                    RunLoadShareCal( offset );
                    
                    EE_Data = int16_t( offset[0] * 32767.0f );
                    
                    EE_ID* ee = GetParameterEE( PARAM_BaseLoadSharePhaseCal );
//	                    ExtSignalSync.SineRef.EEFunc_Sync( ee, (uint16_t*)&EE_Data);
                }

                if ( AutoCalTimer2.CheckTimeout( AutoCal60s ) || !ParallelCan.ParGlobalAndData.SyncStatus.bit.AutoCalUnsync )
                {
                    CalState = AUTOCAL_LOADSHARE_BASE_SAVE;
                    PhaseState = 0;
                    AutoCalStatus.all = 0;
                    AutoCalTimer1.ClearTimer();
                    AutoCalTimer2.ClearTimer();
                }
            }
            break;

        case AUTOCAL_LOADSHARE_BASE_SAVE:
            if ( AutoCalTimer1.CheckTimeout( AutoCal100ms ) )
            {
                EE_ID* ee = GetParameterEE( PARAM_BaseLoadSharePhaseCal );
//	                EE_Data = int16_t( ExtSignalSync.SineRef.GetPhaseCal() * 32767.0f );
//	                PutEepData( ee->eep_addr, ee->eep_length, (uint16_t*)&EE_Data, 0 );
                CalState = AUTOCAL_DONE;

                AutoCalTimer1.ClearTimer();    
            }
            break;

        //
        // just hang out for a bit, make sure everybody's bits are clear so we don't immediately restart...
        //
        case AUTOCAL_DONE:
            if ( AutoCalTimer1.CheckTimeout( AutoCal1s ) )
            {
                CalState = AUTOCAL_INIT;
                NB_SetNodebit(UPM_NB_SYSTEM_TEST_IN_PROGRESS, false);
                AutoCalTimer1.ClearTimer();
                //Inverter.EnableVarControl();
            }
            break;

        default:
            CalState = AUTOCAL_INIT;
            break;
            
    }
}

// ***********************************************************************
// *
// *    FUNCTION: StartAutoCal 
// *
// *    DESCRIPTION: Master only start command. Why only master? Not sure
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void AutoCalControl::StartAutoCal( void )
{
    if ( ParallelCan.UPMNumber() == 0 )
    {
        Start = true;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: CheckAutoCalOK 
// *
// *    DESCRIPTION: Autocal abort conditions
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
bool AutoCalControl::CheckAutoCalOK( void )
{
    //
    // klv: I'm sure there are many more conditions that need to be here, this was
    // all I can think of right now, excepting abnormals which I don't care to add
    // Since this is user initiated, and all users are engineers at this time it's 
    // user beware. If you start auto-cal and the unit is in overload or has active alarms
    // and blows up, that's your own damn fault.
    //
    if ( ( ONLINE_STATE == MCUStateMachine.GetState() )                     &&
//	         ( ParallelCan.ParGlobalAndData.SyncStatus.bit.BypassSyncAvail )    &&
		 (( ParallelCan.ParGlobalAndData.SyncStatus.bit.BypassSyncAvail )||
		  ( ParallelCan.ParGlobalAndData.SyncStatus.bit.InputSyncAvail ))  &&
         ( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )                    &&
         ( ParallelCan.TotalNumberOfUpmsOnline > 1 )                        &&
         ParallelCan.PCan_CheckMOBsClosed() )
    {
        return true;
    }
    else
    {
        return false;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: RunLoadShareCal 
// *
// *    DESCRIPTION: Simple little PI loop to remove load share power error
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void AutoCalControl::RunLoadShareCal( float (&command)[4] )
{
    float ref[] = { 0, 0, 0, 0 };

    for ( uint16_t i = 0; i < ParallelCanNetwork::MAX_NUM_UPS; i++ )
    {
        for ( uint16_t j = 0; j < ParallelCanNetwork::MAX_NUM_UPM; j++ )
        {
            if ( !ParallelCan.UpmData[i][j].Timeout && ParallelCan.UpmData[i][j].InverterStatus.bit.InverterOn )
            {
                ref[0] += float( ParallelCan.CalData[i][j].id ) / 32767.0f;
                ref[1] += float( ParallelCan.CalData[i][j].powerA );
                ref[2] += float( ParallelCan.CalData[i][j].powerB );
                ref[3] += float( ParallelCan.CalData[i][j].powerC );
            }
        }
    }

    float self[] = {
        float(ParallelCan.MyCalData().id) / 32768.0f, 
        float(ParallelCan.MyCalData().powerA),
        float(ParallelCan.MyCalData().powerB),
        float(ParallelCan.MyCalData().powerC)
    };


    if ( 0 != ParallelCan.TotalNumberOfUpmsOnline )
    {
        float invCount = 1.0f / float( ParallelCan.TotalNumberOfUpmsOnline );
        ref[0] *= invCount;
        ref[1] *= invCount;
        ref[2] *= invCount;
        ref[3] *= invCount;

        command[0] = SecondOrderIIRFP( ( ref[0] - self[0] ), &GPControl );
        command[1] = SecondOrderIIRFP( ( ref[1] - self[1] ), &VarControlA );
        command[2] = SecondOrderIIRFP( ( ref[2] - self[2] ), &VarControlB );
        command[3] = SecondOrderIIRFP( ( ref[3] - self[3] ), &VarControlC );
    }
    else
    {
        for (uint16_t i = 0; i < 4; ++i)
        {
            command[i] = 0.0f;
        }
    }
}

// ***********************************************************************
// *
// *    FUNCTION: AutoCalReset 
// *
// *    DESCRIPTION: Resets to default state
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void AutoCalControl::AutoCalReset( void )
{
    CalState = AUTOCAL_DONE; 
    AutoCalStatus.all = 0;
    AutoCalTimer1.ClearTimer();
    AutoCalTimer2.ClearTimer();
}


// ******************************************************************************************************
// *  end of AutoCal.cpp
// ******************************************************************************************************

