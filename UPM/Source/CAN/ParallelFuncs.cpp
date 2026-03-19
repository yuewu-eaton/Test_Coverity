#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ParallelCan.h"
#include "ParallelCanIds.h"

#include "Eeprom_Map.h"
#include "InvSync.h"
#include "NB_Funcs.h"
#include "UPM_NB_Ids.h"
#include "F28335Port.h"
#include "MCUState.h"
#include "BypassInterface.h"
#include "IOExpansion.h"
#include "AutoCal.h"
#include "ACPowerMeter.h"
#include "Abm.h"
#include "Rtc.h"
#include "Spi_Task.h"

#include <stddef.h>
#include <stdlib.h>

#define MASTER_UPM_NUMBER 0

void UpdateParallelLoadShareError(void);
extern uint16_t SingleUPSStartUpEnabled;

namespace {
// ***********************************************************************
// *
// *    FUNCTION:  SendStatusPackets()64*7*100
// *
// *    DESCRIPTION: Send status packets about this node to all other nodes on
// *                 the (parallel) UPS.
// *
// *    ARGUMENTS: onlyDirty  When true, only send packets for status that has changed.
// *                          When false, unconditionally exactly one status packet,
// *                          and rotate which packet gets sent with each invocation.
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void SendStatusPackets( bool onlyDirty, bool resendAll = false )
{
    static uint16_t thisStatusPacket = 0;
    UpmState myState;
        
    myState.BypassStatus = BypassState().GetBypassStatus();
    myState.BatteryStatus = BatteryConverter.GetBatteryStateStatus();
    myState.McuStatus = MCUStateMachine.GetStatus();
    myState.InverterStatus = Inverter.GetStatus();
    myState.SystemError = ParallelCan.GetSystemError();
    myState.AbmStatus = Abm().GetStatus();
    myState.SyncStatus.words[ SYNC_WORD ] = GetSyncStatus();
    myState.SyncStatus.words[ AUTO_CAL_WORD ] = AutoCal.GetAutoCalStatus();
    myState.SyncStatus.words[2] = 0;
    myState.SyncStatus.words[3] = 0;
    myState.ParallelStatus = ParallelCan.GetParallelStatus();
    bool MOBInitiallyClosed = ParallelCan.MyUpmData().McuStatus.bit.BldInpMOBClosed;
    
    using namespace pcan;
    const struct
    {
        uint16_t can_type_tag;
        uint16_t offset;
        // uint16_t len;
    } statusWords[] = 
    {
        { mcu_status,       offsetof(UpmState, McuStatus)/*,       sizeof(myState.McuStatus)*/       },
        { bypass_status,    offsetof(UpmState, BypassStatus)/*,    sizeof(myState.BypassStatus)*/    },
        { battery_status,   offsetof(UpmState, BatteryStatus)/*,   sizeof(myState.BatteryStatus)*/   },
        { inverter_status,  offsetof(UpmState, InverterStatus)/*,  sizeof(myState.InverterStatus)*/  },
        { sync_status,      offsetof(UpmState, SyncStatus)/*,      sizeof(myState.SyncStatus)*/      },
        { system_error,     offsetof(UpmState, SystemError)/*,     sizeof(myState.SystemError)*/     },
        { parallel_status,  offsetof(UpmState, ParallelStatus)/*,  sizeof(myState.ParallelStatus)*/  },
        { abm_status,       offsetof(UpmState, AbmStatus)/*,       sizeof(myState.AbmStatus)*/       },
        { 0,                0/*,                                   0*/ }, // Sentinnel
    };
    
    if ( onlyDirty || resendAll )
    {
        for (int i = 0; statusWords[i].can_type_tag != 0; ++i)
        {
            const uint16_t offset = statusWords[i].offset;
            const uint16_t length = 4; // statusWords[i].len;
            uint16_t* const now = ((uint16_t*)&myState) + offset;
            uint16_t* const last = ((uint16_t*)&ParallelCan.MyUpmData()) + offset;
            
            // Merge the sequence number from its storage.  State machines themselves do not maintain
            // their own sequence number, only CAN does.
            now[3] = (now[3] & 0x0fffu) | (last[3] & 0xf000u);
            
            // A dirty flag for those status bits.  When set, the data is dirty, and
            // this node must update all other nodes about the change.  If resendAll is
            // true, consider all caches to be dirty.
            bool dirty = resendAll;
            
            // update my status and check if changed since last time
            for (uint16_t j = 0; j < length; ++j)
            {
                dirty = dirty || last[j] != now[j];
                last[j] = now[j];
            }
            
            // if changed or commanded to send all data,transmit can packets
            if (dirty)
            {	
            	// Pick next number
            	uint16_t seqNumber = (now[3] & 0xf000u) >> 12;
            	if (resendAll)
            	{
            		// Reset sequence
            		seqNumber = 0;
            	}
            	else if (++seqNumber > 15)
            	{
            		seqNumber = 1;
            	}
            	now[3] = (now[3] & 0x0fffu) | ((seqNumber & 0xfu) << 12);
            	last[3] = now[3];
            	
                ParallelCan.TransmitBroadcastPacket(statusWords[i].can_type_tag, now, length, true);
                bool MOBFinallyClosed = ParallelCan.MyUpmData().McuStatus.bit.BldInpMOBClosed;
                
                if (MOBInitiallyClosed != MOBFinallyClosed)
                {
                	// All global bits must recomputed when the MOB status changes.
                	UpdateGlobalParallelBits();
                }
                else
                {
               		UpdateGlobalParallelBits(offset, length);
                }
            }
        }
    }
    else
    {
        // send 1 status packet per round
        const uint16_t offset = statusWords[thisStatusPacket].offset;
        const uint16_t length = 4; // statusWords[thisStatusPacket].len;
        uint16_t* const now = ((uint16_t*)&myState) + offset;
        uint16_t* const last = ((uint16_t*)&ParallelCan.MyUpmData()) + offset;

        now[3] = (now[3] & 0x0fffu) | (last[3] & 0xf000u);
            
    	// Pick next number
    	uint16_t seqNumber = (now[3] & 0xf000u) >> 12;
    	if (++seqNumber > 15)
    	{
    		seqNumber = 1;
    	}
    	now[3] = (now[3] & 0x0fffu) | ((seqNumber & 0xfu) << 12);
    	// Save the sequence number, such that it is not considered dirty
    	last[3] = now[3];
    	
        ParallelCan.TransmitBroadcastPacket(statusWords[thisStatusPacket].can_type_tag, now, length, true);
        thisStatusPacket++;
        if ( 0 == statusWords[thisStatusPacket].can_type_tag )
        {
            thisStatusPacket = 0;
        }       
    }
}
} // !namespace (anon)

// *****************************************************************************
// *
// * Function: UpdateGlobalParallelBits(const size_t offset, const uint16_t len);
// *
// * Purpose: Update global parallel AND and OR statusbits for a single status packet
// *
// * Parms Passed   :   offset: The base offset corresponding to the status packet's data
// *                    len: The number of DSP words to update
// * Returns        :   Nothing
// *
// * Description: All possible UPMs are included.
// *
// *****************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void UpdateGlobalParallelBits(const size_t offset, const uint16_t len_)
{
	const uint16_t len = 4;
    uint16_t and_data[4] = 
    {
        0xffffU,
        0xffffU,
        0xffffU,
        0xffffU,
    };
    
    uint16_t or_data[4] = 
    {
        0x0000,
        0x0000,
        0x0000,
        0x0000,
    };
    
    if (NB_GetNodebit(UPM_NB_MOB_OPEN))
    {
    	// Then only those UPMs from my UPS are included
    	uint16_t ups = MyUPSNumber;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if ( !ParallelCan.UpmData[ ups ][ upm ].Timeout)
            {
                const uint16_t *baseAddr = (uint16_t *)&ParallelCan.UpmData[ups][upm] + offset;
                for (size_t offset = 0; offset < len; ++offset)
                {
                    and_data[offset] &= baseAddr[offset];
                    or_data[offset] |= baseAddr[offset];
                }
            }
        }
    }
    else
    {
        // All those UPMs whose MOB is closed are included
       for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
       {
           for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
           {
               if ( !ParallelCan.UpmData[ ups ][ upm ].Timeout &&
               	 ParallelCan.UpmData[ ups][upm].McuStatus.bit.BldInpMOBClosed)
               {
                   const uint16_t *baseAddr = (uint16_t *)&ParallelCan.UpmData[ups][upm] + offset;
                   for (size_t offset = 0; offset < len; ++offset)
                   {
                       and_data[offset] &= baseAddr[offset];
                       or_data[offset] |= baseAddr[offset];
                   }
               }
           }
       }
    }
    
    uint16_t* andBaseAddr = (uint16_t *)&ParallelCan.ParGlobalAndData + offset;
    uint16_t* orBaseAddr = (uint16_t *)&ParallelCan.ParGlobalOrData + offset;
    
    {
        CriticalSection enter( IER_DMA_ONLY );
	    for (size_t offset = 0; offset < len; ++offset)
	    {
	        andBaseAddr[offset] = and_data[offset];
	        orBaseAddr[offset] = or_data[offset];
	    }
    }
}

// *****************************************************************************
// *
// * Function: UpdateGlobalParallelBits(void);
// *
// * Purpose: Update global parallel AND and OR statusbits
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: All possible UPMs are included.
// *
// *****************************************************************************

void UpdateGlobalParallelBits(void)
{
    // Update global status bits, to drop the old unit off the global AND
    // and OR data blocks.
    static const size_t offset[] = 
    {
        offsetof(UpmState, BypassStatus.words),
        offsetof(UpmState, BatteryStatus.words),
        offsetof(UpmState, McuStatus.words),
        offsetof(UpmState, InverterStatus.words),
        offsetof(UpmState, SyncStatus.words),
        offsetof(UpmState, SystemError.words),
        offsetof(UpmState, ParallelStatus.words),
        offsetof(UpmState, AbmStatus.words)
    };
    
    static const size_t size[] = 
    {
        sizeof(ParallelCan.UpmData[0][0].BypassStatus.words),
        sizeof(ParallelCan.UpmData[0][0].BatteryStatus.words),
        sizeof(ParallelCan.UpmData[0][0].McuStatus.words),
        sizeof(ParallelCan.UpmData[0][0].InverterStatus.words),
        sizeof(ParallelCan.UpmData[0][0].SyncStatus.words),
        sizeof(ParallelCan.UpmData[0][0].SystemError.words),
        sizeof(ParallelCan.UpmData[0][0].ParallelStatus.words),
        sizeof(ParallelCan.UpmData[0][0].AbmStatus.words)
    };
    
    for (int i = 0; i < sizeof(size) / sizeof(size[0]); ++i)
    {
        UpdateGlobalParallelBits(offset[i], size[i]);
    }
}

// ********************************************************************************************************
// *
// * Function: ParallelSystem392us_Function(void);
// *
// * Purpose:  2,5kHz parallel function.
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Called from 5,105Hz ADC ISR
// *
// ********************************************************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void ParallelSystem392us_Function( void )
{
    if ( ParallelCan.GetState() != CanDriver::Initialization )
    {
        SendStatusPackets(true);  
    }
    else 
    {
        // Update global status bits for self.
        UpmState myState;
        
        myState.BypassStatus = BypassState().GetBypassStatus();
        myState.BatteryStatus = BatteryConverter.GetBatteryStateStatus();
        myState.McuStatus = MCUStateMachine.GetStatus();
        myState.InverterStatus = Inverter.GetStatus();
    	myState.SystemError = ParallelCan.GetSystemError();
        myState.AbmStatus = Abm().GetStatus();
        
        myState.SyncStatus.words[SYNC_WORD ] = GetSyncStatus();
        myState.SyncStatus.words[ AUTO_CAL_WORD ] = AutoCal.GetAutoCalStatus();
        
    	myState.ParallelStatus = ParallelCan.GetParallelStatus();
    	
        // Only update the global AND and OR data to be self.  Do not update
        // the UpmData array since my UPS number is not yet defined.
        ParallelCan.ParGlobalAndData = myState;
        ParallelCan.ParGlobalOrData = myState;
    }
}

// ********************************************************************************************************
// *
// * Function: ParallelSystem5Msec(void);
// *
// * Purpose: This module is called at a 5 ms
// *
// * Parms Passed   :   Nothing
// * Returns        :   Nothing
// *
// * Description: Called from periodic Task
// *
// ********************************************************************************************************
uint16_t        debugCan1;
uint16_t        debugCan2 = 0;
void ParallelSystem5Msec(void)
{
    static uint16_t count_1S = 0;
    static uint16_t count_30S = 6000;
    //Update Bypass available after some UPSs is checkedout from parallel system
    ParallelCan.UpdateBypassAvailable();
    // Note: even though the software tx fifo is zero-size, this periodic function
    // is also responsible for packet error handling, and must be called anyway.
    ParallelCan.DrainTxFifo();
    // TODO: This really aught to be in a TransferState() function...
    static bool everOnline = false;
    static uint16_t wCntDlyCanChkPowerOn = 0;	//for jira483
	//for jira484 begin
	static bool bFlagCanBusError = false;
    static uint16_t wCntDlyCanBusError = 0;	
	bool ParallelCanBusError = false;
	//end
    if (ParallelCan.GetState() != CanDriver::Initialization)
    {
        uint16_t observedUpms = 1;      // count ourself
        uint16_t observedUpss = 0;      // we'll count ourself later
        uint16_t checkedoutUpss = 0;    
        uint16_t everObservedUpms = 0;
                
        for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
        {
            for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
            {
                // don't timeout ourself.  We are off the bus only during Bus Off
                if ( ( ups != MyUPSNumber ) || ( upm != MyUPMNumber ) )
                {
                    if ( ParallelCan.UpmData[ ups ][ upm ].Count )
                    {
                        ParallelCan.UpmData[ ups ][ upm ].Timeout = false;
                        ParallelCan.UpmData[ ups ][ upm ].Count--;
                        observedUpms++;
                        if ( !ParallelCan.UpmData[ ups ][ upm ].Count ) debugCan2++;
                    }
                    else
                    {
                        ParallelCan.UpmData[ ups ][ upm ].Timeout = true;
                        ParallelCan.CurrentUPMsOnBus[ ups ] &= ~( 1 << upm );
                    }
                }
                else
                {
                    ParallelCan.UpmData[ ups ][ upm ].Timeout = false;
                }
            }
        }
        
        for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
        {
            for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
            {
                if( ParallelCan.AllUPMsSinceReset[ ups ] & (1 << upm) )
                {
                    ParallelCan.AllUPSsSinceReset |= ( 1 << ups );
                    everObservedUpms++;
                }
                
                if( ParallelCan.CurrentUPMsOnBus[ ups ] != 0 )
                {
                    ParallelCan.CurrentUPSsOnBus |= ( 1 << ups );
                }
                else
                {
                    ParallelCan.CurrentUPSsOnBus &= ~( 1 << ups );
                }                
            }
            
            if( ParallelCan.CurrentUPSsOnBus & ( 1 << ups ) )
            {
                observedUpss++;
            }
            //Jacob/20130916/add begin...
            else if( ParallelCan.UpmData[ups][MASTER_UPM_NUMBER].McuStatus.bit.CheckedOut )
            {
                checkedoutUpss++;
            }
            //Jacob/20130916/...add end
        }
        
            ///
            /// Handle PCAN failure
            /// 
		//For wombat jira483, being
		//1.Can hardware ready to receive,may need 5.5s (RxPacket_TSK(void)->CanHardwareReady()), 
		//2.After step 1,then begin Canfail detect ((CanDriver::Initialization->InitHardware()))
		//for jira483, now change step2:
		//New2 step2:After step 1,wait 3s,then begin CANfail detect;
		if(wCntDlyCanChkPowerOn++ >= 1200)		//1200=6s/5ms
		{
			wCntDlyCanChkPowerOn = 1201;
		}

		if(wCntDlyCanChkPowerOn >= 1200) 	//delay 6sec
		{
			//Jacob/21030916/add checkedoutUpss 		
			ParallelCan.Status.bit.NodeFailed = (observedUpms + checkedoutUpss * NumOfUPMs) != everObservedUpms  ||
												(observedUpss + checkedoutUpss) < NumOfUPSs; // this persists after reset alarms  

			ParallelCanBusError = ParallelCan.BusError();
		}
		else
		{
			ParallelCan.Status.bit.NodeFailed = 0;
			ParallelCanBusError = false;
		}
		

		if(SingleUPSStartUpEnabled)
		{
			if(count_30S)
	        {
			    count_30S--;
			    if ((ParallelCan.TotalNumberOfUPSs==1)&&(count_30S== 0))
			    {
			       NumOfUPSs = 1;
			       PutEepData( Find_Eep_Num( &NumOfUPSs ), 1, &NumOfUPSs, 0 );
			    }
		    }
		}


        if(observedUpss > NumOfUPSs)
        {
            if(count_1S++ > 200)
            {
                count_1S = 0;
                NumOfUPSs = observedUpss;
                PutEepData( Find_Eep_Num( &NumOfUPSs ), 1, &NumOfUPSs, 0 );
            }
        }
        else
        {
            count_1S = 0;
        }
        //Jacob/20130905/...add end
        
        // If we're a single unit, it's okay to be in error passive
        // If everyone is checked out, it's okay to be in error passive
        bool exParallel = ParallelCan.UPSIsParallel() && !((checkedoutUpss+1) == NumOfUPSs);
        bool inParallel = EEP_IsInternalParallel();

        // don't wait to see other UPM's before issuing CAN failed.
        // Auto standby is now capable of handling a temporary CAN failed alarm
        // A single 30 with NumUPSs set > 1 needs to issue CAN failed
        bool initialError = NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR);

		//debounce: 1*5ms, 1000*5ms
        bool finalError = NB_DebounceAndQue( UPM_NB_PARALLEL_CAN_ERROR,
                        ( ParallelCan.Status.bit.NodeFailed || (ParallelCanBusError && (exParallel || inParallel)) ),
                          (ParallelCan.Status.bit.NodeFailed +
                          ((ParallelCanBusError && (exParallel || inParallel))<<1)));
		
        if (initialError && !finalError)
        {
        	// PCAN failure has cleared, resend all status bits
        	SendStatusPackets(false, true);
        }
        
        if ( everObservedUpms > 1 )
        {
            if (!everOnline)
            {
            	// First word from another node, initialize their cache with initial status bits
            	SendStatusPackets(false, true);
            	everOnline = true;
            }
        }
        else
    	{
    		everOnline = false;
    	}
        
        
          ///
          /// Auto-ID
          ///
        if( !ParallelCan.ParallelStatus.bit.AutoID && !DisableAutoID )
        {
            if( !ParallelCan.ParallelStatus.bit.ActiveNode )
            {
                MyUPSNumber = 0;
            }
            else
            {
                for( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
                {
                    if( !( ParallelCan.CurrentUPSsOnBus & ( 1 << ups ) ) )    
                    {
                        MyUPSNumber = ups;
                        break;
                    }    
                }      
            }                    
        }
        else
        {
            ParallelCan.AllUPMsSinceReset[ MyUPSNumber ] |= ( 1 << MyUPMNumber );
            ParallelCan.CurrentUPMsOnBus[ MyUPSNumber ]  |= ( 1 << MyUPMNumber );
        }
    }
}

//*********************************************************************
//
//  Name    : ParallelSystem 20 Millisecond Task
//
//
//  Abstract: Called from periodic SWI
//
//
//
//*********************************************************************
void ParallelSystem20Msec(void)
{
    if ( ParallelCan.GetState() != CanDriver::Initialization )
    {
        ParallelCan.PCan_UpdateSystemErrorStatus();
        AutoCal.RunAutoCal();
        ParallelCan.UpdateInverterAvailable();
    }
}

// ***********************************************************************
// *
// *    FUNCTION:  
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ParallelSystem100Msec(void)
{
    if (ParallelCan.GetState() != CanDriver::Initialization)
    {
            // Prevent timeout on other nodes by transmitting at 5x the timeout rate
        SendStatusPackets(false);
        
        // count the number of UPMS
        uint16_t observedUpms = 0;      // count ourself
        uint16_t observedUpmsThisUPS = 0;
        uint16_t observedBypass = 0;
        uint16_t everObservedUpms = 0;
        uint16_t observedUpmsOnLine = 0;
        uint16_t observedUpss = 0;
            
        for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
        {
            for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
            {
                if ( ParallelCan.CurrentUPMsOnBus[ ups ] & ( 1 << upm ) )
                {
                    observedUpms++;
                    observedBypass++;
                    observedUpmsThisUPS++;
                    
                    if( ParallelCan.UpmData[ups][upm].InverterStatus.bit.InverterOn )
                    {
                        observedUpmsOnLine++;
                    }    
                }
                if (ParallelCan.AllUPMsSinceReset[ups] & (1 << upm))
                {
                    everObservedUpms++;
                }
                
                //use "UPMsOnBus" instead of "Timeout" for check --> move to up
                //if ( !ParallelCan.UpmData[ups][upm].Timeout && ParallelCan.UpmData[ups][upm].InverterStatus.bit.InverterOn )
                //{
                //    observedUpmsOnLine++;
                //}       
            }
            
            if( ParallelCan.CurrentUPSsOnBus & ( 1 << ups ) )
            {
                observedUpss++;
            }
            
            ParallelCan.NumberOfUPMs[ ups ] = observedUpmsThisUPS;
            observedUpmsThisUPS = 0;
        } 
        
        ParallelCan.TotalNumberOfUPMs = observedUpms;
        ParallelCan.TotalNumberOfBypass = observedBypass;
        ParallelCan.TotalNumberOfUpmsOnline = observedUpmsOnLine;
        ParallelCan.TotalNumberOfUPSs = observedUpss;
        //ParallelCan.Status.bit.NodeFailed = observedUpms != everObservedUpms;
        // Discussion is ongoing about this semantic change
        //ParallelCan.Status.bit.NodeFailed = ( NumOfUPMs != ParallelCan.NumberOfUPMs[ MyUPSNumber ] ) ;
        
        NB_DebounceAndQue( UPM_NB_EXTERNAL_PARALLEL, ParallelCan.UPSIsParallel(), ParallelCan.CurrentUPSsOnBus );
        NB_DebounceAndQue( UPM_NB_SYSTEM_IS_REDUNDANT, ParallelCan.PCan_CheckSystemRedundant() );
        
        ParallelCan.TransmitBroadcastPacket(pcan::loadshare_meter, (uint16_t)ScreenMeters.PercentLoad.sum, 
                                                                   (uint16_t)ScreenMeters.PercentLoad.phA, 
                                                                   (uint16_t)ScreenMeters.PercentLoad.phB, 
                                                                   (uint16_t)ScreenMeters.PercentLoad.phC, true);
        ParallelCan.LoadData[MyUPSNumber][MyUPMNumber].sum= (uint16_t)ScreenMeters.PercentLoad.sum;
        ParallelCan.LoadData[MyUPSNumber][MyUPMNumber].phA= (uint16_t)ScreenMeters.PercentLoad.phA;
        ParallelCan.LoadData[MyUPSNumber][MyUPMNumber].phB= (uint16_t)ScreenMeters.PercentLoad.phB;
        ParallelCan.LoadData[MyUPSNumber][MyUPMNumber].phC= (uint16_t)ScreenMeters.PercentLoad.phC;
        ParallelCan.UpdateSystemLoad();
        ParallelCan.ProcessBatteryTemperature();
    }
}

//*********************************************************************
//
//  Name    : ParallelSystem500MsecTask
//
//
//  Abstract: This module is called at a 500 Msec rate. Called from periodic SWI
//
//
//
//********************************************************************/
void ParallelSystem500Msec(void)
{
    if (ParallelCan.GetState() != CanDriver::Initialization)
    {
        ParallelCan.PCan_ProcessC9Command();
        ParallelCan.PCan_ProcessInternalCommand();
        ParallelCan.PCan_ProcessParallelCommand();
    }
}

//*********************************************************************
//
//  Name    : ParallelSystem1secTask
//
//
//  Abstract: This module is called at a 1 sec rate. Called from periodic SWI
//
//
//
//*********************************************************************
void ParallelSystem1000Msec(void)
{
    static int twenty_sec_divide = 0;

    if (ParallelCan.GetState() != CanDriver::Initialization)
    {
        if (twenty_sec_divide ++> 20)
        {
            twenty_sec_divide = 0;
            
            if (MyUPMNumber == 0 &&
                MyUPSNumber == 0)
            {
                ParallelCan.TransmitBroadcastPacket(
                    pcan::clock_sync, 
                    RTC_SysTime.mSecOfMinute,
                    RTC_SysTime.MinuteOfMonth,
                    RTC_SysTime.YearAndMonth, 0, true);
            }           
        }
        
        ParallelCan.PCan_CheckMaster();
        if ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() )
        {
            ParallelCan.PCan_SyncSharedParams();
            ParallelCan.PCan_CheckSharedParams();
        }

        if( EepResetNoSyncTimer > 0 )
        {
            EepResetNoSyncTimer--;
        }

        if ( MCUStateMachine.GetState() > INITIALIZATION_STATE )
        {
            NB_DebounceAndQue(UPM_NB_REDUNDANCY_LOSS_DUE_TO_OVERLOAD, ParallelCan.UPSIsParallel() &&
                                                                      EEP_ParallelForRedundancy() && 
                                                                      (ONLINE_STATE == MCUStateMachine.GetState()) &&
                                                                      !NB_GetNodebit(UPM_NB_SYSTEM_IS_REDUNDANT));
        }
        
        static bool ParallelUPSHistory = 0;
        if( ParallelUPSHistory != ParallelCan.UPSIsParallel() )
        {
            EE_ID* ee = GetParameterEE( PARAM_InverterSlewRate );
            uint16_t data = ee->eep_DefaultValue[CoefficientsIndex];
            Inverter.EEFunc_Inverter( ee, &data );
        } 
        ParallelUPSHistory = ParallelCan.UPSIsParallel();
        
        //Jacob/20140801/add for parallel load share error
        UpdateParallelLoadShareError();
    }

    
}

// ***********************************************************************
// *
// *    FUNCTION: UpdateParallelLoadShareError 
// *
// *    DESCRIPTION: Update Parallel Load Share Error
// *                 load share error if 
// *                 this UPM phase load - average phase load > limit
// *                 
// *    ARGUMENTS: None
// *
// *    RETURNS: None
// *
// ***********************************************************************
void UpdateParallelLoadShareError(void)		//1s task
{
    float LoadAveragePhA = 0.0f;
    float LoadAveragePhB = 0.0f;
    float LoadAveragePhC = 0.0f;
    
    uint16_t errorcode = 0;
    uint16_t upmSumOnInv = ParallelCan.PCan_UPMSumOnInverter();
    
    if( ( EEP_IsInternalParallel() || ParallelCan.UPSIsParallel() ) &&
        !NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR)                   &&
        ( MCUStateMachine.GetState() != ESS_MODE_STATE )            &&
        ( upmSumOnInv > 1 )                                         &&
		MCUStateMachine.MCUStatus.bit.BldInpMOBClosed               &&
        (MCUStateMachine.GetState() == ONLINE_STATE ) &&
        !NB_GetNodebit(UPM_NB_MOB_OPEN) )
    {

        LoadAveragePhA = float( ParallelCan.SysLoadData.phA ) / float( upmSumOnInv );
        LoadAveragePhB = float( ParallelCan.SysLoadData.phB ) / float( upmSumOnInv );
        LoadAveragePhC = float( ParallelCan.SysLoadData.phC ) / float( upmSumOnInv );
        
        //All the load are in percent, we don't need to cal percent again. 
        ParallelCan.ParallelSystemError.bit.err_load_share_PhA = ( std::fabs( ScreenMeters.PercentLoad.phA - LoadAveragePhA ) 
                                                                        >= ParaLoadShareErrorLimit? 1 : 0 );
                                                                            
        ParallelCan.ParallelSystemError.bit.err_load_share_PhB = ( std::fabs( ScreenMeters.PercentLoad.phB - LoadAveragePhB ) 
                                                                        >= ParaLoadShareErrorLimit? 1 : 0 );
                                                                            
        ParallelCan.ParallelSystemError.bit.err_load_share_PhC = ( std::fabs( ScreenMeters.PercentLoad.phC - LoadAveragePhC ) 
                                                                        >= ParaLoadShareErrorLimit? 1 : 0 );
                   
        if( ParallelCan.ParallelSystemError.bit.err_load_share_PhA )
        {
            errorcode |= 0x01;
        }

        if( ParallelCan.ParallelSystemError.bit.err_load_share_PhB )
        {
            errorcode |= 0x02;
        }
        
        if( ParallelCan.ParallelSystemError.bit.err_load_share_PhC )
        {
            errorcode |= 0x04;
        }
        
        NB_DebounceAndQue(UPM_NB_PARALLEL_LOAD_SHARE_ERROR, errorcode, errorcode);
       
    }
    else
    {
        NB_DebounceAndQue(UPM_NB_PARALLEL_LOAD_SHARE_ERROR, false);
    }
}
 

//*********************************************************************
//
//  Name    : ParallelSystem1secTask
//
//
//  Abstract: This module is called at a 1 sec rate. Called from periodic SWI
//
//
//
//*********************************************************************
void ParallelSystemInitializeParams(void)
{
}

void InitUpsId( void )
{
    if( !DisableAutoID )
    {
        MyUPSNumber = 0;
    }    
}

// ***********************************************************************
// *
// *    FUNCTION: ee_update_parallel 
// *
// *    DESCRIPTION: ee function for parallel params
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ee_update_parallel( const EE_ID* ee, const uint16_t* data )
{
    switch ( ee->paramNum )
    {
        case PARAM_MyUPSNumber:
            if ( *data <= ParallelCanNetwork::MAX_NUM_UPS )
            {
                MyUPSNumber = *data;
            }
            else
            {
                MyUPSNumber = 0;
                EEStatusBits.bit.ParallelSetupError = 1;
            }
            break;

        case PARAM_NumOfUPMs:
            if ( *data <= ParallelCanNetwork::MAX_NUM_UPM )
            {
                NumOfUPMs = *data;
                if ( NumOfUPMs > 1 )
                {
                    // call init slew rate, so inverter has properly frequency/phase gains for parallel. Required because this eep is at the end
                    // of the map, inverter pll has already been initialized as a single UPM.
                    // don't actually have to read the eep, since it is ignored for parallel, just call the function
                    EE_ID* ee = GetParameterEE( PARAM_InverterSlewRate );
                    uint16_t data = ee->eep_DefaultValue[CoefficientsIndex];
                    Inverter.EEFunc_Inverter( ee, &data );
                }
            }
            else
            {
                NumOfUPMs = 0;
                EEStatusBits.bit.ParallelSetupError = 1;
            }
            break;

        default:
            break;
                              
    }
}

// ********************************************************************************************************
// *            END OF Parallelfuncs.c
// ********************************************************************************************************

