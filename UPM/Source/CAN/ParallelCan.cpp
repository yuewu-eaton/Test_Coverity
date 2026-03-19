#include "DSP28x_Project.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
//#include "BypassSlave.h"
#include "Spi_Task.h"
#include "InverterControl.h"
#include "RectifierStateControl.h"
#include "MCUState.h"
#include "Abm.h"
#include "DebuggerBlocks.h"
#include "Rtc.h"
#include "Alarms_AC.h"
#include "CanIds.h"
#include "InternalCan.h"

#include <cstring>
using std::memcpy;

#include <algorithm>
using std::lower_bound;

ParallelCanNetwork ParallelCan;



namespace {

bool PacketValid(const ParallelCanPacket& packet)
{   
    uint_least8_t src_upm = packet.tx_upm();
    uint_least8_t src_ups = packet.tx_ups();
    uint_least8_t dst_ups = packet.rx_ups();
    uint_least8_t dst_upm = packet.rx_upm();
    bool valid = false;
    
    // UPM in range
    if (src_upm < ParallelCanNetwork::MAX_NUM_UPM && 
        src_ups < ParallelCanNetwork::MAX_NUM_UPS &&
        ( src_upm != MyUPMNumber ||
          src_ups != MyUPSNumber )                    &&
        // Directed to this node.
        (dst_upm == 0xf || dst_upm == MyUPMNumber) &&
        (dst_ups == 0xf || dst_ups == MyUPSNumber) )
    {
        valid = true;
        // NOT critical, this function does not need to be re-entrant since it's only called in from 1 task
        ParallelCan.UpmData[src_ups][src_upm].Count = ParallelCanNetwork::UPM_PACKET_TIMEOUT_COUNT;
        ParallelCan.AllUPMsSinceReset[ src_ups ] |= ( 1 << src_upm );
        ParallelCan.CurrentUPMsOnBus[ src_ups ] |= ( 1 << src_upm );
        ParallelCan.ParallelStatus.bit.ActiveNode = 1;
    }
    return valid;
}

} // !namespace (anon)

ParallelCanNetwork::ParallelCanNetwork() 
    : CanDriver(CanDriver::ECANB, 20, 30, 0)
    , SystemLoad(0.0f)
    , SystemBypassAvailable(1)
    , SystemInverterAvailable(1)
{
    for (int ups = 0; ups < MAX_NUM_UPS; ++ups)
    {
        for (int upm = 0; upm < MAX_NUM_UPM; ++upm)
        {
            UpmState& state = UpmData[ups][upm];
            state.BypassStatus.words[0] = 0;
            state.BypassStatus.words[1] = 0;
            state.BypassStatus.words[2] = 0;
            state.BatteryStatus.all = 0;
            state.McuStatus.words[0] = 0;
            state.McuStatus.words[1] = 0;
            state.McuStatus.words[2] = 0;
            state.McuStatus.words[3] = 0;	//new add
            state.InverterStatus.all = 0;
            state.SyncStatus.all = 0;
            state.SystemError.all = 0;
            state.ParallelStatus.all = 0;
            state.Count = 0;
            state.Timeout = true;
        }
        
        ParallelCan.AllUPMsSinceReset[ ups ] = 0;
        ParallelCan.CurrentUPMsOnBus[ ups ] = 0;
        NumberOfUPMs[ ups ] = 0;
        BatteryTemperature[ups] = 250;  //means 25degree
    }
     
    Status.bit.NodeFailed = false;
    Status.bit.ParallelUpsInitializing = false;
    ParallelSystemError.all = 0x0000;
    TotalNumberOfUPMs = 0;
    TotalNumberOfBypass = 0;
    TotalNumberOfUpmsOnline = 0;
    AllUPSsSinceReset = 0x0000;
    CurrentUPSsOnBus = 0x0000;
    TotalNumberOfUPSs = 0;
    MaxBatTempInCommon = 250;   //means 25degree

//  ParallelParamsError = 0;
    ParallelStatus.all = 0;
    ParallelCommand.all = 0;
    InternalCommand.all = 0;
    C9Command.all = 0;
    /* synchronous */
    parallelSynchFlags = (PARALLEL_SYNCH_FLAG_IDENTIFY_MASTER | PARALLEL_SYNCH_FLAG_SCAN_INVERTER |
    					  PARALLEL_SYNCH_FLAG_BYPASS_NOT_AVAILABLE);
}

ParallelCanNetwork::~ParallelCanNetwork()
{
    // Default destructor
}

uint_least8_t ParallelCanNetwork::UPMNumber()
{
    return MyUPMNumber;
}

void ParallelCanNetwork::ResetSticky(void)
{
    // For each UPM that was observed in the past, but is now off the network,
    // clear its data out of the UpmData array and clear it off of the SinceReset
    // array
    uint_least8_t DroppedUPMs[MAX_NUM_UPS] = {0};
    uint_least8_t PresentUPMs[MAX_NUM_UPS] = {0};

	// TODO: TSK_disable()???  WTF, over?
    TSK_disable();
    memcpy(DroppedUPMs, AllUPMsSinceReset, sizeof(DroppedUPMs));
    memcpy(PresentUPMs, CurrentUPMsOnBus, sizeof(PresentUPMs));
    TSK_enable();
    // Which dropped UPM's are cleared is defined by the state of the presence
    // bitmaps as observed this instance.  Additional UPM's may be dropped
    // while this call is processing, but they will not be affected.
    
    for (int i = 0; i < MAX_NUM_UPS; ++i)
    {
        // Yes, XOR does the trick.  Truth table:
        // SinceReset | OnBus | Dropped
        // ------------------------
        // 1          | 1     | 0
        // 0          | 0     | 0
        // 1          | 0     | 1
        // 0          | 1     | Impossible
        DroppedUPMs[i] ^= PresentUPMs[i];
    }
    
    bool dirty = false;
    
    for (int ups = 0; ups < MAX_NUM_UPS; ++ups)
    {
        for (int upm = 0; upm < MAX_NUM_UPM; ++upm)
        {
            if (DroppedUPMs[ups] & (1 << upm))
            {
                dirty = true;
                
                TSK_disable();
                memset(&UpmData[ups][upm], 0, sizeof(UpmData[0][0]));
                UpmData[ups][upm].Timeout = true;
                AllUPMsSinceReset[ups] &= ~(1 << upm);
                TSK_enable();
            }
        }
    }
    
    if (dirty)
    {
        UpdateGlobalParallelBits();
        Status.bit.NodeFailed = 0;
    }
    AllUPSsSinceReset = 0;
}

/*
 * Global predicate for CAN bus failures.  One or more node is offline if this
 * is true.  If we have entered the Error Passive state, then we are the failed node.
 */
bool ParallelCanNetwork::BusError()
{
    bool pcan_fail = false;
    
    if(GetState() == Passive)
    {
        pcan_fail = true;
    }
    
    return pcan_fail;    
}

bool ParallelCanNetwork::CanHardwareReady(void)
{
    return MCUStateMachine.GetADCReadyFlag() && 
    	MyHardwareNumber != 0 && 
    	EEStatusBits.bit.EEDataInitialized;
}

void ParallelCanNetwork::ConfigureRxMailboxes(void)
{
    // WARNING: All of the mailboxes are allocatd to Tx or Rx.  If an additional
    // RX mailbox is configured, the tx mailbox count must be reduced.
    AllUPMsSinceReset[ MyUPSNumber ] |= ( 1 << MyUPMNumber );
    CurrentUPMsOnBus[ MyUPSNumber ]  |= ( 1 << MyUPMNumber );
    UpmData[ MyUPSNumber ][ MyUPMNumber ].Timeout = false;
    
    RxPackets(&ReceiveStatusPacket, NULL, 
              pcan::type_mask(pcan::status_mask),
              pcan::type_mask(pcan::status_base), 2);
    RxPackets(&ReceiveInfoPacket, NULL,
              pcan::type_mask(pcan::info_mask),
              pcan::type_mask(pcan::info_base),2);
    RxPackets(&ReceiveCommandPacket, NULL,
              pcan::type_mask(pcan::command_mask),
              pcan::type_mask(pcan::command_base),1);
    RxPackets(&ReceiveSyncPacket, NULL,
              pcan::type_mask(pcan::sync_mask),
              pcan::type_mask(pcan::sync_type),1);

    RxPackets(&ReceiveRTCPacket, NULL,
              pcan::type_mask(pcan::rtc_mask), 
              pcan::type_mask(pcan::rtc_base),1);
}

/******************** Packet reception functions ******************************/
void ParallelCanNetwork::ReceiveInfoPacket(const MBOX& m_packet, const void *closure)
{
    // handle each one separately
    // cases include clock sync, machine id, and parameter checksum
    const ParallelCanPacket& packet = static_cast<const ParallelCanPacket&>(m_packet);
    
    if (PacketValid(packet))
    {
        switch (packet.type())
        {
            case pcan::inv_cal_meters0:
                ParallelCan.CalData[packet.tx_ups()][packet.tx_upm()].id  = packet.data0();
                ParallelCan.CalData[packet.tx_ups()][packet.tx_upm()].powerA = packet.data1();
                ParallelCan.CalData[packet.tx_ups()][packet.tx_upm()].powerB = packet.data2();
                ParallelCan.CalData[packet.tx_ups()][packet.tx_upm()].powerC = packet.data3();
                break;
                
            case pcan::loadshare_meter:
                ParallelCan.LoadData[packet.tx_ups()][packet.tx_upm()].sum  = packet.data0();
                ParallelCan.LoadData[packet.tx_ups()][packet.tx_upm()].phA  = packet.data1();
                ParallelCan.LoadData[packet.tx_ups()][packet.tx_upm()].phB  = packet.data2();
                ParallelCan.LoadData[packet.tx_ups()][packet.tx_upm()].phC  = packet.data3();
                ParallelCan.UpdateSystemLoad();
                break;   
                 
            case pcan::bypass_power_meter: 
                // nothing to do                
                break;
            case pcan::shared_params:
                 PCan_UpdateSharedParams( m_packet );
                 break;
				 
            case pcan::master_output_freq:  
                 ParallelCan.MasterOutputFrequency = float(packet.data0())/ 1000.0;
                 break;

            case pcan::csb_meters:
                 if( packet.data3() == ( CAN_CRC_NONCE ^ packet.data0() ^ packet.data1() ^ packet.data2() ) )
                 {
                     ParallelCan.BatteryTemperature[packet.tx_ups()] = (packet.data0() <= 500) ? packet.data0() : 500;
                 }
                 break;
            default:
                 break;
                
        };
    }
}

void ParallelCanNetwork::ReceiveCommandPacket(const MBOX& m_packet, const void *closure)
{
    // Feed into appropriate state machine
    const ParallelCanPacket& packet = static_cast<const ParallelCanPacket&>(m_packet);
    
    if ( PacketValid( packet )                &&
         ( !NB_GetNodebit( UPM_NB_MOB_OPEN )    ||
           packet.tx_ups() == MyUPSNumber ) )
    {
    	// Only recieve from other UPM's in your own UPS when your MOB is open, disregard other packets
    	
        switch (packet.type())
        {
            case pcan::bypass_command:
                 if (packet.rx_upm() == 0 && MyUPMNumber == 0)
                 {
                     // Defensive checks.  They should always be true, but help
                     // to defined against a network-driven loop.
                     BypassState().RequestBypassState(bypass_states_t(packet.data0()));
                 }
                 break;
                    
            case pcan::master_rms_command:
                 Inverter.SetMasterRMSData( packet.MDL.word.LOW_WORD, packet.MDL.word.HI_WORD, packet.MDH.word.LOW_WORD );
                 break;    
                    
            case pcan::mcu_command:
            case pcan::auto_id_command:
            case pcan::csb_forward_command:
                 break;
                 
            // Parallel Commands
            case pcan::para_bypass_on_command:
                 if( ParallelCan.ParallelStatus.bit.Master )
                 {
                     ParallelCan.ParallelCommand.bit.para_bypass_on_command = 1;
                 }
                 break;
                 
            case pcan::para_ups_normal_commmand:
                 if( ParallelCan.ParallelStatus.bit.Master )
                 {
                     ParallelCan.ParallelCommand.bit.para_on_normal_command = 1;
                 }   
				 MCUStateMachine.ParalSystemOn = true;					 
                 break;
                 
            case pcan::para_ups_load_off_command:
//	                 MCUStateMachine.StandbyCommand();
			  	 MCUStateMachine.Turn_UPS_Off();
                 break;
                 
            case pcan::para_activate_charger_command:
                 if( ParallelCan.ParallelStatus.bit.Master )
                 {
                     ParallelCan.ParallelCommand.bit.para_activate_charger_command = 1;
                 }   
                 break;
                 
            case pcan::para_deactivate_charger_command:
                 if( MCUStateMachine.InStandby() )
                 {
                     MCUStateMachine.StandbyChargerOffCommand();   
                 }
                 Abm().ChargerCmdOff();
                 break;
                 
            case pcan::para_start_battery_test_command:
                 if( ParallelCan.ParallelStatus.bit.Master )
                 {
                    ParallelCan.ParallelCommand.bit.para_start_battery_test_command = 1;
                 }
                 break;
                 
            // para_eco_on only sent by a master node to the slaves
            case pcan::para_eco_on_command:
           		 MCUStateMachine.EcoOnCommand();
          		 ESSEnabled = 1;
                 PutEepData( Find_Eep_Num(&ESSEnabled), 1, &ESSEnabled, 0 );
                 break;
                 
            case pcan::para_eco_off_command:
                 MCUStateMachine.EcoOffCommand();
                 ESSEnabled = 0;
                 PutEepData( Find_Eep_Num(&ESSEnabled), 1, &ESSEnabled, 0 );
                 break;
                 
            case pcan::para_reset_alarms_command:
                 ResetStickyAlarms();
                 break;
                 
            case pcan::para_clear_history_command:
                 // no need to sync,UPM can handle this command in internal communication
                 break;
                 
            case pcan::para_ups_to_battery_command:
                 // no need to sync at present,comments here to leave a mark for future 
                 break;
                 
            case pcan::para_ups_from_battery_command:
                 // no need to sync at present,comments here to leave a mark for future
                 break;

            case pcan::para_ups_decline_start_command:
                 break;
            
            // System Commands
            case pcan::sys_ups_normal_commmand:
                 MCUStateMachine.NormalCommand();
				 MCUStateMachine.ParalSystemOn = true;				 
                 break;
            
            case pcan::sys_start_battery_test_command:
                 // no need to sync at present,comments here to leave a mark for future
                 break;
                 
            case pcan::sys_eco_on_command:
            	if (ParallelCan.ParallelStatus.bit.Master)
            	{
            		// Command was forwarded by a non-master node zero.  Process it exactly
            		// like a CSB command from my CSB
            		ParallelCan.InternalCommand.bit.csb_eco_on_command = 1;
            	}
                break;
            
            case pcan::sys_activate_charger_command:
                 Abm().ChargerCmdOn();
                 if( ( MCUStateMachine.GetState() == SHUTDOWN_STATE ) || 
                     ( MCUStateMachine.GetState() == BYPASS_STATE ) )
                 {
                     MCUStateMachine.AutoStandbyCommand();                    
                 }
                 break;
            
            case pcan::sys_bypass_on_command:
                 MCUStateMachine.BypassCommand();
                 break;
                 
            // Internal Sync Commands
            case pcan::fw_inverter_off_sync_command:
                 // Online UPM's need to ETB if available; load off if not.
                 // Offline UPM's need to ETB if available; clear normal command if not. 
                 ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 1;
                 break;
                 
            case pcan::fw_redundant_off_command:
                 // UPM responds to this parallel command only when
                 // the inverter of this upm is supporting the load
                 if ( MCUStateMachine.InvSupportingLoad() )
                 {
                 	ParallelCan.ParallelCommand.bit.para_redundant_off = 1;
                 }
                 break;
                 
            case pcan::fw_redundant_battery_off_command:
                 // UPM responds to this parallel command only when
                 // the inverter of this upm is supporting the load
                 if ( MCUStateMachine.InvSupportingLoad() &&
                      packet.rx_ups() == MyUPSNumber )
                 {
                 	ParallelCan.ParallelCommand.bit.para_redundant_battery_off = 1;
                 }
                 break;
            
            case pcan::fw_clear_commands:
                 MCUStateMachine.ClearAllCommands();
                 break;
                 
            case pcan::para_ups_ect_on_command:
                if( ParallelCan.ParallelStatus.bit.Master )
                {
                    ParallelCan.ParallelCommand.bit.para_ect_on_command = 1;
                }
                break; 
 
            case pcan::sys_ups_ect_on_command:
                if( !MCUStateMachine.ECTCommand( ) )
                {
                	if( ParallelCan.ParallelStatus.bit.Master )
        			{
            			MCUStateMachine.ECTPhase = 10;
            			ParallelCan.TransmitCommandPacket( pcan::sys_ups_ect_off_command); 
        			}
        			else
        			{
            			ParallelCan.TransmitCommandPacket( pcan::para_ups_ect_off_command);
        			}
                }
                break;

            case pcan::para_ups_ect_off_command:
                if( ParallelCan.ParallelStatus.bit.Master )
                {
                    ParallelCan.ParallelCommand.bit.para_ect_off_command = 1;
                }
                break;

            case pcan::sys_ups_ect_off_command:
                MCUStateMachine.ECTPhase = 10;
                break;

            case pcan::sys_ups_shut_down_command:
                MCUStateMachine.Turn_UPS_Off( );
                break;

            case pcan::sys_ups_forward_transfer_command:
                // Another UPS wants to forward transfer
                MCUStateMachine.ParaForwardTransfer();
                break;

            default:
                 break;
                
        };
    }
}

// ********************************************************************************************************
// *
// * Function: ReceiveRTCPacket
// *
// * Purpose:  Receive RTC set or sync request. Forward to CSB over ICAN.
// *
// *
// * Parms Passed   :   box:     Packet received over CAN.
// *
// * Returns        :   Nothing
// *
// ********************************************************************************************************
void ParallelCanNetwork::ReceiveRTCPacket(const MBOX& packet, const void *closure)
{
    const ParallelCanPacket& m_packet = static_cast<const ParallelCanPacket&>(packet);
    uint16_t packetType = m_packet.type();

	if(MyUPMNumber == 0)
	{
		//transform the external packet to internal packet.
		InternalCan.PacketTx( InternalCanPacket( packetType, CAN_PRIORITY_LOW )
							  .data0( m_packet.data0() )
							  .data1( m_packet.data1() )
							  .data2( m_packet.data2() )
							  .data3( m_packet.data3() ), CAN_RTC_SYNC == packetType );
	}			
}

namespace {
void CopyStatusBits(uint16_t dstAddr[], uint16_t len, const ParallelCanPacket& packet)
{
	uint16_t lastSequenceNumber = (dstAddr[3] & 0xf000u) >> 12;
	uint16_t thisSequenceNumber = (packet.data3() & 0xf000u) >> 12;
	if (thisSequenceNumber != 0 && lastSequenceNumber != 0)
	{
		uint16_t nextSequenceNumber = lastSequenceNumber + 1;
		if (nextSequenceNumber == 16)
			nextSequenceNumber = 1;
		
		uint16_t priorSequenceNumber = lastSequenceNumber - 1;
		if (priorSequenceNumber == 0)
			priorSequenceNumber = 15;
		
		
		uint16_t secondPriorSequenceNumber = priorSequenceNumber - 1;
		if (secondPriorSequenceNumber == 0)
			secondPriorSequenceNumber = 15;
		
		if (thisSequenceNumber == priorSequenceNumber ||
			thisSequenceNumber == secondPriorSequenceNumber)
		{
			// Packet came from the past, reject it.
			return;
		}
		else if (thisSequenceNumber != nextSequenceNumber)
		{

		}
	}
	else
	{
		// sequence packet zero is always accepted
	}
	
	// Accept the data.
    switch (len)
    {
        case 4 :
            dstAddr[3] = packet.data3();
            //lint -fallthrough
            
        case 3 :
            dstAddr[2] = packet.data2();
            //lint -fallthrough
            
        case 2 :
            dstAddr[1] = packet.data1();
            //lint -fallthrough
            
        case 1 :
            dstAddr[0] = packet.data0();
            break;
            
        default:
            do {} while (false);
            break;
    };
}
} // !namespace (anon)

void ParallelCanNetwork::ReceiveStatusPacket(const MBOX& box, const void *closure)
{
    // Save the status bits
    // increment the node count
    const ParallelCanPacket& packet = static_cast<const ParallelCanPacket&>(box);
    uint16_t type = packet.type();
    uint16_t CountReadyUPMs = 0;
        
    if (PacketValid(packet))
    {
        uint16_t* baseAddr = NULL;
        uint16_t len = 0;
        uint16_t offset = 0;
        switch (packet.type())
        {
            case pcan::bypass_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].BypassStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].BypassStatus.words);
                 offset = offsetof(UpmState, BypassStatus.words);
                 break;
            
            case pcan::battery_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].BatteryStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].BatteryStatus.words);
                 offset = offsetof(UpmState, BatteryStatus.words);
                 break;
                 
                
            case pcan::mcu_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].McuStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].McuStatus.words);
                 offset = offsetof(UpmState, McuStatus.words);
                 break;
                
            case pcan::inverter_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].InverterStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].InverterStatus.words);
                 offset = offsetof(UpmState, InverterStatus.words);
                 break;
                
                
            case pcan::sync_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].SyncStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].SyncStatus.words);
                 offset = offsetof(UpmState, SyncStatus.words);
                 break;
            
            case pcan::system_error:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].SystemError.words;
                 len = sizeof(ParallelCan.UpmData[0][0].SystemError.words);
                 offset = offsetof(UpmState, SystemError.words);
                 break;
            
            case pcan::parallel_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].ParallelStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].ParallelStatus.words);
                 offset = offsetof(UpmState, ParallelStatus.words);
                 break;
                 
            case pcan::abm_status:
                 baseAddr = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].AbmStatus.words;
                 len = sizeof(ParallelCan.UpmData[0][0].AbmStatus.words);
                 offset = offsetof(UpmState, AbmStatus.words);
                 break;
                 
            default:
                 // Unknown packet type error
                 do { } while (false);
        }
        // Atomically update the status bits
        if (baseAddr != NULL)
        {
            bool MOBInitiallyClosed = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].McuStatus.bit.BldInpMOBClosed;
            CopyStatusBits(baseAddr, len, packet);
            bool MOBFinallyClosed = ParallelCan.UpmData[packet.tx_ups()][packet.tx_upm()].McuStatus.bit.BldInpMOBClosed;
            
            if (packet.type() == pcan::mcu_status)
            {
                ParallelCan.UpdateBypassAvailable();
                ParallelCan.UpdateSystemLoad();
            }
            if (MOBInitiallyClosed != MOBFinallyClosed)
            {
                // Assume all bits dirty as a unit has either entered or left the set
                UpdateGlobalParallelBits();
            }
            else
            {
                // Only update bits for the status packet that changed.
                UpdateGlobalParallelBits(offset, len);
            }
        }

        for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
        {
            for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
            {
                if( ParallelCan.UpmData[ups][upm].McuStatus.bit.SyncStartReady )
                {
                    CountReadyUPMs++;
                }
            }
        }
        ParallelCan.ReadyUPMs = CountReadyUPMs;
    }
}

void ParallelCanNetwork::ReceiveSyncPacket(const MBOX& packet, const void *closure)
{
}

/************************************ Packet Transmit functions ***************/

/*
 * Override the base class definition of PacketTx, but only for the parallel CAN
 * bus.  Work that is specifc to PCAN, but common to all transmitted packets on PCAN
 * should go here.
 */
void ParallelCanNetwork::PacketTx(const MBOX& packet, bool periodic)
{
    if( ParallelCan.ParallelStatus.bit.AutoID || DisableAutoID )
    {
        // Reset own timeout counter.
        MyUpmData().Count = UPM_PACKET_ALIVE_COUNT;
        // Remaining behavior is the default behavior.
        CanDriver::PacketTx(packet, periodic);
    }
}

void ParallelCanNetwork::TransmitCommandPacket(uint16_t subword, uint16_t data)
{
    ParallelCanPacket packet(subword, 1);
    packet.data0(data);
    if (NB_GetNodebit( UPM_NB_MOB_OPEN))
    {
        // Only transmit command packets to other UPM's in your own UPS when your MOB is open
        packet.rx_ups(MyUPSNumber);
    }
    PacketTx(packet, false);
}

void ParallelCanNetwork::TransmitBroadcastPacket(uint16_t subword, const uint16_t data[], uint16_t len, bool periodic)
{
//  PacketTx( ParallelCanPacket(subword, len) .data(data, len));
    PacketTx( ParallelCanPacket(subword, 4) .data(data, 4), periodic);
}

void ParallelCanNetwork::TransmitBroadcastPacket(uint16_t subword, uint16_t data0,uint16_t data1,uint16_t data2, uint16_t data3, bool periodic)
{
    ParallelCanPacket packet( subword, 4 );
    
    if( ( subword & 0xff00) == pcan::command_base &&
        NB_GetNodebit( UPM_NB_MOB_OPEN ) )
    {
        packet.rx_ups(MyUPSNumber);
    }
    
    packet.data0(data0);
    packet.data1(data1);
    packet.data2(data2);
    packet.data3(data3);
        
    PacketTx( packet, periodic );
}

void ParallelCanNetwork::TransmitToUPMsInUPS( uint16_t subword, uint16_t data0,uint16_t data1,uint16_t data2, uint16_t data3 )
{
    ParallelCanPacket packet( subword, 4 );
    packet.rx_ups(MyUPSNumber);
    packet.data0(data0);
    packet.data1(data1);
    packet.data2(data2);
    packet.data3(data3);
    PacketTx( packet );
} 

void ParallelCanNetwork::TransmitBypassCommand(uint16_t requestedState)
{
#if 0
    // Any node can request a bypass transfer from all attached UPS master
    // nodes.
    // Historical note: At one time, the design was to allow slave nodes to request
    // a bypass transfer over CAN through this method.  IE, bypass state sync was going to
    // be maintained in the bypass state machine.  For 9E, bypass state sync is being
    // done through the MCU state machine instead.  This code remains for reference
    // purposes.
    const uint_least8_t MasterUPMNumber = 0;
    
    ParallelCanPacket packet( pcan::bypass_command );
    
    if( NB_GetNodebit( UPM_NB_MOB_OPEN ) )
    {
        packet.rx_ups(MyUPSNumber);
    }
    
    packet.rx_upm(MasterUPMNumber);
    packet.data0(requestedState);
    
    PacketTx( packet );
#endif
}

void ParallelCanNetwork::TransmitMasterRMSPacket( const stThreePhase& rmsData )
{
    ParallelCanPacket packet( pcan::master_rms_command, 3 );

    if( NB_GetNodebit( UPM_NB_MOB_OPEN ) )
    {
        packet.rx_ups( MyUPSNumber );
    }
    
    packet.MDL.word.LOW_WORD = int16_t( rmsData.phA * 32767.0f  );
    packet.MDL.word.HI_WORD  = int16_t( rmsData.phB * 32767.0f  );
    packet.MDH.word.LOW_WORD = int16_t( rmsData.phC * 32767.0f  );

    PacketTx( packet, true );
}  
 

//  The unit with both min UPM Number, min UPS Number, and a working inverter
//  will win as the MASTER.  If nobody has a working inverter, then the lowest 
//  numbered UPM wins. 
void ParallelCanNetwork::PCan_CheckMaster(void)		//1s
{
    if (!NB_GetNodebit(UPM_NB_PARALLEL_CAN_ERROR))
    {
        bool anInverterAvailable = ParGlobalOrData.McuStatus.bit.UPSInverterAvailable;
        /* UPSInverterAvailable is determined, by calling interface PCan_CheckUPSInverterAvailable */
        if ((parallelSynchFlags & PARALLEL_SYNCH_FLAG_SCAN_INVERTER) == 0U)
        	parallelSynchFlags &= ~PARALLEL_SYNCH_FLAG_IDENTIFY_MASTER;

        for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
        {
            for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
            {
                if ( ( ParallelCan.CurrentUPMsOnBus[ ups ] & ( 1 << upm ) )  &&
                     ( ups == MyUPSNumber                                      ||
                       ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed         &&
                         !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )    
                {
                    //only include UPM's in your own UPS when your MOB is open
                    //don't ever include UPM's in other UPS's with open MOB's
                    if( ( ( ups < MyUPSNumber ) ||
                          ( ( ups == MyUPSNumber ) && (upm < MyUPMNumber ) ) ) &&
                        ( !anInverterAvailable || UpmData[ups][upm].McuStatus.bit.UPSInverterAvailable ) )
                    {
                         // 1) Lower numbered than I am AND
                         // 2) Either nobody's inverter is available or his is available
                         ParallelStatus.bit.Master = false;
                         return;
                    }
                }                       
            }         
        }
        
        // I am only the master if either nobody's inverter is available, or mine is available
        ParallelStatus.bit.Master = ( !anInverterAvailable || MyUpmData().McuStatus.bit.UPSInverterAvailable );
    }
    else
    {
           // Do not change the master after a CAN failure
    }
}

void ParallelCanNetwork::PCan_OnParameterChanged(uint16_t eepAddress, uint16_t eepValue)
{
    // When a local parameter changes that is in the shared parameter list, immediately broadcast
    // the change to the PCAN network.
    // This function is invoked from a separate task.

    // Critical section avoids a data race with the PCAN periodic task
    CriticalSection enter( IER_DMA_ONLY );
    if( ParallelStatus.bit.ParamsInit == true )		//4. after sync_init, EEPROM_UPDATE_ENABLED sync
    {
        // Inform other nodes of the change
        TransmitBroadcastPacket(pcan::shared_params,
            eepAddress,
            eepValue,
            uint16_t(EEPROM_UPDATE_ENABLED),
            uint16_t(EEPROM_UPDATE_ENABLED), false);
    }

}

// Read shared parameters from UPM's eeprom.
void ParallelCanNetwork::PCan_SyncSharedParams(void)	//1s
{
    uint16_t id,data0,data1,data2,data3;
    static QueueHandle receiver = NULL;
    static const uint16_t maxPending = 10;//5;
    static EE_ID *ParallelSyncIndex = (EE_ID*)eeprom;
    uint16_t eeTableSize = GetEETableSize();
    
    if (!receiver)
    {
        receiver = QueueCreate(maxPending, sizeof(EepAddrValue));
    }
    //find parameter index need be shared this time
    while( ( ParallelSyncIndex->eep_functionIndex & EE_PARALLEL ) != EE_PARALLEL )	//1.1.check type if end
    {
        if ( ParallelSyncIndex->paramNum == eeprom[eeTableSize - 1].paramNum )
        {
            ParallelSyncIndex = (EE_ID*)eeprom;
			//Add for solving eep paral setting issue,as it will not be set(this issue is cause by add CSB_Backupeep)
            ParallelStatus.bit.ParamsInit = true;
        }
        else
        {
            ParallelSyncIndex++;
        }
    }

    if ( PendEepRead( ParallelSyncIndex->paramNum, receiver ) )	//2.read eep dat,
    {
        //prepare for next shared parameter
        if ( ParallelSyncIndex->paramNum == eeprom[eeTableSize - 1].paramNum )	//2.1. last word:eeTableSize
        {
            ParallelSyncIndex = (EE_ID*)eeprom;
            ParallelStatus.bit.ParamsInit = true;
        }
        else
        {
            ParallelSyncIndex++;
        }
    }
    
    // when any shared parameter is changed,this UPM should broadcast it to the network.
    // also,we should make sure at least all the shared parameters have been initialized. 
    EepAddrValue result;
    while ( QueueReceive(receiver, &result, 0) )	//3.1 rd dat, then to all; init end,not run
    {
        CriticalSection enter( IER_DMA_ONLY );
        id = pcan::shared_params;
        data0 = result.addr;
        data1 = result.value;
        data2 = uint16_t(EEPROM_UPDATE_DISABLED);
        data3 = uint16_t(EEPROM_UPDATE_DISABLED); 
        TransmitBroadcastPacket(id,data0,data1,data2,data3);
    }
}

// Check all the online UPMs' shared parameters. If any difference, record the id.
void ParallelCanNetwork::PCan_CheckSharedParams( void )
{    
    uint16_t error = 0;
    uint16_t readValue = 0;
    static uint16_t PcanFailCount = 0;
    if( ( ParallelStatus.bit.ParamsInit == 1 )                 &&
        ( NumOfUPMs != ParallelCan.NumberOfUPMs[ MyUPSNumber ] ) )	//1).master check
    {
        error += 10000;
        
        if ( !ParallelCan.UPSIsParallel() )
        {
            // if unit works on normal/ess/ect, if should shutdown automatically 
            if( MCUStateMachine.InvSupportingLoad() && !UPMCalEnabled )
            {
                ParallelCan.ParallelCommand.bit.para_inverter_off_sync = 1;
                ParallelCan.TransmitCommandPacket( pcan::fw_inverter_off_sync_command);
            }
        }
    }
    
    if ( ParallelStatus.bit.ParamsInit == 1   &&
         ParallelCan.TotalNumberOfUPSs > NumOfUPSs)			//2)paral numbers	
    {
        error += 20000;
    }
    //Update the previous failure sync
//	    if( MissMatchEep.addr != 0 && MissMatchEep.value != 0)	
	if( MissMatchEep.addr != 0)	//correct bug	
    {
		//3.1)after compare,diff; then keep read EE_U_slf, if same, then clr alarm
        uint16_t readResult = GetEepData(MissMatchEep.addr, 1, &readValue, TSK_100_ms);
	    PcanFailCount++;
        if( readResult != 0 &&
            ( readValue == MissMatchEep.value ) )
        {
            MissMatchEep.addr = 0;
            MissMatchEep.value = 0;
	        PcanFailCount = 0;
	    }

	    if(PcanFailCount >= 3)
	    {
	    	error += MissMatchEep.addr;		//
	    	PcanFailCount = 0;
	    }
    }
    
    NB_DebounceAndQue( UPM_NB_PARALLEL_SETUP_FAIL, error, error );		//61
    
    if( ParallelCan.MasterUPM() )
    {
        ParallelStatus.bit.Int_Synced = 1;
    }

    // clear the synced bits so we know if an error occurs, the signals are faster than this task
    // so will be set next time if everythings working
    ParallelStatus.bit.Int_Synced = 0;
    ParallelStatus.bit.Para_Synced = 0;  
}

void ParallelCanNetwork::PCan_UpdateSharedParams( const MBOX& box )
{
    const ParallelCanPacket& packet = static_cast<const ParallelCanPacket&>(box);
    uint_least8_t src_upm = packet.tx_upm();
    uint_least8_t src_ups = packet.tx_ups();
    uint16_t eepAddress = packet.data0();
    uint16_t dataValue = packet.data1();
    uint16_t eeprom_operation = packet.data2();
    uint16_t readValue = 0;
    bool readResult = false;
    
    // don't put receiving data into eeprom here,should do it with manual operation
    readResult = GetEepData(eepAddress, 1, &readValue, TSK_100_ms);  //2.read U_self eep, adr
    if( readResult  &&
        ( readValue != dataValue )  &&
        ( eeprom_operation == uint16_t( EEPROM_UPDATE_ENABLED ) ) )	 
    {
		//2.1.syn en chg: then 
        PutEepData(eepAddress, 1, &dataValue, TSK_100_ms);	//2.1, U_my != U_rx, then save eep
    }
    else if ( readResult  &&
              ( readValue != dataValue )  &&
              ( uint16_t( EEPROM_UPDATE_DISABLED ) == eeprom_operation ) )	
    {
    	//2.2.syn ini: record the ee_adr, mean setup fail;
        ParallelCan.MissMatchEep.addr = eepAddress;
        ParallelCan.MissMatchEep.value = dataValue;
    }
    else
    {
        //readValue == dataValue or readResult is false, do nothing
    }

}
void ParallelCanNetwork::PCan_ProcessC9Command( void )
{
    if( C9Command.bit.c9_ect_on_command )
    {
        if( ParallelStatus.bit.Master )
        {
            if( MCUStateMachine.ECTCommand() )
            {                
                // broadcast to slave
                TransmitCommandPacket( pcan::sys_ups_ect_on_command);
            }
            else
            {
                C9Command.bit.c9_ect_on_command = 0;
            }
        }
        else
        {
            // broadcast to master to further handle this command
            TransmitCommandPacket( pcan::para_ups_ect_on_command);
        }

        // clear command
        C9Command.bit.c9_ect_on_command = 0;
    }

    if( C9Command.bit.c9_ect_off_command )
    {
        if( ParallelStatus.bit.Master )
        {
            MCUStateMachine.ECTPhase = 10;
            TransmitCommandPacket( pcan::sys_ups_ect_off_command); 
        }
        else
        {
            TransmitCommandPacket( pcan::para_ups_ect_off_command);
        }
        C9Command.bit.c9_ect_off_command = 0;
    }                                  
}
            
void ParallelCanNetwork::PCan_ProcessInternalCommand( void )
{
    // check bypass on command from csb
    if( InternalCommand.bit.csb_bypass_on_command )
    {
        if( ParallelStatus.bit.Master )
        {
            if( PCan_CheckSystemBypassAvailable() )
            {
                // transfer to bypass
                MCUStateMachine.BypassCommand( );
                
                // broadcast to bypass if PCAN is OK
                if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
                {
                    TransmitCommandPacket( pcan::sys_bypass_on_command);
                    TransmitCommandPacket( pcan::sys_bypass_on_command);
                    TransmitCommandPacket( pcan::sys_bypass_on_command);
                }
            }
        }
        else
        {
            // broadcast to master to further handle this command if PCAN is OK
            if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
            {
                TransmitCommandPacket( pcan::para_bypass_on_command);
            }
        }
        
        // clear command
        InternalCommand.bit.csb_bypass_on_command = 0;
    }
    
    // check on normal command from csb
    if( InternalCommand.bit.csb_on_normal_command )
    {
		MCUStateMachine.ParalSystemOn = false;	//20180418 for paral online, repair the left

		if( ParallelStatus.bit.Master )
        {
            uint16_t start_up_fail = PCan_CheckSystemOnNormalFail();

            if( !start_up_fail )
            {
                // master on normal
                MCUStateMachine.NormalCommand();
            
                // broadcast to on normal
                TransmitCommandPacket( pcan::sys_ups_normal_commmand);
                TransmitCommandPacket( pcan::sys_ups_normal_commmand);
                TransmitCommandPacket( pcan::sys_ups_normal_commmand);
				MCUStateMachine.ParalSystemOn = true;	//20180418 for paral online, repair the left
            }
            else
            {
                // broadcast to do nothing,or nothing needs to be done
                TransmitCommandPacket( pcan::para_ups_decline_start_command, start_up_fail); 
            }
        }
        else
        {
            // broadcast to master to further handle this command
            if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
            {
                TransmitCommandPacket( pcan::para_ups_normal_commmand);
				MCUStateMachine.ParalSystemOn = true;	//20180418 for paral online, repair the left
            }
        }    
        
        // clear command
        InternalCommand.bit.csb_on_normal_command = 0;        
    }
    
    // check load off command from csb
    if( InternalCommand.bit.csb_load_off_command )
    {
//	        // transfer to standby
//	        MCUStateMachine.StandbyCommand();
		MCUStateMachine.Turn_UPS_Off();
        
        // broadcast to standby
        if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
        {
            TransmitCommandPacket( pcan::para_ups_load_off_command);
        }
        // clear command
        InternalCommand.bit.csb_load_off_command = 0;
    }
    
    // check activate charger command from csb
    if( InternalCommand.bit.csb_activate_charger_command )
    {
        if( ParallelStatus.bit.Master )
        {    
            // handle charger on command
            if( PCan_CheckSystemChargerOnAvailable() )
            {
                // on charger
                Abm().ChargerCmdOn();
                
                if( ( MCUStateMachine.GetState() == SHUTDOWN_STATE) || 
                    ( MCUStateMachine.GetState() == BYPASS_STATE) )
                {
                    MCUStateMachine.AutoStandbyCommand();                    
                }
                
                // broadcast to on charger
                if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
                {
                    TransmitCommandPacket( pcan::sys_activate_charger_command);
                }
            }
        }
        else
        {
            // broadcast to master to further handle this command
            if( !NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ) )
            {
                TransmitCommandPacket( pcan::para_activate_charger_command);
            }
        }
                
        // clear command
        InternalCommand.bit.csb_activate_charger_command = 0;    
    }
    
    // check deactivate charger command from csb
    if( InternalCommand.bit.csb_deactivate_charger_command )
    {
        // off charger
        if( MCUStateMachine.InStandby() )
        {
            MCUStateMachine.StandbyChargerOffCommand();   
        }
        
        Abm().ChargerCmdOff();
        
        // broadcast to off charger
        TransmitCommandPacket( pcan::para_deactivate_charger_command);
        
        // clear command
        InternalCommand.bit.csb_deactivate_charger_command = 0; 
    }
    
    // check start battery test command from csb
    if( InternalCommand.bit.csb_start_battery_test_command )
    {
        // check if the system is ready for battery test
        if( PCan_CheckSystemBatteryTestAvailable() )
        {
            // at present, the battery test command is for specified UPM and needs no Sync.
            // for future, we can add code here to do Sync for new requirement if there is.
            // comments now here for mark.
        }
        
        // clear command
        InternalCommand.bit.csb_start_battery_test_command = 0;
    }
    
    // check eco on command from csb
    if( InternalCommand.bit.csb_eco_on_command )
    {
        if( ParallelStatus.bit.Master )
        {    
            // check if system can work on eco
            if( PCan_CheckSystemEcoOnAvailable() )
            {
                // eco on
                ESSEnabled = 1;
                PutEepData( Find_Eep_Num(&ESSEnabled), 1, &ESSEnabled, 0 );
                MCUStateMachine.EcoOnCommand();
                
                // broadcast to work on eco
                TransmitCommandPacket( pcan::para_eco_on_command);    
            }
            else
            {
                // Explicitly cancel the ECO on command with ECO off command, and
                // mark ECO as disabled.
                TransmitCommandPacket( pcan::para_eco_off_command);
                ESSEnabled = 0;
                PutEepData( Find_Eep_Num(&ESSEnabled), 1, &ESSEnabled, 0 );
            
            }
        }
        else
        {
            // broadcast to master to further handle this command
            TransmitCommandPacket( pcan::sys_eco_on_command);
            TransmitCommandPacket( pcan::sys_eco_on_command);
            TransmitCommandPacket( pcan::sys_eco_on_command);
        }        
        
        // clear command
        InternalCommand.bit.csb_eco_on_command = 0;
    }
    
    // check eco off command from csb
    if( InternalCommand.bit.csb_eco_off_command )
    {
        // always accept eco off command
        MCUStateMachine.EcoOffCommand();
        ESSEnabled = 0;
        PutEepData( Find_Eep_Num(&ESSEnabled), 1, &ESSEnabled, 0 );
        
        // broadcast to eco off
        TransmitCommandPacket( pcan::para_eco_off_command);
            
        // clear command
        InternalCommand.bit.csb_eco_off_command = 0;
    }                       
}

void ParallelCanNetwork::PCan_ProcessParallelCommand( void )
{
    // only master response these commands,
    // except for para_inverter_off_sync, para_redundant_off, and para_redundant_battery_off
    if( !ParallelStatus.bit.Master )
    {
        ParallelCommand.all &= 0x9800;
        return;
    }
    
    // on normal command    
    if( ParallelCommand.bit.para_on_normal_command )
    {
        // check if we can turn on the system
        uint16_t start_up_fail = PCan_CheckSystemOnNormalFail();
        
        // response based on check result    
        if( !start_up_fail )
        {
            // master on normal
            MCUStateMachine.NormalCommand();
           
            // broadcast to on normal
            TransmitCommandPacket( pcan::sys_ups_normal_commmand);
            TransmitCommandPacket( pcan::sys_ups_normal_commmand);
            TransmitCommandPacket( pcan::sys_ups_normal_commmand);
			MCUStateMachine.ParalSystemOn = true;	//20180418 for paral online, repair the left			
        }
        else
        {
            // broadcast to do nothing,or nothing needs to be done
            TransmitCommandPacket( pcan::para_ups_decline_start_command, start_up_fail); 
			MCUStateMachine.ParalSystemOn = false;	//20180418 for paral online, repair the left
        }
        
        // clear command
        ParallelCommand.bit.para_on_normal_command = 0; 
    }
    
    // battery test command
    // at present, the battery test command is for specified UPM and needs no Sync.
    // for future, we can add code here to do Sync for new requirement if there is.
    // comments now here for mark.
    if( ParallelCommand.bit.para_start_battery_test_command )
    {
        // check if we can start system battery test
        uint16_t battery_test = PCan_CheckSystemBatteryTestAvailable();
        
        // response based on check result
        if( battery_test )
        {
        }
        else
        {            
        }
        
        // clear command
        ParallelCommand.bit.para_start_battery_test_command = 0;        
    }
    
    // activate charger command
    if( ParallelCommand.bit.para_activate_charger_command )
    {
        // handle charger on command
        if( PCan_CheckSystemChargerOnAvailable() )
        {
            // on charger
            Abm().ChargerCmdOn();
            
            if( ( MCUStateMachine.GetState() == SHUTDOWN_STATE) || 
                ( MCUStateMachine.GetState() == BYPASS_STATE) )
            {
                MCUStateMachine.AutoStandbyCommand();                    
            }
            
            // broadcast to on charger
            TransmitCommandPacket( pcan::sys_activate_charger_command);
        }
        
        // clear command
        ParallelCommand.bit.para_activate_charger_command = 0;
    }
    
    // bypass on command
    if( ParallelCommand.bit.para_bypass_on_command )
    {
        if( PCan_CheckSystemBypassAvailable() )
        {
            // transfer to bypass
            MCUStateMachine.BypassCommand( );
                
            // broadcast to bypass
            TransmitCommandPacket( pcan::sys_bypass_on_command);
            TransmitCommandPacket( pcan::sys_bypass_on_command);
            TransmitCommandPacket( pcan::sys_bypass_on_command);
        }
        
        // clear command
        ParallelCommand.bit.para_bypass_on_command = 0;
    }           
    
    if( ParallelCommand.bit.para_ect_on_command )
    {
        // transfer to ect mode
        if( MCUStateMachine.ECTCommand() )
        {                
            // broadcast
            TransmitCommandPacket( pcan::sys_ups_ect_on_command);
        }
        else
        {
            // clear command
            ParallelCommand.bit.para_ect_on_command = 0;
        }
    }

    if( ParallelCommand.bit.para_ect_off_command )
    {
        MCUStateMachine.ECTPhase = 10;
        TransmitCommandPacket( pcan::sys_ups_ect_off_command);
    }         
}

void ParallelCanNetwork::PCan_UpdateSystemErrorStatus( void )
{
    // bypass
    ParallelCan.ParallelSystemError.bit.err_bypass = 0;
        
    // bypass phase rotation
    ParallelCan.ParallelSystemError.bit.err_bypass_phase = NB_GetNodebit( UPM_NB_BYPASS_PHASE_ROTATION );
    
    // input
    ParallelCan.ParallelSystemError.bit.err_input = NB_GetNodebit( UPM_NB_INPUT_AC_OVER_VOLTAGE )      ||
                                                    NB_GetNodebit( UPM_NB_INPUT_AC_UNDER_VOLTAGE )     ||
                                                    NB_GetNodebit( UPM_NB_INPUT_UNDER_OVER_FREQUENCY ) ||
                                                    NB_GetNodebit( UPM_NB_UTILITY_NOT_PRESENT );
    
    // input phase rotation
    ParallelCan.ParallelSystemError.bit.err_input_phase = NB_GetNodebit( UPM_NB_RECTIFIER_PHASE_ROTATION );         
    
    // epo
    ParallelCan.ParallelSystemError.bit.err_epo = 0;
    
    // over temperature
    ParallelCan.ParallelSystemError.bit.err_temperature = NB_GetNodebit( UPM_NB_RECTIFIER_OVERTEMPERATURE ) ||
                                                          NB_GetNodebit( UPM_NB_INVERTER_OVERTEMPERATURE )  ||
                                                          NB_GetNodebit( UPM_NB_AMBIENT_OVERTEMPERATURE )   ||
                                                          NB_GetNodebit( UPM_NB_PM_OVERTEMPERATURE );
    
    // output failures
    ParallelCan.ParallelSystemError.bit.err_output = NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD )       ||
                                                     NB_GetNodebit( UPM_NB_OUTPUT_OVERLOAD_TRIP )  ||
                                                     NB_GetNodebit( UPM_NB_OUTPUT_SHORT_CIRCUIT )  ||
                                                     NB_GetNodebit( UPM_NB_OUTPUT_PHASE_ROTATION ) ||
                                                     NB_GetNodebit( UPM_NB_ABNORMAL_OUTPUT_VOLTAGE_AT_STARTUP );                                                      

    // static switch failures
    ParallelCan.ParallelSystemError.bit.err_sts = 0;
                                                  
    // eeprom failures
    ParallelCan.ParallelSystemError.bit.err_eeprom = NB_GetNodebit( UPM_NB_NON_VOLATILE_RAM_FAILURE ) ||
                                                     NB_GetNodebit( UPM_NB_CONFIGURATION_ERROR );
                                                  
    // firmware incompatible
    ParallelCan.ParallelSystemError.bit.err_fw = NB_GetNodebit( UPM_NB_SOFTWARE_INCOMPATIBILITY_DETECTED );
    
    // hardware failures
    ParallelCan.ParallelSystemError.bit.err_hw_fail = NB_GetNodebit( UPM_NB_FUSE_FAILURE )               ||
                                                      NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_FAIL )     ||
                                                      NB_GetNodebit( UPM_NB_RECTIFIER_FAILED )           ||
                                                      NB_GetNodebit( UPM_NB_BATTERY_CONTACTOR_FAIL )     ||
                                                      NB_GetNodebit( UPM_NB_SITE_WIRING_FAULT )          ||
                                                      NB_GetNodebit( UPM_NB_CHECK_PRECHARGE )            ||
                                                      NB_GetNodebit( UPM_NB_INVERTER_CONTACTOR_FAILURE ) ||
                                                      NB_GetNodebit( UPM_NB_INVERTER_STARTUP_FAILURE );                                                                                           

    // maintenance bypass on
    ParallelCan.ParallelSystemError.bit.err_system = NB_GetNodebit( UPM_NB_INTERNAL_MBS_ACTIVE ) ||
                                                     ( NB_GetNodebit( UPM_NB_MBS_CLOSED ) &&
                                                       !NB_GetNodebit( UPM_NB_MOB_OPEN ) );
    
    // battery
    ParallelCan.ParallelSystemError.bit.err_battery = ( ( NB_GetNodebit( UPM_NB_BATTERIES_DISCONNECTED ) ||
                                                          !NB_GetNodebit(UPM_NB_BATTERY_INSTALLED) )        &&
                                                        !BatteryNotRequiredForOnline );
    
    // hardware
    ParallelCan.ParallelSystemError.bit.err_hw = NB_GetNodebit( UPM_NB_INVALID_BOARD_ID );
    
    // communication
    ParallelCan.ParallelSystemError.bit.err_communication = NB_GetNodebit( UPM_NB_LOSS_OF_SYNC_BUS )      ||
                                                            NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR );
                                                            
    // power supply
    ParallelCan.ParallelSystemError.bit.err_powersupply = NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT ) ||
                                                          NB_GetNodebit( UPM_NB_POWER_SUPPLY_15_VOLT_FAULT )||
                                                          NB_GetNodebit( UPM_NB_DRIVER_FAULT );
}

uint16_t ParallelCanNetwork::PCan_CheckSystemOnNormalFail( void )
{
    uint16_t error = 0;
    
    // bypass different : And = 0 && Or = 1
    if( ( !ParallelCan.ParGlobalAndData.SystemError.bit.err_bypass ) && 
        (  ParallelCan.ParGlobalOrData.SystemError.bit.err_bypass ) )
    {
        error |= DECLINE_CHECK_BYPASS;
    }
    
    // bypass phase rotation : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_bypass_phase )
    {
        error |= DECLINE_CHECK_BYPASS_PHASE;
    }
    
    // input different : And = 0 && Or = 1
    if( ( !ParallelCan.ParGlobalAndData.SystemError.bit.err_input ) && 
        (  ParallelCan.ParGlobalOrData.SystemError.bit.err_input ) )
    {
        error |= DECLINE_CHECK_INPUT;
    }
    
    // input phase rotation : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_input_phase )
    {
        error |= DECLINE_CHECK_INPUT_PHASE;
    }
    
    // epo : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_epo )
    {
        error |= DECLINE_CHECK_EPO;
    }
    
    // temperature : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_temperature )
    {
        error |= DECLINE_CHECK_TEMPERATURE;
    }
    
    // output : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_output )
    {
        error |= DECLINE_CHECK_OUTPUT;
    }
    
    // sts : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_sts )
    {
        error |= DECLINE_CHECK_STS;
    }
    
    // eeprom : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_eeprom )
    {
        error |= DECLINE_CHECK_EEPROM;
    }    
    
    // firmware : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_fw )
    {
        error |= DECLINE_CHECK_FW;
    }
    
    // hardware failures : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_hw_fail )
    {
        error |= DECLINE_CHECK_HW_FAIL;
    }
    
    // fan : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_system )
    {
        error |= DECLINE_CHECK_MAINTENANCE_BYPASS;
    }
    
    // battery : And = 0 && Or = 1
    if( ( !ParallelCan.ParGlobalAndData.SystemError.bit. err_battery ) && 
        (  ParallelCan.ParGlobalOrData.SystemError.bit.err_battery ) )
    {
        error |= DECLINE_CHECK_BATTERY;
    }
    
    // hw : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_hw )
    {
        error |= DECLINE_CHECK_HW;
    }
    
    // communication : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_communication )
    {
        error |= DECLINE_CHECK_COMMUNICATION;
    }
    
    // power supply : Or = 1
    if( ParallelCan.ParGlobalOrData.SystemError.bit.err_powersupply )
    {
        error |= DECLINE_CHECK_POWERSUPPLY;
    }
                                                                
    return( error );
}

uint16_t ParallelCanNetwork::PCan_CheckSystemEcoOnAvailable( void )
{
    return ( 1 );
}

uint16_t ParallelCanNetwork::PCan_CheckSystemBatteryTestAvailable( void )
{
    return( 1 );
}

uint16_t ParallelCanNetwork::PCan_CheckSystemChargerOnAvailable( void )
{
    return( 1 );
}

/* Check if this UPS can transfer to bypass */   
// #pragma CODE_SECTION( "ramfuncs" ) 
uint16_t ParallelCanNetwork::PCan_CheckUPSBypassAvailable( void )
{
    uint16_t temp = 1;
    for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
    {
        if(ParallelCan.CurrentUPMsOnBus[MyUPSNumber] & (1 << upm))
        {
            temp = temp && UpmData[MyUPSNumber][upm].McuStatus.bit.BypassAvailable;
        }
    }    
    return(temp);
}    
    
/* Check if this UPS can transfer to online */
// #pragma CODE_SECTION( "ramfuncs" )
uint16_t ParallelCanNetwork::PCan_CheckUPSInverterAvailable( void )
{
	parallelSynchFlags &= ~PARALLEL_SYNCH_FLAG_SCAN_INVERTER;
    if ( UPMCalEnabled )
    {
       return 1;
    }
    else
    {
        uint16_t temp = 1;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if(ParallelCan.CurrentUPMsOnBus[MyUPSNumber] & (1 << upm))
            {
                temp = temp && UpmData[MyUPSNumber][upm].McuStatus.bit.InverterAvailable;
            }
        }
        return(temp);
    }
}
    
/* Checks if enough bypasses are available to support the current load so system can transfer to bypass state */
uint16_t ParallelCanNetwork::PCan_CheckSystemBypassAvailable( void )
{
    // 115% is based on the Engineering Spec for bypass continuous rating
    const float invBypassPowerRating = 1.0/115.0f;
    return SystemLoad*invBypassPowerRating < SystemBypassAvailable;
} 

/* Checks if enough inverters are available to support the current load so system can transfer to online state */
uint16_t ParallelCanNetwork::PCan_CheckSystemInverterAvailable( void )
{
    if ( UPMCalEnabled )
    {
        return true;
    }
    else
    {
        return SystemLoad/float(OverloadLevel1) < SystemInverterAvailable;
    }
}    
   
/* Checks if system is redundant */
uint16_t ParallelCanNetwork::PCan_CheckSystemRedundant( void )
{
    return (SysLoadData.phA/( (float)NumOfUPMs * (float)OverloadLevel1)) < (PCan_SumOnInverter()-1.0) &&
           (SysLoadData.phB/( (float)NumOfUPMs * (float)OverloadLevel1)) < (PCan_SumOnInverter()-1.0) &&
           (SysLoadData.phC/( (float)NumOfUPMs * (float)OverloadLevel1)) < (PCan_SumOnInverter()-1.0);
} 

/* Returns total number of UPSs available to transfer to bypass state. */
void ParallelCanNetwork::UpdateBypassAvailable( void )
{
    uint16_t SumBypassAvail = 0;
    bool valid, temp;
    
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        temp = true;
        valid = false;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & (1 << upm)         &&
                ( ups == MyUPSNumber                                     ||
                  ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed        &&
                    !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )
            {
                //only include UPM's in your own UPS when your MOB is open
                //don't ever inlcude UPM's in other UPS's with open MOB's
                temp = temp && UpmData[ups][upm].McuStatus.bit.BypassAvailable;
                valid = true;
            }
        }
        if( valid && temp )
        {
            SumBypassAvail += 1;
        }
    }
    SystemBypassAvailable = SumBypassAvail;
}

/* Returns total number of UPSs available to transfer to online state. */
void ParallelCanNetwork::UpdateInverterAvailable( void )
{
    uint16_t SumInverterAvail = 0;
    bool valid, temp;
    
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        temp = true;
        valid = false;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & (1 << upm)         &&
                ( ups == MyUPSNumber                                     ||
                  ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed        &&
                    !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )
            {
                //only include UPM's in your own UPS when your MOB is open
                //don't ever inlcude UPM's in other UPS's with open MOB's)
                temp = temp && UpmData[ups][upm].McuStatus.bit.InverterAvailable;
                valid = true;
            }
        }
        if( valid && temp)
        {
            SumInverterAvail += 1;
        }
    }
    SystemInverterAvailable = SumInverterAvail;
} 

/* Returns total number of UPSs currently online */
uint16_t ParallelCanNetwork::PCan_SumOnInverter( void )
{
    uint16_t SumOnInverter = 0;
    bool valid, temp;
    
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        temp = true;
        valid = false;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & (1 << upm)         &&
                ( ups == MyUPSNumber                                     ||
                  ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed        &&
                    !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )
            {
                //only include UPM's in your own UPS when your MOB is open
                //don't ever inlcude UPM's in other UPS's with open MOB's)
                temp = temp && UpmData[ups][upm].McuStatus.bit.OnInverter;
                valid = true;
            }
        }
        if( valid && temp)
        {
            SumOnInverter += 1;
        }
    }
    return SumOnInverter;
} 

/* Returns total number of UPMs currently online */
uint16_t ParallelCanNetwork::PCan_UPMSumOnInverter( void )
{
    uint16_t UPMSumOnInverter = 0;

    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & (1 << upm)         &&
                ( ups == MyUPSNumber                                     ||
                  ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed        &&
                    !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )
            {
                //only include UPM's in your own UPS when your MOB is open
                //don't ever include UPM's in other UPS's with open MOB's)
                if( UpmData[ups][upm].McuStatus.bit.OnInverter )
                {
                    UPMSumOnInverter += 1;
                }
            }
        }

    }
    return UPMSumOnInverter;
}

/* Returns the sum of the percentage loads reported by each UPS. */
void ParallelCanNetwork::UpdateSystemLoad( void )
{
    uint16_t SysLoad = 0; 
    uint16_t UpsLoad = 0;
    LoadShareMeter tempSysLoadData;
    tempSysLoadData.sum = 0;
    tempSysLoadData.phA = 0;
    tempSysLoadData.phB = 0;
    tempSysLoadData.phC = 0;
    
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        UpsLoad = 0;
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & (1 << upm)         &&
                ( ups == MyUPSNumber                                     ||
                  ( UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed        &&
                    !NB_GetNodebit( UPM_NB_MOB_OPEN ) ) ) )
            {
                //only include UPM's in your own UPS when your MOB is open
                //don't ever inlcude UPM's in other UPS's with open MOB's)
                if (UpsLoad < LoadData[ups][upm].sum)
                {
                    UpsLoad = LoadData[ups][upm].sum;
                }
            
                tempSysLoadData.sum += LoadData[ups][upm].sum;
                tempSysLoadData.phA += LoadData[ups][upm].phA;
                tempSysLoadData.phB += LoadData[ups][upm].phB;
                tempSysLoadData.phC += LoadData[ups][upm].phC;
            }
        }
        SysLoad += UpsLoad;
    }
    SystemLoad = float(SysLoad);
    SysLoadData.sum = tempSysLoadData.sum;
    SysLoadData.phA = tempSysLoadData.phA;
    SysLoadData.phB = tempSysLoadData.phB;
    SysLoadData.phC = tempSysLoadData.phC;
}

/* Returns false if any UPM has an open MOB */
bool ParallelCanNetwork::PCan_CheckMOBsClosed( void )
{
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            if( ParallelCan.CurrentUPMsOnBus[ups] & ( 1 << upm ) &&
                !ParallelCan.UpmData[ups][upm].McuStatus.bit.BldInpMOBClosed )
            {
                return false;
            }
        }
    }
    return true;
}

/* Returns true if any UPM has an active MBS signal (external only) */
bool ParallelCanNetwork::PCan_CheckMBSClosed( void )
{
    bool BldInputMaintenanceBypass = false;
    for ( uint16_t ups = 0; ups < ParallelCanNetwork::MAX_NUM_UPS; ups++ )
    {
        for ( uint16_t upm = 0; upm < ParallelCanNetwork::MAX_NUM_UPM; upm++ )
        {
            BldInputMaintenanceBypass |=  
            ParallelCan.UpmData[ups][upm].McuStatus.bit.BldInpMaintenanceBypass;
        }
    }
    return BldInputMaintenanceBypass;
}

// *******************************************************************************************
// * Function:     ProcessBatteryTemperature
// * Purpose:      find the max temperature in the common battery
// * Parms Passed: None
// * Returns:      None
// *******************************************************************************************
void ParallelCanNetwork::ProcessBatteryTemperature( void )
{
    if( ( CommonBattery_Internal == CommonBattery ) ||
        ( CommonBattery_Separate == CommonBattery ) )
    {
        MaxBatTempInCommon = BatteryTemperature[MyUPSNumber];
    }
    else
    {
        for(uint16_t ups = 0; ups < MAX_NUM_UPS; ups++)
        {
            MaxBatTempInCommon = std::max(MaxBatTempInCommon, BatteryTemperature[ups]);
        }
    }
}
/************************************ OS hook functions ***********************/

extern "C"
void ParallelCanRx_TSK(void *)
{
    ParallelCan.RxPacket_TSK();
}

#pragma CODE_SECTION("ramfuncs")
extern "C"
void ParallelCan_Ch1_HWI(void)
{
    ParallelCan.Channel1_HWI();
}

#pragma CODE_SECTION("ramfuncs")
extern "C"
void ParallelCan_Ch0_HWI(void)
{
    ParallelCan.Channel0_HWI();
}
