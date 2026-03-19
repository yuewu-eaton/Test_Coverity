#include "DSP28x_Project.h"
#include "InternalCan.h"
#include "IOexpansion.h"
#include "C9Commands.h"
#include "CanIds.h"
#include "Version.h"
#include "Eeprom_Map.h"
#include "NB_Funcs.h"
#include "CanMeters.h"
#include "Meters.h"
#include "MCUState.h"
#include "RectifierControl.h"
#include "BootloaderAPI.h"
#include "Abm.h"
#include "Rtc.h"
#include "Spi_Task.h"
#include "BTR.h"
#include "ParallelCan.h"
#include "ParallelCanIds.h"
#include "PeriodicTSKs.h"
#include "AcMeter.h"

#include <cstring>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std;

InternalCanPacket&
InternalCanPacket::CrcPacket()
{
    // Parity check only for now. Using 0b0101 0101 0101 0101 in the
    // lead helps to ensure integrity even when the data is
    // all zeros.
    return data3(CAN_CRC_NONCE ^ data0() ^ data1() ^ data2());
}

namespace {

enum
{
    BYPASS_ON_COMMAND,        // system level command,for unit or external parallel system
    UPS_NORMAL_COMMAND,       // system level command,for unit or external parallel system
    UPS_LOAD_OFF_COMMAND,     // system level command,for unit or external parallel system
    ACTIVATE_CHARGER_COMMAND,
    DEACTIVATE_CHARGER_COMMAND,
    START_BATTERY_TEST_COMMAND,
    ECO_ON_COMMAND,
    ECO_OFF_COMMAND,
    RESET_ALARMS_COMMAND,
    CLEAR_HISTORY_COMMAND,
    UPS_OFF_COMMAND,           // for unit only. if 60kVA, for 2 UPMs
    SYSTEM_VERIFIED_OK_COMMAND,
    SYSTEM_VERIFIED_FAIL_COMMAND,
    SINGLE_UPS_NORMAL_COMMAND
};

bool SystemTypeCheckOk = false;
} // !namespace anonymous

InternalCanNetwork InternalCan;
extern uint16_t SingleUPSStartUpEnabled;

InternalCanNetwork::InternalCanNetwork()
    : CanDriver(ECANA, 5, 10, 40)
{
}

bool
InternalCanNetwork::CanHardwareReady(void)
{
    return MyHardwareNumber != 0 && 
    	EEStatusBits.bit.EEDataInitialized;
}

void
InternalCanNetwork::ConfigureRxMailboxes()
{
    RxPackets(&ReceiveC9Packet, &CsbC9StateMachine, 
        CAN_ID_PRIORITY_MASK 
            | (uint32_t(CAN_C9_REQUEST_MASK) << CAN_ID_TYPE_BITS)
            | (uint32_t(CAN_C9_UPM_MASK) << CAN_ID_SUBTYPE_BITS),
        (uint32_t(CAN_C9_REQUEST_BASE) << CAN_ID_TYPE_BITS)
            | (uint32_t(UPMNumber()) << CAN_ID_SUBTYPE_BITS)
            | (uint32_t(CAN_C9_PRIORITY) << CAN_ID_PRIORITY_BITS));
    RxPackets(&ReceiveCommandPacket, NULL, 
        CAN_ID_TYPE_MASK| CAN_ID_SUBTYPE_MASK, 
        (uint32_t(CAN_UPM_COMMAND) << CAN_ID_TYPE_BITS) |
        (uint32_t(UPMNumber()) << CAN_ID_SUBTYPE_BITS)); 
    RxPackets(&ReceiveBuildingInputPacket, NULL,
        CAN_ID_TYPE_MASK, uint32_t(CAN_UPM_BUILDING_INPUT) << CAN_ID_TYPE_BITS);
    RxPackets(&ReceiveCanIDPacket, NULL,
        CAN_ID_TYPE_MASK, uint32_t(CAN_PMF_ID_TYPE) << CAN_ID_TYPE_BITS);

    RxPackets(&ReceiveRTCPacket, NULL,
        CAN_RTC_MASK, CAN_RTC_MATCH );

    RxPackets(&ReceiveSectionResetRequest, NULL,
        CAN_ID_TYPE_MASK | CAN_ID_SUBTYPE_MASK, 
        (uint32_t(CAN_SECTION_RESET_REQUEST) << CAN_ID_TYPE_BITS) |
        (uint32_t(UPMNumber()) << CAN_ID_SUBTYPE_BITS));        
    RxPackets(&ReceiveSetUpsNumberPacket, NULL,
        CAN_ID_TYPE_MASK, uint32_t(CAN_PMF_FROM_CSB_AUTOID) << CAN_ID_TYPE_BITS);       
    RxPackets(&ReceiveCSBMetersPacket, NULL,
        CAN_CSB_METERS_MASK, uint32_t(CAN_CSB_METERS_BASE) << CAN_ID_TYPE_BITS);
}

void
InternalCanNetwork::SendCanId()
{
    const st_Section_Header* header = (const st_Section_Header*)BOOTLOADER_HEADER_ADDRESS;
    uint16_t pld_ver = 1;
    
    if (PLDVersionReg.Ready)
    {
    	pld_ver = PLDVersionReg.Version;
    }
    
    PacketTx( InternalCanPacket(CAN_PMF_ID_TYPE, CAN_PMF_ID_PRIORITY)
        .subtype(UPMNumber())
        .data0(MyHardwareNumber)
        .data1(FirmwareVersion)
        .data2(BinToBcd(header->Version)) // Bootloader version TBD
        .data3(BinToBcd(pld_ver)));
    
    PacketTx( InternalCanPacket(CAN_PMF_XID_TYPE, CAN_PMF_ID_PRIORITY)
        .subtype(UPMNumber())
        .data0(MyHardwareNumber)
        .data1(BinToBcd(FirmwareBuildNum))
        .data2(BinToBcd(header->Build))
        .data3(0));
}

void
InternalCanNetwork::SendUpsNumber()
{
    uint16_t ups_number;
    
    if( !EEStatusBits.bit.EEDataInitialized )
    {
        ups_number = 0;
    }
    else
    {
        ups_number = MyUPSNumber;
    }    
            
    PacketTx(InternalCanPacket(CAN_PMF_TO_CSB_AUTOID, CAN_PRIORITY_LOW)
        .subtype(UPMNumber())
        .data0(ups_number)
        .data1(ParallelCan.AllUPSsSinceReset)
        .data2(ParallelCan.CurrentUPSsOnBus)
        .CrcPacket());
}

void
InternalCanNetwork::SendUPMNodebits()
{
    //uint16_t events[8] = {0, 0, 0, 0, 0, 0, 0, 0};        // Pan/20121012 delete
    uint16_t events[12] = {0};                              // Pan/20121012 add
    NB_GetNodebitBits(events);
    
    PacketTx(InternalCanPacket(CAN_PMF_NB0_TYPE, CAN_PMF_NB_PRIORITY)
        .subtype(UPMNumber())
        .data(events), true);

    PacketTx(InternalCanPacket(CAN_PMF_NB1_TYPE, CAN_PMF_NB_PRIORITY)
        .subtype(UPMNumber())
        .data(events+4), true);
    
    // Pan/20121012 add ,max events is 191, (191+1)/16/4 = 3,have to send another packet
    PacketTx(InternalCanPacket(CAN_PMF_NB2_TYPE, CAN_PMF_NB_PRIORITY)
        .subtype(UPMNumber())
        .data(events+8), true);    
}

void
InternalCanNetwork::SendHistory(const st_HQ_event& event)
{
    PacketTx(InternalCanPacket(CAN_UPM_HISTORY_LOG, CAN_UPM_HISTORY_PRIORITY)
        .subtype(UPMNumber())
        .data0(event.event)
        .data1(event.data)
        .data2(event.timeStamp.MinuteOfMonth)
        .data3(event.timeStamp.mSecOfMinute));
}

void
InternalCanNetwork::SendSectionResetResponse(uint16_t section, bool result)
{
    PacketTx(InternalCanPacket(CAN_SECTION_RESET_RESPONSE, CAN_PRIORITY_LOW)
        .subtype(UPMNumber())
        .data0(section)
        .data1(result)
        .data2(0)
        .CrcPacket());
}

void
InternalCanNetwork::SendCommandResponse(bool result)
{
    PacketTx(InternalCanPacket(CAN_UPM_COMMAND_RESPONSE, CAN_PRIORITY_HIGH)
        .subtype(UPMNumber())
        .data0(result)
        .data1(0)
        .data2(0)
        .CrcPacket());
}

namespace {
    
    template<typename IntT>
    IntT saturated_cast(float param);
    // Perform a float-to-integer cast that saturates at the minimum and
    // maximum values of the integer, while rounding to nearest.
    template<typename IntT>
    IntT saturated_cast(float param)
    {
        // First round to nearest. We would prefer to use roundf(), but it
        // is not provided by CCS4.
        if (param > 0)
        {
            param = std::floor(param+0.5);
        }
        else if (param < 0)
        {
            param = std::ceil(param - 0.5);
        }
        // Then clamp to the limited range
        param = std::max(float(std::numeric_limits<IntT>::min()), param);
        param = std::min(float(std::numeric_limits<IntT>::max()), param);
        return (IntT)param;
    }
// For each unit type, multiply the DSP-native units by this scaler, round
// to nearest, and cast to uint16_t to form the value in transmitted units.
// See the FRS for the definition of these units on the CSB
const float voltageScaleAC = 1.0f;
const float currentScaleAC = 10.0f;
const float pfScaleAC = 100.0f;
const float powerScaleAC = 0.1f;
const float powerRateScaleAC = 0.01f;
const float freqScaleAC = 10.0f;

// battery voltage
const float voltageScaleBattery = 1.0f;
// battery current
const float currentScaleBattery = 10.0f;
// volts per cell
const float vpcScaleBattery = 100.0f;
    
struct canMeter
{
    // The location of the meter in CAN
    can_meter_t packetAddr;
    // The scale factor for the meter
    float scale;
    // The location of the meter in memory. May be NULL if it is not yet defined
    float* location;
};

const canMeter canMeters[] = {
    // uint16_t meters
    { V12_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageLL_RMS.phA },
    { V23_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageLL_RMS.phB },
    { V31_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageLL_RMS.phC },
    { V12_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageLL_RMS.phA },
    { V23_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageLL_RMS.phB },
    { V31_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageLL_RMS.phC },
    { V12_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageLL_RMS.phA },
    { V23_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageLL_RMS.phB },
    { V31_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageLL_RMS.phC },
    
    { V1_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageRMS.phA },
    { V2_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageRMS.phB },
    { V3_BYP_METER, voltageScaleAC, &ScreenMeters.BypassVoltageRMS.phC },   
    { V1_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageRMS.phA },
    { V2_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageRMS.phB },
    { V3_INP_METER, voltageScaleAC, &ScreenMeters.InputVoltageRMS.phC },    
    { V1_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageRMS.phA },
    { V2_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageRMS.phB },
    { V3_OUTP_METER, voltageScaleAC, &ScreenMeters.OutputVoltageRMS.phC },
    
    { VBAT_METER, voltageScaleBattery, &ScreenMeters.BatteryVoltage },
    { VBAT_POS_METER, voltageScaleBattery, &ScreenMeters.BatteryVoltagePos },
    { VPC_METER, vpcScaleBattery, &ScreenMeters.BatteryVPC },
    { VPC1_METER, vpcScaleBattery, &BattVoltsperCell1 },
    { VPC2_METER, vpcScaleBattery, &BattVoltsperCell2 },
    { FREQ_BYP_METER, freqScaleAC, &ScreenMeters.BypassFrequency },
    { FREQ_INP_METER, freqScaleAC, &ScreenMeters.InputFrequency },
    { FREQ_OUTP_METER, freqScaleAC, &ScreenMeters.OutputFrequency },
    // Power factor meters
    { PF_BYP_METER, pfScaleAC, &ScreenMeters.BypassPowerFactor },
    { PF_INP_METER, pfScaleAC, &ScreenMeters.InputPowerFactor },
    { PF_OUTP_METER, pfScaleAC, &ScreenMeters.OutputPowerFactor },
    // Battery meters
    { BTR_METER, 1.0, &ScreenMeters.CalculatedBTR },
    { BAT_PERCENT, 1.0, &ScreenMeters.PercentBatteryRemaining },
    // Load percentage meter
    { LOAD_PERCENT, 1.0, &ScreenMeters.PercentLoad.sum },
    // Voltage ratings
    { INPUT_VOLTAGE_RATING, voltageScaleAC, &ScreenMeters.InputVoltageRating },
    { OUTPUT_VOLTAGE_RATING, voltageScaleAC, &ScreenMeters.OutputVoltageRating },
    { INPUT_VOLTAGE_MAX_RATING, voltageScaleAC, &ScreenMeters.InputVoltageMaxRating },
    { INPUT_VOLTAGE_MIN_RATING, voltageScaleAC, &ScreenMeters.InputVoltageMinRating },
    // Frequency ratings
    { INPUT_FREQUENCY_RATING, freqScaleAC, &ScreenMeters.InputFrequencyRating },
    { OUTPUT_FREQUENCY_RATING, freqScaleAC, &ScreenMeters.OutputFrequencyRating },
    // Output PF rating
    { OUTPUT_PF_RATING, pfScaleAC, &ScreenMeters.OutputPowerFactorRating },
    // Output kVA rating
    { OUTPUT_KVA_RATING, powerRateScaleAC, &ScreenMeters.OutputkVARating },
    // Battery ratings
    { BAT_LOAD_RATING, 1.0, &ScreenMeters.BatteryLoadRating },
    { BAT_TIME_RATING, 1.0, &ScreenMeters.BatteryTimeRating },  
    // Load percentages for phases
    { LOAD_PERCENT_1, 1.0, &ScreenMeters.PercentLoad.phA },
    { LOAD_PERCENT_2, 1.0, &ScreenMeters.PercentLoad.phB },
    { LOAD_PERCENT_3, 1.0, &ScreenMeters.PercentLoad.phC },
    
    // int16_t meters
    { I1_BYP_METER, currentScaleAC, &ScreenMeters.BypassCurrentRMS.phA },
    { I2_BYP_METER, currentScaleAC, &ScreenMeters.BypassCurrentRMS.phB },
    { I3_BYP_METER, currentScaleAC, &ScreenMeters.BypassCurrentRMS.phC },
    { I1_INP_METER, currentScaleAC, &ScreenMeters.InputCurrentRMS.phA },
    { I2_INP_METER, currentScaleAC, &ScreenMeters.InputCurrentRMS.phB },
    { I3_INP_METER, currentScaleAC, &ScreenMeters.InputCurrentRMS.phC },
    { I1_OUTP_METER, currentScaleAC, &ScreenMeters.OutputCurrentRMS.phA },
    { I2_OUTP_METER, currentScaleAC, &ScreenMeters.OutputCurrentRMS.phB },
    { I3_OUTP_METER, currentScaleAC, &ScreenMeters.OutputCurrentRMS.phC },
    { IBAT_METER, currentScaleBattery, &ScreenMeters.BatteryCurrent },
    { IBAT_POS_METER, currentScaleBattery, &ScreenMeters.BatteryCurrentP_Sum },
    { IBAT_NEG_METER, currentScaleBattery, &ScreenMeters.BatteryCurrentN_Sum },

    // Battery negative voltage meter
    { VBAT_NEG_METER, voltageScaleBattery, &ScreenMeters.BatteryVoltageNeg },

    // kVA / kW meters
    { KW_BYP_METER, powerScaleAC, &ScreenMeters.BypassPower.sum },
    { KVA_BYP_METER, powerScaleAC, &ScreenMeters.BypassVoltAmperes.sum },
    { KW_INP_METER, powerScaleAC, &ScreenMeters.InputPower.sum },
    { KVA_INP_METER, powerScaleAC, &ScreenMeters.InputVoltAmperes.sum },
    { KW_OUTP_METER, powerScaleAC, &ScreenMeters.OutputPower.sum },
    { KVA_OUTP_METER, powerScaleAC, &ScreenMeters.OutputVoltAmperes.sum },
	{ KW_OUTP_METER_1,  powerScaleAC, &ScreenMeters.OutputPower.phA       },
    { KVA_OUTP_METER_1, powerScaleAC, &ScreenMeters.OutputVoltAmperes.phA },
    { KW_OUTP_METER_2,  powerScaleAC, &ScreenMeters.OutputPower.phB       },
    { KVA_OUTP_METER_2, powerScaleAC, &ScreenMeters.OutputVoltAmperes.phB },
    { KW_OUTP_METER_3,  powerScaleAC, &ScreenMeters.OutputPower.phC       },
    { KVA_OUTP_METER_3, powerScaleAC, &ScreenMeters.OutputVoltAmperes.phC }   
};
} // !namespace (anon)

void
InternalCanNetwork::SendUPMMeters()
{
    // Over-allocate three words to the meters array. This prevents a buffer
    // overrun in the last packet when looping over packets below.
    uint16_t meters[END_OF_INT16S_METERS + 3];
    memset(meters, sizeof(meters), 0);
    
    for (uint16_t i = 0; i < sizeof(canMeters) / sizeof(canMeters[0]); ++i)
    {
        const canMeter& meter = canMeters[i];
        float sample = (meter.location != NULL) ? *meter.location : 0.0;

        // For HB93E-10, phase-C Vo meter 220 low to 217(2 fail under 4unit); tempor just change meter show as (real volt val ok& ate record cali ok) 
        // because RMS loop not limit, and can not use Vo_rms for rms loop like 93P(as new cali ups will not compatible to old release one) 
        if((meter.packetAddr == V3_OUTP_METER))	//only C phase
        {
            if( MCUStateMachine.GetState() == ONLINE_STATE )
            {
                meters[meter.packetAddr] = saturated_cast<uint16_t>(InverterVoltageRMS.FilteredRMS.phC);
            }
            else
            {
                meters[meter.packetAddr] = saturated_cast<uint16_t>(ScreenMeters.OutputVoltageRMS.phC);
            }
        }
        else
        {		
            if (meter.packetAddr < END_OF_INT16U_METERS)
            {
                meters[meter.packetAddr] = saturated_cast<uint16_t>(meter.scale * sample);
            }
            else if (canMeters[i].packetAddr < END_OF_INT16S_METERS)
            {
                meters[meter.packetAddr] = uint16_t(saturated_cast<int16_t>(meter.scale * sample));
            }
        }
    }
    
#if CAN_PMF_METER_CRC
    const uint16_t stride = 3;
#else
    const uint16_t stride = 4;
#endif

    for (uint16_t i = 0; i < END_OF_INT16S_METERS; i += stride)
    {
        PacketTx(InternalCanPacket(CAN_PMF_METER0_TYPE + i/stride, CAN_PMF_METER_PRIORITY)
            .subtype(UPMNumber())
            .data0(meters[i])
            .data1(meters[i+1])
            .data2(meters[i+2])
#if CAN_PMF_METER_CRC
            .CrcPacket(), true);
#else
            .data3(meters[i+3]), true);
#endif
    }
}

//MyUPSNumber--> UPMNumber, APACTS-687: alarm UPM not response ,when more than 3 UPS 
void InternalCanNetwork::SendRTCrequest( void )
{
    PacketTx(InternalCanPacket(CAN_RTC_REQUEST, CAN_PRIORITY_LOW)
        .subtype(UPMNumber())
        .data0(0)
        .data1(0)
        .data2(0)
        .data3(0));
}

void
InternalCanNetwork::ReceiveC9Packet(const MBOX& packet, const void *cbdata)
{
    CsbC9StateMachine.insertMessage(packet);
}

void
InternalCanNetwork::ReceiveCommandPacket(const MBOX& packet, const void *)
{
    //Process commands.
    uint16_t command = packet.MDL.word.HI_WORD;
    uint16_t param1 = packet.MDL.word.LOW_WORD;
    uint16_t param2 = packet.MDH.word.HI_WORD;
    uint16_t crc = packet.MDH.word.LOW_WORD;
    eMCUState MCUState = MCUStateMachine.GetState();
    bool success = true;
        
    if (crc == (CAN_CRC_NONCE ^ command ^ param1 ^ param2))
    {
        switch (command)
        {
            case BYPASS_ON_COMMAND:
                                  
                 if( MyUPMNumber == 0 )
                 {
                     ParallelCan.InternalCommand.bit.csb_bypass_on_command = 1;
                 }     
                 break;
                 
            case UPS_NORMAL_COMMAND:

			     if( MyUPMNumber == 0 )
			     {
			 	     ParallelCan.InternalCommand.bit.csb_on_normal_command = 1;
					 MCUStateMachine.ParalSystemOn = true;					 
			     }   
                 break;
                 
            case UPS_LOAD_OFF_COMMAND:
				//for hobbit80k 2upm or more,when CAN_ext fail,direct load off from CAN_inner(CSB)
				 if(NB_GetNodebit( UPM_NB_PARALLEL_CAN_ERROR ))	
				 {
					 ParallelCan.InternalCommand.bit.csb_load_off_command = 1;
					 MCUStateMachine.ParalSystemOn = false; 				 				 	
				 }
				 else
				 {
	                 if( MyUPMNumber == 0 )
	                 {
	                     ParallelCan.InternalCommand.bit.csb_load_off_command = 1;
						 MCUStateMachine.ParalSystemOn = false;					 
	                 }  
				 }
                 break;
                 
            case ACTIVATE_CHARGER_COMMAND:                          
                 NB_DebounceAndQue( UPM_NB_CHARGER_ON_COMMAND, true );
                 Abm().ChargerCmdOn();
                 if( (MCUState == SHUTDOWN_STATE) || (MCUState == BYPASS_STATE) )
                 {
                     MCUStateMachine.AutoStandbyCommand();                    
                 }
                 BatteryConverter.CancelBatteryTest();
                 break;
                  
            case DEACTIVATE_CHARGER_COMMAND:
                 NB_DebounceAndQue( UPM_NB_CHARGER_OFF_COMMAND, true );
                 if( MCUStateMachine.InStandby() )
                 {
                     MCUStateMachine.StandbyChargerOffCommand();   
                 }
                 Abm().ChargerCmdOff();
                 BatteryConverter.CancelBatteryTest();
                 break;
                 
            case START_BATTERY_TEST_COMMAND:
                 // One time one UPM, the battery test command is for specified UPM.
                 if( MCUState != ESS_MODE_STATE )
                 {
                    BTR.StartDualStageBatteryTest();
                 }
                 else
                 {
                    MCUStateMachine.LeaveEcoForBatteryTest();
                 }                                 
//               if( MyUPMNumber == 0 )
//               {
//                   ParallelCan.InternalCommand.bit.csb_start_battery_test_command = 1;
//               }   
                 break;
                    
            case ECO_ON_COMMAND:
                 if( MyUPMNumber == 0 )
                 {
                     ParallelCan.InternalCommand.bit.csb_eco_on_command = 1;
                 }   
                 break;
                 
            case ECO_OFF_COMMAND:
                 if( MyUPMNumber == 0 )
                 {
                     ParallelCan.InternalCommand.bit.csb_eco_off_command = 1;
                 }   
                 break;
                 
            case RESET_ALARMS_COMMAND:
            	 ResetStickyAlarms();
                 //transmit over external can
                 ParallelCan.TransmitCommandPacket( pcan::para_reset_alarms_command);
                 ParallelCan.TransmitCommandPacket( pcan::para_reset_alarms_command);
                 ParallelCan.TransmitCommandPacket( pcan::para_reset_alarms_command);
                 break;
                    
            case CLEAR_HISTORY_COMMAND:
                 HistoryQueue.EraseAllEvents();
                 break;
            
            case UPS_OFF_COMMAND:
                 // this command removes a redundant unit from external parallel system
                 if( NB_GetNodebit( UPM_NB_EXTERNAL_PARALLEL )   && 
                     NB_GetNodebit( UPM_NB_SYSTEM_IS_REDUNDANT ) &&
                    !NB_GetNodebit( UPM_NB_REDUNDANCY_LOSS_DUE_TO_OVERLOAD ) )
                 {
                    //MCUStateMachine.StandbyCommand();
//                    ParallelCan.ParallelStatus.bit.UpsOffCmd = 1;
                    MCUStateMachine.Turn_UPS_Off();// single ups turn off
        			MCUStateMachine.ParalSystemOn = false;
                 }
                 break;

            case SYSTEM_VERIFIED_OK_COMMAND:
            	 InternalCan.SetSystemTypeCheckResult( true );
            	 success = false;  // this alarm is not a manual alarm
                 break;

            case SINGLE_UPS_NORMAL_COMMAND:
            	if(SingleUPSStartUpEnabled)
            	{
                    MCUStateMachine.NormalCommand();
                    ResetAlarms();
    			    MCUStateMachine.ParalSystemOn = false;
            	}
                 break;
                         
            default:
                 success = false;
                 break;
        }
    }
    else
    {
        success = false;
    }
    
    if (success)
    {
        ResetAlarms();
        MCUStateMachine.AutoBypassAllowed = false;
    }
    
    InternalCan.SendCommandResponse(success);
}

void
InternalCanNetwork::ReceiveBuildingInputPacket(const MBOX& packet, const void *)
{
    uint16_t building_inputs = packet.MDL.word.HI_WORD;
    uint16_t param1 = packet.MDL.word.LOW_WORD;
    uint16_t param2 = packet.MDH.word.HI_WORD;
    uint16_t crc = packet.MDH.word.LOW_WORD;
        
    if (crc == (CAN_CRC_NONCE ^ building_inputs ^ param1 ^ param2))
    {
        Rectifier.BldInpOnGenerator( (building_inputs & (1 << BI_ON_GENERATOR)) >> BI_ON_GENERATOR );        
        MCUStateMachine.BldInpRemoteCommand( building_inputs & ( (1 << BI_REMOTE_GO_TO_BYPASS) | (1 << BI_REMOTE_UPS_ON) | (1 << BI_REMOTE_UPS_OFF) ) );
        Abm().BldInpChargerOff( (building_inputs & (1 << BI_CHARGER_OFF)) >> BI_CHARGER_OFF );
        MCUStateMachine.BldInpBatteryDisconnected( (building_inputs & (1 << BI_BATTERY_DISCONNECTED)) >> BI_BATTERY_DISCONNECTED );
        MCUStateMachine.BldInpMaintenanceBypass( (building_inputs & (1 << BI_MAINTENANCE_BYPASS)) >> BI_MAINTENANCE_BYPASS );
        MCUStateMachine.BldInpMOBOpen( (building_inputs & (1 << BI_MOB_OPEN)) >> BI_MOB_OPEN );

		//add build-input bloc for ate
		MCUStateMachine.BuildingInputs.bit.BldInpOnGenerator 		  = ((building_inputs & ((uint16_t)1 << BI_ON_GENERATOR)) >> BI_ON_GENERATOR );	
		MCUStateMachine.BuildingInputs.bit.BldInpRemoteCommand_Bypass = ((building_inputs & ((uint16_t)1 << BI_REMOTE_UPS_ON)) >> BI_REMOTE_UPS_ON );	
		MCUStateMachine.BuildingInputs.bit.BldInpRemoteCommand_Online = ((building_inputs & ((uint16_t)1 << BI_REMOTE_GO_TO_BYPASS)) >> BI_REMOTE_GO_TO_BYPASS );	
		MCUStateMachine.BuildingInputs.bit.BldInpRemoteCommand_Loadoff= ((building_inputs & ((uint16_t)1 << BI_REMOTE_UPS_OFF)) >> BI_REMOTE_UPS_OFF );	

		MCUStateMachine.BuildingInputs.bit.BldInpChargerOff 		= ((building_inputs & ((uint16_t)1 << BI_CHARGER_OFF)) >> BI_CHARGER_OFF );	
		MCUStateMachine.BuildingInputs.bit.BldInpBatteryDisconnected= ((building_inputs & ((uint16_t)1 << BI_BATTERY_DISCONNECTED)) >> BI_BATTERY_DISCONNECTED );	
		MCUStateMachine.BuildingInputs.bit.BldInpMaintenanceBypass 	= ((building_inputs & ((uint16_t)1 << BI_MAINTENANCE_BYPASS)) >> BI_MAINTENANCE_BYPASS );	
		MCUStateMachine.BuildingInputs.bit.BldInpMOBOpen 			= ((building_inputs & ((uint16_t)1 << BI_MOB_OPEN)) >> BI_MOB_OPEN );	
    }
}

void
InternalCanNetwork::ReceiveCanIDPacket(const MBOX& packet, const void *)
{
    uint32_t header =  packet.MSGID.all;
    if (header & CAN_ID_SUBTYPE_MASK == 0)
    {
        // Message came from the CSB.
        // TODO: Do something with it. Probably a version-number check.  We can
        // also check for compatability with the other UPM's in the system.
    }   
}

void
InternalCanNetwork::ReceiveSectionResetRequest(const MBOX& packet, const void *)
{
    uint16_t section = packet.MDL.word.HI_WORD;
    uint16_t param1 = packet.MDL.word.LOW_WORD;
    uint16_t param2 = packet.MDH.word.HI_WORD;
    uint16_t crc = packet.MDH.word.LOW_WORD;
    
    if (crc == (CAN_CRC_NONCE ^ section ^ param1 ^ param2))
    {
        bool result;
        
    	if (MCUStateMachine.InvSupportingLoad())
    	{
    		result = false;
    	}
    	else 
    	{
	        uint16_t number_of_sections = GetNumEESections() - 1;
	        if (section < number_of_sections)
	        {
	            // Reset the EEPROM section.
	            ReBootEepromSection(section);
	            result = true;
	        }
	        else
	        {
	            // Incorrect section number.
	            result = false;
	        }
    	}
        // Send a reply to the CSB that operation is finished.
        InternalCan.SendSectionResetResponse(section, result);
    }
}

void
InternalCanNetwork::ReceiveSetUpsNumberPacket(const MBOX& packet, const void *closure)
{
    if( !DisableAutoID )
    {
        ParallelCan.SetAutoID();
        MyUPSNumber = packet.MDL.word.HI_WORD;
    }
}

void
InternalCanNetwork::SetSystemTypeCheckResult(bool result)
{
	SystemTypeCheckOk = result;
}

bool
InternalCanNetwork::GetSystemTypeCheckResult(void)
{
    return SystemTypeCheckOk;
}

// ********************************************************************************************************
// *
// * Function: ReceiveRTCPacket
// *
// * Purpose:  Receive RTC set or sync request from CSB. Forward over ECAN if needed.
// *
// *
// * Parms Passed   :   box:     Packet received over CAN.
// *
// * Returns        :   Nothing
// *
// ********************************************************************************************************
void InternalCanNetwork::ReceiveRTCPacket(const MBOX& box, const void *closure)
{
    // coding standard deviation:
    // static_cast<> used here for performance advantage versus cast-by-constructor.
    InternalCanPacket& packet = static_cast<InternalCanPacket&>(const_cast<MBOX&>(box));

    uint16_t packetType = packet.type();
    if ( ( CAN_RTC_SYNC == packetType ) || ( CAN_RTC_SET == packetType ) )
    {
        RTC_Time newRTC;
        newRTC.mSecOfMinute = packet.data0();
        newRTC.MinuteOfMonth = packet.data1();
        newRTC.YearAndMonth = packet.data2();
        uint16_t crc = packet.data3();
        if ( crc == ( CAN_CRC_NONCE ^ newRTC.mSecOfMinute ^ newRTC.MinuteOfMonth ^ newRTC.YearAndMonth ) )
        {
            // Update system time.
            CriticalSection enter;
            RTC_SysTime.mSecOfMinute = newRTC.mSecOfMinute;
            RTC_SysTime.MinuteOfMonth = newRTC.MinuteOfMonth;
            RTC_SysTime.YearAndMonth = newRTC.YearAndMonth;

          /*  if ( ParallelCan.UPSIsParallel() ) //for JIRA EOSD-172
            {
                // External paralllel RTC sync.
                // Forward over ECAN if any of these conditions are true.
                // a) RTC set packet. Forwarded regardless of who is the system master.
                // b) RTC sync packet and this UPS is the system master.
                if ( ( ParallelCan.ParallelStatus.bit.Master && ( CAN_RTC_SYNC == packetType ) ) ||
                     ( CAN_RTC_SET == packetType ) )
                {
                    //transform the internal packet to external packet.
                    ParallelCan.PacketTx( ParallelCanPacket( packetType, 4 )
                                          .data0( packet.data0() )
                                          .data1( packet.data1() )
                                          .data2( packet.data2() )
                                          .data3( packet.data3() ), CAN_RTC_SYNC == packetType );
                }
            }*/
        }
    }
}

void InternalCanNetwork::ReceiveCSBMetersPacket(const MBOX& packet, const void *closure)
{
    uint16_t meter0 = packet.MDL.word.HI_WORD;
    uint16_t meter1 = packet.MDL.word.LOW_WORD;
    uint16_t meter2 = packet.MDH.word.HI_WORD;
    uint16_t crc = packet.MDH.word.LOW_WORD;
    uint16_t metersType = (packet.MSGID.all & CAN_ID_TYPE_MASK) >> CAN_ID_TYPE_BITS;
    
    if(crc == ( CAN_CRC_NONCE ^ meter0 ^ meter1 ^ meter2 ) )
    {
        switch(metersType)
        {
            case CAN_CSB_METERS1:
                if( 0 == MyUPMNumber)
                {
                    ParallelCan.TransmitBroadcastPacket(pcan::csb_meters, meter0, meter1, meter2, crc, 0);
                }
                //The temperature compensation range is from 0 deg. C.  to 50 deg. C.   
                meter0 = ( meter0 <= 500 ) ? meter0 : 500;
                ParallelCan.BatteryTemperature[MyUPSNumber] = meter0;
                break;
            default:
                break;
        }
    }
}

uint_least8_t
InternalCanNetwork::UPMNumber()
{
    return MyUPMNumber + 1;
}

extern "C"
void ICanRx_TSK(void *)
{
    InternalCan.RxPacket_TSK();
}

#pragma CODE_SECTION("ramfuncs")
extern "C"
void ICan_Ch1_HWI(void)
{
    InternalCan.Channel1_HWI();
}

#pragma CODE_SECTION("ramfuncs")
extern "C"
void ICan_Ch0_HWI(void)
{
    InternalCan.Channel0_HWI();
}
