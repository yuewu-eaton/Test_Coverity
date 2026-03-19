#ifndef PANDA_PARALLELCANIDS_H_
#define PANDA_PARALLELCANIDS_H_

#ifndef __cplusplus
#error This file is not legal C code
#endif

namespace pcan {
namespace {
/*
 * There are 13 bits available for each packet class.  The ID field of the
 * packet should always uniquely identify the packet.  Lower values indicate
 * higher priority.  The highest 5 bits uniquely determine the hardware priority
 * level for a single transmitter.  For a /8 block, all packets within the same
 * block will have the same effective hardware transmit priority.
 * 
 * For a /10 block, there are four possible priority levels within a block.
 */

// Informational packets.  These have the lowest priority, and are always
// broadcast to the entire UPS.
const uint16_t info_base = 0x1000u;
const uint16_t info_mask = 0x1f00u;

const uint16_t clock_sync         = info_base + 0;
const uint16_t machine_id         = info_base + 1;
const uint16_t parameter_checksum = info_base + 2;
const uint16_t inv_cal_meters0    = info_base + 3;
const uint16_t bypass_power_meter = info_base + 4;
const uint16_t loadshare_meter    = info_base + 5;
const uint16_t shared_params      = info_base + 6;
const uint16_t master_output_freq = info_base + 7;
const uint16_t csb_meters         = info_base + 8;

// Allocate a /10 block for status packets, one per state machine.   Status packets
// are always broadcasted to the entire UPS, but only when they change.
const uint16_t status_base = 0x0a00u;
const uint16_t status_mask = 0x1a00u;
// Time-critical status packets get transmitted before all other status changes.
const uint16_t status_base_crit = status_base;
const uint16_t sync_status      = status_base_crit + 0;
const uint16_t mcu_status       = status_base_crit + 1;
const uint16_t bypass_status    = status_base_crit + 2;

// TODO: Stop sending status packets for state machines not actively being used in
// global AND and OR bits.
const uint16_t status_base_used = status_base_crit + 0x100;
const uint16_t battery_status   = status_base_used + 0;
const uint16_t converter_status = status_base_used + 1;
const uint16_t inverter_status  = status_base_used + 2;
const uint16_t rectifier_status = status_base_used + 3;
const uint16_t system_error     = status_base_used + 4;
const uint16_t parallel_status  = status_base_used + 5;
const uint16_t abm_status       = status_base_used + 6;

// Allocate a /8 block for command packets.  Commands may be per-UPM or per-UPS.
const uint16_t command_base = 0x0100u;
const uint16_t command_mask = 0x1f00u;
const uint16_t bypass_command                  = command_base + 1;
const uint16_t mcu_command                     = command_base + 2;
const uint16_t auto_id_command                 = command_base + 3;
const uint16_t csb_forward_command             = command_base + 4;
const uint16_t master_rms_command              = command_base + 5;

// following commands can be processed by any UPM0 inside a unit
const uint16_t para_bypass_on_command          = command_base + 0x10;
const uint16_t para_ups_normal_commmand        = command_base + 0x11;
const uint16_t para_ups_load_off_command       = command_base + 0x12;
const uint16_t para_activate_charger_command   = command_base + 0x13;
const uint16_t para_deactivate_charger_command = command_base + 0x14;
const uint16_t para_start_battery_test_command = command_base + 0x15;
const uint16_t para_eco_on_command             = command_base + 0x16;
const uint16_t para_eco_off_command            = command_base + 0x17;
const uint16_t para_reset_alarms_command       = command_base + 0x18;
const uint16_t para_clear_history_command      = command_base + 0x19;
const uint16_t para_ups_to_battery_command     = command_base + 0x1A;
const uint16_t para_ups_from_battery_command   = command_base + 0x1B;
const uint16_t para_ups_cancel_start_command   = command_base + 0x1C;
const uint16_t para_ups_decline_start_command  = command_base + 0x1D;

// following commands master be handled by master in the system
const uint16_t sys_ups_normal_commmand         = command_base + 0x20;
const uint16_t sys_start_battery_test_command  = command_base + 0x21;
const uint16_t sys_eco_on_command              = command_base + 0x22;
const uint16_t sys_activate_charger_command    = command_base + 0x23;
const uint16_t sys_bypass_on_command           = command_base + 0x24;

// following commands are for fw to handle system state sync
const uint16_t fw_inverter_off_sync_command    = command_base + 0x30;
const uint16_t fw_clear_commands               = command_base + 0x31;
const uint16_t fw_redundant_off_command        = command_base + 0x32;
const uint16_t fw_redundant_battery_off_command = command_base + 0x33;

const uint16_t para_ups_ect_on_command         = command_base + 0x50;
const uint16_t sys_ups_ect_on_command          = command_base + 0x51;
const uint16_t para_ups_ect_off_command        = command_base + 0x52;
const uint16_t sys_ups_ect_off_command         = command_base + 0x53;
const uint16_t sys_ups_shut_down_command       = command_base + 0x54;
const uint16_t sys_ups_forward_transfer_command = command_base + 0x55;

// The sync state packet, whose priority dominates all others.
const uint16_t sync_type = 0x0000u;
const uint16_t sync_mask = 0x1fffu;

const uint32_t src_upm_mask = 0x0000ff00UL;
const uint32_t dst_upm_mask = 0x000000ffUL;

const uint32_t rtc_mask = 0x0ff0u;
const uint16_t rtc_base = 0x0700u;
const uint16_t rtc_sync = 0x0700u;
const uint16_t rtc_set  = 0x0702u;

/*
 * Given a type base or mask, shift it into position for the CAN ID mask and
 * match registers.
 */
inline uint32_t type_mask(uint16_t mask)
{
    return uint32_t(mask) << 16;
}

} } // !namespace pcan::(anonymous)


#endif /*PARALLELCANIDS_H_*/
