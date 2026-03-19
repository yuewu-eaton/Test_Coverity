#ifndef PANDA_CANIDS_H
#define PANDA_CANIDS_H

// ****************************************************************************
// *            CanIds.h
// ****************************************************************************
// ****************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ****************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ****************************************************************************
// ****************************************************************************
// *    FILE NAME: CanIds.h
// *
// *    DESCRIPTION: Defines the CAN Packet magic numbers
// *        WARNING FOR PANDA: Changes in this file must be exactly synchronized
// *       with the Panda CSB and UPM projects.
// *
// *    ORIGINATORS: Jonathan Brandmeyer and Jan-Erik Berger
// *
// *    DATE: May 28, 2010
// *
// ****************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Can packet identifiers. Each header definition provides the can packet's
 * priority, type, subtype, and routing. The use of the subtype field is packet-
 * dependent, and gets computed with a macro that is documented here.
 *
 * This header file must be both valid C and valid C++. Maintain it exactly equal
 * between CSB and UPM projects.
 *
 * Can Priority:
 * named: CAN_$(PACKET_GROUP)_PRIORITY
 * value: One of the values of enum can_priority. Within a type, the priorities
 * should be common.
 *
 * Can Type:
 * named: CAN_$(PACKET_NAME)_TYPE
 * value: between zero and 0xfff inclusive.
 *
 * Can Type mask:
 * named CAN_$(PACKET_GROUP)_MASK
 * value: between zero and 0xfff inclusive.
 * All of the CAN_$(PACKET_NAME)_TYPE values within a particular group shall be
 * covered by this mask. Fundamentally, each of the packet classes are divided
 * into groups similar to the way IP networks are defined. Each packet class is
 * assigned a base address and range. For example, the CAN status and meter
 * packets are in group 0xA00/8, which means that every packet whose upper four
 * type bits match 0xA is in this group. Smaller and larger packet groups may
 * be defined. However, recognize that there is a hard upper bound on the number
 * of groups that any one board may respond to in hardware. A maximum of 14 Rx
 * filters may be defined for the CSB.
 *
 * Packets with increasing numerical type have decreasing network priority.
 *
 * Can Subtype:
 * named: CAN_$(PACKET_NAME)_SUBTYPE(param1, param2, etc)
 * value: produces a value between 0 and 0x7f inclusive. If the UPM id is one of
 * the subtype fields, then it shall use the top three bits to indicate the UPM
 * id. If a packet's datatype is subdivided into multiple packets, then the lower
 * four bits should be used to indicate which packet this belongs to in the
 * sequence.
 *
 */

// General masks and bit definitions for the subfields in the CAN packet ID
// headers.
#define CAN_ID_SOURCE_MASK ((uint_least32_t)0x0F)
#define CAN_ID_SOURCE_BITS 0
#define CAN_ID_DEST_MASK ((uint_least32_t)0xF0)
#define CAN_ID_DEST_BITS 4

#define CAN_ID_SUBTYPE_BITS 8
#define CAN_ID_SUBTYPE_MASK ((uint_least32_t)0x7f << CAN_ID_SUBTYPE_BITS)
#define CAN_ID_TYPE_BITS 15
#define CAN_ID_TYPE_MASK ((uint_least32_t)0xfff << CAN_ID_TYPE_BITS)
#define CAN_ID_PRIORITY_BITS 27
#define CAN_ID_PRIORITY_MASK ((uint_least32_t)0x3 << CAN_ID_PRIORITY_BITS)
#define CAN_CSB_METERS_MASK ((uint_least32_t)0xff0 << CAN_ID_TYPE_BITS)

typedef enum _can_priority
{
    CAN_PRIORITY_HIGHEST,
    CAN_PRIORITY_HIGH,
    CAN_PRIORITY_LOW,
    CAN_PRIORITY_LOWEST
} can_priority;

// Commands sent from CSB to UPM.
#define CAN_UPM_COMMAND             ((uint_least16_t)0x100)
// Command response from UPM to CSB.
#define CAN_UPM_COMMAND_RESPONSE    ((uint_least16_t)0x101)

// Building input status sent from CSB to UPM.
#define CAN_UPM_BUILDING_INPUT      ((uint_least16_t)0x300)
// History log entries from the UPM to the CSB
#define CAN_UPM_HISTORY_PRIORITY    CAN_PRIORITY_LOW
#define CAN_UPM_HISTORY_LOG         ((uint_least16_t)0x400)
// Meters from CSB, reserved 0x501~0x50f
#define CAN_CSB_METERS_BASE         ((uint_least16_t)0x500)
#define CAN_CSB_METERS1             CAN_CSB_METERS_BASE

// RTC Send CAN ID
#define CAN_RTC_REQUEST             ((uint_least16_t)0x701)

// RTC Reveive mask & match
#define CAN_RTC_MASK                ((uint_least32_t)0xFF0 << CAN_ID_TYPE_BITS)
#define CAN_RTC_MATCH               ((uint_least32_t)0x700 << CAN_ID_TYPE_BITS)

// RTC Reveive CAN ID
#define CAN_RTC_SYNC                ((uint_least16_t)0x700)
#define CAN_RTC_SET                 ((uint_least16_t)0x702)

// Nodebit packets. Up to 128 UPM nodebits may be defined. The UPM transmits its
// nodebit status as an ordered bitfield in big-endian form. Packet NB0 includes
// nodebit states for index 0 through 63. Packet NB1 includes nodebit states
// for index 64 through 127. Additional packets shall be defined as needed. Bit
// set means that the nodebit is active. Bit clear means that the nodebit is
// inactive.
#define CAN_STATUS_METER_BASE ((uint_least16_t)0xA00)
#define CAN_STATUS_METER_MASK ((uint_least16_t)0xF00)
// Up to 64 packets in the status/meter block

#define CAN_PMF_NB_PRIORITY CAN_PRIORITY_LOW
#define CAN_PMF_NB_SUBTYPE(upm) (upm)
#define CAN_PMF_NB0_TYPE (CAN_STATUS_METER_BASE + 0)
#define CAN_PMF_NB1_TYPE (CAN_STATUS_METER_BASE + 1)
#define CAN_PMF_NB2_TYPE (CAN_STATUS_METER_BASE + 2)        // Pan/20121012 add 
// Leave a gap for NB2 and NB3 if needed later

// Meter packets. Each meter packet contains three 16-bit integer values, and is
// protected with an auxiliary CRC. Signedness and scale factor depend on the
// particular parameter. There are about 50 meters defined today, which would
// require 17 packets with three meters per packet, or 13 packets with four per
// packet
#define CAN_PMF_METER_PRIORITY CAN_PRIORITY_LOW
#define CAN_PMF_METER_SUBTYPE(upm) (upm)
// Set to nonzero if the meters packet contains the CRC.
#define CAN_PMF_METER_CRC       0
#define CAN_PMF_METER0_TYPE (CAN_STATUS_METER_BASE + 4)
#define CAN_PMF_METER1_TYPE (CAN_STATUS_METER_BASE + 5)
#define CAN_PMF_METER2_TYPE (CAN_STATUS_METER_BASE + 6)
#define CAN_PMF_METER3_TYPE (CAN_STATUS_METER_BASE + 7)
#define CAN_PMF_METER4_TYPE (CAN_STATUS_METER_BASE + 8)
#define CAN_PMF_METER5_TYPE (CAN_STATUS_METER_BASE + 9)
#define CAN_PMF_METER6_TYPE (CAN_STATUS_METER_BASE + 10)
#define CAN_PMF_METER7_TYPE (CAN_STATUS_METER_BASE + 11)
#define CAN_PMF_METER8_TYPE (CAN_STATUS_METER_BASE + 12)
#define CAN_PMF_METER9_TYPE (CAN_STATUS_METER_BASE + 13)
#define CAN_PMF_METER10_TYPE (CAN_STATUS_METER_BASE + 14)
#define CAN_PMF_METER11_TYPE (CAN_STATUS_METER_BASE + 15)
#define CAN_PMF_METER12_TYPE (CAN_STATUS_METER_BASE + 16)
#define CAN_PMF_METER13_TYPE (CAN_STATUS_METER_BASE + 17)
#define CAN_PMF_METER14_TYPE (CAN_STATUS_METER_BASE + 18)
#define CAN_PMF_METER15_TYPE (CAN_STATUS_METER_BASE + 19)
#define CAN_PMF_METER16_TYPE (CAN_STATUS_METER_BASE + 20)
#define CAN_PMF_METER17_TYPE (CAN_STATUS_METER_BASE + 21)
#define CAN_PMF_METER18_TYPE (CAN_STATUS_METER_BASE + 22)
#define CAN_PMF_METER19_TYPE (CAN_STATUS_METER_BASE + 23)

// The ID packet
#define CAN_PMF_ID_PRIORITY CAN_PRIORITY_LOW
#define CAN_PMF_ID_SUBTYPE(upm) (upm)
#define CAN_PMF_ID_TYPE          (CAN_STATUS_METER_BASE + 30)
#define CAN_PMF_XID_TYPE         (CAN_STATUS_METER_BASE + 31)
#define CAN_PMF_TO_CSB_AUTOID    (CAN_STATUS_METER_BASE + 32)
#define CAN_PMF_FROM_CSB_AUTOID  (CAN_STATUS_METER_BASE + 33)
#define CAN_PMF_ID_NOT_INSTALLED ((uint_least16_t)0x0000)
#define CAN_PMF_ID_NOT_AVAILABLE ((uint_least16_t)0xffff)

// C9 over CAN command set. See Panda_CSB/Documents/Communications/C9 over CAN.doc
// for rationale and other details.
#define CAN_C9_PRIORITY CAN_PRIORITY_LOWEST
#define CAN_C9_RESPONSE_BASE ((uint_least16_t)0xB00)
#define CAN_C9_RESPONSE_MASK ((uint_least16_t)0xF80)
#define CAN_C9_REQUEST_BASE ((uint_least16_t)0xB80)
#define CAN_C9_REQUEST_MASK ((uint_least16_t)0xF80)

// C9 address space bits
#define CAN_C9_SPACE_VARS ((uint_least8_t)0x0 << 5)
#define CAN_C9_SPACE_PARAMS ((uint_least8_t)0x1 << 5)
#define CAN_C9_SPACE_PROG_FLASH ((uint_least8_t)0x2 << 5)
#define CAN_C9_SPACE_BOOT_FLASH ((uint_least8_t)0x3 << 5)
#define CAN_C9_SPACE_MASK ((uint_least8_t)0x03 << 5)
#define CAN_C9_UPM_MASK ((uint_least8_t)0xf)

// Begin flash command
#define CAN_C9_BEGIN_FLASH_REQUEST CAN_C9_REQUEST_BASE
#define CAN_C9_BEGIN_FLASH_RESPONSE CAN_C9_RESPONSE_BASE
// Begin flash response error codes
#define CAN_C9_FLASH_READY 1

// Complete flash command
#define CAN_C9_COMPLETE_FLASH_REQUEST (CAN_C9_REQUEST_BASE + 1)
#define CAN_C9_COMPLETE_FLASH_RESPONSE (CAN_C9_RESPONSE_BASE + 1)

// Memory read request
#define CAN_C9_READ_REQUEST (CAN_C9_REQUEST_BASE + 2)
#define CAN_C9_READ_ERROR (CAN_C9_RESPONSE_BASE + 2)
// Allocation note: type ID's are allocated up to CAN_C9_READ_RESPONSE_BASE +
// 16.
#define CAN_C9_READ_RESPONSE_BASE (CAN_C9_RESPONSE_BASE + 4)
#define CAN_C9_READ_MAX ((size_t)64)
#define CAN_C9_READ_TIMEOUT 500

// Memory write request
#define CAN_C9_TWRITE_BEGIN (CAN_C9_REQUEST_BASE + 3)
// Allocation note: type ID's are allocated up to CAN_C9_TWRITE_DATA_BASE +
// 16.
#define CAN_C9_TWRITE_DATA_BASE (CAN_C9_REQUEST_BASE + 4)
#define CAN_C9_TWRITE_DATA_COUNT_BITS 4
#define CAN_C9_TWRITE_DATA_COUNT_MASK ((uint_least16_t)0x7 << CAN_C9_TWRITE_DATA_COUNT_BITS)
#define CAN_C9_TWRITE_RESPONSE (CAN_C9_RESPONSE_BASE + 3)
#define CAN_C9_TWRITE_MAX CAN_C9_READ_MAX
#define CAN_C9_TWRITE_TIMEOUT CAN_C9_READ_TIMEOUT

#define CAN_SECTION_RESET_REQUEST  ((uint_least16_t)0xC00)
#define CAN_SECTION_RESET_RESPONSE ((uint_least16_t)0xC01) 

// Response error codes
typedef enum _can_c9_error {
    CAN_C9_ERROR_INVALID_PACKET = 1,
    CAN_C9_ERROR_INVALID_ADDRESS,
    CAN_C9_ERROR_SUCCESS,
    CAN_C9_ERROR_INVALID_STATE,
    CAN_C9_ERROR_INVALID_VALUE,
    CAN_C9_ERROR_CHECKSUM,
    CAN_C9_ERROR_TIMEOUT,
    CAN_C9_ERROR_INPROGRESS,
    CAN_C9_ERROR_INVALID_COMMAND
} can_c9_error;

// The CRC always begins with this word, followed by the other
// three data words. At this time the "CRC" is just an XOR
// of the nonce and three data words.
#define CAN_CRC_NONCE ((uint_least16_t)0x9999)

// FOR REFERENCE INFORMATION ONLY
// The following represents the CAN ID allocation for Big Blue, usable as a
// rough template only. Design note: Most of the lowest-priority packets from
// Big Blue don't have an analogue for Panda. Therefore, in general, priorities
// have been shifted down by one, except for some of the most critical packets.

// Can packet groups:
// Reserved ID areas:
// These ranges may have gaps. This depends on the usage of
// the other fields: boards, destination, and source.

// 000 0011 0000 x     // SBM sync
// i.e. 0x00300000...0x00307FFF

// 000 0011 1000 x     // SBM commands
// i.e. 0x00380000...0x0038FFFF

// 000 0100 xxxx x     // PMF sync, and high priority cmd packets
// i.e. 0x00400000...0x004FFFFF

// 000 0100 xxxx x     // PMF low priority cmd packets (exluding 0000) (with low priority bit 0 set)
// i.e. 0x08400000...0x084FFFFF (with gaps: 0xD0 among others)

// 000 101x xxxx x     // PMF Status/Meter packets (with low priority bit 0 set)
// i.e. 0x08A00000...0x08BFFFFF

// 000 1100 0000 0     // Packets to event log (to CSB)
// i.e. 0x00C00000...0x00C07FFF

// 011 00xx xxxx x     // Virtual EEP packets lower bits are reserved for sequence number and command
//                     // (with low priority bit set)
// i.e. 0x0B000000...0x0B3FFFFF

// 011 1001 0000 x     // boot loader packets
// i.e. 0x03900000...0x0390FFFF

// 011 1011 0010 0     // Can bridge commands
// i.e. 0x03B20000...0x03B27FFF

// 011 1011 0010 1     // parallel monitoring (meters) packets
// i.e. 0x03B28000...0x03B2FFFF

// 100 0101 1000 x     // SBM data
// i.e. 0x04580000...0x0458FFFF

// 111 1011 0010 0     // Can bridge responds, data: version etc.
// i.e. 0x07B20000...0x07B27FFF

#define SYSTEM_KAYTU        0
#define SYSTEM_KUDZU        1
#define SYSTEM_AKULA        2
#define SYSTEM_KAYTUPLUS    3
#define SYSTEM_INDIGO       4
#define SYSTEM_BIGBLUE      5
#define SYSTEM_PANDA        6

#define SYSTEM_ID           SYSTEM_PANDA


#ifdef __cplusplus
}
#endif

#endif // !defined(PANDA_CANIDS_H)
