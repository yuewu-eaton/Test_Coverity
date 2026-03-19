
// ******************************************************************************************************
// *            EncryptionVry.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2013 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: EncryptionVry.h
// *
// *    DESCRIPTION:
// *
// *    Author: E9908825
// *
// *    DATE: 2013-12-10
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#ifndef EncryptionVry_H_
#define EncryptionVry_H_

#include <stdint.h>

#define SHA204_SUCCESS              ((uint16_t)  0x00) //!< Function succeeded.
#define SHA204_Init_FAIL            ((uint16_t)  0xC0) //!< Init result as initial failed.
#define SHA204_CFG_FAIL             ((uint16_t)  0xC1) //!< Configure Configure Zone failed.
#define SHA204_DATA_FAIL            ((uint16_t)  0xC2) //!< Configure Data Zone failed.
#define SHA204_PARSE_ERROR          ((uint16_t)  0xD2) //!< response status byte indicates parsing error
#define SHA204_CMD_FAIL             ((uint16_t)  0xD3) //!< response status byte indicates command execution error
#define SHA204_STATUS_CRC           ((uint16_t)  0xD4) //!< response status byte indicates CRC error
#define SHA204_STATUS_UNKNOWN       ((uint16_t)  0xD5) //!< response status byte is unknown
#define SHA204_FUNC_FAIL            ((uint16_t)  0xE0) //!< Function could not execute due to incorrect condition / state.
#define SHA204_GEN_FAIL             ((uint16_t)  0xE1) //!< unspecified error
#define SHA204_BAD_PARAM            ((uint16_t)  0xE2) //!< bad argument (out of range, null pointer, etc.)
#define SHA204_INVALID_ID           ((uint16_t)  0xE3) //!< invalid device id, id not set
#define SHA204_INVALID_SIZE         ((uint16_t)  0xE4) //!< Count value is out of range or greater than buffer size.
#define SHA204_BAD_CRC              ((uint16_t)  0xE5) //!< incorrect CRC received
#define SHA204_RX_FAIL              ((uint16_t)  0xE6) //!< Timed out while waiting for response. Number of bytes received is > 0.
#define SHA204_RX_NO_RESPONSE       ((uint16_t)  0xE7) //!< Not an error while the Command layer is polling for a command response.
#define SHA204_RESYNC_WITH_WAKEUP   ((uint16_t)  0xE8) //!< re-synchronization succeeded, but only after generating a Wake-up

#define SHA204_COMM_FAIL            ((uint16_t)  0xF0) //!< Communication with device failed. Same as in hardware dependent modules.
#define SHA204_TIMEOUT              ((uint16_t)  0xF1) //!< Timed out while waiting for response. Number of bytes received is 0.

//////////////////////////////////////////////////////////////////////
// command op-code definitions
#define SHA204_CHECKMAC                 ((uint16_t) 0x28)       //!< CheckMac command op-code
#define SHA204_DERIVE_KEY               ((uint16_t) 0x1C)       //!< DeriveKey command op-code
#define SHA204_DEVREV                   ((uint16_t) 0x30)       //!< DevRev command op-code
#define SHA204_GENDIG                   ((uint16_t) 0x15)       //!< GenDig command op-code
#define SHA204_HMAC                     ((uint16_t) 0x11)       //!< HMAC command op-code
#define SHA204_LOCK                     ((uint16_t) 0x17)       //!< Lock command op-code
#define SHA204_MAC                      ((uint16_t) 0x08)       //!< MAC command op-code
#define SHA204_NONCE                    ((uint16_t) 0x16)       //!< Nonce command op-code
#define SHA204_PAUSE                    ((uint16_t) 0x01)       //!< Pause command op-code
#define SHA204_RANDOM                   ((uint16_t) 0x1B)       //!< Random command op-code
#define SHA204_READ                     ((uint16_t) 0x02)       //!< Read command op-code
#define SHA204_TEMPSENSE                ((uint16_t) 0x18)       //!< TempSense command op-code
#define SHA204_UPDATE_EXTRA             ((uint16_t) 0x20)       //!< UpdateExtra command op-code
#define SHA204_WRITE                    ((uint16_t) 0x12)       //!< Write command op-code

//////////////////////////////////////////////////////////////////////
// packet size definitions
#define SHA204_RSP_SIZE_VAL             ((uint16_t)  7)         //!< size of response packet containing four bytes of data: count(1 byte)+ 4 bytes + CRC(2 byte)

//////////////////////////////////////////////////////////////////////
// parameter range definitions
#define SHA204_KEY_ID_MAX               ((uint16_t) 15)         //!< maximum value for key id
#define SHA204_OTP_BLOCK_MAX            ((uint16_t)  1)         //!< maximum value for OTP block

//////////////////////////////////////////////////////////////////////
// definitions for command packet indexes common to all commands : address(1byte)+count(1)+opcode(1)+Param1(1)+Param2(2)+Data(N:0-32)+CRC(2)
#define SHA204_ADDR_IDX                 ( 0)
#define SHA204_SHIFT_IDX                ( 1)                   // Since we add addr as the first byte. some IDX need to shift one byte(Only write , not read)
#define SHA204_COUNT_IDX                ( 0 + SHA204_SHIFT_IDX)//!< command packet index for count
#define SHA204_OPCODE_IDX               ( 1 + SHA204_SHIFT_IDX)//!< command packet index for op-code
#define SHA204_PARAM1_IDX               ( 2 + SHA204_SHIFT_IDX)//!< command packet index for first parameter
#define SHA204_PARAM2_IDX               ( 3 + SHA204_SHIFT_IDX)//!< command packet index for second parameter
#define SHA204_DATA_IDX                 ( 5 + SHA204_SHIFT_IDX)//!< command packet index for second parameter

//////////////////////////////////////////////////////////////////////
// zone definitions
#define SHA204_ZONE_CONFIG              ((uint16_t)  0x00)      //!< Configuration zone
#define SHA204_ZONE_OTP                 ((uint16_t)  0x01)      //!< OTP (One Time Programming) zone
#define SHA204_ZONE_DATA                ((uint16_t)  0x02)      //!< Data zone
#define SHA204_ZONE_COUNT_FLAG          ((uint16_t)  0x80)      //!< Zone bit 7 set: Access 32 bytes, otherwise 4 bytes.
#define SHA204_ZONE_ACCESS_4            ((uint16_t)     4)      //!< Read or write 4 bytes.
#define SHA204_ZONE_ACCESS_32           ((uint16_t)    32)      //!< Read or write 32 bytes.
#define SHA204_ADDRESS_MASK_CONFIG      ((uint16_t)  0x1F)      //!< Address bits 5 to 7 are 0 for Configuration zone.
#define SHA204_ADDRESS_MASK_OTP         ((uint16_t)  0x0F)      //!< Address bits 4 to 7 are 0 for OTP zone.
#define SHA204_ADDRESS_MASK             ((uint16_t) 0x007F)    //!< Address bit 7 to 15 are always 0.
#define SHA204_DATA_BLOCK0              ((uint16_t)  0x00)
#define SHA204_DATA_BLOCK1              ((uint16_t)  0x08)
#define SHA204_DATA_BLOCK2              ((uint16_t)  0x10)
#define SHA204_DATA_BLOCK3              ((uint16_t)  0x18)
#define SHA204_DATA_BLOCK4              ((uint16_t)  0x20)
#define SHA204_DATA_BLOCK5              ((uint16_t)  0x28)
#define SHA204_DATA_BLOCK6              ((uint16_t)  0x30)
#define SHA204_DATA_BLOCK7              ((uint16_t)  0x38)
#define SHA204_DATA_BLOCK8              ((uint16_t)  0x40)
#define SHA204_DATA_BLOCK9              ((uint16_t)  0x48)
#define SHA204_DATA_BLOCK10             ((uint16_t)  0x50)
#define SHA204_DATA_BLOCK11             ((uint16_t)  0x58)
#define SHA204_DATA_BLOCK12             ((uint16_t)  0x60)
#define SHA204_DATA_BLOCK13             ((uint16_t)  0x68)
#define SHA204_DATA_BLOCK14             ((uint16_t)  0x70)
#define SHA204_DATA_BLOCK15             ((uint16_t)  0x78)
#define SHA204_OFFSET0                  ((uint16_t)  0x00)
#define SHA204_OFFSET1                  ((uint16_t)  0x01)
#define SHA204_OFFSET2                  ((uint16_t)  0x02)
#define SHA204_OFFSET3                  ((uint16_t)  0x03)
#define SHA204_OFFSET4                  ((uint16_t)  0x04)
#define SHA204_OFFSET5                  ((uint16_t)  0x05)
#define SHA204_OFFSET6                  ((uint16_t)  0x06)
#define SHA204_OFFSET7                  ((uint16_t)  0x07)
#define SHA204_CONFIG_BLOCK0            ((uint16_t)  0x00)
#define SHA204_CONFIG_BLOCK1            ((uint16_t)  0x08)
#define SHA204_CONFIG_BLOCK2            ((uint16_t)  0x10)
#define SHA204_OTP_BLOCK0               ((uint16_t)  0x00)
#define SHA204_OTP_BLOCK1               ((uint16_t)  0x08)


//////////////////////////////////////////////////////////////////////
// CheckMAC command definitions
#define CHECKMAC_MODE_IDX               SHA204_PARAM1_IDX      //!< CheckMAC command index for mode
#define CHECKMAC_KEYID_IDX              SHA204_PARAM2_IDX      //!< CheckMAC command index for key identifier
#define CHECKMAC_CLIENT_CHALLENGE_IDX   SHA204_DATA_IDX        //!< CheckMAC command index for client challenge
#define CHECKMAC_CLIENT_RESPONSE_IDX    (37 + SHA204_SHIFT_IDX)//!< CheckMAC command index for client response
#define CHECKMAC_DATA_IDX               (69 + SHA204_SHIFT_IDX)//!< CheckMAC command index for other data
#define CHECKMAC_COUNT                  (84)                   //!< CheckMAC command packet size
#define CHECKMAC_MODE_MASK              ((uint16_t) 0x27)       //!< CheckMAC mode bits 3, 4, 6, and 7 are 0.
#define CHECKMAC_CLIENT_CHALLENGE_SIZE  (32)                   //!< CheckMAC size of client challenge
#define CHECKMAC_CLIENT_RESPONSE_SIZE   (32)                   //!< CheckMAC size of client response
#define CHECKMAC_OTHER_DATA_SIZE        (13)                   //!< CheckMAC size of "other data"

//////////////////////////////////////////////////////////////////////
// DeriveKey command definitions
#define DERIVE_KEY_RANDOM_IDX           SHA204_PARAM1_IDX      //!< DeriveKey command index for random bit
#define DERIVE_KEY_TARGETKEY_IDX        SHA204_PARAM2_IDX      //!< DeriveKey command index for target slot
#define DERIVE_KEY_MAC_IDX              SHA204_DATA_IDX        //!< DeriveKey command index for optional MAC
#define DERIVE_KEY_COUNT_SMALL          SHA204_CMD_SIZE_MIN    //!< DeriveKey command packet size without MAC
#define DERIVE_KEY_COUNT_LARGE          (39)                   //!< DeriveKey command packet size with MAC
#define DERIVE_KEY_RANDOM_FLAG          ((uint16_t) 4)         //!< DeriveKey 1. parameter Only Input
#define DERIVE_KEY_MAC_SIZE             (32)                   //!< DeriveKey MAC size
#define DERIVE_KEY_PARENT               (2)                    //!< DeriveKey Parent Key slot
#define DERIVE_KEY_TARGET               (9)                    //!< DeriveKey Target Key slot

//////////////////////////////////////////////////////////////////////
// DevRev command definitions
#define DEVREV_PARAM1_IDX               SHA204_PARAM1_IDX      //!< DevRev command index for 1. parameter (ignored)
#define DEVREV_PARAM2_IDX               SHA204_PARAM2_IDX      //!< DevRev command index for 2. parameter (ignored)
#define DEVREV_COUNT                    SHA204_CMD_SIZE_MIN    //!< DevRev command packet size

//////////////////////////////////////////////////////////////////////
// GenDig command definitions
#define GENDIG_ZONE_IDX                 SHA204_PARAM1_IDX      //!< GenDig command index for zone
#define GENDIG_KEYID_IDX                SHA204_PARAM2_IDX      //!< GenDig command index for key id
#define GENDIG_DATA_IDX                 SHA204_DATA_IDX        //!< GenDig command index for optional data
#define GENDIG_COUNT                    SHA204_CMD_SIZE_MIN    //!< GenDig command packet size without "other data"
#define GENDIG_COUNT_DATA               (11)//!< GenDig command packet size with "other data"
#define GENDIG_OTHER_DATA_SIZE          (4)                    //!< GenDig size of "other data"
#define GENDIG_ZONE_OTP                 ((uint16_t) 1)          //!< GenDig zone id OTP
#define GENDIG_ZONE_DATA                ((uint16_t) 2)          //!< GenDig zone id data

//////////////////////////////////////////////////////////////////////
// HMAC command definitions
#define HMAC_MODE_IDX                   SHA204_PARAM1_IDX      //!< HMAC command index for mode
#define HMAC_KEYID_IDX                  SHA204_PARAM2_IDX      //!< HMAC command index for key id
#define HMAC_COUNT                      SHA204_CMD_SIZE_MIN    //!< HMAC command packet size
#define HMAC_MODE_MASK                  ((uint16_t) 0x74)       //!< HMAC mode bits 0, 1, 3, and 7 are 0.

//////////////////////////////////////////////////////////////////////
// Lock command definitions
#define LOCK_ZONE_IDX                   SHA204_PARAM1_IDX      //!< Lock command index for zone
#define LOCK_SUMMARY_IDX                SHA204_PARAM2_IDX      //!< Lock command index for summary
#define LOCK_COUNT                      SHA204_CMD_SIZE_MIN    //!< Lock command packet size
#define LOCK_ZONE_NO_CONFIG             ((uint16_t) 0x01)       //!< Lock zone is OTP or Data
#define LOCK_ZONE_NO_CRC                ((uint16_t) 0x80)       //!< Lock command: Ignore summary.
#define LOCK_ZONE_MASK                  (0x81)                 //!< Lock parameter 1 bits 2 to 6 are 0.

//////////////////////////////////////////////////////////////////////
// Mac command definitions
#define MAC_MODE_IDX                    SHA204_PARAM1_IDX      //!< MAC command index for mode
#define MAC_KEYID_IDX                   SHA204_PARAM2_IDX      //!< MAC command index for key id
#define MAC_CHALLENGE_IDX               SHA204_DATA_IDX        //!< MAC command index for optional challenge
#define MAC_COUNT_SHORT                 SHA204_CMD_SIZE_MIN    //!< MAC command packet size without challenge
#define MAC_COUNT_LONG                  (39)                   //!< MAC command packet size with challenge
#define MAC_MODE_BLOCK2_TEMPKEY         ((uint16_t) 0x01)       //!< MAC mode bit   0: second SHA block from TempKey
#define MAC_MODE_BLOCK1_TEMPKEY         ((uint16_t) 0x02)       //!< MAC mode bit   1: first SHA block from TempKey
#define MAC_MODE_SOURCE_FLAG_MATCH      ((uint16_t) 0x04)       //!< MAC mode bit   2: match TempKey.SourceFlag
#define MAC_MODE_PASSTHROUGH            ((uint16_t) 0x07)       //!< MAC mode bit 0-2: pass-through mode
#define MAC_MODE_INCLUDE_OTP_88         ((uint16_t) 0x10)       //!< MAC mode bit   4: include first 88 OTP bits
#define MAC_MODE_INCLUDE_OTP_64         ((uint16_t) 0x20)       //!< MAC mode bit   5: include first 64 OTP bits
#define MAC_MODE_INCLUDE_SN             ((uint16_t) 0x40)       //!< MAC mode bit   6: include serial number
#define MAC_MODE_INCLUDE_All            ((uint16_t) 0x50)       //!< MAC mode include serial number and 88 OTP bits
#define MAC_CHALLENGE_SIZE              (32)                   //!< MAC size of challenge
#define MAC_MODE_MASK                   ((uint16_t) 0x77)       //!< MAC mode bits 3 and 7 are 0.


//////////////////////////////////////////////////////////////////////
// Nonce command definitions
#define NONCE_MODE_IDX                  SHA204_PARAM1_IDX      //!< Nonce command index for mode
#define NONCE_PARAM2_IDX                SHA204_PARAM2_IDX      //!< Nonce command index for 2. parameter
#define NONCE_INPUT_IDX                 SHA204_DATA_IDX        //!< Nonce command index for input data
#define NONCE_COUNT_SHORT               (27 )                  //!< Nonce command packet size for 20 bytes of data
#define NONCE_COUNT_LONG                (39 )                  //!< Nonce command packet size for 32 bytes of data
#define NONCE_MODE_MASK                 ((uint16_t) 3)          //!< Nonce mode bits 2 to 7 are 0.
#define NONCE_MODE_SEED_UPDATE          ((uint16_t) 0x00)       //!< Nonce mode: update seed
#define NONCE_MODE_NO_SEED_UPDATE       ((uint16_t) 0x01)       //!< Nonce mode: do not update seed
#define NONCE_MODE_INVALID              ((uint16_t) 0x02)       //!< Nonce mode 2 is invalid.
#define NONCE_MODE_PASSTHROUGH          ((uint16_t) 0x03)       //!< Nonce mode: pass-through
#define NONCE_NUMIN_SIZE                (20)                   //!< Nonce data length
#define NONCE_NUMIN_SIZE_PASSTHROUGH    (32)                   //!< Nonce data length in pass-through mode (mode = 3)

//////////////////////////////////////////////////////////////////////
// Pause command definitions
#define PAUSE_SELECT_IDX                SHA204_PARAM1_IDX      //!< Pause command index for Selector
#define PAUSE_PARAM2_IDX                SHA204_PARAM2_IDX      //!< Pause command index for 2. parameter
#define PAUSE_COUNT                     SHA204_CMD_SIZE_MIN    //!< Pause command packet size

//////////////////////////////////////////////////////////////////////
// Random command definitions
#define RANDOM_MODE_IDX                 SHA204_PARAM1_IDX      //!< Random command index for mode
#define RANDOM_PARAM2_IDX               SHA204_PARAM2_IDX      //!< Random command index for 2. parameter
#define RANDOM_COUNT                    SHA204_CMD_SIZE_MIN    //!< Random command packet size
#define RANDOM_SEED_UPDATE              ((uint16_t) 0x00)       //!< Random mode for automatic seed update
#define RANDOM_NO_SEED_UPDATE           ((uint16_t) 0x01)       //!< Random mode for no seed update

//////////////////////////////////////////////////////////////////////
// Read command definitions
#define READ_ZONE_IDX                   SHA204_PARAM1_IDX      //!< Read command index for zone
#define READ_ADDR_IDX                   SHA204_PARAM2_IDX      //!< Read command index for address
#define READ_COUNT                      SHA204_CMD_SIZE_MIN    //!< Read command packet size
#define READ_ZONE_MASK                  ((uint16_t) 0x83)       //!< Read zone bits 2 to 6 are 0.
#define READ_ZONE_MODE_32_BYTES         ((uint16_t) 0x80)       //!< Read mode: 32 bytes

//////////////////////////////////////////////////////////////////////
// TempSense command definitions
#define TEMP_SENSE_PARAM1_IDX           SHA204_PARAM1_IDX      //!< TempSense command index for 1. parameter
#define TEMP_SENSE_PARAM2_IDX           SHA204_PARAM2_IDX      //!< TempSense command index for 2. parameter
#define TEMP_SENSE_COUNT                SHA204_CMD_SIZE_MIN    //!< TempSense command packet size

//////////////////////////////////////////////////////////////////////
// UpdateExtra command definitions
#define UPDATE_MODE_IDX                  SHA204_PARAM1_IDX     //!< UpdateExtra command index for mode
#define UPDATE_VALUE_IDX                 SHA204_PARAM2_IDX     //!< UpdateExtra command index for new value
#define UPDATE_COUNT                     SHA204_CMD_SIZE_MIN   //!< UpdateExtra command packet size
#define UPDATE_CONFIG_BYTE_86            ((uint16_t) 0x01)      //!< UpdateExtra mode: update Config byte 86

//////////////////////////////////////////////////////////////////////
// Write command definitions
#define WRITE_ZONE_IDX                  SHA204_PARAM1_IDX      //!< Write command index for zone
#define WRITE_ADDR_IDX                  SHA204_PARAM2_IDX      //!< Write command index for address
#define WRITE_VALUE_IDX                 SHA204_DATA_IDX        //!< Write command index for data
#define WRITE_MAC_VS_IDX                ( 9 + SHA204_SHIFT_IDX)//!< Write command index for MAC following short data
#define WRITE_MAC_VL_IDX                (37 + SHA204_SHIFT_IDX)//!< Write command index for MAC following long data
#define WRITE_COUNT_SHORT               (11)                   //!< Write command packet size with short data and no MAC
#define WRITE_COUNT_LONG                (39)                   //!< Write command packet size with long data and no MAC
#define WRITE_COUNT_SHORT_MAC           (43)                   //!< Write command packet size with short data and MAC
#define WRITE_COUNT_LONG_MAC            (71)                   //!< Write command packet size with long data and MAC
#define WRITE_MAC_SIZE                  (32)                   //!< Write MAC size
#define WRITE_ZONE_MASK                 ((uint16_t) 0xC3)       //!< Write zone bits 2 to 5 are 0.
#define WRITE_ZONE_WITH_MAC             ((uint16_t) 0x40)       //!< Write zone bit 6: write encrypted with MAC

//////////////////////////////////////////////////////////////////////
// Response size definitions
#define CHECKMAC_RSP_SIZE               SHA204_RSP_SIZE_MIN    //!< response size of DeriveKey command
#define DERIVE_KEY_RSP_SIZE             SHA204_RSP_SIZE_MIN    //!< response size of DeriveKey command
#define DEVREV_RSP_SIZE                 SHA204_RSP_SIZE_VAL    //!< response size of DevRev command returns 4 bytes
#define GENDIG_RSP_SIZE                 SHA204_RSP_SIZE_MIN    //!< response size of GenDig command
#define HMAC_RSP_SIZE                   SHA204_RSP_SIZE_MAX    //!< response size of HMAC command
#define LOCK_RSP_SIZE                   SHA204_RSP_SIZE_MIN    //!< response size of Lock command
#define MAC_RSP_SIZE                    SHA204_RSP_SIZE_MAX    //!< response size of MAC command
#define NONCE_RSP_SIZE_SHORT            SHA204_RSP_SIZE_MIN    //!< response size of Nonce command with mode[0:1] = 3
#define NONCE_RSP_SIZE_LONG             SHA204_RSP_SIZE_MAX    //!< response size of Nonce command
#define PAUSE_RSP_SIZE                  SHA204_RSP_SIZE_MIN    //!< response size of Pause command
#define RANDOM_RSP_SIZE                 SHA204_RSP_SIZE_MAX    //!< response size of Random command
#define READ_4_RSP_SIZE                 SHA204_RSP_SIZE_VAL    //!< response size of Read command when reading 4 bytes
#define READ_32_RSP_SIZE                SHA204_RSP_SIZE_MAX    //!< response size of Read command when reading 32 bytes
#define TEMP_SENSE_RSP_SIZE             SHA204_RSP_SIZE_VAL    //!< response size of TempSense command returns 4 bytes
#define UPDATE_RSP_SIZE                 SHA204_RSP_SIZE_MIN    //!< response size of UpdateExtra command
#define WRITE_RSP_SIZE                  SHA204_RSP_SIZE_MIN    //!< response size of Write command


//////////////////////////////////////////////////////////////////////
// command timing definitions for maximum execution times (ms)
//! CheckMAC maximum execution time
#define CHECKMAC_EXEC_MAX                (38)
//! DeriveKey maximum execution time
#define DERIVE_KEY_EXEC_MAX              (62)
//! DevRev maximum execution time
#define DEVREV_EXEC_MAX                  (2)
//! GenDig maximum execution time
#define GENDIG_EXEC_MAX                  (43)
//! HMAC maximum execution time
#define HMAC_EXEC_MAX                    (69)
//! Lock maximum execution time
#define LOCK_EXEC_MAX                    (24)
//! MAC maximum execution time
#define MAC_EXEC_MAX                     (35)
//! Nonce maximum execution time
#define NONCE_EXEC_MAX                   (60)
//! Pause maximum execution time
#define PAUSE_EXEC_MAX                   (2)
//! Random maximum execution time
#define RANDOM_EXEC_MAX                  (50)
//! Read maximum execution time
#define READ_EXEC_MAX                    (4)
//! TempSense maximum execution time
#define TEMP_SENSE_EXEC_MAX              (11)
//! UpdateExtra maximum execution time
#define UPDATE_EXEC_MAX                  (6)
//! Write maximum execution time
#define WRITE_EXEC_MAX                   (42)
//! maximum command delay
#define SHA204_COMMAND_EXEC_MAX          (69)

//! minimum number of bytes in command (from count byte to second CRC byte)
#define SHA204_CMD_SIZE_MIN          ((uint16_t)  7  )
//! maximum size of command packet (CheckMac)
#define SHA204_CMD_SIZE_MAX          ((uint16_t) 85  )
//! number of CRC bytes
#define SHA204_CRC_SIZE              ((uint16_t)  2)
//! Key max bytes
#define SHA204_KEY_MAX               ((uint16_t) 32)
//! Random out bytes
#define SHA204_RAND_BYTES            ((uint16_t) 32)
//! TempKey bytes
#define SHA204_TEMPKEY_BYTES         ((uint16_t) 32)
//! Serial Number bytes
#define SHA204_SN_BYTES              ((uint16_t) 9)
//! Hash results max bytes
#define SHA204_HASH_MAX              ((uint16_t) 32)
//! SHA204 max message in bytes
#define SHA204_MSG_MAX               ((uint16_t) 96)
//! Max message used in SHA256
#define SHA256_MSG_MAX               ((uint16_t) 128)

//! status byte after wake-up
#define SHA204_STATUS_BYTE_WAKEUP    ((uint16_t) 0x11)
//! command parse error
#define SHA204_STATUS_BYTE_PARSE     ((uint16_t) 0x03)
//! command execution error
#define SHA204_STATUS_BYTE_EXEC      ((uint16_t) 0x0F)
//! communication error
#define SHA204_STATUS_BYTE_COMM      ((uint16_t) 0xFF)

#define SHA204_RSP_SIZE_MIN          ((uint16_t)  4)  //!< minimum number of bytes in response
#define SHA204_RSP_SIZE_MAX          ((uint16_t) 35)  //!< maximum size of response packet

#define SHA204_BUFFER_POS_COUNT      (0 + SHA204_SHIFT_IDX)             //!< buffer index of count byte in command
#define SHA204_RSP_POS_COUNT         (0)                                //!< buffer index of count byte in response
#define SHA204_BUFFER_POS_DATA       (1 + SHA204_SHIFT_IDX)             //!< buffer index of data in response
#define SHA204_BUFFER_POS_STATE      (1)                                //!< buffer index of state in response

//! delay between Wakeup pulse and communication in ms
#define SHA204_WAKEUP_DELAY          (uint16_t) (3.0)


#define lastchunk 16*Nblocks

/*** SHA Function Macros *******************************/

#define SR(x,a)          (x >> a)                         /*Shift right Function - Shift x by a  */
#define ROTR(x,n)        (( x >> n ) | ( x << (32 - n ))) /* Rotate Right Function - rotate x by n */
#define ROTR25(x)		 (((x >> 9) >> 16) | ( x << 7 ))

/* Algoorithm defined logical functions */
#define Ch(x , y, z)    ((x & y) ^ (~x & z))
#define Maj(x, y, z)    ((x & y) ^ (x & z) ^ ( y & z))
/*Alternate Ch and MAJ functions that could be faster.*/  //With IAR optimixations, this method is not faster.
//#define Ch(x, y, z)      (((x) & ((y) ^ (z))) ^ (z))
//#define Maj(x, y, z)     (((x) & ((y) | (z))) | ((y) & (z)))
#define SIGZ(x)         (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22))    //M0
#define SIG1(x)          (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25))   //M1  (ROTR(x,6)  ^ ROTR(x,11) ^ ROTR25(x))
#define sigmaZ(x)       (ROTR(x,7) ^ ROTR(x,18) ^ SR(x,3))    // q0
#define sigma1(x)       (ROTR(x,17) ^ ROTR(x,19) ^ SR(x,10))  // q1

#define Mode_SHA256     1

#define VERIFY_MAX_TIME 5
/** \brief This enumeration lists all packet types sent to a SHA204 device.
 *
 * The following byte stream is sent to a SHA204 TWI device:
 *    {I2C start} {I2C address} {word address} [{data}] {I2C stop}.
 * Data are only sent after a word address of value #SHA204_I2C_PACKET_FUNCTION_NORMAL.
 */
enum i2c_word_address {
	SHA204_I2C_PACKET_FUNCTION_RESET = 0,  //!< Reset device.
	SHA204_I2C_PACKET_FUNCTION_SLEEP = 1,  //!< Put device into Sleep mode.
	SHA204_I2C_PACKET_FUNCTION_IDLE  = 2,   //!< Put device into Idle mode.
	SHA204_I2C_PACKET_FUNCTION_NORMAL= 3 //!< Write / evaluate data that follow this word address byte.
};


/** \brief This enumeration lists flags for I2C read or write addressing. */
enum i2c_read_write_flag {
	I2C_WRITE = (uint16_t) 0x00,  //!< write command flag
	I2C_READ  = (uint16_t) 0x01   //!< read command flag
};


extern uint16_t EncryptionTest;

class cEncypt_Verify
{
public:
	cEncypt_Verify();
	~cEncypt_Verify(){};
    uint16_t Configure_204( void );
    uint16_t Encryption_Result(void)
    {
    	return Configure_Result;
    }
    uint16_t Command[SHA204_CMD_SIZE_MAX/2 + 1];   // Command MAX is 85 bytes /2 + 1 = 43 words
    uint16_t Response[SHA204_RSP_SIZE_MAX/2 + 1];  // SHA204_RSP_SIZE_MAX = 35   in words we use 18 word to record
    uint32_t HashResult[SHA204_HASH_MAX/4];        // SHA204_HASH_MAX = 32 in words we use 16 words
    uint16_t *pHashResult;                         // Point to HashResult in uint16
    uint32_t Message[SHA256_MSG_MAX/4];            // SHA256_MSG_MAX  = 128
    uint16_t *pMessage;                            // Point to Message in uint16
    uint16_t TempKey[SHA204_TEMPKEY_BYTES/2];
    //bool ResReady;          // To indicate Response has finished receiving.
    bool I2C_Block;
    uint16_t Verify_Time;     // Verify time
    uint16_t MessageBytes;

private:
    uint16_t Sha204m_Read(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t zone, uint16_t address);
	uint16_t Sha204p_Wakeup(void);
    uint16_t sha204p_receive_response(uint16_t bcount, uint16_t *presponse);
    uint16_t Sha204c_Send_And_Receive(uint16_t *tx_buffer, uint16_t rx_size, uint16_t *rx_buffer,
    			uint16_t execution_delay);
    void Sha204c_Calculate_CRC(uint16_t length, uint16_t *data);
    uint16_t Sha204c_Check_Crc(const uint16_t *response);
    uint16_t Sha204m_Write(uint16_t *tx_buffer, uint16_t *rx_buffer,
    			uint16_t zone, uint16_t address, const uint16_t *new_value, const uint16_t *mac);
    uint16_t Sha204m_Update_Extra(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode, uint16_t new_value);
    uint16_t Sha204m_Random(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode);
    uint16_t Sha204m_Nonce(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode, const uint16_t *numin);
    uint16_t Sha204p_Idle(uint16_t *tx_buffer, uint16_t wakeup = 1);
    uint16_t Sha204p_Sleep(uint16_t *tx_buffer);
    uint16_t Sha204m_Lock(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t zone, uint16_t summary);
    uint16_t Sha204m_Mac(uint16_t *tx_buffer, uint16_t *rx_buffer,
    			uint16_t mode, uint16_t key_id, const uint16_t *challenge);
    uint16_t Sha204m_Derive_Key(uint16_t *tx_buffer, uint16_t *rx_buffer,
    			uint16_t random, uint16_t target_key, const uint16_t *mac);
    uint16_t Sha204m_Gen_Dig(uint16_t *tx_buffer, uint16_t *rx_buffer,
    			uint16_t zone, uint16_t key_id, const uint16_t *other_data);
    void Sha_Set_Message(uint16_t &MsgBytes, uint16_t SetBytes, const uint16_t *SourceMsg);
    void ClearShortData(uint16_t *ShortData, uint16_t DataNum);
    void ClearLongData(uint32_t *LongData, uint16_t DataNum);
    bool CheckResponse(const uint16_t *pResponse, const uint16_t *pHashResultCheck);

    void UpdateKey(uint16_t *TagetKey);
    void InitKey(uint16_t *TagetKey);

    uint16_t Configure_Result;
	uint16_t SerialNum[SHA204_SN_BYTES/2 + 1]; // SHA204_SN_Bytes is 9 need 9/2+1= 5 short word to store
	uint16_t Key[SHA204_KEY_MAX/2];

};

uint16_t ReadByteinWord(const uint16_t *SourceWord, uint16_t ByteNum );
void WriteBytetoWord(uint16_t *SourceWord, uint16_t ByteNum, uint16_t ByteData);
//extern cEncrypt_Verify Encrypt_Verify_204;
extern cEncypt_Verify Encrypt_Verify_204;
void SHA_256( uint32_t* const Message, uint64_t Mbit_Length, uint32_t *Hash, short mode);
void Swap32bitEndian(uint32_t *SwapMessage, uint16_t SwapWords);

#endif /* EncryptionVry_H_ */
