// ********************************************************************************************************
// *            BootloaderAPI.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME   : BootloaderAPI.h
// *
// *    DESCRIPTION : 
// *
// *    ORIGINATORS : Jun Zhang
// *
// *    DATE        : 06/04/2010
// *
// *    HISTORY     : 
// *********************************************************************************************************
#ifndef _BOOTLOADER_API_H
#define _BOOTLOADER_API_H

// ********************************************************************
// *    Section Header Structure
// ********************************************************************

typedef struct 
{
    uint16_t MachineCode;
    uint16_t Version;
    uint16_t Port;
    uint16_t *FlashAddress;
    uint32_t FlashLength;
    uint16_t CpuId;
    uint16_t ErrorCode;
} stEntry_Header;

typedef void(*XCP_Bootloader_Entry)( uint16_t param );

struct st_Section_Header
{
    uint16_t                Checksum;        // Checksum
    uint16_t                ChkSumDisabled;  // Disable checksum
    uint32_t                Size;            // Section size in words
    XCP_Bootloader_Entry    EntryPoint;      // Function pointer to enter program
    uint16_t                Version;         // Version information 101 = 1.01
    uint16_t                Build;           // Build revision information
    uint16_t                MachineCode;     // Matching hardware id for firmware
};

// Checksum is disabled only when both ChkSumDisabled
// and Checksum fields have CHECKSUM_DISABLED keyword
#define CHECKSUM_ENABLED                ((uint16_t)0x0000)
#define CHECKSUM_DISABLED               ((uint16_t)0xA3A3)
#define MAINPROG_ENTRY                  ((XCP_Bootloader_Entry)0x0030000A)    // bootheader and mainheader don't align, this is actually _c_int00 of
                                                                              // application. Never called though, kind of sucky implementation
#define MAINPROG_SIZE                   ((uint32_t)0x00038000)
#define BOOTPROG_ENTRY                  ((uint32_t)0x0033FFF6)
#define BOOTPROG_SIZE                   ((uint32_t)0x00008000)

#define BOOTLOADER_HEADER_ADDRESS       ((uint32_t)0x00338000)

#define PORT_CAN                        ((uint16_t)0)
#define PORT_SCI                        ((uint16_t)1)

void Transfer2Bootloader( uint16_t space );

extern const struct st_Section_Header MainProg_Header;
extern const struct st_Section_Header BootProg_Header;

#endif

// ********************************************************************************************************
// *            END OF BootloaderAPI.h
// ********************************************************************************************************



