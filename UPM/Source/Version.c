// ********************************************************************************************************
// *            Version.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2004 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Version.c
// *
// *    DESCRIPTION: Contains version number
// *
// *    ORIGINATORS: Daniel Pope
// *
// *    DATE: 08/21/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES  (included files must have #ifndef protection)
// *********************************************************************************************************
#include <stdint.h>
#include "version.h"
#include "BootloaderAPI.h"

uint16_t FirmwareVersion; 
uint16_t FirmwareBuildNum;

void InitVersionNumber( void )
{
    if (MainProg_Header.Version)
    {
        FirmwareVersion = BinToBcd( MainProg_Header.Version / 100 ) << 8;
        FirmwareVersion += BinToBcd( MainProg_Header.Version % 100 );
    }
    else
    {
        FirmwareVersion = BinToBcd( CONST_FirmwareVersion / 100 ) << 8;
        FirmwareVersion += BinToBcd( CONST_FirmwareVersion % 100 );
    }
    
    FirmwareBuildNum = CONST_FirmwareBuildNum;
}

// *****************************************************************************
// *
// * Function: uint16_t BinToBcd(uint16_t Bin)
// *
// * Purpose: Convert between straight integer and Binary Coded Decimal formats.
// *
// * Parms Passed   :   Binary number.
// * Returns        :   Same number, encoded in Binary Coded Decimal
// * Description:
// *
// *****************************************************************************
uint16_t BinToBcd(uint16_t n)
{
uint16_t d3, d2, d1, d0, q;
//d4…d0 - decimal numbers
//Find n0…n3 numbers
d0 = n & 0xF;
d1 = (n>>4) & 0xF;
d2 = (n>>8) & 0xF;
d3 = (n>>12) & 0xF;
//Calculate d0…d4 numbers
d0 = 6*(d3 + d2 + d1) + d0;
q = d0 / 10;
d0 = d0 % 10;
d1 = q + 9*d3 + 5*d2 + d1;
q = d1 / 10;
d1 = d1 % 10;
d2 = q + 2*d2;
q = d2 / 10;
d2 = d2 % 10;
d3 = q + 4*d3;
//q = d3 / 10;
d3 = d3 % 10;
//d4 = q;
d0 |= ((d1<<4) & 0x00f0);
d0 |= ((d2<<8) & 0x0f00);
d0 |= ((d3<<12) & 0xf000);

return d0;
}

// *****************************************************************************
// *
// * Function: uint16_t BinToBcd(uint16_t Bin)
// *
// * Purpose: Convert between straight integer and Binary Coded Decimal formats.
// *
// * Parms Passed   :   BCD number.
// * Returns        :   Same number, encoded as an integer
// * Description:
// *
// *****************************************************************************
uint16_t BcdToBin(uint16_t Bcd)
{
    uint16_t mask = 0x000f;
    uint16_t multiplier = 1;
    uint16_t result = 0;
    uint16_t shift;
    for (shift = 0; shift < 16; shift += 4, multiplier *= 10)
    {
        uint16_t digit = (Bcd >> shift) & mask;
        result += digit * multiplier;
    }
    return result;
}

// ********************************************************************************************************
// *            END OF Version.c
// ********************************************************************************************************
