// ********************************************************************************************************
// *            Version.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Version.h
// *
// *    DESCRIPTION: Provides access to version number.
// *                   
// *
// *    ORIGINATOR: Daniel Pope
// *
// *    DATE: 8/21/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************
#ifndef VERSION_H
#define VERSION_H
// *********************************************************************************************************
// *        INCLUDE FILES 
// *********************************************************************************************************
#include <stdint.h>

// ********************************************************************************************************
// *  Version Number and tagging practice:
// *
// *  Released code uses even number                1.00, 1.02, 1.04...
// *  Development code uses odd numbers             1.01, 1.03, 1.05...
//*
// *  Build number is changed via the SubWCRev.exe utility provided by
// *  TortoiseSVN.
// *
// ********************************************************************************************************  
#ifdef __cplusplus
extern "C" {
#endif

//20210308. first ver0
#define CONST_InterfaceBrdRev_ID_P0     6  //b110
//20210625, 1th new ver1: 
//	hw try add 1)Vbat+ sample,  but not correct  
//			   2)Curr limit ref new ratio
#define CONST_InterfaceBrdRev_ID_P1     1  //b001
//20210830, 2th new ver2:  
//	hw will add right: 1)Vbat+ sample 
//                     2)EPO_in righ circuit, then PLD_v>04 can use
#define CONST_InterfaceBrdRev_ID_P2     2  //b010
//next build(add pullchain...)
#define CONST_InterfaceBrdRev_ID_P3     3  //b011
// Hobbit 93PR T, UPM debugger block 24 InterfaceBoardRevID
#define CONST_InterfaceBrdRev_ID_P4     4  //b100
// Vbat ratio change to meet with battery voltage sample up to 50 blocks
#define CONST_InterfaceBrdRev_ID_P5     5  //b101
//precharge && fanfault update
#define CONST_InterfaceBrdRev_ID_P6     7  //b111

#define MACHINE_ID_PANDA_ESSENTIAL      ((uint16_t)590)
#define MACHINE_ID_PANDA_PREMIER        ((uint16_t)600)
#define MACHINE_ID_HOBBIT_93PR          ((uint16_t)720)
#define MACHINE_ID_ORIONDO              ((uint16_t)591)

//// For 3C3 HD 6.02.0008
//#define CONST_FirmwareVersion       602     //New 0.1.1 for Hobbit (basic 93E: 2.14.6)
//#define CONST_FirmwareBuildNum      9


// For 93PRT 9.00.0005
#define CONST_FirmwareVersion       1100 //902     //New 0.1.1 for Hobbit (basic 93E: 2.14.6)
#define CONST_FirmwareBuildNum      67 //9



#define CONST_Machine_ID            MACHINE_ID_ORIONDO

// *********************************************************************************************************
// *        GLOBAL DATA
// *********************************************************************************************************
extern uint16_t FirmwareVersion;
extern uint16_t FirmwareBuildNum;
extern uint16_t InterfaceBoardRevID;

void InitVersionNumber( void );
uint16_t BinToBcd(uint16_t Bin);
uint16_t BcdToBin(uint16_t Bcd);

#ifdef __cplusplus
}
#endif

// ********************************************************************************************************
// *            END OF Version.h
// ********************************************************************************************************
#endif

