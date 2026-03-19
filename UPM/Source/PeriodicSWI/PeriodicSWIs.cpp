// ******************************************************************************************************
// *            PeriodicSWIs.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: PeriodicSWIs.c
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/19/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"         // Device Headerfile and Examples Include File
#include "IOexpansion.h"
#include "BatteryStateControl.h"
#include "InvSync.h"
#include "Alarms_AC.h"
#include "DebuggerBlocks.h"
#include "Alarms.h"
#include "InverterControl.h"
#include "Version.h"

extern "C" {
    void SWIFunc_33msSWI( void );
    void SWIFunc_5msSWI( void );

}

extern void UpdateRam( parameter_t param );

uint16_t InterfaceBoardRevID = 0;

const uint16_t hwMap[16] = 
{
    MACHINE_ID_PANDA_ESSENTIAL,       // Panda Essential
    MACHINE_ID_PANDA_PREMIER,         // Panda Premier
    2,                                // unknown
    3,                                // unknown 
    4,                                // unknown
    5,                                // unknown
    6,                                // unknown
    7,                                // unknown
    8,                                // unknown
    9,                                // unknown
    10,                               // unknown
    11,                               // unknown
    MACHINE_ID_ORIONDO,                // Oriondo_Tower
    13,                               // unknown
    MACHINE_ID_HOBBIT_93PR,           // Hobbit 93PR, ~InterfaceID 0001 = 1110 = 14
    15                                // unknown
};
    
const uint16_t IntCanIDMap[4] = 
{
    0,         // UPM1, INTCANID[2:1] = 00, MyUPMNumber = 0
    2,         // UPM3, INTCANID[2:1] = 01, MyUPMNumber = 2
    1,         // UPM2, INTCANID[2:1] = 10, MyUPMNumber = 1
    3          // UPM4, INTCANID[2:1] = 11, MyUPMNumber = 3
};       

//void I2c_Communication( void )
void I2c_Spi_Communication( void )		//SWI: 5ms
{
    static uint16_t cnt = 0;
    static bool initialize = true;
    
    if( ExpansionInputReg.Init == false)
    {
        ClearExpansionIOReg();
        return; // we have to wait until firmware has initialized ExpansionIO for read
    }
    else if( initialize )
    {
        switch ( cnt )
        {
            case 0:
                if (ReadExpansionIOReg())
                {
                    cnt++;
                }
                break;

            case 1:
                ReadPldVersion();
                ReadPldTrap();
                cnt++;
                
                break;

            case 2:
                if (ExpansionInputReg.bit.InterfaceID < 16)
                {
                    MyHardwareNumber = hwMap[(~ExpansionInputReg.bit.InterfaceID) & 0xf];
                }
                else
                {
                    // Safety. This branch should never be taken, it is in place only
                    // in the event that we add additional hardware ID bits.
                    MyHardwareNumber = ~ExpansionInputReg.bit.InterfaceID;
                }

				//1.ExpansionInputReg.bit.RevID=b11;  
//	                InterfaceBoardRevID = ~( ExpansionInputReg.bit.RevID | ( PLDStatusReg.bit.REV_ID3 << 2 ) );
				InterfaceBoardRevID = (ExpansionInputReg.bit.Rev_ID1) | (ExpansionInputReg.bit.Rev_ID2 << 1) | (ExpansionInputReg.bit.Rev_ID3 << 2);
                InterfaceBoardRevID &= 0x7;		//1.  
                PreChargeOff();//init precharge base different revid


                if (CONST_InterfaceBrdRev_ID_P5 == InterfaceBoardRevID)
                {
                    UpdateRam(PARAM_CAL_BatteryVoltageCal);
                }

                // Check software compatability
//	                EEStatusBits.bit.PLDVersionFail = CheckValidPLDVersion() ? 0 : 1;

                //Check control board compatability
                EEStatusBits.bit.CntlBoardVersionFail = CheckValidCntlVersion() ? 0 : 1;
                uint16_t idError = ((MyHardwareNumber == MACHINE_ID_PANDA_ESSENTIAL) || 
				                    (MyHardwareNumber == MACHINE_ID_HOBBIT_93PR) ||
				                    (MyHardwareNumber == MACHINE_ID_ORIONDO)) ? 0 : 0x10 + ExpansionInputReg.bit.InterfaceID;

                MyUPMNumber = IntCanIDMap[~ExpansionInputReg.bit.CanID & 0x3];
                
                cnt = 3;
                initialize = false;

                break;
            default:
                break;
        }
    }
    else
    {
        switch ( cnt )
        {
            case 3:
                ClearEPOTrap();
                cnt = 0;
                break;

            case 0:
            default:
                if ( ResetLatchedEPO() )
                {
                    ClearEPOTrap();
                    cnt = 1;
                }
                // Clear the PLD Trap if it is set, this shouldn't happen ever due to case 0
                //Bishlant/20140304 modified
                //else if ( PLDTrapReg.all != 0 ), ignore bit 6 and 5(OP: &0x9F) which is for UpmModel
                else if( PLDTrapReg.Trap != 0 )
                {
                    ClearPldlTrap();
                    cnt = 1;
                }
                else
                {
                    if ( ReadExpansionIOReg() )
                    {
                        cnt = 1;
                    }
                }
                break;

            case 1:
                WriteExpansionIOReg();
                cnt = 0;
                break;
        }
    }
}


// ***********************************************************************
// *
// *    FUNCTION: SWIFunc_333msSWI 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SWIFunc_33msSWI( void )
{
    BypassPLL.PLLPeriodicFunction();
    OutputPLL.PLLPeriodicFunction();
}

// ***********************************************************************
// *
// *    FUNCTION: SWIFunc_5msSWI 
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void SWIFunc_5msSWI( void )
{
    CheckSiteWiring();
    I2c_Spi_Communication();
    BatteryConverter.RunCharger();
    SEM_ipost( &PeriodicTaskSem );
}

// ******************************************************************************************************
// *            End of PeriodicSWIs.c
// ******************************************************************************************************
