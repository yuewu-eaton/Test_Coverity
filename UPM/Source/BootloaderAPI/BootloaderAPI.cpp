// ********************************************************************************************************
// *            BootloaderAPI.c
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
// *    FILE NAME   : BootloaderAPI.c
// *
// *    DESCRIPTION : 
// *
// *    ORIGINATORS : Jun Zhang
// *
// *    DATE        : 06/04/2010
// *
// *    HISTORY     : 
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES  (included files must have #ifndef protection)
// *********************************************************************************************************
#include "DSP28x_Project.h"
#include "Version.h"
#include "MCUState.h"
#include "BootloaderAPI.h"
#include "InternalCan.h"
#include "Constants.h"
#include "SystemTrap.h"
#include "I2c_driver.h"

#pragma DATA_SECTION("BootHeader");
const struct st_Section_Header BootProg_Header; 

#pragma DATA_SECTION("MainHeader");
const struct st_Section_Header MainProg_Header = 
{
    CHECKSUM_DISABLED,
    CHECKSUM_DISABLED,
    MAINPROG_SIZE,
    MAINPROG_ENTRY,
    CONST_FirmwareVersion,
    CONST_FirmwareBuildNum,
    591
};

/* 
 * Verify conditions for transferring control to the bootloader, and do so if
 * the prerequisites are met.
 * 
 * @param space One of the address spaces specified by CAN_C9_SPACE_PROG_FLASH
 * 	or CAN_C9_SPACE_BOOT_FLASH
 * @return If the prerequisites are met, it doesn't.
 */
void Transfer2Bootloader(uint16_t space)
{   
    if ( MCUStateMachine.CheckOKToFlash() )
    {
        I2c_Enable = false;
        
        SetIOGood( false );
        
        DisableWatchdog();
        
        while( I2c_Get_Idle_Status() == false )
        {
            TSK_sleep( TSK_5_ms );
        }
        
        DINT;

    	BootProg_Header.EntryPoint(space);
    }
}

// ********************************************************************************************************
// *            END OF BootloaderAPI.c
// ********************************************************************************************************



