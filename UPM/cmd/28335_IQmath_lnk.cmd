/*
// TI File $Revision: /main/3 $
// Checkin $Date: July 9, 2008   14:12:45 $
//###########################################################################
//
// FILE:    28335_RAM_lnk.cmd
//
// TITLE:   Linker Command File For IQmath examples that run out of RAM
//
// NOTE; The example project uses memory protected by the
//       Code Security Module (CSM).  Make sure the CSM is
//       unlocked before you load the project.  One quick way
//       to do this on an erased device is to open a memory
//       window to the CSM password locations.  If these locations
//       read back 0xFFFF (or non-zero), then the CSM is unlocked:
//
//      Device    Password locations
//      28335:    0x33FFF8 - 0x33FFFF
//
//###########################################################################
// $TI Release: IQmath V1.5a $
// $Release Date: June 2, 2009 $
//###########################################################################
*/

SECTIONS
{
    /* Not defined in bios */

    .binit           : > FLASH,             PAGE = 0
    codestart        : > BEGIN              PAGE = 0
    MainHeader       : > MAINHEADER         PAGE = 0
    BootHeader       : > BOOTHEADER         PAGE = 0
    
    .adc_cal         : load = ADC_CAL,   PAGE = 0, TYPE = NOLOAD



/*** Run from RAM functions                  ***/
    ramfuncs         : LOAD = FLASH,        PAGE = 0    /* Load section to Flash */ 
                       RUN = RAMFUNCS,      PAGE = 0    /* Run section from RAM */
                       LOAD_START(_ramfuncs_loadstart),
	                   LOAD_SIZE(_ramfuncs_loadsize),
	                   RUN_START(_ramfuncs_runstart)
    ramdata          : LOAD = FLASH,        PAGE = 0    /* Load section to Flash */ 
                       RUN = L47SARAM,      PAGE = 1    /* Run section from RAM */
                       LOAD_START(_ramdata_loadstart),
	                   LOAD_SIZE(_ramdata_loadsize),
	                   RUN_START(_ramdata_runstart)
    .text:rtsram       LOAD = FLASH,        PAGE = 0    /* Load section to Flash */ 
                       RUN = RAMFUNCS,      PAGE = 0    /* Run section from RAM */
                       LOAD_START(_rtsram_loadstart),
	                   LOAD_SIZE(_rtsram_loadsize),
	                   RUN_START(_rtsram_runstart)
    {
                     rts2800_fpu32_fast_supplement.lib(.text)
    }

    FPUmathTables    : > FPUTABLES, PAGE = 0, TYPE = NOLOAD

 
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
