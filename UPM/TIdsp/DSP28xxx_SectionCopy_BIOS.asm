;############################################################################
;
; FILE:   DSP28xxx_SectionCopy_BIOS.asm
;
; DESCRIPTION:  Provides functionality for copying intialized sections from 
;				flash to ram at runtime before entering the _c_int00 startup
;				routine
;############################################################################
; Author: Tim Love
; Modified by Jonathan Brandmeyer for Eaton Corporation, based on 
; 		spraau8 and the '2812 bootloader
; Release Date: March 2008	
;############################################################################


	.ref _c_int00
	.include UserInit.inc
	.global copy_sections
	.global _ramfuncs_loadstart, _ramfuncs_loadsize, _ramfuncs_runstart
	.global _ramdata_loadstart, _ramdata_loadsize, _ramdata_runstart
	.global _rtsram_loadstart, _rtsram_loadsize, _rtsram_runstart
	
	
***********************************************************************
* Function: copy_sections
*
* Description: Copies initialized sections from flash to ram
***********************************************************************

	.sect ".text"

copy_sections:
	    ;************************************************************************************************
        ;* Zero memory ranges, taken from the 2812 bootloader project
        ;* ZERO RAM Range 1 from UserInit.inc
        ;************************************************************************************************
	movl	XAR2, #ZERO1_START						; Start Address to Zero
	mov		@AR0, #( (ZERO1_END - ZERO1_START)/2 )	; COMPUTE LENGTH
	mov		ACC, #0									; zero the accumulator	

_loop1:
	movl	*XAR2++, ACC							; ZERO LOCATION
	banz	_loop1, AR0--							; Zero all data specified

    	;************************************************************************************************
    	;* ZERO RAM Range 2 from UserInit.inc 
    	;************************************************************************************************
	movl	XAR2, #ZERO2_START						; Start Address to Zero
	mov		@AR0, #( (ZERO2_END - ZERO2_START)/2 )	; COMPUTE LENGTH
	mov		ACC, #0									; zero the accumulator	

_loop2:
	movl	*XAR2++, ACC							; ZERO LOCATION
	banz	_loop2, AR0--							; Zero all data specified
        ;************************************************************************************************
        ;* ZERO RAM Range 3 from UserInit.inc 
        ;************************************************************************************************
	movl	XAR2, #ZERO3_START						; Start Address to Zero
	mov		@AR0, #( (ZERO3_END - ZERO3_START)/2 )	; COMPUTE LENGTH
	mov		ACC, #0									; zero the accumulator	

_loop3:
	movl	*XAR2++, ACC							; ZERO LOCATION
	banz	_loop3, AR0--							; Zero all data specified
	
; copy-sections, originally from spraau8.  Only copy in ramfuncs
	MOVL XAR5,#_ramfuncs_loadsize ; Store Section Size in XAR5
	MOVL ACC,@XAR5 ; Move Section Size to ACC
	MOVL XAR6,#_ramfuncs_loadstart ; Store Load Starting Address in XAR6
	MOVL XAR7,#_ramfuncs_runstart ; Store Run Address in XAR7
	LCR copy ; Branch to Copy

	MOVL XAR5,#_ramdata_loadsize ; Store Section Size in XAR5
	MOVL ACC,@XAR5 ; Move Section Size to ACC
	MOVL XAR6,#_ramdata_loadstart ; Store Load Starting Address in XAR6
	MOVL XAR7,#_ramdata_runstart ; Store Run Address in XAR7
	LCR copy ; Branch to Copy
	
	MOVL XAR5,#_rtsram_loadsize ; Store Section Size in XAR5
	MOVL ACC,@XAR5 ; Move Section Size to ACC
	MOVL XAR6,#_rtsram_loadstart ; Store Load Starting Address in XAR6
	MOVL XAR7,#_rtsram_runstart ; Store Run Address in XAR7
	LCR copy ; Branch to Copy
	
	
    LB _c_int00				 			; Branch to start of boot.asm in RTS library

copy:	
	B return,EQ
	RPT AL								; Repeat Copy Instruction Until Complete Section is Copied
    || PWRITE  *XAR7, *XAR6++			; Copy From Load Address to Run Address
return:
	LRETR								; Return 

	.end
	
;//===========================================================================
;// End of file.
;//===========================================================================