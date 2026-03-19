;********************************************************************
;*      UserInit.inc
;********************************************************************
;********************************************************************
;*
;* THIS INFORMATION IS PROPRIETARY TO INVENSYS Powerware Corporation
;*
;********************************************************************
;*
;*      Copyright (c) 1999, 2000, 2001, 2002, 2003 Invensys Powerware
;*                      ALL RIGHTS RESERVED
;*
;********************************************************************
;********************************************************************
;*      FILE NAME:   UserInit.Inc
;*
;*      DESCRIPTION: This file definitions for User Initialization
;*
;*
;*      ORIGINATOR:  Fred Tassitino
;*
;*      DATE:        10/22/2003
;*
;*      HISTORY:     See visual source safe history
;********************************************************************

;-----------------------------------------------------------------------
;RAM test testing pattern
;-----------------------------------------------------------------------

PATTERN         .set  0xA5F2        ; randomly created testing pattern 
RAM_FAIL        .set  0x0000ABCD    ; randomly created indication of RAM fail 

;-----------------------------------------------------------------------
;  CLEAR RAM DEFINITIONS, SKIP AREA's THAT YOU DO NOT WANT CLEARED
;-----------------------------------------------------------------------

ZERO1_START		.set  0h  			; M0, M1
ZERO1_END		.set  7ffh

ZERO2_START		.set  8000h  		; L0-L3
ZERO2_END		.set  0bfffh

ZERO3_START		.set  0c000h  		; L4-L7
ZERO3_END		.set  0ffffh




