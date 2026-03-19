;********************************************************************
;********************************************************************
;*
;* THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
;*
;********************************************************************
;*
;*      Copyright (c) 1999, 2000, 2001, 2002, 2003 Eaton
;*                      ALL RIGHTS RESERVED
;*
;********************************************************************
;********************************************************************
;*      FILE NAME:   Algos.asm
;*
;*      DESCRIPTION: This file contains the algorithms needed to
;*                   generate rectifier pwm.
;*
;*
;*     ORIGINATOR:  ASa
;*
;*     DATE:        6/5/2003
;*
;*     HISTORY:     See visual source safe history
;********************************************************************

;*********************************************************************
;        INCLUDE    FILES
;*********************************************************************


;*********************************************************************
;        Variable Declaration
;*********************************************************************

;*********************************************************************
;       LOCAL CONSTANTS USED IN THIS FILE ONLY
;*********************************************************************

    .sect   "ramfuncs"
    .global  _SecondOrderIIR

;*********************************************************************
;
;   Function Name   : _IIR_Second_Order;
;
;   Entry Assumptions:  ACC = new sample
;                       *XAR4 = address of filter structure
;
;   Register Usage:     XAR4
;
;   Stack Usage:        6
;
;   Function Exit :     32 bit result in ACC
;     
;
;   Function Description : Second order IIR transfer function. Normalized form:
;            H(z) = C1 * ((1 + B1*Z^-1 + B2*Z^-2)/(1 - A1*Z^-1 - A2*Z^-2)). Implemented in 
;            canonical direct form II:
;
;            x[n]--->C1--->[+]------->------->[+]---->y[n]
;                           \        \         \
;                           \      [1/Z]       \
;                           \        \         \
;                           \        \         \
;                          [+]<--A1------B1-->[+]
;                           \        \         \
;                           \      [1/Z]       \
;                           \        \         \
;                           \        \         \
;                          [+]<--A2------B2-->[+]
;
;           y[n] = C1x[n] - A1X1 - A2X2 + B1X1 + B2X2
;           X1   = C1x[n] - A1X1 - A2X2
;           X2   = X1
;           All data is assumed Q30 
;                           
;   C-prototype: INT32S IIR_Second_Order(INT32S sample, stSecondOrderIIR* filterTable);
;   Note: an extra word is allocated to local frame in case called from assembly
;   and SP not aligned. Is more correct to push ST1 and use asp, but is faster just
;   to allocate one extra word.
;                             
;*********************************************************************
_SecondOrderIIR:
        
        push    ST0                             ; save status register
        setc    SXM                             ; sign extension on
        setc    OVM                             ; overflow protection on
        spm     +1
        addb    SP, #5                          ; local variable space

        movl    XT, ACC                         ; x[n] sample to TREG
        qmpyl   P, XT, *XAR4++                  ; P = B0*x[n], XAR4->X1
        movl    XT, *XAR4++                     ; X1 to XT, XAR4->A1
        movl    ACC, P << PM                    ; ACC = (B0*x[n] << 1)
        qmpyal  P, XT, *XAR4++                  ; ACC = (B0*x[n] << 2), P = A1*X1, XAR4->B1
        subl    ACC, P << PM                    ; ACC = (B0*x[n] << 2) - (A1*X1 << 1)
        qmpysl  P, XT, *XAR4                    ; ACC = (B0*x[n] << 2) - (A1*X1 << 2), P = B1*X1, XAR4->B1
        movl    *-SP[4], ACC                    ; temp1 = B0*x[n] - A1*X1
        movl    XT, *+XAR4[2]                   ; XT = X2
        addl    ACC, P << PM                    ; ACC = (B0*x[n]<<2) - (A1*X1<<2) + (B1*X1<<1)
        qmpyal  P, XT, *+XAR4[6]                ; A = (B0*x[n]<<2) - (A1*X1<<2) + (B1*X1<<1), P = B2*X2
        addl    ACC, P << PM                    ; A = (B0*x[n]<<2) - (A1*X1<<2) + (B1*X1<<2)
        qmpyal  P, XT, *+XAR4[4]                ; A = (B0*x[n]<<2) - (A1*X1<<2) + (B1*X1<<2) + (B2*X2 << 1), P = A2*X2
        subl    ACC, P << PM                    ; A = result
        subl    ACC, P << PM                    ; twice for left shift 2
        movl    *-SP[2], ACC                    ; temp2 = result
        movl    ACC, *-SP[4]                    ; retrieve temp 1
        subl    ACC, P << PM                    ; 
        subl    ACC, P << PM                    ; ACC = B0+x[n] - A1X1 - A2X2
        subb    XAR4, #4                        ; XAR4->X1
        movl    P, *XAR4                        ; old X1 to P
        movl    *XAR4, ACC                      ; store new X1
        movl    *+XAR4[6], P                    ; store new X2
        movl    ACC, *-SP[2]                    ; retrieve result 

        subb    SP, #5                          ; restore stack pointer
        pop     ST0                             ; restore status register 0
        lretr                                   ;  


;*********************************************************************
;   End of __IQExp()
;*********************************************************************
        
;*********************************************************************
;   End of Algos.asm
;*********************************************************************
