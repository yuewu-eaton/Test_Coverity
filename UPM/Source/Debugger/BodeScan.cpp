/*********************************************************************
File Name   : BodeScan.c 
Description : Scan bode graph for transfer function analysis with different frequences 
              from low to high, and transmit data with SCI to the PC for next processing.                             
Version     : V1.0
Author      : Yang Yifan  
CreateDate  : 2011-8-26
*********************************************************************/

#include "DSP28x_Project.h"
#include "BodeScan.h"
#include "Adc.h"

#define cSAMPFREQ   10204               // 19200
#define PI          3.14159265358979    // Pi
#define DFTN        256                 // N point number of DFT

uint16_t START_BODESCAN_MARK = 0;
uint16_t FINISH_BODESCAN_MARK = 0;

uint16_t uwF_SAMP;
uint32_t udwTIMECNT;
uint32_t udwTIME_OFFSET; /*begin to sample the DFT point when the responce has been steady*/
int16_t  wTheta;
int16_t  wInject;
uint16_t uwF_INPUT ;
uint16_t uwpreF_INPUT ;
uint16_t uwA_INPUT ;
uint16_t uwcntN = 0;
uint16_t dftn = 128;
uint16_t uwCircle;
float    fUnitTheta;
uint16_t BSCommandCnt = 0;
uint16_t BTCommandCnt = 0;
uint16_t BQCommandCnt = 0;
uint16_t InValidCommandCnt = 0;

const int16_t wSin_table512[512]=
       {0 	,201 ,402 	,603 	,804 	,1005 	,1205 	,1406 	,1606 	,1806 	,2006 	,2205 	,2404 	,2603 	,
        2801 	,2999 	,3196 	,3393 	,3590 	,3786 	,3981 	,4176 	,4370 	,4563 	,4756 	,4948 	,5139 	,
        5330 	,5520 	,5708 	,5897 	,6084 	,6270 	,6455 	,6639 	,6823 	,7005 	,7186 	,7366 	,7545 	,
        7723 	,7900 	,8076 	,8250 	,8423 	,8595 	,8765 	,8935 	,9102 	,9269 	,9434 	,9598 	,9760 	,
        9921 	,10080 	,10238 	,10394 	,10549 	,10702 	,10853 	,11003 	,11151 	,11297 	,11442 	,11585 	,11727 	,
        11866 	,12004 	,12140 	,12274 	,12406 	,12537 	,12665 	,12792 	,12916 	,13039 	,13160 	,13279 	,13395 	,
        13510 	,13623 	,13733 	,13842 	,13949 	,14053 	,14155 	,14256 	,14354 	,14449 	,14543 	,14635 	,14724 	,
        14811 	,14896 	,14978 	,15059 	,15137 	,15213 	,15286 	,15357 	,15426 	,15493 	,15557 	,15619 	,15679 	,
        15736 	,15791 	,15843 	,15893 	,15941 	,15986 	,16029 	,16069 	,16107 	,16143 	,16176 	,16207 	,16235 	,
        16261 	,16284 	,16305 	,16324 	,16340 	,16353 	,16364 	,16373 	,16379 	,16383 	,16384 	,16383 	,16379 	,
        16373 	,16364 	,16353 	,16340 	,16324 	,16305 	,16284 	,16261 	,16235 	,16207 	,16176 	,16143 	,16107 	,
        16069 	,16029 	,15986 	,15941 	,15893 	,15843 	,15791 	,15736 	,15679 	,15619 	,15557 	,15493 	,15426 	,
        15357 	,15286 	,15213 	,15137 	,15059 	,14978 	,14896 	,14811 	,14724 	,14635 	,14543 	,14449 	,14354 	,
        14256 	,14155 	,14053 	,13949 	,13842 	,13733 	,13623 	,13510 	,13395 	,13279 	,13160 	,13039 	,12916 	,
        12792 	,12665 	,12537 	,12406 	,12274 	,12140 	,12004 	,11866 	,11727 	,11585 	,11442 	,11297 	,11151 	,
        11003 	,10853 	,10702 	,10549 	,10394 	,10238 	,10080 	,9921 	,9760 	,9598 	,9434 	,9269 	,9102 	,
        8935 	,8765 	,8595 	,8423 	,8250 	,8076 	,7900 	,7723 	,7545 	,7366 	,7186 	,7005 	,6823 	,
        6639 	,6455 	,6270 	,6084 	,5897 	,5708 	,5520 	,5330 	,5139 	,4948 	,4756 	,4563 	,4370 	,
        4176 	,3981 	,3786 	,3590 	,3393 	,3196 	,2999 	,2801 	,2603 	,2404 	,2205 	,2006 	,1806 	,
        1606 	,1406 	,1205 	,1005 	,804 	,603 	,402 	,201 	,
        0 	,-201 	,-402 	,-603 	,-804 	,-1005 	,-1205 	,-1406 	,-1606 	,-1806 	,-2006 	,-2205 	,-2404 	,-2603 	,
        -2801 	,-2999 	,-3196 	,-3393 	,-3590 	,-3786 	,-3981 	,-4176 	,-4370 	,-4563 	,-4756 	,-4948 	,-5139 	,-5330 	,
        -5520 	,-5708 	,-5897 	,-6084 	,-6270 	,-6455 	,-6639 	,-6823 	,-7005 	,-7186 	,-7366 	,-7545 	,-7723 	,-7900 	,
        -8076 	,-8250 	,-8423 	,-8595 	,-8765 	,-8935 	,-9102 	,-9269 	,-9434 	,-9598 	,-9760 	,-9921 	,-10080 	,-10238 	,
        -10394 	,-10549 	,-10702 	,-10853 	,-11003 	,-11151 	,-11297 	,-11442 	,-11585 	,-11727 	,
        -11866 	,-12004 	,-12140 	,-12274 	,-12406 	,-12537 	,-12665 	,-12792 	,-12916 	,-13039 	,
        -13160 	,-13279 	,-13395 	,-13510 	,-13623 	,-13733 	,-13842 	,-13949 	,-14053 	,-14155 	,
        -14256 	,-14354 	,-14449 	,-14543 	,-14635 	,-14724 	,-14811 	,-14896 	,-14978 	,-15059 	,
        -15137 	,-15213 	,-15286 	,-15357 	,-15426 	,-15493 	,-15557 	,-15619 	,-15679 	,-15736 	,
        -15791 	,-15843 	,-15893 	,-15941 	,-15986 	,-16029 	,-16069 	,-16107 	,-16143 	,-16176 	,
        -16207 	,-16235 	,-16261 	,-16284 	,-16305 	,-16324 	,-16340 	,-16353 	,-16364 	,-16373 	,
        -16379 	,-16383 	,-16384 	,-16383 	,-16379 	,-16373 	,-16364 	,-16353 	,-16340 	,-16324 	,
        -16305 	,-16284 	,-16261 	,-16235 	,-16207 	,-16176 	,-16143 	,-16107 	,-16069 	,-16029 	,
        -15986 	,-15941 	,-15893 	,-15843 	,-15791 	,-15736 	,-15679 	,-15619 	,-15557 	,-15493 	,
        -15426 	,-15357 	,-15286 	,-15213 	,-15137 	,-15059 	,-14978 	,-14896 	,-14811 	,-14724 	,
        -14635 	,-14543 	,-14449 	,-14354 	,-14256 	,-14155 	,-14053 	,-13949 	,-13842 	,-13733 	,
        -13623 	,-13510 	,-13395 	,-13279 	,-13160 	,-13039 	,-12916 	,-12792 	,-12665 	,-12537 	,
        -12406 	,-12274 	,-12140 	,-12004 	,-11866 	,-11727 	,-11585 	,-11442 	,-11297 	,-11151 	,
        -11003 	,-10853 	,-10702 	,-10549 	,-10394 	,-10238 	,-10080 	,-9921 	,-9760 	,-9598 	,-9434 	,
        -9269 	,-9102 	,-8935 	,-8765 	,-8595 	,-8423 	,-8250 	,-8076 	,-7900 	,-7723 	,-7545 	,-7366 	,-7186 	,-7005 	,
        -6823 	,-6639 	,-6455 	,-6270 	,-6084 	,-5897 	,-5708 	,-5520 	,-5330 	,-5139 	,-4948 	,-4756 	,-4563 	,-4370 	,
        -4176 	,-3981 	,-3786 	,-3590 	,-3393 	,-3196 	,-2999 	,-2801 	,-2603 	,-2404 	,-2205 	,-2006 	,-1806 	,-1606 	,
        -1406 	,-1205 	,-1005 	,-804 	,-603 	,-402 	,-201 	
};

void BodeInit( void )
{
	START_BODESCAN_MARK = 0;
	FINISH_BODESCAN_MARK = 0;
    udwTIMECNT = 0;
    wTheta = 0;
    wInject = 0;
    uwF_SAMP = cSAMPFREQ;     // 192*50
    uwF_INPUT = 0;
    uwA_INPUT = 0;
    uwcntN = 0;
    dftn = DFTN;
	uwCircle = 6;
    udwTIME_OFFSET = 0;       //2 * uwCircle*(float)uwF_SAMP/(float)uwF_INPUT
    BSCommandCnt = 0;
    BTCommandCnt = 0;
    BQCommandCnt = 0;
    InValidCommandCnt = 0;
}

void BodeScan( uint16_t wSample )
{
    //finish bode scan
    if( uwF_INPUT == 0 )
    {
    	START_BODESCAN_MARK = 0; 
        udwTIMECNT = 0; 
        uwcntN = 0; 
        wInject = 0;
        return;
    }
    
    if( ( udwTIMECNT == udwTIME_OFFSET ) && ( uwF_INPUT != 0 ) )
    {     
        //finish detecting and reset BodeInit      
        uwF_INPUT = 0;
        START_BODESCAN_MARK = 0;
        FINISH_BODESCAN_MARK = 1;  
        udwTIMECNT = 0; 
        uwcntN = 0; 
        wInject = 0;       
    }
     
    //output inject signal
    wTheta = (int16_t)((float)udwTIMECNT * fUnitTheta);
    wTheta = wTheta % 512;
    wInject = (uint16_t)(((uint32_t)wSin_table512[wTheta] * uwA_INPUT) >> 9);//!!!!this value should be modify when execute, '>>5'is a origin vulue for simulating!!!   
    
    //store time series to buffer
    if( udwTIMECNT >= ( udwTIME_OFFSET >> 1 ) && uwF_INPUT != 0 )
    {   
    	uint16_t wBoundFreq = (uint16_t)(( (uint32_t)uwF_SAMP * 3 ) >> 7 );
    	
    	if( ( uwF_INPUT <= wBoundFreq ) || ( uwF_INPUT > wBoundFreq * 8 ) )    
	    {
	    	int16_t uwmod = 0;
	    	
	    	if( ( ( uwcntN * udwTIME_OFFSET ) & 0x00ff ) > 0x0080 )
	    	{
	    	    uwmod = 1;
	    	}
	    	
	        if(udwTIMECNT == (udwTIME_OFFSET >> 1) + ((uwcntN * udwTIME_OFFSET) >> 8) + uwmod)// "div dftn*2" equ ">>8" when dftn=128
	        {
	            if( uwcntN < 128 )
	            {
	              //uwBodeDataBuf[uwcntN]=wSample;//wPwmDuty_temp;//wInvPwm;//wInvVoltSample+2500;//this is where you want to get the response
	                CaptureData[uwcntN].s[0] = wSample;//wPwmDuty_temp;//wInvPwm;//wInvVoltSample+2500;//this is where you want to get the response
	            }
	            uwcntN++;
	        }
	    }
	    
	    if( ( uwF_INPUT > wBoundFreq ) && ( uwF_INPUT <= wBoundFreq * 8 ) )
	    {
	    	if( udwTIMECNT == ( udwTIME_OFFSET >> 1 ) + uwcntN )
	        {
	            if( uwcntN < dftn )
	            {
	              //uwBodeDataBuf[uwcntN]=wSample;//wPwmDuty_temp;//wInvPwm;//wInvVoltSample+2500;//this is where you want to get the response
	                CaptureData[uwcntN].s[0] = wSample;//wPwmDuty_temp;//wInvPwm;//wInvVoltSample+2500;//this is where you want to get the response	                
	            }
	            uwcntN++;
	        }
	    } 
    }
    
    udwTIMECNT++;
}
