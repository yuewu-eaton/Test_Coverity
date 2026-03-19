// ********************************************************************
// *            Thermal.cpp
// ********************************************************************
// ********************************************************************
// * 
// * This information is proprietry to Eaton Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 2005 Eaton Corporation                       
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   Thermal.cpp
// *                                                                      
// *     DESCRIPTION: 
// *                  
// *     ORIGINATOR:  Jun Zhang                                         
// *                                                                      
// *     DATE:        05/17/2010                                            
// *                                                                      
// *     HISTORY:                                                         
// ********************************************************************

// ********************************************************************
// *            Include files                                                                           
// ********************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include "Adc.h"
#include "Meters.h"
#include "BypassInterface.h"
#include "Rtc.h"
#include "IOexpansion.h"      
#include "Alarms.h"           
#include "Thermal.h"

// Thermal channel comparison in 9E 30kVA and 100kVA
// Channel          9E 30K          9E 100K
// PFC_Thermal0     PFC_Thermal0	PM_Thermal0
// PFC_Thermal1     PFC_Thermal1	PM_Thermal1
// PFC_Thermal2     NC	            PM_Thermal2
// INV_Thermal0     INV_Thermal0	BAT_Thermal
// INV_Thermal1     INV_Thermal1	NC
// INV_Thermal2     NC              NC
// AMB_Thermal      NC	            NC
enum ThermalTableType
{
    ThermalTableType_HV_20K_30K
};

//Constant definition in {rawdata : temperature }
#define cMaxThermalEntry      34
  

// Alex modified for new NTC
const int16_t ThermalTable_Hobbit_20K_80K[cMaxThermalEntry][2] =
{
    {   4031     ,   125	}   , // 0
    {   3848     ,   120	}   , // 1
    {   3649     ,   115	}   , // 2
    {   3437     ,   110	}   , // 3
    {   3212     ,   105	}   , // 4
    {   2976     ,   100	}   , // 5
    {   2732     ,   95	}   , // 6
    {   2484     ,   90	}   , // 7
    {   2236     ,   85	}   , // 8
    {   1991     ,   80	}   , // 9
    {   1754     ,   75	}   , // 10
    {   1529     ,   70	}   , // 11
    {   1319     ,   65	}   , // 12
    {   1125     ,   60	}   , // 13
    {   951     ,   55	}   , // 14
    {   794      ,   50	}   , // 15
    {   660     ,   45	}   , // 16
    {   543      ,   40	}   , // 17
    {   443      ,   35	}   , // 18
    {   359      ,   30	}   , // 19
    {   290      ,   25	}   , // 20
    {   233      ,   20	}   , // 21
    {   186      ,   15	}   , // 22
    {   148      ,   10	}   , // 23
    {   119      ,   5	}   , // 24
    {   95      ,   0	}   , // 25
    {   95      ,   0	}   , // 26
    {   95       ,   0	}   , // 27
    {   95       ,   0	}   , // 28
    {   95       ,   0	}   , // 29
    {   95       ,   0	}   , // 30
    {   95       ,   0	}   , // 31
    {   95        ,   0	}   , // 32
    {   95      ,   0	}   , // 33
};

//Ac Cap Rec thermal
const int16_t ThermalTable_Hobbit_20K_80K_AcCapRec[cMaxThermalEntry][2] =
{
    {   4028     ,   125	}   , // 0
    {   3854     ,   120	}   , // 1
    {   3668     ,   115	}   , // 2
    {   3471     ,   110	}   , // 3
    {   3263     ,   105	}   , // 4
    {   3046     ,   100	}   , // 5
    {   2822     ,   95	}   , // 6
    {   2595     ,   90	}   , // 7
    {   2367     ,   85	}   , // 8
    {   2147     ,   80	}   , // 9
    {   1930     ,   75	}   , // 10
    {   1720     ,   70	}   , // 11
    {   1519     ,   65	}   , // 12
    {   1330     ,   60	}   , // 13
    {   1154     ,   55	}   , // 14
    {   992      ,   50	}   , // 15
    {   848     ,   45	}   , // 16
    {   717      ,   40	}   , // 17
    {   602      ,   35	}   , // 18
    {   514      ,   30	}   , // 19
    {   403      ,   25	}   , // 20
    {   339      ,   20	}   , // 21
    {   276      ,   15	}   , // 22
    {   224      ,   10	}   , // 23
    {   180      ,   5	}   , // 24
    {   145       ,   0	}   , // 25
    {   145       ,   0	}   , // 26
    {   145        ,   0	}   , // 27
    {   145        ,   0	}   , // 28
    {   145        ,   0	}   , // 29
    {   145        ,   0	}   , // 30
    {   145        ,   0	}   , // 31
    {   145         ,   0	}   , // 32
    {   145       ,   0	}   , // 33
};

//Ac Cap Inv thermal
const int16_t ThermalTable_Hobbit_20K_80K_AcCapInv[cMaxThermalEntry][2] =
{
    {   4028     ,   125	}   , // 0
    {   3854     ,   120	}   , // 1
    {   3668     ,   115	}   , // 2
    {   3471     ,   110	}   , // 3
    {   3263     ,   105	}   , // 4
    {   3046     ,   100	}   , // 5
    {   2822     ,   95	}   , // 6
    {   2595     ,   90	}   , // 7
    {   2367     ,   85	}   , // 8
    {   2147     ,   80	}   , // 9
    {   1930     ,   75	}   , // 10
    {   1720     ,   70	}   , // 11
    {   1519     ,   65	}   , // 12
    {   1330     ,   60	}   , // 13
    {   1154     ,   55	}   , // 14
    {   992      ,   50	}   , // 15
    {   848     ,   45	}   , // 16
    {   717      ,   40	}   , // 17
    {   602      ,   35	}   , // 18
    {   514      ,   30	}   , // 19
    {   403      ,   25	}   , // 20
    {   339      ,   20	}   , // 21
    {   276      ,   15	}   , // 22
    {   224      ,   10	}   , // 23
    {   180      ,   5	}   , // 24
    {   145       ,   0	}   , // 25
    {   145       ,   0	}   , // 26
    {   145        ,   0	}   , // 27
    {   145        ,   0	}   , // 28
    {   145        ,   0	}   , // 29
    {   145        ,   0	}   , // 30
    {   145        ,   0	}   , // 31
    {   145         ,   0	}   , // 32
    {   145       ,   0	}   , // 33
};


// Global data
uint16_t PHASEB_Thermal0_avg = 3413;    // average thermal rawdata
uint16_t BAT_Thermal_avg = 3413;
uint16_t PHASEA_Thermal0_avg = 3413;
uint16_t PHASEC_Thermal_avg = 3413;
uint16_t INVCAP_Thermal_avg  = 3413;
uint16_t SCR_Thermal_avg  = 3413;
float AMB_Thermal_avg  = 20.0f;

uint32_t PHASEB_Thermal0_sum = 0;       // sum of thermal rawdata
uint32_t BAT_Thermal_sum = 0;
uint32_t PHASEA_Thermal0_sum = 0;
uint32_t PHASEC_Thermal_sum = 0;
uint32_t INVCAP_Thermal_sum = 0;
uint32_t SCR_Thermal_sum = 0;
float AMB_Thermal_sum  = 0.0f;

uint16_t PHASEB_Thermal0_cnt = 0;       // sum counts
uint16_t BAT_Thermal_cnt = 0;
uint16_t PHASEA_Thermal0_cnt = 0;
uint16_t PHASEC_Thermal_cnt = 0;
uint32_t INVCAP_Thermal_cnt = 0;
uint32_t SCR_Thermal_cnt = 0;
uint16_t AMB_Thermal_cnt  = 0;

int16_t PHASEB_Thermal_C = 20;         // real temperature in Centigrade
int16_t PHASEA_Thermal_C = 20;
int16_t PHASEC_Thermal_C = 20;
int16_t BAT_Thermal_C = 20;
int16_t INVCAP_Thermal_C = 20;
int16_t SCR_Thermal_C = 20;
float AMB_Thermal_C  = 20.0f;  //degree centigrade
float AMB_Thermal_K  = 293.15f; //degree Kelvin

int16_t PHASEB_Thermal_Max = 0;         // for 9E 30K - warning & trip check
int16_t PHASEB_Thermal_Min = 0;
int16_t PHASEA_Thermal_Max = 0;
int16_t PHASEA_Thermal_Min = 0;

int16_t BAT_Thermal_Max = 0;
int16_t BAT_Thermal_Min = 0;
int16_t PHASEC_Thermal_Max = 0;
int16_t INVCAP_Thermal_Max = 0;
//int16_t RECCAP_Thermal_Max = 0;


inline int16_t FindMax(int16_t data1,int16_t data2)
{
    if(data1 > data2)
    {
        return(data1);
    }
    
    return(data2);    
}

inline int16_t FindMin(int16_t data1, int16_t data2)
{
    if(data1 < data2)
    {
        return(data1);
    }
    
    return(data2);    
}

inline int16_t FindMax(int16_t data1,int16_t data2,int16_t data3)
{
    int16_t data;
    data = data1>data2?data1:data2;
    data = data>data3?data:data3;
    return(data);    
}

inline int16_t FindMin(int16_t data1,int16_t data2,int16_t data3)
{
    int16_t data;
    data = data1<data2?data1:data2;
    data = data<data3?data:data3;
    return(data);    
}

// ********************************************************************
// *                                                                                                                     
// * Function     : FindTemperature() 
// *
// * Purpose      : Find real temperature based on raw sampling data
// *                    
// * Parms Passed : rawdata - Average raw thermal data
// *
// * Returns      : Real temperature in degree                                                                           
// *
// * Description  : This function is used to find the real temperature
// *                in degree based on the passed raw sampling data.
// ********************************************************************
int16_t FindTemperature(uint16_t rawdata)
{
    int16_t cnt = 0;
    const int16_t (*ThermalTable)[cMaxThermalEntry][2];

    ThermalTable = &ThermalTable_Hobbit_20K_80K;
    
    if(rawdata >= (*ThermalTable)[0][0])
    {
        return((*ThermalTable)[0][1]);
    }
    else if(rawdata <= (*ThermalTable)[cMaxThermalEntry - 1][0])
    {
        return((*ThermalTable)[cMaxThermalEntry - 1][1]);
    }
    
    for(cnt = 0;cnt < cMaxThermalEntry - 1;cnt++)
    {
        if((rawdata <= (*ThermalTable)[cnt][0] + 5) && \
           (rawdata >= (*ThermalTable)[cnt][0] - 5))
        {
            return((*ThermalTable)[cnt][1]);
        }
        else if((rawdata < (*ThermalTable)[cnt][0]) && \
                (rawdata > (*ThermalTable)[cnt + 1][0]))
        {
            float temp_result = (float)( rawdata - (*ThermalTable)[cnt + 1][0] )/( (*ThermalTable)[cnt][0] - (*ThermalTable)[cnt + 1][0] );
            temp_result *= 5;
            temp_result += (*ThermalTable)[cnt + 1][1];
            return((int16_t)temp_result);
        }
    }
    
    return(0);
}

// ********************************************************************
// *                                                                                                                     
// * Function     : FindTemperature_AcCapRec() 
// *
// * Purpose      : Find real temperature based on raw sampling data
// *                    
// * Parms Passed : rawdata - Average raw thermal data
// *
// * Returns      : Real temperature in degree                                                                           
// *
// * Description  : This function is used to find the real temperature
// *                in degree based on the passed raw sampling data.
// ********************************************************************
int16_t FindTemperature_AcCapRec(uint16_t rawdata)
{
    int16_t cnt = 0;
    const int16_t (*ThermalTable)[cMaxThermalEntry][2];

    ThermalTable = &ThermalTable_Hobbit_20K_80K_AcCapRec;
    
    if(rawdata >= (*ThermalTable)[0][0])
    {
        return((*ThermalTable)[0][1]);
    }
    else if(rawdata <= (*ThermalTable)[cMaxThermalEntry - 1][0])
    {
        return((*ThermalTable)[cMaxThermalEntry - 1][1]);
    }
    
    for(cnt = 0;cnt < cMaxThermalEntry - 1;cnt++)
    {
        if((rawdata <= (*ThermalTable)[cnt][0] + 5) && \
           (rawdata >= (*ThermalTable)[cnt][0] - 5))
        {
            return((*ThermalTable)[cnt][1]);
        }
        else if((rawdata < (*ThermalTable)[cnt][0]) && \
                (rawdata > (*ThermalTable)[cnt + 1][0]))
        {
            float temp_result = (float)( rawdata - (*ThermalTable)[cnt + 1][0] )/( (*ThermalTable)[cnt][0] - (*ThermalTable)[cnt + 1][0] );
            temp_result *= 5;
            temp_result += (*ThermalTable)[cnt + 1][1];
            return((int16_t)temp_result);
        }
    }
    
    return(0);
}

// ********************************************************************
// *                                                                                                                     
// * Function     : FindTemperature_AcCapInv() 
// *
// * Purpose      : Find real temperature based on raw sampling data
// *                    
// * Parms Passed : rawdata - Average raw thermal data
// *
// * Returns      : Real temperature in degree                                                                           
// *
// * Description  : This function is used to find the real temperature
// *                in degree based on the passed raw sampling data.
// ********************************************************************
int16_t FindTemperature_AcCapInv(uint16_t rawdata)
{
    int16_t cnt = 0;
    const int16_t (*ThermalTable)[cMaxThermalEntry][2];

    ThermalTable = &ThermalTable_Hobbit_20K_80K_AcCapInv;
    
    if(rawdata >= (*ThermalTable)[0][0])
    {
        return((*ThermalTable)[0][1]);
    }
    else if(rawdata <= (*ThermalTable)[cMaxThermalEntry - 1][0])
    {
        return((*ThermalTable)[cMaxThermalEntry - 1][1]);
    }
    
    for(cnt = 0;cnt < cMaxThermalEntry - 1;cnt++)
    {
        if((rawdata <= (*ThermalTable)[cnt][0] + 5) && \
           (rawdata >= (*ThermalTable)[cnt][0] - 5))
        {
            return((*ThermalTable)[cnt][1]);
        }
        else if((rawdata < (*ThermalTable)[cnt][0]) && \
                (rawdata > (*ThermalTable)[cnt + 1][0]))
        {
            float temp_result = (float)( rawdata - (*ThermalTable)[cnt + 1][0] )/( (*ThermalTable)[cnt][0] - (*ThermalTable)[cnt + 1][0] );
            temp_result *= 5;
            temp_result += (*ThermalTable)[cnt + 1][1];
            return((int16_t)temp_result);
        }
    }
    
    return(0);
}

// ********************************************************************
// *                                                                                                                     
// * Function     : ProcessThermal()
// *
// * Purpose      : Calculate the average rawdata for 100KVA system
// *
// * Parms Passed : None
// *
// * Returns      : None
// *
// * Description  : This function is run every 0.4ms. It means rawdata
// *                of each thermal channel would be accumulated every
// *                2.8ms and average rawdata will be updated every
// *                2.8ms * 256 = 716.8ms
// ********************************************************************
#pragma CODE_SECTION( "ramfuncs" )
void ProcessThermal(void)
{
    static uint16_t cnt = 0;

    switch ( cnt )
    {
        case 0 :
        	PHASEB_Thermal0_sum += PHASEB_THERMAL_RAWDATA;
        	PHASEB_Thermal0_cnt++;
            if(PHASEB_Thermal0_cnt >= 256)
            {
            	PHASEB_Thermal0_avg = (uint16_t)(PHASEB_Thermal0_sum >> 8);
            	PHASEB_Thermal0_sum = 0;
            	PHASEB_Thermal0_cnt = 0;
            }
            cnt++;
            break;

        case 1 :
        	PHASEA_Thermal0_sum += PHASEA_THERMAL_RAWDATA;
        	PHASEA_Thermal0_cnt++;
            if(PHASEA_Thermal0_cnt >= 256)
            {
            	PHASEA_Thermal0_avg = (uint16_t)(PHASEA_Thermal0_sum >> 8);
            	PHASEA_Thermal0_sum = 0;
            	PHASEA_Thermal0_cnt = 0;
            }
            cnt++;
            break;

        case 2 :
            BAT_Thermal_sum += BAT_THERMAL_RAWDATA;
            BAT_Thermal_cnt++;
            if ( BAT_Thermal_cnt >= 256 )
            {
                BAT_Thermal_avg = uint16_t( BAT_Thermal_sum >> 8 );
                BAT_Thermal_sum = 0;
                BAT_Thermal_cnt = 0;
            }
            cnt++;
            break;

        case 3 :
        	PHASEC_Thermal_sum += PHASEC_THERMAL_RAWDATA;
        	PHASEC_Thermal_cnt++;
            if ( PHASEC_Thermal_cnt >= 256 )
            {
            	PHASEC_Thermal_avg = uint16_t( PHASEC_Thermal_sum >> 8 );
            	PHASEC_Thermal_sum = 0;
            	PHASEC_Thermal_cnt = 0;
            }
            cnt++;
            break;


            
        case 4:
            AMB_Thermal_sum += AMB_THERMAL_RAWDATA;
            AMB_Thermal_cnt++;
            if ( AMB_Thermal_cnt >= 256 )
            {
                AMB_Thermal_avg = AMB_Thermal_sum/256.0f;
                AMB_Thermal_sum = 0.0;
                AMB_Thermal_cnt = 0;
            }
            cnt++;
            break;


        case 5:
            SCR_Thermal_sum += SCR_THERMAL_RAWDATA;
            SCR_Thermal_cnt++;
            if ( SCR_Thermal_cnt >= 256 )
            {
                SCR_Thermal_avg = uint16_t( SCR_Thermal_sum >> 8 );
                SCR_Thermal_sum = 0;
                SCR_Thermal_cnt = 0;
            }
            cnt = 0;
            break;


        default :
            break;
    }
}

// ********************************************************************
// *                                                                                                                     
// * Function     : CalculateThermal()
// *
// * Purpose      : Update real temperature from average rawdata
// *
// * Parms Passed : None
// *
// * Returns      : None
// *
// * Description  : The function FindTemperature() is called to update
// *                real temperature from average rawdata. This function
// *                should run in 100ms TASK so that real temperature
// *                can be updated every 700ms.
// ********************************************************************
void CalculateThermal(void)
{
    static uint16_t cnt = 0;
    
    switch(cnt)
    {
        case 0 :
//	            PFC_Thermal_C = FindTemperature((uint16_t)((PFC_Thermal0_avg *10) / 16));
        	PHASEB_Thermal_C = FindTemperature((uint16_t)PHASEB_Thermal0_avg);
        	PHASEB_Thermal_Max = PHASEB_Thermal_C;
        	PHASEB_Thermal_Min = PHASEB_Thermal_C;
            cnt++;
            break;
        case 1 :
        	PHASEA_Thermal_C = FindTemperature(PHASEA_Thermal0_avg);
        	PHASEA_Thermal_Max = PHASEA_Thermal_C;
        	PHASEA_Thermal_Min = PHASEA_Thermal_C;
            cnt++;
            break;
        case 2 :
//				BAT_Thermal_C = FindTemperature( (uint16_t)((BAT_Thermal_avg *10) / 16)); //HW sample
			BAT_Thermal_C = FindTemperature( (uint16_t)BAT_Thermal_avg); //HW sample
			BAT_Thermal_Max = BAT_Thermal_C;
			BAT_Thermal_Min = BAT_Thermal_C;
            cnt++;
			break;

        case 3 :
        	PHASEC_Thermal_C = FindTemperature( PHASEC_Thermal_avg );
        	PHASEC_Thermal_Max = PHASEC_Thermal_C;
            cnt++;
            break;

        case 4:
            //LM335 10 mv/K, circuit gain = 2/3, 10*0.001*2/3*4096/3 = 9.1022222, meaning 1K.
            //AMB_Thermal_C  = 1/9.1022222 * AMB_Thermal_avg - 273.15;//degree			
            AMB_Thermal_K = AMB_Thermal_avg;
	        AMB_Thermal_C = AMB_Thermal_K - 273.15f;//degree
            ScreenMeters.AmbientTemperature = AMB_Thermal_C;
	        cnt++;
	        break;
        case 5:
	        SCR_Thermal_C = FindTemperature( (uint16_t)SCR_Thermal_avg);//degree
	        cnt = 0;
	        break;
        default :
            cnt = 0;
            break;
    }

}


// ********************************************************************
// *            End of Thermal.cpp   
// ********************************************************************
