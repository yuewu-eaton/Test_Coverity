// ********************************************************************
// *            Thermal.h
// ********************************************************************
// ********************************************************************
// * 
// * This information if proprietary to Eaton Corporation
// * 
// ********************************************************************
// *                                                                        
// *    Copyright (c) 2005 Eaton Corporation                       
// *                      ALL RIGHTS RESERVED                              
// *                                                                       
// ********************************************************************
// ********************************************************************
// *     FILE NAME:   Fan.h
// *                                                                      
// *     DESCRIPTION: 
// *                  
// *     ORIGINATOR:  Jun Zhang                                         
// *                                                                      
// *     DATE:        05/17/2010                                            
// *                                                                      
// *     HISTORY:                                                         
// ********************************************************************
#ifndef _THERMAL_H
#define _THERMAL_H



// ********************************************************************
// *            Constant declarations 
// ********************************************************************
#define PHASEB_THERMAL_RAWDATA   SlowAdcDataPtr->st.PhaseBTemperature   //Power board S phase temp
#define PHASEA_THERMAL_RAWDATA   SlowAdcDataPtr->st.PhaseATemperature   //Power board R phase temp
#define BAT_THERMAL_RAWDATA      SlowAdcDataPtr->st.BatteryTemperature  //Power board BAT temp
#define PHASEC_THERMAL_RAWDATA   SlowAdcDataPtr->st.PhaseCTemperature   //Power board T phase temp
#define INVCAP_THERMAL_RAWDATA   RawAdcDataPtr->st.InvCapTemperature
#define SCR_THERMAL_RAWDATA      RawAdcDataPtr->st.SCRTemperature  //STS temp
#define AMB_THERMAL_RAWDATA      RawAdcDataPtr->st.AMBTemperature
// ********************************************************************
// *            Global data declarations 
// ********************************************************************
extern int16_t PHASEB_Thermal_C;
extern int16_t PHASEA_Thermal_C;
extern int16_t PHASEC_Thermal_C;
extern int16_t BAT_Thermal_C;
extern float AMB_Thermal_C;
extern int16_t INVCAP_Thermal_C;
extern int16_t SCR_Thermal_C;


extern int16_t PHASEB_Thermal_Max;
extern int16_t PHASEB_Thermal_Min;
extern int16_t PHASEA_Thermal_Max;
extern int16_t PHASEA_Thermal_Min;
extern int16_t BAT_Thermal_Max;
extern int16_t BAT_Thermal_Min;

// ********************************************************************
// *            Function prototypes
// ********************************************************************
void CalculateThermal(void);
void ProcessThermal(void);

#endif
// ********************************************************************
// *            End of Thermal.h   
// ********************************************************************
