#ifndef COEFFICIENTS_H_
#define COEFFICIENTS_H_
// ********************************************************************************************************
// *            Coefficients.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: Coefficients.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Puck Liu
// *
// *    DATE: 02/09/2012
// *
// *    HISTORY: See CVS history
// *********************************************************************************************************

#include "Parameters.h"
#include "algos.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ********************************************************************************************************
// *            Structure definitions
// ********************************************************************************************************
/*
typedef struct
{
	void* variables_ptr;
	float DefaultValue_LV;
	float DefaultValue_HV;
}VARIABLES_ID;
*/
#define LoopCoeffiIndex20K	0
#define LoopCoeffiIndex40K	1


typedef struct
{
    const stSecondOrderIIRFP** coefficients_ptr;         // coefficients ptr
    const stSecondOrderIIRFP * coefficients_20KVA_HV;   
    const stSecondOrderIIRFP * coefficients_40KVA_HV; 
}COEFFICIENTS_ID;

typedef struct
{
    const stFirstOrderIIRFP** coefficients_ptr;         // coefficients ptr
    const stFirstOrderIIRFP * coefficients_20KVA_HV;
	const stFirstOrderIIRFP * coefficients_40KVA_HV;
}COEFFICIENTS_ID_FIRST;
 
// ********************************************************************************************************
// *            Global data
// ********************************************************************************************************
extern float capacitance;

// Battery coefficients
extern const stFirstOrderIIRFP * BoostVGains_ptr;
extern const stFirstOrderIIRFP * BoostPowerModeGains_ptr;
extern const stFirstOrderIIRFP * ChargeVGains_ptr;
extern const stFirstOrderIIRFP * OuterChargeIGains_ptr;

// Rectifier coefficients
extern const stFirstOrderIIRFP * RectifierVoltageLoopCoefficients_ptr; 
extern const stFirstOrderIIRFP * RectifierOffsetCoefficients_ptr;      
extern const stSecondOrderIIRFP * RectifierCurrentLoopCoefficients1_ptr;
extern const stSecondOrderIIRFP * RectifierCurrentLoopCoeffiNewRecTopo_ptr; 
extern const stSecondOrderIIRFP * RectifierVoltageFilterGains_ptr;     
extern const stFirstOrderIIRFP * RectifierAutoZeroFilterGains_ptr;  


// Inverter coefficients
extern const stSecondOrderIIRFP * InverterVoltageLoopCoefficients_ptr;   
extern const stSecondOrderIIRFP * InverterVoltageLoopCoefficients2_ptr;  
extern const stSecondOrderIIRFP * InverterCurrentLoopCoefficients_ptr;   
extern const stSecondOrderIIRFP * InverterCurrentLoopCoefficients2_ptr;  
extern const stFirstOrderIIRFP * InverterRMSLoopCoefficients_ptr;       
extern const stSecondOrderIIRFP * InverterDCCompensationCoefficients_ptr;
extern const stFirstOrderIIRFP * LoadShareDroopCoefficients_ptr;        
extern const stFirstOrderIIRFP * ECTCurrentLoopCoefficients_ptr;        
extern const stFirstOrderIIRFP * ECTPowerLoopCoefficients_ptr;          
extern const stFirstOrderIIRFP * CapCurrentEstimatorCoefficients_ptr;   
extern const stFirstOrderIIRFP * ReactiveLoadShareDroopCoefficients_ptr;
extern const stFirstOrderIIRFP * LoadShareDCFeedForwardCoefficients_ptr;
extern const stFirstOrderIIRFP * ReactivePowerDroopCoefficients_ptr;    
extern const stSecondOrderIIRFP * HighDCVPhaseDroopCoefficients_ptr;     
extern const stSecondOrderIIRFP * ReactiveCurrentFilterCoefficients_ptr; 

// Auto cal coefficients
extern const stSecondOrderIIRFP * LoadShareCalCoefficients_ptr;
extern const stSecondOrderIIRFP * VoltageBalanceCoefficients_ptr;

// inverter capacitance
extern const float capacitance_HV_20K_30K;

extern void InitParameters( void );

#ifdef __cplusplus
}
#endif

// ********************************************************************************************************
// *            END OF Coefficients.h
// ********************************************************************************************************
#endif /*COEFFICIENTS_H_*/
