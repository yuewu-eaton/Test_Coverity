// ******************************************************************************************************
// *            Coefficients.c
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2005 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: Coefficients.c
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Puck Liu
// *
// *    DATE: 02/09/2012
// *
// *    HISTORY: See CVS history
// ******************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Coefficients.h"
#include "SystemTrap.h"
#include "F28335Port.h"
#include "BatteryConverterControl.h"
#include "BatteryStateControl.h"
#include "InverterControl.h"
#include "RectifierStateControl.h"
#include "ExtSignalPLL.h"
#include "SineReference.h"

// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
const float BatteryInductanceGain_HV_30K     = 0.0001;     //100uH
const float BatteryInductanceGain_HV_20K     = 0.000135;   //135uH


//2.second-order param matrix
const COEFFICIENTS_ID coefficients[] =
{
    //{coefficients_ptr              20KVA coefficients addr              40KVA coefficients addr     }
                                                                                                                                                                         
	// Battery coefficients   

    // Inverter coefficients
		//2.second-order
    { &InverterVoltageLoopCoefficients_ptr,     &InverterVoltageLoopCoefficients_20KVA_HV,    &InverterVoltageLoopCoefficients_40KVA_HV},
    { &InverterVoltageLoopCoefficients2_ptr,    &InverterVoltageLoopCoefficients2_20KVA_HV,   &InverterVoltageLoopCoefficients2_40KVA_HV},
    { &InverterCurrentLoopCoefficients_ptr,     &InverterCurrentLoopCoefficients_20KVA_HV,    &InverterCurrentLoopCoefficients_40KVA_HV },
    { &InverterCurrentLoopCoefficients2_ptr,    &InverterCurrentLoopCoefficients2_20KVA_HV,   &InverterCurrentLoopCoefficients2_40KVA_HV },
		//2.second-order
    { &InverterDCCompensationCoefficients_ptr,  &InverterDCCompensationCoefficients_20KVA_HV,   &InverterDCCompensationCoefficients_40KVA_HV },
		//2.second-order
    { &HighDCVPhaseDroopCoefficients_ptr,       &HighDCVPhaseDroopCoefficients_20KVA_HV,        &HighDCVPhaseDroopCoefficients_40KVA_HV},
    { &ReactiveCurrentFilterCoefficients_ptr,   &ReactiveCurrentFilterCoefficients_20KVA_HV,    &ReactiveCurrentFilterCoefficients_40KVA_HV},
        
    // Rectifier coefficients
		//2.second-order
    { &RectifierCurrentLoopCoefficients1_ptr,   &RectifierCurrentLoopCoefficients1_20KVA_HV,   &RectifierCurrentLoopCoefficients1_40KVA_HV},
    { &RectifierCurrentLoopCoeffiNewRecTopo_ptr,   &RectifierCurrentLoopCoeffiNewRec_20KVA_HV,   &RectifierCurrentLoopCoeffiNewRec_40KVA_HV},
    { &RectifierVoltageFilterGains_ptr,         &RectifierVoltageFilterGains_20KVA_HV,        &RectifierVoltageFilterGains_40KVA_HV},
		
};

#define COEFFICIENTS_TABLE_SIZE (sizeof (coefficients)  / sizeof ( COEFFICIENTS_ID ) )


//1.first-order param matrix
const COEFFICIENTS_ID_FIRST coefficients_firstorder[] =
{
	//{coefficients_ptr 						20KVA coefficients addr              40KVA coefficients addr }
																																										 
	// Battery coefficients   
		//1.first-order
	{ &BoostVGains_ptr, 						&BoostVGains_20KVA_HV,          &BoostVGains_40KVA_HV,},
	{ &BoostPowerModeGains_ptr, 				&BoostPowerModeGains_20KVA_HV,  &BoostPowerModeGains_40KVA_HV},
	{ &ChargeVGains_ptr,						&ChargeVGains_20KVA_HV,         &ChargeVGains_40KVA_HV },
	{ &OuterChargeIGains_ptr,					&OuterChargeIGains_20KVA_HV,    &OuterChargeIGains_40KVA_HV},

	// Inverter coefficients
		//1.first-order
	{ &InverterRMSLoopCoefficients_ptr, 		&InverterRMSLoopCoefficients_20KVA_HV,  &InverterRMSLoopCoefficients_40KVA_HV},
		//1.first-order
	{ &LoadShareDroopCoefficients_ptr,			&LoadShareDroopCoefficients_20KVA_HV,   &LoadShareDroopCoefficients_40KVA_HV},
	{ &ECTCurrentLoopCoefficients_ptr,			&ECTCurrentLoopCoefficients_20KVA_HV,   &ECTCurrentLoopCoefficients_40KVA_HV},
	{ &ECTPowerLoopCoefficients_ptr,			&ECTPowerLoopCoefficients_20KVA_HV,     &ECTPowerLoopCoefficients_40KVA_HV},
	{ &CapCurrentEstimatorCoefficients_ptr, 	&CapCurrentEstimatorCoefficients_20KVA_HV,     &CapCurrentEstimatorCoefficients_40KVA_HV},
	{ &ReactiveLoadShareDroopCoefficients_ptr,	&ReactiveLoadShareDroopCoefficients_20KVA_HV,  &ReactiveLoadShareDroopCoefficients_40KVA_HV},
	{ &LoadShareDCFeedForwardCoefficients_ptr,	&LoadShareDCFeedForwardCoefficients_20KVA_HV,  &LoadShareDCFeedForwardCoefficients_40KVA_HV},
	{ &ReactivePowerDroopCoefficients_ptr,		&ReactivePowerDroopCoefficients_20KVA_HV,      &ReactivePowerDroopCoefficients_40KVA_HV},

	// Rectifier coefficients
		//1.first-order    
	{ &RectifierVoltageLoopCoefficients_ptr,	&RectifierVoltageLoopCoefficients_20KVA_HV,   &RectifierVoltageLoopCoefficients_40KVA_HV},
	{ &RectifierOffsetCoefficients_ptr, 		&RectifierOffsetCoefficients_20KVA_HV,        &RectifierOffsetCoefficients_40KVA_HV},
		//1.first-order
	{ &RectifierAutoZeroFilterGains_ptr,		&RectifierAutoZeroFilterGains_20KVA_HV,       &RectifierAutoZeroFilterGains_40KVA_HV}
		
};

#define COEFFICIENTS_TABLE_SIZE_FIRST (sizeof (coefficients_firstorder)  / sizeof ( COEFFICIENTS_ID_FIRST) )

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
float capacitance;

// Battery coefficients
const stFirstOrderIIRFP * BoostVGains_ptr;
const stFirstOrderIIRFP * BoostPowerModeGains_ptr;
const stFirstOrderIIRFP * ChargeVGains_ptr;
//	const stSecondOrderIIRFP * OuterChargeIGains_ptr;
const stFirstOrderIIRFP * OuterChargeIGains_ptr;

// Rectifier coefficients
const stFirstOrderIIRFP * RectifierVoltageLoopCoefficients_ptr;
const stFirstOrderIIRFP * RectifierOffsetCoefficients_ptr;
const stSecondOrderIIRFP * RectifierCurrentLoopCoefficients1_ptr;
const stSecondOrderIIRFP * RectifierCurrentLoopCoeffiNewRecTopo_ptr;
const stSecondOrderIIRFP * RectifierVoltageFilterGains_ptr;
const stSecondOrderIIRFP * RectifierFrequencyFilterGains_ptr;
const stFirstOrderIIRFP * RectifierAutoZeroFilterGains_ptr;

// Inverter coefficients  
const stSecondOrderIIRFP * InverterVoltageLoopCoefficients_ptr;
const stSecondOrderIIRFP * InverterVoltageLoopCoefficients2_ptr;
const stSecondOrderIIRFP * InverterCurrentLoopCoefficients_ptr;
const stSecondOrderIIRFP * InverterCurrentLoopCoefficients2_ptr;
const stFirstOrderIIRFP * InverterRMSLoopCoefficients_ptr;
const stSecondOrderIIRFP * InverterDCCompensationCoefficients_ptr;
const stFirstOrderIIRFP * LoadShareDroopCoefficients_ptr;
const stFirstOrderIIRFP * ECTCurrentLoopCoefficients_ptr;
const stFirstOrderIIRFP * ECTPowerLoopCoefficients_ptr;
const stFirstOrderIIRFP * CapCurrentEstimatorCoefficients_ptr;
const stFirstOrderIIRFP * ReactiveLoadShareDroopCoefficients_ptr;
const stFirstOrderIIRFP * LoadShareDCFeedForwardCoefficients_ptr;
const stFirstOrderIIRFP * ReactivePowerDroopCoefficients_ptr;
const stSecondOrderIIRFP * HighDCVPhaseDroopCoefficients_ptr;
const stSecondOrderIIRFP * ReactiveCurrentFilterCoefficients_ptr;
// Auto cal coefficients
const stSecondOrderIIRFP * LoadShareCalCoefficients_ptr;
const stSecondOrderIIRFP * VoltageBalanceCoefficients_ptr;
extern uint16_t LoopCoeffiIndex;


// ****************************************************************************
// *
// *  Function      :  InitVariables_20K_30K_HV()
// *
// *  Purpose         :  To initialize variables for HV system
// *
// *  Parms Passed  :  none
// *
// *  Returns       :  none
// *
// *  Description:     To initialize variables in coefficients_HV[].
// *
// ****************************************************************************
void InitVariables_20K_30K_40K_HV( void )
{
    InverterTs = InverterTs_HV;
    PLL_Ts = PLL_Ts_HV;

    if ( UPMSystem == HV_20K )
    {
		capacitance = Capacitance_INV20K;
    }
    else
    {
		capacitance = Capacitance_INV40K;
    }
}

//Jacob/20130808/merge 120k,modify begin...
// ****************************************************************************
// *
// *  Function      :  InitCoefficients_20KVA_HV()
// *
// *  Purpose         :  To initialize coefficients for HV system
// *
// *  Parms Passed  :  none
// *
// *  Returns       :  none
// *
// *  Description:     To initialize HV coefficients.
// *
// ****************************************************************************
void InitCoefficients_20KVA_HV(void)
{
    // index for second order param init
    uint16_t count = 0;
    // check for valid index
    while ( count < COEFFICIENTS_TABLE_SIZE )
    {
        const COEFFICIENTS_ID* temp_coefficients = &coefficients[ count++ ];
        *temp_coefficients->coefficients_ptr = temp_coefficients->coefficients_20KVA_HV;
    }
	// index for first order param init
    count = 0;
    // check for valid index
    while ( count < COEFFICIENTS_TABLE_SIZE_FIRST)
    {
        const COEFFICIENTS_ID_FIRST* temp_coefficients_first = &coefficients_firstorder[ count++ ];
        *temp_coefficients_first->coefficients_ptr = temp_coefficients_first->coefficients_20KVA_HV;
    }
}

// ****************************************************************************
// *
// *  Function      :  InitCoefficients_40KVA_HV()
// *
// *  Purpose         :  To initialize coefficients for HV system
// *
// *  Parms Passed  :  none
// *
// *  Returns       :  none
// *
// *  Description:     To initialize HV coefficients.
// *
// ****************************************************************************
void InitCoefficients_40KVA_HV(void)
{
    // index for second order param init
    uint16_t count = 0;
    // check for valid index
    while ( count < COEFFICIENTS_TABLE_SIZE )
    {
        const COEFFICIENTS_ID* temp_coefficients = &coefficients[ count++ ];
        *temp_coefficients->coefficients_ptr = temp_coefficients->coefficients_40KVA_HV;
    }
	// index for first order param init
    count = 0;
    // check for valid index
    while ( count < COEFFICIENTS_TABLE_SIZE_FIRST)
    {
        const COEFFICIENTS_ID_FIRST* temp_coefficients_first = &coefficients_firstorder[ count++ ];
        *temp_coefficients_first->coefficients_ptr = temp_coefficients_first->coefficients_40KVA_HV;
    }
}

// ***********************************************************************
// *
// *    FUNCTION: InitFrequencyAndAngleStep() 
// *
// *    DESCRIPTION: initializes frequency and angle step data to known default 
// *
// *    ARGUMENTS: 
// *               
// *    RETURNS:
// *
// ***********************************************************************
void InitFrequencyAndAngleStep( void )
{
    Inverter.SineRef.InitFrequencyAndAngleStep(InverterTs);
    
    Rectifier.UtilityPLL.SineRef.InitFrequencyAndAngleStep(PLL_Ts);
//	    Rectifier.UtilityPLL.SineRef.InitFrequencyAndAngleStep(PLL_Ts*0.5);
    BaseSineRef.InitFrequencyAndAngleStep(PLL_Ts);
    BypassPLL.SineRef.InitFrequencyAndAngleStep(PLL_Ts);
   // ExtSyncSource.SineRef.InitFrequencyAndAngleStep(PLL_Ts);
    ExtSignalSync.SineRef.InitFrequencyAndAngleStep(PLL_Ts);
    OutputPLL.SineRef.InitFrequencyAndAngleStep(PLL_Ts);
}

// ***********************************************************************
// *
// *    FUNCTION: InitParameters()
// *
// *    DESCRIPTION: Initialize global variables and start PWM timers
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
//Jacob/20130808,merge 120k, modify begin...
void InitParameters( void )
{
    switch(UPMSystem)
    {

        case HV_20K:
            InitVariables_20K_30K_40K_HV();  // Init variables
            InitCoefficients_20KVA_HV();     // Init Control Coefficients
            LoopCoeffiIndex = LoopCoeffiIndex20K;
            break;

        case HV_30K:
        case HV_40K:
            InitVariables_20K_30K_40K_HV();  // Init variables
            InitCoefficients_40KVA_HV();     // Init Control Coefficients
            LoopCoeffiIndex = LoopCoeffiIndex40K;
            break;

        default:
            InitVariables_20K_30K_40K_HV();  // Init variables
            InitCoefficients_40KVA_HV();     // Init Control Coefficients
            LoopCoeffiIndex = LoopCoeffiIndex40K;
            break;
    }

    // Update PLL_Ts
    InitFrequencyAndAngleStep();
}
//Jacob/20130808,merge 120k, ...modify end
// ************************************************************************************
//  No more
// ************************************************************************************
