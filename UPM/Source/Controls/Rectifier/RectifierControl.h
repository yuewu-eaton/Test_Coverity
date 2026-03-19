// ******************************************************************************************************
// *            Rectifier.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: Rectifier.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 3/30/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#ifndef _RECTIFIERCONTROL_H
#define _RECTIFIERCONTROL_H

#include "Algos.h"
#include "Eeprom_Map.h"
#include "DQPhaseLockLoop.h"
#include "Adc.h"
#include "NB_Funcs.h"
#include "Coefficients.h"

typedef struct
{
    // LSW is status
    uint16_t RectifierOnNormal       : 1;
    uint16_t RectifierOnBattery      : 1;
    uint16_t RectifierPrecharging    : 1;
    uint16_t WalkIn                  : 1;

    uint16_t PowerShare              : 1;
    uint16_t BatteryNormalCycleTest  : 1;
    uint16_t GLUtilityNotPresent     : 1;    //Vayn/20153/23 add, Only for GL, used for choicing the activepower
    uint16_t GoToSuspendState        : 1;
    
    uint16_t BatteryECT              : 1;
    uint16_t UseFastOffsetController : 1;
    uint16_t UseResonantComp         : 1;
    uint16_t UseDynamicRCGains       : 1;
        
    uint16_t OptimizedPFmode         : 1;
    uint16_t SurgeProtectionOn       : 1;
    uint16_t SiteWiringLocked        : 1;
    uint16_t OptimizedPF_CapComp     : 1;

    // MSW is commands
    uint16_t StartupCommand          : 1;
    uint16_t OffCommand              : 1;
    uint16_t SurgeRPWMOff_History 	 : 1;
    uint16_t SurgeSTPWMOff_History	 : 1;

//  uint16_t BoostReuseRectST        : 1;
    uint16_t FixedDutyCycle          : 1;
    uint16_t ACPreChargeFail         : 1;
    uint16_t unused22_31             : 10;

    uint16_t word2placeholder;
    
    uint16_t Rsd_12                  : 12;
    uint16_t SequenceNumber          : 4;
} stRectifierStatus;

typedef union
{
    stRectifierStatus bit;
    uint16_t            words[ sizeof( stRectifierStatus ) ];
    uint32_t            all;
} uRectifierStatus;  

#define RECTIFIER_STATUS_WORD       0
#define RECTIFIER_COMMAND_WORD      1

#define DELAY_3S_BASE_20MS       uint16_t(150) //50*3
#define DELAY_5S_BASE_20MS       uint16_t(250) //50*5
#define DELAY_20S_BASE_20MS      uint16_t(1000) //50*20

extern const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_20KVA_HV;
extern const stFirstOrderIIRFP RectifierOffsetCoefficients_20KVA_HV;
extern const stSecondOrderIIRFP RectifierCurrentLoopCoefficients1_20KVA_HV;
extern const stSecondOrderIIRFP RectifierCurrentLoopCoeffiNewRec_20KVA_HV;
extern const stSecondOrderIIRFP RectifierVoltageFilterGains_20KVA_HV;
extern const stSecondOrderIIRFP RectifierFrequencyFilterGains_20KVA_HV;
extern const stFirstOrderIIRFP RectifierAutoZeroFilterGains_20KVA_HV;

extern const stFirstOrderIIRFP RectifierVoltageLoopCoefficients_40KVA_HV;
extern const stFirstOrderIIRFP RectifierOffsetCoefficients_40KVA_HV;
extern const stSecondOrderIIRFP RectifierCurrentLoopCoefficients1_40KVA_HV;
extern const stSecondOrderIIRFP RectifierCurrentLoopCoeffiNewRec_40KVA_HV;
extern const stSecondOrderIIRFP RectifierVoltageFilterGains_40KVA_HV;
extern const stSecondOrderIIRFP RectifierFrequencyFilterGains_40KVA_HV;
extern const stFirstOrderIIRFP RectifierAutoZeroFilterGains_40KVA_HV;

extern uint16_t RectHWCurrentLimit;
extern float MaxRMSInputCurrentEEValue;
extern float VoltageForwardGain;
extern uint16_t RectHWCurrentLimitRealTime;
extern uint16_t FlagHighBusP;
extern uint16_t FlagHighBusN;
extern float RectCurrentLoopGain;
extern float RectCurrentLoopGainReal;


class RectifierControl
{
    public:
        RectifierControl() : UtilityPLL( SLEW_RATE_LUDICROUS, &RawAdcDataPtr->st.InputVoltage, ( ( float(10.0) * PI ) / 180.0 ) ) //10 degree default
							, NeutralLoss( PASS_1, PASS_8 )
        {
        }
        virtual ~RectifierControl()
        {
        }

    public:
        void Init( void );
        void Run_ISR( void );
		void RecCtrlThreePhase( void );
        inline void RectiferResonantCompensateControl( void );
        void OnNormal( bool walkin = false );
		void RecOnOnePhase(uint16_t phase);		
        void OnBattery( void );
        void RectifierOff( void );
        void OuterVoltageControl( void );
        void SetHWCurrentLimit( int16_t currentLimit );
        void SetMaxRMSInputCurrent( int16_t RecRMSCurrentLimit );
        void StartFixedDuty( int16_t duty );
        void Rectifier20msTask( void );
		void DCLinkSafety( void );		
        void EEFunc_Rectifier( const EE_ID* ee, const uint16_t* data );
        float GetMaxRMSCurrent( void );
		float GetRectifierCurrentMargin( void );
        void UseResonantCompensate( uint16_t command );
        void UpdateMaxRectifierCurrent( uint16_t voltage );

        void SetMixedModeGain( int32_t gain );
        void SetMixedModeGainForOptimizeInputPF( uint16_t gain );  //Jacob/20130815/add
        void SetRectifierAngleOffset( uint16_t tempData );         //Jacob/20130815/add
        void SetRectCurrentloopGain( uint16_t gain );               //Wuyue/20250529/add
        
        const uRectifierStatus& GetStatus( void ) const
        {
            return RectifierStatus;
        }
        
        float GetDCLinkVoltageRef( void )
        {
            return DCLinkReference;           
        }
		
        float GetDCLinkVoltRefNorm( void )
        {
        }

        //EEP Value
        float GetDCLinkSet( void )
        {
            return RectifierDCLinkSet;
        }

        float GetDCLinkOffset( void )
        {
            //return OffsetCmd;
        	return CurrentCmdN;
        }

        float GetACCurrentCmd( void )
        {
            return CurrentCmdP;
        }

        stFirstOrderIIRFP  GetLinkVoltageTable( void )
        {
            return LinkVoltageTableP;
        }

        void SetLinkVoltageTable( stFirstOrderIIRFP * coefficients )
        {
            LinkVoltageTableP = *coefficients;
            LinkVoltageSaturationFactor = (float)1.0 / ( LinkVoltageTableP.B1 - LinkVoltageTableP.A1 );
            LinkVoltageTableN = LinkVoltageTableP;
        }

//        stFirstOrderIIRFP  GetLinkOffsetTable( void )
//        {
//            return LinkOffsetTable;
//        }
//
//        void SetLinkOffsetTable( stFirstOrderIIRFP * coefficients )
//        {
//            LinkOffsetTable = *coefficients;
//        }

        stSecondOrderIIRFP  GetPFCCurrentLoop( void )
        {
            return PFCpha2P;
        }

        void SetPFCCurrentLoop( stSecondOrderIIRFP * coefficients )
        {
            PFCpha2P = *coefficients;
            PFCphb2P = PFCpha2P;
            PFCphc2P = PFCpha2P;
            PFCpha2N = PFCpha2P;
            PFCphb2N = PFCpha2P;
            PFCphc2N = PFCpha2P;
            PFCISatFactor2.phA = 1.0f / ( ( PFCpha2P.B1 + PFCpha2P.B2 ) - ( PFCpha2P.A1 + PFCpha2P.A2 ) );
            PFCISatFactor2.phB = 1.0f / ( ( PFCphb2P.B1 + PFCphb2P.B2 ) - ( PFCphb2P.A1 + PFCphb2P.A2 ) );
            PFCISatFactor2.phC = 1.0f / ( ( PFCphc2P.B1 + PFCphc2P.B2 ) - ( PFCphc2P.A1 + PFCphc2P.A2 ) );
        }

        bool  IsNeutralFault( void )
        {
            return NeutralLoss.GetState();
        }

        void StartupRectifier( void )
        {
            RectifierStatus.bit.StartupCommand = 1;
        }

		void ClearACPreChargeFail( void )
        {
            RectifierStatus.bit.ACPreChargeFail = 0;
        }

        void ToBatteryECT( void )
        {
            RectifierStatus.bit.BatteryECT = 1;
        }
        
        void ClearBatteryECT( void )
        {
            RectifierStatus.bit.BatteryECT = 0;
        }
        
        inline void BldInpOnGenerator( bool active )
        {
            NB_DebounceAndQue( UPM_NB_UPS_ON_GENERATOR, active );
        }
		void SetRectifierFilterLoopVFil( uint16_t freq);
		void OptimizedPFmodeLoop(void);
            
    // needed by charger, not used internally
    public:
        DQPhaseLockLoop     UtilityPLL;
        float               ScaledIrefMax;
        float               ScaledIrefMax_Ori;
        uint16_t            FastOffsetCnt;
        uint16_t            SlowOffsetCnt;
        float               SineRefGain;
        float               RectifierAngleOffset;
        float               SineRefGain_OptimizeInputPF;
        float               UtilityGain_OptimizeInputPF;
        float               LinkVoltageSaturationFactor;
        float               UtilityPeak;
		float               ECTSineRefGain;
		float               NormalSineRefGain;
		uint16_t            SineRefForwardEnabled;
		uint16_t            ECTSineRefForwardEnabled;
		uint16_t            NormalSineRefForwardEnabled;
//			uint16_t			FlagHighBusP;
//			uint16_t			FlagHighBusN;
		float				CurrentCmdP;
		float				RectifierPowerShareBuffer;
		float 				IcmdPowershareP;
		float 				IcmdPowershareN;
		
    protected:
        inline void LimitRectifierDuty( float* duty )
        {
            if ( *duty > 1.0 )
            {
                *duty = 1.0;
            }
            else
            {
                if ( *duty < 0.0 )
                {
                    *duty = 0.0;
                }
            }
        }
		
        inline void DeadZoneCompenste(float InputControlCurrent,float CompensatePercent,float* duty )
        {
			if(InputControlCurrent > 5)
			{
				*duty -= CompensatePercent;
			}
			else if(InputControlCurrent < -5)
			{
				*duty += CompensatePercent;
			}
        }       
        void CalculateResonantPeakFilterGains(float frequency);
		void RefreshHWCurrLimit(void);

    protected:
        float               VoltageNormFactor;
        float               PWMPeriod;
        float               DCLinkReference;
		float               DCLinkVoltageHalf;
        float               DCLinkUnbalanceOVLimSetting;
     //   float               CurrentCmd;
//	        float               CurrentCmdP;
        float               CurrentCmdN;
        float               CurrentCmdP_1;
        float               CurrentCmdN_1;		
        float               CurrentCmdP_Delta;
        float               CurrentCmdN_Delta;
        float               CurrentCmd_A;
        float               CurrentCmd_B;
        float               CurrentCmd_C;
      //  float               OffsetCmd;
        float               outA;
        float               outB;
        float               outC; 
        float               UtilityGain;
        float               SineReferenceMagnitude; 
        uint16_t            ReadyForPI;
        uint16_t            ReadyForPIRef;
        uint16_t            ReadyForOffset;
        uint16_t            PLLDownSampleCount;

//        stFirstOrderIIRFP   LinkVoltageTable;
		stFirstOrderIIRFP	LinkVoltageTableP;
		stFirstOrderIIRFP	LinkVoltageTableN;
//        stFirstOrderIIRFP   LinkOffsetTable;
//        stSecondOrderIIRFP  FastLinkOffsetTable;
        float               VdcSafety;
        stThreePhase        RectifierSineRef;
        stThreePhase        RectifierUtilityReference;

        uRectifierStatus    RectifierStatus;

        // ee data
        float               RectifierDCLinkSet;
        float               RectifierGenInputCurrentMax;
        float               RectifierInputCurrentMax;
        float               RectifierWalkInRate;
        float               MaxRMSInputCurrent;
        float               MaxGeneratorRMSInputCurrent;
        uint16_t            RectifierWalkInDelay;
//	        float               RectifierPowerShareBuffer;
//        float               OffsetCmdNewTopoA;
//        float               OffsetCmdNewTopoB;
//        float               OffsetCmdNewTopoC;

//	        stSecondOrderIIRFP  PFCpha1;
//	        stSecondOrderIIRFP  PFCphb1;
//	        stSecondOrderIIRFP  PFCphc1;

        stSecondOrderIIRFP  PFCpha2P;
        stSecondOrderIIRFP  PFCphb2P;
        stSecondOrderIIRFP  PFCphc2P;
        stSecondOrderIIRFP  PFCpha2N;
		stSecondOrderIIRFP  PFCphb2N;
		stSecondOrderIIRFP  PFCphc2N;
        static const uint16_t nCompensators = 1;
        
        stSecondOrderIIRFP  ResonantCompensatorsPhA[nCompensators];
        stSecondOrderIIRFP  ResonantCompensatorsPhB[nCompensators];
        stSecondOrderIIRFP  ResonantCompensatorsPhC[nCompensators];
        float               ResonantCommandPhA[nCompensators];
        float               ResonantCommandPhB[nCompensators];
        float               ResonantCommandPhC[nCompensators];

        uint16_t            ResonantState;

        stSecondOrderIIRFP  FastFilteredFrequencyTable;
        float FastFilteredFrequency;
       // stSecondOrderIIRFP	FastFilteredDCVoltageTable;
//        stSecondOrderIIRFP	FastFilteredDCOffsetTable;
//		   	float			   	RailVoltagePositiveFastFilt;
//	       	float               RailVoltageNegativeFastFilt;
        stSecondOrderIIRFP  ResonantPeakFilterGains;
        CombinedVector      RectifierVoltage;
        CombinedVector      RectifierCurrent;
        CombinedVector      RectifierReference;
        CombinedVector      RectifierOut;
        float               CapCurrentFactor;
        stThreePhase        RectOut;
        stThreePhase        PFCISatFactor2;
		float DCLinkRampCurrent;
		float RectiferCurrentMargin;
		float UtilityNominalAvgSd;
		stFirstOrderIIRFP   RectifierFilterPowershrP;
		stFirstOrderIIRFP   RectifierFilterPowershrN;
		stSecondOrderIIRFP   RectifierFilterLoopVFilP_1th;
		stSecondOrderIIRFP   RectifierFilterLoopVFilN_1th;
		stSecondOrderIIRFP   RectifierFilterLoopVFilP_3th;
		stSecondOrderIIRFP   RectifierFilterLoopVFilN_3th;
		FilteredBit 		NeutralLoss;
		
};

void ee_calc_rectifier_limits( const EE_ID* ee, const uint16_t* data );
inline void SetRect3LvlPwm( float pw, float period, const uint16_t phase, uint16_t Polarity );

#endif
// ******************************************************************************************************
// *            End of Rectifier.h
// ******************************************************************************************************
