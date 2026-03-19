#ifndef BATTERYCONVERTERCONTROL_H_
#define BATTERYCONVERTERCONTROL_H_
// ******************************************************************************************************
// *            BatteryConverterControl.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton  
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2008 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: BatteryConverterControl.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/8/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#include "Constants.h"
#include "algos.h"
#include "Eeprom_Map.h"
#include "Coefficients.h"

typedef struct
{
    uint16_t ChargerOn        : 1;
    uint16_t BoostOn          : 1;
    uint16_t ChargerOpenLoop  : 1;
    uint16_t BoostOpenLoop    : 1;
    
    uint16_t fixedDuty        : 1;
    uint16_t BoostPowerModeOn : 1;
    uint16_t ChargerPosOn     : 1; 
    uint16_t ChargerNegOn     : 1;

    uint16_t unused8_15       : 8;
    uint16_t word1placeholder;
    uint16_t word2placeholder;
    
    uint16_t /*unused */: 12;
    uint16_t SequenceNumber: 4;
} stBatteryConverterStatus;

typedef union
{
    stBatteryConverterStatus bit;
    uint16_t           all;
    uint16_t            words[sizeof(stBatteryConverterStatus)];
} uBatteryConverterStatus;    

class BatteryConverterControl
{
    public:
        BatteryConverterControl( void )
        {
        }
        virtual ~BatteryConverterControl()
        {
        }

    public:
        void Init( void );
        void BoostLegShare( void );
        void ChargerCurrControl( void );
        void RunBoost( void );
        void RunBoostPowerMode( void );
        void RunCharger( void );
        void SetChargeCurrentLimit( float currentLimit );
        void SetChargeVoltage( float voltage );
        void SetDCLinkVoltage( float voltage );
        void CalculateLimits( void );
        void BoostOn( void );
        void BoostPWMOn( void );
        void BoostPowerModeOn( void );
        void BoostOnFF( void );
        void BoostOnOpenLoop( void );
        void BoostOff( void );
		void BoostOffLegA( void );	//only off legA			
        void ChargerOn( void );
        void ChargerOnOpenLoop( void );
        void ChargerOff( void );
        void ChargerFixedDutyOn( void );
        void ChargerFixedDutyPosOn( void );
        void ChargerFixedDutyNegOn( void );
        void ChargerFixedDutyOff( void ); 
        void ChargerFixedDutyPosOff( void );
        void ChargerFixedDutyNegOff( void );
        void SetPowerModeTarget( float Target )
        {
            //range check the battery power target
            if( !( Target < 0.0 ) && ( Target < ( (float)OutputkWRating * 100.0 ) ) )
            {
                PowerTarget = Target * 0.5f;
            }
        }    
        float GetPowerModeTarget( void )
        {
            return PowerTarget;
        }   

        stFirstOrderIIRFP  GetBoostVGains( void )
        {
        	return BoostVGainsPos;
        }

        void SetBoostVGains( stFirstOrderIIRFP * coefficients )
        {
        	BoostVGainsPos = *coefficients;
        	BoostVSatFactor = 1.0 / ( BoostVGainsPos.B1 - BoostVGainsPos.A1 );
        	BoostVGainsNeg = BoostVGainsPos;
        }

        stFirstOrderIIRFP  GetBoostPowerModeGains( void )
        {
        	//return BoostPowerModeGains;
        	return BoostLegShareGainsPos;
        }

        void SetBoostPowerModeGains( stFirstOrderIIRFP * coefficients )
        {
        	//BoostPowerModeGains = *coefficients;
        	//BoostPowerModeSatFactor = 1.0 / ( BoostPowerModeGains.B1 - BoostPowerModeGains.A1 );
        	BoostLegShareGainsPos = *coefficients;
        	BoostLegShareSatFactor = 1.0 / ( BoostLegShareGainsPos.B1 - BoostLegShareGainsPos.A1 );
        	BoostLegShareGainsNeg = BoostLegShareGainsPos;
        }

        stFirstOrderIIRFP  GetChargeVGains( void )
        {
        	return ChargeVGainsPos;
        }

        void SetChargeVGains( stFirstOrderIIRFP * coefficients )
        {
        	ChargeVGainsPos = *coefficients;
        	ChargeVSatFactor = (float)1 / ( ChargeVGainsPos.B1 - ChargeVGainsPos.A1 );
            ChargeVGainsNeg = ChargeVGainsPos;
        }

        stFirstOrderIIRFP  GetOuterChargeIGains( void )
        {
        	return OuterChargeIGainsPos;
        }

        void SetOuterChargeIGains( stFirstOrderIIRFP * coefficients )
        {
        	OuterChargeIGainsPos = *coefficients;
        	OuterChargeISatFactor = 1 / ( OuterChargeIGainsPos.B1 - OuterChargeIGainsPos.A1 );
            OuterChargeIGainsNeg = OuterChargeIGainsPos;
        }

        void SetHWCurrentLimit( uint16_t currentLimit );
        
        const uBatteryConverterStatus& GetStatus( void )
        {
            return BatteryConverterStatus;
        }
		void BoostTargetNewCal(void);
		void BatConverter20msTask(void);        
    protected:
        void DetermineMaxChargeCurrent( void );

        uBatteryConverterStatus BatteryConverterStatus;
        float               VoltageNormFactor;
        float               ChargePWMPeriod;
        float               BoostPWMPeriod;
        uint16_t            BoostDownSampleCount;
//	        uint16_t            BoostPowerModeDownSampleCount;
//        uint16_t            ChargeDownSampleCount;  //L47SARAM improve

    public:        
        //boost control variables
        stFirstOrderIIRFP  BoostVGainsPos;
        stFirstOrderIIRFP  BoostVGainsNeg;
        float               BoostSWCurrentLimit;
        stFirstOrderIIRFP  BoostLegShareGainsPos;
        stFirstOrderIIRFP  BoostLegShareGainsNeg;
        float               BoostDutyLimit;
        float               DCLinkVoltageTarget;
        float               BoostVSatFactor;
        float               ChargeDuty;
        float               dutyPos_chg;
        float               dutyNeg_chg;
        float               IrefPos;
        float               IrefNeg;
    	float               MaxDutyChargePos;
    	float               MaxDutyChargeNeg;
        float               PowerTarget;

        float               BoostLegShareSatFactor;
        float               DCLinkVoltageTargetNewPos;
        float               DCLinkVoltageTargetNewNeg;

        //charge control variables
//	        stSecondOrderIIRFP  ChargeVGains;
		stFirstOrderIIRFP	ChargeVGainsPos;
		stFirstOrderIIRFP	ChargeVGainsNeg;
		stFirstOrderIIRFP	ChargeVGainsPos_Float;
		stFirstOrderIIRFP	ChargeVGainsNeg_Float;

		float               ChargerCurrentReference;
//	        stSecondOrderIIRFP  BoostPowerModeGains;
//			stFirstOrderIIRFP	BoostPowerModeGains;
//	        float               BoostPowerModeSatFactor;
        float               ChargeMaxCurrent;
        float               ChargeSWCurrentLimit;
        float               ChargeVSatFactor;
        float               ChargeVSatFactor_Float;
//	        stSecondOrderIIRFP  OuterChargeIGains;
		stFirstOrderIIRFP	OuterChargeIGainsPos;
		stFirstOrderIIRFP	OuterChargeIGainsNeg;
		float               OuterChargeISatFactor;
        float               ChargeIError;
		bool 				ChgFlagCVmode_pos;
		bool 				ChgFlagCVmode_neg;

        float               ChargeDutyLimit;
        float               ChargeVoltageTarget;
        float               MaxDutyPos;
        float               MaxDutyNeg;
        float               CurrMaxDutyPos;
        float               CurrMaxDutyNeg;
        float               OneOverDCLinkVoltPos;
        float               OneOverDCLinkVoltNeg;
        float               BoostDutyPos;
        float               BoostDutyNeg;
        float               BattPrechargeDuty;
        float               WorkingMaxChargeCurrent;
		//vbus_target loop
//		float 				KfwBatBoost;
//		float 				ErrVoltPos;
		float 				ErrVoltPos_1;
//		float 				ErrVoltNeg;
		float 				ErrVoltNeg_1;
		
        bool                BatteryCurrentCalEnable;
        uint16_t            BatteryCurrentPhase;
};

extern const stFirstOrderIIRFP BoostVGains_20KVA_HV;
extern const stFirstOrderIIRFP BoostPowerModeGains_20KVA_HV;
extern const stFirstOrderIIRFP ChargeVGains_20KVA_HV;
extern const stFirstOrderIIRFP OuterChargeIGains_20KVA_HV;

extern const stFirstOrderIIRFP BoostVGains_40KVA_HV;
extern const stFirstOrderIIRFP BoostPowerModeGains_40KVA_HV;
extern const stFirstOrderIIRFP ChargeVGains_40KVA_HV;
extern const stFirstOrderIIRFP OuterChargeIGains_40KVA_HV;

extern uint16_t BattHWCurrentLimit;

#endif /*BATTERYCONVERTERCONTROL_H_*/

// ******************************************************************************************************
// *            End of Inverter.h
// ******************************************************************************************************

