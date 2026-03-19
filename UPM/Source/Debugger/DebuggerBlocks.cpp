// ********************************************************************************************************
// *            DebuggerBlocks.c
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO Eaton Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: DebuggerBlocks.c
// *
// *    DESCRIPTION: Blocks array for the debugger.
// *
// *    ORIGINATOR: Jonathan Rodriguez
// *
// *    DATE: 1/8/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Debugger.h"

// needed for blocks
#include "F28335Port.h"
#include "Rtc.h"
#include "Adc.h"
#include "BypassInterface.h"
#include "Meters.h"
#include "IOexpansion.h"
#include "RectifierStateControl.h"
#include "InitPWM.h"
#include "Eeprom_Map.h"
#include "Version.h"
#include "Spi_Task.h"
#include "PeriodicTSKs.h" 
#include "Alarms.h"        
#include "DQPhaseLockLoop.h"
#include "Fan.h"
#include "ProcessAdc.h"
#include "InverterControl.h"
#include "MCUState.h"
#include "Thermal.h"      
#include "BatteryStateControl.h"
#include "ParallelCan.h"
#include "Abm.h"
#include "BTR.h"
#include "ACMeter.h"
#include "ACPowerMeter.h"
#include "VirtualNeutralMeter.h"
#include "I2C_Driver.h"
#include "ExtSignalPLL.h"
#include "InternalCan.h"
#include "AutoCal.h"
#include "FCTState.h"
#include "Alarms.h"
#include "Fan.h"

uint16_t DebugWord[12] = { 0, 0, 0, 0, 0, 0 };
float FuncRunTimeUs[6] = { 0, 0, 0, 0, 0, 0 };
float DebugFloat[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float DebugPara[50][4] = { 0.0 };

// stack utilization 
extern float PortStackPercent;
extern float SPIControlStackPercent;
extern float HistQueStackPercent;
extern float PeriodicTaskPercent;
extern float HWIStackPercent;
extern float PCanStackPercent;
extern float ICanRxStackPercent;
extern float HeapPercent;
extern float PercentIdle;

extern float AbsDCOVSet;

extern uint16_t ExtSyncState;

extern int16_t AnalogZeroRaw;
extern int16_t AnalogZero;

extern unParallelStatus  ParallelStatus;
extern unParallelCommand ParallelCommand;

extern uint16_t INV_Thermal1_avg;
extern float RectVoltageLoopGain;
extern float RectCurrentLoopGain;
extern uint16_t  THDiStartFlag;
extern float  KvfwRec; 
extern float VoltNormRecVa;
extern float VoltNormRecVb;
extern float VoltNormRecVc;
extern float ScaledIrefBattestMin;
extern float ScaledIrefBattestSet;
extern float CurrentCmdP_Record;
extern float OutputPowerBatBoost;
extern uint16_t FlagCurrentCmdP_Record;
extern float PercentLoadSum_Record;
extern float BatteryCurrentPos_Sum_SlowFilter;	//legA + legB
extern float BatteryCurrentNeg_Sum_SlowFilter;	//legA + legB
//bat test
extern float ScaledIrefBattestMin;
extern float ScaledIrefBattestSet;
extern float CurrentCmdP_Record;
extern float OutputPowerBatBoost;
extern uint16_t FlagCurrentCmdP_Record;
extern float PercentLoadSum_Record;
extern uint16_t FlagLoadOver50;
extern float DynamicLoadPercentRec;
extern uint16_t  FlagUnblanceLoad;

extern uint16_t PHASEB_Thermal0_avg;    // average thermal rawdata
extern uint16_t BAT_Thermal_avg;
extern uint16_t PHASEA_Thermal0_avg;
extern uint16_t PHASEC_Thermal_avg;
extern uint16_t INVCAP_Thermal_avg;
extern uint16_t SCR_Thermal_avg;
extern float AMB_Thermal_avg;
extern uint16_t FlagLowCurrChg_p;
extern uint16_t FlagLowCurrChg_n;
extern uint32_t OverloadCnt1;
extern uint16_t OverloadCnt2;
extern uint16_t OverloadCnt3;
extern uint16_t OverloadCnt4;
extern unsigned int     ECapISRCount1;
extern unsigned int     ECapDebug1;
extern float DynamicLoadPercent;


namespace {
    // Wrapper methods to fetch data from classes etc.
    // Direct pointers to non static member methods in blocks are not possible.
uint16_t GetMCUStateMachineState()   { return MCUStateMachine.GetState();              }
uint16_t GetMonitoredBypassState()   { return BypassState().GetMonitoredBypassState();   }
uint16_t GetRectifierState()         { return (uint16_t)Rectifier.GetState();          }
uint16_t GetRectStatusW0()           { return Rectifier.GetStatus().words[0];          }
uint16_t GetRectStatusW1()           { return Rectifier.GetStatus().words[1];          }
uint16_t GetInverterStatus()         { return (uint16_t)Inverter.GetStatus().all;      }
uint16_t GetBypassStatus()           { return BypassState().GetBypassStatus().bit.status;   }
uint16_t GetBatteryConverterStatus() { return BatteryConverter.GetStatus().all;        }
uint16_t GetMCUStatusW0()            { return MCUStateMachine.GetStatus().words[0]; }
uint16_t GetMCUStatusW1()            { return MCUStateMachine.GetStatus().words[1]; }
uint16_t GetMCUStatusW2()            { return MCUStateMachine.GetStatus().words[2]; }
uint16_t GetMCUStatusW3()            { return MCUStateMachine.GetStatus().words[3]; }
uint16_t GetAbmStatus()              { return Abm().GetStatus().words[0]; }
uint16_t GetAbmState()               { return Abm().GetState(); }
uint16_t GetBatteryStatus()          { return BatteryConverter.GetStatus().all; }
uint16_t PCanSysLoad()               { return ParallelCan.SystemLoad; }
uint16_t PCanSumBypass()             { return ParallelCan.SystemBypassAvailable; }
uint16_t PCanSumInverter()           { return ParallelCan.SystemInverterAvailable; }
uint16_t PCanSysBypass()             { return ParallelCan.PCan_CheckSystemBypassAvailable(); }
uint16_t PCanUpsBypass()             { return ParallelCan.PCan_CheckUPSBypassAvailable(); }
uint16_t PCanSysInverter()           { return ParallelCan.PCan_CheckSystemInverterAvailable(); }
uint16_t PCanUpsInverter()           { return ParallelCan.PCan_CheckUPSInverterAvailable(); }
uint16_t GetInverterRelayState()     { return MCUStateMachine.GetInverterRelayState(); }
uint16_t GetSystemVoltageLevel()     { return SystemType.bit.VoltageLevel;}
uint16_t GetSystemUPMKVALevel()      { return SystemType.bit.UPMKVALevel;}
uint16_t GetAutoCalState()           { return AutoCal.GetState(); }


float    GetRectifierVoltageRef()    { return Rectifier.GetDCLinkVoltageRef();         }
float    GetRectifierOffset()        { return Rectifier.GetDCLinkOffset();             }
float    GetInverterPhaseError()     { return Inverter.GetPhaseError();                }
float    GetInverterAngle()          { return Inverter.GetAngle();                     }
//	float    GetBatteryPower()           { return  ( BatteryCurrentPos.FastFiltered * BatteryVoltage.FastFiltered );   }
float    GetBatteryPower()           { return  ( BatteryCurrentPos_Sum_SlowFilter * BatteryVoltage.FastFiltered );   }
float    GetRectIMag()               { return Rectifier.GetACCurrentCmd();             }

float    GetBoostVGains_B0()    { return BatteryConverter.GetBoostVGains().B0;             }
float    GetBoostVGains_B1()    { return BatteryConverter.GetBoostVGains().B1;             }
float    GetBoostVGains_A1()    { return BatteryConverter.GetBoostVGains().A1;             }

float    GetBoostPowerModeGains_B0()    { return BatteryConverter.GetBoostPowerModeGains().B0;             }
float    GetBoostPowerModeGains_B1()    { return BatteryConverter.GetBoostPowerModeGains().B1;             }
float    GetBoostPowerModeGains_A1()    { return BatteryConverter.GetBoostPowerModeGains().A1;             }

float    GetChargeVGains_B0()    { return BatteryConverter.GetChargeVGains().B0;             }
float    GetChargeVGains_B1()    { return BatteryConverter.GetChargeVGains().B1;             }
float    GetChargeVGains_A1()    { return BatteryConverter.GetChargeVGains().A1;             }

float    GetOuterChargeIGains_B0()    { return BatteryConverter.GetOuterChargeIGains().B0;             }
float    GetOuterChargeIGains_B1()    { return BatteryConverter.GetOuterChargeIGains().B1;             }
float    GetOuterChargeIGains_A1()    { return BatteryConverter.GetOuterChargeIGains().A1;             }

float    GetLinkVoltageTable_B0()    { return Rectifier.GetLinkVoltageTable().B0;             }
float    GetLinkVoltageTable_B1()    { return Rectifier.GetLinkVoltageTable().B1;             }
float    GetLinkVoltageTable_A1()    { return Rectifier.GetLinkVoltageTable().A1;             }

//float    GetLinkOffsetTable_B0()     { return Rectifier.GetLinkOffsetTable().B0;             }
//float    GetLinkOffsetTable_B1()     { return Rectifier.GetLinkOffsetTable().B1;             }
//float    GetLinkOffsetTable_A1()     { return Rectifier.GetLinkOffsetTable().A1;             }

//float    GetFastLinkOffsetTable_B0()     { return Rectifier.GetFastLinkOffsetTable().B0;             }
//float    GetFastLinkOffsetTable_B1()     { return Rectifier.GetFastLinkOffsetTable().B1;             }
//float    GetFastLinkOffsetTable_A1()     { return Rectifier.GetFastLinkOffsetTable().A1;             }

float    GetPFCCurrentLoop_B0()     { return Rectifier.GetPFCCurrentLoop().B0;             }
float    GetPFCCurrentLoop_B1()     { return Rectifier.GetPFCCurrentLoop().B1;             }
float    GetPFCCurrentLoop_A1()     { return Rectifier.GetPFCCurrentLoop().A1;             }
float    GetPFCCurrentLoop_B2()     { return Rectifier.GetPFCCurrentLoop().B2;             }
float    GetPFCCurrentLoop_A2()     { return Rectifier.GetPFCCurrentLoop().A2;             }

float    GetInverterVRMSGains_B0()     { return Inverter.GetInverterVRMSGains().B0;             }
float    GetInverterVRMSGains_B1()     { return Inverter.GetInverterVRMSGains().B1;             }
float    GetInverterVRMSGains_A1()     { return Inverter.GetInverterVRMSGains().A1;             }

float    GetInverterVGains1_B0()     { return Inverter.GetInverterVGains1().B0;             }
float    GetInverterVGains1_B1()     { return Inverter.GetInverterVGains1().B1;             }
float    GetInverterVGains1_A1()     { return Inverter.GetInverterVGains1().A1;             }

float    GetInverterVGains2_B0()     { return Inverter.GetInverterVGains2().B0;             }
float    GetInverterVGains2_B1()     { return Inverter.GetInverterVGains2().B1;             }
float    GetInverterVGains2_A1()     { return Inverter.GetInverterVGains2().A1;             }
float    GetInverterVGains2_B2()     { return Inverter.GetInverterVGains2().B2;             }
float    GetInverterVGains2_A2()     { return Inverter.GetInverterVGains2().A2;             }

float    GetInverterIGains_B0()     { return Inverter.GetInverterIGains().B0;             }
float    GetInverterIGains_B1()     { return Inverter.GetInverterIGains().B1;             }
float    GetInverterIGains_A1()     { return Inverter.GetInverterIGains().A1;             }

float    GetInverterDCCompensator_B0()     { return Inverter.GetInverterDCCompensator().B0;             }
float    GetInverterDCCompensator_B1()     { return Inverter.GetInverterDCCompensator().B1;             }
float    GetInverterDCCompensator_A1()     { return Inverter.GetInverterDCCompensator().A1;             }

float    GetLoadShareDroopTable_B0()     { return Inverter.GetLoadShareDroopTable().B0;             }
float    GetLoadShareDroopTable_B1()     { return Inverter.GetLoadShareDroopTable().B1;             }
float    GetLoadShareDroopTable_A1()     { return Inverter.GetLoadShareDroopTable().A1;             }

float	 GetReactiveLoadShareDroopTable_B0()	 { return Inverter.GetReactiveLoadShareDroopTable().B0; 			}
float	 GetReactiveLoadShareDroopTable_B1()	 { return Inverter.GetReactiveLoadShareDroopTable().B1; 			}
float	 GetReactiveLoadShareDroopTable_A1()	 { return Inverter.GetReactiveLoadShareDroopTable().A1; 			}
	
float	 GetReactivePowerDroopTable_B0()	 { return Inverter.GetReactivePowerDroopTable().B0; 			}
float	 GetReactivePowerDroopTable_B1()	 { return Inverter.GetReactivePowerDroopTable().B1; 			}
float	 GetReactivePowerDroopTable_A1()	 { return Inverter.GetReactivePowerDroopTable().A1; 			}
		
float	 GetLoadShareDCFeedForwardTable_B0()	 { return Inverter.GetLoadShareDCFeedForwardTable().B0; 			}
float	 GetLoadShareDCFeedForwardTable_B1()	 { return Inverter.GetLoadShareDCFeedForwardTable().B1; 			}
float	 GetLoadShareDCFeedForwardTable_A1()	 { return Inverter.GetLoadShareDCFeedForwardTable().A1; 			}

float    GetECTPowerGains_B0()     { return Inverter.GetECTPowerGains().B0;             }
float    GetECTPowerGains_B1()     { return Inverter.GetECTPowerGains().B1;             }
float    GetECTPowerGains_A1()     { return Inverter.GetECTPowerGains().A1;             }

float    GetECTInverterIGains_B0()     { return Inverter.GetECTInverterIGains().B0;             }
float    GetECTInverterIGains_B1()     { return Inverter.GetECTInverterIGains().B1;             }
float    GetECTInverterIGains_A1()     { return Inverter.GetECTInverterIGains().A1;             }

float 	 GetInverterDCOA()           { return Inverter.GetDCOffset().phA; }
float 	 GetInverterDCOB()           { return Inverter.GetDCOffset().phB; }
float 	 GetInverterDCOC()           { return Inverter.GetDCOffset().phC; }

}

// a rather complicated cast to get rid of a compiler warning
#define FUNC_CAST(f)    (void*)((uint32_t)f)


// 11 characters are visible

const char* const BlocksStrings[] =
{

// ********************************************************************************************************
// *    MCU BLOCK STRINGS 0-35
// ********************************************************************************************************
    "Prt Stk %",                // block 0
    "SPI Stk %",                // block 1
    "HsQ Stk %",                // block 2

    "Prd Stk %",                // block 3
    "Gbl Stk %",                // block 4
    "Heap %",                   // block 5

    "Par Stk %",                // block 6
    "iCan Stk %",               // block 7
    "Status0",                  // block 8
    
    "Byp State",                // block 9
    "Rdy Cnt",                  // block 10
    "Status1",                  // block 11

    "State",                    // block 12
    "% Idle",                   // block 13
    "Status2",                  // block 14

    "5ms overrun",              // block 15  Some temporary debug words. Remove/move if required.
    "20msOverrun",              // block 16
    "Status3",                  // block 17

    "100msOoverrun",            // block 18
    "500msOverrun",             // block 19
    "Byp Stat",                 // block 20

    "1sOverrun",                // block 21
    "DebugWrd8",                // block 22
    "DebugWrd9",                // block 23

    "CtrlIntf_ID",           	// block 24
    "IOR State",                // block 25
    "CtrlBoad_ID ",				// block 26

    "PLD Ver",                  // block 27
    "PLD TRAP",                 // block 28
    "PLD BOOT",                 // block 29
    
    "EEMap Ver",                // block 30
    "EE Ver",                   // block 31
    "EE Fail",                  // block 32

    "FW Ver",                   // block 33
    "FW Bld",                   // block 34
    "Cfg Fail",                 // block 35

// ********************************************************************************************************
// *    INVERTER BLOCK STRINGS 36-71
// ********************************************************************************************************
    "V phA",                    // block 36
    "V phB",                    // block 37
    "V phC",                    // block 38

    "I phA",                    // block 39
    "I phB",                    // block 40
    "I phC",                    // block 41

    "Pwr A",                    // block 42
    "Pwr B",                    // block 43
    "Pwr C",                    // block 44

    "VA A",                     // block 45
    "VA B",                     // block 46
    "VA C",                     // block 47

    "Inv nom V",                // block 48
    "Load%",                    // block 49
    "Nom Frq",                  // block 50

    "Freq",                     // block 51
    "Sync St",                  // block 52
    "Phs Err",                  // block 53

    "OutkVA",                   // block 54
    "OutkW",                    // block 55
    "Ext Frq",                  // block 56

    "UPM0.Id",                  // block 57
    "UPM1.Id",                  // block 58
    "Ext St",                   // block 59

    "Mst RMSA",                 // block 60
    "Mst RMSB",                 // block 61
    "Mst RMSC",                 // block 62

    "Real I",                   // block 63
    "Inv Angle",                // block 64
    "Status",                   // block 65

/*
    "Iq A",                     // block 66
    "Iq B",                     // block 67
    "Iq C",                     // block 68
*/
    
    "Vmag.A",                   // block 66
    "VNorm",                    // block 67
    "RMSVNorm",                 // block 68
    
    "PhA.T.Max",                // block 69,
    "InvVoltCorr",                 // block 70,
    "PhA.T.Min",                // block 71,

// ********************************************************************************************************
// *    RECTIFIER BLOCK STRINGS 72-107
// ********************************************************************************************************
    "InputV A",                 // block 72
    "InputV B",                 // block 73
    "InputV C",                 // block 74

    "InputI A",                 // block 75
    "InputI B",                 // block 76
    "InputI C",                 // block 77
    
    "State",                    // block 78
    "Stat0",                    // block 79
    "Stat1",                    // block 80

    "Rect F",                   // block 81
    "IrefMax",                  // block 82
    "SinRefGain",               // block 83
    
    "Sd",                       // block 84
    "Sq",                       // block 85
    "S0",                       // block 86
    
    "SqNorm",                   // block 87
    "SqNorm",                   // block 88
    "S0Norm",                   // block 89

    "FanSpeed",                 // block 90
    "FanLoad",                  // block 91
    "ImagBuffer",               // block 92

    "DC Link",                  // block 93
    "DC+ Raw",                  // block 94
    "DC- Raw",                  // block 95

    "FastCnt",                  // block 96
    "SlowCnt",                  // block 97
    "Link Set",                 // block 98

    "Imag",                     // block 99
    "IOffset",                  // block 100
    "AutoCali",                 // block 101

    "DCOV Lim",                 // block 102
    "DC+ slow",                 // block 103
    "DC- slow",                 // block 104

    "PhB.T.Max",                // block 105
    "RecMaxCL",                 // block 106
    "PhB.T.Min",                // block 107

// ********************************************************************************************************
// *    SYSTEM BLOCK STRINGS 108-143
// ********************************************************************************************************
    "DSPOutALo",                // block 108
    "DSPOutAHi",                // block 109
    "DSPOutBLo",                // block 110

    "DSPOutBHi",                // block 111
    "DSPOutCLo",                // block 112
    "DSPOutChi",                // block 113

    "DSPInALo",                 // block 114
    "DSPInAHi",                 // block 115
    "DSPInBLo",                 // block 116

    "DSPInBHi",                 // block 117
    "DSPInCLo",                 // block 118
    "DSPInCHi",                 // block 119

    "RTC ms",                   // block 120
    "RTC min",                  // block 121
    "RTC year",                 // block 122

    "IO Read",                  // block 123
    "Fan Status",               // block 124
    "Sum Load %",               // block 125
    
    "phA Load%",                // block 126
    "phB Load%",                // block 127
    "phC Load%",                // block 128

    "phA Watt",                 // block 129
    "phB Watt",                 // block 130
    "phC Watt",                 // block 131

    "phA VA",                   // block 132
    "phB VA",                   // block 133
    "phC VA",                   // block 134

    "ChasVolt",                 // block 135
    "InputVn",               	// block 136
    "BuildInput",               // block 137

    "PhB.T",                   // block 138
    "Batt.T ",                  // block 139
    "SCR.T",                 // block 140

    "PhA.T",                    // block 141
    "PhC.T ",                   // block 142
    "AMB.T",                    // block 143

// ********************************************************************************************************
// *    CHARGER BLOCK STRINGS 144-179
// ********************************************************************************************************
    "Batt V",                   // block 144
    "Batt I",                   // block 145
    "Batt V1",                  // block 146

    "State",                    // block 147
    "PhState",                  // block 148
    "Status",                   // block 149

    "ChgVtarg",                 // block 150
    "Chg I Lim",                // block 151
    "DC Link Ref",              // block 152
                                
    "ChgdutyLim",               // block 153
    "Maxduty",                  // block 154
    "LinkUVTrig",               // block 155

    "BatLegA_Ip",               // block 156
    "Batt % rem",               // block 157
    "Bst Dty",                  // block 158

    "BatLegB_Ip",               // block 159
    "Cvtr St",                  // block 160
    "Charge D",                 // block 161

    "AbmState",                 // block 162
    "AbmPState",                // block 163
    "ABM bits",                 // block 164

    "BattV2",                   // block 165
    "BattV1",                 	// block 166
    "BattIp_Sum",               // block 167

    "BTR",                      // block 168
    "SOC",                      // block 169
    "BattPower",                // block 170

    "ChargeV %",                // block 171
    "ChargeI %",                // block 172
    "ChargeT %",                // block 173
                                
    "FCBTR",                    // block 174
    "NumOfSDT",                 // bolck 175
    "NumOfLDT",                 // block 176
    
    // 3 BTR debugger blocks
    "UPSLoad",                  // block 177
    "BatEff",                   // block 178
    "BatLoad",                  // block 179
 

// ********************************************************************************************************
// *    Meters block screen 1 180-215
// ********************************************************************************************************
    // Do not modify the meter debugger blocks unless you are sure what you're
    // doing!! These blocks are read by the CSB and if addresses are changed etc
    // it will affect the LCD code. Remember to also update the documentation 
    // and inform CSB designers of the changes in case something needs to be 
    // modified.
    "Util V A",                 // block 180
    "Util V B",                 // block 181
    "Util V C",                 // block 182

    "Inv V A",                  // block 183
    "Inv V B",                  // block 184
    "Inv V C",                  // block 185

    "Out V A",                  // block 186
    "Out V B",                  // block 187
    "Out V C",                  // block 188

    "Byp V A",                  // block 189
    "Byp V B",                  // block 190
    "Byp V C",                  // block 191

    "Util I A",                 // block 192
    "Util I B",                 // block 193
    "Util I C",                 // block 194

    "Inv I A",                  // block 195
    "Inv I B",                  // block 196
    "Inv I C",                  // block 197

    "Byp I A",                  // block 198
    "Byp I B",                  // block 199
    "Byp I C",                  // block 200

    "DC Link+",                 // block 201
    "DC Link-",                 // block 202
    "Batt V",                   // block 203

    "Batt I",                   // block 204
    "Batt V+",                  // block 205
    "Batt V-",                  // block 206

    "Byp Frq",                  // block 207
    "Out Frq",                  // block 208
    "In Frq",                   // block 209

    "Byp W",                    // block 210
    "Out W",                    // block 211
    "In W",                     // block 212

    "Byp2 A",                   // block 213
    "Byp2 B" ,                  // block 214
    "Byp2 C",                   // block 215
    
// ********************************************************************************************************
// *    Meters block screen 2 216-251
// ********************************************************************************************************
    "Out I A",                  // block 216
    "Out I B",                  // block 217
    "Out I C",                  // block 218

    "In V LL A",                // block 219
    "In V LL B",                // block 220
    "In V LL C",                // block 221

    "Inv V LL A",               // block 222
    "Inv V LL B",               // block 223
    "Inv V LL C",               // block 224

    "Out V LL A",               // block 225
    "Out V LL B",               // block 226
    "Out V LL C",               // block 227

    "Byp V LL A",               // block 228
    "Byp V LL B",               // block 229
    "Byp V LL C",               // block 230

    "Inp VA",                   // block 231
    "Out VA",                   // block 232
    "Byp VA",                   // block 233

    "Inp IpA",                  // block 234
    "Inp IpB",                  // block 235
    "Inp IpC",                  // block 236

    "Batt I+",                  // block 237
    "Batt I-",                  // block 238
    "Batt V",                   // block 239

    "+1.5v",                    // block 240, 
    "+5.0v",                    // block 241, 
    "+15.0v",                   // block 242, 

    "IOX Val",                  // block 243, for IO expander TPT29555
    "IOX NACK",                 // block 244
    "IOX ESCD",                 // block 245

    "+24.0v",                   // block 246, 
	"InvVCalB",                 // block 247
	"InvVCalC",                 // block 248

    "Out_I_A",                  // block 249
    "Out_I_B",                  // block 250
    "Out_I_C",                  // block 251
    
// ********************************************************************************************************
// *    PARALLEL BLOCK STRINGS 252-287
// ********************************************************************************************************
    "Sys Bypass",               // block 252
    "Ups Bypass",               // block 253
    "PCTxCnt",                  // block 254

    "Sys Invrt",                // block 255
    "Ups Invrt",                // block 256
    "PCRxCnt",                  // block 257

    "FCTOn",                    // block 258
    "FCTResult",                // block 259
    "FCTState",                 // block 260

    "FCTTime",                  // block 261
    "FCTError",                 // block 262
    "FCTStatus",                // block 263

    "My UPS#",                  // block 264
    "My UPM#",                  // block 265
    "UPMs OL",                  // block 266

    "GblAnd.MCU0",              // block 267
    "Sum Bypass",               // block 268
    "Sum Invrtr",               // block 269

	"CurUpm[1]",                // block 270
    "AllUpm[1]",                // block 271
    "SystemLoad",               // block 272

    "CurUps",                   // block 273
    "AllUps",                   // block 274
    "Total UPS",                // block 275
                                                
    "PCan TxOVF",               // block 276
    "PCan RxOVF",               // block 277
    "PCan MBOVF",               // block 278
                                            
    "PCan:STS",                 // block 279
    "PCan:CMD",                 // block 280
    "ICan:CMD",                 // block 281
                                            
    "UPM[0].CNT",               // block 282
    "UPM[1].CNT",               // block 283
    "PCErrCnt",                 // block 284

    "ECAP1",                    // block 285
    "ECAPdbg1",                 // block 286
    "PCQueMax",                 // block 287

// ********************************************************************************************************
// *    BLOCK G1 288-323
// ********************************************************************************************************
    "Unused",                   // block 288
    "SystemChanged",            // block 289
    "UPMSystem",                // block 290

    "PreUPMSystem",             // check ECT power setting
    "OutputKVARating",          // check ECT PF angle
    "EEP System",               // block 293

    "EEP voltage",              // check ECT line mode time setting (min)
    "EEP KVA",                  // check ECT battery mode time setting (min)
    "PWMFrequency",             // block 296

    "VRMSB0 10",                // block 297
    "VRMS B1",                  // block 298
    "VRMS A1",                  // block 299

    "InvV1B0 11",               // block 300
    "InvV1 B1",                 // block 301
    "InvV1 A1",                 // block 302

    "InvV2B0 12",               // block 303
    "InvV2 B1",                 // block 304
    "InvV2 B2",                 // block 305

    "Inv V2 A1",                // block 306
    "Inv V2 A2",                // block 307
    "block 308",                // block 308

    "InvIB0 13",                // block 309
    "InvI B1",                  // block 310
    "InvI A1",                  // block 311

    "InvDCB0 14",               // block 312
    "InvDC B1",                 // block 313
    "InvDC A1",                 // block 314

    "LoadB0 15",                // block 315
    "Load B1",                  // block 316
    "Load A1",                  // block 317

    "ECTPB0 16",                // block 318
    "ECTP B1",                  // block 319
    "ECTP A1",                  // block 320

    "ECTIB0 17",                // block 321
    "ECTIB1",                   // block 322
    "ECTI A1",                  // block 323

// ********************************************************************************************************
// *   G2 324-359
// ********************************************************************************************************
    "BoostVB020",               // block 324
    "BoostV B1",                // block 325
    "BoostV A1",                // block 326

    "BoostPB021",               // block 327
    "BoostP B1",                // block 328
    "BoostP A1",                // block 329

    "ChargeVB022",              // block 330
    "ChargeV B1",               // block 331
    "ChargeV A1",               // block 332

    "ChargeIB023",              // block 333
    "ChargeI B1",               // block 334
    "ChargeI A1",               // block 335

    "LinkVB0 30",               // block 336
    "LinkV B1",                 // block 337
    "LinkV A1",                 // block 338

    "DebugF_0",                 // block 339
    "DebugF_1",                 // block 340
    "DebugF_2",                 // block 341
		
    "DebugF_3",                 // block 342
    "DebugF_4",                 // block 343
    "DebugF_5",                 // block 344

    "RECIB0 33",                // block 345
    "REC I B1",                 // block 346
    "REC I B2",                 // block 347

    "REC I A1",                 // block 348
    "REC I A2",                 // block 349
    "Rsd",                      // block 350

    "RecCurLtRT",               // block 351
    "InvCurLtRT",               // block 352
    "ESS time",                 // block 353

    "VNormRecA",                // block 354
    "VNormRecB",                // block 355
    "VNormRecB",                // block 356

    "FlagTHDi",                 // block 357
    "FlagUnblLoa",              // block 358
    "Kvfw_Rec",                 // block 359

// ********************************************************************************************************
// *   G3 360-395
// ********************************************************************************************************
/*block 360 0*/ "FANS A",
/*block 361 1*/ "FANS B",
/*block 362 2*/ "FANS C",

/*block 363 0*/ "FANF A",
/*block 364 1*/ "FANF B",
/*block 365 2*/ "FANF C",

/*block 366 0*/ "STSTestSp",
/*block 367 1*/ "STSTestEn",
/*block 368 2*/ "STSDutyPLD",

/*block 369 0*/ "UPMTestSp",
/*block 370 1*/ "UPMTestEn",
/*block 371 2*/ "UPMDutyPLD",

/*block 372 0*/ "Thm_PhaseB_AD",
/*block 373 1*/ "Thm_Bat_AD",
/*block 374 2*/ "Thm_PhaseA_AD",

/*block 375 0*/ "Thm_PhaseC_AD",
/*block 376 1*/ "Thm_CapInvAD",
/*block 377 2*/ "Thm_SCR_AD",

/*block 378 0*/ "Thm_PhaseB_C",
/*block 379 1*/ "Thm_Bat_C",
/*block 380 2*/ "Thm_PhaseA_C",

/*block 381 0*/ "Thm_PhaseC_C",
/*block 382 1*/ "Thm_CapInv_C",
/*block 383 2*/ "Thm_SCR_C",

/*block 384 0*/ "Thm_Amb_AD",
/*block 385 1*/ "Thm_Amb_C",
/*block 386 2*/ "InputN_V ",

/*block 387 0*/ "Po_W",
/*block 388 1*/ "So_VA",
/*block 389 2*/ "PFo_",

/*block 390 0*/ " ",
/*block 391 1*/ " ",
/*block 392 2*/ " ",

/*block 393 0*/ " ",
/*block 394 1*/ " ",
/*block 395 2*/ " ",

// ********************************************************************************************************
// *   G4 396-431
// ********************************************************************************************************
/*block 396 0*/ "BatLegA_Ip",
/*block 397 1*/ "BatLegA_In",
/*block 398 2*/ "BatSum_Ip",

/*block 399 0*/ "BatLegB_Ip",
/*block 400 1*/ "BatLegB_In",
/*block 401 2*/ "BatSum_In",

/*block 402 0*/ "BatV",
/*block 403 1*/ "BatChgVp",
/*block 404 2*/ "BatChgVn",

/*block 405 1*/ "BatVp_AD",
/*block 406 2*/ "BatVn_AD",
/*block 405 0*/ "Lvl1_Cnt1",

/*block 406 1*/ "PFin",
/*block 407 2*/ "PFout",
/*block 408 0*/ "PFbyp",
	
/*block 411 0*/ "Iq_ins_B0",
/*block 412 1*/ "Iq_ins_B1",
/*block 413 2*/ "Iq_ins_A1",
	
/*block 414 0*/ "Dr_Vdcff_B0",
/*block 415 1*/ "Dr_Vdcff_B1",
/*block 416 2*/ "Dr_Vdcff_A1",

/*block 417 0*/ "FlgCurCmd_Rd",
/*block 418 1*/ "FlgLoadOv50",
/*block 419 0*/ "Iq_fil_B0",

/*block 420 1*/ "P_all",
/*block 421 2*/ "S_all",
/*block 422 2*/ "None",

/*block 423 0*/ "Load_P%",
/*block 424 1*/ "Load_S%",
/*block 425 2*/ "Load_all%",

/*block 426 0*/ "DebugWd_0",
/*block 427 1*/ "DebugWd_1",
/*block 428 2*/ "DebugWd_2",

/*block 429 0*/ "DebugWd_3",
/*block 430 1*/ "DebugWd_4",
/*block 431 2*/ "DebugWd_5",

    
};   

// ********************************************************************************************************
// *    BLOCK ARRAY
// ********************************************************************************************************

const stBlock BlockArray[] = {

// ********************************************************************************************************
// *    MCU BLOCKS 0-35
// ********************************************************************************************************
    {DEC4, MemTypeFLOAT, &PortStackPercent,                         },              // block 0
    {DEC4, MemTypeFLOAT, &SPIControlStackPercent,                   },              // block 1
    {DEC4, MemTypeFLOAT, &HistQueStackPercent,                      },              // block 2         
    
    {DEC4, MemTypeFLOAT, &PeriodicTaskPercent,                      },              // block 3
    {DEC4, MemTypeFLOAT, &HWIStackPercent,                          },              // block 4
    {DEC4, MemTypeFLOAT, &HeapPercent                               },              // block 5

    {DEC4, MemTypeFLOAT, &PCanStackPercent,                         },              // block 6
    {DEC4, MemTypeFLOAT, &ICanRxStackPercent,                       },              // block 7
    {HEX,  FuncTypeUINT, FUNC_CAST(GetMCUStatusW0),                 },              // block 8
    
    {DEC0, FuncTypeUINT, FUNC_CAST(GetMonitoredBypassState),        },              // block 9
    {DEC0, FuncTypeUINT, FUNC_CAST(GetRectStatusW0),                },              // block 10
    {HEX,  FuncTypeUINT, FUNC_CAST(GetMCUStatusW1),                 },              // block 11

    {DEC0, FuncTypeUINT, FUNC_CAST(GetMCUStateMachineState),        },              // block 12
    {DEC4, MemTypeFLOAT, &PercentIdle,                              },              // block 13
    {HEX,  FuncTypeUINT, FUNC_CAST(GetMCUStatusW2),                 },              // block 14

    {DEC0, MemTypeUINT, &Period5msOverruns,                         },              // block 15  Some temporary debug words. Remove/move if required.
    {DEC0, MemTypeUINT, &Period20msOverruns,                        },              // block 16
    {HEX,  FuncTypeUINT, FUNC_CAST(GetMCUStatusW3),                 },              // block 17

    {DEC0, MemTypeUINT, &Period100msOverruns,                       },              // block 18
    {DEC0, MemTypeUINT, &Period500msOverruns,                       },              // block 19
    {HEX,  FuncTypeUINT, FUNC_CAST(GetBypassStatus),                },              // block 20

    {DEC0, MemTypeUINT, &Period1sOverruns,                          },              // block 21
    {DEC0, MemTypeUINT, &DebugWord[8],                              },              // block 22
    {DEC0, MemTypeUINT, &DebugWord[9],                              },              // block 23

    {DEC0, MemTypeUINT, &InterfaceBoardRevID,                       },              // block 24
    {DEC0, FuncTypeUINT, FUNC_CAST(GetInverterRelayState),          },              // block 25
    {DEC0, MemTypeUINT, &ControlBoardRevID,                         },              // block 26

    {DEC0, MemTypeUINT, (uint16_t*)&PLDVersionReg.Version,          },              // block 27
    {HEX,  MemTypeUINT, (uint16_t*)&PLDTrapReg,                     },              // block 28
    {DEC0, MemTypeUINT, (uint16_t*)&PLDTrapBoot,                    },              // block 29

    {DEC0, MemTypeUINT, &EEVersion,                                 },              // block 30
    {DEC0, MemTypeUINT, &EERevision,                                },              // block 31
    {HEX,  MemTypeUINT, &EEStatusBits.w[ EE_STATUS_CHECKSUM_WORD ], },              // block 32

    {HEX,  MemTypeUINT, &FirmwareVersion,                           },              // block 33
    {DEC0, MemTypeUINT, &FirmwareBuildNum,                          },              // block 34
    {HEX,  MemTypeUINT, &EEStatusBits.w[ EE_STATUS_CONFIG_WORD ],   },              // block 35

// ********************************************************************************************************
// *    INVERTER BLOCKS 36-71
// ********************************************************************************************************
    {DEC1, MemTypeFLOAT, &InverterVoltageRMS.RawRMS.phA, },                         // block 36
    {DEC1, MemTypeFLOAT, &InverterVoltageRMS.RawRMS.phB, },                         // block 37
    {DEC1, MemTypeFLOAT, &InverterVoltageRMS.RawRMS.phC, },                         // block 38
    
    {DEC2, MemTypeFLOAT, &InverterCurrentRMS.RawRMS.phA, },                         // block 39
    {DEC2, MemTypeFLOAT, &InverterCurrentRMS.RawRMS.phB, },                         // block 40
    {DEC2, MemTypeFLOAT, &InverterCurrentRMS.RawRMS.phC, },                         // block 41

    {DEC0, MemTypeFLOAT, &InverterPower.ActivePowerFiltered.phA, },                 // block 42
    {DEC0, MemTypeFLOAT, &InverterPower.ActivePowerFiltered.phB, },                 // block 43
    {DEC0, MemTypeFLOAT, &InverterPower.ActivePowerFiltered.phC, },                 // block 44

    {DEC0, MemTypeFLOAT, &ScreenMeters.OutputVoltAmperes.phA, },                    // block 45
    {DEC0, MemTypeFLOAT, &ScreenMeters.OutputVoltAmperes.phB, },                    // block 46
    {DEC0, MemTypeFLOAT, &ScreenMeters.OutputVoltAmperes.phC, },                    // block 47

    {DEC1, MemTypeUINT,  &OutNomVolts, },                                           // block 48
    {DEC1, MemTypeFLOAT, &ScreenMeters.PercentLoad.sum, },                          // block 49
    {DEC0, MemTypeUINT,  &OutNomFreq, },                                            // block 50

    {DEC2, MemTypeFLOAT, &ScreenMeters.InverterFrequency, },                        // block 51
    {DEC0, MemTypeUINT,  &Sync_State, },                                            // block 52
    {DEC3, FuncTypeFLOAT, FUNC_CAST(GetInverterPhaseError) , },                     // block 53

    {DEC0, MemTypeUINT,  &OutputkVARating, },                                       // block 54
    {DEC0, MemTypeUINT,  &OutputkWRating, },                                        // block 55
    {DEC2, MemTypeFLOAT, &ExtSignalSync.SineRef.Frequency, },                       // block 56

    {DEC0, MemTypeINT,   &ParallelCan.CalData[0][0].id, },                          // block 57
    {DEC0, MemTypeINT,   &ParallelCan.CalData[0][1].id, },                          // block 58
    {DEC0, MemTypeUINT,  &ExtSyncState, },                                          // block 59

    {DEC3, MemTypeFLOAT, &Inverter.MasterRMSData.phA, },                            // block 60
    {DEC3, MemTypeFLOAT, &Inverter.MasterRMSData.phB, },                            // block 61
    {DEC3, MemTypeFLOAT, &Inverter.MasterRMSData.phC, },                            // block 62

    {DEC4, MemTypeFLOAT, &Inverter.ActivePowerFiltered, },                          // block 63
    {DEC3, FuncTypeFLOAT,FUNC_CAST(GetInverterAngle),   },                          // block 64
    {HEX,  FuncTypeUINT, FUNC_CAST(GetInverterStatus),  },                          // block 65
    
    {DEC3, MemTypeFLOAT, &Inverter.InverterVmag.phA,    },    	                    // block 66
    {DEC3, MemTypeFLOAT, &Inverter.VoltageNormFactor,   },                          // block 67
    {DEC3, MemTypeFLOAT, &Inverter.RMSVoltageNormFactor,},   						// block 68

    {DEC0, MemTypeINT,  &PHASEA_Thermal_Max,    },                                     // block 69
    {DEC4, MemTypeFLOAT, &Inverter.VoltageRefCorrection },                            // block 70
    {DEC0, MemTypeINT,  &PHASEA_Thermal_Min,    },                                     // block 71

// ********************************************************************************************************
// *    RECTIFIER BLOCKS 72-107
// ********************************************************************************************************
    {DEC1, MemTypeFLOAT, &UtilityVoltageRMS.RawRMS.phA, },                          // block 72
    {DEC1, MemTypeFLOAT, &UtilityVoltageRMS.RawRMS.phB, },                          // block 73
    {DEC1, MemTypeFLOAT, &UtilityVoltageRMS.RawRMS.phC, },                          // block 74 

    {DEC1, MemTypeFLOAT, &UtilityCurrentRMS.RawRMS.phA, },                          // block 75
    {DEC1, MemTypeFLOAT, &UtilityCurrentRMS.RawRMS.phB, },                          // block 76
    {DEC1, MemTypeFLOAT, &UtilityCurrentRMS.RawRMS.phC, },                          // block 77
    
    {DEC0, FuncTypeUINT, FUNC_CAST(GetRectifierState),      },                      // block 78
    {HEX,  FuncTypeUINT, FUNC_CAST(GetRectStatusW0),        },                      // block 79
    {HEX,  FuncTypeUINT, FUNC_CAST(GetRectStatusW1),        },                      // block 80
    
    {DEC1, MemTypeFLOAT, &Rectifier.UtilityPLL.SineRef.Frequency, },                // block 81
    {DEC3, MemTypeFLOAT, &Rectifier.ScaledIrefMax, },                               // block 82
    {DEC3, MemTypeFLOAT, &Rectifier.SineRefGain, },                                 // block 83

    {DEC1, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceDQOData.Sd, },                 // block 84
    {DEC1, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceDQOData.Sq, },                 // block 85
    {DEC1, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceDQOData.S0, },                 // block 86
    
    {DEC3, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceNominalDQO.Sd, },              // block 87
    {DEC3, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceNominalDQO.Sq, },              // block 88
    {DEC3, MemTypeFLOAT, &Rectifier.UtilityPLL.SourceNominalDQO.S0, },              // block 89

    {DEC0, MemTypeUINT,  &FanSpeedNew, },                                           // block 90
    {DEC0, MemTypeUINT,  &FanLoad, },                                               // block 91
//	    {NONE, MemTypeNONE,   0},                                              			// block 92
    {DEC4, MemTypeFLOAT,  &Rectifier.RectifierPowerShareBuffer, },                  // block 92


    {DEC1, MemTypeFLOAT, &ScreenMeters.DCLinkVoltage, },                            // block 93
    {DEC1, MemTypeFLOAT, &RawAdcData.st.RailVoltagePositive, },                     // block 94
    {DEC1, MemTypeFLOAT, &RawAdcData.st.RailVoltageNegative, },                     // block 95

    {DEC0, MemTypeUINT,  &Rectifier.FastOffsetCnt, },                               // block 96
    {DEC0, MemTypeUINT,  &Rectifier.SlowOffsetCnt, },                               // block 97
    {DEC1, FuncTypeFLOAT, FUNC_CAST(GetRectifierVoltageRef) },                      // block 98

//	    {DEC3, FuncTypeFLOAT, FUNC_CAST(GetRectIMag), },                                // block 99
//	    {DEC3, FuncTypeFLOAT, FUNC_CAST(GetRectifierOffset), },                         // block 100
	{DEC4, MemTypeFLOAT, &Rectifier.IcmdPowershareP, },								// block 99
	{DEC4, MemTypeFLOAT, &Rectifier.IcmdPowershareN, }, 						// block 100
    {DEC0, FuncTypeUINT, FUNC_CAST(GetAutoCalState), },                             // block 101

    {DEC1, MemTypeFLOAT, &AbsDCOVSet, },                                            // block 102
    {DEC1, MemTypeFLOAT, (float*)&DCLinkVoltagePositive.SlowFiltered, },            // block 103
    {DEC1, MemTypeFLOAT, (float*)&DCLinkVoltageNegative.SlowFiltered, },            // block 104

    {DEC0, MemTypeINT,  &PHASEB_Thermal_Max,     },                                    // block 105
    {DEC0, MemTypeUINT, 0, },                                                       // block 106
    {DEC0, MemTypeINT,  &PHASEB_Thermal_Min,     },                                    // block 107

// ********************************************************************************************************
// *    SYSTEM BLOCKS 108-143
// ********************************************************************************************************
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoA.word.LoWord,  },            // block 108
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoA.word.HiWord,  },            // block 109
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoB.word.LoWord,  },            // block 110         
    
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoB.word.HiWord,  },            // block 111
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoC.word.LoWord,  },            // block 112
    {HEX,  MemTypeUINT, (uint16_t*)&DSPOutRegister.GpoC.word.HiWord,  },            // block 113

    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiA.word.LoWord,   },            // block 114
    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiA.word.HiWord,   },            // block 115
    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiB.word.LoWord,   },            // block 116

    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiB.word.HiWord,   },            // block 117
    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiC.word.LoWord,   },            // block 118
    {HEX,  MemTypeUINT, (uint16_t*)&DSPInRegister.GpiC.word.HiWord,   },            // block 119

    {DEC0, MemTypeUINT, &RTC_SysTime.mSecOfMinute,                    },            // block 120
    {DEC0, MemTypeUINT, &RTC_SysTime.MinuteOfMonth,                   },            // block 121
    {DEC0, MemTypeUINT, &RTC_SysTime.YearAndMonth,                    },            // block 122

    {HEX,  MemTypeUINT, (uint16_t*)&ExpansionInputReg.all,            },            // block 123
    {HEX,  MemTypeUINT, &FanStatus,                                   },            // block 124
    {DEC1, MemTypeFLOAT,&ScreenMeters.PercentLoad.sum,                },            // block 125
    
    {DEC1, MemTypeFLOAT,&ScreenMeters.PercentLoad.phA,                },            // block 126
    {DEC1, MemTypeFLOAT,&ScreenMeters.PercentLoad.phB,                },            // block 127
    {DEC1, MemTypeFLOAT,&ScreenMeters.PercentLoad.phC,                },            // block 128

    {DEC1, MemTypeFLOAT,&LoadPower.ActivePower.phA,                   },            // block 129
    {DEC1, MemTypeFLOAT,&LoadPower.ActivePower.phB,                   },            // block 130
    {DEC1, MemTypeFLOAT,&LoadPower.ActivePower.phC,                   },            // block 131

    {DEC1, MemTypeFLOAT,&LoadPower.TotalPower.phA,                    },            // block 132
    {DEC1, MemTypeFLOAT,&LoadPower.TotalPower.phB,                    },            // block 133
    {DEC1, MemTypeFLOAT,&LoadPower.TotalPower.phC,                    },            // block 134

	{DEC1, MemTypeFLOAT, &ChassisVoltageRMS.FilteredRMS.phA, },                     // block 135, ChassicV: N phase cal
	{DEC1, MemTypeFLOAT, (float*)&InputNeutral.SlowFiltered, },                     // block 136, Nin:rsd, hw sample not correct
//		{DEC1, MemTypeFLOAT, 0, },                                                      // block 137
	{HEX, MemTypeUINT, &MCUStateMachine.BuildingInputs.words[0] , },				// block 137 

    {DEC0, MemTypeINT, &PHASEB_Thermal_C, },                                           // block 138 ok
    {DEC0, MemTypeINT, &BAT_Thermal_C, },                                              // block 139 bat
    {DEC0, MemTypeINT, &SCR_Thermal_C, },		                                       // block 140

    {DEC0, MemTypeINT, &PHASEA_Thermal_C , },                                          // block 141
	{DEC0, MemTypeINT, &PHASEC_Thermal_C, },									        // block 142
    {DEC0, MemTypeFLOAT, &AMB_Thermal_C,},                   				        // block 143

// ********************************************************************************************************
// *    CHARGER BLOCKS 144-179
// ********************************************************************************************************
    {DEC1, MemTypeFLOAT, &ScreenMeters.BatteryVoltage,  },                          // block 144
    {DEC1, MemTypeFLOAT, &ScreenMeters.BatteryCurrent, },                           // block 145
    {HEX,  FuncTypeUINT, FUNC_CAST(GetBatteryStatus) },                             // block 146
    
    {DEC0, MemTypeUINT, &BatteryConverter.BoChMachineState, },                      // block 147
    {DEC0, MemTypeUINT, &BatteryConverter.ChargerPhState, },                        // block 148
    {HEX,  MemTypeUINT, &BatteryConverter.BatteryStatus.words[0], },                // block 149

    {DEC1, MemTypeFLOAT, &BatteryConverter.ChargeVoltageTarget, },                  // block 150
    {DEC1, MemTypeFLOAT, &BatteryConverter.ChargeSWCurrentLimit, },                 // block 151
    {DEC1, MemTypeFLOAT, &BatteryConverter.DCLinkVoltageTarget, },                  // block 152

    {DEC3, MemTypeFLOAT, &BatteryConverter.ChargeDutyLimit, },                      // block 153
    {DEC3, MemTypeFLOAT, &BatteryConverter.MaxDutyPos, },                           // block 154
    {DEC1, MemTypeFLOAT, &BatteryConverter.RailBoostLim, },                         // block 155

    {DEC1, MemTypeFLOAT, (float*)&BatteryCurrentPos.SlowFiltered, },                // block 156
    {DEC1, MemTypeFLOAT, &ScreenMeters.PercentBatteryRemaining, },                  // block 157 
    {DEC4, MemTypeFLOAT, &BatteryConverter.BoostDutyNeg, },                         // block 158

    {DEC1, MemTypeFLOAT, (float*)&BatteryCurrentPos_LegB.SlowFiltered,},            // block 159
    {HEX,  FuncTypeUINT, FUNC_CAST(GetBatteryConverterStatus),  },                  // block 160
    {DEC4, MemTypeFLOAT, &BatteryConverter.dutyPos_chg, },                          // block 161

    {DEC0, FuncTypeUINT, FUNC_CAST(GetAbmState), },                                 // block 162
    {DEC0, MemTypeUINT, &AbmMaster.PreviousState, },                                // block 163
    {HEX,  FuncTypeUINT, FUNC_CAST(GetAbmStatus), },                                // block 164

    {DEC1, MemTypeFLOAT, &RawAdcData.st.BatteryVoltageChgNeg, },                    // block 165
    {DEC1, MemTypeFLOAT, &RawAdcData.st.BatteryVoltageChgPos, },                    // block 166
	{DEC1, MemTypeFLOAT, &ScreenMeters.BatteryCurrentP_Sum, },						// block 167

    {DEC1, MemTypeFLOAT, &ScreenMeters.CalculatedBTR, },                            // block 168
    {DEC3, MemTypeFLOAT, &BTR.PercentOfCapacity, },                                 // block 169
    {DEC0, FuncTypeFLOAT, FUNC_CAST(GetBatteryPower), },                            // block 170

    {DEC3, MemTypeFLOAT, &BTR.ChargeVoltagePercent, },                              // block 171
    {DEC3, MemTypeFLOAT, &BTR.ChargeCurrentPercent, },                              // block 172
    {DEC3, MemTypeFLOAT, &BTR.ChargeTimePercent, },                                 // block 173

    {DEC1, MemTypeFLOAT, &BTR.FullCapacityBTR, },                                   // block 174
    {DEC0, MemTypeUINT, &BTR.NumOfSDTIteration, },                                  // block 175
    {DEC0, MemTypeUINT, &BTR.NumOfLDTIteration, },                                  // block 176

    // 3 BTR debugger blocks
    {DEC1, MemTypeFLOAT, &BTR.UPSPower, },                                          // block 177
    {DEC3, MemTypeFLOAT, &BTR.BatEff, },                                            // block 178
    {DEC1, MemTypeFLOAT, &BTR.BatPower, },                                          // block 179
    
// ********************************************************************************************************
// *    Meters block screen 1 180-215 Gt page
// ********************************************************************************************************
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageRMS.phA, },                      // block 180
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageRMS.phB, },                      // block 181
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageRMS.phC, },                      // block 182
    
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageRMS.phA, },                   // block 183
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageRMS.phB, },                   // block 184
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageRMS.phC, },                   // block 185

    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageRMS.phA, },                     // block 186
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageRMS.phB, },                     // block 187
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageRMS.phC, },                     // block 188

    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltageRMS.phA, },                     // block 189
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltageRMS.phB, },                     // block 190
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltageRMS.phC, },                     // block 191

    {DEC1, MemTypeFLOAT, &ScreenMeters.InputCurrentRMS.phA, },                      // block 192
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputCurrentRMS.phB, },                      // block 193
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputCurrentRMS.phC, },                      // block 194

    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterCurrentRMS.phA, },                   // block 195
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterCurrentRMS.phB, },                   // block 196
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterCurrentRMS.phC, },                   // block 197

    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassCurrentRMS.phA, },                     // block 198
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassCurrentRMS.phB, },                     // block 199
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassCurrentRMS.phC, },                     // block 200

    {DEC1, MemTypeFLOAT, (float*)&DCLinkVoltagePositive.FastFiltered, },            // block 201
    {DEC1, MemTypeFLOAT, (float*)&DCLinkVoltageNegative.FastFiltered, },            // block 202
    {DEC1, MemTypeFLOAT, &ScreenMeters.BatteryVoltage, },                           // block 203

    {DEC1, MemTypeFLOAT, &ScreenMeters.BatteryCurrent, },                           // block 204
    {DEC1, MemTypeFLOAT, (float*)&BatteryVoltageChgPos.SlowFiltered, },             // block 205
    {DEC1, MemTypeFLOAT, (float*)&BatteryVoltageChgNeg.SlowFiltered, },             // block 206

    {DEC2, MemTypeFLOAT, &ScreenMeters.BypassFrequency, },                          // block 207
    {DEC2, MemTypeFLOAT, &ScreenMeters.OutputFrequency, },                          // block 208
    {DEC2, MemTypeFLOAT, &ScreenMeters.InputFrequency, },                           // block 209

    {DEC0, MemTypeFLOAT, &ScreenMeters.BypassPower.sum, },                          // block 210
    {DEC0, MemTypeFLOAT, &ScreenMeters.OutputPower.sum, },                          // block 211
    {DEC0, MemTypeFLOAT, &ScreenMeters.InputPower.sum, },                           // block 212

    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltage2RMS.phA, },                    // block 213
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltage2RMS.phB, },                    // block 214
    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltage2RMS.phC, },                    // block 215

// ********************************************************************************************************
// *    Meters block screen 2 216-251 Ge page
// ******************************************************************************************************** 
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputCurrentRMS.phA, },                     // block 216
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputCurrentRMS.phB, },                     // block 217
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputCurrentRMS.phC, },                     // block 218
    
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageLL_RMS.phA, },                   // block 219
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageLL_RMS.phB, },                   // block 220
    {DEC1, MemTypeFLOAT, &ScreenMeters.InputVoltageLL_RMS.phC, },                   // block 221

    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageLL_RMS.phA, },                // block 222
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageLL_RMS.phB, },                // block 223
    {DEC1, MemTypeFLOAT, &ScreenMeters.InverterVoltageLL_RMS.phC, },                // block 224

    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageLL_RMS.phA, },                  // block 225
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageLL_RMS.phB, },                  // block 226
    {DEC1, MemTypeFLOAT, &ScreenMeters.OutputVoltageLL_RMS.phC, },                  // block 227

    {DEC1, MemTypeFLOAT, &ScreenMeters.BypassVoltageLL_RMS.phA, },                  // block 228
    {DEC4, MemTypeFLOAT, &DebugFloat[0], },                                         // block 229

	{DEC4, MemTypeFLOAT, &DebugPara[0][0]},                                            // block 230
	{DEC4, MemTypeFLOAT, &DebugPara[0][1]},                                            // block 231
	{DEC4, MemTypeFLOAT, &DebugPara[0][2]},                                            // block 232
	{DEC4, MemTypeFLOAT, &DebugPara[0][3]},                                            // block 234

	{DEC4, MemTypeFLOAT, &DebugPara[1][0]},                                            // block 235
	{DEC4, MemTypeFLOAT, &DebugPara[1][1]},                                            // block 236
	{DEC4, MemTypeFLOAT, &DebugPara[1][2]},                                            // block 237
	{DEC4, MemTypeFLOAT, &DebugPara[1][3]},                                            // block 238

	{DEC4, MemTypeFLOAT, &DebugPara[2][0]},                                            // block 239
	{DEC4, MemTypeFLOAT, &DebugPara[2][1]},                                            // block 240
	{DEC4, MemTypeFLOAT, &DebugPara[2][2]},                                            // block 241
	{DEC4, MemTypeFLOAT, &DebugPara[2][3]},                                            // block 242

	{DEC4, MemTypeFLOAT, &DebugPara[3][0]},                                            // block 243
	{DEC4, MemTypeFLOAT, &DebugPara[3][1]},                                            // block 244
	{DEC4, MemTypeFLOAT, &DebugPara[3][2]},                                            // block 245
	{DEC4, MemTypeFLOAT, &DebugPara[3][3]},                                            // block 246

	{DEC4, MemTypeFLOAT, &DebugPara[4][0]},                                            // block 247
	{DEC4, MemTypeFLOAT, &DebugPara[4][1]},                                            // block 248
	{DEC4, MemTypeFLOAT, &DebugPara[4][2]},                                            // block 249
	{DEC4, MemTypeFLOAT, &DebugPara[4][3]},                                            // block 250
	{DEC4, MemTypeFLOAT, &DebugPara[5][0]},                                            // block 251
        
// ********************************************************************************************************
// *    PARALLEL BLOCK STRINGS 252-287
	{DEC4, MemTypeFLOAT, &DebugPara[5][1]},                                            // block 252
	{DEC4, MemTypeFLOAT, &DebugPara[5][2]},                                            // block 253
	{DEC4, MemTypeFLOAT, &DebugPara[5][3]},                                            // block 254

	{DEC4, MemTypeFLOAT, &DebugPara[6][0]},                                            // block 255
	{DEC4, MemTypeFLOAT, &DebugPara[6][1]},                                            // block 256
	{DEC4, MemTypeFLOAT, &DebugPara[6][2]},                                            // block 257
	{DEC4, MemTypeFLOAT, &DebugPara[6][3]},                                            // block 258

	{DEC4, MemTypeFLOAT, &DebugPara[7][0]},                                            // block 259
	{DEC4, MemTypeFLOAT, &DebugPara[7][1]},                                            // block 260
	{DEC4, MemTypeFLOAT, &DebugPara[7][2]},                                            // block 261
	{DEC4, MemTypeFLOAT, &DebugPara[7][3]},                                            // block 262
	
	{DEC4, MemTypeFLOAT, &DebugPara[8][0]},                                            // block 263
	{DEC4, MemTypeFLOAT, &DebugPara[8][1]},                                            // block 264
	{DEC4, MemTypeFLOAT, &DebugPara[8][2]},                                            // block 265
	{DEC4, MemTypeFLOAT, &DebugPara[8][3]},                                            // block 266

	{DEC4, MemTypeFLOAT, &DebugPara[9][0]},                                            // block 267
	{DEC4, MemTypeFLOAT, &DebugPara[9][1]},                                            // block 268
	{DEC4, MemTypeFLOAT, &DebugPara[9][2]},                                            // block 269
	{DEC4, MemTypeFLOAT, &DebugPara[9][3]},                                            // block 270
    
	{DEC4, MemTypeFLOAT, &DebugPara[10][0]},                                            // block 271
	{DEC4, MemTypeFLOAT, &DebugPara[10][1]},                                            // block 272
	{DEC4, MemTypeFLOAT, &DebugPara[10][2]},                                            // block 273
	{DEC4, MemTypeFLOAT, &DebugPara[10][3]},                                            // block 274
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[11][0]},                                            // block 275
	{DEC4, MemTypeFLOAT, &DebugPara[11][1]},                                            // block 276
	{DEC4, MemTypeFLOAT, &DebugPara[11][2]},                                            // block 277
	{DEC4, MemTypeFLOAT, &DebugPara[11][3]},                                            // block 278
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[12][0]},                                            // block 279
	{DEC4, MemTypeFLOAT, &DebugPara[12][1]},                                            // block 280
	{DEC4, MemTypeFLOAT, &DebugPara[12][2]},                                            // block 281
	{DEC4, MemTypeFLOAT, &DebugPara[12][3]},                                            // block 282
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[13][0]},                                            // block 283
	{DEC4, MemTypeFLOAT, &DebugPara[13][1]},                                            // block 284
	{DEC4, MemTypeFLOAT, &DebugPara[13][2]},                                            // block 285
	{DEC4, MemTypeFLOAT, &DebugPara[13][3]},                                            // block 286
	{DEC4, MemTypeFLOAT, &DebugPara[14][0]},                                            // block 287

// ********************************************************************************************************
// *    HEALTH BLOCK 1 Addresses 288-323//G1
// ********************************************************************************************************
	{DEC4, MemTypeFLOAT, &DebugPara[14][1]},                                            // block 288
	{DEC4, MemTypeFLOAT, &DebugPara[14][2]},                                            // block 289
	{DEC4, MemTypeFLOAT, &DebugPara[14][3]},                                            // block 290

	{DEC4, MemTypeFLOAT, &DebugPara[15][0]},                                            // block 291
	{DEC4, MemTypeFLOAT, &DebugPara[15][1]},                                            // block 292
	{DEC4, MemTypeFLOAT, &DebugPara[15][2]},                                            // block 293
	{DEC4, MemTypeFLOAT, &DebugPara[15][3]},                                            // block 294

	{DEC4, MemTypeFLOAT, &DebugPara[16][0]},                                            // block 295
	{DEC4, MemTypeFLOAT, &DebugPara[16][1]},                                            // block 296
	{DEC4, MemTypeFLOAT, &DebugPara[16][2]},                                            // block 297
	{DEC4, MemTypeFLOAT, &DebugPara[16][3]},                                            // block 298
	
	{DEC4, MemTypeFLOAT, &DebugPara[17][0]},                                            // block 299
	{DEC4, MemTypeFLOAT, &DebugPara[17][1]},                                            // block 300
	{DEC4, MemTypeFLOAT, &DebugPara[17][2]},                                            // block 301
	{DEC4, MemTypeFLOAT, &DebugPara[17][3]},                                            // block 302

	{DEC4, MemTypeFLOAT, &DebugPara[18][0]},                                            // block 303
	{DEC4, MemTypeFLOAT, &DebugPara[18][1]},                                            // block 304
	{DEC4, MemTypeFLOAT, &DebugPara[18][2]},                                            // block 305
	{DEC4, MemTypeFLOAT, &DebugPara[18][3]},                                            // block 306
    
	{DEC4, MemTypeFLOAT, &DebugPara[19][0]},                                            // block 307
	{DEC4, MemTypeFLOAT, &DebugPara[19][1]},                                            // block 308
	{DEC4, MemTypeFLOAT, &DebugPara[19][2]},                                            // block 309
	{DEC4, MemTypeFLOAT, &DebugPara[19][3]},                                            // block 310
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[20][0]},                                            // block 311
	{DEC4, MemTypeFLOAT, &DebugPara[20][1]},                                            // block 312
	{DEC4, MemTypeFLOAT, &DebugPara[20][2]},                                            // block 313
	{DEC4, MemTypeFLOAT, &DebugPara[20][3]},                                            // block 314
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[21][0]},                                            // block 315
	{DEC4, MemTypeFLOAT, &DebugPara[21][1]},                                            // block 316
	{DEC4, MemTypeFLOAT, &DebugPara[21][2]},                                            // block 317
	{DEC4, MemTypeFLOAT, &DebugPara[21][3]},                                            // block 318
                                                                        
	{DEC4, MemTypeFLOAT, &DebugPara[22][0]},                                            // block 319
	{DEC4, MemTypeFLOAT, &DebugPara[22][1]},                                            // block 320
	{DEC4, MemTypeFLOAT, &DebugPara[22][2]},                                            // block 321
	{DEC4, MemTypeFLOAT, &DebugPara[22][3]},                                            // block 322
	{DEC4, MemTypeFLOAT, &DebugPara[23][0]},                                            // block 323

// ********************************************************************************************************
// *    G2 324-359
// ********************************************************************************************************
	{DEC4, MemTypeFLOAT, &DebugPara[23][1]},                                            // block 324
	{DEC4, MemTypeFLOAT, &DebugPara[23][2]},                                            // block 325
	{DEC4, MemTypeFLOAT, &DebugPara[23][3]},                                            // block 326

	{DEC4, MemTypeFLOAT, &DebugPara[24][0]},                                            // block 327
	{DEC4, MemTypeFLOAT, &DebugPara[24][1]},                                            // block 328
	{DEC4, MemTypeFLOAT, &DebugPara[24][2]},                                            // block 329
	{DEC4, MemTypeFLOAT, &DebugPara[24][3]},                                            // block 330

	{DEC4, MemTypeFLOAT, &DebugPara[25][0]},                                            // block 331
	{DEC4, MemTypeFLOAT, &DebugPara[25][1]},                                            // block 332
	{DEC4, MemTypeFLOAT, &DebugPara[25][2]},                                            // block 334
	{DEC4, MemTypeFLOAT, &DebugPara[25][3]},                                            // block 335
		
	{DEC4, MemTypeFLOAT, &DebugPara[26][0]},                                            // block 336
	{DEC4, MemTypeFLOAT, &DebugPara[26][1]},                                            // block 337
	{DEC4, MemTypeFLOAT, &DebugPara[26][2]},                                            // block 338
	{DEC4, MemTypeFLOAT, &DebugPara[26][3]},                                            // block 339

	{DEC4, MemTypeFLOAT, &DebugPara[27][0]},                                            // block 340
	{DEC4, MemTypeFLOAT, &DebugPara[27][1]},                                            // block 341
	{DEC4, MemTypeFLOAT, &DebugPara[27][2]},                                            // block 342
	{DEC4, MemTypeFLOAT, &DebugPara[27][3]},                                            // block 343

	{DEC4, MemTypeFLOAT, &DebugPara[28][0]},                                            // block 344
	{DEC4, MemTypeFLOAT, &DebugPara[28][1]},                                            // block 345
	{DEC4, MemTypeFLOAT, &DebugPara[28][2]},                                            // block 346
	{DEC4, MemTypeFLOAT, &DebugPara[28][3]},                                            // block 347

	{DEC4, MemTypeFLOAT, &DebugPara[29][0]},                                            // block 348
	{DEC4, MemTypeFLOAT, &DebugPara[29][1]},                                            // block 349
	{DEC4, MemTypeFLOAT, &DebugPara[29][2]},                                            // block 350
	{DEC4, MemTypeFLOAT, &DebugPara[29][3]},                                            // block 351

	{DEC4, MemTypeFLOAT, &DebugPara[30][0]},                                            // block 352
	{DEC4, MemTypeFLOAT, &DebugPara[30][1]},                                            // block 353
	{DEC4, MemTypeFLOAT, &DebugPara[30][2]},                                            // block 354
	{DEC4, MemTypeFLOAT, &DebugPara[30][3]},                                            // block 355
	
	{DEC4, MemTypeFLOAT, &DebugPara[31][0]},                                            // block 356
	{DEC4, MemTypeFLOAT, &DebugPara[31][1]},                                            // block 357
	{DEC4, MemTypeFLOAT, &DebugPara[31][2]},                                            // block 358
	{DEC4, MemTypeFLOAT, &DebugPara[31][3]},                                            // block 359

// ********************************************************************************************************
// *    G3 360-395
// ********************************************************************************************************
	{DEC4, MemTypeFLOAT, &DebugPara[32][0]},                                            // block 360
	{DEC4, MemTypeFLOAT, &DebugPara[32][1]},                                            // block 361
	{DEC4, MemTypeFLOAT, &DebugPara[32][2]},                                            // block 362
	{DEC4, MemTypeFLOAT, &DebugPara[32][3]},                                            // block 363

	{DEC4, MemTypeFLOAT, &DebugPara[33][0]},                                            // block 364
	{DEC4, MemTypeFLOAT, &DebugPara[33][1]},                                            // block 365
	{DEC4, MemTypeFLOAT, &DebugPara[33][2]},                                            // block 366
	{DEC4, MemTypeFLOAT, &DebugPara[33][3]},                                            // block 367

	{DEC4, MemTypeFLOAT, &DebugPara[34][0]},                                            // block 368
	{DEC4, MemTypeFLOAT, &DebugPara[34][1]},                                            // block 369
	{DEC4, MemTypeFLOAT, &DebugPara[34][2]},                                            // block 370
	{DEC4, MemTypeFLOAT, &DebugPara[34][3]},                                            // block 371
		
	{DEC4, MemTypeFLOAT, &DebugPara[35][0]},                                            // block 372
	{DEC4, MemTypeFLOAT, &DebugPara[35][1]},                                            // block 373
	{DEC4, MemTypeFLOAT, &DebugPara[35][2]},                                            // block 374
	{DEC4, MemTypeFLOAT, &DebugPara[35][3]},                                            // block 375
	
	{DEC4, MemTypeFLOAT, &DebugPara[36][0]},                                            // block 376
	{DEC4, MemTypeFLOAT, &DebugPara[36][1]},                                            // block 377
	{DEC4, MemTypeFLOAT, &DebugPara[36][2]},                                            // block 378
	{DEC4, MemTypeFLOAT, &DebugPara[36][3]},                                            // block 379
	
	{DEC4, MemTypeFLOAT, &DebugPara[37][0]},                                            // block 380
	{DEC4, MemTypeFLOAT, &DebugPara[37][1]},                                            // block 381
	{DEC4, MemTypeFLOAT, &DebugPara[37][2]},                                            // block 382
	{DEC4, MemTypeFLOAT, &DebugPara[37][3]},                                            // block 383


	{DEC4, MemTypeFLOAT, &DebugPara[38][0]},                                            // block 384
	{DEC4, MemTypeFLOAT, &DebugPara[38][1]},                                            // block 385
	{DEC4, MemTypeFLOAT, &DebugPara[38][2]},                                            // block 386
	{DEC4, MemTypeFLOAT, &DebugPara[38][3]},                                            // block 387
	
	{DEC4, MemTypeFLOAT, &DebugPara[39][0]},                                            // block 388
	{DEC4, MemTypeFLOAT, &DebugPara[39][1]},                                            // block 389
	{DEC4, MemTypeFLOAT, &DebugPara[39][2]},                                            // block 390
	{DEC4, MemTypeFLOAT, &DebugPara[39][3]},                                            // block 391

	{DEC4, MemTypeFLOAT, &DebugPara[40][0]},                                            // block 392
	{DEC4, MemTypeFLOAT, &DebugPara[40][1]},                                            // block 393
	{DEC4, MemTypeFLOAT, &DebugPara[40][2]},                                            // block 394
	{DEC4, MemTypeFLOAT, &DebugPara[40][3]},                                            // block 395

// ********************************************************************************************************
// *    G4 396-431
// ********************************************************************************************************
	{DEC4, MemTypeFLOAT, &DebugPara[41][0]},                                            // block 396
	{DEC4, MemTypeFLOAT, &DebugPara[41][1]},                                            // block 397
	{DEC4, MemTypeFLOAT, &DebugPara[41][2]},                                            // block 398
	{DEC4, MemTypeFLOAT, &DebugPara[41][3]},                                            // block 399

	{DEC4, MemTypeFLOAT, &DebugPara[42][0]},                                            // block 400
	{DEC4, MemTypeFLOAT, &DebugPara[42][1]},                                            // block 401
	{DEC4, MemTypeFLOAT, &DebugPara[42][2]},                                            // block 402
	{DEC4, MemTypeFLOAT, &DebugPara[42][3]},                                            // block 403


	{DEC4, MemTypeFLOAT, &DebugPara[43][0]},                                            // block 404
	{DEC4, MemTypeFLOAT, &DebugPara[43][1]},                                            // block 405
	{DEC4, MemTypeFLOAT, &DebugPara[43][2]},                                            // block 406
	{DEC4, MemTypeFLOAT, &DebugPara[43][3]},                                            // block 407

	{DEC4, MemTypeFLOAT, &DebugPara[44][0]},                                            // block 408
	{DEC4, MemTypeFLOAT, &DebugPara[44][1]},                                            // block 409
	{DEC4, MemTypeFLOAT, &DebugPara[44][2]},                                            // block 410
	{DEC4, MemTypeFLOAT, &DebugPara[44][3]},                                            // block 411
	
	{DEC4, MemTypeFLOAT, &DebugPara[45][0]},                                            // block 412
	{DEC4, MemTypeFLOAT, &DebugPara[45][1]},                                            // block 413
	{DEC4, MemTypeFLOAT, &DebugPara[45][2]},                                            // block 414
	{DEC4, MemTypeFLOAT, &DebugPara[45][3]},                                            // block 415

	{DEC4, MemTypeFLOAT, &DebugPara[46][0]},                                            // block 416
	{DEC4, MemTypeFLOAT, &DebugPara[46][1]},                                            // block 417
	{DEC4, MemTypeFLOAT, &DebugPara[46][2]},                                            // block 418
	{DEC4, MemTypeFLOAT, &DebugPara[46][3]},                                            // block 419

	{DEC4, MemTypeFLOAT, &DebugPara[47][0]},                                            // block 420
	{DEC4, MemTypeFLOAT, &DebugPara[47][1]},                                            // block 421
	{DEC4, MemTypeFLOAT, &DebugPara[47][2]},                                            // block 422
	{DEC4, MemTypeFLOAT, &DebugPara[47][3]},                                            // block 423
	
	{DEC4, MemTypeFLOAT, &DebugPara[48][0]},                                            // block 424
	{DEC4, MemTypeFLOAT, &DebugPara[48][1]},                                            // block 425
	{DEC4, MemTypeFLOAT, &DebugPara[48][2]},                                            // block 426
	{DEC4, MemTypeFLOAT, &DebugPara[48][3]},                                            // block 427
	
	{DEC4, MemTypeFLOAT, &DebugPara[49][0]},                                            // block 428
	{DEC4, MemTypeFLOAT, &DebugPara[49][1]},                                            // block 429
	{DEC4, MemTypeFLOAT, &DebugPara[49][2]},                                            // block 430
	{DEC4, MemTypeFLOAT, &DebugPara[49][3]},                                            // block 431
};


const uint16_t BlockArraySize = sizeof( BlockArray ) / sizeof( stBlock );

// ********************************************************************************************************
// *            END OF DebuggerBlocks.c
// ********************************************************************************************************
