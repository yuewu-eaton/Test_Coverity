#ifndef PANDA_CANMETERS_H
#define PANDA_CANMETERS_H
//******************************************************************************
//*                          CanMeters.h
//******************************************************************************

//******************************************************************************
//*
//* This Information Proprietary  To Eaton Corporation
//*
//******************************************************************************
//*
//*           Copyright (c) 2010 Eaton Corporation
//*                  ALL RIGHTS RESERVED
//*
//******************************************************************************

//******************************************************************************
//* FILE NAME: CanMeters.h
//*
//* DESCRIPTION: Meters header. These indexes correspond to the order of the meters
//* as they are transferred on the wire. The names also correspond to the index
//* to be used by user code on the CSB.  This file must be synchronized between
//* the CSB and UPM projects.
//*
//* CSB code should not #include this file directly, but should only #include
//* Meters.h.
//*
//*
//* ORIGINATOR: Jan-Erik Berger
//*
//* DATE: 04/13/2010
//*
//******************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

// Meter indexes
typedef enum _can_meter_t
{
    // Implementation note: CSB FRS Sections 10.3.1, 4.9.2, and 4.12.5 define
    // these values.

    // uint16_t meters start
    // Voltage meters
    V12_BYP_METER = 0x0000,
    V23_BYP_METER,
    V31_BYP_METER,
    V1_BYP_METER,
    V2_BYP_METER,
    V3_BYP_METER,
    V12_INP_METER,
    V23_INP_METER,
    V31_INP_METER,
    V1_INP_METER,
    V2_INP_METER,
    V3_INP_METER,
    V12_OUTP_METER,
    V23_OUTP_METER,
    V31_OUTP_METER,
    V1_OUTP_METER,
    V2_OUTP_METER,
    V3_OUTP_METER,
    VBAT_METER,
    VBAT_POS_METER,
    VPC_METER,
    VPC1_METER,
    VPC2_METER,
    // Frequency meters
    FREQ_BYP_METER,
    FREQ_INP_METER,
    FREQ_OUTP_METER,
    // Power factor meters
    PF_BYP_METER,
    PF_INP_METER,
    PF_OUTP_METER,
    // Battery meters
    BTR_METER,
    BAT_PERCENT,
    // Load percentage meter
    LOAD_PERCENT,
    // Voltage ratings
    INPUT_VOLTAGE_RATING,
    OUTPUT_VOLTAGE_RATING,
    INPUT_VOLTAGE_MAX_RATING,
    INPUT_VOLTAGE_MIN_RATING,    
    // Frequency ratings
    INPUT_FREQUENCY_RATING,
    OUTPUT_FREQUENCY_RATING,
    // Output PF rating
    OUTPUT_PF_RATING,
    // Output kVA rating
    OUTPUT_KVA_RATING,
    // Battery ratings
    BAT_LOAD_RATING,
    BAT_TIME_RATING,
    // Load percentage meter
    LOAD_PERCENT_1,
    LOAD_PERCENT_2,
    LOAD_PERCENT_3,    
    // uint16_t meters end
    END_OF_INT16U_METERS,

    // int16_t meters start
    // Current meters
    I1_BYP_METER = END_OF_INT16U_METERS,
    I2_BYP_METER,
    I3_BYP_METER,
    I1_INP_METER,
    I2_INP_METER,
    I3_INP_METER,
    I1_OUTP_METER,
    I2_OUTP_METER,
    I3_OUTP_METER,
    IBAT_METER,
    IBAT_POS_METER,
    IBAT_NEG_METER,
    // Battery negative voltage meter
    VBAT_NEG_METER,
    // kVA / kW meters
    KW_BYP_METER,
    KVA_BYP_METER,
    KW_INP_METER,
    KVA_INP_METER,
    KW_OUTP_METER,
    KVA_OUTP_METER,
    // kVA / kW meters for phases
    KW_OUTP_METER_1,  
    KVA_OUTP_METER_1, 
    KW_OUTP_METER_2,  
    KVA_OUTP_METER_2, 
    KW_OUTP_METER_3,  
    KVA_OUTP_METER_3, 
    // int16_t meters end
    END_OF_INT16S_METERS,
    // Int32S meters end. None defined for now, but retain the capability.
    END_OF_INT32S_METERS = END_OF_INT16S_METERS
} can_meter_t;

#ifdef __cplusplus
}
#endif

#endif
/***************************END OF Meters.h************************************/
