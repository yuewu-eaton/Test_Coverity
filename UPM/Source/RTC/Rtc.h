// ********************************************************************************************************
// *            RTC.h
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
// *    FILE NAME: RTC.h
// *
// *    DESCRIPTION: Structure and functions for Real Time Clock.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/8/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************
#ifndef _RTC_H
#define _RTC_H

// *********************************************************************************************************
// *        DEFINES
// *********************************************************************************************************
#define RTC_SYSTIME_UPDATE_RATE     5       // Dependent on however often SysTime update is called, in mSec
                                            // At present time, SysTime is updated every 5 mSec.
#define RTC_MSECPERMINUTE           60000   // mSec per minute
#define RTC_MSECPERSECOND           1000

#define RTC_MIN_TO_NEXT_SAVE        180     // save RTC every 3 hours


// *********************************************************************************************************
// *        Definition of SysTime Structure
// *********************************************************************************************************
typedef struct{
    uint16_t      mSecOfMinute;           // 0 .. 60 000 mSec per minute

    uint16_t      MinuteOfMonth;          // 0 .. 44640 Minutes per month (60 min/hr * 24 hr/day * 31 day/month)

    uint16_t      YearAndMonth;           // Bits 15 .. 4 = Year  ( Years from 0 to 4095 can be represented. )
                                        // Bits  3 .. 0 = Month ( Months from 0 to 12 can be represented. )

} RTC_Time;

typedef struct{
    uint16_t  mSec;
    uint16_t  second;
    uint16_t  minute;
    uint16_t  hour;
    uint16_t  day;
    uint16_t  month;
    uint16_t  year;
} RTC_FormattedTime;

typedef struct {
    uint16_t  secs_bcd;
    uint16_t  min_bcd;
    uint16_t  hours_bcd;
    uint16_t  day_bcd;
    uint16_t  date_bcd;
    uint16_t  month_bcd;
    uint16_t  year_bcd;
} stRTC_Bcd_Time;

typedef union {
    stRTC_Bcd_Time  s;
    uint16_t          w[sizeof(stRTC_Bcd_Time)];
} uRTC_Bcd_Time;

typedef struct
{
    uint16_t  Ctrl_Reg;
    uint16_t  Stat_Reg;
    uint16_t  Chrg_Reg;
} st_RTC_Regs;

typedef union
{
    st_RTC_Regs     reg;
    uint16_t          w[sizeof(st_RTC_Regs)];
} uRTC_Regs;

// *********************************************************************************************************
// *        Public Data
// *********************************************************************************************************
extern RTC_Time            RTC_SysTime;    // Time stamp for history queue
extern RTC_FormattedTime   RTC_FormTime;   // For xcp meters block
extern uRTC_Bcd_Time       RTC_BcdTime;    // Original time from RTC

// *********************************************************************************************************
// *        Function Prototypes
// *********************************************************************************************************
extern "C"
{
void RTC_InitSysTime(void);
}

void RTC_IncSysTime(void);
void RTC_InitSysTime(void);
void RTC_FormatTime(const RTC_Time* TimeCube, RTC_FormattedTime* PrettyTime);
void BcdTimeToSysTime(const uRTC_Bcd_Time* bcd, RTC_Time* sys);
void SysTimeToBcdTime(const RTC_Time* sys, uRTC_Bcd_Time* bcd);

// ********************************************************************************************************
// *        SPI Defines
// ********************************************************************************************************
// only use upper byte
#define RTC_CON_REG_WRITE_ADDRESS       0x8f00
#define RTC_CON_REG_READ_ADDRESS        0x0f00
#define RTC_CHARGE_REG_WRITE_ADDRESS    0x9100
#define RTC_CHARGE_REG_READ_ADDRESS     0x1100

#define RTC_CHRG_CONFIG                 0x00A5      // 1 diode, 2k - highest charge current
#define RTC_CTRL_CONFIG                 0x0000
#define RTC_WP_DISABLE                  0x0000
#define RTC_WP_ENABLE                   0x4000

#define RTC_SEC_REG_WRITE_ADDRESS       0x8000
#define RTC_SEC_REG_READ_ADDRESS        0x0000
#define RTC_MIN_REG_WRITE_ADDRESS       0x8100
#define RTC_MIN_REG_READ_ADDRESS        0x0100
#define RTC_HOUR_REG_WRITE_ADDRESS      0x8200
#define RTC_HOUR_REG_READ_ADDRESS       0x0200
#define RTC_DAY_REG_WRITE_ADDRESS       0x8300
#define RTC_DAY_REG_READ_ADDRESS        0x0300
#define RTC_DATE_REG_WRITE_ADDRESS      0x8400
#define RTC_DATE_REG_READ_ADDRESS       0x0400
#define RTC_MON_REG_WRITE_ADDRESS       0x8500
#define RTC_MON_REG_READ_ADDRESS        0x0500
#define RTC_YEAR_REG_WRITE_ADDRESS      0x8600
#define RTC_YEAR_REG_READ_ADDRESS       0x0600

#define RTC_NVRAM_START_ADDRESS         0x0020
#define RTC_NVRAM_END_ADDRESS           0x007F

#define RTC_NVRAM_SIZE                  (RTC_NVRAM_READ_END_ADDRESS - RTC_NVRAM_READ_START_ADDRESS)

#define RTC_ADDRESS_DUMMY               100
#define RTC_NUM_WORDS_DUMMY             1

// RTC stores year as 0-100 need offset to get actual year. 
#define RTC_YEAR_OFFSET                 2000


enum {
    SYS_TIME,
    FORM_TIME,
    BCD_TIME
};

// ********************************************************************************************************
// *            END OF RTC.h
// ********************************************************************************************************
#endif
