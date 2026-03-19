// ********************************************************************************************************
// *            RTC.c
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
// *    FILE NAME: RTC.c
// *
// *    DESCRIPTION: Real Time Clock functions for SysTime.  Currently the RTC is entirely a software clock,
// *                    but when a hardware-based RTC is integrated into the system, this framework can be used.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 4/8/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// BladeUPS was SPI, needs changing to I2C. I think.

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Rtc.h"
#include "Spi_Task.h"
#include "NB_Funcs.h"
#include "Version.h"
#include "Constants.h"
#include <string.h>

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************
RTC_Time            RTC_SysTime;    // Time stamp for history queue
RTC_FormattedTime   RTC_FormTime;   // For xcp meters block
uRTC_Bcd_Time       RTC_BcdTime;    // Original time from RTC


// ********************************************************************************************************
// * LOCAL VARIABLES
// ********************************************************************************************************
const uint16_t MinutesPerMonth[] =
{
    0,
    44640,  // Jan, 31 days
    40320,  // Feb, 28 days
    44640,  // Mar, 31 days
    43200,  // Apr, 30 days
    44640,  // May, 31 days
    43200,  // Jun, 30 days
    44640,  // Jul, 31 days
    44640,  // Aug, 31 days
    43200,  // Sep, 30 days
    44640,  // Oct, 31 days
    43200,  // Nov, 30 days
    44640,  // Dec, 31 days
    41760   // Feb, 29 days
};
// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************
uint16_t RTC_CalcMinutesThisMonth(uint16_t Month, uint16_t Year);
uint16_t AccessTime(uint16_t *time, uint16_t mode);
uint16_t AccessRegs(uint16_t *reg, uint16_t mode);
uint16_t AccessNVRam(uint16_t start_add, uint16_t numwds, uint16_t* dataptr, uint16_t mode);
uint16_t CheckValidRTCAddress(uint16_t start, uint16_t numwds, uint16_t mode);
uint16_t RTC_IsLeapYear(uint16_t Year);
uint16_t RTC_DaysThisMonth(uint16_t month, uint16_t leap_year);

void FormTimeToBcdTime(uRTC_Bcd_Time* rtc_time, const RTC_FormattedTime* time);



// ********************************************************************************************************
// *
// * Function: RTC_InitSysTime
// *
// * Purpose: initialize RTC
// *
// * Parms Passed   :   none
// *
// * Returns        :   none
// *
// * Description    :   Initializes the RTC and timer
// *
// ********************************************************************************************************
void RTC_InitSysTime(void)
{
    // Set the time to zero. Indicates that the clock hasn't been set yet.
    RTC_SysTime.MinuteOfMonth = 0;
    RTC_SysTime.mSecOfMinute = 0;
    RTC_SysTime.YearAndMonth = 0;
}

// ********************************************************************************************************
// *
// * Function: RTC_FormatTime(RTC_FormattedTime* PrettyTime)
// *
// * Purpose: Takes a time object and returns a formatted time object.
// *
// * Parms Passed   :   TimeCube - RTC_Time object to be formatted.
// *                    PrettyTime - RTC_FormattedTime object that will contain the formatted time.
// *
// * Returns        :   nothing
// *
// * Description    :   Takes a time object and returns a formatted time object.
// *
// ********************************************************************************************************
void RTC_FormatTime(const RTC_Time* TimeCube, RTC_FormattedTime* PrettyTime)
{
    if (TimeCube != 0)
    {
        PrettyTime->mSec = TimeCube->mSecOfMinute % 1000;           // 0 .. 999
        PrettyTime->second = TimeCube->mSecOfMinute / 1000;         // 0 .. 59
        PrettyTime->minute = TimeCube->MinuteOfMonth % 60;          // 0 .. 59
        PrettyTime->hour = (TimeCube->MinuteOfMonth % 1440) / 60;   // 0 .. 23
        PrettyTime->day = (TimeCube->MinuteOfMonth / 1440) + 1;           // 1 .. 31
        PrettyTime->month = (TimeCube->YearAndMonth & 0x0F);// + 1;          // 1 .. 12
        PrettyTime->year = ((TimeCube->YearAndMonth & 0xFFF0) >> 4);// + 1;  // 0 .. 2^12-1
    }
}

// ********************************************************************************************************
// *
// * Function: RTC_DaysThisMonth(uint16_t month, uint16_t leap year)
// *
// * Purpose: Gets system time
// *
// * Parms Passed : the month and leap year
// *
// * Returns: Number of days in the month
// *
// * Description: February is 28 days, except leap year, is 29 days. July and earlier, odd months have
// *              31 days, even months have 30, then after even months have 31 days, odd have 30. Mr. Gregor
// *              was obviously an opium addict.
// *                
// ********************************************************************************************************
uint16_t RTC_DaysThisMonth(uint16_t month, uint16_t leap_year)
{
    uint16_t days;

        // February is special case
    if (month == 2)
    {
        if (leap_year)
        {
            days = 29;
        }
        else
        {
            days = 28;
        }
    }
    else
    {
        // prior to and including July, odd months have 31 days, even have 30 (except Feb.)
        if (month <= 7)
        {
            if (month & 0x1)
            {
                days = 31;
            }
            else
            {
                days = 30;
            }
        }
        // AFTER July, even months have 31 days, odd have 30
        else
        {   
            if (month & 0x1)
            {
                days = 30;
            }
            else
            {
                days = 31;
            }
        }
    }

    return days;
}

// ********************************************************************************************************
// *
// * Function: RTC_IsLeapYear
// *
// * Purpose: To determine if the year is a Leap year or not.
// *
// * Parms Passed   :   Year - year to be checked for "Leap'ness"
// *
// * Returns        :   1 if leap, 0 otherwise
// *
// * Description    :   Figures out whether Year is Leap or Not.  Uses following formula:
// *                    Note: Got Leap year formula from: http://www.mitre.org/research/cots/LEAPCALC.html
// *
// *                        Leap Year calculation looks like:
// *                             if (year mod 4 != 0)
// *                                 {use 28 for days in February}
// *                             else if  (year mod 400 == 0)
// *                                 {use 29 for days in February}
// *                             else if (year mod 100 == 0)
// *                                 {use 28 for days in February}
// *                             else
// *                                 {use 29 for days in February}
// ********************************************************************************************************
uint16_t  RTC_IsLeapYear(uint16_t Year)
{
    uint16_t  leap;

    if (Year%4 != 0)
    {
        leap = 0;
    }
    else if (Year%400 == 0 )
    {
        leap = 1;
    }
    else if (Year%100 == 0)
    {
        leap = 0;
    }
    else
    {
        leap = 1;
    }

    return leap;
}

// ********************************************************************************************************
// *
// * Function: RTC_IncSysTime(void)
// *
// * Purpose: To increment SysTime.  Function disables interrupts during the critical region, to prevent
// *            inserting wrong time-stamps into the history que.
// *
// * Parms Passed   :   none
// *
// * Returns        :   none
// *
// * Description    :   Increments SysTime by RTC_SYSTIME_UPDATE_RATE.  Updates Minutes of Month, Month and Year
// *                    whenever mSec counts above 60000.
// *
// ********************************************************************************************************
void RTC_IncSysTime( void )		//5ms
{
    uint16_t  mSecOfMinute;
    uint16_t  MinuteOfMonth;
    uint16_t  Month;
    uint16_t  Year;
    uint32_t  timer;
    static  uint32_t lastTimer = 0;
    static  int16_t  lastMonth = -1;
    static  uint16_t MinutesThisMonth = 0;

    mSecOfMinute    = RTC_SysTime.mSecOfMinute;
    MinuteOfMonth   = RTC_SysTime.MinuteOfMonth;
    Month           = RTC_SysTime.YearAndMonth & 0x0F;
    Year            = (RTC_SysTime.YearAndMonth & 0xFFF0)>>4;

    if ( -1 == lastMonth )
    {
        lastTimer = ReadCpuTimer0Counter();
    }
            
    if (lastMonth != Month)
    {
        MinutesThisMonth = RTC_CalcMinutesThisMonth(Month,Year);
        lastMonth = Month;
    }

    timer = ReadCpuTimer0Counter();                                     // Read CPU timer0
    mSecOfMinute += ((lastTimer - timer) + 5000)/10000;                 // timer0 tick = 100ns. Change to ms, round
    lastTimer = timer;                                                  // save for next time

    if (mSecOfMinute >= RTC_MSECPERMINUTE)  // If new minute,
    {
        mSecOfMinute -= RTC_MSECPERMINUTE;  // Must subtract by 60K, since last
                                            // mSecOfMinute += RTC_SYSTIME_UPDATE_RATE may have ended up > 60K
        MinuteOfMonth ++;                   // Update minute of month as well

        if (MinuteOfMonth >= MinutesThisMonth)  // if new month,
        {
            MinuteOfMonth = 0;
            Month++;

            if (Month > 12)                // if new Year,
            {
                Month = 1;                  // Make it January again
                Year ++;                    // Increment year.
            }   // end of 'if (Month >= 12)

            MinutesThisMonth = RTC_CalcMinutesThisMonth(Month,Year);   //Recalc number of minutes this new month

        }   // end of "if MinuteOfMonth >= MinutesThisMonth"

    }       // end of 'if mSecOfMinute >= 60000'


    Year = (Year<<4)|Month;     // Pack Year and Month as needed in SysTime

    // Now update SysTime.  Interrupts are disabled during this operation, as DebounceAndQue could
    // request SysTime whenever queing anything into history que (could be from higher priority ints)
    // Obviously, do not doddle while here

    CriticalSection enter;
    RTC_SysTime.mSecOfMinute  = mSecOfMinute;
    RTC_SysTime.MinuteOfMonth = MinuteOfMonth;
    RTC_SysTime.YearAndMonth  = Year;
}           // end of RTC_IncSysTime

// ********************************************************************************************************
// *
// * Function: RTC_CalcMinutesThisMonth
// *
// * Purpose: Calculates number of minutes for this month.  Takes Leap years into account.
// *
// * Parms Passed   :   Year - year to be checked for "Leap'ness"
// *                :   Month: 0 = January, 1 = February, .. 11 = December
// *
// * Returns        :   Number of minutes for this month
// *
// * Description    :   Returns number of minutes for this month.  If February, checks whether leap year.
// ********************************************************************************************************
uint16_t RTC_CalcMinutesThisMonth(uint16_t Month, uint16_t Year)
{
    uint16_t  tmp;

    tmp = RTC_IsLeapYear(Year);

    if ((Month == 2) & tmp)              // if February, and Year is Leap,
    {
        tmp = 13;                       // If Leap Year, use 29 day February
    }
    else
    {
        tmp = Month;
    }

    tmp = MinutesPerMonth[tmp];
    return tmp;

}

// ********************************************************************************************************
// *
// * Function: void SysTimeToBcdTime(RTC_Time* sys, uRTC_Bcd_Time* bcd)
// *
// * Purpose: Converts system time to RTC bcd time 
// *
// * Parms Passed   :   pointer to structures
// *
// * Returns        :   Nothing
// *
// * Description    :  mSecOfMinute = RTC seconds * 1000
//                     MinuteOfMonth = (RTC date-1) * 24 * 60 +
//                                      RTC hour * 60 + RTC minute
//                     YearAndMonth = (RTC year + 2000) << 4 | RTC month
//  
// ********************************************************************************************************
void SysTimeToBcdTime(const RTC_Time* sys, uRTC_Bcd_Time* bcd)
{
    uint16_t temp;

    bcd->s.secs_bcd = BinToBcd(sys->mSecOfMinute/1000);

    temp = sys->MinuteOfMonth / 1440;
    bcd->s.date_bcd = BinToBcd(temp + 1);
    temp = sys->MinuteOfMonth % 1440;
    bcd->s.hours_bcd = BinToBcd(temp / 60);
    bcd->s.min_bcd = BinToBcd(temp % 60);

    bcd->s.year_bcd = BinToBcd((sys->YearAndMonth >> 4) - RTC_YEAR_OFFSET);
    bcd->s.month_bcd = BinToBcd((sys->YearAndMonth & 0x000F));

    bcd->s.day_bcd = 0;                 // unused
}

// ********************************************************************************************************
// *            END OF RTC.c
// ********************************************************************************************************
