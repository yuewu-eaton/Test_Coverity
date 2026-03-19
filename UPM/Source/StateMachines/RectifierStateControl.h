#ifndef _RECT_STATE_H
#define _RECT_STATE_H
// *****************************************************************************
// *            RectState.h
// *****************************************************************************
// *****************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON
// *
// *****************************************************************************
// *
// *  Copyright (c) 2010 Eaton corp
// *    ALL RIGHTS RESERVED
// *
// *****************************************************************************
// *****************************************************************************
// *    FILE NAME: RectState.h
// *
// *    DESCRIPTION:
// *
// *    ORIGINATOR: Hans Pfitzer
// *
// *    DATE: 5/13/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *****************************************************************************

// *****************************************************************************
// *        INCLUDE FILES
// *****************************************************************************
#include "RectifierControl.h"
#include "StateTimer.h"

// *****************************************************************************
// *        Public Data
// *****************************************************************************

/*** DEFINE THE MACHINE STATES ****/
enum rectifier_states_t
{
    RECTIFIER_INIT_STATE            =           0,
    RECTIFIER_SHUTDOWN_STATE        =           1,
    RECTIFIER_PRECHARGE_STATE       =           2,
    RECTIFIER_NORMAL_STATE          =           3,
    RECTIFIER_TO_BATTERY_STATE      =           4,
    RECTIFIER_ON_BATTERY_STATE      =           5,
    RECTIFIER_FROM_BATTERY_STATE    =           6,
    RECTIFIER_WALKIN_STATE          =           7,
    RECTIFIER_SUSPEND_STATE         =           8,
    RECTIFIER_FROM_SUSPEND_STATE    =           9
};

#define RECT_STATE_MACHINE_RUN_FREQUENCY    (float)( 2165 )
#define RECT_SAMPLE_PERIOD_US               (uint32_t)( 462 )

#define RECT_DELAY_1MS                      uint32_t( 1000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_2MS                      uint32_t( 2000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_4MS                      uint32_t( 4000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_5MS                      uint32_t( 5000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_8MS                     uint32_t( 8000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_10MS                     uint32_t( 10000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_14MS                     uint32_t( 14000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_20MS                     uint32_t( 20000L / RECT_SAMPLE_PERIOD_US )
#define RECT_DELAY_50MS                     uint32_t( 5L * RECT_DELAY_10MS )
#define RECT_DELAY_100MS                    uint32_t( 10L * RECT_DELAY_10MS )
#define RECT_DELAY_200MS                    uint32_t( 20L * RECT_DELAY_10MS )
#define RECT_DELAY_500MS                    uint32_t( 50L * RECT_DELAY_10MS )
#define RECT_DELAY_1000MS                   uint32_t( 100L * RECT_DELAY_10MS )
#define RECT_DELAY_2000MS                   uint32_t( 200L * RECT_DELAY_10MS )
#define RECT_DELAY_3000MS                   uint32_t( 300L * RECT_DELAY_10MS )
#define RECT_DELAY_5000MS                   uint32_t( 500L * RECT_DELAY_10MS )
#define RECT_DELAY_5_SEC                    uint32_t( 5L * RECT_DELAY_1000MS )
#define RECT_DELAY_10_SEC                   uint32_t( 10L * RECT_DELAY_1000MS )
#define RECT_DELAY_15_SEC                   uint32_t( 15L * RECT_DELAY_1000MS )
#define RECT_DELAY_19_SEC                   uint32_t( 19L * RECT_DELAY_1000MS )
#define RECT_DELAY_30_SEC                   uint32_t( 30L * RECT_DELAY_1000MS )

#define RECT_DELAY_RELAY                    RECT_DELAY_10MS  // RECT_DELAY_20MS
#define RecBatInv_Delay_Relay_Open         	RECT_DELAY_8MS  //follow new hw define: rec/bat/inv same relay- 14ms close/8ms open
#define RecBatInv_Delay_Relay_Close         RECT_DELAY_14MS  //
#define REC_100MS_BASE_20MS                 uint32_t(5 * RECT_DELAY_20MS)

#define RECT_DELAY_FAILURE_TIME             ( RECT_DELAY_5_SEC )

#define RECTIFIER_INHIBIT_DECREMENT_TIMEOUT  300     // 30 * 10 * 100ms = 30 seconds
#define RECTIFIER_INHIBIT_TRANSFER_LIMIT     3      // number of transfers before delaying normal transfer  Keming/20130227
#define RECTIFIER_INHIBIT_TIME               1200     // 1200 * 10 * 100ms = 2 minutes
#define DCLINK_780_VOLT                      ((float)780.0)    //MAX DC linke voltage for HV is 780V
#define DCLINK_790_VOLT                      ((float)790.0)    //MAX DC linke voltage for HV is 790V
#define DCLINK_810_VOLT                      ((float)810.0)    //MAX DC linke voltage for HV is 810V
class RectifierStateControl : public RectifierControl
{
    public:
        RectifierStateControl( void ) : RectifierControl(),
            PrechargeTimer(RECT_DELAY_10_SEC)
        {
            RectMachineState = RECTIFIER_INIT_STATE;
            RectifierPhState = 0;
            BatteryTransferCounter = 0;
            RectifierInhibitted = 0;
            RectifierInhibitTimer.ClearTimer();
        }
        ~RectifierStateControl( void )
        {
        }
        
    public:
        void RunRectiferState( void );
        bool CheckRectifierFailure( void );
        bool CheckDelayedRectifierFailure( void );
        void Rectifier100msTask( void );
        void ResetRectifierAlarms( void );
        
        rectifier_states_t GetState( void )
        {
            return RectMachineState;
        }

        void RectifierSuspend( void )
        {
            RectifierStatus.bit.GoToSuspendState = 1;
        }
        
        void StartBatteryCycleTest(void)
        {
            RectifierStatus.bit.BatteryNormalCycleTest = 1;
        }

        void StopBatteryCycleTest(void)
        {
            RectifierStatus.bit.BatteryNormalCycleTest = 0;
        }
		
        bool PrechargerReady( void )
        {   
            return PrechargeTimer.TimerValue() >= RECT_DELAY_10_SEC; 
        }
        
        void SetGLUtilityNotPresent(bool state)
        {
            RectifierStatus.bit.GLUtilityNotPresent = state;
        }

        void ResetSiteWiringFault( void )
        {
            RectifierStatus.bit.SiteWiringLocked = 0;
        }
		
        void EnableLegBTest( void )
		{
        	ReuseRectSTPWMEnable = 1;
		}

        void DisableLegBTest( void )
		{
        	ReuseRectSTPWMEnable = 0;
		}

    public:
        bool    FuseFailRestart;
        uint16_t FuseFailRestartTimes;
        uint16_t UtilityStable_OnBattery;
		uint16_t BoostReuseRectST;
		uint16_t ReuseRectSTPWMEnable;

        bool InputAvailable( void );
		bool RectifierOnReal;

    protected:
        void RectInitializingState( void );
        void RectShutdownState( void );
        void RectPrechargingState( void);
        inline void RectOnNormalState( void );
        inline void RectToBatteryState( void );
        inline void RectOnBatteryState( void );
        inline void RectFromBatteryState( void );
        inline void RectWalkinState( void );
        inline void RectSuspendState( void );
        inline void RectFromSuspendState( void );
        void RectTransferState( rectifier_states_t new_state );

    protected:
        rectifier_states_t RectMachineState;
        uint16_t RectifierPhState;
        uint16_t BatteryTransferCounter;
        uint16_t RectifierInhibitted;
        uint16_t PrechargeAttempts;

        StateTimer RectifierTimer1;
        StateTimer RectifierTimer2;
        StateTimer RectifierInhibitTimer;
        /*
         * Prevents restarting the precharge until after 10 seconds of idle time.
         */
        StateTimer PrechargeTimer;
        
        StateTimer DelayedFailureTimer;
    private:
        // Override the global version with local behavior
        inline void PreChargeOff();
};

extern RectifierStateControl Rectifier;

inline void ee_update_utility_sync( EE_ID* ee, uint16_t* data )
{
    Rectifier.UtilityPLL.SineRef.EEFunc_Sync( ee, data );
}
// *****************************************************************************
// *            END OF RectState.h
// *****************************************************************************
#endif

