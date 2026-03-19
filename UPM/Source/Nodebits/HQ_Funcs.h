// ********************************************************************************************************
// *            HQ_Funcs.h
// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2003 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: HQ_Funcs.h
// *
// *    DESCRIPTION: Structure and Prototypes for all public History Queue functions.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 5/30/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

#ifndef _HQ_FUNCS_H
#define _HQ_FUNCS_H

#include "Rtc.h"
#include "Queue.h"

struct st_HQ_event
{
    uint16_t    event;                          // event being logged
    uint16_t    data;                           // optional additional info
    RTC_Time    timeStamp;                      // entire SysTime structure 
};

// *********************************************************************************************************
// *        Defines
// *********************************************************************************************************
#define HQ_EVENT_IS_TRUE                    0x8000              // Event MSB = 1 to indicate event is True
#define HQ_EVENT_IS_FALSE                   0x0000              // Event MSB = 0 to indicate event is False

#define HQ_ERASE_ALL_EVENTS                 0xffff              // Request to Erase the history queue

// *********************************************************************************************************
// *        Public Data
// *********************************************************************************************************

class EventHistory
{
    public:
        EventHistory()
        {
        	HQTriggered = false;
            EEWritePointer = 0;
            HQFullErrors = 0;
            HQReady = false;
        }
        ~EventHistory()
        {
        }

    public:
        void MaintainHistoryQueue( void );
        void QueueEvent( uint16_t eventNum, bool localOnly = false, uint16_t evData = 0 );
        void GetEvent( uint16_t eventNum, st_HQ_event* event ); 
        void GetTriggeredEvent( uint16_t eventNum, st_HQ_event* event);
        
        uint16_t GetNumEvents( void )
        {
            return EventsInQueue;
        }
        bool IsQueueReady( void )
        {
            return HQReady;
        }
        void EraseAllEvents( void )
        {
            QueueEvent( HQ_ERASE_ALL_EVENTS, true );    //Jacob/20130916/add true, GPE-1267
        }
		bool HasTriggeredEvents( void)
		{
			return HQTriggered;
		}
		
        void clearEEPROMEvents( void );

    protected:
        void moveRamEventToEEPROM( st_HQ_event* event );
        void getNumberOfEEPROMEvents( void );
        
    public:
        uint16_t        HQFullErrors;           // for debugging
           
    protected:
        uint16_t        EEWritePointer;
        uint16_t        EventsInQueue;
        QueueHandle     HQ_EventQueue;
        bool            HQReady;
        bool            HQTriggered;
};

extern EventHistory HistoryQueue;
extern uint16_t HistoryTrigger;

// ********************************************************************************************************
// *            END OF HQ_Funcs.h
// ********************************************************************************************************
#endif

