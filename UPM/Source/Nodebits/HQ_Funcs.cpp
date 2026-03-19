// ********************************************************************************************************
// *            HQ_Funcs.c
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
// *    FILE NAME: HQ_Funcs.c
// *
// *    DESCRIPTION: History Queue Functions.
// *
// *    ORIGINATORS: Costin Radoias
// *
// *    DATE: 5/30/2003
// *
// *    HISTORY: See Visual Source Safe history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILE
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Rtc.h"
#include "HQ_Funcs.h"
#include "Spi_Task.h"
#include "Constants.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "Queue.h"
#include "Eeprom_Map.h"
#include "InternalCan.h"
#include "IOexpansion.h"

// *********************************************************************************************************
// *        Data structures for the History Queue
// *********************************************************************************************************

const uint16_t HQ_QueueSize = 20;
const uint16_t HQ_EEP_START_ADDRESS = 501;            // eeprom location for the start of the history queue
const uint16_t HQ_LAST_EVENT_ADDRESS = 1991;          // address of the last entry
const uint16_t HQ_EEP_END_ADDRESS = 1996;             // history queue ends

const uint16_t HQ_MAX_NUM_EVENTS = ( ( HQ_EEP_END_ADDRESS - HQ_EEP_START_ADDRESS ) / sizeof( st_HQ_event ) );         

const uint16_t HQ_TRIGGERED_EEP_START_ADDRESS = 3601;
const uint16_t HQ_TRIGGERED_EEP_END_ADDRESS = 3699;
const uint16_t HQ_TRIGGERED_MAX_NUM_EVENTS = 
	( HQ_TRIGGERED_EEP_END_ADDRESS - HQ_TRIGGERED_EEP_START_ADDRESS ) 
		/ sizeof( st_HQ_event );

EventHistory HistoryQueue;
uint16_t HistoryTrigger;

// ********************************************************************************************************
// * GLOBAL VARIABLES
// ********************************************************************************************************

// ********************************************************************************************************
// * LOCAL FUNCTION PROTOTYPES
// ********************************************************************************************************

extern "C"
{
    void HistoryQueueTaskEntry( void );
}

// ***********************************************************************
// *
// *    FUNCTION: HistoryQueueTaskEntry 
// *
// *    DESCRIPTION: Task entry 
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: it doesn't
// *
// * Note: Normally a non-class member function should not go in a class definition file. However, BIOS is
// *       C-based and can't handle a class member as a task
// *
// ***********************************************************************
void HistoryQueueTaskEntry( void )
{
    HistoryQueue.MaintainHistoryQueue();
}    

// ********************************************************************************************************
// *
// * Function: HQ_MaintainHistoryQueue
// *
// * Purpose: History queue task function, queue initialization.
// *
// * Parms Passed   :   none.
// * 
// * Returns        :   it doesn't
// *
// ********************************************************************************************************
void EventHistory::MaintainHistoryQueue( void )
{
    uint16_t resend_attempts = 0;
    RTC_Time RTC_at_init;

    HQ_EventQueue = QueueCreate( HQ_QueueSize, sizeof( st_HQ_event ) );
    
    if ( HQ_EventQueue != NULL )
    {
        // wait for eeprom initialization
        while ( !EEStatusBits.bit.EEDataInitialized )
        {
            TSK_sleep(TSK_100_ms);    
        }
        
        // get log position
        getNumberOfEEPROMEvents(); 

        // RTC initialization
        if (RTC_SysTime.YearAndMonth == 0)
		{
            // Let's hope one of the UPM's will do the RTC request for us.
            // Wait for a while and see if anything happens. 
        	InternalCan.SendRTCrequest();	
            TSK_sleep(TSK_100_ms);
        
            // Request the RTC from CSB if we still don't have it. 
            // Try resending a few times if there is no response.
            while((RTC_SysTime.YearAndMonth == 0) &&				//need about 0.6~1.5s(so set 2.0s)
				  (resend_attempts < 10))
            {
				
                InternalCan.SendRTCrequest();
                TSK_sleep(TSK_200_ms); 
                resend_attempts++;    
            }
        }

        // "Estimate" what the time was at initialization.
        RTC_at_init = RTC_SysTime;
        if (RTC_at_init.mSecOfMinute > 100)
        {
            RTC_at_init.mSecOfMinute -= 100;
        }
        else if (RTC_at_init.MinuteOfMonth > 0)
        {
            RTC_at_init.MinuteOfMonth--;
            RTC_at_init.mSecOfMinute = 59900;
        }
        else
        {
            RTC_at_init.mSecOfMinute = 0;
        }

        // wait until TRAP info. has been read out, which will be recorded into UPM_NB_CONTROL_POWER_ON
        while( !PLDTrapReg.Ready)
        {
        	TSK_sleep(TSK_100_ms);
        }
        
        // event history is ready
        HQReady = true;
        
        // startup
        NB_DebounceAndQue( UPM_NB_CONTROL_POWER_ON, true, PLDTrapBoot & 0x0F );
        
    
        for ( ; ; )
        {
            st_HQ_event event;
            
            // wait for somebody to post something
            if ( QueueReceive( HQ_EventQueue, &event, SYS_FOREVER ) )
            {
                // Update the nodebits
                NB_Parse();

                // Check if someone managed to log something before the RTC was initialized
                // and the timestamp is garbage. In that case replace it with our own 
                // estimated timestamp.
                if ((event.timeStamp.YearAndMonth == 0) &&
                    (RTC_at_init.YearAndMonth != 0))
                {
                    event.timeStamp =  RTC_at_init;                   
                }
        
                // Queue the event in EEPROM
                moveRamEventToEEPROM( &event );
            } 
        }
    }    
}

// ********************************************************************************************************
// *
// * Function: QueueEvent
// *
// * Purpose: Queues one event into the RAM history queue
// *
// * Parms Passed   :   eventNum    : event to be Queued
// *                :   evData      : optional additional infor
// * Returns        :   Nothing
// *
// * Description    :   The function logs the event into the history queue, and adds a time stamp
// *
// ********************************************************************************************************
void EventHistory::QueueEvent( uint16_t eventNum, bool localOnly, uint16_t evData)
{   
    st_HQ_event event;
    
    event.event = eventNum;
    event.data = evData;
    event.timeStamp = RTC_SysTime;
    
    // Queue send will ignore block time is this is a HWI or SWI
    if( !QueueSend( HQ_EventQueue, &event, TSK_100_ms ) )
    {
        // if this is ever non-zero, need to think about increasing the queue size
        HQFullErrors++;
    }
    // Ship to the Internal CAN bus, too.
    if (!localOnly)
    {
    	InternalCan.SendHistory(event);
    }
}   // end of HQ_QueueEvent

// ********************************************************************************************************
// *
// * Function: GetEvent
// *
// * Purpose: Retrieves an event from the queue
// *
// * Parms Passed   :   eventNum    : event to be read, 0 = newest
// *                :   event       : buffer to place the event data
// * Returns        :   Nothing
// *
// ********************************************************************************************************
void EventHistory::GetEvent( uint16_t eventNum, st_HQ_event* event )
{
    uint16_t eventAddress;
    
    // check that event exists 
    if ( eventNum <= EventsInQueue )
    {
        if ( EventsInQueue < HQ_MAX_NUM_EVENTS )
        {
            // queue has not wrapped yet
            eventAddress = EEWritePointer - ( eventNum * sizeof( st_HQ_event ) );
        }
        else
        {
            // queue has wrapped, index from current position. indexed to 0, subtract 1 from max events
            // current pointer points to the oldest event, next is second oldest etc.
            eventAddress = ( ( HQ_MAX_NUM_EVENTS - 1 ) - eventNum ) * sizeof( st_HQ_event );
            eventAddress += EEWritePointer;
            // wrap to beginning if address is past last event
            if ( eventAddress > HQ_LAST_EVENT_ADDRESS )
            {
                eventAddress -= HQ_EEP_END_ADDRESS;
                eventAddress += HQ_EEP_START_ADDRESS;    
            }
        }
        
        // read eeprom
        GetEepData( eventAddress, sizeof( st_HQ_event ), (uint16_t*)event, TSK_1000_ms );
    }      
}

void EventHistory::GetTriggeredEvent( uint16_t eventNum, st_HQ_event* event )
{
    uint16_t eventAddress;
    
    // check that event exists
    if ( eventNum <= EventsInQueue && 
    	eventNum < HQ_TRIGGERED_MAX_NUM_EVENTS)
    {
    	eventAddress = HQ_TRIGGERED_EEP_START_ADDRESS + eventNum * sizeof( st_HQ_event);
    	
        // read eeprom
        GetEepData( eventAddress, sizeof( st_HQ_event ), (uint16_t*)event, TSK_1000_ms );
    }      
}


// ********************************************************************************************************
// *
// * Function: moveRamEventToEEPROM
// *
// * Purpose: moves one history event from Ram queue to Eeprom Queue
// *
// * Parms Passed   :   none.
// * 
// * Returns        :   none.
// *
// * Description    :   Copies an event to eeprom, adjusts the write pointer
// *
// ********************************************************************************************************
void EventHistory::moveRamEventToEEPROM( st_HQ_event* event )
{
    // erase queue event?
    if ( event->event == HQ_ERASE_ALL_EVENTS )
    {
        clearEEPROMEvents();
    }
    else
    {    
        // see if this is a queued event or not
        uint16_t NB_idx = ( event->event & ~HQ_EVENT_IS_TRUE );
        
        if ( NB_idx < UPM_NB_PRIVATE_SUPPORTED_EVENTS )
        {
        	uint16_t QueueControl = NB_Cfg_Flash[NB_idx].bit.QueueControl & QUE_ALL;
            if ( ( QUE_ALL == QueueControl ) ||
                 ( ( QUE_ACTIVE   == QueueControl ) &&  ( event->event & HQ_EVENT_IS_TRUE ) ) ||
                 ( ( QUE_INACTIVE == QueueControl ) && !( event->event & HQ_EVENT_IS_TRUE ) ) )
            {     
                // save if active or not active
                bool eventIsTrue;
                if ( event->event & HQ_EVENT_IS_TRUE )
                {
                    eventIsTrue = true;
                }
                else
                {
                    eventIsTrue = false;
                }        
                
                bool triggeringEvent = !HQTriggered && HistoryTrigger == NB_idx;
                
                // replace the NB index with the XCP alarm number
                event->event = NB_Cfg_Flash[ NB_idx ].bit.XCPAlarmNumber;
                // add the mask
                if ( eventIsTrue )
                {
                    event->event |= HQ_EVENT_IS_TRUE;
                }    
                // erase the Ptr+1
                uint16_t tempPtr = EEWritePointer + sizeof( st_HQ_event );
                if ( tempPtr >= ( HQ_EEP_START_ADDRESS + ( HQ_MAX_NUM_EVENTS * sizeof( st_HQ_event ) ) ) )
                {
                    tempPtr = HQ_EEP_START_ADDRESS;
                }
                EraseEepData( tempPtr, sizeof( st_HQ_event ), TSK_1000_ms );     
                
                // try to write event to eeprom
                if ( PutEepData( EEWritePointer, sizeof( st_HQ_event ), (uint16_t*)event, TSK_1000_ms ) ) 
                {
                    // If successfully written event to eeprom,
                    EEWritePointer += sizeof( st_HQ_event );                    // adjust the write pointer
                    // wrap the pointer if necessary
                    if ( EEWritePointer >= ( HQ_EEP_START_ADDRESS + ( HQ_MAX_NUM_EVENTS * sizeof( st_HQ_event ) ) ) )
                    {
                        EEWritePointer = HQ_EEP_START_ADDRESS;
                    }
                    // increment number of events if not full
                    if ( EventsInQueue < HQ_MAX_NUM_EVENTS )
                    {
                        ++EventsInQueue;
                    }    
                }
                
                if (triggeringEvent)
                {
                	HQTriggered = true;
                	// Copy this event and the previous 20 to the auxiliary history
                	// queue
                	uint16_t writePtr = HQ_TRIGGERED_EEP_START_ADDRESS;
					uint16_t nEvents = GetNumEvents();
					
                	for (uint16_t i = 1;
                		i < HQ_TRIGGERED_MAX_NUM_EVENTS+1 && i < nEvents+1;
                		++i, writePtr += sizeof( st_HQ_event ))
                	{
                		GetEvent(i, event);
	           		    EraseEepData( writePtr, sizeof( st_HQ_event ), TSK_1000_ms );  
                		PutEepData(writePtr, sizeof(st_HQ_event), (uint16_t*)event, TSK_1000_ms);
                	}
                }
            }    
        }    
    }    
}


// ********************************************************************************************************
// *
// * Function: clearEEPROMEvents
// *
// * Purpose: erases entire Eeprom history queue
// *
// * Parms Passed   :   none.
// * 
// * Returns        :   none.
// *
// * Description    :   This function erases the Eeprom History Queue.
// *
// ********************************************************************************************************
void EventHistory::clearEEPROMEvents( void )
{
    EventsInQueue = 0; 
    disableSPIQueue=true;	
    (void)EraseEepData( HQ_EEP_START_ADDRESS, ( HQ_MAX_NUM_EVENTS * sizeof( st_HQ_event ) ), TSK_1000_ms ); 
    EEWritePointer = HQ_EEP_START_ADDRESS;
    
    EraseEepData( HQ_TRIGGERED_EEP_START_ADDRESS, ( HQ_TRIGGERED_MAX_NUM_EVENTS * sizeof( st_HQ_event ) ), TSK_1000_ms );
    disableSPIQueue=false;
    HQTriggered = false;
}

// ***********************************************************************
// *
// *    FUNCTION: getNumberOfEEPROMEvents 
// *
// *    DESCRIPTION: read the current eeprom queue pointer. If it's a 
// *                 valid event, assume the ee queue has wrapped and
// *                 contains the max number of events. It it's invalid,
// *                 then ( ee_pointer - 1 ) is assumed to be the latest event 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void EventHistory::getNumberOfEEPROMEvents( void )
{
    st_HQ_event event = {0};

    // start at beginning, find the first 0xffff event
    EEWritePointer = HQ_EEP_START_ADDRESS;
    
    while ( EEWritePointer < HQ_EEP_END_ADDRESS )
    {
        // read the current write address
        (void)GetEepData( EEWritePointer, sizeof( st_HQ_event ), (uint16_t*)&event, TSK_1000_ms );
        
        if ( 0xffff == event.event )
        {
            break;
        }
        else
        {
            EEWritePointer += sizeof( st_HQ_event );
        }
    }
    
    GetEepData( HQ_TRIGGERED_EEP_START_ADDRESS, sizeof( st_HQ_event), (uint16_t*)&event, TSK_1000_ms);
    if (event.event != 0xffffu)
    {
    	HQTriggered = true;
    }
    else
    {
    	HQTriggered = false;
    }
    
    // check for valid pointer
    if ( EEWritePointer < HQ_EEP_END_ADDRESS )
    {            
        // check for wrap, read the next one
        uint16_t tempPtr = EEWritePointer + sizeof( st_HQ_event );
        
        if ( tempPtr < HQ_EEP_END_ADDRESS )
        {
            (void)GetEepData( tempPtr, sizeof( st_HQ_event ), (uint16_t*)&event, TSK_1000_ms );
            
            if ( 0xffff == event.event )
            {
                EventsInQueue = ( EEWritePointer - HQ_EEP_START_ADDRESS ) / sizeof( st_HQ_event );    
            }
            else
            {
                EventsInQueue = HQ_MAX_NUM_EVENTS;    
            }
        }
        else
        {
            EventsInQueue = HQ_MAX_NUM_EVENTS - 1;    
        }
    }
    else
    {        
        // somethings hosed, reset
        EEWritePointer = HQ_EEP_START_ADDRESS;
        EventsInQueue = 0;
    }    
}

// ********************************************************************************************************
// *            No more
// ********************************************************************************************************







