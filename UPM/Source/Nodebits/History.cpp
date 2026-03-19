// ******************************************************************************************************
// *            History.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: History.cpp
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 3/2/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "Constants.h"
#include "Rtc.h"
#include "HQ_Funcs.h"
#include "NB_Funcs.h"
#include "NB_Config.h"
#include "History.h"
#include "Version.h"
#include "XCPDefs.h"

#include <cstring>
#include <cstdio>

const char* const hqNotReadyStr     = {"The history queue is not ready"};
const char* const hqNotTriggeredStr = {"There is no triggered log\r\f"};
const char* const hqTriggeredStr    = {"Triggered history log:\r\f"};
const char* const hqEmptyStr        = {"There are no events in the queue"};
const char* const idStr             = {"Panda control\r\f"};                // temp, until we get a real one
const char* const fwVersionStr      = {"Firmware Version: "};
const char* const alarmStr          = {"Alarm #"};
const char* const noticeStr         = {"Notice #"};
const char* const statusStr         = {"Status #"};
const char* const clearStr          = {"CLEAR:"};

const char* GetEventText( uint16_t strNum );

// ***********************************************************************
// *
// *    FUNCTION: LogStart 
// *
// *    DESCRIPTION: Entry point
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void History::LogStart( void )
{
    TextBuffer = new char[ LogTextBufferSize ];
    
    if ( NULL != TextBuffer )
    {
        ClrScrn();
        SetCursor( 0, 0 );
        PrintUPSID();
        
        // check to see if the queue is ready to be read
        if ( !HistoryQueue.IsQueueReady() )
        {
            // give them the bad news...
            PutTxString( hqNotReadyStr );
        }
        else
        {
            LogMain();
        }
        
        if (HistoryQueue.HasTriggeredEvents() )
        {
        	PutTxString( hqTriggeredStr);
        	int numEvents = 20;
            st_HQ_event event;
            std::memset(&event, 0xff, sizeof(event));
	        do
	        {
	            HistoryQueue.GetTriggeredEvent( numEvents, &event );
	            if (event.event != 0xffffu)
	        	{
	            	PrintEvent( &event, numEvents );
	            	--numEvents;
	        	}    
	        } while ( numEvents >= 0 && event.event != 0xffffu);
        }
        else
        {
        	PutTxString( hqNotTriggeredStr);
        }
    }
    delete [] TextBuffer;
    TextBuffer = NULL;
}

// ***********************************************************************
// *
// *    FUNCTION: LogMain 
// *
// *    DESCRIPTION: Entry point
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void History::LogMain( void )
{
    // print current events
    int16_t numEvents = (int16_t)HistoryQueue.GetNumEvents();
    
    if ( 0 == numEvents )
    {
        PutTxString( hqEmptyStr );
    }
    else
    {    
        do
        {
            st_HQ_event event = {0};
            HistoryQueue.GetEvent( numEvents, &event );
            PrintEvent( &event, numEvents );
            --numEvents;    
        } while ( numEvents >= 0 );
        
    }
}

// ***********************************************************************
// *
// *    FUNCTION: PrintUPSID 
// *
// *    DESCRIPTION: Not much to go in here yet 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void History::PrintUPSID( void )
{
    PutTxString( idStr );
    SYS_sprintf( TextBuffer, "%s%02x.%02x.%04u\r\f\r\f", 
    	fwVersionStr, 
    	FirmwareVersion >> 8,
    	FirmwareVersion & 0xff, 
    	FirmwareBuildNum );
    PutTxString( TextBuffer ); 
}

// ***********************************************************************
// *
// *    FUNCTION: PrintEvent 
// *
// *    DESCRIPTION: sends the event string to the port 
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void History::PrintEvent( const st_HQ_event* event, uint16_t num )
{
    // display: Number: (Alarm/Notice/Status)# NB_number (CLEAR:) NB_text
    // get index to nodebit table
    uint16_t NB_idx = GetNodeBitArrayIndex( ( event->event & ~HQ_EVENT_IS_TRUE ) );
    
    if ( NB_idx < GetNumDefinedNB() )
    {
        const st_NB_Cfg* nb = &NB_Cfg_Flash[ NB_idx ];
        const char* level = NULL;
        RTC_FormattedTime fTime;
        
        // get event level string
        if ( nb->bit.XCPAlarmLevel < EVENT_LEVEL_NOTICE )
        {
            level = statusStr; 
        }
        else if ( nb->bit.XCPAlarmLevel < EVENT_LEVEL_ALARM )
        {
            level = noticeStr;    
        }
        else
        {
            level = alarmStr;
        }
        
        // print number
        SYS_sprintf( TextBuffer, "%d:\t", num );
        PutTxString( TextBuffer );
        // convert timestamp
        RTC_FormatTime( &(event->timeStamp ), &fTime );
        
        // American format, sorry HPO and STK...
        SYS_sprintf( TextBuffer, "%02u/%02u/%04u %02u:%02u:%02u.%03u\t", 
        	fTime.month, fTime.day, fTime.year, 
        	fTime.hour, fTime.minute, fTime.second, fTime.mSec );
        PutTxString( TextBuffer ); 
        
        // alarm/notice/status + number:
        SYS_sprintf( TextBuffer, "%s%u\t", level, nb->bit.XCPAlarmNumber );
        PutTxString( TextBuffer );
        
        if ( 0 == ( event->event & HQ_EVENT_IS_TRUE ) )
        {
            PutTxString( clearStr );
        } 
        
        // get the string
        level = GetEventText( nb->bit.XCPAlarmNumber );
        
        if ( NULL != level )
        {
            PutTxString( level );
        }    

        if ( event->data != 0 )
        {
            // print data
            SYS_sprintf( TextBuffer, " %d\t", event->data );
            PutTxString( TextBuffer );
        }
        
        PutTxChar( CR );
        PutTxChar( LF );
            
    }
}

// ******************************************************************************************************
// *            End of HistoryLog.cpp
// ******************************************************************************************************
