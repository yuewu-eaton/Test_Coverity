// ******************************************************************************************************
// *            Queue.c
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
// *    FILE NAME: Queue.c
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll, borrowing liberally from FreeRTOS source code
// *
// *    DATE: 2/24/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "F28335Port.h"
#include <stdint.h>

extern int16_t heap1;

typedef struct QueueDefinition
{
    uint16_t*             Head;                  // Points to the beginning of the queue 
    uint16_t*             Tail;                  // Points to the last queue entry

    // WriteTo and ReadFrom do not need volatile protection because they are only called
    // within this file, while interrupts are disabled
    uint16_t*             WriteTo;               // Points to the free next place in the storage area
    uint16_t*             ReadFrom;              // Points to the last place that a queued item was read from

    volatile uint16_t     MessagesWaiting;       // The number of items currently in the queue
    uint16_t              Length;                // The length of the queue defined as the number of items it will hold, not the number of words
    size_t                ItemSize;              // The size of each item that the queue will hold

} QUEUE;

//  * Inside this file QueueHandle is a pointer to a QUEUE structure.
//  * To keep the definition private the API header file defines it as a
//  * pointer to void.

typedef QUEUE* QueueHandle;

// local prototypes
static void CopyDataToQueue( QUEUE* const queue, const void* const itemToQueue );
static void CopyDataFromQueue( QUEUE* const queue, void* buffer );


// ***********************************************************************
// *
// *    FUNCTION: Creates a new queue. Allocates the storage for the new queue  
// *
// *    DESCRIPTION: 
// *
// *    ARGUMENTS: queueLength: the number of items the queue holds
// *               itemSize: the size of each item. Each item must be the same size.
// *                         queue items are stored by copy, not by reference.
// *
// *    RETURNS: queue handle or NULL if no memory 
// *
// ***********************************************************************
QueueHandle QueueCreate( uint16_t queueLength, size_t itemSize )
{
    QUEUE* newQueue;
    size_t queueSizeInMAUs;         // typically would be bytes, but is words in this case

    // Allocate the new queue structure
    if( queueLength > (size_t)0 )
    {
        newQueue = (QUEUE*)MEM_alloc( heap1, sizeof(QUEUE), 0 );
        if( newQueue != NULL )
        {
            // Create the actual queue, make it 1 larger than requested to make wrap checking easier
            queueSizeInMAUs = ( ( size_t )( queueLength * itemSize ) ) + ( size_t )1;

            newQueue->Head = ( uint16_t * ) MEM_alloc( heap1, queueSizeInMAUs, 0 );
            if( newQueue->Head != NULL )
            {
                // Initialize the queue members as described above
                newQueue->Tail = newQueue->Head + ( queueLength * itemSize );
                newQueue->MessagesWaiting = 0;
                newQueue->WriteTo = newQueue->Head;
                newQueue->ReadFrom = newQueue->Head + ( ( queueLength  - 1 ) * itemSize );
                newQueue->Length = queueLength;
                newQueue->ItemSize = itemSize;
                return  newQueue;
            }
            else
            {
                MEM_free( heap1, newQueue, sizeof( QUEUE ) );
            }
        }
    }

    // Will only reach here if we could not allocate enough memory or no memory
    // was required.
    return NULL;
}

// ***********************************************************************
// *
// *    FUNCTION: QueueSend  
// *
// *    DESCRIPTION: Writes an item to the queue if there is room. What happens 
// *                 if the queue is full depends on current context. If called from
// *                 a task, and ticksToWait != 0, the function will block until timeout
// *                 or until the queue is not full. If called from a HWI or SWI the write
// *                 fails and the function returns. 
// *
// *    ARGUMENTS: queue: handle to the queue to write to
// *               itemToQueue: data to write the the queue, the function will write "itemSize" words of data,
// *                            itemSize passed when the queue was created.
// *               tickToWait: timeout time for tasks. Ignored for HWI or SWI. if ticksToWait == SYS_FOREVER,
// *                           this function will not return until the item is posted to the queue.
// *
// *    RETURNS: true if the item was successfully written, false if not
// *
// ***********************************************************************
#pragma CODE_SECTION( QueueSend, "ramfuncs" );
uint16_t QueueSend( QUEUE* const queue, const void * const itemToQueue, uint16_t ticksToWait )
{
    uint16_t success = FALSE;
    uint16_t savedStatusRegister;

    // check for valid queue
    if ( NULL != queue )
    {
        // get current context task or SWI/HWI?
        if ( !TSK_isTSK() )
        {
            // called from HWI or SWI, no blocking allowed
            // disable interrupts and save the status register, so that intm state will not be changed
            // by this function
            savedStatusRegister = DSP28x_DisableInt();
            
            // is there room in the queue?
            if ( queue->MessagesWaiting < queue->Length )
            {
                // there's room so copy the data
                CopyDataToQueue( queue, itemToQueue );
                success = TRUE;
            }
            // restore global interrupt enable
            DSP28x_RestoreInt( savedStatusRegister );

        }
        else
        {
            // current context is task, so blocking is allowed. Also normal DINT/EINT is OK to use, since 
            // no task should be calling a blocking function with interrupts disabled...
            do
            {
                DINT;
                // check for room in the queue
                if ( queue->MessagesWaiting == queue->Length )
                {
                    // queue is full, enable interrupts and go to sleep
                    EINT;                                                   // enable 1
                    if ( ticksToWait != 0 )
                    {
                        // deliberately using '1' here instead of TSK_x_ms, want the smallest delay allowed, always = 1.
                        TSK_sleep( 1 );
                        // decrement ticks to wait 
                        if ( ticksToWait != SYS_FOREVER )
                        {
                            --ticksToWait;
                        }
                    }
                }
                else
                {
                    // there's room in the queue
                    CopyDataToQueue( queue, itemToQueue );
                    EINT;                                                   // enable 2
                    success = TRUE;
                    ticksToWait = 0;
                }
            } while ( ticksToWait != 0 ); 
        }
    }
    return success;
}

// ***********************************************************************
// *
// *    FUNCTION: QueueReceive  
// *
// *    DESCRIPTION: Reads an item from the queue if there is any. What happens 
// *                 if the queue is empty depends on current context. If called from
// *                 a task, and ticksToWait != 0, the function will block until timeout
// *                 or until the queue is not empty. If called from a HWI or SWI the read
// *                 fails and the function returns. 
// *
// *    ARGUMENTS: queue: handle to the queue to read from
// *               buffer: data to write queue item to, the function will write "itemSize" words of data,
// *                       itemSize passed when the queue was created.
// *               tickToWait: timeout time for tasks. Ignored for HWI or SWI. if ticksToWait == SYS_FOREVER,
// *                           this function will not return until an item is posted to the queue.
// *
// *    RETURNS: true if the item was successfully removed fromt the queue, false if not
// *
// ***********************************************************************
uint16_t QueueReceive( QUEUE* const queue, void * const buffer, uint16_t ticksToWait )
{
    uint16_t success = FALSE;
    uint16_t savedStatusRegister;

    // check for valid queue
    if ( NULL != queue )
    {
        // get current context task or SWI/HWI?
        if ( !TSK_isTSK() )
        {
            // called from HWI or SWI, no blocking allowed
            // disable interrupts and save the status register, so that intm state will not be changed
            // by this function
            savedStatusRegister = DSP28x_DisableInt();
            
            // are there any messages in the queue?
            if ( queue->MessagesWaiting > 0 )
            {
                // retrieve the queue item
                CopyDataFromQueue( queue, buffer );
                success = TRUE;
            }
            // restore intm
            DSP28x_RestoreInt( savedStatusRegister );
        }
        else
        {
            // current context is task, so blocking is allowed. Also normal DINT/EINT is OK to use, since 
            // no task should be calling a blocking function with interrupts disabled...
            do
            {
                DINT;
                // check for messages in the queue
                if ( queue->MessagesWaiting == 0 )
                {
                    // queue is empty, enable interrupts and go to sleep
                    EINT;                                                   // enable 1
                    if ( ticksToWait != 0 )
                    {
                        TSK_sleep( 1 );
                        // decrement ticks to wait 
                        if ( ticksToWait != SYS_FOREVER )
                        {
                            --ticksToWait;
                        }
                    }
                }
                else
                {
                    // retrieve the queue item
                    CopyDataFromQueue( queue, buffer );
                    EINT;                                                   // enable 2
                    success = TRUE;
                    ticksToWait = 0;
                }
            } while ( ticksToWait != 0 ); 
        }
    }

    return success;

}

// ***********************************************************************
// *
// *    FUNCTION: CopyDataToQueue 
// *
// *    DESCRIPTION: Copies item to queue 
// *
// *    ARGUMENTS: queue: queue to write to
// *               itemToQueue: data to write to the queue
// *
// *    RETURNS: 
// *
// ***********************************************************************
#pragma CODE_SECTION( CopyDataToQueue, "ramfuncs" );
static void CopyDataToQueue( QUEUE* const queue, const void* const itemToQueue )
{
    // copy the data to the write pointer
    memcpy( queue->WriteTo, itemToQueue, queue->ItemSize );
    // increment the write pointer, wrapping if necessary
    queue->WriteTo += queue->ItemSize;
    if ( queue->WriteTo >= queue->Tail )
    {
        queue->WriteTo = queue->Head;
    }
    // one message added to the queue
    ++( queue->MessagesWaiting );
}

// ***********************************************************************
// *
// *    FUNCTION: CopyDataFromQueue 
// *
// *    DESCRIPTION: Copies item from queue 
// *
// *    ARGUMENTS: queue: queue to read from
// *               buffer: data buffer for queue data
// *
// *    RETURNS: 
// *
// ***********************************************************************
static void CopyDataFromQueue( QUEUE* const queue, void* buffer )
{
    // increment the read pointer, wrapping if necessary
    queue->ReadFrom += queue->ItemSize;
    if ( queue->ReadFrom >= queue->Tail )
    {
        queue->ReadFrom = queue->Head;
    }
    // copy data from the read pointer to the buffer
    memcpy( buffer, queue->ReadFrom, queue->ItemSize );
    
    // one message removed from the queue
    --( queue->MessagesWaiting );
}



// ******************************************************************************************************
// *            End of Queue.c
// ******************************************************************************************************
