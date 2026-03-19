// ******************************************************************************************************
// *            Queue.h
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
// *    FILE NAME: Queue.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR: Kevin VanEyll
// *
// *    DATE: 2/28/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void * QueueHandle;

//
// * Creates the queue, uses malloc. Queue must be created before it can be used.
//

QueueHandle QueueCreate( uint16_t queueLength, size_t itemSize );

//
// * QueueSend and QueueRecieve are context aware, they will not block if called from a SWI or HWI.
// * However that means that if called from an interrupt, it must be a BIOS-enabled interrupt, or HWI.
// * In other words it must use the dispatcher.
//

uint16_t QueueSend( QueueHandle const queue, const void * const itemToQueue, uint16_t ticksToWait );
uint16_t QueueReceive( QueueHandle const queue, void * const buffer, uint16_t ticksToWait );

#ifdef __cplusplus
}
#endif

#endif
// ******************************************************************************************************
// *            End of Queue.h
// ******************************************************************************************************
