#ifndef CRITICALSECTION_H_
#define CRITICALSECTION_H_

// ******************************************************************************************************
// *            CriticalSection.h
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton | Powerware 
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton | Powerware
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: CriticalSection.h
// *
// *    DESCRIPTION: Critical section management
// *
// *    ORIGINATOR: Jonathan Brandmeyer
// *
// *    DATE: 7/20/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************
#include "DSP2833x_Device.h"

/* Critical section management.  If some code wants to perform an atomic 
 * operation, then it may use this class, from any context, to safely disable
 * and restore interrupts.  Interrupts are restored automatically when this
 * class leaves scope.  You can create a fresh scope immediately prior to
 * instantiating this class for the purpose.  ex:
 * 
 * {
 *     CriticalSection enter;
 *     // do some very short operation
 * } // critical section exit.
 * 
 * See also the Boost.Threads and C++0x standard mutex lock objects design.
 */

#define IER_DISABLE_ALL             uint16_t( 0x0000u )      // disabled all interrupts
#define IER_DMA_ONLY                uint16_t( 0x0048u )      // disables all except DMA and eCaps
#define IER_DMA_PROC_ADC_ONLY       uint16_t( 0x004au )      // disables all except DMA and ProcessA2D

class CriticalSection
{
private:
    uint16_t savedIer;
    // Declare these prototypes to be private to force the class to be non-
    // copyable.  See also boost::noncopyable.
    CriticalSection(const CriticalSection& other);
    const CriticalSection& operator=(const CriticalSection& other);

public:
    /*
     * Save the interrupt enable status, and disable selected interrupts.
     */
    explicit CriticalSection( uint16_t mask = IER_DISABLE_ALL )
        : savedIer(IER)
    {
        IER = mask;
    }
    
    /*
     * Restore the interrupt enable status
     */
    ~CriticalSection()
    {
        IER = savedIer;
    }
};

#endif /*CRITICALSECTION_H_*/
