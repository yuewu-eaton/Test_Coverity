#ifndef ECANDRIVER_H_
#define ECANDRIVER_H_

#include "DSP2833x_ECan.h"
#include "Queue.h"
#include <sem.h>

#define CAN_DRIVER_MAX_BOXES 32

class CanDriver
{
// Public types
public:
	enum CanDriverId
	{
		ECANA,
		ECANB
	};
	
	/*
	 * The type of a function that can handle received CAN packets.
	 */
    typedef void (*RxPacketHandler)(const MBOX& message, const void *closure);
	
	/*
	 * The type of a message receiver handle, which may be used to ignore rx
	 * packets after registering for reception of them.
	 */
	typedef int16_t RxReceiverHandle;
	
	enum State
	{
		Initialization,
		Running, // Also error active
		Passive // Error Passive state.
	};

// Private types and typedefs
private:
	State state;
	
	struct TxPacket : public MBOX
	{
		TxPacket()
		{
		}
		
		TxPacket(const MBOX& base, bool isPeriodic)
			: isPeriodic(isPeriodic)
		{
	    	MSGID.all = base.MSGID.all;
	    	MSGCTRL.all = base.MSGCTRL.all;
	    	MDL.all = base.MDL.all;
	    	MDH.all = base.MDH.all;
		}
		
		bool isPeriodic;
	};
	
	struct RxPacket : public MBOX
	{
		RxPacket()
		{
		}
		
		#pragma CODE_SECTION("ramfuncs")
		inline RxPacket(const volatile MBOX* base, uint16_t mailboxId)
			: mailboxId(mailboxId)
		{
			MDL.all = base->MDL.all;
			MDH.all = base->MDH.all;
			MSGID.all = base->MSGID.all;
			MSGCTRL.all = base->MSGCTRL.all;
		}
		
		uint16_t mailboxId;
	};
	
// Inheritable data
protected:
	volatile ECAN_REGS *Regs;
	volatile ECAN_MBOXES *Mboxes;
	volatile LAM_REGS *LAMRegs;
	volatile MOTO_REGS *MOTORegs;
	volatile MOTS_REGS *MOTSRegs;
	
// Non-inheritable data
private:
	/* The number of transmit mailboxes that have been configured for message
	 * transmission.  All others are available for reception.  The bottom
	 * TxMailboxCount mailboxes get configured (from 32-TxMailboxCount to 32) as
	 * transmit boxes.
	 */
	const size_t TxMailboxCount;
	
	/*
	 * A bitmap of those transmit mailboxes that are currently filled with
	 * periodic packets.
	 */
	uint32_t TxMailboxPeriodicMap;
	
	/*
	 * Find the first available transmit mailbox ID number, and return it as a
	 * bitmask.  WARNING: The operational pair of FindFirstTxMailbox() followed
	 * by TransmitMailbox() must be wrapped around a single CriticalSection.
	 * 
	 * @param mask[out] The mask corresponding to the transmit mailbox, if one
	 *    is found
	 * @return The first available transmit mailbox, or NULL if none is available.
	 */
	volatile MBOX* FindFirstTxMailbox(uint32_t& mask);
	
	/*
	 * Low-level function to perform the grunt work of transferring data to
	 * a mailbox. WARNING: The operational pair of FindFirstTxMailbox() followed
	 * by TransmitMailbox() must be wrapped around a single CriticalSection.
	 * 
	 * @param mask The mask returned by a previous call to FindFirstTxMailbox().
	 * @param dst[out] The mailbox returned by a previous call to FindFirstTxMailbox().
	 * @param src[in]  The data to be sent.
	 */
	void TransmitMailbox(uint32_t mask, volatile MBOX* dst, const TxPacket& src);
	
	/*
	 * Service the transmit message complete ISR.
	 */
	void TxPacket_HWI(uint16_t box);
	
	/**
	 * Handle the rx message ready ISR.
	 */
	void RxPacket_HWI(uint16_t box);
	
	/**
	 * Handle the Can bus system ISR
	 */
	void System_HWI(void);
	
	/*
	 * Top-level mailbox interrupt handler.  Determines the nature of the service
	 * request, and dispatches to one of the lower-level handlers.
	 */
	void Mailbox_HWI(void);
	
	/*
	 * Queue of packets that have been received but not processed.
	 */
	QueueHandle RxPacketsReady;
	
	/*
	 * Overflow queue of packets that are yet to be sent.  May be null if a soft
	 * queue is not being used.
	 */
	QueueHandle TransmitOverflow;
	
	/*
	 * Rx message handlers
	 */
	RxPacketHandler RxHandlers[CAN_DRIVER_MAX_BOXES];
	void *RxHandlerClosures[CAN_DRIVER_MAX_BOXES];

	/*
	 * Initialize the CAN hardware.  While the constructor may safely be invoked
	 * on a global object, the hardware setup needs to be delayed somewhat
	 * in the startup sequence for the board.
	 */
	void InitHardware(void);
	
	/*
	 * Run the periodic state machine.  This check also monitors the hardware.
	 */
	void RunStateMachine(void);

// Functions which must be overriden in derived classes.
protected:
	/*
	 * Derived classes will implement this function to configure their Rx
	 * mailboxes after basic hardware initialization is complete.
	 */
	virtual void ConfigureRxMailboxes(void);
	
	/*
	 * Derived classes will implement this function.
	 * @return true when the hardware is safe to initialize.
	 */
	virtual bool CanHardwareReady(void);
	
// Publically available data and functions
public:

	// Statistic counters
	// The number of received packets
	uint32_t RxCount;
	// The number of transmitted packets
	uint32_t TxCount;
	// The number of packet errors
	uint32_t ErrorCount;
	// Subclassifications for CAN errors.
	uint16_t ErrorCountFE;
	uint16_t ErrorCountBE;
	uint16_t ErrorCountCRCE;
	uint16_t ErrorCountSE;
	uint16_t ErrorCountACKE;
	uint16_t ErrorCountBoxReset;
	// The maximum number of hardware transmit buffers that have ever been full
	// at once
	uint16_t MaxQueueFill;
	
	// The number of packets that could not be sent, because no hardware mailbox
	// was available, and there was no space in the software transmit buffer.
	uint16_t TransmitOverflowFailCount;
	// The number of packets that could not be received because the software rx
	// queue was full.
	uint16_t RxOverflowFailCount;
	// The number of packets that could not be received, because the ISR did
	// not read out the packet prior to the arrival of a new packet in the
	// buffer.
	uint16_t RxMailboxOverrunCount;
	
	/*
	 * Construct a new CanDriver object.
	 * @param id: The CAN driver to initialize
	 * @param TxMailboxes The number of hardware mailboxes to configure for
	 * message transmission
	 * @param RxPendingSize The number of messages that may be pending for later
	 * processing
	 * @param TransmitOverflowSize The number of messages that may be pending
	 * for transmission, in addition to the hardware transmit boxes.
	 */
	CanDriver(CanDriverId id, uint16_t TxMailboxes, size_t RxPendingSize, 
		size_t TransmitOverflowSize);
	
	/*
	 * Virtual destructor, is the default
	 */
	virtual ~CanDriver();
	
	/*
	 * Return the UPM ID number, as used on this bus.
	 */
	virtual uint_least8_t UPMNumber();
	
	/*
	 * Read the current state of the state driver.
	 */
	State GetState() { return state; }
	
	/*
	 *  Return the number of transmit mailboxes that are currently occupied.
	 */
	uint16_t TxQueueFill(void);
	
	/*
	 * Return the number of entries available according to the hardware
	 * protection semaphore.
	 */
	uint16_t TxMailboxesStuck(void);
	
	/*
	 * Register a CAN rx packet handler that will process packets as they arrive.
	 * the provided function will be invoked from the context of RxPacketTask()
	 * @param handler The function that will receive the packet
	 * @param closure An extra argument that will be passed to the handler, in
	 *   addition to the packet itself.
	 * @param mask The mask of bits which must match in match in order for the
	 *   packet to be received.
	 * @param match The bits which must match those present in mask in order to
	 *   be processed.
	 * @param overflow The number of extra hardware mailboxes that should be
	 *   configured to receive this packet type.
	 */
	void RxPackets(RxPacketHandler handler, void *closure, 
		uint32_t mask, uint32_t match, uint16_t overflow = 1);
	
	/**
	 * Transmit a CAN packet.  Attempt to use a hardware buffer if available,
	 * and fall back to a software queue entry if not.
	 * @param data The packet to transmit
	 * @param periodic True if the packet is periodic, and false otherwise. If
	 * the packet is regularly sent periodic data, then setting periodic to true
	 * will change the error-handling policy for the packet.  Periodic packets
	 * are not automatically retried.
	 */
	void PacketTx(const MBOX& data, bool periodic = false);
	
	/**
	 * Periodic function that will drain the overflow FIFO.  This function should
	 * be called every 5 ms by the normal periodic task.
	 */
	void DrainTxFifo(void);
	
	/**
	 * Continuously running task that handles received packets in task context.
	 */
	void RxPacket_TSK(void);
	
	/*
	 * Hardware interrupt on eCAN channel 0.
	 */
	void Channel0_HWI(void);
	
	/*
	 * Hardware interrupt on eCAN channel 1.
	 */
	void Channel1_HWI(void);
};

#endif /*ECANDRIVER_H_*/
