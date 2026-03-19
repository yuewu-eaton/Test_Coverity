#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "CanDriver.h"
#include "Constants.h"
#include "Eeprom_Map.h"
#include <tsk.h>
#include "InternalCan.h"

namespace {
    void EmptyMailboxFunc(const MBOX& packet, const void *data)
    {
    }
}

CanDriver::CanDriver(CanDriverId id, uint16_t TxMailboxes, size_t RxPendingSize, 
        size_t TransmitOverflowSize)
    : state(Initialization)
    , TxMailboxCount(TxMailboxes)
    , TxMailboxPeriodicMap(0)
    , RxPacketsReady( QueueCreate(RxPendingSize, sizeof(RxPacket)))
    , TransmitOverflow((TransmitOverflowSize == 0) 
        ? NULL : QueueCreate(TransmitOverflowSize, sizeof(TxPacket)))
    , RxCount(0)
    , TxCount(0)
    , ErrorCount(0)
    , ErrorCountFE(0)
    , ErrorCountBE(0)
    , ErrorCountCRCE(0)
    , ErrorCountSE(0)
    , ErrorCountACKE(0)
    , ErrorCountBoxReset(0)
    , MaxQueueFill(0)
    , TransmitOverflowFailCount(0)
    , RxOverflowFailCount(0)
    , RxMailboxOverrunCount(0)
{
    for (size_t i = 0; i < CAN_DRIVER_MAX_BOXES; ++i)
    {
        RxHandlers[i] = EmptyMailboxFunc;
        RxHandlerClosures[i] = NULL;
    }

    switch (id) {
        case ECANA:
            Regs = &ECanaRegs;
            Mboxes = &ECanaMboxes;
            LAMRegs = &ECanaLAMRegs;
            MOTORegs = &ECanaMOTORegs;
            MOTSRegs = &ECanaMOTSRegs;
            break;

        case ECANB:
            Regs = &ECanbRegs;
            Mboxes = &ECanbMboxes;
            LAMRegs = &ECanbLAMRegs;
            MOTORegs = &ECanbMOTORegs;
            MOTSRegs = &ECanbMOTSRegs;
            break;
    };
}

CanDriver::~CanDriver(void)
{
}

void
CanDriver::InitHardware(void)
{
    if (Regs == &ECanaRegs)
    {
        InitECanaGpio();
        InitECana();
    }
    else
    {
        InitECanbGpio();
        InitECanb();
    }
    
    // Disable all mailboxes
    Regs->CANME.all = 0L;
    // Clear all transmit requests
    Regs->CANTRS.all = 0L;
    Regs->CANTRR.all = 0xffffffffL;
    
    // Bits set in this mask correspond to the mailboxes that are going to be
    // configured for transmission.
    uint32_t txMask = ~( (1L << CAN_DRIVER_MAX_BOXES - TxMailboxCount) -1);
    EALLOW;
    // Set mailbox interrupt level to be level 1 for all transmit mailbox ISRs,
    // and level 0 for receive
    Regs->CANMIL.all = txMask;
    // Enable interrupts for all of the transmit boxes
    Regs->CANMIM.all = txMask;
    EDIS;
    
    // Clear all packet ack flags
    while (Regs->CANAA.all != 0)
    {
        Regs->CANAA.all = 0xffffffff;
    }
    Regs->CANTA.all = 0xffffffff;
    Regs->CANRFP.all = 0xffffffff;
    Regs->CANRML.all = 0xffffffff;
    
    ConfigureRxMailboxes();
    
    CANGIM_REG cangim;
    cangim.all = 0;
    /*
    // The following section would enable system interrupts.  In the current
    // design, system errors and state changes are handled by polling the
    // driver.  Only packet handling is done in interrupts.
    cangim.bit.AAIM = 1; // abort ack
    cangim.bit.EPIM = 1; // error-passive
    cangim.bit.WLIM = 1; // warning level
    cangim.bit.BOIM = 1; // bus-off
    
    // System interrupt on level 0
    cangim.bit.GIL = 0;
    */
    // enable levels zero and one
    cangim.bit.I0EN = 1;
    cangim.bit.I1EN = 1;
    
    EALLOW;
    Regs->CANGIM.all = cangim.all;
    
    state = Running;
    EDIS;
    
    // Enable module interrupts.
    if (Regs == &ECanaRegs)
    {
        PieCtrlRegs.PIEIER9.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx6 = 1;
    }
    else
    {
        PieCtrlRegs.PIEIER9.bit.INTx8 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx7 = 1;
    }
    IER |= M_INT9;
}

uint16_t
CanDriver::TxQueueFill(void)
{
    uint32_t canme;
    {
    	CriticalSection enter( IER_DMA_ONLY );
        canme = Regs->CANME.all;
    }
    uint16_t boxesEmpty = 0;
    
    uint16_t firstBox = (CAN_DRIVER_MAX_BOXES - TxMailboxCount);
    uint32_t mask = 1L << firstBox;
    const volatile MBOX* box = &Mboxes->MBOX0 + firstBox;
    for ( ; box < &Mboxes->MBOX0 + CAN_DRIVER_MAX_BOXES; box++, mask <<= 1)
    {
        if (!(canme & mask))
        {
            boxesEmpty++;
        }
    }
    
    return TxMailboxCount - boxesEmpty;
}

uint16_t
CanDriver::TxMailboxesStuck(void)
{
	// Somehow, after rolling through a bus off state, a mailbox may end up configured
	// for transmission, but with neither a transmission pending or an abort request.
	// Clear such mailboxes out.
	// Note: This has since been linked to the silicon errata for the TMS320F28335
	uint32_t canme, canta, cantrs, cantrr, canaa, canmd;
	{
		CriticalSection enter;
		canme = Regs->CANME.all;
		canta = Regs->CANTA.all;
		cantrs = Regs->CANTRS.all;
		cantrr = Regs->CANTRR.all;
		canaa = Regs->CANAA.all;
		canmd = Regs->CANMD.all;
	}
	
	uint32_t mysteryState =
		canme & // enabled
		~canmd & // transmit
		~canta & // transmit ack
		~cantrs & // transmit request
		~cantrr & // transmit abort request
		~canaa; // transmit abort ack
	// Find out which (if any) packets are in the "mystery state"
	uint16_t count = 0;
	for (uint32_t mask = 1; mask; mask <<= 1)
	{
		if (mysteryState & mask)
			count++;
	}
	return count;
}

void
CanDriver::RxPackets(RxPacketHandler handler, void *closure, 
    uint32_t mask, uint32_t match, uint16_t overflow)
{
    uint16_t box = 0;
    // Find the lowest un-enabled mailbox
    CANME_REG canmeShadow;
    canmeShadow.all = Regs->CANME.all;
    while ((1L << box) & canmeShadow.all)
    {
        box++;
    }
    
    bool lowest = true;
    do
    {
        // Configure one mailbox, plus any overflow mailboxes.  The primary box
        // is the lowest one to be enabled.
        RxHandlers[box] = handler;
        RxHandlerClosures[box] = closure;
        // See the eCAN manual, sec 3.2.3
        // Temporary variable to act as a 32-bit shadow register.
        uint32_t shadow;
        uint32_t boxMask = 1L << box;
        // Write the LAM and MSGID bits
        (&LAMRegs->LAM0 + box)->all = (~mask) & 0x1fffffffL;
        (&Mboxes->MBOX0 + box)->MSGID.all = (match & 0x1fffffffL) | 0xc0000000L;
        // Configure for receive
        shadow = Regs->CANMD.all;
        shadow |= boxMask;
        Regs->CANMD.all = shadow;

        // Enable overflow protection, except for the lowest-index mailbox
        if (lowest)
        {
            lowest = false;
        }
        else
        {
            shadow = Regs->CANOPC.all;
            shadow |= boxMask;
            Regs->CANOPC.all = shadow;
        }
        // Enable the interrupt
        EALLOW;
        shadow = Regs->CANMIM.all;
        shadow |= boxMask;
        Regs->CANMIM.all = shadow;
        EDIS;
        
        // Enable the mailbox
        shadow = Regs->CANME.all;
        shadow |= boxMask;
        Regs->CANME.all = shadow;

        // pick next box
        ++box; 
    } while (overflow --> 0);
}

void
CanDriver::RxPacket_TSK(void)
{   
    // Wait for prereq's to be met, and then initialize the hardware
    while (!CanHardwareReady())
    {
        TSK_sleep(TSK_10_ms);
    }   
    InitHardware();
    
    // Receive packet handling.  Dequeue the packet and dispatch to its handler.
    while (true)
    {
        RxPacket packet;
        if (QueueReceive(RxPacketsReady, &packet, SYS_FOREVER))
        {
            RxCount++;
            RxHandlers[packet.mailboxId](packet, RxHandlerClosures[packet.mailboxId]);
        }
    }
}

volatile MBOX*
CanDriver::FindFirstTxMailbox(uint32_t& mask)
{
    uint32_t canme = Regs->CANME.all;
    
    // This search needs to start at the highest numbered mailbox and work down.  Otherwise
    // packets are likely to arrive out-of-order
    
    uint16_t firstBox = CAN_DRIVER_MAX_BOXES - 1;
    // lastBox is actually one-past-the-end of the loop, as is the C++ custom
    uint16_t lastBox = (CAN_DRIVER_MAX_BOXES - TxMailboxCount - 1);
    
    mask = 1L << firstBox;
    volatile MBOX* box = &Mboxes->MBOX0 + firstBox;
    volatile MBOX* end = &Mboxes->MBOX0 + lastBox;
    uint16_t boxPos = 1;
    for ( ; box != end && mask != 0; box--, mask >>= 1, boxPos++)
    {
        if (!(canme & mask))
        {
        	if (boxPos > MaxQueueFill)
        		MaxQueueFill = boxPos;
            return box;
        }
    }
    MaxQueueFill = boxPos - 1;
    return NULL;
}

void
CanDriver::TransmitMailbox(uint32_t boxMask, volatile MBOX* dst, const TxPacket& src)
{
    // write mail box data
    dst->MSGID.all = src.MSGID.all;
    dst->MSGCTRL.all = src.MSGCTRL.all;
    dst->MDH.all = src.MDH.all;
    dst->MDL.all = src.MDL.all;

    // enable mailbox
    {
    	CriticalSection enter( IER_DMA_ONLY );
	    CANME_REG tempME;
	    tempME.all = Regs->CANME.all;
	    tempME.all |= boxMask;
	    Regs->CANME.all = tempME.all;
    }
    
    if (src.isPeriodic)
    {
        TxMailboxPeriodicMap |= boxMask;
    }
    
    Regs->CANTRS.all = boxMask;
}

void
CanDriver::PacketTx(const MBOX& data, bool periodic)
{
	/*
	 * Warning: This is not allowed to block for any reason.  If the transmit
	 * buffers are empty, then the packet must be rejected outright.  Otherwise
	 * various periodic tasks in the system may fail to meet their deadlines.
	 */
    uint32_t txMask;
    TxPacket packet(data, periodic);
    switch (state) {
    case Initialization:
        // Suppress all packet transmission until after startup is complete.
        break;
        
    case Passive:
        // All packets go through the soft transmit buffer
        if (TransmitOverflow)
        {
            if (!QueueSend(TransmitOverflow, &packet, 0))
            {
                TransmitOverflowFailCount++;
            }
        }
        else
        {
    	    CriticalSection enter( IER_DMA_ONLY );
            volatile MBOX *txBox = FindFirstTxMailbox(txMask);
            if (txBox != NULL)
            {
                TransmitMailbox(txMask, txBox, packet);
            }
            else
            {
                TransmitOverflowFailCount++;
            }
        }
        break;
        
    case Running: {
    		// Note: Only operations on the hardware tx mailbox are within the
    		// critical section.  Operations on the software queue are not.
    		volatile MBOX *txBox = NULL;
    		{
    	    	CriticalSection enter( IER_DMA_ONLY );
	            txBox = FindFirstTxMailbox(txMask);
	            if (txBox != NULL)
	            {
	                TransmitMailbox(txMask, txBox, packet);
	            }
    		}
    		if (txBox == NULL)
    		{
	            if (TransmitOverflow)
	            {
		            if (!QueueSend(TransmitOverflow, &packet, 0))
		            {
		                TransmitOverflowFailCount++;
							//delete for jira 385, under short inner can, may trip can module fail can not feedback back
//								//follow 3C3 upm lost comm with csb
//								if(Regs == &ECanaRegs)
//								{
//									InternalCan.InitHardware();
//								}
		            }
	            }
	            else
	        	{
	        		TransmitOverflowFailCount++;
	        	}
    		}
    		else
    		{
    			// If txbox != NULL, then a box was available and the transmission
    			// is complete.
    		}
        }
        break;
    }
}

void
CanDriver::DrainTxFifo(void)
{
    RunStateMachine();
    
    // Limit the number of packets that may be processed in one run.  When the
    // CAN network is down, it may take more than 5 ms to completely
    // drain the tx fifo of periodic traffic.  10 packets per pulse represents
    // about 65% of the available network bandwidth.
    const uint16_t maxPackets = 10;
    
    if (TransmitOverflow && state != Initialization)
    {
    	TxPacket packet;
    	for (uint16_t packets = 0; 
    		packets < maxPackets && QueueReceive(TransmitOverflow, &packet, 0);
    		packets++)
        {
			// A packet to be transmitted is available.  Check for a hardware transmit buffer.
			// If none is available, then recycle the transmitted packet back through the soft queue
			// and stop sending packets for this pulse only. Otherwise, send the packet.
			volatile MBOX *txBox = NULL;
			{
	    	    CriticalSection enter( IER_DMA_ONLY );
	            uint32_t boxMask;
	            txBox = FindFirstTxMailbox(boxMask);
	            if (txBox)
	            {
	                TransmitMailbox(boxMask, txBox, packet);
	            }
			}
			if (txBox == NULL)
			{
				if (!QueueSend(TransmitOverflow, &packet, 0))
				{
	            	TransmitOverflowFailCount++;
				}
				// Do not send any more packets, the hardware is full
				break;
        	}
        }
    }
    else
    {
        // No software overflow buffer is configured and there is nothing to do.
    }
}

void
CanDriver::RunStateMachine(void)
{
    // Handle major system errors and status changes.
    CANES_REG basicErrorMask = { 0 };
    
    CANES_REG canes = { Regs->CANES.all };
    basicErrorMask.bit.FE = 1; // Framing error
    basicErrorMask.bit.BE = 1; // Bit error
    basicErrorMask.bit.CRCE = 1; // CRC error
    basicErrorMask.bit.SE = 1; // Stuffing error
    basicErrorMask.bit.ACKE = 1; // Acknowledgement error
    

    if (canes.all & basicErrorMask.all)
    {
    	if (canes.bit.FE)
    		ErrorCountFE++;
    	if (canes.bit.BE)
    		ErrorCountBE++;
    	if (canes.bit.CRCE)
    		ErrorCountCRCE++;
    	if (canes.bit.SE)
    		ErrorCountSE++;
    	if (canes.bit.ACKE)
    		ErrorCountACKE++;
        // A basic error has been signaled.  Just increment a counter and move
        // on.  There may have been more than one of these errors since the last
        // time we checked it, but it is impossible to determine how many.
        Regs->CANES.all = basicErrorMask.all;
        ErrorCount++;
    }
    
    if (canes.bit.BO)
    {
        // Hardware is in the bus-off state.  It will automatically return to
        // normal after a set number of bus clocks.  Therefore, transition the
        // state machine to the running state and ack the bit.
        state = Running;
        CANES_REG clearStateFlags;
        clearStateFlags.all = 0;
        clearStateFlags.bit.BO = 1;
        // Note: Only handle bus off in this pass.  Even though error passive is also set,
        // wait until the next pass to handle it.  Otherwise the hardware locks up(?!).
        Regs->CANES.all = clearStateFlags.all;
    }
    else if (canes.bit.EP)
    {
        // Hardware has entered the error-passive state
        // Check to see if the TEC has decreased below the
        // TEC limit before dropping this state machine out of
        // the error-passive state.
        if (Regs->CANTEC.all < 128)
        {
            // Transitioned out of error-passive due to successful
            // packet transmission.  Transition the system state to running.
            state = Running;
            CANES_REG clearErrorPassive;
            clearErrorPassive.all = 0;
            clearErrorPassive.bit.EP = 1;
            Regs->CANES.all = clearErrorPassive.all;
        }
        else
        {
            // Still (or newly) in error-passive.
            state = Passive;
            if (TxMailboxPeriodicMap)
            {
                // Clear all periodic packets that have not yet been sent.
                Regs->CANTRR.all = TxMailboxPeriodicMap;
                TxMailboxPeriodicMap = 0;
            }
        }
    }
    
    if (canes.bit.EW)
    {
        // Hardware has entered the error-warning state.
        // Just ack the error and carry on.
        CANES_REG clearErrorWarning;
        clearErrorWarning.all = 0;
        clearErrorWarning.bit.EW = 1;
        Regs->CANES.all = clearErrorWarning.all;
    }
    
    // Clean up after abort ack
    uint32_t canaa = Regs->CANAA.all;
    if (canaa)
    {
    	CriticalSection enter( IER_DMA_ONLY );
        // Disable the abort-acked mailboxes
        uint32_t canme = Regs->CANME.all;
        Regs->CANME.all = canme & ~canaa;

        // Ack for the counted bits and no others.
        Regs->CANAA.all = canaa;
    }
    
	uint32_t canme, canta, cantrs, cantrr, canmd;
	{
		CriticalSection enter( IER_DMA_ONLY);
		canme = Regs->CANME.all;
		canta = Regs->CANTA.all;
		cantrs = Regs->CANTRS.all;
		cantrr = Regs->CANTRR.all;
		canaa = Regs->CANAA.all;
		canmd = Regs->CANMD.all;
	}
	
	// Somehow, after rolling through a bus off state, a mailbox may end up configured
	// for transmission, but with neither a transmission pending or an abort request.
	// Clear such mailboxes out.
	// Note: This has since been linked to the silicon errata for the TMS320F28335
	uint32_t mysteryState =
		canme &   // enabled
		~canmd &  // configured for transmit
		~canta &  // not in transmit ack
		~cantrs & // not in transmit request
		~cantrr & // not in transmit abort request
		~canaa;   // not in transmit abort ack
	if (mysteryState != 0) 
	{
		ErrorCountBoxReset++;
		CriticalSection enter( IER_DMA_ONLY);
		uint32_t canme = Regs->CANME.all;
		Regs->CANME.all = canme & ~mysteryState;
	}
}

#pragma CODE_SECTION("ramfuncs")
void
CanDriver::TxPacket_HWI(uint16_t box)
{
    uint32_t boxMask = uint32_t(1) << box;
    
    // Clear the periodic packet map
    TxMailboxPeriodicMap &= ~boxMask;
    
    // Disable the mailbox
    {
    	CriticalSection enter( IER_DMA_ONLY ); //JIRA GPE-1327 critical section needed hear or else semaphore could not in another thread
		uint32_t tempReg = Regs->CANME.all;
		tempReg &= ~boxMask;
		Regs->CANME.all = tempReg;
		// Ack the transmit ack bit
		Regs->CANTA.all = boxMask;
    }
    
    TxCount++;
}

#pragma CODE_SECTION("ramfuncs")
void
CanDriver::RxPacket_HWI(uint16_t boxId)
{
    // Two behaviors, depending on whether the mailbox has overwrite protection
    // or not.
    const volatile MBOX* box = (&Mboxes->MBOX0) + boxId;
    uint32_t boxMask = 1L << boxId;
    if (Regs->CANOPC.all & boxMask)
    {
        // The box has overwrite protection enabled.  First read the packet,
        // then clear the RMP bit to enable additional message Rx.
        RxPacket packet(box, boxId);
        if (!QueueSend(RxPacketsReady, &packet, 0))
        {
            RxOverflowFailCount++;
        }
        Regs->CANRMP.all = boxMask;
    }
    else
    {
        // Box does not have overflow protection enabled.  Read the packet per
        // sec 3.2.4 of the eCAN hardware reference.
        // Clear RMP
        Regs->CANRMP.all = boxMask;
        // Check RML.  If set, increment the error counter, but otherwise
        // continue.  If RML is already set, then one or more whole packets
        // have been lost.
        if (Regs->CANRML.all & boxMask)
        {
            Regs->CANRML.all = boxMask;
            RxMailboxOverrunCount++;
        }
        // Read out the data
        RxPacket packet(box, boxId);
        // Check RMP again.  If it is set, then the message read was clobbered
        // by a newly arrived packet. Increment the error counter and ignore the
        // data.
        if (Regs->CANRMP.all & boxMask)
        {
            Regs->CANRMP.all = boxMask;
            // The overrun is incremented by two since we have lost both the old
            // packet and the new packet.
            RxMailboxOverrunCount += 2;
        }
        else
        {
            if (!QueueSend(RxPacketsReady, &packet, 0))
            {
                RxOverflowFailCount++;
            }
        }
    }
}

#pragma CODE_SECTION( "ramfuncs" )
void
CanDriver::Channel0_HWI(void)
{
    // Mailbox receive
    CANGIF0_REG cangif0 = { Regs->CANGIF0.all };
    uint16_t mbNum = cangif0.bit.MIV0;
    //If we rx one CAN packet, this bit should be set
    if( 0 != cangif0.bit.GMIF0 )    
    {
        RxPacket_HWI(mbNum);
    }
    // Acknowledge interrupt and return
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

#pragma CODE_SECTION( "ramfuncs" )
void
CanDriver::Channel1_HWI(void)
{
    // Mailbox transmit
    CANGIF1_REG cangif1 = { Regs->CANGIF1.all };
    uint16_t mbNum = cangif1.bit.MIV1;
    TxPacket_HWI(mbNum);
    // Acknowledge interrupt and return
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

void
CanDriver::ConfigureRxMailboxes(void)
{
    // By default, do nothing.
}

bool
CanDriver::CanHardwareReady(void)
{
    // By default, never ready.  Child classes must override this function.
    return false;
}

uint_least8_t
CanDriver::UPMNumber()
{
    // By default, the UPM number on the CAN bus is the hardware UPM number on
    // a zero-index basis.
    return MyUPMNumber;
}
