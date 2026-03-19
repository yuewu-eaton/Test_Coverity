#ifndef ICANDRIVER_HPP_
#define ICANDRIVER_HPP_

#include "CanDriver.h"
#include "HQ_Funcs.h"
#include "CanIds.h"

enum BUILDING_INPUT_STATES
{
    BI_ON_GENERATOR,
    BI_REMOTE_GO_TO_BYPASS,
    BI_CHARGER_OFF,
    BI_BATTERY_DISCONNECTED,
    BI_MAINTENANCE_BYPASS,
    BI_MOB_OPEN,
    BI_NOT_CONFIGURED,
    BI_CHIMNEY_FAN_FAIL,
    BI_REMOTE_UPS_ON,
    BI_REMOTE_UPS_OFF,
    // add here....


    BI_NUM_OF_MAX

};

class InternalCanNetwork : public CanDriver
{
public:
	InternalCanNetwork();

	// InternalCanPacket sending functions
	void SendCanId(void);
	void SendUpsNumber(void);
	void SendUPMNodebits(void);
	void SendHistory(const st_HQ_event& ev);
	void SendUPMMeters(void);
	void SendSectionResetResponse(uint16_t section, bool result);
    void SendRTCrequest( void );
	void SendCommandResponse(bool result);
	void SetSystemTypeCheckResult(bool result);
	bool GetSystemTypeCheckResult(void);
	virtual uint_least8_t UPMNumber();
	virtual bool CanHardwareReady(void);
	
private:
	// InternalCanPacket receiving functions
    static void ReceiveC9Packet(const MBOX& packet, const void *closure);
    static void ReceiveCommandPacket(const MBOX& packet, const void *closure);
    static void ReceiveBuildingInputPacket(const MBOX& packet, const void *closure);
    static void ReceiveCanIDPacket(const MBOX& packet, const void *closure);
    static void ReceiveRTCPacket(const MBOX& box, const void *closure);
    static void ReceiveSectionResetRequest(const MBOX& packet, const void *closure);
    static void ReceiveSetUpsNumberPacket(const MBOX& packet, const void *closure);
    static void ReceiveCSBMetersPacket(const MBOX& packet, const void *closure);
	
protected:
	virtual void ConfigureRxMailboxes(void);
};

extern InternalCanNetwork InternalCan;

struct InternalCanPacket : public MBOX
{
public:
    /* Performance note. This style of construction by *this-return
     * chaining looks quite bad from an esw perspective. However, the
     * compiler is perfectly capable of optimizing away all of the AGU
     * ops into a series of constants. In particular, this packet construction
     * CanQueWrt(&LoCanQue, InternalCanPacket(0x123)
            .priority(InternalCanPacket::PRIORITY_LOWEST)
            .subtype(0x45)
            .data0(value1)
            .data1(value2)
            .data2(value3)
            .CrcPacket());
        
        Gets lowered to a bunch of simple assignments.
        
        Also, do not refactor these member functions to be separately declared
        and defined. Listing them in the class inline makes the functions inline, too.
        
        A note on word order. The hardware is configured to transmit data
        bytes in big-endian order on the wire. However, it is being setup using
        whole-word writes rather than halfword writes. Therefore, the halfwords
        need to be swapped when formatting the data packets. So long as this
        class is used for that purpose, there will be no problems.
     */
    enum UpsAddr {
        REMOTE_UPS_1 = 1,
        REMOTE_UPS_2,
        REMOTE_UPS_3,
        REMOTE_UPS_4,
        REMOTE_UPS_5,
        REMOTE_UPS_6,
        REMOTE_UPS_7,
        REMOTE_UPS_8
    };
    
    enum UpmId {
        UPM1 = 1,
        UPM2,
        UPM3,
        UPM4,
        UPM5,
        UPM6,
        UPM7,
        UPM8,
        UPM_ALL
    };
    
    // Implicit copy constructor
    InternalCanPacket(const MBOX& base)
    {
    	MSGID.all = base.MSGID.all;
    	MSGCTRL.all = base.MSGCTRL.all;
    	MDL.all = base.MDL.all;
    	MDH.all = base.MDH.all;
    }
    
    explicit InternalCanPacket(uint16_t type, can_priority pri)
    {
        MSGID.all = uint32_t(type & 0xfff) << 15;
        MSGCTRL.all = 0;
        MDL.all = 0;
        MDH.all = 0;
        
        MSGID.bit.IDE = 1;
        MSGID.bit.AME = 1;
        MSGCTRL.bit.RTR = 0;
        MSGCTRL.bit.DLC = 8;
        MSGCTRL.bit.TPL = 0;
        priority(pri);
    }
    
    /*
     * Get the type value for the packet header.
     */
    uint16_t type() const
    {
    	return (MSGID.all >> 15) & 0xfff;
    }
    
    /*
     *  Set the subtype value for the packet. This value is allocated on
     * a packet-by-packet basis.
     */
    InternalCanPacket& subtype(uint16_t m_subtype)
    {
        const uint32_t subtypeMask = 0x00007f00;
        MSGID.all = (MSGID.all & ~subtypeMask) | (uint32_t(m_subtype) << 8);
        return *this;
    }
    
    /*
     * Get the subtype value for the packet header.
     */
    uint16_t subtype() const
    {
        const uint32_t subtypeMask = 0x00007f00;
    	return (MSGID.all & subtypeMask) >> 8;
    }
    /* 
     * Set the packet's pCan source
     */
    InternalCanPacket& source(UpmId upm)
    {
        const uint32_t sourceMask = 0x0f;
        MSGID.all = (MSGID.all & ~sourceMask) | uint32_t(upm);
        return *this; 
    }
    
    /*
     *  Set the packet's pCan destination
     */
    InternalCanPacket& dest(UpmId upm)
    {
        const uint32_t destMask = 0xf0;
        MSGID.all = (MSGID.all & ~destMask) | (uint32_t(upm) << 4);
        return *this; 
    }
    
    /* 
     * Set the packet's priority
     */
    InternalCanPacket& priority(can_priority pri)
    {
        const uint32_t priorityMask = uint32_t(0x3) << 27;
        MSGID.all = (MSGID.all & ~priorityMask) | (uint32_t(pri) << 27);
        return *this;
    }
    
    /*
     * Set all four of the packet's data words from an array
     */
    InternalCanPacket& data(uint16_t words[])
    {
        data0(words[0]);
        data1(words[1]);
        data2(words[2]);
        data3(words[3]);
        return *this;
    }
    
    /*
     * Add three words of data from an array, and add CRC protection to
     * the packet.
     */
    InternalCanPacket& data_crc(uint16_t words[])
    {
        data0(words[0]);
        data1(words[1]);
        data2(words[2]);
        return CrcPacket();
    }
    
    /*
     * Set data word 0 of the packet.
     */
    InternalCanPacket& data0(uint16_t data)
    {
        MDL.word.HI_WORD = data;
        return *this;
    }

    /*
     * Get data word 1 of the packet.
     */
    uint16_t data0(void) const
    {
        return MDL.word.HI_WORD;
    }
    
    /*
     * Set data word 1 of the packet
     */
    InternalCanPacket& data1(uint16_t data)
    {
        MDL.word.LOW_WORD = data;
        return *this;
    }
    
    /*
     * Get data word 1 of the packet
     */
    uint16_t data1(void) const
    {
        return MDL.word.LOW_WORD;
    }
    
    /*
     * Set data word 2 of the packet
     */
    InternalCanPacket& data2(uint16_t data)
    {
        MDH.word.HI_WORD = data;
        return *this;
    }
    
    /*
     * Get data word 2 of the packet
     */
    uint16_t data2(void) const
    {
        return MDH.word.HI_WORD;
    }
    
    /*
     * Set data word 3 of the packet
     */
    InternalCanPacket& data3(uint16_t data)
    {
        MDH.word.LOW_WORD = data;
        return *this;
    }
    
    /*
     * Get data word three of the packet
     */
    uint16_t data3(void) const
    {
        return MDH.word.LOW_WORD;
    }
    
    /* Create a CRC16 and add it to the third word of the packet
     * TODO: Actually use a CRC16 instead of this cheap method
     */
    InternalCanPacket& CrcPacket();
};


#endif /*ICANDRIVER_HPP_*/
