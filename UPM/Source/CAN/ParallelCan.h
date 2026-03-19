#ifndef PARALLELCAN_H_
#define PARALLELCAN_H_

#include <cstring>
using std::memset;

#include "CanDriver.h"
#include "Eeprom_Map.h"

#include "BypassInterface.h"
#include "MCUState.h"
#include "BatteryStateControl.h"
#include "RectifierStateControl.h"
#include "InverterControl.h"
#include "RectifierControl.h"
#include "Abm.h"
#include "Spi_Task.h"

#define DECLINE_CHECK_BYPASS                 0x0001
#define DECLINE_CHECK_BYPASS_PHASE           0x0002
#define DECLINE_CHECK_INPUT                  0x0004
#define DECLINE_CHECK_INPUT_PHASE            0x0008
#define DECLINE_CHECK_EPO                    0x0010
#define DECLINE_CHECK_TEMPERATURE            0x0020
#define DECLINE_CHECK_OUTPUT                 0x0040
#define DECLINE_CHECK_STS                    0x0080
#define DECLINE_CHECK_EEPROM                 0x0100
#define DECLINE_CHECK_FW                     0x0200
#define DECLINE_CHECK_HW_FAIL                0x0400
#define DECLINE_CHECK_MAINTENANCE_BYPASS     0x0800
#define DECLINE_CHECK_BATTERY                0x1000
#define DECLINE_CHECK_COMMUNICATION          0x2000
#define DECLINE_CHECK_HW                     0x4000
#define DECLINE_CHECK_POWERSUPPLY            0x8000

#define PARALLEL_SYNCH_FLAG_IDENTIFY_MASTER         (1U << 0)
#define PARALLEL_SYNCH_FLAG_SCAN_INVERTER           (1U << 1)
#define PARALLEL_SYNCH_FLAG_BYPASS_NOT_AVAILABLE	(1U << 2)

struct stParallelStatus
{
    uint16_t Master      : 1;      // This upm/ups is the master in the system
    uint16_t ParamsInit  : 1;      // At least all shared parameters has been sent one time
    uint16_t ParamsError : 1;      // At least one shared parameter is different
    uint16_t Int_Synced  : 1;      // For slave upm,internal pwm sync signal is received,for master upm always 1
    
    uint16_t Para_Synced : 1;      // For slave upm,parallel sync signal is received,for master upm always 1
    uint16_t AutoID      : 1;      // If Auto ID is latched
    uint16_t ActiveNode  : 1;      // For Auto ID check
    uint16_t UpsOffCmd   : 1;      // UPS Off command
    
    uint16_t ErrorNumber : 8;      // The first number of different shared parameter ranging from 1 to 17
    
    uint16_t word1;
    uint16_t word2;
    uint16_t /*unused*/: 12;
    uint16_t SequenceNumber: 4; 
};

union unParallelStatus
{
    uint16_t         all;
    uint16_t         words[sizeof(stParallelStatus)];
    stParallelStatus bit;
};

struct stC9Command
{
    uint16_t c9_ect_on_command              : 1;
    uint16_t c9_ect_off_command             : 1;
    uint16_t c9_rsvd_command                : 14;
};

union unC9Command
{
    uint16_t        all;
    stC9Command     bit;
};

struct stInternalCommand
{
    uint16_t csb_bypass_on_command          : 1;  // command from csb
    uint16_t csb_on_normal_command          : 1;  // command from csb
    uint16_t csb_load_off_command           : 1;  // command from csb
    uint16_t csb_activate_charger_command   : 1;  // command from csb
    
    uint16_t csb_deactivate_charger_command : 1;  // command from csb
    uint16_t csb_start_battery_test_command : 1;  // command from csb
    uint16_t csb_eco_on_command             : 1;  // command from csb
    uint16_t csb_eco_off_command            : 1;  // command from csb
    
    uint16_t csb_reset_all_alarms_command   : 1;  // command from csb
    uint16_t csb_clear_history_command      : 1;  // command from csb
    uint16_t csb_rsvd_command1              : 2;  // command rsvd
        
    uint16_t csb_rsvd_command2              : 4;  // command rsvd
};

union unInternalCommand
{
    uint16_t          all;
    stInternalCommand bit;
};

struct stParallelCommand
{
    uint16_t para_on_normal_command          : 1; // command from slave upm/ups
    uint16_t para_start_battery_test_command : 1; // command from slave upm/ups
    uint16_t para_reset_alarms_command       : 1; // command from slave upm/ups
    uint16_t para_activate_charger_command   : 1; // command from slave upm/ups
        
    uint16_t para_bypass_on_command          : 1; // command from slave upm/ups
    uint16_t para_rsvd_command1              : 3; // command rsvd
    
    uint16_t para_line_to_battery            : 1; // command rsvd at present
    uint16_t para_battery_to_line            : 1; // command rsvd at present
    uint16_t para_rsvd_command2              : 1; // command rsvd
    uint16_t para_redundant_battery_off      : 1; // flag for mcu, load off and clear normal command
                            
    uint16_t para_inverter_off_sync          : 1; // flag for mcu statemachine
    uint16_t para_ect_on_command             : 1; // ect on command from slave upm/ups
    uint16_t para_ect_off_command            : 1; // ect off command from slave upm/ups
    uint16_t para_redundant_off              : 1; // flag for mcu, shutdown but keep normal command
};

union unParallelCommand
{
    uint16_t          all;
    stParallelCommand bit;
};

typedef enum
{
    EEPROM_UPDATE_ENABLED  = 0xA500,
    EEPROM_UPDATE_DISABLED = 0xABCD 
}  eEepromType;

struct stSyncStatus
{
    // word 0
    uint16_t BypassSyncAvail : 1;
    uint16_t InputSyncAvail  : 1;
    uint16_t SyncState       : 3;
    uint16_t ExtSyncBypass   : 1;
    uint16_t ExtSyncInput    : 1;
    uint16_t /* unused */    : 1;
    
    uint16_t ForceRetransmit : 1;
    uint16_t CanBusFailed    : 1;
    uint16_t unused          : 6;

    // word 1
    uint16_t AutoCalRun      : 1;
    uint16_t AutoCalBypass   : 1;
    uint16_t AutoCalUnsync   : 1;
    uint16_t AutocalInput    : 1;       // place holder, not used in essential

    uint16_t /* unused */    : 12;
    uint16_t word2placeholder;
    
    uint16_t /* unused word 3 */: 12;
    uint16_t SequenceCount : 4;
};

#define SYNC_WORD           0
#define AUTO_CAL_WORD       1

union uSyncStatus
{
    stSyncStatus bit;
    uint32_t     all;
    uint16_t     words[sizeof(stSyncStatus)];
};

struct stSystemError
{
    uint16_t err_bypass        : 1;
    uint16_t err_bypass_phase  : 1;
    uint16_t err_input         : 1;
    uint16_t err_input_phase   : 1;
                               
    uint16_t err_epo           : 1;
    uint16_t err_temperature   : 1;
    uint16_t err_output        : 1;
    uint16_t err_sts           : 1;
                               
    uint16_t err_eeprom        : 1;
    uint16_t err_fw            : 1;
    uint16_t err_hw_fail       : 1;
    uint16_t err_system        : 1;
                               
    uint16_t err_battery       : 1;
    uint16_t err_communication : 1;
    uint16_t err_hw            : 1;
    uint16_t err_powersupply   : 1;
    
    uint16_t word1placeholder;
    uint16_t word2placeholder;
    
    uint16_t err_load_share_PhA : 1;
    uint16_t err_load_share_PhB : 1;
    uint16_t err_load_share_PhC : 1;
    uint16_t /*unused*/: 9;
    uint16_t SequenceNumber: 4;
};

union unSystemError
{
    stSystemError bit;
    uint16_t      words[sizeof(stSystemError)];
    uint16_t      all;
};

/*
 * A data structure that records the observed state of each UPM in the system.
 */
struct UpmState
{
    uBypassStatus BypassStatus;
    uBatteryStateStatus BatteryStatus;
    uMCUStatus McuStatus;
    uInverterStatus InverterStatus;
    uSyncStatus SyncStatus;
    unSystemError SystemError;
    unParallelStatus ParallelStatus;
    uAbmStatus AbmStatus;
    
    // A down-counter indicating the time remaining before considering a node
    // to be off of the bus.  When transmitting as self, reset this count
    // to be UPM_PACKET_ALIVE_COUNT.  When recieving from other, set it to
    // UPM_PACKET_TIMEOUT_COUNT.  If the counter for another node reaches zero,
    // then the node is considered to be offline.  If the counter for this node
    // reaches zero, then resend the status packets.
    uint16_t Count;
    // True if we haven't heard from this UPM in the timeout period.  Such UPMs
    // are considered to be off the bus.
    bool Timeout;
    UpmState()
    {
        memset(this, 0, sizeof(*this));
        Timeout = true;
    }
};

struct CalMeter
{
    // real power total.  Normalized, with -1 == -32766 to 1.0 == 32767
    int16_t id;
    // real power ph A
    int16_t powerA;
    // real power ph B
    int16_t powerB;
    // real power ph C
    int16_t powerC;
    CalMeter()
    {
        memset(this, 0, sizeof(*this));
    }
};

struct LoadShareMeter
{
    // total load percent, in percent of one UPM's rating 
    uint16_t sum;
    // single phase load percent, in percent of one UPM's phase rating
    uint16_t phA;
    uint16_t phB;
    uint16_t phC;
};

class ParallelCanNetwork: public CanDriver
{
private:
    bool BusError();
    friend void ParallelSystem5Msec(void);
public:
    struct StatusBits
    {
        // True if this node is part of a parallel UPS system, and its UPS number
        // has not yet been assigned.  Traffic is never transmitted until after
        // our number has been assigned.
        uint16_t ParallelUpsInitializing : 1;
        // True if one or more other nodes on the network have failed.
        uint16_t NodeFailed : 1;
        uint16_t : 14; // reserved
    };
    
    union uStatus
    {
        StatusBits bit;
        uint16_t all;
    } Status;
    
    // Global constant data
    static const uint_least8_t MAX_NUM_UPM = 2;
    static const uint_least8_t MAX_NUM_UPS = 4;
    static const uint_least8_t UPM_PACKET_TIMEOUT_COUNT = 100;
    static const uint_least8_t UPM_PACKET_ALIVE_COUNT = 20;
    
    ParallelCanNetwork();
    virtual ~ParallelCanNetwork();
    virtual uint_least8_t UPMNumber();
    
    void ResetSticky();
    
    void PacketTx(const MBOX& packet, bool periodic = false);
    // All command packets go through this function.
    void TransmitCommandPacket(uint16_t subword, uint16_t data = 0);
    void TransmitBroadcastPacket(uint16_t subword, const uint16_t data[], uint16_t len, bool periodic = true);
    void TransmitBroadcastPacket(uint16_t subword, uint16_t data0,uint16_t data1,uint16_t data2, uint16_t data3, bool periodic = true);
    void TransmitToUPMsInUPS(uint16_t subword, uint16_t data0,uint16_t data1,uint16_t data2, uint16_t data3);
    void TransmitBypassCommand(uint16_t requestedState);
    void TransmitMasterRMSPacket( const stThreePhase& rmsData );
    
    unParallelStatus  ParallelStatus;
    unC9Command       C9Command;
    unInternalCommand InternalCommand;    
    unParallelCommand ParallelCommand;
    unSystemError ParallelSystemError;
    EepAddrValue      MissMatchEep;
    
    void        PCan_CheckMaster(void);
    void        PCan_SyncSharedParams(void);
    void        PCan_CheckSharedParams(void);  
    void        PCan_OnParameterChanged(uint16_t eepAddress, uint16_t eepValue);
    static void PCan_UpdateSharedParams(const MBOX& packet);
    void        PCan_ProcessC9Command( void );
    void        PCan_ProcessInternalCommand(void);
    void        PCan_ProcessParallelCommand(void);
    void        PCan_ProcessSystemCommand(void);
    void        PCan_UpdateSystemErrorStatus(void);
    uint16_t    PCan_CheckSystemOnNormalFail(void);
    uint16_t    PCan_CheckSystemEcoOnAvailable(void);
    uint16_t    PCan_CheckSystemBatteryTestAvailable(void);
    uint16_t    PCan_CheckSystemChargerOnAvailable(void);
    uint16_t    PCan_CheckUPSBypassAvailable(void);
    uint16_t    PCan_CheckUPSInverterAvailable(void);
    uint16_t    PCan_CheckSystemBypassAvailable(void);
    uint16_t    PCan_CheckSystemInverterAvailable(void);
    uint16_t    PCan_CheckSystemRedundant(void);
    void        UpdateInverterAvailable(void);
    uint16_t    PCan_SumOnInverter(void);      //total number of UPSs currently online
    uint16_t    PCan_UPMSumOnInverter(void);   //total number of UPMs currently online
    void        UpdateSystemLoad(void);
    bool        PCan_CheckMOBsClosed( void );
    bool        PCan_CheckMBSClosed( void );
    void        PCan_UpdateUpsNumber(void);
    
    /*********************** Network-global data ****************************/
    UpmState UpmData[MAX_NUM_UPS][MAX_NUM_UPM];
    CalMeter CalData[MAX_NUM_UPS][MAX_NUM_UPM];
    LoadShareMeter LoadData[MAX_NUM_UPS][MAX_NUM_UPM];
    LoadShareMeter SysLoadData;

    // The global bitwise AND of all status bits in the system.
    UpmState ParGlobalAndData;
    // The global bitwise OR of all status bits in the system.
    UpmState ParGlobalOrData;
    // The bitmapping of all UPM's observed since startup
    uint_least8_t AllUPMsSinceReset[MAX_NUM_UPS];
    // The bitmapping of all UPM's observed recently
    uint_least8_t CurrentUPMsOnBus[MAX_NUM_UPS];
    // Total number of UPMs sorted by UPS number
    uint16_t NumberOfUPMs[MAX_NUM_UPS];
    // The total number of UPM's in this system, as determined by CurrentUPMsOnBus.
    uint16_t TotalNumberOfUPMs;
    // The total number of static switches present, as determined by the lowest
    // order bit of CurrentUPMsOnBus
    uint16_t TotalNumberOfBypass;
    // number of upms online
    uint16_t TotalNumberOfUpmsOnline;
    // The bitmapping of all UPS's observed since startup
    uint16_t AllUPSsSinceReset;
    // The bitmapping of all UPS's observed recently
    uint16_t CurrentUPSsOnBus;
    // The total number of UPS's in the Parallel System, as determined by CurrentUPSsOnBus
    uint16_t TotalNumberOfUPSs;
    // the number if read UPM when pull-chain start
    uint16_t ReadyUPMs;
    // The total system load, in percent of one UPM's rating
    float SystemLoad;
    // The number of STSW that are available.
    uint16_t SystemBypassAvailable;
    // The number of UPS's whose inverters are fully available
    uint16_t SystemInverterAvailable;
    // Output freuqency of UPS1
    float MasterOutputFrequency; 
    uint16_t MaxBatTempInCommon;
    uint16_t BatteryTemperature[MAX_NUM_UPS];
    uint16_t parallelSynchFlags;
    // Return a reference for the UpmData object that represents self.  Precondition:
    // The network must be initialized by a self call to ConfigureRxMailboxes().
    inline UpmState& MyUpmData(void)
    {
        return UpmData[MyUPSNumber][MyUPMNumber];
    }
    inline CalMeter& MyCalData(void)
    {
        return CalData[MyUPSNumber][MyUPMNumber];
    }
    
    inline const unSystemError& GetSystemError( void )
    {
        return ParallelSystemError;
    }
    
    inline const unParallelStatus& GetParallelStatus( void )
    {
        return ParallelStatus;
    }
    inline bool MasterUPM( void )
    {
        return ( MyUPMNumber == 0 );
    }
    inline bool UPSIsParallel( void )
    {
        return NumOfUPSs > 1   ||
               ( AllUPSsSinceReset & ~(1 << MyUPSNumber)) != 0;
    }
    inline void SetAutoID( void )
    {
        ParallelStatus.bit.AutoID = 1;
    }
    inline void ClearAutoID( void )
    {
        ParallelStatus.bit.AutoID = 0;
    }
    void ProcessBatteryTemperature( void );
    
protected:
	void UpdateBypassAvailable( void );
    
    virtual bool CanHardwareReady(void);
    virtual void ConfigureRxMailboxes(void);
    
    static void ReceiveInfoPacket(const MBOX& packet, const void *closure);
    static void ReceiveCommandPacket(const MBOX& packet, const void *closure);
    static void ReceiveStatusPacket(const MBOX& packet, const void *closure);
    static void ReceiveSyncPacket(const MBOX& packet, const void *closure);
    static void ReceiveRTCPacket(const MBOX& packet, const void *closure);
};

extern ParallelCanNetwork ParallelCan;

struct ParallelCanPacket : public MBOX
{
    static const uint_least8_t broadcast_addr = 0xfu;
    
    // Implicit copy constructor
    ParallelCanPacket(const MBOX& base)
    {
        MSGID.all = base.MSGID.all;
        MSGCTRL.all = base.MSGCTRL.all;
        MDL.all = base.MDL.all;
        MDH.all = base.MDH.all;
    }
    
    /*
     * Construct a CAN packet initialized with its priority and type.  By default,
     * the transmit UPM is self, and the receiving UPM is all. 
     */
    ParallelCanPacket( uint16_t type, uint16_t len )
    {
        MSGID.all = uint32_t(type & 0x1fff) << 16;
        MSGCTRL.all = 0;
        MDL.all = 0;
        MDH.all = 0;

        if ( len >= 4 )
        {
            len = 8;
        }
        else
        {
            len = len * 2u;
        }
        
        MSGID.bit.IDE = 1;
        MSGID.bit.AME = 1;
        MSGCTRL.bit.RTR = 0;
        MSGCTRL.bit.DLC = len;
        MSGCTRL.bit.TPL = 0;
        
        rx_ups(broadcast_addr);
        rx_upm(broadcast_addr);
        
        tx_ups(MyUPSNumber);
        tx_upm(MyUPMNumber);
        
        // Set hardware transmit priority based on the dominant 5 bits of the
        // CAN arbitration priority.  In the driver, higher numbers get transmitted first.
        // On the network, lower numbers are transmitted first.  Hence the bitwise NOT.
        MSGCTRL.bit.TPL = ~((id() >> 24) & 0x1fu);
    }
    
    /*
     * The ID portion of the packet
     */
    uint32_t id() const
    {
        return MSGID.all & 0x1fffffffUL;
    }

private:
    /*
     * Utility function for write access to a software-created bitfield
     * @param data Up to 8 data bits to set
     * @param bits The index of the least significant bit in the field.
     */
    ParallelCanPacket& set_subfield(uint_least8_t data, uint_least8_t offset)
    {
        data &= 0xf;
        // Set one of the four 4-bit subfields of a parallel can network packet
        const uint32_t mask = uint32_t(0xf) << offset;
        MSGID.all = (MSGID.all & ~mask) | (uint32_t(data) << offset);
        return *this;
    }
    
    /*
     * Utility function for read access to a software-created bitfield.
     * @param bit_len The width of the field
     * @param bit_offset The index of the least significant bit in the field.
     */
    uint16_t get_subfield(uint_least8_t bit_len, uint_least8_t bit_offset) const
    {
        uint32_t mask = (uint32_t(1UL << bit_len) - 1UL) << bit_offset;
        return (MSGID.all & mask) >> bit_offset;
    }
    
    /*
     * Transmitting UPS field.  Filled out by the driver.
     */
    ParallelCanPacket& tx_ups(uint_least8_t addr)
    {
        return set_subfield(addr, 12);
    }
    
    /*
     * Transmitting UPM field.  Filled out by the driver.
     */
    ParallelCanPacket& tx_upm(uint_least8_t addr)
    {
        return set_subfield(addr, 8);
    }
    
public:
    /*
     * Receiving UPS field.  Default: Broadcast
     */
    ParallelCanPacket& rx_ups(uint_least8_t addr)
    {
        return set_subfield(addr, 4);
    }
    
    /*
     * Receiving UPM field.  Default: Broadcast.
     */
    ParallelCanPacket& rx_upm(uint_least8_t addr)
    {
        return set_subfield(addr, 0);
    }
    
    /*
     * Read the receiving UPM
     */
    uint16_t rx_upm(void) const
    {
        return get_subfield(4, 0);
    }
    
    /*
     * Read the receiving UPS
     */
    uint16_t rx_ups(void) const
    {
        return get_subfield(4, 4);
    }
    
    /*
     * Read the transmitting UPM
     */
    uint16_t tx_upm(void) const
    {
        return get_subfield(4, 8);
    }
    
    /*
     * Read the transmitting UPS
     */
    uint16_t tx_ups(void) const
    {
        return get_subfield(4, 12);
    }
    
    /*
     * Read the type field of the packet.
     */
    uint16_t type(void) const
    {
        return get_subfield(13, 16);
    }
    
    
    /*
     * Set up to four of the packet's data words from an array
     */
    ParallelCanPacket& data(const uint16_t words[], uint16_t len)
    {
        switch (len) {
            case 4:
                data3(words[3]);
                // FALLTHROUGH
            case 3:
                data2(words[2]);
                // FALLTHROUGH
            case 2:
                data1(words[1]);
                // FALLTHROUGH
            case 1:
                data0(words[0]);
                break;
                
            default:
            // invalid value
            do { } while(false);
        }
        return *this;
    }
    
    /*
     * Set data word 0 of the packet.
     */
    ParallelCanPacket& data0(uint16_t data)
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
    ParallelCanPacket& data1(uint16_t data)
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
    ParallelCanPacket& data2(uint16_t data)
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
    ParallelCanPacket& data3(uint16_t data)
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
};

// global functions
void UpdateGlobalParallelBits(const size_t offset, const uint16_t len);
void UpdateGlobalParallelBits(void);
void ParallelSystemInitializeParams( void );
void ParallelSystem392us_Function( void );
void ParallelSystem5Msec( void );
void ParallelSystem20Msec( void );
void ParallelSystem100Msec( void );
void ParallelSystem500Msec( void );
void ParallelSystem1000Msec( void );
void InitUpsId( void );

#endif /*PARALLELCAN_H_*/
