#ifndef C9COMMANDS_H_
#define C9COMMANDS_H_

#include "CanIds.h"
#include "CanDriver.h"

#define ATE_AUTH_ID                     0xA500
#define ATE_PARAMS_START_ADDR           4096
#define ATE_PARAMS_END_ADDR             4099
#define ATE_CLEAR_HISTORY_ADDR          4100
#define ATE_RESET_EEPROM_ADDR           4101
#define ATE_SET_TIME_ADDR               4102           
#define ATE_SEND_COMMAND_ADDR           4103
#define ATE_SET_ECT_PARAMS_ADDR         4104
#define ATE_FCT_SET_ADC_ADDR            4105
#define ATE_MAX_ADDR                    ATE_FCT_SET_ADC_ADDR + 1
#define ATE_MAX_LENGTH                  ATE_MAX_ADDR - ATE_PARAMS_START_ADDR

#define BATTERY_LEG_A                   0
#define BATTERY_LEG_B                   1
class C9StateMachine
{
private:
	enum
	{
		IDLE,
		// Write request packets require buffering. No other packet may be
		// processed while handling a write packet.
		HANDLE_WRITE
	} state;
	
	// The time when the first write packet was observed, in OS ticks.
	uint32_t initialWritePacketTime;
	// The base address of the write request.
	uint32_t requestBaseAddress;
	// The length of the write request.
	uint16_t requestLength;
	
	// A bitmap of write request packets that have arrived. None of the packets
	// can be processed until all of them have arrived.
	uint32_t writeRequestMap;
	
	// Data storage for the write or read requests. This union allocates a
	// single buffer that is the larger of the two sizes. Since a read request
	// and write request cannot be simultaneously outstanding on the same port,
	// this works.
	union {
		uint16_t requestData[CAN_C9_TWRITE_MAX];
		uint16_t readData[CAN_C9_READ_MAX];
	} buffer;
	
//	uint16_t ATE_parameter[ATE_MAX_LENGTH];
//	uint16_t ATE_Status[ATE_MAX_LENGTH];       
	uint16_t ATE_ClearHistory(void);                  
	uint16_t ATE_ResetEeprom(void);                   
	uint16_t ATE_SendCommand(void);                   
	uint16_t ATE_FCT_SetAdcAddr(void);
	void ATE_SetParameters(void);
	uint16_t ATE_SetECTParams(void);                     
	void ATE_CMD_ENTER_TEST_MODE_func(void);      
    void ATE_CMD_EXIT_TEST_MODE_func(void);       
    void ATE_CMD_TURN_ON_PRECHARGER_func(void);   
    void ATE_CMD_TURN_OFF_PRECHARGER_func(void);  
    void ATE_CMD_TURN_ON_INVERTER_func(void);     
    void ATE_CMD_TURN_OFF_INVERTER_func(void);    
    void ATE_CMD_UPS_NORMAL_func(void);           
    void ATE_CMD_UPS_LOAD_OFF_func(void);         
    void ATE_CMD_UPS_BYPASS_ON_func(void);
    void ATE_CMD_UPM_BYPASS_ON_func(void);
//    void ATE_CMD_ENABLE_ECT_MODE_func(void);
//    void ATE_CMD_ENTER_LINE_ECT_MODE_func(void);       
//    void ATE_CMD_ENTER_BAT_ECT_MODE_func(void);
    void ATE_CMD_EXIT_ECT_MODE_func(void);
	void ATE_CMD_ENTER_ECT_MODE_func(void);         
    void ATE_CMD_START_PARA_AUTOCAL_func(void);
    void ATE_CMD_UPM_NORMAL_func(void);
    void ATE_CMD_UPM_LOAD_OFF_func(void); 
    void ATE_CMD_CAL_BATTERY_CURRENT_func(uint16_t leg);
    void ATE_CMD_CAL_BATTERY_CURRENT_EXIT_func(void);
    void ATE_CMD_ECT_RESTART_TIMES_func(void);
	void ATE_CMD_BATTERY_TEST_func(void);
	void checkWritePacket(uint16_t addressSpace);
	void checkWritePacketTimeout(void);
	void resetWritePacketMachine(void);
    void FCT_CMD_Enter_FCTMode_func( void );
    bool FCT_CMD_Receive_Condition ( void );
    void FCT_CMD_CHARGE_TEST_func  ( void );
    void FCT_CMD_INV_TEST_func     ( void );
    void FCT_CMD_INV_SHORT_TEST_func( void );
    void FCT_CMD_BOOST_TEST_func (void);
    void FCT_CMD_REC_TEST_func( void );
    void FCT_CMD_REC_SHORT_TEST_func( void );
    void FCT_CMD_LINEPMDISCHARGE_func( void );
    void FCT_CMD_CLEAR_RESULT_func( void );
    void FCT_QUIT_CURRENTFCT_TEST_func(void);
    void FCT_QUIT_FCTMODE_func(void);
    void FCT_CMD_ENABLE_BOARDID_func(void);
    
	CanDriver &can;
public:
    uint16_t ATE_parameter[ATE_MAX_LENGTH];
	uint16_t ATE_Status[ATE_MAX_LENGTH];   
	C9StateMachine(CanDriver& can);
	void insertMessage(const MBOX& msg);

protected:
	
	virtual void writeParameters();
	virtual void readParameters(uint32_t baseAddress, uint16_t length);
	
	virtual void writeVariables();
	virtual void readVariables(uint32_t baseAddress, uint16_t length);
	
	virtual void readProgramFlash(uint32_t baseAddress, uint16_t length);
	virtual void writeProgramFlash();
	
	virtual void readBootloaderFlash(uint32_t baseAddress, uint16_t length);
	virtual void writeBootloaderFlash();
	
	virtual void beginProgramFlash();
	virtual void beginBootloaderFlash();
	
	// Produce response packet to a write command. Either an error or success
	// flag may be returned.
	virtual void writeResponse(uint16_t space, uint32_t baseAddress, uint16_t length, uint16_t errorCode);
	virtual void readError(uint16_t space, uint32_t baseAddress, uint16_t length, uint16_t reason);
	virtual void completeFlashError(uint16_t space, uint16_t reason);
	virtual void beginFlashError(uint16_t space, uint16_t reason);
	virtual void prepareForFlash( uint16_t space );
	virtual uint16_t ATE_process(void);
};

extern C9StateMachine CsbC9StateMachine;

#endif /*C9COMMANDS_H_*/
