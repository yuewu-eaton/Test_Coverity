// ******************************************************************************************************
// *            IOexpansion.h
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
// *    FILE NAME: IOexpansion.h
// *
// *    DESCRIPTION: 
// *
// *    ORIGINATOR:
// *
// *    DATE: 3/4/2010
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************

const uint16_t Rev5 = 0x01;
const uint16_t Rev7 = 0x02;

// I2C IO port expansion definitions
struct stExpansionReg
{
    uint16_t  CanID             : 2;    // HW INT CAN ID 1~2, 0:1
    uint16_t  InterfaceID       : 4;    // HW Board ID 1~4, 2:5
    uint16_t  FanFailSTS        : 1;    // STS fan fault detection
    uint16_t  /*unused*/        : 1;    // HW INT CAN ID 3

    uint16_t  /*unused*/        : 1;    // HW INT CAN ID 4
    uint16_t  /*unused*/        : 1;    // UPM.LOCK
    uint16_t  FanCheck_RevID_P6 : 1;    // FanFault, CONST_InterfaceBrdRev_ID_P6 == InterfaceBoardRevID
    uint16_t  Board_ID          : 1;    // HW Power Board ID
    
    uint16_t  EpoDisable        : 1;    // Disable EPO
//	    uint16_t  EPO_I2C         : 1;    // EPO.I2C
    uint16_t  Rev_ID3           : 1;    // HW UPM REV ID 3
    uint16_t  Rev_ID1           : 1;    // HW UPM REV ID 1
	uint16_t  Rev_ID2		    : 1;	  // HW UPM REV ID 2
};

struct uExpansionReg
{
    union
    {
        stExpansionReg  bit;
        uint16_t        all;
    };
    bool Ready; // true after the first read 
    bool Init;  // true after the first write
};

// I2C PLD Version Register Definitions
struct stPLDVersionReg
{
	uint16_t  Version;
	bool      Ready;
};

// I2C PLD Trap Register Definitions
struct stPLDTrapReg
{
    uint16_t  Trap;
    bool      Ready;
};

struct stPLDStatusReg
{                             
    uint16_t  MCU_UPMID     :1;
    uint16_t  rsvd1       	:1;
    uint16_t  UPM_Type_ID   :2;
                              
    uint16_t  PowerModuleOK :1;
    uint16_t  rsvd          :11;
};

struct uPLDStatusReg
{
    union
    {
    	stPLDStatusReg  bit;
        uint16_t        all;
    };
};

extern volatile uExpansionReg  ExpansionInputReg;
extern volatile uExpansionReg  ExpansionOutputReg;
extern volatile stPLDVersionReg PLDVersionReg;
extern volatile stPLDTrapReg    PLDTrapReg;
extern volatile uPLDStatusReg  PLDStatusReg;
extern volatile uint16_t PLDTrapBoot;
extern uint16_t UpmModel; 
extern bool DeadtimeConfEnd;
extern uint16_t CoefficientsIndex;
extern volatile uint16_t PLDFan1pwm;
extern volatile uint16_t PLDFanSTSpwm;
extern volatile uint16_t PLDFanAReg;
extern volatile uint16_t PLDFanBReg;
extern volatile uint16_t PLDFanCReg;


// function prototypes
bool ReadExpansionIOReg( void );
void WriteExpansionIOReg( void );
bool ClearExpansionIOReg( void );
void ReadPldVersion( void );
bool CheckValidPLDVersion( void );
bool CheckValidCntlVersion( void );
void ReadPldTrap( void );
void ClearEPOTrap( void );
bool ResetLatchedEPO( void );
void ClearLatchedEPO( void );
void PLDDeadBand_Conf(void);
void WritePldFan1(uint16_t dutyCount);
void ClearPldlTrap(void);
void WritePldFanSTS(uint16_t dutyCount);
void ReadPldFanA(void);
void ReadPldFanB(void);
void ReadPldFanC(void);

// ******************************************************************************************************
// *            End of IOexpansion.h
// ******************************************************************************************************
