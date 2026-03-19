// ******************************************************************************************************
// *            IOexpansion.cpp
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
// *    FILE NAME: IOexpansion.cpp
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
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "IOexpansion.h"
#include "I2c_Driver.h"
#include "F28335Port.h"
#include "Spi_Driver.h"
#include "Spi_Task.h"
#include "Eeprom_Map.h"
#include "NB_Funcs.h"
#include "Version.h"

extern "C"
{
    void IdentifyModuleByHWID(void);
}

volatile uExpansionReg ExpansionInputReg;
volatile uExpansionReg ExpansionOutputReg;
volatile stPLDVersionReg PLDVersionReg;
volatile stPLDTrapReg PLDTrapReg;
volatile uint16_t PLDTrapBoot = 0;
volatile uPLDStatusReg  PLDStatusReg;
volatile uint16_t PLDFan1pwm = 255; //100% duty
volatile uint16_t PLDFanSTSpwm = 255; //100% duty
volatile uint16_t PLDFanAReg;
volatile uint16_t PLDFanBReg;
volatile uint16_t PLDFanCReg;

const uint16_t PLD_ADDR2_write_data = 0x0001;
const uint16_t AcceptableCNTLBoardID = 0x07;

uint16_t UpmModel = 0xFFFF;
bool DeadtimeConfEnd = false;

uint16_t CoefficientsIndex = 0;

bool clear_epo_enable = false;
extern bool PLD_DT_Init;

const uint16_t AcceptablePLDVersions[] =
{
    001,
    002,
    003,
    010,
};

extern void QueueSpiNoFeedB( uint16_t start_address,  uint16_t num_words, uint16_t* data_ptr, uint16_t op_type);

bool ClearExpansionIOReg( void)
{
    ExpansionOutputReg.all = 0;
    if( ControlBoardRevID != ControlBoardTPT29555 )
    {
        //    addr, len, dataPtr,        ,  device
        QueueSpiNoFeedB( 0,     1, (uint16_t*)&ExpansionOutputReg.all, IOEXPANDER_WRITE );
    }

    return true;
}
// ***********************************************************************
// *
// *    FUNCTION: ReadExpansionIOReg 
// *
// *    DESCRIPTION: Reads IO expansion, copies to RAM shadow register
// *        The read is only pended, there is no indication when it is complete.
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none 
// *
// ***********************************************************************
bool ReadExpansionIOReg( void )
{
	if( ControlBoardRevID == ControlBoardTPT29555 )
	{
		uint16_t data = 0xFFFF;
		return I2c_Write_Read(cDevice_TPT29555,cOperation_Read,&data);
	}
	else
	{
		//    addr, len, dataPtr,        ,  device
		QueueSpiNoFeedB( 0,     1, (uint16_t*)&ExpansionInputReg.all, IOEXPANDER_READ );
	    return true;
	}
}

// ***********************************************************************
// *
// *    FUNCTION: WriteExpansionIOReg 
// *
// *    DESCRIPTION: Write IO expansion, copies to RAM shadow register
// *        The write is only pended, there is no indication when it is complete.
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: none 
// *
// ***********************************************************************

void WriteExpansionIOReg( void )
{
	if( ControlBoardRevID != ControlBoardTPT29555 )
	{
		//    addr, len, dataPtr,        ,  device
        QueueSpiNoFeedB( 0,     1, (uint16_t*)&ExpansionOutputReg.all, IOEXPANDER_WRITE );
    }
}

// ***********************************************************************
// *
// *    FUNCTION: ReadPldVersion
// *
// *    DESCRIPTION: Read version number from PLD
// *        The read is only pended, there is no indication when it is complete.
// *    ARGUMENTS: none
// *
// *    RETURNS: none
// *
// ***********************************************************************
void ReadPldVersion(void)
{
    QueueSpiNoFeedB( PLD_ADDR_VER1,1, (uint16_t *)&PLDVersionReg, PLD_READ_VERSION  );
}

// ***********************************************************************
// *
// *    FUNCTION    : ReadPldTrap
// *
// *    DESCRIPTION : Read Trap code from PLD
// *
// *    ARGUMENTS   : none
// *
// *    RETURNS     : none 
// *
// ***********************************************************************
void ReadPldTrap(void)
{
    QueueSpiNoFeedB(PLD_ADDR_TRAP,     1, (uint16_t *)&PLDTrapReg.Trap, PLD_READ );

}
// ***********************************************************************
// *
// *    FUNCTION    : WritePldTrap
// *
// *    DESCRIPTION : Write Trap code to PLD. Here just clear the Trap Code.
// *
// *    ARGUMENTS   : none
// *
// *    RETURNS     : none 
// *
// ***********************************************************************
bool WritePldTrap(void)
{

	uint16_t data = ( 0x00 | UpmModel );
    return I2c_Write_Read(cDevice_PLD_TRAP,cOperation_Write,&data);
}

// ***********************************************************************
// *
// *    FUNCTION    : ClearEPOTrap
// *
// *    DESCRIPTION : Clear the PLD EPO latch.
// *
// *    ARGUMENTS   : none
// *
// *    RETURNS     : none 
// *
// ***********************************************************************
void ClearEPOTrap(void)
{
    // data must be constant, delayed write
    QueueSpiNoFeedB( PLD_ADDR_STAT2,     1, (uint16_t *)&PLD_ADDR2_write_data, PLD_WRITE );
}

// ***********************************************************************
// *
// *    FUNCTION: CheckValidPLDVersion 
// *
// *    DESCRIPTION: Checks PLD version for ones we allow
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: true = PLD version OK, false = PLD Version Not OK 
// *
// ***********************************************************************
bool CheckValidPLDVersion( void )
{
    bool PLDGood = false;

    for ( uint16_t idx = 0; idx < sizeof( AcceptablePLDVersions ); idx++ )
    {
        if ( PLDVersionReg.Version == AcceptablePLDVersions[ idx ] )
        {
            PLDGood = true;
            break;
        }
    }

    return PLDGood;
}

// ***********************************************************************
// *
// *    FUNCTION: CheckValidCntlVersion
// *
// *    DESCRIPTION: Checks CNTL board version for ones we allow
// *
// *    ARGUMENTS: none
// *
// *    RETURNS: true = CNTL version OK, false = CNTL Version Not OK
// *
// ***********************************************************************
bool CheckValidCntlVersion( void )
{
    bool CntlBoardGood = false;
    return true;
//    return CntlBoardGood = !ExpansionInputReg.bit.Board_ID;
}

// ***********************************************************************
// *
// *    FUNCTION: ResetLatchedEPO 
// *
// *    DESCRIPTION: Resets EPO latch in the PLD
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
bool ResetLatchedEPO( void )
{
    bool ret = clear_epo_enable;
    
    clear_epo_enable = false;
    
    return ret;
}

// ***********************************************************************
// *
// *    FUNCTION: ClearLatchedEPO 
// *
// *    DESCRIPTION: Resets EPO latch in the PLD
// *
// *    ARGUMENTS: 
// *
// *    RETURNS: 
// *
// ***********************************************************************
void ClearLatchedEPO( void )
{
    clear_epo_enable = true;
}

/*
// ***************************************************************************
// *
// *    FUNCTION    : PLDDeadBand_Conf
// *
// *    DESCRIPTION : This function try to configure dead band for PWM on PLD
//					  via cDevice_PLD_TRAP
//					  bit6 and bit5 of PLDTrapReg is UpmModel for distinguish
//					  Goldilocks HV_20K or HV_30K or other series
// *    ARGUMENTS   : none
// *
// *    RETURNS     : bool
// *
// ***************************************************************************
void PLDDeadBand_Conf(void)
{
	static uint16_t ConfProcess = 0;
	static uint16_t VerifyCnt = 0;
	uint16_t data = 0;

	switch(ConfProcess)
	{
		case 0:		//write process
			data = (( PLDTrapReg.all & 0xFF9F ) | UpmModel);
			if( I2c_Write_Read(cDevice_PLD_TRAP,cOperation_Write,&data) )
			{
				ConfProcess = 1;
			}
			break;

		case 1:		//read process
			data = 0xFFFF;
			if( I2c_Write_Read(cDevice_PLD_TRAP,cOperation_Read,&data) )
			{
				ConfProcess = 2;
			}
			break;

		case 2:		//verify process
			VerifyCnt++;
			if( (PLDTrapReg.all & 0x60) == UpmModel )
			{
				ConfProcess = 0;
				DeadtimeConfEnd = true;
				PLD_DT_Init = false;
			}
			else
			{
				ConfProcess = 0;
				if( VerifyCnt >= 3)
				{
					DeadtimeConfEnd = true;
					if(true == PLD_DT_Init )
					{
						EEStatusBits.bit.DeadBandConfFail = 1;
						PLD_DT_Init = false;
					}
					else
				    {
						NB_SetNodebit( UPM_NB_PLD_CONFIG_FAIL, true );
					}
				}
			}
			break;

		default:
			break;
	}
}
*/

// ***********************************************************************
// *
// *    FUNCTION    : ClearPldlTrap
// *
// *    DESCRIPTION : Write Trap code to PLD. Here just clear the Trap Code.
// *                  See SystemTrap.cpp that writes actual trap code.
// *    ARGUMENTS   : none
// *
// *    RETURNS     : none
// *
// ***********************************************************************
void ClearPldlTrap(void)
{
    PLDTrapReg.Trap =0;
    QueueSpiNoFeedB(PLD_ADDR_TRAP,     1,(uint16_t *) &PLDTrapReg.Trap, PLD_WRITE );
}

// ***********************************************************************
// *
// *    FUNCTION: IdentifModuleByHWID
// *
// *    DESCRIPTION: identify module type by hardware ID
// *
// *    ARGUMENTS:
// *
// *    RETURNS:
// *
// ***********************************************************************
void IdentifyModuleByHWID(void)
{
    DSPOutRegister.GpoB.bit.Driver_Detect = 0;
    DSPOutRegister.GpoC.bit.Supply_24VOff = 0;
}

// ***************************************************************************
// *    FUNCTION    : WritePldFan1
// *    DESCRIPTION : Fan dutyCount transfered to PLD
// *    ARGUMENTS   : duty = dutyCount/255
// *    RETURNS     : None
// ***************************************************************************
void WritePldFan1(uint16_t dutyCount)
{
    if( dutyCount > 255)
    {
        dutyCount = 255;
    }
    PLDFan1pwm = dutyCount | 0x0001; //bit0==0 PWM frequency 117k,bit0==1 PWM23.4k
//	    PLDFan1pwm = dutyCount & 0xfffe;
    QueueSpiNoFeedB( PLD_ADDR_FAN1,     1,(uint16_t *) &PLDFan1pwm, PLD_WRITE );
}

// ***************************************************************************
// *    FUNCTION    : WritePldFanSTS
// *    DESCRIPTION : Fan dutyCount transfered to PLD
// *    ARGUMENTS   : duty = dutyCount/255
// *    RETURNS     : None
// ***************************************************************************
void WritePldFanSTS(uint16_t dutyCount)
{
    if( dutyCount > 255)
    {
        dutyCount = 255;
    }
    PLDFanSTSpwm = dutyCount | 0x0001; //bit0==0 PWM frequency 117k,bit0==1 PWM23.4k

    QueueSpiNoFeedB( PLD_ADDR_FAN_STS,     1,(uint16_t *) &PLDFanSTSpwm, PLD_WRITE );
}

// ***************************************************************************
// *    FUNCTION    : ReadPldFanA ReadPldFanB ReadPldFanC
// *    DESCRIPTION : Read fan speed
// *    ARGUMENTS   : None
// *    RETURNS     : None
// ***************************************************************************
void ReadPldFanA(void)
{
    QueueSpiNoFeedB( PLD_ADDR_READFANA,     1,(uint16_t *) &PLDFanAReg, PLD_READ );
}

void ReadPldFanB(void)
{
    QueueSpiNoFeedB( PLD_ADDR_READFANB,     1,(uint16_t *) &PLDFanBReg, PLD_READ );
}

void ReadPldFanC(void)
{
    QueueSpiNoFeedB( PLD_ADDR_READFANC,     1,(uint16_t *) &PLDFanCReg, PLD_READ );
}

// ******************************************************************************************************
// *            End of IOexpansion.cpp
// ******************************************************************************************************

