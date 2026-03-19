
// ******************************************************************************************************
// *            EncryptionVry.cpp
// ******************************************************************************************************
// ******************************************************************************************************
// *
// * This information is proprietary to Eaton
// *
// ******************************************************************************************************
// *
// *  Copyright (c) 2013 Eaton
// *    ALL RIGHTS RESERVED
// *
// ******************************************************************************************************
// ******************************************************************************************************
// *    FILE NAME: EncryptionVry.cpp
// *
// *    DESCRIPTION:
// *
// *    Author: E9908825
// *
// *    DATE: 2013-12-10
// *
// *    HISTORY: See Subversion history.
// ******************************************************************************************************

// ******************************************************************************************************
// *            Include Files
// ******************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "EncryptionVry.h"
#include "I2c_Driver.h"
#include "Rtc.h"
#include "F28335Port.h"

//cEncrypt_Verify Encrypt_Verify_204;
cEncypt_Verify Encrypt_Verify_204;
//If Const_Key OR XorKey changed , must update EncryptionVry(or EncryptionVfy) also.
static const uint16_t Const_Key[SHA204_KEY_MAX/2]= { 0x0000, 0xACA1, 0xFF57, 0x4E40,
			                                         0xD445, 0x0104, 0x0EBD, 0xC6D3,
			                                         0xD373, 0xB8B7, 0x852D, 0xF3D9,
			                                         0xB513, 0xDA5E, 0x943D, 0x0000 };
static const uint16_t XorKey = 0x6CA2;

/*** SHA-XYZ INITIAL HASH VALUES AND CONSTANTS ************************/
/* Hash constant words K for SHA-256: */
static const uint32_t K256[64] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf,
		0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98,
		0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7,
		0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
		0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8,
		0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85,
		0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e,
		0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819,
		0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c,
		0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3, 0x748f82ee,
		0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
		0xc67178f2 };

/* Initial hash value H for SHA-256: */
static const uint32_t Initial_Hash[8] = { 0x6a09e667, 0xbb67ae85,
		0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
//Initial Hash values for SHA-224
static const uint32_t Initial_Hash_224[8] = { 0xc1059ed8, 0x367cd507,
		0x3070dd17, 0xf70e5939, 0xfc00b31, 0x68581511, 0x64f98fa7, 0xbefa4fa4 };

uint16_t ZeroValue[SHA204_ZONE_ACCESS_32/2] ={ 0 };
uint16_t RandOut[SHA204_RAND_BYTES/2] = { 0 };
uint16_t Challenge[NONCE_NUMIN_SIZE/2] = {0x7954,0x6570,0x4320,0x6168,0x6c6c,0x6e65,0x6567,0x4820,0x7265,0x0065};
uint16_t EncryptionTest = 0;
uint16_t Opcode = 0;
uint16_t Param1 = 0;
uint16_t Param2 = 0;
uint16_t NonceMode = 0;
uint16_t MacMode = 0;
uint16_t MacParam2 = 0;
// ********************************************************************************************************
// *
// * Function:     Class Constructor
// *
// * Purpose:      Initialize things
// *
// ********************************************************************************************************
cEncypt_Verify::cEncypt_Verify()
{
	ClearShortData(Command, SHA204_CMD_SIZE_MAX/2 + 1);
	ClearShortData(Response, SHA204_RSP_SIZE_MAX/2 + 1);
	ClearLongData(HashResult, SHA204_HASH_MAX/4);
	//HashResult = NULL;
	ClearLongData(Message, SHA256_MSG_MAX/4);
	ClearShortData(SerialNum, SHA204_SN_BYTES/2 + 1);
	ClearShortData(TempKey, SHA204_TEMPKEY_BYTES/2);

	pHashResult = (uint16_t *)HashResult;
	pMessage = (uint16_t *)Message;

	I2C_Block = false;
	Verify_Time = 0;
	Configure_Result = SHA204_Init_FAIL;
	InitKey(Key);
	MessageBytes = 0;

}

uint16_t cEncypt_Verify::Configure_204()
{
	EncryptionTest = 0;
	I2C_error_counters[7] = 0;
	uint16_t ret_code = 0;
	//Redefine Challenge with RTC random number
	for(uint16_t i = 0; i < (NONCE_NUMIN_SIZE / 2); i++)
	{
		if(i < sizeof(stRTC_Bcd_Time))
		{
			Challenge[i] = RTC_BcdTime.w[i] ^ i;
		}
		else
		{
			Challenge[i] = RTC_BcdTime.w[0] ^ i;
		}
	}


    //Wake up SHA-204 before operating.
    ret_code = Sha204p_Wakeup();
    if (ret_code != SHA204_SUCCESS)
    {
    	EncryptionTest |= ret_code;
    	return ret_code;
    }
    EncryptionTest = 0x0100;

    //Read Serial Number for SHA-204
    ret_code = Sha204m_Read(Command, Response, SHA204_ZONE_CONFIG | SHA204_ZONE_COUNT_FLAG, 0x0000 & SHA204_ADDRESS_MASK_CONFIG );
    if (ret_code != SHA204_SUCCESS)
    {
    	EncryptionTest |= ret_code;
    	return ret_code;
    }
    EncryptionTest = 0x0200;
    //Read Serial Number in Response
	for(uint16_t i = 0; i < SHA204_SN_BYTES; i++)
	{
		if(i < 4)
		{
			WriteBytetoWord(SerialNum, i, ReadByteinWord(Response, i + 1));
		}
		else
		{
			WriteBytetoWord(SerialNum, i, ReadByteinWord(Response, i + 5));
		}
	}

	//Use Nonce command to digest a TempKey in SHA-204 using SHA256 algorithm.
    NonceMode = NONCE_MODE_SEED_UPDATE;
	ret_code = Sha204m_Nonce(Command, Response, NonceMode, Challenge);
	if (ret_code != SHA204_SUCCESS)
	{
		EncryptionTest |= ret_code;
		return ret_code;
	}
	EncryptionTest = 0x0300;

	// Read out The Random Out from response
	for(uint16_t i = 0; i < SHA204_RAND_BYTES; i++)
	{
		WriteBytetoWord(RandOut, i, ReadByteinWord(Response, i + 1));
	}

	MacMode = MAC_MODE_BLOCK2_TEMPKEY | MAC_MODE_INCLUDE_SN;
	MacParam2 = DERIVE_KEY_TARGET;
	ret_code = Sha204m_Mac(Command, Response, MacMode, MacParam2, NULL);
	//ret_code = Sha204m_Mac(Command, Response, 0, 0, ChallengeNew);
	if (ret_code != SHA204_SUCCESS)
	{

		EncryptionTest |= ret_code;
		return ret_code;
	}
	ret_code = Sha204p_Sleep(Command);
	if (ret_code != SHA204_SUCCESS)
	{

		EncryptionTest |= ret_code;
		return ret_code;
	}
	EncryptionTest = 0x0400;

	// Start of encryption algorithm of Slot 9
	// Generate Parent Key
	UpdateKey(Key);
	// MessageBytes and Message need to be clear when use message
	MessageBytes = 0;
	ClearLongData(Message, SHA256_MSG_MAX/4);
	// Update TempKey : SN 0-9 and i
	for(uint16_t i = 0; i < (SHA204_TEMPKEY_BYTES/2); i++)
	{
		if( i < ( SHA204_SN_BYTES/2 + 1 ))
		{
			TempKey[i] = SerialNum[i];
		}
		else
		{
			TempKey[i] = i;
		}
	}
	Opcode = SHA204_DERIVE_KEY;
	Param1 = DERIVE_KEY_RANDOM_FLAG;
	Param2 = DERIVE_KEY_TARGET;
	Sha_Set_Message(MessageBytes, SHA204_KEY_MAX, Key); // 32B Parent Key
	Sha_Set_Message(MessageBytes, 1, &Opcode);          // 1B  Opcode
	//Sha_Set_Message(MessageBytes, 1, &Param1);          // 1B  Param1
	Sha_Set_Message(MessageBytes, 2, &Param2);          // 1B  Param2
	Sha_Set_Message(MessageBytes, 1, &SerialNum[4]);    // 1B  SN8
	Sha_Set_Message(MessageBytes, 2, &SerialNum[0]);    // 1B  SN0-1
	Sha_Set_Message(MessageBytes, 25, ZeroValue);       // 25B 0
	Sha_Set_Message(MessageBytes, SHA204_TEMPKEY_BYTES, TempKey);// 32B TempKey
	// Digest message with SHA256
	Swap32bitEndian(Message, SHA256_MSG_MAX/4);
	SHA_256(Message, 8 * (uint64_t)MessageBytes, HashResult, Mode_SHA256);
	//Swap32bitEndian(HashResult, SHA204_HASH_MAX/4);
	// Update Key use the Target Key
	for(uint16_t i = 0; i < (SHA204_KEY_MAX / 2) ; i++)
	{
		Key[i] = *(pHashResult + i);
	}
	// End of encryption algorithm of Slot 9

	// MessageBytes and Message need to be clear when use message
	MessageBytes = 0;
	ClearLongData(Message, SHA256_MSG_MAX/4);

	// Nonce 256 digest to produce the TempKey
	Opcode = SHA204_NONCE;
	Param1 = NonceMode;
	Param2 = 0;
	Sha_Set_Message(MessageBytes, SHA204_RAND_BYTES, RandOut);  //32B Random out
	Sha_Set_Message(MessageBytes, NONCE_NUMIN_SIZE, Challenge); //20B Challenge
	Sha_Set_Message(MessageBytes, 1, &Opcode);                  //1B  Opcode
	Sha_Set_Message(MessageBytes, 1, &Param1);                  //1B  Param1
	Sha_Set_Message(MessageBytes, 1, &Param2);                  //1B  Param2
	// Digest message with SHA256
	Swap32bitEndian(Message, SHA256_MSG_MAX/4);
	SHA_256(Message, 8 * (uint64_t)MessageBytes, HashResult, Mode_SHA256);
	Swap32bitEndian(HashResult, SHA204_HASH_MAX/4);

	// MessageBytes and Message need to be clear when use message
	MessageBytes = 0;
	ClearLongData(Message, SHA256_MSG_MAX/4);
	// MAC digest
	Opcode = SHA204_MAC;
	Param1 = MacMode;
	Param2 = MacParam2;
	Sha_Set_Message(MessageBytes, SHA204_KEY_MAX, Key);
	InitKey(Key); // Re init Key
	Sha_Set_Message(MessageBytes, SHA204_HASH_MAX, pHashResult); //32B HashResult
	Sha_Set_Message(MessageBytes, 1, &Opcode);                   //1B  Opcode
	Sha_Set_Message(MessageBytes, 1, &Param1);                   //1B  Param1
	Sha_Set_Message(MessageBytes, 2, &Param2);                   //1B  Param2
	Sha_Set_Message(MessageBytes, 8, ZeroValue);                 //8B OTP 0-7 or Bypass: 0
	Sha_Set_Message(MessageBytes, 3, ZeroValue);                 //3B OTP 8-10 or Bypass: 0
	Sha_Set_Message(MessageBytes, 1, &SerialNum[4]);             //1B SN 8
	Sha_Set_Message(MessageBytes, 4, &SerialNum[2]);             //4B SN 4-7
	Sha_Set_Message(MessageBytes, 2, &SerialNum[0]);             //2B SN 0-1
	Sha_Set_Message(MessageBytes, 2, &SerialNum[1]);             //2B SN 2-3
    //Digest message with SHA256
	Swap32bitEndian(Message, SHA256_MSG_MAX/4);
	SHA_256(Message, 8 * (uint64_t)MessageBytes, HashResult, Mode_SHA256);
	Swap32bitEndian(HashResult, SHA204_HASH_MAX/4);

	if(CheckResponse(Response, pHashResult))
	{
		Configure_Result = SHA204_SUCCESS;
		ret_code = SHA204_SUCCESS;
		EncryptionTest = 1;
	}
	else
	{
		Configure_Result = SHA204_DATA_FAIL;
        ret_code = SHA204_DATA_FAIL;
		EncryptionTest = 0;
	}

    return ret_code;
}

uint16_t cEncypt_Verify::Sha204p_Wakeup()
{
/*	uint16_t RxData[4] = { 0, 0, 0, 0 };
	bool bool_ret_code = I2c_Write_Read( cDevice_SHA204, cOperation_Write, &RxData[0], 4) ; // 4 bytes should be execute 80us since the clk is 400K
	I2caRegs.I2CSTR.bit.NACK = 1; // Write 1 to clear an NACK, which is received when wakeup undergoing.
	if(bool_ret_code == false)
	{
		return SHA204_RX_FAIL;
	}
*/
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;     //Set GPIO32 as a GPIO
	GpioCtrlRegs.GPBDIR.bit.GPIO32 |= 1;     //Set Output
	EDIS;

	DSPOutRegister.GpoB.bit.reservedB32 = 0; // Clear SDA
	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
	DELAY_US(60); //Dealy 60us for SHA204 to wakeup
	DSPOutRegister.GpoB.bit.reservedB32 = 1; // Set SDA
	GpioDataRegs.GPBSET.bit.GPIO32 = 1;

	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;     //Set GPIO32 as SDA
	EDIS;
	I2caRegs.I2CSTR.bit.NACK = 1; // Write 1 to clear an NACK

    TSK_sleep(SHA204_WAKEUP_DELAY); // delay 3 ms to wait SHA204 to wake
    //DELAY_US(SHA204_WAKEUP_DELAY * 1000ul);
    uint16_t ret_code = sha204p_receive_response( SHA204_RSP_SIZE_MIN, Response); // get response to ensure SHA204 is wakeup
    if( ret_code != SHA204_SUCCESS )
    {
    	return ret_code;
    }
	// Verify status response.
	if ( ReadByteinWord( Response , SHA204_RSP_POS_COUNT) != SHA204_RSP_SIZE_MIN) // Count of response in BYTE
	{
		ret_code = SHA204_INVALID_SIZE;
	}
	else if ( ReadByteinWord( Response, SHA204_BUFFER_POS_STATE ) != SHA204_STATUS_BYTE_WAKEUP) // Response state should be wake up
	{
		ret_code = SHA204_COMM_FAIL;
	}
	else
	{
		if (( ReadByteinWord( Response, ( SHA204_RSP_SIZE_MIN - SHA204_CRC_SIZE)) != 0x33)   // Check CRC
					|| ( ReadByteinWord( Response, ( SHA204_RSP_SIZE_MIN + 1 - SHA204_CRC_SIZE)) != 0x43))
		{
			ret_code = SHA204_BAD_CRC;
		}
	}

	return ret_code;
}

uint16_t cEncypt_Verify::sha204p_receive_response(uint16_t bcount, uint16_t *presponse)
{
	//uint32_t timeout = cI2c_Byte_Timeout * bcount;
	uint16_t RecieveCount = 0;
	bool bool_ret_code = I2c_Write_Read( cDevice_SHA204, cOperation_Read, presponse, bcount) ;
	if(bool_ret_code == false)
	{
		return SHA204_RX_FAIL;
	}
	RecieveCount = ReadByteinWord(Response, SHA204_RSP_POS_COUNT);
	if ((RecieveCount < SHA204_RSP_SIZE_MIN) || (RecieveCount > bcount))
	{
		return SHA204_INVALID_SIZE;
	}
	return 	SHA204_SUCCESS;
}

/** \brief This function sends a Read command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  zone 0: Configuration:0-87 Byte ; 1: OTP 0-63 Byte; 2: Data 0-511 Byte
 * \param[in]  byte address address to read from\n
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Read(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t zone, uint16_t address)
{
	uint16_t rx_size = 0;

	if (!tx_buffer || !rx_buffer || ((zone & ~READ_ZONE_MASK) != 0)                 // buffer is NOT NULL, and bit 2-5 of zone should be 0
				|| ((zone & READ_ZONE_MODE_32_BYTES) && (zone == SHA204_ZONE_OTP))) // OTP can't read 32 bytes one time
		return SHA204_BAD_PARAM;

/*	if (zone & SHA204_ZONE_DATA) // if zone is Data
	{
		address >>= 2;
		if (address & 1)
		{	// If we would just mask this bit, we would
			// read from an address that was not intended.
			return SHA204_BAD_PARAM;
		}
	}*/
	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX,  READ_COUNT);  // command consist of count(1 byte), opcode(1 byte),parameter 1(1byte), parameter 2(2 bytes) data, CRC(2 bytes)
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_READ);
	WriteBytetoWord(tx_buffer, READ_ZONE_IDX, zone);
	WriteBytetoWord(tx_buffer, READ_ADDR_IDX, (address & SHA204_ADDRESS_MASK)); // address no longer than 128 ( Read bytes are 4 or 32bytes one time )
	WriteBytetoWord(tx_buffer, (READ_ADDR_IDX + 1), 0 );

	rx_size = (zone & SHA204_ZONE_COUNT_FLAG) ? READ_32_RSP_SIZE : READ_4_RSP_SIZE;

	return Sha204c_Send_And_Receive(tx_buffer, rx_size, rx_buffer, READ_EXEC_MAX);
}

/** \brief This function runs a communication sequence:
 * Append CRC to tx buffer, send command, delay, and verify response after receiving it.
 *
 * The first byte in tx buffer must be the byte count of the packet.
 * If CRC or count of the response is incorrect, or a command byte got "nacked" (TWI),
 * this function requests re-sending the response.
 * If the response contains an error status, this function resends the command.
 *
 * \param[in] tx_buffer pointer to command
 * \param[in] rx_size size of response buffer
 * \param[out] rx_buffer pointer to response buffer
 * \param[in] execution_delay Start polling for a response after this many ms .
 * \param[in] execution_timeout polling timeout in ms
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204c_Send_And_Receive(uint16_t *tx_buffer, uint16_t rx_size, uint16_t *rx_buffer,
			uint16_t execution_delay)
{
	uint16_t ret_code = SHA204_FUNC_FAIL;
	uint16_t i = 0;
	uint16_t status_byte = 0;
	uint16_t count = ReadByteinWord( tx_buffer, SHA204_BUFFER_POS_COUNT );
	uint16_t count_minus_crc = count - SHA204_CRC_SIZE;

	// Append CRC.
	Sha204c_Calculate_CRC( count_minus_crc, tx_buffer );
	// Send command.
	if(I2c_Write_Read(cDevice_SHA204, cOperation_Write, tx_buffer, count + SHA204_SHIFT_IDX))// Add SHA204_SHIFT_IDX because of wordaddress.
	{
		ret_code = SHA204_SUCCESS;
	}
	else
	{
		ret_code = SHA204_RX_FAIL;
		return ret_code;
	}
	// Wait max command execution time and then start polling for a response.
	TSK_sleep(execution_delay);
	//DELAY_US(execution_delay * 1000ul);
	// Reset response buffer.
	for (i = 0; i < (rx_size / 2 + 1); i++) //rx_size is count in byte.
	{
		rx_buffer[i] = 0;
	}
	// Poll for response.
	ret_code = sha204p_receive_response(rx_size, rx_buffer);
	if (ret_code != SHA204_SUCCESS)
	{
		return ret_code;
	}
	// We received a response of valid size.
	// Check the consistency of the response.
	ret_code = Sha204c_Check_Crc(rx_buffer);
	if (ret_code == SHA204_SUCCESS)
	{
		// Received valid response.
		if (ReadByteinWord(rx_buffer, SHA204_RSP_POS_COUNT) > SHA204_RSP_SIZE_MIN)
		{	// Received non-status response. We are done.
			return ret_code;
		}
		// Received status response.
		status_byte = ReadByteinWord(rx_buffer, SHA204_BUFFER_POS_STATE);
		// Translate the three possible device status error codes
		// into library return codes.
		if (status_byte == SHA204_STATUS_BYTE_PARSE)
		{
			return SHA204_PARSE_ERROR;
		}
		else if (status_byte == SHA204_STATUS_BYTE_EXEC)
		{
			return SHA204_CMD_FAIL;
		}
		else if (status_byte == SHA204_STATUS_BYTE_COMM)
		{
			return SHA204_STATUS_CRC;
		}
		else
		{
            // Received status response from CheckMAC, DeriveKey, GenDig,
	        // Lock, Nonce, Pause, UpdateExtra, or Write command.
			ret_code = status_byte;
		    return ret_code;
		}
	}
	else  //If checkCRC is failed
	{
		return ret_code;
	} // block end of check response consistency
	//return ret_code;
}


/**\brief This function sends an UpdateExtra command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  mode 0: update Configuration zone byte 85 User Extra;   1: byte 86  Selector
 * \param[in]  new_value  to write
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Update_Extra(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode, uint16_t new_value)
{
	if (!tx_buffer || !rx_buffer || (mode > UPDATE_CONFIG_BYTE_86))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, UPDATE_COUNT);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_UPDATE_EXTRA);
	WriteBytetoWord(tx_buffer, UPDATE_MODE_IDX, mode);
	WriteBytetoWord(tx_buffer, UPDATE_VALUE_IDX, new_value);
	WriteBytetoWord(tx_buffer, UPDATE_VALUE_IDX + 1, 0);

	return Sha204c_Send_And_Receive(tx_buffer, UPDATE_RSP_SIZE, rx_buffer, UPDATE_EXEC_MAX );
}


// * \brief This function sends a Write command to the device.
// * \param[in]  tx_buffer pointer to transmit buffer
// * \param[out] rx_buffer pointer to receive buffer
// * \param[in]  zone 0: Configuration; 1: OTP; 2: Data
// * \param[in]  address address to write to\n
// * \param[in]  new_value pointer to 32 (zone bit 7 set) or 4 bytes of data
// * \param[in]  mac pointer to MAC (ignored if zone is unlocked)
// * \return status of the operation
// */
uint16_t cEncypt_Verify::Sha204m_Write(uint16_t *tx_buffer, uint16_t *rx_buffer,
			uint16_t zone, uint16_t address, const uint16_t *new_value, const uint16_t *mac)
{
	uint16_t Count = 0;
    uint16_t CpyCount = 0;
	if (!tx_buffer || !rx_buffer || !new_value || ((zone & ~WRITE_ZONE_MASK) != 0))
	{
		return SHA204_BAD_PARAM;
	}
/*	if (zone & SHA204_ZONE_DATA)
	{
		address >>= 2;
		if (address & 1)
		{	// If we would just mask this bit, we would
			// write to an address that was not intended.
			return SHA204_BAD_PARAM;
		}
	}*/

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_WRITE);
	WriteBytetoWord(tx_buffer, WRITE_ZONE_IDX, zone);
	WriteBytetoWord(tx_buffer, WRITE_ADDR_IDX, (address & SHA204_ADDRESS_MASK));
	WriteBytetoWord(tx_buffer, (WRITE_ADDR_IDX + 1), 0);
	Count = (zone & SHA204_ZONE_COUNT_FLAG) ? SHA204_ZONE_ACCESS_32 : SHA204_ZONE_ACCESS_4;
	for(CpyCount = 0; CpyCount < Count; CpyCount++)
	{
		WriteBytetoWord(tx_buffer, (WRITE_VALUE_IDX + CpyCount), ReadByteinWord(new_value, CpyCount));
	}
	//memcpy((tx_buffer + (WRITE_VALUE_IDX / 2) ), new_value, Count / 2);
	Count += WRITE_VALUE_IDX;

	if (mac != NULL)
	{
		for(CpyCount = 0; CpyCount < WRITE_MAC_SIZE; CpyCount++)
		{
			WriteBytetoWord(tx_buffer, (Count + CpyCount), ReadByteinWord(mac, CpyCount));
		}
		//memcpy(tx_buffer + (Count / 2), mac, WRITE_MAC_SIZE);
		Count += WRITE_MAC_SIZE;
	}

	// Supply count.
    WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, (Count - SHA204_SHIFT_IDX) + SHA204_CRC_SIZE);


	return Sha204c_Send_And_Receive(tx_buffer, WRITE_RSP_SIZE, rx_buffer, WRITE_EXEC_MAX);
}
/** \brief This function sends a Random command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  mode 0: update seed; 1: no seed update
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Random(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode)
{
	if (!tx_buffer || !rx_buffer || (mode > RANDOM_NO_SEED_UPDATE))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, RANDOM_COUNT);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_RANDOM);
	WriteBytetoWord(tx_buffer, RANDOM_MODE_IDX, mode);
	WriteBytetoWord(tx_buffer, RANDOM_PARAM2_IDX, 0);
	WriteBytetoWord(tx_buffer, RANDOM_PARAM2_IDX + 1, 0);

	return Sha204c_Send_And_Receive(tx_buffer, RANDOM_RSP_SIZE, rx_buffer, RANDOM_EXEC_MAX);
}

/** \brief This function sends a Nonce command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  mode controls the mechanism of the internal random number generator and seed update
 * \param[in]  numin pointer to system input\n
 *             (mode = 3: 32 bytes same as in TempKey;\n
 *              mode < 2: 20 bytes\n
 *              mode == 2: not allowed)
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Nonce(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t mode, const uint16_t *numin)
{
	uint16_t rx_size = 0;
    uint16_t Loop_i = 0;
	if (!tx_buffer || !rx_buffer || !numin
				|| (mode > NONCE_MODE_PASSTHROUGH) || (mode == NONCE_MODE_INVALID))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_NONCE);
	WriteBytetoWord(tx_buffer, NONCE_MODE_IDX, mode);
	WriteBytetoWord(tx_buffer, NONCE_PARAM2_IDX, 0);
	WriteBytetoWord(tx_buffer, NONCE_PARAM2_IDX + 1, 0);

	if (mode != NONCE_MODE_PASSTHROUGH)  // mode 0 or 1 / 20 bytes input
	{
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, NONCE_COUNT_SHORT);
		for(Loop_i = 0; Loop_i < NONCE_NUMIN_SIZE; Loop_i++)
		{
			WriteBytetoWord(tx_buffer, NONCE_INPUT_IDX + Loop_i, ReadByteinWord(numin, Loop_i));
		}
		rx_size = NONCE_RSP_SIZE_LONG;
	}
	else                                // mode 3 32 bytes input
	{
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, NONCE_COUNT_LONG);
		for(Loop_i = 0; Loop_i < NONCE_NUMIN_SIZE_PASSTHROUGH; Loop_i++)
		{
			WriteBytetoWord(tx_buffer, NONCE_INPUT_IDX + Loop_i, ReadByteinWord(numin, Loop_i));
		}
		rx_size = NONCE_RSP_SIZE_SHORT;
	}
	return Sha204c_Send_And_Receive(tx_buffer, rx_size, rx_buffer, NONCE_EXEC_MAX);
}

/** \brief This function sends a Lock command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  zone zone id to lock
 * \param[in]  summary zone digest
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Lock(uint16_t *tx_buffer, uint16_t *rx_buffer, uint16_t zone, uint16_t summary)
{
	if (!tx_buffer || !rx_buffer || ((zone & ~LOCK_ZONE_MASK) != 0)
				|| ((zone & LOCK_ZONE_NO_CRC) && (summary != 0)))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, LOCK_COUNT);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_LOCK);
	WriteBytetoWord(tx_buffer, LOCK_ZONE_IDX, zone & LOCK_ZONE_MASK);
	WriteBytetoWord(tx_buffer, LOCK_SUMMARY_IDX, summary & 0x00FF);
	WriteBytetoWord(tx_buffer, LOCK_SUMMARY_IDX + 1, summary >> 8);

	return Sha204c_Send_And_Receive(tx_buffer, LOCK_RSP_SIZE, rx_buffer, LOCK_EXEC_MAX);
}
/** \brief This I2C function puts the SHA204 device into idle state.
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204p_Idle(uint16_t *tx_buffer, uint16_t wakeup)
{
	uint16_t ret_code = SHA204_RX_FAIL;
	if (!tx_buffer )
	{
		return SHA204_BAD_PARAM;
	}
	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_IDLE);
	// Send command.
	if(I2c_Write_Read(cDevice_SHA204, cOperation_Write, tx_buffer, 1))// Add SHA204_SHIFT_IDX because of wordaddress.
	{
		ret_code = SHA204_SUCCESS;
	}
	else
	{
		ret_code = SHA204_RX_FAIL;
		return ret_code;
	}
	//Sleep PAUSE_EXEC_MAX = 2ms
	TSK_sleep(PAUSE_EXEC_MAX);
	//DELAY_US(PAUSE_EXEC_MAX * 1000ul);
	if(wakeup != 0)
	{
		//Wake up SHA-204 before operating.
		ret_code = Sha204p_Wakeup();
		if (ret_code != SHA204_SUCCESS)
		{
			EncryptionTest |= ret_code;
			return ret_code;
		}
	}

	return ret_code;
}
/** \brief This I2C function puts the SHA204 device into sleep state.
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204p_Sleep(uint16_t *tx_buffer)
{
	uint16_t ret_code = SHA204_RX_FAIL;
	if (!tx_buffer )
	{
		return SHA204_BAD_PARAM;
	}
	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_SLEEP);
	// Send command.
	if(I2c_Write_Read(cDevice_SHA204, cOperation_Write, tx_buffer, 1))// Add SHA204_SHIFT_IDX because of wordaddress.
	{
		ret_code = SHA204_SUCCESS;
	}
	else
	{
		ret_code = SHA204_RX_FAIL;
		return ret_code;
	}
	return ret_code;
}
/** \brief This function sends a MAC command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  mode selects message fields
 * \param[in]  key_id slot index of key
 * \param[in]  challenge pointer to challenge (not used if mode bit 0 is set)
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Mac(uint16_t *tx_buffer, uint16_t *rx_buffer,
			uint16_t mode, uint16_t key_id, const uint16_t *challenge)
{
	uint16_t Loop_i = 0;
	if (!tx_buffer || !rx_buffer || ((mode & ~MAC_MODE_MASK) != 0)
				|| (((mode & MAC_MODE_BLOCK2_TEMPKEY) == 0) && !challenge))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, MAC_COUNT_SHORT);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_MAC);
	WriteBytetoWord(tx_buffer, MAC_MODE_IDX, mode);
	WriteBytetoWord(tx_buffer, MAC_KEYID_IDX, key_id & 0x00FF);
	WriteBytetoWord(tx_buffer, MAC_KEYID_IDX + 1, key_id >> 8);

	if ((mode & MAC_MODE_BLOCK2_TEMPKEY) == 0)
	{
		for(Loop_i = 0; Loop_i < MAC_CHALLENGE_SIZE; Loop_i++)
		{
			WriteBytetoWord(tx_buffer, MAC_CHALLENGE_IDX + Loop_i, ReadByteinWord(challenge, Loop_i));
		}
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, MAC_COUNT_LONG);
	}

	return Sha204c_Send_And_Receive(tx_buffer, MAC_RSP_SIZE, rx_buffer, MAC_EXEC_MAX);
}

/** \brief This function sends a DeriveKey command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  random type of source key (0: slot; 1: parent key)
 * \param[in]  target_key slot index of key (0..15); not used if random is 1
 * \param[in]  mac pointer to optional MAC
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Derive_Key(uint16_t *tx_buffer, uint16_t *rx_buffer,
			uint16_t random, uint16_t target_key, const uint16_t *mac)
{
	uint16_t Loop_i = 0;
	if (!tx_buffer || !rx_buffer || ((random & ~DERIVE_KEY_RANDOM_FLAG) != 0)
				 || (target_key > SHA204_KEY_ID_MAX))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_DERIVE_KEY);
	WriteBytetoWord(tx_buffer, DERIVE_KEY_RANDOM_IDX, random);
	WriteBytetoWord(tx_buffer, DERIVE_KEY_TARGETKEY_IDX, target_key & 0x00FF);
	WriteBytetoWord(tx_buffer, DERIVE_KEY_TARGETKEY_IDX + 1, 0);

	if (mac != NULL)
	{
		for(Loop_i = 0; Loop_i < DERIVE_KEY_MAC_SIZE; Loop_i++)
		{
			WriteBytetoWord(tx_buffer, DERIVE_KEY_MAC_IDX + Loop_i, ReadByteinWord(mac, Loop_i));
		}
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, DERIVE_KEY_COUNT_LARGE);
	}
	else
	{
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, DERIVE_KEY_COUNT_SMALL);
	}

	return Sha204c_Send_And_Receive(tx_buffer, DERIVE_KEY_RSP_SIZE, rx_buffer, DERIVE_KEY_EXEC_MAX);
}

/** \brief This function sends a GenDig command to the device.
 * \param[in]  tx_buffer pointer to transmit buffer
 * \param[out] rx_buffer pointer to receive buffer
 * \param[in]  zone 1: OTP zone, 2: data zone
 * \param[in]  key_id zone 1: OTP block; zone 2: key id
 * \param[in]  other_data pointer to 4 bytes of data when using CheckOnly key
 * \return status of the operation
 */
uint16_t cEncypt_Verify::Sha204m_Gen_Dig(uint16_t *tx_buffer, uint16_t *rx_buffer,
			uint16_t zone, uint16_t key_id, const uint16_t *other_data)
{
	uint16_t Loop_i = 0;
	if (!tx_buffer || !rx_buffer
				|| ((zone != GENDIG_ZONE_OTP) && (zone != GENDIG_ZONE_DATA)))
		return SHA204_BAD_PARAM;

	if (((zone == GENDIG_ZONE_OTP) && (key_id > SHA204_OTP_BLOCK_MAX))
				|| ((zone == GENDIG_ZONE_DATA) && (key_id > SHA204_KEY_ID_MAX)))
		return SHA204_BAD_PARAM;

	WriteBytetoWord(tx_buffer, SHA204_ADDR_IDX, (uint16_t)SHA204_I2C_PACKET_FUNCTION_NORMAL);
	WriteBytetoWord(tx_buffer, SHA204_OPCODE_IDX, SHA204_GENDIG);
	WriteBytetoWord(tx_buffer, GENDIG_ZONE_IDX, zone);
	WriteBytetoWord(tx_buffer, GENDIG_KEYID_IDX, key_id);
	WriteBytetoWord(tx_buffer, GENDIG_KEYID_IDX + 1, 0);

	if (other_data != NULL)
	{
		for(Loop_i = 0; Loop_i < GENDIG_OTHER_DATA_SIZE; Loop_i++)
		{
			WriteBytetoWord(tx_buffer, GENDIG_DATA_IDX + Loop_i, ReadByteinWord(other_data, Loop_i));
		}
		WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, GENDIG_COUNT_DATA);
	}
	else
	{
	    WriteBytetoWord(tx_buffer, SHA204_COUNT_IDX, GENDIG_COUNT);
	}

	return Sha204c_Send_And_Receive(tx_buffer, GENDIG_RSP_SIZE, rx_buffer, GENDIG_EXEC_MAX);
}

// * \brief This function is Update Key
// * If This is changed, EncryptionVry or EncryptionVfy also Need Change.
// */

void cEncypt_Verify::UpdateKey(uint16_t *TagetKey)
{
	for(uint16_t i = 0; i < ( SHA204_KEY_MAX/2 ); i++)
	{
		*TagetKey = Const_Key[i] ^ (XorKey) ^ i;
		TagetKey++;
	}
	// MessageBytes and Message need to be clear when use message
	MessageBytes = 0;
	ClearLongData(Message, SHA256_MSG_MAX/4);
	Sha_Set_Message(MessageBytes, SHA204_KEY_MAX, Key);// 32B
	// Digest message with SHA256
	Swap32bitEndian(Message, SHA256_MSG_MAX/4);
	SHA_256(Message, 8 * (uint64_t)MessageBytes, HashResult, Mode_SHA256);
	Swap32bitEndian(HashResult, SHA204_HASH_MAX/4);
	// Update final Key
	for(uint16_t i = 0; i < (SHA204_KEY_MAX / 2) ; i++)
	{
		Key[i] = *(pHashResult + i);
	}
}
// * \brief This function is Init Key
// *
// */
void cEncypt_Verify::InitKey(uint16_t *TagetKey)
{
	for(uint16_t i = 0; i < ( SHA204_KEY_MAX/2 ); i++)
	{
		*TagetKey = Const_Key[i];
		TagetKey++;
	}
}
// * \brief This function calculates CRC.
// *
// * \param[in] length number of bytes in buffer
// * \param[in] data pointer to data for which CRC should be calculated
// */
void cEncypt_Verify::Sha204c_Calculate_CRC(uint16_t length, uint16_t *data)
{
	uint16_t counter = 0;
	uint16_t crc_register = 0;
	uint16_t polynom = 0x8005;
	uint16_t shift_register;
	uint16_t data_bit = 0;
	uint16_t crc_bit = 0;

	for (counter = 1; counter < (length + 1); counter++) // The first byte of data is wordaddress which don't need to be included in CRC
	{
	  for ( shift_register = 0x0001; (shift_register & 0x00FF) > 0x00; shift_register <<= 1)
	  {
		 data_bit = ( ( ReadByteinWord(data, counter) ) & shift_register) ? 1 : 0;
		 crc_bit = crc_register >> 15;

		 // Shift CRC to the left by 1.
		 crc_register <<= 1;

		 if ((data_bit ^ crc_bit) != 0)
		 {
			crc_register ^= polynom;
		 }
	  }
	}
	WriteBytetoWord(data, length+1, crc_register & 0x00FF ); // The first byte of data is wordaddress which don't need to be included in CRC
	WriteBytetoWord(data, length+2, crc_register >> 8 );
}


// * \brief This function checks the consistency of a response.
// * \param[in] response pointer to response
// * \return status of the consistency check
// */

uint16_t cEncypt_Verify::Sha204c_Check_Crc(const uint16_t *response)
{
	uint16_t ret_code = SHA204_SUCCESS;
	uint16_t Crc[SHA204_CRC_SIZE/2] = {0};
	uint16_t ShiftCounter = 0;
	uint16_t Count = ReadByteinWord(response, SHA204_RSP_POS_COUNT);
	uint16_t crc_register = 0;
	uint16_t polynom = 0x8005;
	uint16_t shift_register = 0;
	uint16_t data_bit = 0;
	uint16_t crc_bit = 0;

	Count -= SHA204_CRC_SIZE;
	for (ShiftCounter = 0; ShiftCounter < Count; ShiftCounter++) // In response the first byte is Count of the package, so CRC start with the first byte
	{
	  for ( shift_register = 0x0001; (shift_register & 0x00FF) > 0x00; shift_register <<= 1)
	  {
		 data_bit = ( ( ReadByteinWord(response, ShiftCounter) ) & shift_register) ? 1 : 0;
		 crc_bit = crc_register >> 15;

		 // Shift CRC to the left by 1.
		 crc_register <<= 1;

		 if ((data_bit ^ crc_bit) != 0)
		 {
			crc_register ^= polynom;
		 }
	  }
	}
	WriteBytetoWord(Crc, 0, crc_register & 0x00FF );
	WriteBytetoWord(Crc, 1, crc_register >> 8 );

	for( ShiftCounter = 0; ShiftCounter < SHA204_CRC_SIZE; ShiftCounter++)
	{
		if( ( ReadByteinWord(Crc, ShiftCounter) != ReadByteinWord(response, Count + ShiftCounter) ) &&
			( ret_code == SHA204_SUCCESS))
		{
			ret_code = SHA204_BAD_CRC;
		}
	}

	return ret_code;
}
bool cEncypt_Verify::CheckResponse(const uint16_t *pResponse, const uint16_t *pHashResultCheck)
{
	for(uint16_t i = 0; i < SHA204_HASH_MAX; i++)
	{
		if(ReadByteinWord(pHashResultCheck, i) != ReadByteinWord(pResponse, i+1)) // If Response and HashResult don't match
		{
			return false; // Check Response failed.
		}
	}
	return true; // Check Response success.
}

// * \brief This function Clear data.
// * \param[in] *ShortData :data pointer to Short(16 bits) data
// * \param[in] DataNum:   Number of data to clear
// */
void cEncypt_Verify::ClearShortData(uint16_t *ShortData, uint16_t DataNum)
{
	if( ShortData != NULL)
	{
		for(uint16_t i = 0; i < DataNum; i++)
		{
			ShortData[i] = 0;
		}
	}
}
// * \brief This function Clear data.
// * \param[in] *LongData :data pointer to Long(32 bits) data
// * \param[in] DataNum:   Number of data to clear
// */
void cEncypt_Verify::ClearLongData(uint32_t *LongData, uint16_t DataNum)
{
	if( LongData != NULL)
	{
		for(uint16_t i = 0; i < DataNum; i++)
		{
			LongData[i] = 0;
		}
	}
}


void cEncypt_Verify::Sha_Set_Message(uint16_t &MsgBytes, uint16_t SetBytes, const uint16_t *SourceMsg)
{
	for(uint16_t i = MsgBytes; i < (MsgBytes + SetBytes); i++)
	{
		WriteBytetoWord(pMessage, i, ReadByteinWord(SourceMsg, (i - MsgBytes)));
	}
	MsgBytes += SetBytes;
}


/** ReadByteinWord: read ByteNum byte from Sourceword
 *
 * \param[in] SourceWord point to source word
 * \param[in] ByteNum number of word to be read from 0 bit
 *
 */
uint16_t ReadByteinWord(const uint16_t * SourceWord, uint16_t ByteNum )
{
	uint16_t ReadByteData = 0;
	SourceWord += ( ByteNum / 2 );
	if( ByteNum % 2 != 0 ) // If ByteNum is an odd value use High Byte of word
	{
		ReadByteData = (*SourceWord) >> 8;
	}
	else                  // If even value   use Low Byte of word
	{
		ReadByteData = (*SourceWord) & 0x00FF;
	}
	return ReadByteData;
}
/** WriteBytetoWord: write ByteNum byte to Sourceword
 *
 * \param[in] SourceWord point to source word
 * \param[in] ByteNum number of word to be write from 0 bit
 * \param[in] ByteData data to be write
 */
void WriteBytetoWord(uint16_t *SourceWord, uint16_t ByteNum, uint16_t ByteData)
{
	SourceWord += ( ByteNum / 2 );
	if( ByteNum % 2 != 0 ) // If ByteNum is an odd value use High Byte of word
	{
		*SourceWord &= 0x00FF; // Clear high byte of SourceWord
		*SourceWord |= ( ByteData << 8 ); // Use ByteData to write high byte of SourceWord
	}
	else                  // If even value   use Low Byte of word
	{
		*SourceWord &= 0xFF00; // Clear low byte of SourceWord
		ByteData &= 0x00FF;
		*SourceWord |= ( ByteData ); // Use ByteData to write low byte of SourceWord
	}
}
/*===================================================================
// BRIEF: Converse Endian based on uint32_t words (bigEndian to smallEndian,  or smallEndian to bigEndian )
//
// INPUTS:     uint32_t *SwapMessage : messages(32 bits) that need to be change into different endian
//		       uint16_t SwapWords	:  numbers of message to be change.
// OUTPUTS:  None
//
//==================================================================*/
void Swap32bitEndian(uint32_t *SwapMessage, uint16_t SwapWords)
{
	 uint32_t SwapWord = 0;
     for(uint16_t i = 0; i < SwapWords; i++)
     {
    	 SwapWord =  (*SwapMessage & 0x000000fful) << 24;
    	 SwapWord |= (*SwapMessage & 0x0000ff00ul) << 8;
    	 SwapWord |= (*SwapMessage & 0x00ff0000ul) >> 8;
    	 SwapWord |= (*SwapMessage & 0xff000000ul) >> 24;
    	 *SwapMessage = SwapWord;
    	 SwapMessage++;
     }
}

/*===================================================================
// NAME: void SHA_256 ( uint32_t *Message, uint64_t Mbit_Length, uint32_t *Hash);
//
// BRIEF: Is used to execute SHA-256 hashing algorithm on the data referenced by Message.
//        *Hash will contain the final hashing at completion of function.
//
// INPUTS:       uint32_t *Message -- Pointer to array of 32-bit long to be hashed. Size of array must be a multiple of a hashing block.( I.e. 512 bits or sixteen 32-bit longs)
//		                              use Big-endian to store message
//                         uint64_t Mbit_Length --  64-bit value containing the precise number of
//					       bits to be hashed within Message[].
//				          **Note: If Mbit_Length %(mod) 512 is >= 448 bits, then
//					      an additional hashing block is needed. User
//					      must allocate the additional 512 bits
//		        uint32_t *Hash	--	pointer to hashing array. Final hash will be stored here.
//				       	  size of array should equal 8 32-bit longs
//		        short  mode	--	If Mode =='0', SHA-224 is used, all else SHA-256
//
// OUTPUTS:      results stored at given pointers. Final Hash stored at Hash pointer.
//
// Process:
//
// Note:
//
//==================================================================*/

/* This code being developed to implement SHA-244/256 on the MSP430.
* This code is by no means  optimized as of this moment.
* The object is to develop an understandable implementation of SHA-2 on the MSP430
* The algorithm will be used as a function call with inputs being a pointer to the message
* needing encryption, the length of the message in longs and a pointer to the
* Hash (size of 8 longs) array in which will contain the answer after the function is done.
*/
/*Function*/
void SHA_256 ( uint32_t* const Message, uint64_t Mbit_Length, uint32_t *Hash, short mode)
{

/*Variable Declarations go here*/
    unsigned int leftoverlong = 0;
    unsigned int leftoverbits = 0;
    unsigned char ChunkOffset = 0;
    uint32_t a=0;
    uint32_t b=0;
    uint32_t c=0;
    uint32_t d=0;
    uint32_t e=0;
    uint32_t f=0;
    uint32_t g=0;
	uint32_t h=0;
	uint64_t Nblocks = 0;
	uint32_t temp1=0;
	uint32_t temp2=0;
	uint32_t W[64] = {0};  //for Speed opt.
	unsigned int i= 0;
	unsigned int p =0;
	unsigned int v =0;
	unsigned int t =0;
	uint64_t M_Length;
	uint32_t mask = 0;
	//uint32_t W[16] = {0};   // For Code Size opt.  */ //With IAR Optimizations, this method does not improve codesize


	/* Pre-processing:
	* 1. Initialize Hash Values 2. Parse the Message block 3. Padd the message block*****/
	if( mode==0)
	{
		 for (i=0;i<=7; i++)
		 {
			 Hash[i] = Initial_Hash_224[i];
		 }  // Initialize Hash for SHA-224
	}
	else
	{
		 for (i=0;i<=7; i++)
		 {
			 Hash[i] = Initial_Hash[i];
		 }  // Initialize Hash for SHA-256
	}
	i = 0;  //clear i

	/* Message Parsing */
	M_Length = Mbit_Length >> 5;    // Converting Bit length of message to How many longs in a message
	Nblocks = M_Length >> 4;         // Number of whole buckets (512 bits or 16 32-bit buckets)
	leftoverlong = M_Length % 16; // leftover longs not in a full bucket
	leftoverbits = Mbit_Length % 32;  // leftover bits in last long

	/* Message Padding: The next set of statements finds the end of a message, appends a 1, then adds 0's
	* to pad the message to a 512bit chunk. The length of the original message is parsed into the last 2 bytes**/

	if (leftoverbits == 0)
	{  				   //Check to see if last 32bit long is full of data
		Message[lastchunk + leftoverlong ] = 0x80000000; // Message[lastchunk + leftoverlong + 1] = 0x80000000;
	}
	else
	{
		//Last long is not full
	    mask = (0xFFFFFFFF >> (leftoverbits));

 	    if(leftoverlong == 0)
 	    {
 		    Message[lastchunk ] = (Message[lastchunk ] | mask) & (~(mask >> 1)); // append one to last bit
 	    }
 	    else
 	    {
 		    Message[lastchunk + leftoverlong] = (Message[lastchunk + leftoverlong ] | mask) & (~(mask >>1)); // append one to last bit
 	    }
    }
	if ((Mbit_Length % 512) < 448)//Check to see if a new block (chunk) is needed
	{
		// no new chunk needed
		for(v=1; v < (14-leftoverlong); v++)
		{
			Message[lastchunk + leftoverlong + v] &= 0x00000000; // zero pad
		}
		Message[lastchunk + 14]= Mbit_Length >> 32;   //append bit length to end of chunk
		Message[lastchunk + 15] = Mbit_Length & 0x00000000ffffFFFF;
		ChunkOffset =0;
	}
	else
	{
		//new chunk needed
		for (p=1; p < (16-leftoverlong); p++)
		{
			Message[lastchunk +leftoverlong +p] = 0x00000000; // zero out remaining bits in chunk
		}
		for (p=0; p <14; p++)
		{
			Message[lastchunk + 16 + p] = 0x00000000;   //zero out next chunk
		}
		Message[lastchunk + 30]= Mbit_Length >> 32;   //append bit length to end of chunk
		Message[lastchunk + 31] = Mbit_Length & 0x00000000ffffFFFF;
		ChunkOffset = 1;
	}
	/** End Pre-Processing  **/

    /** Main algorithm  **/
    /* Chunk control. Process 512 bits at a time*/
    do{
		/*Place i-1 Hash into letters. Initialize with initial hash values.*/
		a = Hash[0];
		b = Hash[1];
		c = Hash[2];
		d = Hash[3];
		e = Hash[4];
		f = Hash[5];
		g = Hash[6];
		h = Hash[7];

		for (t=0; t < 64; t++)
		{ // need to change to do/while loop.

			//W calculation. Can optimize for speed or codesize
			//speed opt. Assume W[64]
			if(t >= 16)
			{
				W[t] = sigma1(W[(t-2)]) + W[(t-7)] + sigmaZ(W[(t-15)]) + W[(t-16)]; //000000000000000000000000000000
			}
			else
			{
				W[t] = Message[ 16*i + t ];
			}
			/*
			// Code size opt. Assume W[16]   //With IAR Optimizations, this method does not improve codesize
			if (t < 16 ) {
					W[t] = Message[ 16*i + t ];
					}
			else {
					W[t%16] = sigma1(W[(t-2)%16]) + W[(t-7)%16] + sigmaZ(W[(t-15)%16]) + W[(t-16)%16];
					}
			*/

			// Algorithm Proper
			temp1 = h + SIG1(e) + Ch(e, f, g) + K256[t] + W[t];     // for speed opt.
			 //temp1 = h + SIG1(e) + Ch(e, f, g) + K256[t] + W[t%16];  //for code size opt. //With IAR Optimizations, this method does not improve codesize
			temp2 = Maj(a, b, c) +SIGZ(a);
			h = g;
			g = f;
			f = e;
			e = d +temp1;
			d = c;
			c = b;
			b = a;
			a = temp1 + temp2;
		}
		Hash[0] = Hash[0] + a;
		Hash[1] = Hash[1] + b;
		Hash[2] = Hash[2] + c;
		Hash[3] = Hash[3] + d;
		Hash[4] = Hash[4] + e;
		Hash[5] = Hash[5] + f;
		Hash[6] = Hash[6] + g;
		Hash[7] = Hash[7] + h;

		i++;
   }while( i <= (Nblocks + ChunkOffset) );
}

