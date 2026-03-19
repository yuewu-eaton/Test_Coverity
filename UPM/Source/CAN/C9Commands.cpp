// ********************************************************************************************************
// ********************************************************************************************************
// *
// * THIS INFORMATION IS PROPRIETARY TO EATON Corporation
// *
// ********************************************************************************************************
// *
// *  Copyright (c) 2010 Eaton
// *    ALL RIGHTS RESERVED
// *
// ********************************************************************************************************
// ********************************************************************************************************
// *    FILE NAME: C9Commands.cpp
// *
// *    DESCRIPTION: Contains remote memory handling functions
// *
// *    ORIGINATOR: Jonathan Brandmeyer
// *
// *    DATE: June 10, 2010
// *
// *    HISTORY: See revision control system's history.
// *********************************************************************************************************

// *********************************************************************************************************
// *        INCLUDE FILES
// *********************************************************************************************************
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <cstring>
#include <clk.h>

#include "C9Commands.h"
#include "Queue.h"
#include "CanIds.h"
#include "Eeprom_Map.h"
#include "Constants.h"
#include "Spi_Task.h"
#include "DebuggerBlocks.h"
#include "InternalCan.h"
#include "BootloaderAPI.h"
#include "MCUState.h"
#include "BatteryStateControl.h" 
#include "BTR.h"
#include "HQ_Funcs.h"
#include "NB_Funcs.h"
#include "InverterControl.h"
#include "MCUState.h"
#include "AutoCal.h"
#include "adc.h"
#include "FCTState.h"
#include "Alarms.h"

using namespace std;

enum
{
    ATE_CMD_ENTER_TEST_MODE             = 1,
    ATE_CMD_EXIT_TEST_MODE              = 2,
    ATE_CMD_TURN_ON_PRECHARGER          = 3,
    ATE_CMD_TURN_OFF_PRECHARGER         = 4,
    ATE_CMD_TURN_ON_INVERTER            = 5,
    ATE_CMD_TURN_OFF_INVERTER           = 6,
    ATE_CMD_UPS_NORMAL                  = 7,
    ATE_CMD_UPS_LOAD_OFF                = 8,
    ATE_CMD_UPS_BYPASS_ON               = 9,
    ATE_CMD_EXIT_ECT_MODE               = 10,
	ATE_CMD_ENTER_ECT_MODE              = 11,
    ATE_CMD_START_PARA_AUTOCAL          = 13,
    ATE_CMD_UPM_NORMAL                  = 14,
    ATE_CMD_UPM_LOAD_OFF                = 15,
    ATE_CMD_UPS_CHARGE_OFF              = 16,
    ATE_CMD_UPS_CHARGE_ON               = 17,
    ATE_CMD_UPS_ESS_TIME_TEST           = 18,
    ATE_CMD_UPS_ESS_TEST_EXIT           = 19,
	ATE_CMD_CAL_BATTERY_CURRENT_LEG_A   = 20,
    ATE_CMD_CAL_BATTERY_CURRENT_LEG_B   = 21,
    ATE_CMD_CAL_BATTERY_CURRENT_EXIT    = 22,
    ATE_CMD_ECT_RESTART_TIMES           = 23,	
    ATE_CMD_BATTERY_TEST                = 30, //Auto test command should be the same for other machines.
    ATE_CMD_UPM_BYPASS_ON               = 31, //hobbit 2*upm on
    ATE_CMD_UPS_5V_ALARM_MASK           = 32,
    ATE_CMD_UPS_5V_ALARM_BACK           = 33,

    FCT_CMD_Enter_FCTMode               = 300,
    FCT_CMD_CHARGE_TEST                 = 301,
    FCT_CMD_INV_TEST                    = 302,
    FCT_CMD_INV_SHORT_TEST              = 303,
    FCT_CMD_BOOST_TEST                  = 304,
    FCT_CMD_REC_TEST                    = 305,
    FCT_CMD_REC_SHORT_TEST              = 306,
    FCT_CMD_LINEPMDISCHARGE             = 307,
    FCT_CMD_CLEAR_RESULT                = 308,
    FCT_QUIT_CURRENTFCT_TEST            = 309,
    FCT_QUIT_FCTMODE                    = 310,
    FCT_CMD_ENABLE_BOARDID              = 311
};

// Global storage
C9StateMachine CsbC9StateMachine(InternalCan);

uint16_t swap(uint16_t data)
{
    uint16_t temp;
    
    temp = (data << 8) | (data >> 8);
    
    return(temp);    
}

C9StateMachine::C9StateMachine(CanDriver& can)
    : state(IDLE)
    , initialWritePacketTime(0)
    , requestBaseAddress(0)
    , requestLength(0)
    , writeRequestMap(0)
    , can(can)
{
    memset(&buffer.requestData, 0, sizeof(buffer));
    
    for(uint16_t cnt = 0; cnt < ATE_MAX_LENGTH; cnt++)
    {
        ATE_parameter[cnt] = 0;
        ATE_Status[cnt] = 0;   
    }
}

void C9StateMachine::insertMessage(const MBOX& _msg)
{
    InternalCanPacket msg(_msg);
    
    uint16_t type = msg.type() & ~CAN_C9_SPACE_MASK;
    uint16_t space = msg.type() & CAN_C9_SPACE_MASK;
        
    switch (type)
    {
        case CAN_C9_BEGIN_FLASH_REQUEST:
             prepareForFlash( space );
             switch (space)
             {
                case CAN_C9_SPACE_PROG_FLASH:
                Transfer2Bootloader( space );
                break;
            
                case CAN_C9_SPACE_BOOT_FLASH:
                Transfer2Bootloader( space );
                break;
                
                default:
                    break;
             }
             
             // If you didn't transfer to bootloader, something is wrong
             beginFlashError(space, CAN_C9_ERROR_INVALID_STATE);
             break;
        
        case CAN_C9_COMPLETE_FLASH_REQUEST:
             completeFlashError(space, CAN_C9_ERROR_INVALID_STATE);
             break;
    
        case CAN_C9_READ_REQUEST:
             {
                uint32_t baseAddress = uint32_t(msg.data0()) << 16 | msg.data1();
                uint16_t length = msg.data2();
        
                switch (space)
                {
                    case CAN_C9_SPACE_PROG_FLASH:
                         readProgramFlash(baseAddress, length);
                         break;
               
                    case CAN_C9_SPACE_BOOT_FLASH:
                         readBootloaderFlash(baseAddress, length);
                         break;
               
                    case CAN_C9_SPACE_VARS:
                         readVariables(baseAddress, length);
                         break;
               
                    case CAN_C9_SPACE_PARAMS:
                         readParameters(baseAddress, length);
                         break;
               
                    default:
                         readError(space, baseAddress, length, CAN_C9_ERROR_INVALID_PACKET);
                }
                break;
             }
             
        case CAN_C9_TWRITE_BEGIN:
             {
                checkWritePacketTimeout();
                requestBaseAddress = uint32_t(msg.data0()) << 16 | msg.data1();
                requestLength = msg.data2();
                checkWritePacket(space);
                break;
             }
        
        default:
             // check for TWRITE data block
             if (type >= CAN_C9_TWRITE_DATA_BASE && 
                 type < CAN_C9_TWRITE_DATA_BASE + CAN_C9_TWRITE_MAX/4)
             {
                // process write data payload packet
                checkWritePacketTimeout();
                size_t base = type - CAN_C9_TWRITE_DATA_BASE;
                size_t words = (msg.subtype() & CAN_C9_TWRITE_DATA_COUNT_MASK) 
                               >> CAN_C9_TWRITE_DATA_COUNT_BITS;
                if (words <= 4)
                {
                    uint16_t data[4] = { msg.data0(), msg.data1(), msg.data2(), msg.data3() };
                    for (size_t i = 0; i < words; ++i)
                    {
                        buffer.requestData[base*4 + i] = data[i];
                    }
                    // Record the write packet in the map.
                    writeRequestMap |= 1L << base;
                }
                else
                {
                    // TODO: Invalid packet.
                }
                checkWritePacket(msg.type() & CAN_C9_SPACE_MASK);
             }
             else
             {
                 // TODO: Unknown packet type field.
             }
             break;
    }
}

void C9StateMachine::writeParameters()
{
    uint16_t errorCode;
    if (requestBaseAddress + requestLength <= MAX_EE_LENGTH)
    {
        if (!PutEepData((uint16_t)requestBaseAddress, requestLength, 
            buffer.requestData, TSK_1000_ms) == 0)
        {
            errorCode = CAN_C9_ERROR_SUCCESS;
        }
        else
        {
            errorCode = CAN_C9_ERROR_INVALID_VALUE;
        }
    } 
    else if (requestBaseAddress + requestLength <= ATE_MAX_ADDR)
    {
        errorCode = ATE_process();
        errorCode = CAN_C9_ERROR_SUCCESS;
    }
    else
    {
        errorCode = CAN_C9_ERROR_INVALID_ADDRESS;
    }
    writeResponse(CAN_C9_SPACE_PARAMS, requestBaseAddress, requestLength, errorCode);
}

void C9StateMachine::readParameters(uint32_t baseAddress, uint16_t length)
{
    memset(&buffer, 0x00, sizeof(buffer));

    if( baseAddress + length <= MAX_EE_LENGTH )
    {
        if (GetEepData(baseAddress, length, buffer.readData, TSK_1000_ms))
        {
            for (int word = 0; word < length; word += 4)
            {
                int words = (length - word >= 4) ? 4 : (length - word);
                can.PacketTx(InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + word/4) | CAN_C9_SPACE_PARAMS, 
                        CAN_C9_PRIORITY)
                    .subtype(can.UPMNumber() | (words << CAN_C9_TWRITE_DATA_COUNT_BITS))
                    .data0(buffer.readData[word])
                    .data1(buffer.readData[word+1])
                    .data2(buffer.readData[word+2])
                    .data3(buffer.readData[word+3]));
            }
        }
        else
        {
            readError(CAN_C9_SPACE_PARAMS, baseAddress, length, CAN_C9_ERROR_INVALID_STATE);
        }    
    }
    else if( baseAddress + length <= ATE_MAX_ADDR )
    {    
        uint16_t ate_status[ ATE_MAX_LENGTH ] = {0};
        
        for( uint16_t cnt = 0; cnt < length; cnt++ )
        {
        	ate_status[ cnt ] = ATE_Status[ baseAddress - ATE_PARAMS_START_ADDR + cnt ];
        }
        
        for (uint16_t word = 0; word < length; word += 4)
        {
            uint16_t words = (length - word >= 4) ? 4 : (length - word);
            can.PacketTx(InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + word/4) | CAN_C9_SPACE_PARAMS, CAN_C9_PRIORITY)
                .subtype(can.UPMNumber() | (words << CAN_C9_TWRITE_DATA_COUNT_BITS))
                .data0(ate_status[word])
                .data1(ate_status[word+1])
                .data2(ate_status[word+2])
                .data3(ate_status[word+3]));
        }
    }
    else
    {
        readError(CAN_C9_SPACE_PARAMS, baseAddress, length, CAN_C9_ERROR_INVALID_ADDRESS);
    }
}

void C9StateMachine::writeVariables()
{
    uint16_t errorCode;
    if ((requestBaseAddress + requestLength) <= BLOCKS_4_END)
    {
        for (uint16_t i = 0; i < requestLength; ++i)
        {
            const stBlock* block = &BlockArray[requestBaseAddress + i];
            uint16_t value = buffer.requestData[i];
            switch (block->memtype) {
            case MemTypeINT:
                *(int16_t*)block->data = (int16_t)value;
                break;
                
            case MemTypeUINT:
                *(uint16_t*)block->data = value;
                break;
                
            case MemTypeLONG:
                *(int32_t*)block->data = (int32_t)value;
                break;
                
            case MemTypeULONG:
                *(uint32_t*)block->data = (uint32_t)value;
                break;
                
            case MemTypeFLOAT: {
                float meter = (float)(int16_t)value;
                float scale = 1.0;
                switch (block->type) {
                case DEC1:
                    scale = 10;
                    break;
                    
                case DEC2:
                    scale = 100;
                    break;
                    
                case DEC3:
                    scale = 1000;
                    break;
                    
                case DEC4:
                    scale = 10000;
                    break;
                
                case NONE:
                    // Unallocated block.
                    continue;
                    
                case DEC0:
                    // FALLTHROUGH
                case HEX:
                    // FALLTHROUGH
                default:
                    scale = 1.0;
                }
                *(float*)block->data = meter / scale;
                break;
            }
                
            default:
                break;    
            }
        }
        errorCode = CAN_C9_ERROR_SUCCESS;
    }
    else
    {
        errorCode = CAN_C9_ERROR_INVALID_ADDRESS;
    }
    writeResponse(CAN_C9_SPACE_VARS, requestBaseAddress, requestLength, errorCode);
}

void C9StateMachine::readVariables(uint32_t baseAddress, uint16_t length)
{
    memset(buffer.readData, 0, sizeof(buffer.readData));

    if ((baseAddress + length) >= BLOCKS_4_END)
    {
        readError(CAN_C9_SPACE_PARAMS, baseAddress, length, CAN_C9_ERROR_INVALID_ADDRESS);
    }
    else
    {
        for (size_t i = 0; i < length; ++i)
        {
            const stBlock* block = &BlockArray[baseAddress+i];
            switch (block->memtype) {
            case MemTypeINT:
                buffer.readData[i] = (uint16_t)*(int16_t*)block->data;
                break;
                
            case MemTypeUINT:
                buffer.readData[i] = *(uint16_t*)block->data;
                break;
                
            case MemTypeLONG:
                buffer.readData[i] = *(int32_t*)block->data;
                break;
                
            case MemTypeULONG:
                buffer.readData[i] = *(uint32_t*)block->data;
                break;
                
            case MemTypeFLOAT: {
                float meter = *(float*)block->data;
                float scale = 1.0;
                switch (block->type) {
                case DEC1:
                    scale = 10;
                    break;
                    
                case DEC2:
                    scale = 100;
                    break;
                    
                case DEC3:
                    scale = 1000;
                    break;
                    
                case DEC4:
                    scale = 10000;
                    break;
                    
                case NONE:
                    // Unallocated debugger block. Skip value and continue with
                    // the next one.
                    buffer.readData[i] = 0;
                    continue;
                    
                case DEC0:
                    // FALLTHROUGH
                case HEX:
                    // FALLTHROUGH
                default:
                    scale = 1.0;
                }
                buffer.readData[i] = int16_t(meter * scale);
                break;
            }

            case FuncTypeINT:
                if ( ((uint32_t)(block->data) >= (uint32_t)MAINPROG_ENTRY) &&
                     ((uint32_t)(block->data) < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
                {
                    buffer.readData[i] = (uint16_t)((FUNC_TYPE_INT_PTR)((uint32_t)(block->data)))();
                }
                break;

            case FuncTypeUINT:
                if ( ((uint32_t)(block->data) >= (uint32_t)MAINPROG_ENTRY) &&
                     ((uint32_t)(block->data) < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
                {
                    buffer.readData[i] = ((FUNC_TYPE_UINT_PTR)((uint32_t)(block->data)))();
                }
                break;

            case FuncTypeLONG: // MJD: not sure this works, readData[i] is only 16 bits
                if ( ((uint32_t)(block->data) >= (uint32_t)MAINPROG_ENTRY) &&
                     ((uint32_t)(block->data) < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
                {
                    buffer.readData[i] = ((FUNC_TYPE_LONG_PTR)((uint32_t)(block->data)))();
                }
                break;

            case FuncTypeULONG: // MJD: not sure this works, readData[i] is only 16 bits
                if ( ((uint32_t)(block->data) >= (uint32_t)MAINPROG_ENTRY) &&
                     ((uint32_t)(block->data) < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
                {
                    buffer.readData[i] = ((FUNC_TYPE_ULONG_PTR)((uint32_t)(block->data)))();
                }
                break;

            case FuncTypeFLOAT:
                if ( ((uint32_t)(block->data) >= (uint32_t)MAINPROG_ENTRY) &&
                     ((uint32_t)(block->data) < ((uint32_t)MAINPROG_ENTRY+MAINPROG_SIZE)) )
                {
                    float meter = ((FUNC_TYPE_FLOAT_PTR)((uint32_t)(block->data)))();
                    float scale = 1.0f;
                    switch (block->type) {
                    case DEC1:
                        scale = 10.0f;
                        break;

                    case DEC2:
                        scale = 100.0f;
                        break;

                    case DEC3:
                        scale = 1000.0f;
                        break;

                    case DEC4:
                        scale = 10000.0f;
                        break;

                    case NONE:
                        // Unallocated debugger block. Skip value and continue with
                        // the next one.
                        buffer.readData[i] = 0;
                        continue;

                    case DEC0:
                        // FALLTHROUGH
                    case HEX:
                        // FALLTHROUGH
                    default:
                        scale = 1.0f;
                    }
                    buffer.readData[i] = uint16_t(meter * scale);
                    break;
                }
                break;

            default:
                break;    
            }
            
        }
        
        for (int word = 0; word < length; word += 4)
        {
            int words = (length - word >= 4) ? 4 : (length - word);
            can.PacketTx(InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + word/4) | CAN_C9_SPACE_PARAMS, 
                    CAN_C9_PRIORITY)
                .subtype(can.UPMNumber() | (words << CAN_C9_TWRITE_DATA_COUNT_BITS))
                .data0(buffer.readData[word])
                .data1(buffer.readData[word+1])
                .data2(buffer.readData[word+2])
                .data3(buffer.readData[word+3]));
        }
    }
}

void C9StateMachine::writeProgramFlash()
{
    writeResponse(CAN_C9_SPACE_BOOT_FLASH, 0xffffffff, 0xffff, CAN_C9_ERROR_TIMEOUT);
}

void C9StateMachine::readProgramFlash(uint32_t baseAddress, uint16_t length)
{
    uint16_t index = 0;
    uint32_t flash_address = baseAddress;
        
    if((baseAddress < 0x300000) || (baseAddress + length > 0x338000) || (length > 64))
    {
        readError(CAN_C9_SPACE_PROG_FLASH, baseAddress, length, CAN_C9_ERROR_INVALID_ADDRESS);
    }
    else
    {
        for( index = 0; index < length / 4; index++ )
        {
            can.PacketTx(InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_PROG_FLASH, CAN_C9_PRIORITY)
                .subtype(0x40 | can.UPMNumber()) // 4 words in this packet
                .data0(swap((*(uint16_t *)(flash_address + 0))))
                .data1(swap((*(uint16_t *)(flash_address + 1))))
                .data2(swap((*(uint16_t *)(flash_address + 2))))
                .data3(swap((*(uint16_t *)(flash_address + 3)))));
            
            flash_address += 4;
        }
        
        // this is to process the last frame with less than 4 words,invalid data field will be filled 0xFFFF
        switch( length & 0x03 )
        {
            case 1 :
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_PROG_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x10 | can.UPMNumber())      // 1 valid word in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(0xFFFF)
                    .data2(0xFFFF)
                    .data3(0xFFFF));
                flash_address += 1;
                break;
                
            case 2 :
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_PROG_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x20 | can.UPMNumber())      // 2 valid words in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(swap((*(uint16_t *)(flash_address + 1))))
                    .data2(0xFFFF)
                    .data3(0xFFFF));
                flash_address += 2;
                break;
                
            case 3 :    
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_PROG_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x30 | can.UPMNumber())      // 3 valid words in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(swap((*(uint16_t *)(flash_address + 1))))
                    .data2(swap((*(uint16_t *)(flash_address + 2))))
                    .data3(0xFFFF));
                flash_address += 3;
                break;
            default : 
                break;   
        }     
    }          
}

void C9StateMachine::writeBootloaderFlash()
{
    writeResponse(CAN_C9_SPACE_BOOT_FLASH, 0xffffffff, 0xffff, CAN_C9_ERROR_TIMEOUT);
}

void C9StateMachine::readBootloaderFlash(uint32_t baseAddress, uint16_t length)
{
    uint16_t index = 0;
    uint32_t flash_address = baseAddress;
    
    if((baseAddress < 0x338000) || (baseAddress + length > 0x340000) || (length > 64))
    {
        readError(CAN_C9_SPACE_BOOT_FLASH, baseAddress, length, CAN_C9_ERROR_INVALID_ADDRESS);  
    }
    else
    {
         for( index = 0; index < length / 4; index++ )
         {
            can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_BOOT_FLASH, CAN_C9_PRIORITY)
                .subtype(0x40 | can.UPMNumber()) // 4 words in this packet
                .data0(swap((*(uint16_t *)(flash_address + 0))))
                .data1(swap((*(uint16_t *)(flash_address + 1))))
                .data2(swap((*(uint16_t *)(flash_address + 2))))
                .data3(swap((*(uint16_t *)(flash_address + 3)))));
            
            flash_address += 4;
         }
        
        // this is to process the last frame with less than 4 words,invalid data field will be filled 0xFFFF
        switch( length & 0x03 )
        {
            case 1 :
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_BOOT_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x10 | can.UPMNumber())      // 1 valid word in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(0xFFFF)
                    .data2(0xFFFF)
                    .data3(0xFFFF));
                flash_address += 1;
                break;
                
            case 2 :
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_BOOT_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x20 | can.UPMNumber())      // 2 valid words in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(swap((*(uint16_t *)(flash_address + 1))))
                    .data2(0xFFFF)
                    .data3(0xFFFF));
                flash_address += 2;
                break;
                
            case 3 :    
                can.PacketTx( InternalCanPacket((CAN_C9_READ_RESPONSE_BASE + index) | CAN_C9_SPACE_BOOT_FLASH, CAN_C9_PRIORITY)
                    .subtype(0x30 | can.UPMNumber())      // 3 valid words in this packet
                    .data0(swap((*(uint16_t *)(flash_address + 0))))
                    .data1(swap((*(uint16_t *)(flash_address + 1))))
                    .data2(swap((*(uint16_t *)(flash_address + 2))))
                    .data3(0xFFFF));
                flash_address += 3;
                break;
            default : 
                break;   
        }      
    }    
}


void C9StateMachine::beginProgramFlash()
{
    // TODO: Drop into bootloader and direct it to erase program flash
}

void C9StateMachine::beginBootloaderFlash()
{
    // TODO: Drop into bootloader and direct it to erase the boot block flash
}

void C9StateMachine::writeResponse(uint16_t space, uint32_t baseAddress, uint16_t length, uint16_t reason)
{
    // Format and send a write error packet.
    can.PacketTx(InternalCanPacket(CAN_C9_TWRITE_RESPONSE | space, CAN_C9_PRIORITY)
        .subtype(can.UPMNumber())
        .data0(baseAddress >> 16)
        .data1(baseAddress & 0xffff)
        .data2(length)
        .data3(reason));
}

void C9StateMachine::readError(uint16_t space, uint32_t baseAddress, uint16_t length, uint16_t reason)
{
    // Format and send a read error packet
    can.PacketTx(InternalCanPacket(CAN_C9_READ_ERROR | space, CAN_C9_PRIORITY)
        .subtype(can.UPMNumber())
        .data0(baseAddress >> 16)
        .data1(baseAddress & 0xffff)
        .data2(length)
        .data3(reason));
}

void C9StateMachine::completeFlashError(uint16_t space, uint16_t reason)
{
    // TODO: Format and queue an error packet
}

void C9StateMachine::beginFlashError(uint16_t space, uint16_t reason)
{
    // Format and queue an error packet
    can.PacketTx(InternalCanPacket(CAN_C9_BEGIN_FLASH_RESPONSE | space, CAN_C9_PRIORITY)
        .subtype(can.UPMNumber())
        .data0(reason)
        .data1(0)
        .data2(0)
        .data3(0));
}

void C9StateMachine::checkWritePacketTimeout(void)
{
    if (initialWritePacketTime == 0)
    {
        // First packet. Save the time that we observed it.
        initialWritePacketTime = CLK_getltime();
    }
    else
    {
        // Read the current time. If it is greater than the initial time + the
        // timeout, then reset the write packet reception states and send a
        // timeout error packet.
        uint32_t now = CLK_getltime();
        uint32_t elapsed = now - initialWritePacketTime;
        if (elapsed * CLK_getprd() / CLK_countspms() > CAN_C9_TWRITE_TIMEOUT)
        {
            resetWritePacketMachine();
            initialWritePacketTime = now;
        }
    }
}
void C9StateMachine::resetWritePacketMachine(void)
{
    writeRequestMap = 0;
    memset(buffer.requestData, 0, sizeof(buffer));
    requestLength = 0;
    requestBaseAddress = 0;
    initialWritePacketTime = 0;
}

void C9StateMachine::checkWritePacket(uint16_t addressSpace)
{   
    if (requestLength != 0)
    {
        // A bitmask of all of the request data packets that must arrive before
        // we can process the packet.
        const uint16_t nPackets = requestLength / 4 
            + ((requestLength % 4) ? 1 : 0);
        const uint16_t reqPacketMask = (1L << nPackets) - 1;
        if (reqPacketMask == writeRequestMap)
        {
            if (requestLength > CAN_C9_TWRITE_MAX)
            {
                writeResponse(addressSpace, requestBaseAddress, requestLength, 
                    CAN_C9_ERROR_INVALID_ADDRESS);
            }
            else
            {
                switch (addressSpace) {
                case CAN_C9_SPACE_PROG_FLASH:
                    writeProgramFlash();
                    break;
                    
                case CAN_C9_SPACE_BOOT_FLASH:
                    writeBootloaderFlash();
                    break;
                    
                case CAN_C9_SPACE_VARS:
                    writeVariables();
                    break;
                    
                case CAN_C9_SPACE_PARAMS:
                    writeParameters();
                    break;
                    
                default:
                    writeResponse(addressSpace, 
                        requestBaseAddress, requestLength, 
                        CAN_C9_ERROR_INVALID_PACKET);
                    break;
                }
            }
            resetWritePacketMachine();
        }
    }
}

void C9StateMachine::prepareForFlash( uint16_t space )
{
    if ( MCUStateMachine.InvSupportingLoad() )
    {
    	beginFlashError(space, CAN_C9_ERROR_INVALID_STATE);
    }
//  else if ( MCUStateMachine.InStandby() )
//  Here in case the unit is in the process of startup, we should unconditionally shutdown it for safety
    else
    {
    	// Shut down the rectifier and charger and yield for some time
    	MCUStateMachine.ClearAllCommands();
    	MCUStateMachine.StandbyChargerOffCommand();
    	
    	// Send out an in progress packet to prevent errors
    	beginFlashError(space, CAN_C9_ERROR_INPROGRESS);
    	
    	TSK_sleep( TSK_200_ms );
    }
}

uint16_t C9StateMachine::ATE_process( void )
{   
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
    
    ATE_SetParameters();
    
    switch( requestBaseAddress )
    {
        case ATE_CLEAR_HISTORY_ADDR:         // 4100
             errorCode = ATE_ClearHistory();
             break;
        case ATE_RESET_EEPROM_ADDR:          // 4101
             errorCode = ATE_ResetEeprom();
             break;
        case ATE_SET_TIME_ADDR:              // 4102 No ATE set time function 
             break;
        case ATE_SEND_COMMAND_ADDR:          // 4103
             errorCode = ATE_SendCommand();
             break;
        case ATE_SET_ECT_PARAMS_ADDR:		 //4104
             errorCode = ATE_SetECTParams();
             break;
        case ATE_FCT_SET_ADC_ADDR:           // 4105
             errorCode = ATE_FCT_SetAdcAddr();
             break;
        default:                             // 4096 - 4099
             errorCode = CAN_C9_ERROR_SUCCESS;
             break;
    }
    
    return(errorCode);
}

void C9StateMachine::ATE_SetParameters(void)
{
    uint16_t cnt = 0;
    uint16_t address = requestBaseAddress - ATE_PARAMS_START_ADDR;
    
    for( cnt = 0; cnt < requestLength ; cnt++ )
    {
        ATE_parameter[ address + cnt ] = buffer.requestData[ cnt ];
        ATE_Status[ address + cnt ] = buffer.requestData[ cnt ];
    }
}

uint16_t C9StateMachine::ATE_ClearHistory(void)
{
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
    
    if( ATE_parameter[ ATE_CLEAR_HISTORY_ADDR - ATE_PARAMS_START_ADDR ] == ATE_AUTH_ID )
    {
        if( NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT ) )
        {
            errorCode = CAN_C9_ERROR_INVALID_STATE;
        }
        else
        {
            HistoryQueue.clearEEPROMEvents();
        }
    }
    else
    {
        errorCode = CAN_C9_ERROR_INVALID_VALUE;
    }    
    
    ATE_Status[ ATE_CLEAR_HISTORY_ADDR - ATE_PARAMS_START_ADDR ] = errorCode;
    
    return(errorCode);
}
    
uint16_t C9StateMachine::ATE_ResetEeprom(void)
{  
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
    uint16_t thisSection = EE_System_ID;
    
    if( ATE_parameter[ ATE_RESET_EEPROM_ADDR - ATE_PARAMS_START_ADDR ] == ATE_AUTH_ID )    
    {
        if( NB_GetNodebit( UPM_NB_POWER_SUPPLY_5_VOLT_FAULT ) )
        {
            errorCode = CAN_C9_ERROR_INVALID_STATE;
        }
        else
        {
            if( ATE_parameter[0] == 255 )
            {
                while ( thisSection < EE_Last_Section )
                {   
                    ReBootEepromSection( thisSection );
                    writeResponse(CAN_C9_SPACE_PARAMS, requestBaseAddress, requestLength, CAN_C9_ERROR_INPROGRESS);  
                    ++thisSection;
                    TSK_sleep(TSK_5_ms);
                }
            }
            else if( ATE_parameter[0] < EE_Last_Section )
            {
                ReBootEepromSection( ATE_parameter[0] );
            }
            else
            {
                errorCode = CAN_C9_ERROR_INVALID_VALUE;
            }
        }
    }
    else
    {
        errorCode = CAN_C9_ERROR_INVALID_VALUE;
    }
        
    ATE_Status[ ATE_RESET_EEPROM_ADDR - ATE_PARAMS_START_ADDR ] = errorCode;
    
    return(errorCode);
}

uint16_t C9StateMachine::ATE_SetECTParams(void)
{
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
	if( !MCUStateMachine.GetStatus().bit.ToECTCmd  &&
		(ATE_parameter[ ATE_SET_ECT_PARAMS_ADDR - ATE_PARAMS_START_ADDR ] == ATE_AUTH_ID))
	{
	    //Set percent of nominal power 2000= 20%, 10000 = 100%
        //Set power factor of output, 70 = 0.7, 140 = -0.6 
		if( (ATE_parameter[0] >= 2000) && (ATE_parameter[0] <= 10000) &&  //4096
			(ATE_parameter[1] >= 70) && (ATE_parameter[1] <= 140))        //4097
		{
			Inverter.SetECTPowerAndPf( ATE_parameter[0], ATE_parameter[1] );
		}
		else
        {
            errorCode = CAN_C9_ERROR_INVALID_VALUE;
        }

        //Set duration time
        //ATE_parameter[2]: bat  ECT time: minute
        //ATE_parameter[3]: line ECT time: minute
		MCUStateMachine.SetBatECTDuration ( (uint32_t)ATE_parameter[2] * 60L );	//4098
		MCUStateMachine.SetLineECTDuration( (uint32_t)ATE_parameter[3] * 60L );	//4099	
	}
	else
    {
        errorCode = CAN_C9_ERROR_INVALID_VALUE;
    }

    ATE_Status[ ATE_SET_TIME_ADDR - ATE_PARAMS_START_ADDR ] = errorCode;

    return(errorCode);   
}

uint16_t C9StateMachine::ATE_FCT_SetAdcAddr(void)
{
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
    uint16_t adc_result = Get_ADC_Data( ATE_parameter[0] );
    
    ATE_Status[ ATE_FCT_SET_ADC_ADDR - ATE_PARAMS_START_ADDR ] = adc_result;
    
    return errorCode;
}

uint16_t C9StateMachine::ATE_SendCommand(void)
{
    uint16_t errorCode = CAN_C9_ERROR_SUCCESS;
    
    if( ATE_parameter[ ATE_SEND_COMMAND_ADDR - ATE_PARAMS_START_ADDR ] == ATE_AUTH_ID )
    {
        switch( ATE_parameter[0] )
        {
            case ATE_CMD_ENTER_TEST_MODE:
                 ATE_CMD_ENTER_TEST_MODE_func();
                 break;
                 
            case ATE_CMD_EXIT_TEST_MODE:
                 ATE_CMD_EXIT_TEST_MODE_func();
                 break;
                     
            case ATE_CMD_TURN_ON_PRECHARGER:
                 ATE_CMD_TURN_ON_PRECHARGER_func();
                 break;
                     
            case ATE_CMD_TURN_OFF_PRECHARGER:
                 ATE_CMD_TURN_OFF_PRECHARGER_func();
                 break;    
            
            case ATE_CMD_TURN_ON_INVERTER:
                 ATE_CMD_TURN_ON_INVERTER_func();
                 break;
                     
            case ATE_CMD_TURN_OFF_INVERTER:
                 ATE_CMD_TURN_OFF_INVERTER_func();
                 break;
                     
            case ATE_CMD_UPS_NORMAL:
                 ATE_CMD_UPS_NORMAL_func();
                 break;
                 
            case ATE_CMD_UPS_LOAD_OFF:
                 ATE_CMD_UPS_LOAD_OFF_func();
                 break;
                 
            case ATE_CMD_UPS_BYPASS_ON:
                 ATE_CMD_UPS_BYPASS_ON_func();
                 break;
                          
/*            case ATE_CMD_ENTER_LINE_ECT_MODE:
                 ATE_CMD_ENTER_LINE_ECT_MODE_func();
                 break;
                 
            case ATE_CMD_ENTER_BAT_ECT_MODE:
                 ATE_CMD_ENTER_BAT_ECT_MODE_func();
                 break;*/
                      
            case ATE_CMD_EXIT_ECT_MODE:
                 ATE_CMD_EXIT_ECT_MODE_func();
                 break;
                 
            /*case ATE_CMD_ENABLE_ECT_MODE:
                 ATE_CMD_ENABLE_ECT_MODE_func();
                 break;*/
				 
			case ATE_CMD_ENTER_ECT_MODE:
                 ATE_CMD_ENTER_ECT_MODE_func();
                 break;    
				 
			case ATE_CMD_START_PARA_AUTOCAL:
                 ATE_CMD_START_PARA_AUTOCAL_func();
                 break;
            
            case ATE_CMD_UPM_NORMAL:
                 ATE_CMD_UPM_NORMAL_func();
                 break;
                 
            case ATE_CMD_UPM_LOAD_OFF:
                 ATE_CMD_UPM_LOAD_OFF_func();
                 break;

            case ATE_CMD_UPS_CHARGE_ON:
                 NB_DebounceAndQue( UPM_NB_CHARGER_ON_COMMAND, true );
                 Abm().ChargerCmdOn();
                 if( (MCUStateMachine.GetState() == SHUTDOWN_STATE) || (MCUStateMachine.GetState() == BYPASS_STATE) )
                 {
                     MCUStateMachine.AutoStandbyCommand();
                 }
                 BatteryConverter.CancelBatteryTest();
                 break;

            case ATE_CMD_UPS_CHARGE_OFF:
                NB_DebounceAndQue( UPM_NB_CHARGER_OFF_COMMAND, true );
                if( MCUStateMachine.InStandby() )
                {
                    MCUStateMachine.StandbyChargerOffCommand();
                }
                Abm().ChargerCmdOff();
                BatteryConverter.CancelBatteryTest();
                 break;

            case ATE_CMD_UPS_ESS_TIME_TEST:
            	EssTimeTest = true;
                 break;

            case ATE_CMD_UPS_ESS_TEST_EXIT:
            	EssTimeTest = false;
                 break;

            case ATE_CMD_CAL_BATTERY_CURRENT_LEG_A:
                 ATE_CMD_CAL_BATTERY_CURRENT_func(BATTERY_LEG_A);
                 break;

            case ATE_CMD_CAL_BATTERY_CURRENT_LEG_B:
                 ATE_CMD_CAL_BATTERY_CURRENT_func(BATTERY_LEG_B);
                 break;

            case ATE_CMD_CAL_BATTERY_CURRENT_EXIT:
                 ATE_CMD_CAL_BATTERY_CURRENT_EXIT_func();
                 break;

            case ATE_CMD_ECT_RESTART_TIMES:
				 ATE_CMD_ECT_RESTART_TIMES_func();
				 break;
				 
			case ATE_CMD_BATTERY_TEST:
				 ATE_CMD_BATTERY_TEST_func();
				 break;
                 
            case ATE_CMD_UPM_BYPASS_ON:
                 ATE_CMD_UPM_BYPASS_ON_func();
                 break;

            case ATE_CMD_UPS_5V_ALARM_MASK:
                 AteMaskAlarm5VforRsEep = true;
            break;

            case ATE_CMD_UPS_5V_ALARM_BACK:
                 AteMaskAlarm5VforRsEep = false;
            break;

            case FCT_CMD_Enter_FCTMode:
                 FCT_CMD_Enter_FCTMode_func();
                 break;
                 
            case FCT_CMD_CHARGE_TEST:
                 FCT_CMD_CHARGE_TEST_func();
                 break;
                 
            case FCT_CMD_INV_TEST:
                 FCT_CMD_INV_TEST_func();
                 break;
                 
            case FCT_CMD_INV_SHORT_TEST:
                 FCT_CMD_INV_SHORT_TEST_func();
                 break;
                 
            case FCT_CMD_BOOST_TEST:
                 FCT_CMD_BOOST_TEST_func();
                 break;
                 
            case FCT_CMD_REC_TEST:
                 FCT_CMD_REC_TEST_func();
                 break;
                 
            case FCT_CMD_REC_SHORT_TEST:
                 FCT_CMD_REC_SHORT_TEST_func();
                 break;
                 
            case FCT_CMD_LINEPMDISCHARGE:
                 FCT_CMD_LINEPMDISCHARGE_func();
                 break;
                 
            case FCT_CMD_CLEAR_RESULT:
                 FCT_CMD_CLEAR_RESULT_func();
                 break;
                 
            case FCT_QUIT_CURRENTFCT_TEST:
                 FCT_QUIT_CURRENTFCT_TEST_func();
                 break;
                 
            case FCT_QUIT_FCTMODE:
                 FCT_QUIT_FCTMODE_func();
                 break;

            case FCT_CMD_ENABLE_BOARDID:
            	 FCT_CMD_ENABLE_BOARDID_func();

                                
            default:
                 break;
        }
    }
    else
    {
        errorCode = CAN_C9_ERROR_INVALID_VALUE;
    }    
    
    ATE_Status[ ATE_SEND_COMMAND_ADDR - ATE_PARAMS_START_ADDR ] = errorCode;
    
    return(errorCode);    
}

void C9StateMachine::ATE_CMD_ENTER_TEST_MODE_func(void)
{
    MCUStateMachine.Turn_UPS_Off();
    MCUStateMachine.EnableTestMode();
}
    
void C9StateMachine::ATE_CMD_EXIT_TEST_MODE_func(void)
{
    MCUStateMachine.DisableTestMode();
}

void C9StateMachine::ATE_CMD_TURN_ON_PRECHARGER_func(void)
{
    if( MCUStateMachine.UPMTestMode )
    {
        PreChargeOn();
    }
}

void C9StateMachine::ATE_CMD_TURN_OFF_PRECHARGER_func(void)
{
    PreChargeOff();
}

void C9StateMachine::ATE_CMD_TURN_ON_INVERTER_func(void)
{
    if( MCUStateMachine.UPMTestMode )
    {
        Inverter.OnOpenLoop();
    }
}

void C9StateMachine::ATE_CMD_TURN_OFF_INVERTER_func(void)
{
    Inverter.Off();
}

void C9StateMachine::ATE_CMD_UPS_NORMAL_func(void)
{
//  MCUStateMachine.DisableTestMode();
//  MCUStateMachine.NormalCommand();

    ResetAlarms();
    
    if( MyUPMNumber == 0 )
	{
		 ParallelCan.InternalCommand.bit.csb_on_normal_command = 1;
	}
}

void C9StateMachine::ATE_CMD_UPS_LOAD_OFF_func(void)
{
//  MCUStateMachine.DisableTestMode();
//  MCUStateMachine.Turn_UPS_Off();

    ResetAlarms();

    if( MyUPMNumber == 0 )
    {
        ParallelCan.InternalCommand.bit.csb_load_off_command = 1;
    }
}

void C9StateMachine::ATE_CMD_UPS_BYPASS_ON_func(void)
{
//  MCUStateMachine.BypassCommand();

    ResetAlarms();

    if( MyUPMNumber == 0 )
    {
        ParallelCan.InternalCommand.bit.csb_bypass_on_command = 1;
    } 
}

void C9StateMachine::ATE_CMD_UPM_BYPASS_ON_func(void)
{
	MCUStateMachine.BypassCommand();

    ResetAlarms(); 
}

void C9StateMachine::ATE_CMD_START_PARA_AUTOCAL_func(void)
{
    MCUStateMachine.DisableTestMode();
    AutoCal.StartAutoCal();
}

void C9StateMachine::ATE_CMD_UPM_NORMAL_func(void)
{

    ResetAlarms();
    MCUStateMachine.DisableTestMode();
    MCUStateMachine.NormalCommand();
}

void C9StateMachine::ATE_CMD_UPM_LOAD_OFF_func(void)
{
    ResetAlarms();
    MCUStateMachine.DisableTestMode();
    MCUStateMachine.Turn_UPS_Off();
}

/*void C9StateMachine::ATE_CMD_ENTER_LINE_ECT_MODE_func(void)
{
	//MCUStateMachine.ECTLineCommand();
}

void C9StateMachine::ATE_CMD_ENTER_BAT_ECT_MODE_func(void)
{
	//MCUStateMachine.ECTBatteryCommand();	
}*/

void C9StateMachine::ATE_CMD_EXIT_ECT_MODE_func(void)
{
	//MCUStateMachine.ECTPhase = 10;
	
    if( MyUPMNumber == 0 )
    {
        ParallelCan.C9Command.bit.c9_ect_off_command = 1;
    }        
}

/*void C9StateMachine::ATE_CMD_ENABLE_ECT_MODE_func(void)
{
    MCUStateMachine.EnableECT();
}*/

void C9StateMachine::ATE_CMD_ENTER_ECT_MODE_func(void)
{
    if( MyUPMNumber == 0 )
    {
        ParallelCan.C9Command.bit.c9_ect_on_command = 1;
    }
}

void C9StateMachine::ATE_CMD_CAL_BATTERY_CURRENT_func(uint16_t leg)
{
    ResetAlarms();
    BatteryConverter.BatteryCurrentCalEnable = true;
    BatteryConverter.BatteryCurrentPhase = leg;
}

void C9StateMachine::ATE_CMD_CAL_BATTERY_CURRENT_EXIT_func(void)
{
    ResetAlarms();
    BatteryConverter.BatteryCurrentCalEnable = false;
}

void C9StateMachine::ATE_CMD_ECT_RESTART_TIMES_func(void)
{
	MCUStateMachine.ECTRestartNumber = ATE_parameter[1];
}

void C9StateMachine::ATE_CMD_BATTERY_TEST_func(void)
{
	BatteryConverter.BatteryTestCmdOn();
}

void C9StateMachine::FCT_CMD_Enter_FCTMode_func(void)
{
    if( MCUStateMachine.GetState() == SHUTDOWN_STATE &&   // UPM in shutdown and not in test mode
        !MCUStateMachine.UPMTestMode )
    {
        FCTStateMachine.EnableFCT();
    }
}

bool C9StateMachine::FCT_CMD_Receive_Condition ( void )
{
   return (  FCTStateMachine.GetFCTMode()                      && //FCT mode on
             FCTStateMachine.GetFCTState() == FCT_IDLE_STATE   && //FCT in IDLE state && (test not start or pass)
             FCTStateMachine.FCT_Result < FCT_IN_PROGRESS );
}

void C9StateMachine::FCT_CMD_CHARGE_TEST_func( void )
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.ChargerTestCmd = 1;
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}
void C9StateMachine::FCT_CMD_INV_TEST_func( void )
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.InverterTestCmd = 1;
        FCTStateMachine.FCT_Status.bit.LineShortTest = 0;    // Normal test
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_INV_SHORT_TEST_func( void )
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.InverterTestCmd = 1;
        FCTStateMachine.FCT_Status.bit.LineShortTest = 1;    // Short test
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_BOOST_TEST_func (void)
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.BoostTestCmd = 1;
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_REC_TEST_func( void )
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.RectifierTestCmd = 1;
        FCTStateMachine.FCT_Status.bit.LineShortTest = 0;    // Normal test
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_REC_SHORT_TEST_func(void)
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.RectifierTestCmd = 1;
        FCTStateMachine.FCT_Status.bit.LineShortTest = 1;    // Short test
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_LINEPMDISCHARGE_func(void)
{
    if( FCT_CMD_Receive_Condition() )
    {
        FCTStateMachine.FCT_Status.bit.LineDischargeCmd = 1;
        FCTStateMachine.FCT_Time = ATE_parameter[1] <= 30? ATE_parameter[1]:30;
    }
}

void C9StateMachine::FCT_CMD_CLEAR_RESULT_func( void )
{
    if( FCTStateMachine.GetFCTState() == FCT_IDLE_STATE )
    {
        FCTStateMachine.ClearResult();
    }
}

void C9StateMachine::FCT_QUIT_CURRENTFCT_TEST_func(void)
{
    if( FCTStateMachine.GetFCTState() != FCT_IDLE_STATE )
    {
        FCTStateMachine.FCT_Status.bit.CancelTestCmd = 1;
    }
}

void C9StateMachine::FCT_QUIT_FCTMODE_func(void)
{
    FCTStateMachine.DisableFCT();
}

void C9StateMachine::FCT_CMD_ENABLE_BOARDID_func(void)
{
	FCTStateMachine.BoardID_Warning = ATE_parameter[1] == 0? ATE_parameter[1]:1;
}
