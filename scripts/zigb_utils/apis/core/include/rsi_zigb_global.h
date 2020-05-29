/**
 *    @file     rsi_zigb_global.h
 *    @version  1.0
 *    @date     2014-Aug-23
 *
 *    Copyright(C) Redpine Signals 2014
 *    All rights reserved by Redpine Signals.
 *
 *    @section License
 *    This program should be used on your own responsibility.
 *    Redpine Signals assumes no responsibility for any losses
 *    incurred by customers or third parties arising from the use of this file.
 *
 *    @brief API: Definitions of various data structures and variables
 *
 *    @section Description
 *    This file contain different global structures which are used in application
 *
 *    @section Improvements
 *
 */


/**
 * Includes
 * */
 
#ifndef _RSI_ZIGB_GLOBAL_H
#define _RSI_ZIGB_GLOBAL_H

#include "rsi_zigb_types.h"

#define RSI_ZIGB_MAX_PAYLOAD_SIZE       256

#define RSI_ZIGB_PAYLOAD_LEN_MASK       0xFFF
#define RSI_ZIGB_QUEUE                  0x1
#define RSI_ZIGB_FRAME_DESC_LEN         16
#define RSI_ZIGB_BYTES_2                2

#define RSI_ZIGB_SEQ_NUM_OFFSET         12
#define RSI_ZIGB_DIR_OFFSET             13
#define RSI_ZIGB_INTF_ID_OFFSET         14
#define CARD_READY_IND                  0xff
#define ZB_PKT                          (RSI_ZIGB_QUEUE << 4)

#define RSI_TRUE            1    
#define RSI_FALSE           0
#ifndef NULL
#define NULL                0
#endif

#ifdef LINUX_PLATFORM
#define RSI_DPRINT(lvl, fmt, args...)              if (lvl & RSI_DEBUG_LVL) printf(fmt, ##args)
#else
#define RSI_DPRINT(lvl, fmt, args...)              1
#endif

#ifndef RSI_LITTLE_ENDIAN
#define RSI_LITTLE_ENDIAN       0
#endif

//! Debug Print Levels
#define RSI_DEBUG_LVL         0x00ff

//! These bit values may be ored to all different combinations of debug printing
#define RSI_PL0                0xffff
#define RSI_PL1                0x0001
#define RSI_PL2                0x0002
#define RSI_PL3                0x0004
#define RSI_PL4                0x0008
#define RSI_PL5                0x0010
#define RSI_PL6                0x0020
#define RSI_PL7                0x0040
#define RSI_PL8                0x0080
#define RSI_PL9                0x0100
#define RSI_PL10               0x0200
#define RSI_PL11               0x0400
#define RSI_PL12               0x0800
#define RSI_PL13               0x1000
#define RSI_PL14               0x2000
#define RSI_PL15               0x4000
#define RSI_PL16               0x8000

uint8_t proc_read_buf[RSI_ZIGB_MAX_PAYLOAD_SIZE];

typedef struct {
  uint8_t                 status;
  uint8_t                 type;
} rsi_DevTypeResp;


typedef struct {
  uint8_t                 channel;
} rsi_ChannelResp;

typedef struct {
  uint8_t                 status;
} rsi_StatusResp;

typedef struct {
  uint8_t                 status;
} rsi_SetSimpleDescResp;

typedef struct {
  uint32_t                channel;
  uint32_t                 status;
} rsi_ScanDoneResp;

typedef struct {
  uint8_t                Self_ieee[8] ;
} rsi_SelfIEEEAddrResp;

typedef struct {
  uint8_t                Addr[8] ;
} rsi_IEEEAddResp;

typedef struct {
  uint8_t                Ext_Pan_Id[8] ;
} rsi_ExtPanIdResp;

typedef struct {
  uint16_t                short_addr ;
} rsi_ShortAddrResp;

typedef struct {
  uint16_t                cluster ;
} rsi_ClusterResp;

typedef struct {
  uint16_t                pan_id ;
} rsi_PanIdResp;


typedef struct {
  uint8_t                 status;
  uint8_t                 Ieee_Add[8];
} rsi_IEEEResp;


typedef struct {
  uint8_t                 status;
  uint8_t                 Ieee_Addr[8];
  uint8_t                 device_type;
} rsi_ChildDetailsResp;
/*=========================================================================*
 * Frame Descriptor
 *=========================================================================*/

typedef struct {
    uint8_t   uFrmDscBuf[RSI_ZIGB_FRAME_DESC_LEN];       //@ byte format for spi interface, 16 bytes
} rsi_zigb_uFrameDsc;

/*=========================================================================*
 * Response Frame Payload
 *=========================================================================*/

typedef struct {
  // !seq_no and dir are for debugging purpose
  uint8_t                 seq_no;
  uint8_t                 dir;
  uint8_t                 intf_id;                  
  uint8_t                 cmd_id;
  //@ 0- For Success ,Non-Zero Value is the Error Code return
  union {
    //@ response payload	
    rsi_ChannelResp         DevChannel;
    rsi_StatusResp          statusResp;
    rsi_ScanDoneResp        scandoneResp;
    rsi_DevTypeResp         GetDevResp;
    rsi_SetSimpleDescResp   SetSimpleDescResp;
    rsi_IEEEAddResp         IEEEAddrResp;
    rsi_SelfIEEEAddrResp    DevIEEEAddr;
    rsi_ShortAddrResp       DevShortAddr;
    rsi_PanIdResp           DevPanId;
    rsi_ExtPanIdResp        Extnd_PanId;
    rsi_ClusterResp         EpClusterResp;
    rsi_IEEEResp            GetIEEEResp;
    rsi_ChildDetailsResp    GetChildDetails;
    uint8_t					        uRspData[RSI_ZIGB_MAX_PAYLOAD_SIZE - RSI_ZIGB_FRAME_DESC_LEN];
  }uCmdRspPayLoad;
} rsi_zigb_uCmdRsp;

void rsi_zigb_delay(uint32_t time);
void rsi_zigb_build_frame_descriptor(rsi_zigb_uFrameDsc *uFrameDscFrame, uint8_t *cmd, uint8_t );

#endif
