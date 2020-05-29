/**
 *  @file     rsi_zigb_oper_mode.h
 *  @version  1.0
 *  @date     2014-Sep-25
 *
 *  Copyright(C) Redpine Signals 2014
 *  All rights reserved by Redpine Signals.
 *
 *  @section License
 *  This program should be used on your own responsibility.
 *  Redpine Signals assumes no responsibility for any losses
 *  incurred by customers or third parties arising from the use of this file.
 *
 *  @brief API: Definitions of various data structures and variables
 *
 *  @section Description
 *  This file contain definition of different mangament, control & data commands variables.
 *  These definition are used to construct frames. 
 *
 *  @section Improvements
 *  New command frames are added.
 *
 */


/**
 * Includes
 * */

#ifndef RSI_ZIGB_OPER_MODE_H
#define RSI_ZIGB_OPER_MODE_H

#ifdef ZB_ONLY
 
#define BIT(a) ((long int)1 << a)

#define RSI_ZIGB_RSP_CARD_READY                 0x89
#define RSI_ZIGB_RSP_OPERMODE                   0x10


#define RSI_ZIGB_WIFI_OPER_MODE           0                    
#define RSI_ZIGB_COEX_MODE                3                      
#define RSI_ZIGB_OPERMODE                 (RSI_ZIGB_WIFI_OPER_MODE | ( RSI_ZIGB_COEX_MODE << 16))            
#define RSI_ZIGB_FEATURE_BIT_MAP          BIT(0)          
#define RSI_ZIGB_TCP_IP_FEATURE_BIT_MAP   (BIT(2) | BIT(1))
#define RSI_ZIGB_CUSTOM_FEATURE_BIT_MAP   0                          
#define RSI_ZIGB_STATUS_OFFSET            12
#define RSI_ZIGB_RSP_TYPE_OFFSET          2 

/*===================================================*/
/**
 * Operational Mode 
 *
 */  
typedef union {
    struct {
       uint32_t    oper_mode;                       //@ operating mode 0-client, 1-p2p, 2-EAP, 6-AP, 8-PER
       uint32_t    feature_bit_map;                 //@ BIT(0)-Open mode security, BIT(1)-PSK security, BIT(2) JSON objects
       uint32_t    tcp_ip_feature_bit_map;          //@ BIT(0)-tcp/ip bypass, BIT(1)-HTTP server,BIT(2)-DHCPV4 client, 
                                                  //@ BIT(3)-DHCPV6 client, BIT(4)-DHCPV4 server, BIT(5)-DHCPV6 server
       uint32_t   custom_feature_bit_map; 
    } operModeFrameSnd;
    uint8_t    uOperModeBuf[16];                
} rsi_zigb_uOperMode;

/** End of the certificate loading structure **/

typedef struct {
	uint8_t                     		rspCode[2];
	uint8_t                                   status[2];                  
	//@ 0- For Success ,Non-Zero Value is the Error Code return
	union {
		//@ response payload	
		uint8_t				                 uCmdRspBuf[256];
	}uCmdRspPayLoad;
} rsi_init_uCmdRsp;

#endif
#endif
