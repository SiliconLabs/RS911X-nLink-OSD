/**
 *   @file     rsi_zigb_callbacks.h
 *   @version  1.0
 *   @date     2014-Aug-23
 *
 *   Copyright(C) Redpine Signals 2014
 *   All rights reserved by Redpine Signals.
 *
 *   @section License
 *   This program should be used on your own responsibility.
 *   Redpine Signals assumes no responsibility for any losses
 *   incurred by customers or third parties arising from the use of this file.
 *
 *   @brief API: Definitions of various data structures and variables
 *
 *   @section Description
 *   This file contain definition of different mangament, control & data commands variables.
 *   These definition are used to construct frames. 
 *
 *   @section Improvements
 *   New command frames are added.
 *
 */


/**
 * Includes
 * */


#ifndef RSI_ZIGB_CALLBACKS_H
#define RSI_ZIGB_CALLBACKS_H

#include "rsi_zigb_types.h"
#include "rsi_zigb_interfaces.h"

void rsi_zigb_app_scan_complete_handler(uint32_t , uint8_t );
void rsi_zigb_app_energy_scan_result_handler(uint32_t ,uint8_t *);
void rsi_zigb_app_network_found_handler(ZigBeeNetworkDetails);
void rsi_zigb_app_stack_status_handler(ZigBeeNWKStatusInfo *);
void rsi_zigb_app_incoming_many_to_one_route_req_handler(uint16_t , uint8_t *,uint8_t );
void rsi_zigb_app_handle_data_confirmation (APSDE_Data_Confirmation_t *pDataConfirmation);
void rsi_zigb_api_test_data_indication_handler(APSDE_Data_Indication_t *pDataIndication);
void rsi_zigb_app_child_join_handler(uint16_t short_address, BOOL joining);

#endif
