/**
 *   @file     rsi_zigb_interfaces.h
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
 *   This file contain various structures which are ZigBee specific 
 *   interface commands
 *
 *   @section Improvements
 *
 */


/**
 * Includes
 * */


#ifndef RSI_ZIGB_INTERFACES_H
#define RSI_ZIGB_INTERFACES_H

#include "rsi_zigb_types.h"

#define g_EXTENDED_ADDRESS_LENGTH_c                   0x08

enum DEVICE_TYPE {

  g_COORDINATOR_c,
  g_ROUTER_c,
  g_END_DEVICE_c

};

typedef enum MAC_Scan_Types_Tag {

  /*! -ED Scan */
  g_MAC_ED_SCAN_TYPE_c = (uint8_t) 0x00,
  /*! -Active Scan */
  g_MAC_ACTIVE_SCAN_TYPE_c,
  /*! -Passive Scan */
  g_MAC_PASSIVE_SCAN_TYPE_c,
  /*! -Orphan Scan */
  g_MAC_ORPHAN_SCAN_TYPE_c

} MAC_Scan_Types_t;

typedef enum {

  g_ZigBeeNotPartOfNWK_c,
  g_ZigBeeInTheProcessOfJoiningNWK_c,
  g_ZigBeeJoinedNWK_c,
  g_ZigBeeJoinedNWKNoParent_c,
  g_ZigBeePerformingLeaveFromNWK_c

} ZigBeeJoinStatus;

typedef struct{
  
  /* The 802.15.4 channel associated with the network.*/
  uint16_t          shortPanId; /* The network's PAN identifier.*/
  uint8_t           channel; 
  uint8_t           extendedPanId[8]; /*The network's extended PAN identifier. */
  uint8_t           stackProfile; /* The Stack Profile associated with the network. */
  uint8_t           nwkUpdateId; /* The instance of the Network.*/
  BOOL              allowingJoining; /* Whether the network is allowing MAC associations.*/

}ZigBeeNetworkDetails; 


typedef enum {

  /* ZigBeeNWKIsUp  indicates of the NWK is formed or joined successfully */
  ZigBeeNWKIsUp,

  /* ZigBeeNWKIsDown indicates of the NWK formation failed or the device 
   * left the network */
  ZigBeeNWKIsDown, 

  /* ZigBeeJoinFailed indicates if the network join failed */
  ZigBeeJoinFailed, 

  /* ZigBeeCannotJoinAsRouter indicates if the network was unable
   *  to start as Router */
  ZigBeeCannotJoinAsRouter, 
  
  /* ZigBeeChangedNodeID indicates if the NodeID is changed after 
   * resolving address conflict */
  ZigBeeChangedNodeID, 
                      
  /* ZigBeeChangedPANId indicates if the PANID is changed after resolving
   * PAN ID conflict */
  ZigBeeChangedPANId, 
                      
  /* ZigBeeChangedChannel indicates if the the channel is changed due to
   * frequency agility mechanism */
  ZigBeeChangedChannel, 
  
  /* ZigBeeNoBeacons indicates no beacons during Discovery procedure*/
  ZigBeeNoBeacons, 
                   
  /* ZigBeeReceivedKeyInClear indicates the Network Key is received 
   * in clear */
  ZigBeeReceivedKeyInClear, 
                          
  /* ZigBeeNoNWKKeyReceived indicates no Network key is received */
  ZigBeeNoNWKKeyReceived, 

  /* ZigBeeNoLinkKeyReceived indicates no Link key is received */
  ZigBeeNoLinkKeyReceived, 
                           
  /* ZigBeePreconfiguredKeyRequired indicates Preconfigured 
   * Link key is required */
  ZigBeePreconfiguredKeyRequired, 
                           
  /* ZigBeeChangedManagerAddress indicates nwk manager changed*/
  ZigBeeChangedManagerAddress 
                           
} ZigBeeNWKStatusInfo;

typedef union Address_Tag {
  uint16_t short_address;
  uint8_t IEEE_address[8];
} Address;

typedef enum Addr_Mode_Tag {
  /*! -0x00 - No address mode specified */
  g_NO_ADDR_MODE_c,

  /*! -0x01 - Reserved */
  g_RESERVED_MODE_c,

  /*! -0x02 - Short address mode */
  g_SHORT_ADDR_MODE_c,

  /*! -0x03 - Long or extended address mode */
  g_EXTENDED_ADDR_MODE_c
} Addr_Mode_t;

/*******************************************************************************
  APSDE_Data_Indication_Tag
  The data indication structure is received from APS layer(APS).
*******************************************************************************/

typedef struct APSDE_Data_Indication_Tag {
  /* Destination address in the received message.*/ 
  Address           dest_address;
  /* Destination address mode in the received
     message can take one of the following values:
     0x00       - Indirect data

     0x01       - 16-bit group address

     0x02       - 16-bit address of destination device

     0x03       - 64-bit extended address of destination device */ 
  uint8_t           dest_addr_mode;

  /* Destination endpoint of the received message*/ 
  uint8_t           dest_endpoint;

  /* This field indicates the source address mode in the received
     message. This field can have one of the following values: 
     0x00          - Indirect data
              
     0x01          - 16-bit group address
              
     0x02          - 16-bit address of destination device
              
     0x03          - 64-bit extended address of destination device */ 
  uint8_t           src_addr_mode;
  /* This field indicates the source address from which the message is
     originated. This field can have one of the following values:
     If the source address mode is 0x01, this field will have 16-bit address.
                        If mode is 0x03, this field will have 64-bit extended address */ 
  Address           src_address;
  /* Source endpoint from which the data frame was originated.*/ 
  uint8_t           src_endpoint;
  /* This field indicates the 16-bit profile ID */ 
  profile_id_t      profile_id;
  /* This field indicates the cluster ID*/ 
  cluster_id_t      cluster_id;
  /* This field indicates the length of the data received*/ 
  uint8_t           asdulength;
  /* Indicates whether the data frame is received through broadcast*/ 
  uint8_t           was_broadcast;
  /* Indicates if the received message was secured or
     not and type of the security applied. The enum values are as follows:

     *g_APS_UNSECURED_c - ASDU is received without any security.

     *g_Recieved_Nwk_Key_Secured_Asdu_c - ASDU is received with security
                                          using the Network Key.

     *g_Recieved_Link_Key_Secured_Asdu_c - ASDU is received with
                                           security using the link Key*/ 
  uint8_t           security_status;
  /* This field indicates the LQI of the received message*/ 
  uint8_t           link_quality;
  /* This field points to the actual message received */ 
  uint8_t           *a_asdu;
}APSDE_Data_Indication_t;




/*******************************************************************************
  node power descriptor specific to a device/node
  Field current_powermode_avail_power_sources - the first 4 bits of LSB gives the
  current sleep/ power saving mode of the node
  The MSB 4 bits gives the power sources available in this node
  Field current_powersource_currentpowersourcelevel - the first 4 bit of LSB gives
  the current power source
  The 4 bits of MSB gives the current power source level

  b3 b2 b1 b0 (bit3 bit2 bit1 bit0)

  0  0  0  0 - Receiver synchronized with the receiver on when idle
  subfield of the node descriptor
  0  0  0  1 - Receiver comes on periodically as defined by the node
  power descriptor
  0  0  1  0 - Receiver comes on when stimulated, e.g. by a user pressing
  a  button
  0  0  1  1
  to
  1  1  1  1 - Reserved
 *******************************************************************************/
typedef struct Node_Power_Descriptor_Tag {
  uint8_t          current_powermode_avail_power_sources;
  uint8_t          current_powersource_currentpowersourcelevel;
}Node_Power_Descriptor_t;

typedef enum {
  ZigBeeCoordinator,
  ZigBeeRouter,
  ZigBeeEndDevice,
  ZigBeeActiveEndDevice,
  ZigBeeMobileEndDevice,
  ZigBeeUnknownDevice,
} ZigBeeDeviceType;

typedef struct ZigBeeSimpleDescriptor_Tag {
  uint16_t         profileId;
  uint16_t         deviceId;
  uint8_t          deviceVersion;
  uint8_t          inputClusterCount;
  uint8_t          endpoint;
  uint8_t          outputClusterCount;
  uint16_t         *pInputClusterList;
  uint16_t         *pOutputClusterList;
}ZigBeeSimpleDescriptor_t; 

typedef struct ZigBeeNeighborTableEntry_Tag {
  uint16_t         shortId;
  uint8_t          averageLqi;
  uint8_t          incomingCost;
  uint8_t          outgoingCost;
  uint8_t          age;
  uint8_t          aIEEEAddress[8];
}ZigBeeNeighborTableEntry_t;

typedef struct ZigBeeRoutingTableEntry_Tag {
  uint16_t         destAddr;
  uint16_t         nextHop;
  uint8_t          status;
  uint8_t          age;
  uint8_t          concentratorType;
  uint8_t          routeRecordState;
}ZigBeeRoutingTableEntry_t;

typedef struct APSME_Address_Map_Table_Tag {
  uint8_t          a_IEEE_addr[8];
  uint16_t         nwk_addr;
}APSME_Address_Map_Table_t;

typedef struct APSDE_Data_Confirmation_Tag {
  Address          dest_address;
  uint8_t          dest_addr_mode;
  uint8_t          dest_endpoint;
  uint8_t          src_endpoint;
  uint8_t          status;
}APSDE_Data_Confirmation_t;

typedef enum
{
  ZigBee_Outgoing_Direct,
  ZigBee_Via_Address_Map,
  ZigBee_Via_Binding_Table,
  ZigBee_Via_Multicast,
  ZigBee_Broadcast
}ZigBee_Outgoing_Msg_Type;

typedef struct {
  uint8_t          DestEndpoint;
  uint8_t          SrcEndpoint;
  ProfileID        ProfileId;
  ClusterID        ClusterId;
  uint8_t          AsduLength;
  uint8_t          TxOptions;
  uint8_t          Radius;
  uint8_t          aReserved[0x31];
  uint8_t          aPayload[0x33];
}ZigBeeAPSDEDataRequest_t;


typedef struct ZDP_Bind_Request_Tag {
  uint8_t          a_src_addr[8];
  uint8_t          src_endpoint;
  uint8_t          a_cluster_id[2];
  uint8_t          dest_addr_mode;
  /*dest addr may be 2 bytes or 8 bytes depends on dst addr mode */
  uint8_t          a_dest_addr[8];
  /*dest endpoint may be 1 byte or zero bytes depends on dst addr mode */
  uint8_t          dest_endpoint;
}ZDP_Bind_Request_t;

typedef struct Simple_Descriptor_Tag {
  profile_id_t     app_profile_id;
  uint16_t         app_device_id;
  uint8_t          app_device_version;
  uint8_t          incluster_count;
  cluster_id_t const *p_incluster_list;
  uint8_t          outcluster_count;
  cluster_id_t const *p_outcluster_list;
}Simple_Descriptor_t;

/*enumerations for txoptions
 g_APS_Tx_Opt_Unsecured_c
 - data transmission with out security.

 g_APS_Tx_Opt_Ack_Req_c
 - data transmission without security and with APS Ack.

 g_APS_Tx_Opt_Use_NWK_Key_c
 - data transmission with security and with NWK Key.

 g_APS_Tx_Opt_Fragmentation_Permitted_c
 - data transmission with Fragmentation.

 g_APS_Tx_Opt_Secured_Nwk_Key_Ack_c
 - data transmission with security using NWK Key and with APS Ack.
 */
enum Tx_Options {
    g_APS_Tx_Opt_Unsecured_c = 0x00,
    g_APS_Tx_Opt_Ack_Req_c = 0x04,
    g_APS_Tx_Opt_Use_NWK_Key_c = 0x02,
    g_APS_Tx_Opt_Use_Link_c = 0x01,
    g_APS_Tx_Opt_Fragmentation_Permitted_c = 0x08,
    g_APS_Tx_Opt_Secured_Nwk_Key_Ack_c = 0x07
};

typedef struct Endpoint_Description_Tag {
  ZigBeeSimpleDescriptor_t  *p_simple_desc;
  uint8_t                   endpoint_id;
}Endpoint_Description_t;

/* Security Related Structures */
typedef enum ZigBeeKeyStructBitmask_Tag {

    /* This indicates that the key has a sequence number associated
     with it. (i.e. a Network Key).*/
    g_Key_Has_Sequence_Number_c = 0x01,

    /* This indicates that the key has an outgoing frame counter */
    g_Key_Has_Outgoing_Frame_Counter_c = 0x02,

    /* This indicates that the key has an incoming frame counter */
    g_Key_Has_Incoming_Frame_Counter_c = 0x04,

    /* This indicates that the key has an associated Partner IEEE address
     and the corresponding value within the ZigBeeKeyStructure_t has been
     populated with the data. */
    g_Key_Has_Partner_IEEE_Addr_c = 0x08,

    /* This indicates the key is authorized for use in APS data messages. If the
     key is not authorized for use in APS data messages it has not yet gone
     through a key agreement protocol, such as CBKE (i.e. ECC) */
    g_Key_Is_Authorized_c = 0x10

} ZigBeeKeyStructBitmask_t;

typedef enum Security_Key_Types_Tag {
    g_Trust_Center_Master_Key_c,
    g_Network_Key_c,
    g_Application_Master_Key_c,
    g_Link_Key_c,
    g_Trust_Center_Link_Key_c,
    g_Next_Network_Key_c
} Security_Key_Types;

typedef struct ZigBeeKeyStructure_Tag {
    ZigBeeKeyStructBitmask_t bitmask;
    Security_Key_Types       type;
    uint8_t          key[16];
    uint32_t         outgoingFrameCounter;
    uint32_t         incomingFrameCounter;
    uint8_t          sequenceNumber;
    uint8_t          apartnerIEEEAddress[g_EXTENDED_ADDRESS_LENGTH_c];
}ZigBeeKeyStructure_t;
/* End of Security Related Structures */


//ZCL related defines.
typedef enum Frame_Control
{
  g_Generic_Cmd_c,
  g_Cluster_Specific_Cmd_c,
  g_Manufacture_Specific_Bit_c   = 0x04,
  g_Disable_Default_Response_c   = 0x10
}Frame_Control;


typedef struct Read_Attribute_Command_Request_tag{
  uint8_t            a_attrib_list[2];			
}Read_Attribute_Command_Request_t;


typedef struct Write_Attribute_Command_Tag{
   uint8_t           a_attribute_id[2];
   uint8_t           attribute_data_type;
   uint8_t           a_attribute_data[1];
}Write_Attribute_Command_t;

typedef struct Discover_Attribute_Command_Tag
{
   uint8_t           a_start_attribute_identifier[2];
   uint8_t           maximum_count;
}Discover_Attribute_Command_t;


typedef struct Report_Attributes_Command_Tag
{
   uint8_t           a_attribute_identifier[2];
   uint8_t           attribute_data_type;
   uint8_t           attribute_data[1];
}Report_Attributes_Command_t;

typedef struct Attribute_reporting_Configuration_Record_Tag
{
	uint8_t            direction;
	uint8_t            a_attrbute_id[2]; /* in little endian order */
  uint8_t            data_type;
  uint8_t            a_minimum_reporting_interval[2];
  uint8_t            a_maximum_reporting_interval[2];
  uint8_t            a_reportable_change[2];
}Attribute_reporting_Configuration_Record;



typedef struct Configure_Report_Attributes_Command_Tag
{
  Attribute_reporting_Configuration_Record reporting_Configuration_Record[2];
}Configure_Report_Attributes_Command_t;

typedef struct Read_Reporting_Configurartion_Command_Tag
{
   uint8_t           irection;
   uint8_t           _attribute_id[2];
}Read_Reporting_Configurartion_Command_t;


typedef struct ZCL_Command_Tag
{
   uint8_t command_identifier;
   union Foundation_Commands
   {
      Read_Attribute_Command_Request_t   read_attribute_command;
      Write_Attribute_Command_t          write_attribute_command;
      Discover_Attribute_Command_t       discover_attribute_command;
      Report_Attributes_Command_t        report_attributes_command;
      Configure_Report_Attributes_Command_t configure_report_attributes_command;
      Read_Reporting_Configurartion_Command_t  a_attribute_records[1];
   }Foundation_Commands;
}ZCL_Command_t;


typedef struct ZCL_Header_And_Payload_Tag
{
   uint8_t           frame_control;
   uint8_t           a_manufacture_code[2];
   uint8_t           transaction_sequence_number;
   ZCL_Command_t ZCL_Command;
}ZCL_Header_And_Payload_t;






typedef struct App_ZCL_Request_Tag
{
   uint8_t           command_type;
   uint8_t           manufacture_specific;
   uint8_t           a_manufacturer_code[2];
   uint8_t           disable_default_response;
   ZCL_Command_t     ZCL_command_received;
}App_ZCL_Request_t;




#define DEAFULT_RADIUS 0x5

#endif
