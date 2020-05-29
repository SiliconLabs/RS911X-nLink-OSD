/**
 *  @file     rsi_zigb_api.h
 *   @version  1.0
 *   @date     2014-Sep-13
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


#ifndef RSI_ZIGB_API_H
#define RSI_ZIGB_API_H

#include "rsi_zigb_types.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_interfaces.h"

#ifdef ZB_ONLY
#include "rsi_zigb_oper_mode.h"
#endif

#define SERIAL_READ_BUFFER_COUNT  10 
#define MAX_PACKET_SIZE           256
#define MAX_SCAN_DURATION	        0xA

//! Interface Types
#define MANAGEMENT_INTERFACE	   0x01
#define DATA_INTERFACE			     0x02
#define SECURITY_INTERFACE		   0x03
#define BINDING_INTERFACE		     0x04
#define PACKET_DATA				       0x05
#define INTERFACE_CALLBACK		   0x06
#define INTERNAL_MANAGEMENT_INTERFACE	0x07

//! Direction
#define DIR_HOST_TO_DEVICE 0x01
#define DIR_DEVICE_TO_HOST 0x02

//! Commands
//! Management APIs
#define ZIGBEEFORMNETWORK                        0x01
#define ZIGBEEJOINNETWORK                        0x02
#define ZIGBEEPERMITJOIN                         0x03
#define ZIGBEELEAVENETWORK                       0x04
#define ZIGBEEFINDNETWORKANDPERFORMREJOIN        0x05
#define ZIGBEEREJOINNETWORK                      0x06
#define ZIGBEENETWORKRESTORE                     0x07
#define ZIGBEEINITIATESCAN                       0x08
#define ZIGBEESTOPSCAN                           0x09
#define ZIGBEENETWORKSTATE                       0x0A
#define ZIGBEESTACKISUP                          0x0B
#define ZIGBEEGETSELFIEEEADDRESS                 0x0C
#define ZIGBEEISITSELFIEEEADDRESS                0x0D
#define ZIGBEEGETSELFSHORTADDRESS                0x0E
#define ZIGBEESETMANUFACTURERCODEFORNODEDESC     0x0F
#define ZIGBEESETPOWERDESCRIPTOR                 0x10
#define ZIGBEESETMAXMINCOMINGTXFRSIZE            0x11
#define ZIGBEESETMAXMOUTGOINGTXFRSIZE            0x12
#define ZIGBEESETOPERATINGCHANNEL                0x13
#define ZIGBEEGETDEVICETYPE                      0x14
#define ZIGBEEGETOPERATINGCHANNEL                0x15
#define ZIGBEEGETSHORTPANID                      0x16
#define ZIGBEEGETEXTENDEDPANID                   0x17
#define ZIGBEEGETENDPOINTID                      0x18
#define ZIGBEEGETSIMPLEDESCRIPTOR                0x19
#define ZIGBEEGETENDPOINTCLUSTER                 0x1A
#define ZIGBEEGETSHORTADDRFORSPECIFIEDIEEEADDR   0x1B
#define ZIGBEESTACKPROFILE                       0x1C
#define ZIGBEEGETIEEEADDRFORSPECIFIEDSHORTADDR   0x1D
#define ZIGBEEREADNEIGHBORTABLEENTRY             0x1E
#define ZIGBEEGETROUTETABLEENTRY                 0x1F
#define ZIGBEETREEDEPTH                          0x20
#define ZIGBEEGETNEIGHBORTABLEENTRYCOUNT         0x21
#define ZIGBEEGETCHILDSHORTADDRESSFORTHEINDEX    0x22
#define ZIGBEEGETCHILDINDEXFORSPECIFIEDSHORTADDR 0x23
#define ZIGBEEGETCHILDDETAILS                    0x24
#define ZIGBEEENDDEVICEPOLLFORDATA               0x25
#define ZIGBEEREADCOUNTOFCHILDDEVICES            0x26
#define ZIGBEEREADCOUNTOFROUTERCHILDDEVICES      0x27
#define ZIGBEEREADMAXCHILDROUTERDEVICESCOUNT     0x28
#define ZIGBEEGETPARENTSHORTADDRESS              0x29
#define ZIGBEEGETPARENTIEEEADDRESS               0x2A
#define ZIGBEEINITIATEENERGYSCANREQUEST          0x2B
#define ZIGBEEBROADCASTNWKMANAGERREQUEST         0x2C
#define ZDPSENDNWKADDRREQUEST                    0x2D
#define ZDPSENDIEEEADDRREQUEST                   0x2E
#define ZDPSENDDEVICEANNOUNCEMENT                0x2F
#define ZDPSENDMATCHDESCRIPTORSREQUEST           0x30
#define ZIGBEEACTIVEENDPOINTSREQUEST             0x31
#define ZDPSENDPOWERDESCRIPTORREQUEST            0x32
#define ZDPSENDNODEDESCRIPTORREQUEST             0x33
#define ZIGBEESIMPLEDESCRIPTORREQUEST            0x34
#define ZIGBEEGETADDRESSMAPTABLEENTRY            0x35
#define ZIGBEESENDUNICASTDATA                    0x36
#define ZIGBEESENDGROUPDATA                      0x37
#define ZIGBEESENDBROADCASTDATA                  0x38
#define ZIGBEEGETMAXAPSPAYLOADLENGTH             0x39
#define ZIGBEESETBINDINGENTRY                    0x3A
#define ZIGBEEDELETEBINDING                      0x3B
#define ZIGBEEISBINDINGENTRYACTIVE               0x3C
#define ZIGBEECLEARBINDINGTABLE                  0x3D
#define ZIGBEEBINDREQUEST                        0x3E
#define ZIGBEEENDDEVICEBINDREQUEST               0x3F
#define ZIGBEEUNBINDREQUEST                      0x40

//security API
#define ZIGBEEGETKEY                             0x41
#define ZIGBEEHAVELINKKEY                        0x42
#define ZIGBEESWITCHNETWORKKEYHANDLER            0x43
#define ZIGBEEREQUESTLINKKEY                     0x44
#define ZIGBEEGETKEYTABLEENTRY                   0x45
#define ZIGBEESETKEYTABLEENTRY                   0x46
#define ZIGBEEADDORUPDATEKEYTABLEENTRY           0x47
#define ZIGBEEFINDKEYTABLEENTRY                  0x48
#define ZIGBEEERASEKEYTABLEENTRY                 0x49

#define ZIGBEESETSIMPLEDESCRIPTOR                0x4A

#define ZIGBEELASTTYPE                           0x4A

//!  Call back Handlers
#define APPSCANCOMPLETEHANDLER                   (ZIGBEELASTTYPE + 1)
#define APPENERGYSCANRESULTHANDLER               (APPSCANCOMPLETEHANDLER+1)
#define APPNETWORKFOUNDHANDLER                   (APPENERGYSCANRESULTHANDLER+1)
#define APPZIGBEESTACKSTATUSHANDLER              (APPNETWORKFOUNDHANDLER +1)
#define ZIGBEECHILDJOINHANDLER                   (APPZIGBEESTACKSTATUSHANDLER+1)
#define APPCHILDJOINHANDLER                      ZIGBEECHILDJOINHANDLER
#define APPINCOMINGMANYTOONEROUTEREQUESTHANDLER  (ZIGBEECHILDJOINHANDLER + 1)
#define APPHANDLEDATAINDICATION                  (APPINCOMINGMANYTOONEROUTEREQUESTHANDLER + 1)
#define APPHANDLEDATACONFIRMATION                (APPHANDLEDATAINDICATION + 1)
#define APPADDRESSCONFLICTHANDLER                (APPHANDLEDATACONFIRMATION + 1)

#define ZIGBEEGETBINDINGINDICES                  0x60
#define ZIGBEESTACKINIT                          0x61
#define ZIGBEESTACKRESET                         0x62
#define ZIGBEESLEEPRQT                           0x63
#define ZIGBEEWAKEUPRQT                          0x64

#define ZIGBEEUPDATESAS                    0x65
#define ZIGBEEUPDATEZDO                    0X66
#define ZIGBEESETEXTENDEDPANID             0x67
#define ZIGBEEINITPS                       0x68

#define ZIGBEE_OPER_MODE_RSP               0xFE
#define ZIGBEE_CARD_READY                  0xFF
#define ZIGBEEDEINITSTACK                  0xFF

//! Status defines
#define TRUE                         1
#define FALSE                        0
#define RSI_ZB_SUCCESS                0x00
#define	RSI_ZB_FAIL                   0xFF
#define RSI_ZB_CRC_FAIL					      0x02
#define RSI_ZB_RCVD_UNKNOWN_COMMAND   0x03
#define RSI_ZB_ERROR_SEQUENCE_NUMBER  0x04
#define RSI_ZB_DIRECTION_ERROR        0x05
#define RSI_ZB_INTERFACE_ERROR        0x06
#define RSI_ZB_COMMAND_ERROR          0x06
#define RSI_ZB_PAYLOAD_SIZE_ERROR     0x07


/** @brief  Defines the value for Success */
#define g_SUCCESS_c                 0x00

/** @brief  Defines the value for Failure */
#define g_FAILURE_c                 0xFF



/*******************************************************************************
 Startup_Attribute_Set_Tag
 -------------------------
 a_extended_pan_id[8]
 - This field holds the extended PAN ID of the network, in which the
 device needs to be a member.

 channel_mask
 - This field contains the set of channels, the device needs to scan
 as part of the network join or formation procedure.

 startup_control
 - This field indicates how the device needs to respond or start
 depending on the startup_control value.

 a_trust_center_address[8]
 - This field contains the 64-bit extended address of the TC.

 a_network_key[16]
 - This field indicates the key that is used for communication in
 the network.

 use_insecure_join
 - This field is a flag which indicates that a device needs to use
 insecure join during startup.

 network_manager_address[8]
 -This field indicates the 64-bit extended address of the network
 manager.

 scan_attempts
 - This field defines the number of scan attempts to be done by the
 device.

 time_between_scans
 - This field defines the time gap between two successive scans.

 rejoin_interval
 - This field indicates the time interval after which a device tries
 to rejoin the network, if disconnected.

 max_rejoin_interval
 - This field indicates the upper time limit within which multiple
 rejoins are done, that are separated by rejoin_intervals.

 indirect_poll_rate
 - This field indicates the rate at which an end device polls for
 messages, from its parent.

 parent_retry_threshold
 -This field indicates the parent retry threshold.

 a_pan_id
 -This field indicates the PAN ID of the device.

 a_master_key[16]
 -This field indicates the master key the node need to use.

 a_preconfigured_link_key[16]
 -This field indicates the link key the node need to use.

 end_device_bind_timeout
 - This field indicates the end device bind timeout.
 *******************************************************************************/
typedef struct Startup_Attribute_Set_Tag {
    uint8_t a_extended_pan_id[8];
    uint32_t channel_mask;
    uint8_t startup_control;
    uint8_t use_insecure_join;
    uint8_t scan_attempts;
    uint8_t parent_retry_threshold;
    uint8_t a_trust_center_address[8];
    uint8_t a_network_key[16];
    uint16_t time_between_scans;
    uint16_t rejoin_interval;
    uint16_t max_rejoin_interval;
    uint16_t indirect_poll_rate;
    uint16_t a_pan_id;
    uint16_t network_manager_address;
    uint8_t a_trustcenter_master_key[16];
    uint8_t a_preconfigured_link_key[16];
    uint8_t end_device_bind_timeout;
} Startup_Attribute_Set_t;



/*******************************************************************************
 ZDO_Configuration_Table_Tag
 ---------------------------
 config_permit_join_duration
 - This field indicates the duration for which the device permits
 other devices to join.

 config_NWK_secure_all_frames
 - TC uses this field to determine, whether the network layer
 security is applied to all frames in the network or not.

 config_formation_attempts
 - This field indicates the number of formation retrials, after
 formation failure.

 config_scan_duration
 - The field indicates the duration of active scan while performing
 startup, join or rejoin the network.

 config_join_attempts
 - This field indicates the number of times join is retried once
 the join fails.

 config_preconfigured_key
 - This field indicates whether a preconfigured key is already
 available in the device or not.

 a_config_trust_center_short_address
 - This field holds the short address of the TC.

 automatic_poll_allowed
 - This field indicates whether an end device does an auto poll or
 not.

 config_authentication_poll_rate
 - This field indicates the time out in milliseconds for a device to
 wait for authentication after joining the network.

 config_switch_key_time
 - This field indicates the time out in seconds for switching the
 key after receiving a Switch Key Request from the TC.

 config_aps_ack_poll_time_out
 -The field indicates the time out after which an end device polls
 its parent, to retrieve an APS acknowledgement.
 *******************************************************************************/
typedef struct ZDO_Configuration_Table_Tag {
    uint8_t config_permit_join_duration;
    uint8_t config_NWK_secure_all_frames;
    uint8_t config_formation_attempts;
    uint8_t config_scan_duration;
    uint8_t config_join_attempts;
    uint8_t config_preconfigured_key;
    uint16_t a_config_trust_center_short_address;
    uint8_t automatic_poll_allowed;
    uint8_t config_authentication_poll_rate;
    uint16_t config_switch_key_time;
    uint8_t config_security_level;
    uint8_t config_aps_ack_poll_time_out;
    uint8_t a_manufacturer_code[2];
} ZDO_Configuration_Table_t;

/* API Structure information*/
typedef struct {
  uint32_t                timeout;
}sleepReqFrameSnd;

typedef struct {
  uint8_t                 RadioChannel;
  uint8_t                 power;
  uint8_t                 ExtPanId[8];
}formNetworkFrameSnd;

typedef struct {
  uint8_t                 DeviceType;
  uint8_t                 RadioChannel;
  uint8_t                 power;
  uint8_t                 ExtPanId[8];
}joinNetworkFrameSnd;

typedef struct {
  uint8_t                 PermitDuration;
}permitJoinFrameSnd;

typedef struct {
  uint32_t                ChannelMask;
  uint8_t                 Secured;
}findNWKRejoinFrameSnd;

typedef struct {
  uint8_t                 Secured;
}rejoinNWKFrameSnd;

typedef struct {
  uint32_t                ChannelMask;
  uint8_t                 ScanType;
  uint8_t                 duration;
}initScanFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
}isitSelfIEEEFrameSnd;

typedef struct {
  uint16_t                ManufacturerCode;
}setManufFrameSnd;

typedef struct {
  uint8_t                 PowerSources;
  uint8_t                 CurPowerLevel;
}setPowerDescFrameSnd;

typedef struct {
  uint16_t                MaxIncomingTxfrSize;
}setMaxIncomingTxfrFrameSnd;

typedef struct {
  uint16_t                MaxOutgoingTxfrSize;
}setMaxOutTxfrFrameSnd;

typedef struct {
  uint8_t                 Channel;
}setOperChanFrameSnd;

typedef struct {
  uint8_t                 Index;
}getEndPointIdFrameSnd;

typedef struct {
  uint8_t                 EndPointId;
}getSimpleDescFrameSnd;

typedef struct {
  uint8_t                 EndPointId;
  uint8_t                 ClusterType;
  uint8_t                 ClusterIndex;
}getEPClusterFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
}getShortAddrForIeeeAddrFrameSnd;

typedef struct {
  uint16_t                 ShortAddr;
}getIeeeAddrForShortAddrFrameSnd;

typedef struct {
  uint8_t                 Index;
}readNeighborTableFrameSnd;

typedef struct {
  uint8_t                 Index;
}getRouteTableFrameSnd;

typedef struct {
  uint8_t                 ChildIndex;
}getChildShortAddrFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
}getChildIndexForShortAddrFrameSnd;

typedef struct {
  uint8_t                 Index;
}getChildDetailsFrameSnd;

typedef struct {
  uint32_t                ScanChannels;
  uint16_t                DestAddr;
  uint16_t                ScanRetry;
  uint8_t                 ScanDuration;
}initEnergyScanFrameSnd;

typedef struct {
  uint8_t                 ShortAddr;
  uint8_t                 ActiveChannels;
}bcastNWKManagerFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
  uint8_t                 RequestType;
  uint8_t                 StartIndex;
}getZDPNWKShortAddrFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint8_t                 RequestType;
  uint8_t                 StartIndex;
  uint8_t                 APSAckRequired;
}getZDPIEEEAddrFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint8_t                 APSAckRequired;
}activeEPOfShortAddrFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint8_t                 APSAckRequired;
}powerDescFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint8_t                 APSAckRequired;
}nodeDescFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint8_t                 EndPointId;
}getSimpleDescOfShortAddrFrameSnd;

typedef struct {
  uint16_t                ProfileId;
  uint16_t                ClusterId;
  uint8_t                 msgType;
  uint8_t                 ieee_Addr[8];
  uint8_t                 DestEndpoint;
  uint8_t                 SrcEndpoint;
  uint8_t                 AsduLength;
  uint8_t                 TxOptions;
  uint8_t                 Radius;
  uint8_t                 Data[120]; //AsduLength Data
}unicastDataFrameSnd;

typedef struct {
  uint16_t                ProfileId;
  uint16_t                ClusterId;
  uint16_t                group_addr;
  uint8_t                 DestEndpoint;
  uint8_t                 SrcEndpoint;
  uint8_t                 AsduLength;
  uint8_t                 TxOptions;
  uint8_t                 Radius;
  uint8_t                 Data[120]; //AsduLength Data
}groupDataFrameSnd;

typedef struct {
  uint16_t                ProfileId;
  uint16_t                ClusterId;
  uint8_t                 DestEndpoint;
  uint8_t                 SrcEndpoint;
  uint8_t                 AsduLength;
  uint8_t                 TxOptions;
  uint8_t                 Radius;
  uint8_t                 Data[120]; //AsduLength Data
}bcastDataFrameSnd;

typedef struct {
  uint8_t                 SrcIEEEAddr[8];
  uint8_t                 SrcEndpoint;
  uint16_t                ClusterId;
  uint8_t                 DestAddrMode;
  uint8_t                 DestIEEEAddr[8];
  uint8_t                 DestEndpoint;
}setBindEntryFrameSnd;

typedef struct {
  uint8_t                 BindIndex;
}delBindEntryFrameSnd;

typedef struct {
  uint8_t                 BindIndex;
}isBindEntryActiveFrameSnd;

typedef struct {
  uint16_t                SrcShortAddr;
  uint16_t                ClusterId;
  uint8_t                 SrcIEEEAddr[8];
  uint8_t                 SrcEndpoint;
  uint8_t                 DestAddrMode;
  Address                 DestAddress;
  uint8_t                 DestEndpoint;
  uint8_t                 APSAckRequired;
}bindReqFrameSnd;

typedef struct {
  uint8_t                 EndPointId;
  uint8_t                 APSAckRequired;
}endDevBindFrameSnd;

typedef struct {
  uint16_t                SrcShortAddr;
  uint16_t                ClusterId;
  uint8_t                 SrcIEEEAddr[8];
  uint8_t                 SrcEndpoint;
  uint8_t                 DestAddrMode;
  Address                 DestAddress;
  uint8_t                 DestEndpoint;
  uint8_t                 APSAckRequired;
}unbindReqFrameSnd;

typedef struct {
  uint16_t                ProfileId;
  uint16_t                DevId;
  uint8_t                 EndPointId;
  uint8_t                 DevVersion;
  uint8_t                 InClusterCnt;
  uint8_t                 OutClusterCnt;
  uint8_t                 ClusterInfo[40];
}setSimpleDescFrameSnd;

typedef struct {
  uint16_t                ShortAddr;
  uint16_t                ProfileId;
  uint16_t                DestAddress;
  uint8_t                 InClusterCnt;
  uint8_t                 OutClusterCnt;
  uint8_t                 APSAckRequired;
  uint8_t                 ClusterInfo[40];
}sendMatchDescFrameSnd;

typedef struct {
  uint8_t                 Index;
}getAddrMapTableFrameSnd;

typedef struct {
  uint8_t                 KeyType;
}getKeyFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
}haveLinkKeyFrameSnd;

typedef struct {
  uint8_t                 TrustCenterIEEEAddr[8];
  uint8_t                 PartnerIEEEAddr[8];
}reqLinkKeyFrameSnd;

typedef struct {
  uint8_t                 Index;
}getKeyTableFrameSnd;

typedef struct {
  uint8_t                 Index;
  uint8_t                 ieee_Addr[8];
  uint8_t                 LinkKey;
  uint8_t                 KeyData[16];
}setKeyTableFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
  uint8_t                 LinkKey;
  uint8_t                 KeyData[16];
}addKeyTableFrameSnd;

typedef struct {
  uint8_t                 ieee_Addr[8];
  uint8_t                 LinkKey;
}findKeyTableFrameSnd;

typedef struct {
  uint8_t                 Index;
}eraseKeyTableFrameSnd;

typedef struct {
  uint8_t                 TcLinkKey[16];
}setTcMasterKeyFrameSnd;

typedef struct {
  uint8_t                 PcLinkKey[16];
}preConfigLinkKeyFrameSnd;

typedef struct {
  uint8_t                 ExPanId[16];
}setExtPanIdFrameSnd;

typedef struct {
  uint8_t                 ps_en;
  uint8_t                 deep_sleep_wkp_period;
  uint8_t                 slp_mode;
}pwrModeFrameSnd;



/****** Global frame desc structures *****/

extern const    uint8_t   rsi_zigb_frameInitStack[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameResetStack[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSleepReq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameWakeupReq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameFormNWK[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameJoinNWK[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_framePermitJoin[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameLeaveNWK[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameFindNWKnRejoin[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameRejoinNWK[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameNWKRestore[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameInitScan[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameStopScan[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameNWKState[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameStackIsUp[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetSelfIEEE[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameIsItSelfIEEE[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetSelfShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetManufNodeDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetPwrDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetMaxIncmgSize[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetMaxOutSize[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetChan[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetOperChan[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetDevType[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetShortPanId[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetExtPanId[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetEP[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetSimpleDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetEPCluster[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetShortAddrForIEEE[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetStackProfile[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetIEEEForShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameReadNeighborTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetRouteTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameTreeDepth[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetNeighborTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetChildShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetChildIndexForShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetChildDetails[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_framePollForData[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameReadChildCnt[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameReadRouterChildCnt[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetParentShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetParentIEEEAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameInitEnergyScan[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameBcastNWKManagerReq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetZDPNWKShortAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetZDPIEEEAddr[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameZDPDevAnnounce[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameActiveEPreq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameZDPPwrDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameZDPNodeDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSimpleDescReq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameUcastData[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGroupData[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameBcastData[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetMaxAPSSize[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetSimpleDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSendMatchDesc[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetTCMasterKey[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_framePreconfigureLinkKey[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetExtPanId[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetBindingIndices[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetBindEntry[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameDelBindEntry[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameIsBindEntryActive[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameClearBindTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameBindReq[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameEndDevBind[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameUnbind[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetAddrMapTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetKey[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameHaveLinkKey[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameReqLinkKey[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameGetKeyTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameSetKeyTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameAddKeyTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameFindKeyTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameEraseKeyTable[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameUpdateSAS[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameUpdateZDO[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_frameDeInitStack[RSI_ZIGB_BYTES_2];
extern const    uint8_t   rsi_zigb_framePwrMode[RSI_ZIGB_BYTES_2];

/* Function Declarations */
int16_t rsi_zigb_execute_cmd(uint8_t *desc, uint8_t *payload, uint16_t length);
void rsi_zigb_app_cb_handler(uint8_t cmd_id, uint8_t *buffer);
void rsi_zigb_app_rsp_msg_handler(uint8_t cmd_id, uint8_t length, uint8_t *buffer);
rsi_zigb_uCmdRsp* rsi_zigb_app_frame_process(uint8_t *buffer);

int16_t rsi_zigb_init_stack(void);
int16_t rsi_zigb_form_network ( uint8_t  RadioChannel, uint8_t power ,uint8_t * pExtendedPanId );
int16_t rsi_zigb_join_network(uint8_t DeviceType, uint8_t RadioChannel,uint8_t power ,uint8_t * pExtendedPanId);
int16_t rsi_zigb_permit_join(uint8_t PermitDuration);
int16_t rsi_zigb_leave_network(void);
int16_t rsi_zigb_find_network_and_perform_rejoin(BOOL Secured, uint32_t channelMask);
int16_t rsi_zigb_rejoin_network(BOOL Secured);
int16_t rsi_zigb_network_restore(void);
int16_t rsi_zigb_initiate_scan(uint8_t scanType, uint32_t channelMask,uint8_t duration);
int16_t rsi_zigb_stop_scan(void);
int16_t rsi_zigb_network_state(void);
int16_t rsi_zigb_stack_is_up(void);
int16_t rsi_zigb_get_self_ieee_address(void);
int16_t rsi_zigb_is_it_selfieee_address(uint8_t *);
int16_t rsi_zigb_get_self_short_address(void);
int16_t rsi_zigb_set_manufacturer_code_for_node_desc(uint16_t);
int16_t rsi_zigb_set_power_descriptor(Node_Power_Descriptor_t *);
int16_t rsi_zigb_set_maxm_incoming_txfr_size(uint16_t);
int16_t rsi_zigb_set_maxm_outgoing_txfr_size(uint16_t);
int16_t rsi_zigb_set_operating_channel(uint8_t );
int16_t rsi_zigb_get_device_type(void);
int16_t rsi_zigb_get_operating_channel(void);
int16_t rsi_zigb_get_short_pAN_id(void);
int16_t rsi_zigb_get_extended_pan_id(uint8_t * );
int16_t rsi_zigb_get_endpoint_id(uint8_t);
int16_t rsi_zigb_get_simple_descriptor(uint8_t endpointId);
int16_t rsi_zigb_set_simple_descriptor(uint8_t endpointId, Simple_Descriptor_t *SimpleDesc);
int16_t rsi_zigb_get_endpoint_cluster(uint8_t , uint8_t ,uint8_t);
int16_t rsi_zigb_get_short_addr_for_specified_ieee_addr(uint8_t *);
int16_t rsi_zigb_stack_profile(void);
int16_t rsi_zigb_get_ieee_addr_for_specified_short_addr(uint16_t );
int16_t rsi_zigb_read_neighbor_table_entry(uint8_t);
int16_t rsi_zigb_get_route_table_entry(uint8_t);
int16_t rsi_zigb_tree_depth(void);
int16_t rsi_zigb_get_neighbor_table_entry_count(void);
int16_t rsi_zigb_get_child_short_address_for_the_index(uint8_t);
int16_t rsi_zigb_get_child_index_for_specified_short_addr(uint16_t);
int16_t rsi_zigb_get_child_details(uint8_t);
int16_t rsi_zigb_end_device_poll_for_data( void );
int16_t rsi_zigb_read_count_of_child_devices(void);
int16_t rsi_zigb_read_count_of_router_child_devices(void);
int16_t rsi_zigb_read_max_child_router_devices_count(void);
int16_t rsi_zigb_get_parent_short_address(void);
int16_t rsi_zigb_Get_parent_ieee_address(uint8_t *);
int16_t rsi_zigb_initiate_energy_scan_request(uint16_t ,uint32_t , uint8_t , uint16_t );
int16_t rsi_zigb_broadcast_nwk_manager_request(uint16_t , uint32_t );
int16_t rsi_zigb_send_nwk_addr_request(uint8_t * , BOOL , uint8_t );
int16_t rsi_zigb_send_ieee_addr_request(uint16_t , BOOL ,uint8_t , BOOL );
int16_t rsi_zigb_send_device_announcement(void);
int16_t rsi_zigb_send_match_descriptors_request(uint16_t , uint16_t , uint8_t *, uint8_t ,uint8_t *, uint8_t , BOOL ,uint16_t );
int16_t rsi_zigb_active_endpoints_request(uint16_t, uint8_t);
int16_t rsi_zigb_send_power_descriptor_request(uint16_t, uint8_t);
int16_t rsi_zigb_send_node_descriptor_request(uint16_t, uint8_t);
int16_t rsi_zigb_simple_descriptor_request(uint16_t, uint8_t);
int16_t rsi_zigb_get_address_map_table_entry(uint8_t );
int16_t rsi_zigb_zdp_send_node_descriptor_request(uint16_t shortAddress, uint8_t APSAckRequired);
int16_t rsi_zigb_zdp_send_power_descriptor_request(uint16_t shortAddress, uint8_t APSAckRequired);
int16_t rsi_zigb_zdp_send_ieee_addr_request(uint16_t shortAddress, BOOL RequestType,uint8_t StartIndex, BOOL APSAckRequired);
int16_t rsi_zigb_send_unicast_data(ZigBee_Outgoing_Msg_Type ,Address , ZigBeeAPSDEDataRequest_t *);
int16_t rsi_zigb_send_group_data( GroupID GroupAddress, ZigBeeAPSDEDataRequest_t * pAPSDERequest);
int16_t rsi_zigb_send_broadcast_data(ZigBeeAPSDEDataRequest_t * pAPSDERequest);
int16_t rsi_zigb_get_max_aps_payload_length(void);
int16_t rsi_zigb_set_binding_entry(ZDP_Bind_Request_t * pSetBindingEntry);
int16_t rsi_zigb_delete_binding(uint8_t bindIndex);
int16_t rsi_zigb_is_binding_entry_active(uint8_t bindIndex);
int16_t rsi_zigb_clear_binding_table(void);
int16_t rsi_zigb_bind_request(uint16_t , uint8_t *, uint8_t , uint16_t , uint8_t ,Address , uint8_t , BOOL );
int16_t rsi_zigb_end_device_bind_request(uint8_t , BOOL );
int16_t rsi_zigb_unbind_request(uint16_t , uint8_t *, uint8_t , uint16_t , uint8_t ,Address , uint8_t , BOOL );
BOOL rsi_zigb_is_it_self_ieee_address(uint8_t *pIEEEAddress);
int16_t rsi_zigb_set_maxm_out_going_txfr_size(uint16_t MaxOutgoingTxfrSize);
int16_t rsi_zigb_get_short_panid(void);
int16_t rsi_zigb_get_extended_panid(void);
int16_t rsi_zigb_get_parent_ieee_address(void);
int16_t rsi_zigb_zdp_send_device_announcement(void);
int16_t rsi_zigb_zdp_send_nwk_addr_request(uint8_t * pIEEEAddrOfInterest, BOOL RequestType, uint8_t StartIndex);

/* Securty APIs */
int16_t rsi_zigb_get_key(Security_Key_Types keytype);
int16_t rsi_zigb_have_link_key(uint8_t *pRemoteDeviceIEEEAddr);
int16_t rsi_zigb_switch_network_key_handler( uint8_t sequenceNumber );
int16_t rsi_zigb_request_link_key(uint8_t* trustCentreIEEEAddr, uint8_t* partnerIEEEAddr);
int16_t rsi_zigb_get_key_table_entry (uint8_t index, ZigBeeKeyStructure_t *pResult);
int16_t rsi_zigb_set_key_table_entry( uint8_t index, uint8_t * pIEEEAddress, BOOL linkKey, uint8_t * pKeyData );
int16_t rsi_zigb_add_or_update_key_table_entry(uint8_t *pIEEEAddress, BOOL linkKey, uint8_t *pKeyData, uint8_t *);
int16_t rsi_zigb_find_key_table_entry(uint8_t * pIEEEAddress, BOOL linkKey);
int16_t rsi_zigb_erase_key_table_entry(uint8_t index);


/** Supportive functions **/
void rsi_zigb_copy_xt_panid(uint8_t* Panid, uint8_t *buffer);
uint8_t rsi_zigb_pkt_parser(uint8_t *buffer);
void rsi_zigb_xtract_xt_panid(uint8_t* buffer, uint8_t *Panid);
uint8_t rsi_zigb_mcmp(uint8_t *buf1,uint8_t *buf2,uint8_t len);
uint8_t rsi_zigb_form_return_packet_mgmt(uint8_t seq_number, uint8_t commandType, uint8_t payloadSize, uint8_t *payload);
RSI_ZB_STATUS rsi_zigb_send_packet(uint8_t *buffer, uint8_t length);
uint8_t* rsi_zigb_get_host_pkt_buffer(void);
void rsi_zigb_reset_host_pkt_info(uint8_t *);
uint8_t* rsi_zigb_get_empty_host_buffer(void);
void rsi_zigb_uint32to_buffer(uint32_t data, uint8_t* buffer);
void rsi_zigb_mcpy(uint8_t *src, uint8_t *dst, uint8_t size);
void rsi_zigb_memset(uint8_t *src, uint8_t data, uint8_t size);
uint16_t rsi_zigb_buffer2_uint16(uint8_t* buffer);
uint32_t rsi_zigb_buffer2_uint32(uint8_t* buffer);
void rsi_zigb_uint16to_buffer(uint16_t data, uint8_t* buffer);
int32_t rsi_zigb_framework_init(void);
uint8_t* rsi_zigb_form_pkt_header_sz(uint8_t seq_number, uint8_t interface, uint8_t command, uint8_t payload_length);
int16_t rsi_zigb_addcrc(uint8_t * buf);
int16_t rsi_zigb_reset_stack(void);
int16_t rsi_zigb_sleep_request(uint32_t timeout);
int16_t rsi_zigb_wake_up_request(void);

#ifdef ZB_ONLY
int16_t rsi_zigb_oper_mode(rsi_zigb_uOperMode *);
#endif

int16_t rsi_zigb_update_sas(Startup_Attribute_Set_t *);
int16_t rsi_zigb_update_zdo_configuration(ZDO_Configuration_Table_t *);
int16_t rsi_zigb_set_extended_pan_Id(uint8_t *);

uint8_t rsi_zigb_app_send_data( uint8_t direction, uint8_t commandType, uint8_t destEP, uint16_t dest_short_address,
		uint8_t commandId, uint16_t cluster, uint8_t dataLength,uint8_t* payloadPointer );

//! Interface functions
int16_t rsi_frame_read(uint8_t *);
/** End of Supportive functions **/
#endif
