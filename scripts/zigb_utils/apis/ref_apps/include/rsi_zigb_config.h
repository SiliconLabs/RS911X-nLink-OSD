
#ifndef _ZIGB_APP_CONFIG_H_
#define _ZIGB_APP_CONFIG_H_

/******************************************************************************/
/* Default values of the Startup Attribute Set */
/******************************************************************************/
/*******************************************************************************
 g_EXTENDED_PAN_ID_c
 - This macro defines the default value of the extended PAN ID of
 the network in which the device is operating.

 Specify the ExtendedPANID to Set/Join in case of router and Coordinator and
 Sepcify Zero's for other
******************************************************************************/
#define g_EXTENDED_PAN_ID_c     {0x00,0x00,0x00,0x00,\
                                           0x00,0x00,0x00,0x00}

/*******************************************************************************
 g_CHANNEL_MASK_c
 - This macro defines a 32-bit mask starting from the 11th bit from
 the LSB to  the 26th bit, consisting of 16 channels represented by
 each bit.
******************************************************************************/
#define g_MASK_FOR_11_CHANNEL_c  0x00000800
#define g_MASK_FOR_12_CHANNEL_c  0x00001000
#define g_MASK_FOR_13_CHANNEL_c  0x00002000
#define g_MASK_FOR_14_CHANNEL_c  0x00004000
#define g_MASK_FOR_15_CHANNEL_c  0x00008000
#define g_MASK_FOR_16_CHANNEL_c  0x00010000
#define g_MASK_FOR_17_CHANNEL_c  0x00020000
#define g_MASK_FOR_18_CHANNEL_c  0x00040000
#define g_MASK_FOR_19_CHANNEL_c  0x00080000
#define g_MASK_FOR_20_CHANNEL_c  0x00100000
#define g_MASK_FOR_21_CHANNEL_c  0x00200000
#define g_MASK_FOR_22_CHANNEL_c  0x00400000
#define g_MASK_FOR_23_CHANNEL_c  0x00800000
#define g_MASK_FOR_24_CHANNEL_c  0x01000000
#define g_MASK_FOR_25_CHANNEL_c  0x02000000
#define g_MASK_FOR_26_CHANNEL_c  0x04000000

#define g_CHANNEL_MASK_c         g_MASK_FOR_26_CHANNEL_c

/*******************************************************************************
 Power save control
*******************************************************************************/
#define	ENABLE                    1
#define	DISABLE                   0
#define PWR_MODE                  DISABLE

#define LP_MOD		                1
#define	ULP_MOD		                2
#define SLP_TYPE                  ULP_MOD


#define DEEPSLEEP_WKP_PERIOD     4
/*******************************************************************************
 User Poll rate: if Automatic poll is disabled , user configureable poll rate in
                 Seconds
*******************************************************************************/
#define USER_POLL_RATE            3
/*******************************************************************************
 g_STARTUP_CONTROL_c
 - This macro defines how certain parameters are used. The values
 are as follows:

 0x00 - Indicates that the device consideres itself as a part of
 the network indicated by the extended PAN ID attribute.
 In this case device does  not perform any explicit join
 or rejoin operation

 0x01 - Indicates that the device forms a network with extended
 PAN ID given by the extended PAN ID attribute. The AIB's
 attribute APS Designated Coordinator is set to TRUE in
 this case

 0x02 - Indicates that the device rejoins the network with
 extended PAN ID given by the extended PAN ID attribute

 0x03 - Indicates that the device  starts "from scratch" and join
 the network using association

 The default value for an un-commissioned device is 0x03.
******************************************************************************/
#define g_STARTUP_CONTROL_c                0x03

/*******************************************************************************
 g_TRUST_CENTER_ADDRESS_c
 - This macro defines the 64-bit IEEE address of the TC.
******************************************************************************/
#define g_TRUST_CENTER_ADDRESS_c      {0x00,0x00,0x00,0x00,\
                                           0x00,0x00,0x00,0x00}

/*******************************************************************************
 g_NETWORK_KEY_c
 - This macro defines the 16-byte Network Key used for security in
 the network.
******************************************************************************/
#define g_NETWORK_KEY_c              {0x11,0x22,0x33,0x44,\
                                         0x55,0x66,0x77,0x88,\
                                         0x99,0x00,0xaa,0xbb,\
                                         0xcc,0xdd,0xee,0xFF}

/*******************************************************************************
 g_USE_INSECURE_JOIN_c
 - This macro provides information on whether the device is using
 insecure join when the status is true and not using insecure join
 when status is false.
******************************************************************************/
#define g_USE_INSECURE_JOIN_c              0x01

/*******************************************************************************
 g_NETWORK_MANAGER_ADDRESS_c

 - This macro defines the network address of the network manager.
******************************************************************************/
#define g_NETWORK_MANAGER_ADDRESS_c          0x0000

/*******************************************************************************
 g_SCAN_ATTEMPTS_c
 - This macro defines the number of scan attempts that is performed
 by the device after issuing the Join Request.
******************************************************************************/
#define g_SCAN_ATTEMPTS_c                  0x03

/*******************************************************************************
 g_TIME_BETWEEN_SCANS_c
 - This macro holds the value of the time intervals in milliseconds between each
 scan retry.
******************************************************************************/
#define g_TIME_BETWEEN_SCANS_c             0x0032

/*******************************************************************************
 g_REJOIN_INTERVAL_c
 - This macro defines the rejoin interval in seconds. This is
 converted to milliseconds before the timer module is called. i.e.
 this value has to be multiplied by 0x1F4 or 500.
******************************************************************************/
#define g_REJOIN_INTERVAL_c                0x01F4

/*******************************************************************************
 g_MAX_REJOIN_INTERVAL_c
 - This macro defines the maximum rejoin interval in seconds. This
 is converted to milliseconds before the timer module is called.i.e.
 this value has to be multiplied by 0x9C4 or 2500.
******************************************************************************/
#define g_MAX_REJOIN_INTERVAL_c            0x09C4

/*******************************************************************************
 g_POLL_RATE_c
 - This macro defines the maximum time within which an end device
 needs to poll from its parent. It is 7.5 seconds as per the ZigBee
 stack profile. Here it is defined as 7500 milli seconds or 0x1D4C
 in hex.
******************************************************************************/
#define g_POLL_RATE_c                      0x07D0

/*******************************************************************************
 g_PARENT_RETRY_THRESHOLD_c
 - This macro defines how many times the end device needs to attempt
 to contact its parent before initiating Rejoin process.
******************************************************************************/
#define g_PARENT_RETRY_THRESHOLD_c          0x03

/*******************************************************************************
 g_PANID_c
 - This macro defines the 16-byte MAC PANID to be used
******************************************************************************/
#define g_PANID_c                           0xFFFF

/*******************************************************************************
 g_TC_MASTER_KEY_c
 - This macro defines the 16-byte Pre configured master Key used for security in
 the APS layer
******************************************************************************/
#define g_TC_MASTER_KEY_c                  {0x11,0x22,0x33,0x44,\
                                           0x55,0x66,0x77,0x88,\
                                           0x99,0x00,0xaa,0xbb,\
                                           0xcc,0xdd,0xee,0xff}

/*******************************************************************************
 g_PRECONFG_LINK_KEY_c
 - This macro defines the 16-byte Pre configured Link Key used for security in
 the APS {ayer.
******************************************************************************/
#define g_PRECONFG_LINK_KEY_c              {0x5A,0x69,0x67,0x42,\
                                           0x65,0x65,0x41,0x6C,\
                                           0x6C,0x69,0x61,0x6E,\
                                           0x63,0x65,0x30,0x39}

/*******************************************************************************
  g_END_DEVICE_TIMEOUT_c
  - This time in seconds after inititaing END Device Bind Request.
******************************************************************************/
#define g_END_DEVICE_TIMEOUT_c                          0x0a

/******************************************************************************/
/* Default values of the ZDO Configuration Table */
/******************************************************************************/
/*******************************************************************************
 g_PERMIT_JOIN_DURATION_c
 - This macro defines the time for which a coordinator or router
 device allows other devices to join to itself. The values are:
 0x00           - Indicates that no devices can join

 0xFF           - Indicates that devices are always allowed to
 join

 0x01 - 0xFE    -  Indicates the time in seconds for which the
 device allows other devices to join
******************************************************************************/
#define g_PERMIT_JOIN_DURATION_c                   0x00

/*******************************************************************************
 g_NWK_SECURE_ALL_FRAMES_c
 - This macro defines whether security is applied for incoming and
 outgoing network data frames or not.
******************************************************************************/
#define g_NWK_SECURE_ALL_FRAMES_c                  0x01

/*******************************************************************************
 g_FORMATION_ATTEMPTS_c
 - This macro defines the number of times the devices attempts for
 formation failure.
******************************************************************************/
#define g_FORMATION_ATTEMPTS_c                     0x01

/*******************************************************************************
 g_SCAN_DURATION_c
 - Its the duration for which a device will scan for beacons.
******************************************************************************/
#define g_SCAN_DURATION_c                          0x05

/*******************************************************************************
 g_JOIN_ATTEMPTS_c
 - This macro defines the number of times the devices attempts to
 join when join fails.
******************************************************************************/
#define g_JOIN_ATTEMPTS_c                          0x02

/*******************************************************************************
 g_PRECONFIGURED_KEY_c
 - This macro must be set to 0x01 if supporting only preconfigured nwk key,
or else to be set with 0x02 if we hight security (Application link key and nwk
key to be supported)
- value  = 0x01
- value  = 0x02
******************************************************************************/
#define g_PRECONFIGURED_KEY_c                      0x02

/*******************************************************************************
 g_TRUST_CENTER_SHORT_ADDRESS_c
 - This macro defines the short address of the TC.
******************************************************************************/
#define g_TRUST_CENTER_SHORT_ADDRESS_c             0x0000

/*******************************************************************************
 g_AUTOMATIC_POLL_ALLOWED_c
 - This macro provides information on whether automatic poll is
 allowed or not. It's better not to allow this for ZigBee - WiFi Coexistence
 
 0 - Not allowed
 1 - Allowed
******************************************************************************/
#define g_AUTOMATIC_POLL_ALLOWED_c                 0x00

/*******************************************************************************
 g_AUTHENTICATION_POLL_RATE_c
 -This macro defines the poll rate of end device while waiting for
 authentication.
******************************************************************************/
#define g_AUTHENTICATION_POLL_RATE_c                0x64    /* 100 ms */

/*******************************************************************************
 g_SWITCH_KEY_TIMEOUT_c
 -This macro defines the the time after which active key sequence
 number is changed, once the device receives Switch Key request.
******************************************************************************/
#define g_SWITCH_KEY_TIMEOUT_c                     0x06

/*******************************************************************************
 g_NWK_SECURITY_LEVEL_c
 - This macro defines the security level for outgoing and incoming network
 frames.
 *****************************************************************************/
#define g_NWK_SECURITY_LEVEL_c                      0x05


/*******************************************************************************
 g_APS_ACK_POLL_TIME_OUT_c
 -Poll rate of end device while waiting for authentication
******************************************************************************/
#define g_APS_ACK_POLL_TIME_OUT_c                 0xFA    /* 250 ms */

/*-----------------------------------------------------------------------------
 The below section gives the default values of node descriptor
 -----------------------------------------------------------------------------*/
/*Manufacturer code*/
#define g_MANUFACTURER_CODE_c		{0x00,0x00}

/* Default Radius */
#define DEFAULT_RADIUS      0x1e


/* Application Specific declarations */
/* Max Scan attempts Used by Application */
#define MAX_SCAN_ATTEMPTS       5

extern Startup_Attribute_Set_t Startup_Attribute_Set_Default;
extern Simple_Descriptor_t SimpleDesc;
extern ZDO_Configuration_Table_t g_Table_Default;

#endif

