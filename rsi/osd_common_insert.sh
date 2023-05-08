
#!/bin/bash/

cat /dev/null > /var/log/messages


#Driver Mode 1 END-TO-END mode, 
#            2 RF Evaluation Mode

DRIVER_MODE=1

# DEV_OPER_MODE:                  
#							1    WLAN STATION /WIFI-Direct/WLAN PER/WLAN ACCESS POINT + STATION MODE

#							4    BT CLASSIC MODE/BT CLASSIC PER MODE
#							5    WLAN STATION + BT CLASSIC MODE
#							6    WLAN ACCESS POINT + BT CLASSIC MODE
#							8    BT LE MODE /BT LE PER MODE
#							9    WLAN STATION + BT LE MODE
#                                                       10   WLAN_AP + BT LE MODE
#							12   BT CLASSIC + BT LE MODE 							
#							13   WLAN STATION + BT CLASSIC MODE + BT LE MODE
#							14   WLAN ACCESS POINT + BT CLASSIC MODE+ BT LE MODE

DEV_OPER_MODE=1



#Power Save Feature

#U-APSD Power Save                                  	
#0 - Disable U-APSD mode							
#1 - Enable U-APSD mode
ENABLED_UAPSD=0

#Rx data inactive interval for enhanced max psp
RX_DATA_INACTIVE_INTERVAL=0

#Max Service Period length
#0-All buffered packets will be delivered(Default).
#1-Two buffered packets will be delivered.
#2-Four buffered packets will be delivered.
#3-Six buffered packets will be delivered.
MAX_SP_LEN=0 				

#0 - No handshake Mode (Default)
#1 - GPIO Handshake Mode
LP_HANDSHAKE_MODE=0			

#BT Related Module Params

#0 - EXTERNAL RF
#1 - INTERNAL RF (Default)
BT_RF_TYPE=1
				
#Range for the BLE Tx LP Chain Power, Index is 1 - 63 (0, 32 are invalid)
#Range for the BLE Tx HP Chain Power, Index is 64 - 76
#Default value = 30
BLE_TX_PWR_INX=30                              	

#BLE_DUTY_CYCLING   BIT(0)
#BLR_DUTY_CYCLING   BIT(1)
#BLE_PWR_SAVE_4X_MODE BIT(2)
BLE_POWER_SAVE_OPTIONS=2

#Ble Roles                       
#BIT[3:0] - Number of Central Roles Allowed
#BIT[7:4] - Number of Peripheral Roles Allowed				 							 
BLE_ROLES=19						

#BDR mode in classic
#BIT(0) - BDR only selection
#BIT(1) - BDR in LP chain selection
BT_EDR_MODE=0

BT_RF_TX_POWER_MODE=0

BT_RF_RX_POWER_MODE=0

#BIT[0] - For Enabling role switch from host set this bit to 1
#BIT[1] - For Enabling sniff from host set this bit to 1
#BIT[5] - For Enabling BT Spoof MAC address i.e to use HARDCODE_MAC_ADDR in BT this bit should be set to #1
#BIT[7:3] - Reserved
BT_FEATURE_BITMAP=0 					

#Miscellaneous Features

#SDIO Clock speed
#SDIO_CLOCK_SPEED=25
SDIO_CLOCK=50

#In GPIO handshake it is used to configure Host GPIO read pin. It will vary form platform to platform.Default value is 0xFF.
ULP_GPIO_READ=0xFFF					

#In GPIO handshake it is use to configure Host GPIO write pin. It will vary form platform to platform.Default value is 0xFF
ULP_GPIO_WRITE=0xFFF					

#Developer Mode Configuration Parameters

#Power Save options
#0 - Disable Duty Cycling & Undestined Packet Drop
#1 - Enable Duty Cycling
#2 - Enable Undestined Packet Drop
#3 - Enable Duty Cycling & Undestined Packet Drop (Default)
POWER_SAVE_OPT=3

#LP/HP Chain Selection in standby associated mode
#0 - HP Chain Enabled
#1 - LP Chain Enabled(Default)												
STANDBY_ASSOC_CHAIN_SEL=1      			

#9116 Feature Bitmap						
#BIT(0): Disable auto rate enhancements
#BIT(1): 1.8V enable
#BIT(2): Reserved
#BIT(3): Airplane mode				
FEATURE_BITMAP_9116=0										

#LMAC BEACON DROP Feature Options
#0 - Disable LMAC BEACON DROP Feature
#1 - Enable LMAC BEACON DROP Feature
LMAC_BCON_DROP=1   					

ANCHOR_POINT_GAP=1

#Sleep clock selection
#0 - Use RC clock as sleep clock
#1 - Use 32KHz clock from external XTAL OSCILLATOR
#2 - Use 32KHz bypass clock on UULP_GPIO_3
#3 - Use 32KHz bypass clock on UULP_GPIO_4
#Default value = 0
SLEEP_CLK_SOURCE_SEL=0      											

#sleep indication from device to host.
#0 - UULP_GPIO_3
#1 - UULP_GPIO_0
#Default value is 0.
SLEEP_IND_GPIO_SEL=0					
							
#Host Interface on Demand Feature has the following possible values.
#0 - Disable Host Interface on Demand Feature
#1 - Enable Host Interface on Demand Feature
#Default value for Host Interface on Demand Feature Options is 0, which indicates that Host Interface on Demand Feature is disabled.					
HOST_INTF_ON_DEMAND=0					

#Extended Options.0 - NA.Default value for Extended options is 0.
EXT_OPT=0

#Driver Logs Enable.
#BIT(0) - ERROR ZONE
#BIT(1) - INFO ZONE,  BIT(2) - INIT ZONE, BIT(3) - MGMT TX ZONE
#BIT(4) - MGMT RX ZONE, BIT(5) - DATA TX ZONE, BIT(6) - DATA RX ZONE
#BIT(7) - FSM ZONE, BIT(8) - ISR ZONE, BIT(9) - INT_MGMT_ZONE
#BIT(10) - MGMT_DEBUG_ZONE
RSI_ZONE_ENABLED=0x1

PARAMS=" driver_mode_value=$DRIVER_MODE"
PARAMS=$PARAMS" dev_oper_mode=$DEV_OPER_MODE"
PARAMS=$PARAMS" rsi_zone_enabled=$RSI_ZONE_ENABLED"
: '

#OSD Module Params
PARAMS=$PARAMS" host_intf_on_demand=$HOST_INTF_ON_DEMAND"
PARAMS=$PARAMS" sleep_ind_gpio_sel=$SLEEP_IND_GPIO_SEL"
PARAMS=$PARAMS" anchor_point_gap=$ANCHOR_POINT_GAP"
PARAMS=$PARAMS" lmac_bcon_drop=$LMAC_BCON_DROP"
PARAMS=$PARAMS" feature_bitmap_9116=$FEATURE_BITMAP_9116"
PARAMS=$PARAMS" standby_assoc_chain_sel=$STANDBY_ASSOC_CHAIN_SEL"
PARAMS=$PARAMS" power_save_opt=$POWER_SAVE_OPT"
PARAMS=$PARAMS" ulp_gpio_write=$ULP_GPIO_WRITE"
PARAMS=$PARAMS" ulp_gpio_read=$ULP_GPIO_READ"
PARAMS=$PARAMS" bt_feature_bitmap=$BT_FEATURE_BITMAP"
PARAMS=$PARAMS" bt_rf_rx_power_mode=$BT_RF_RX_POWER_MODE"
PARAMS=$PARAMS" bt_rf_tx_power_mode=$BT_RF_TX_POWER_MODE"
PARAMS=$PARAMS" bt_edr_mode=$BT_EDR_MODE"
PARAMS=$PARAMS" ble_roles=$BLE_ROLES"
PARAMS=$PARAMS" ble_power_save_options=$BLE_POWER_SAVE_OPTIONS"
PARAMS=$PARAMS" ble_tx_pwr_inx=$BLE_TX_PWR_INX"
PARAMS=$PARAMS" bt_rf_type=$BT_RF_TYPE"
PARAMS=$PARAMS" lp_handshake_mode=$LP_HANDSHAKE_MODE"
PARAMS=$PARAMS" max_sp_len=$MAX_SP_LEN"
PARAMS=$PARAMS" rx_data_inactive_interval=$RX_DATA_INACTIVE_INTERVAL"
PARAMS=$PARAMS" enabled_uapsd=$ENABLED_UAPSD"
'

modprobe mac80211
modprobe bluetooth

flag=0

usb=`lsusb | grep 9116 | cut -d ":" -f 3`
mod_name=${usb:0:4}
if [ "$mod_name" = 9116 ]
then
	flag=1
fi

sdio=`ls /sys/bus/sdio/devices | cut -d " " -f 2`
declare -a sdio_interfcaes
sdio_interfaces=($sdio)
len=${#sdio_interfaces[@]}

t_vid='xxxxxx'
for(( i=0; i<$len; i++))
do
  sdio_name=${sdio_interfaces[$i]}
  vendor_id=`cat /sys/bus/sdio/devices/$sdio_name/vendor`
  if [ $vendor_id == "0x041b" ]
  then
	  t_vid=$vendor_id
	  flag=2
	  break
  fi
done

if [ $flag  == 1 ]
then
	yes | cp ../Firmware/* /lib/firmware/
	insmod rsi_91x.ko $PARAMS
	insmod rsi_usb.ko
	sleep 2
	
elif [ $flag == 2 ]
then
	yes | cp ../Firmware/* /lib/firmware/
	insmod rsi_91x.ko $PARAMS
	insmod rsi_sdio.ko sdio_clock=50
	sleep 2
else
	echo "0"
fi

sleep 1
