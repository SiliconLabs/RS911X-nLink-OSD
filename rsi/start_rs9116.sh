#!/bin/bash
if [ "$1" == "STA" ] || [ "$1" == "AP" ] || [ "$1" == "STA_BT" ] || [ "$1" == "STA_BLE" ] || [ "$1" == "STA_BT_BLE" ] || [ "$1" == "AP_BT" ] || [ "$1" == "AP_BLE" ] || [ "$1" == "AP_BT_BLE" ] || [ "$1" == "AP_STA" ] || [ "$1" == "BT" ] || [ "$1" == "BLE" ]
then
	echo "Mode Selected: $1"
	string=`echo $PATH`
	IFS=':' read -r -a array <<< "$string"
	supplicant=""
	host_apd=""
	if [ "$1" == "STA" ] || [ "$1" == "STA_BT" ] || [ "$1" == "STA_BLE" ] || [ "$1" == "STA_BT_BLE" ] || [ "$1" == "AP_STA" ]
	then 
		dflag=0
		for element in "${array[@]}"
		do
     			str=`ls $element/ | grep wpa_supplicant`
     			if [ "$str" = '' ] 
     			then
     				dflag=0
     			else
     				dflag=1
     			fi
     			if [ $dflag == 1 ]
     			then
     				break
     			fi
		done
		pwd_name=`pwd`
		cflag=0
		st=`ls $pwd_name/ | grep wpa_supplicant`
		if [ "$st" = '' ]
		then
			cflag=0
		else
			cflag=1
		fi	
		if [ $dflag == 0 ] && [ $cflag == 0 ]
		then
			echo "Please ensure wpa_supplicant application is present in system"
			exit 0
		elif [ $dflag == 0 ] && [ $cflag == 1 ]
		then
			supplicant="./wpa_supplicant"
		elif [ $dflag == 1 ] && [ $cflag == 0 ]
		then
			supplicant="wpa_supplicant"
		elif [ $dflag == 1 ] && [ $cflag == 1 ]
		then
			supplicant="./wpa_supplicant"
		fi
	fi
	if [ "$1" == "AP" ] || [ "$1" == "AP_BT" ] || [ "$1" == "AP_BLE" ] || [ "$1" == "AP_BT_BLE" ] || [ "$1" == "AP_STA" ]
	then
		dflag=0
		for element in "${array[@]}"
		do
     			str=`ls $element/ | grep hostapd`
     			if [ "$str" = '' ] 
     			then
     				dflag=0
     			else
     				dflag=1
     			fi
     			if [ $dflag == 1 ]
     			then
     				break
     			fi
		done
		pwd_name=`pwd`
		cflag=0
		st=`ls $pwd_name/ | grep hostapd`
		if [ "$st" = '' ]
		then
			cflag=0
		else
			cflag=1
		fi	
		if [ $dflag == 0 ] && [ $cflag == 0 ]
		then
			echo "Please ensure hostapd application is present in system"
			exit 0
		elif [ $dflag == 0 ] && [ $cflag == 1 ]
		then
			host_apd="./hostapd"
		elif [ $dflag == 1 ] && [ $cflag == 0 ]
		then
			host_apd="hostapd"
		elif [ $dflag == 1 ] && [ $cflag == 1 ]
		then
			host_apd="./hostapd"
		fi
	fi
	sdio_ko=`lsmod | grep rsi_sdio`
	usb_ko=`lsmod | grep rsi_usb`
	rsi_ko=`lsmod | grep rsi_91x`
	v=0
	if [ "$usb_ko" == "" ]
	then
    		v=1
	else
    		echo "Unloading the existing rsi_usb"
		rmmod rsi_usb.ko
	fi
	if [ "$sdio_ko" == "" ]
	then
		v=1
	else
    		echo "Unloading the existing rsi_sdio"
		rmmod rsi_sdio.ko
	fi
	if [ "$rsi_ko" == "" ]
	then
    		v=1
	else
    		echo "Unloading the existing rsi_91x"
		rmmod rsi_91x.ko
	fi
	bti_count=0
	sleep 2
	wnp=`iw dev | grep Interface`
	wnpl=(${wnp//"Interface"/ })
	len=${#wnpl[@]}
	
	if [ "$1" == "STA" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=1/' osd_common_insert.sh
	elif [ "$1" == "AP" ]
	then	
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=1/' osd_common_insert.sh
	elif [ "$1" == "AP_STA" ] 
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=1/' osd_common_insert.sh
	elif [ "$1" == "BT" ]
	then	
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=4/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "BLE" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=8/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "STA_BT" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=5/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "STA_BLE" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=9/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "AP_BLE" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=10/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "AP_BT" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=6/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "STA_BT_BLE" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=13/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	elif [ "$1" == "AP_BT_BLE" ]
	then
		sed -i 's/^DEV_OPER_MODE.*=.*/DEV_OPER_MODE=14/' osd_common_insert.sh
		bti=`hciconfig | grep hci | cut -d ":" -f 1`
		declare -a btil
		btil=($bt)
		bti_count=${#btil[@]}
	fi

	
	var=`./osd_common_insert.sh`
	sleep 2
	
	if [ "$var" = 0 ]
	then
		echo "Error!!! No Module Connected"
		exit 0
	else
		echo "*********************************** Driver Loaded ***********************************"
		
		rfkill unblock all
	
		sleep 1

		service network-manager stop
	

		wn=`iw dev | grep Interface`
		wnl=(${wn//"Interface"/ })
		len=${#wnl[@]}

		tp_wval=''

		l1=${#wnpl[@]}
		l2=${#wnl[@]}
		flag=1
	


		if [ $l1 == 0 ]
		then
			tp_wval=${wnl[0]}
		else
			for (( i=0; i<$l2; i++))
			do
				for (( j=0; j<$l1; j++))
				do
					if [ ${wnl[$i]} != ${wnpl[$j]} ]
					then
    	    			flag=0
					else
    	    			flag=1
					fi
				done
				if [ $flag == 0 ]
				then
					tp_wval=${wnl[$i]}
				break
				fi
			done
		fi
		
		killall wpa_supplicant
		sleep 2
		killall hostapd
		sleep 2 
		
		
		if [ "$1" == "STA" ] || [ "$1" == "STA_BT" ] || [ "$1" == "STA_BLE" ] || [ "$1" == "STA_BT_BLE" ]
		then
			$supplicant -i $tp_wval -D nl80211 -c ../scripts/sta_settings.conf -dddt>log &
			sleep 2
			dhclient -r $tp_wval
			dhclient -v $tp_wval
			echo "Station with name : "$tp_wval " is up."
			bti=`hciconfig | grep hci | cut -d ":" -f 1`
			declare -a btil
			btil=($bt)
			tbti_count=${#btil[@]}
			if [ $tbti_count > $bti_count ] && [ "$1" == "STA_BT" ]
			then
				echo "BT-Classic Protocol Enabled"
			elif [ $tbti_count > $bti_count ] && [ "$1" == "STA_BLE" ]
			then
				echo "BLE-Classic Protocol Enabled"
			elif [ $tbti_count > $bti_count ] && [ "$1" == "STA_BT_BLE" ]
			then
				echo "BT-Classic Protocol Enabled"
				echo "BLE-Classic Protocol Enabled"
			elif [  "$1" == "STA" ]
			then
				echo "Success"
			else
				echo "Could  Not Start the BT/BLE Prtocol"
			fi

		elif [ "$1" == "AP" ] || [ "$1" == "AP_BT" ] || [ "$1" == "AP_BLE" ] || [ "$1" == "AP_BT_BLE" ]
		then	
			while [ 1 ];
			do	
				read -p "In what security you want to start the AP.For open-security mode 1 OR for secured mode type 2 : " ap_val	
				if [ $ap_val == '1' ]
				then
					echo "$(tail -n +2 ../scripts/ap_open.conf)" > ../scripts/ap_open.conf
					echo -e "interface=$tp_wval\n$(cat ../scripts/ap_open.conf)" > ../scripts/ap_open.conf
					$host_apd ../scripts/ap_open.conf -ddddt > log&
					break
				elif [ $ap_val == '2' ]
				then
					echo "$(tail -n +2 ../scripts/ap_wpa.conf)" > ../scripts/ap_wpa.conf
					echo -e "interface=$tp_wval\n$(cat ../scripts/ap_wpa.conf)" > ../scripts/ap_wpa.conf
					$host_apd ../scripts/ap_wpa.conf -ddddt > log&
					break
				else
					echo "Please!!!! Enter a correct option"
					continue
				fi
			done
			sleep 2
			echo "AP VAP Created"
			
			bti=`hciconfig | grep hci | cut -d ":" -f 1`
			declare -a btil
			btil=($bt)
			tbti_count=${#btil[@]}
			if [ $tbti_count > $bti_count ] && [ "$1" == "AP_BT" ]
			then
				echo "BT-Classic Protocol Enabled"
			elif [ $tbti_count > $bti_count ] && [ "$1" == "AP_BLE" ]
			then
				echo "BLE-Classic Protocol Enabled"
			elif [ $tbti_count > $bti_count ] && [ "$1" == "AP_BT_BLE" ]
			then
				echo "BT-Classic Protocol Enabled"
				echo "BLE-Classic Protocol Enabled"
			elif [ "$1" == "AP" ]
			then
				echo "Success"
			else
				echo "Could  Not Start the BT/BLE Prtocol"
			fi


		elif [ "$1" == "AP_STA" ] 
		then
			$supplicant -i $tp_wval -D nl80211 -c ../scripts/sta_settings.conf -dddt>log &
			sleep 2
			dhclient -r $tp_wval
			dhclient -v $tp_wval
			echo "Station with name : "$tp_wval " is up."
			while [ 1 ];
			do	
				read -p "In what security you want to start the AP.For open-security mode 1 OR for secured mode type 2: " ap_val	
				if [ $ap_val == '1' ]
				then
					echo "$(tail -n +2 ../scripts/ap_open.conf)" > ../scripts/ap_open.conf
					echo -e "interface=$tp_wval\n$(cat ../scripts/ap_open.conf)" > ../scripts/ap_open.conf
					$host_apd ../scripts/ap_open.conf -ddddt > log&
					break
				elif [ $ap_val == '2' ]
				then
					echo "$(tail -n +2 ../scripts/ap_wpa.conf)" > ../scripts/ap_wpa.conf
					echo -e "interface=$tp_wval\n$(cat ../scripts/ap_wpa.conf)" > ../scripts/ap_wpa.conf
					$host_apd ../scripts/ap_wpa.conf -ddddt > log&
					break
				else
					echo "Please!!!! Enter a correct option"
					continue

				fi
			done
			sleep 2
			echo "AP VAP Created"
			echo "Success"
		elif [ "$1" == "BT" ]
		then	
			bti=`hciconfig | grep hci | cut -d ":" -f 1`
			declare -a btil
			btil=($bt)
			tbti_count=${#btil[@]}
			if [ $tbti_count > $bti_count ]
			then
				echo "BT-Classic Protocol Enabled"
			else
				echo "Could  Not Start the BT/BLE Prtocol"
			fi
		
		elif [ "$1" == "BLE" ]
		then
			bti=`hciconfig | grep hci | cut -d ":" -f 1`
			declare -a btil
			btil=($bt)
			tbti_count=${#btil[@]}
			if [ $tbti_count > $bti_count ]
			then
				echo "BLE Protocol Enabled"
			else
				echo "Could  Not Start the BT/BLE Prtocol"
			fi
		fi
	fi

	else
		echo "Wrong Arguments! Please specify a input for mode"
		echo "Available Options are : "
		echo "AP"	
		echo "STA"
		echo "AP_STA"
		echo "BT"
		echo "BLE"
		echo "STA_BT"
		echo "AP_BT"
		echo "STA_BLE"
		echo "AP_BLE"
		echo "STA_BT_BLE"
		echo "AP_BT_BLE"
fi

