FILE=/etc/modprobe.d/rasp_denylist.conf
if [ -f "$FILE" ]; then
	echo "$FILE exist."
else
	echo "File  does not exist created a new $FILE"
	echo "Reboot the platform"
	echo "blacklist brcmfmac" > $FILE
	echo "blacklist brcmutil" >> $FILE

fi
