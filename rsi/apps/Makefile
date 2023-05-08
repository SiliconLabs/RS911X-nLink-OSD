cpy=cp -rf


all: 
	@echo -e "\033[32mCompiling Application...\033[0m"
	$(CC) -o receive per_receive.c per_sock.c $(CFLAGS)
	$(CC) -o bt_util bt_util.c per_sock.c $(CFLAGS)
	$(CC) -o onebox_util update_wlan_gain_table.c onebox_util.c per_sock.c $(CFLAGS)
	$(CC) -o bt_ble_gain_table_update update_bt_ble_gain_table.c per_sock.c $(CFLAGS)
	$(CC) -o transmit transmit.c per_sock.c $(CFLAGS)
	$(CC) -o transmit_packet transmit_packet.c per_sock.c $(CFLAGS)

	@echo -e "Copying to release folder...."
	$(cpy) receive ../release/
	$(cpy) bt_util ../release/
	$(cpy) onebox_util ../release/
	$(cpy) transmit ../release/
	$(cpy) transmit_packet ../release/
	$(cpy) bt_ble_gain_table_update ../release/
