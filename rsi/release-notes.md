# RS9116 n-Link&trade; Linux OSD Release Notes

Release notes are provided with each version of [RS9116 n-Link Linux OSD drivers](https://github.com/SiliconLabs/RS911X-nLink-OSD).

## Versions
  - [n-Link OSD 2.6.1](#nlink-osd-2-6-1-release-notes)  
  - [n-Link OSD 2.5.1](#nlink-osd-2-5-1-release-notes)
  - [n-Link OSD 2.3.2](#nlink-osd-2-3-2-release-notes)

## Reference Documents
  - [RS9116 n-Link OSD Reference Manual](https://www.silabs.com/documents/login/reference-manuals/rs9116n-open-source-driver-technical-reference-manual.pdf) 
  - [RS9116N Users Guide](https://www.silabs.com/documents/login/user-guides/ug452-rs9116n-evk-software-users-guide.pdf)

# n-Link OSD 2.6.1 Release Notes 

Last updated: May 8, 2023

## Index
  - [Highlights](#highlights)
  - [Release Details](#release-details)
  - [New Features](#new-features)
  - [Changes and Issues Fixed](#changes-and-fixes)
  - [Known Issues](#known-issues)
  - [Limitations and Unsupported Features](#limitations-and-unsupported-features)
  - [Terminology](#terminology)

## Highlights
  - PTA 3-Wire Co-Existence Support in Wi-Fi STA alone mode.
  - STA + AP Concurrent Mode.
  - Fix for SDIO Buffer flow management issues during high data transfers.
  - Added configurable option to enable/disable power save before Wi-Fi connection.
  - Added support for RS916AC0 and RS916AC1 modules.
  - Added script to load/unload the driver with different operational modes.
  - Fix for handling EAPOL packets with wpa_supplicant version 2.10 and above.
  - Added gain tables and configurable options for all certified antennas.
  - Fix for AFH channel mapping issue during role switch.
  - Fix for BT Packet type selection during BT PER Transmit tests.
  - Fix for BLE Tx power issue.
  - Fix for BLE request remote version issue.
  - Added a fix while connecting to a remote peer device using LE privacy.
  - Fix for BT/BLE interface bring up issue on few embedded platforms.
  - Improved BT/BLE connection stability and throughput.

## Release Details
| Item                         | Details                                	|
|------------------------------|------------------------------------------------|
| Release date                 | 8th May, 2023                   		|
| Firmware Version             | 1610.2.6.1.0.6                       	 	|
| Package Name                 | RS9116.NB0.NL.GNU.LNX.OSD.2.6.1.6     		|
| Hardware Modules             | QMS, B00, C00, CC1, AB0, AB1, AA0, AA1,AC0, AC1|
| Hardware Chipsets            | Chip Revision 1.3, 1.4, 1.5            	|
| Linux Kernel Version support | From v3.18 to v5.15                    	|
| Host interfaces supported    | USB, SDIO                              	|

## New Features

### Wi-Fi
  - Added PTA 3-Wire Co-Existence support in Wi-Fi STA alone mode.
  - STA + AP Concurrent Mode.
  - Added support for RS916AC0 and RS916AC1 modules.
  - Added script to load/unload the driver with different operational modes.

### Bluetooth - Common
  - None

### Bluetooth – Classic
  - None

### Bluetooth – LE
  - None

## Changes and Fixes

### Wi-Fi
  - Added a fix for handling EAPOL packets with wpa_supplicant version 2.10 and above.
  - Added gain tables and configurable options for all certified antennas.
  - Added a fix for SDIO Buffer flow management issues during high data transfers.
  - Added configurable option to enable/disable power save before Wi-Fi connection.

### Bluetooth – Common
  - Added a fix for BT/BLE interface bring up issue on few embedded platforms.
  - Improved BT/BLE connection stability and throughput.

### Bluetooth – Classic
  - Added a fix for AFH channel mapping issue during role switch.
  - Added a fix for BT Packet type selection during BT PER Transmit tests.

### Bluetooth – LE
  - Added a fix for BLE Tx power issue.
  - Added a fix for BLE request remote version issue.
  - Added a fix while connecting to a remote peer device using LE privacy.

## Known Issues
  - BT-HID might not inter-operate with Apple devices.
  - In Wi-Fi + BT/BLE co-ex mode, high Wi-Fi broadcast traffic, might cause BT/BLE disconnections.
  - Set rate can not be used for setting non MCS and non basic rates in kernels above 4.13.16.
  - Enterprise security with WPA3 is not supported.
  - Tx Rate information is not being updated in WLAN interfaces stats reported by iwconfig tool.
  - Auto Channel Selection in AP with WORLD region will not work.
  - PTA 3-wire co-existence does not work with powersave. However, if a weak pull up is applied to the GRANT pin, PTA 3-wire functionality will work with powersave.
  - PTA 3-Wire co-existence receive packets will not be protected.
  - In Concurrent Mode, DUT-STA doesn't reconnect to testbed AP if the testbed AP changes the channel(Only if DUT-AP is up), also DUT-AP stays in the same channel and continues to beacon.
  - In Concurrent Mode, Wi-Fi performance may be reduced if more than four clients are connected to DUT-AP.
  - Deep sleep is disable by default. Users can enable it using default_deep_sleep_enable module_param mentioned in the TRM.

## Limitations and Unsupported Features

## Wi-Fi
  - Tx AMSDU is not supported.
  - Fragmentation is not supported.
  - AMSDU's within an AMPDU is not supported.
  - Wi-Fi performance may be reduced in dense environments.
  - 802.11j is not supported.
  - In AP mode, more than 16 clients are not supported; AP + BT + BLE mode supports 4 WLAN clients.
  - USB ULP is not supported.
  - LP powersave using GPIO handshake is not supported.
  - DFS Master is not supported.
  - Radar detection in STA mode is not supported.
  - Background scan(Scan after DUT-STA connection) and powersave features are not supported for the station mode vap in concurrent mode.
  - In concurrent mode, Both DUT AP and Testbed AP should be in same channel. The Channel in hostapd configuration file needs to be updated manually to the same channel as the Testbed AP.

## BT/BLE
  - In BLE, the recommended range for the Connection Interval in
    - Power Save: 100 ms to 1.28 seconds.
    - BT Classic + BLE Dual Mode: >= 200 ms.
    - Wi-Fi + BLE co-ex: 30 ms to 4 seconds
  - In BLE, during connection, using the same value for Scan Interval and Scan Window is not recommended.
  - In BT Classic, the recommended value of Sniff Interval during Power Save is limited to 100 ms (<= 100).
  - In Wi-Fi + BT Classic co-ex, for normal BT file transfer operation, the recommended value of Wi-Fi protocol DTIM is > 3, and that of Beacon Interval is > 200ms.
  - In Wi-Fi + BLE, during Wi-Fi connection, using a lower BLE Scan Window and larger BLE Scan Interval is recommended.
  - In CC0/CC1 dual-band modules,  power consumption during transmission in 2.4 GHz is sometimes higher by up to 30% to 40%
  - Power consumption in BLE connected sleep is higher by ~40%
  - BT Sniff mode does not work if BT multiple slaves feature is enabled.
  - When BT multiple slaves feature is enabled, Master to Slave role switch will not happen.
  - In BLE, if Advertising/Scanning is in progress, and the device switches its Slave/Master role, Advertising/Scanning will stop. The command to start Advertising/Scanning must be given again.
  - Glitches can be observed in BT audio if Wi-Fi is in connected state.
  - In BLE, if device is acting as Master/Slave, Scan Window (in `set_scan_params` and `create_connection command`) shall be less than the existing Connection Interval.
  - In BLE, if BLE Connection is established with small Connection Interval (< 15 ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.
  - BT Classic Simultaneous Slave & Master roles (Scatter-net) is not supported.

## Co-existence
  - In Wi-Fi + BLE, if the BLE Scan Interval and Scan Window are identical, Wi-Fi connection may be unsuccessful.
  - BLE disconnection may be observed with Wi-Fi + BLE configuration with Wi-Fi continuous data transfer for low BLE supervision timeout configured. If the Supervision timeout is configured to >= 16 seconds, no disconnections are observed
  - If BT/BLE is connected first with small connection interval, it may be difficult to achieve a Wi-Fi connection. It is recommended to use a longer connection interval and supervision timeout for BLE or a longer supervision timeout and sniff interval for BT.


# n-Link OSD 2.5.1 Release Notes 

Last updated: March 25, 2022

## Index
  - [Highlights](#highlights)
  - [Release Details](#release-details)
  - [New Features](#new-features)
  - [Changes and Issues Fixed](#changes-and-fixes)
  - [Known Issues](#known-issues)
  - [Limitations and Unsupported Features](#limitations-and-unsupported-features)
  - [Terminology](#terminology)

## Highlights
  - Added support to update gain tables to help with regulatory certification
  - Added fast PSP support in power save profile
  - Fixed transmit power issue in AA0/CC0 Modules
  - Added fixes for Wi-Fi FragAttack vulnerability

## Release Details
| Item                         | Details                                |
|------------------------------|----------------------------------------|
| Release date                 | 25th March, 2022                        |
| Firmware Version             | 1610.2.5.1.0.11                        |
| Package Name                 | RS9116.NB0.NL.GNU.LNX.OSD.2.5.1.11     |
| Hardware Modules             | QMS, B00, C00, CC1, AB0, AB1, AA0, AA1 |
| Hardware Chipsets            | Chip Revision 1.3, 1.4, 1.5            |
| Linux Kernel Version support | From v3.18 to v5.15                    |
| Host interfaces supported    | USB, SDIO                              |

## New Features

### Wi-Fi
  - Provision to update Gain Table for certification
  - Fast PSP support in power save profile

### Bluetooth - Common
  - None

### Bluetooth – Classic
  - None

### Bluetooth – LE
  - None

## Changes and Fixes

### Wi-Fi
  - Fixed transmit power issue in AA0/CC0 Modules.
  - Added minimum supported rate for EAPOL packet transmission.
  - Added a fix for SDIO suspend/resume.
  - Added fixes for Wi-Fi FragAttack vulnerability.
  - RS9116X-DB00-CCX-BXX certified TELEC gain tables included.

### Bluetooth – Common
  - Added a fix to initiate AFH periodically to resolve a BT and Wi-Fi hang issue.
  - Resolved a Wi-Fi ICMP ping issue that used to occur while connecting to BT in co-ex mode in CC0/AA0 modules based on chip version 1.3.

### Bluetooth – Classic
  - None

### Bluetooth – LE
  - None

## Known Issues
  - BT-HID might not inter-operate with Apple devices.
  - In Wi-Fi + BT/BLE co-ex mode, high Wi-Fi broadcast traffic, might cause BT/BLE disconnections.
  - Set rate can not be used for setting non MCS and non basic rates in kernels above 4.13.16.
  - Enterprise security with WPA3 is not supported.
  - Tx Rate information is not being updated in WLAN interfaces stats reported by iwconfig tool.
  - Auto Channel Selection in AP with WORLD region will not work.

## Limitations and Unsupported Features

### Wi-Fi
  - Tx AMSDU is not supported.
  - Fragmentation is not supported.
  - AMSDU's within an AMPDU is not supported.
  - Wi-Fi performance may be reduced in dense environments.
  - 802.11j is not supported.
  - In AP mode, more than 16 clients are not supported; AP + BT + BLE mode supports 4 clients.
  - USB ULP is not supported.
  - External 3-wire co-existence is not supported.
  - LP powersave using GPIO handshake is not supported.
  - DFS Master is not supported.
  - Radar detection in STA mode is not supported.

### BT/BLE
  - In BLE, the recommended range for the Connection Interval in
    - Power Save: 100 ms to 1.28 seconds.
    - BT Classic + BLE Dual Mode: >= 200 ms.
    - Wi-Fi + BLE co-ex: 30 ms to 4 seconds
  - In BLE, during connection, using the same value for Scan Interval and Scan Window is not recommended.
  - In BT Classic, the recommended value of Sniff Interval during Power Save is limited to 100 ms (<= 100).
  - In Wi-Fi + BT Classic co-ex, for normal BT file transfer operation, the recommended value of Wi-Fi protocol DTIM is > 3, and that of Beacon Interval is > 200ms.
  - In Wi-Fi + BLE, during Wi-Fi connection, using a lower BLE Scan Window and larger BLE Scan Interval is recommended.
  - In CC0/CC1 dual-band modules,  power consumption during transmission in 2.4 GHz is sometimes higher by up to 30% to 40%
  - Power consumption in BLE connected sleep is higher by ~40%
  - BT Sniff mode does not work if BT multiple slaves feature is enabled.
  - When BT multiple slaves feature is enabled, Master to Slave role switch will not happen.
  - In BLE, if Advertising/Scanning is in progress, and the device switches its Slave/Master role, Advertising/Scanning will stop. The command to start Advertising/Scanning must be given again.
  - Glitches can be observed in BT audio if Wi-Fi is in connected state.
  - In BLE, if device is acting as Master/Slave, Scan Window (in `set_scan_params` and `create_connection command`) shall be less than the existing Connection Interval.
  - In BLE, if BLE Connection is established with small Connection Interval (< 15 ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.
  - BT Classic Simultaneous Slave & Master roles (Scatter-net) is not supported.

### Co-existence
  - In Wi-Fi + BLE, if the BLE Scan Interval and Scan Window are identical, Wi-Fi connection may be unsuccessful.
  - BLE disconnection may be observed with Wi-Fi + BLE configuration with Wi-Fi continuous data transfer for low BLE supervision timeout configured. If the Supervision timeout is configured to >= 16 seconds, no disconnections are observed
  - If BT/BLE is connected first with small connection interval, it may be difficult to achieve a Wi-Fi connection. It is recommended to use a longer connection interval and supervision timeout for BLE or a longer supervision timeout and sniff interval for BT.


# n-Link OSD 2.3.2 Release Notes 

Last updated: July 5, 2021

## Index
  - [Highlights](#highlights-1)
  - [Release Details](#release-details-1)
  - [New Features](#new-features-1)
  - [Changes and Issues Fixed](#changes-and-fixes-1)
  - [Known Issues](#known-issues-1)
  - [Limitations and Unsupported Features](#limitations-and-unsupported-features-1)
  - [Terminology](#terminology)

## Highlights
- Added Wi-Fi support for EN 300 328 V2.2.2
- Fixed STA disconnection issue in co-ex mode while giving `lescan` command

## Release Details
| Item                                                | Details                                |
|-----------------------------------------------------|----------------------------------------|
| Release date                                        | July 5, 2021                           |
| Firmware Version                                    | 1610.2.3.2.0003                        |
| Package Name                                        | RS9116.NB0.NL.GNU.LNX.OSD.2.3.2.0003   |
| Hardware Modules                                    | QMS, B00, C00, CC1, AB0, AB1, AA0, AA1 |
| Hardware Chipsets                                   | Chip Revision 1.4                      |
| Supported Linux Kernel Versions                     | From v3.18 to v5.7                     |
| Host interfaces supported                           | USB, SDIO                              |


## New Features

### Wi-Fi
- Added Support for EN 300 328 V2.2.2.

### Bluetooth - Common
- None

### Bluetooth – Classic
- None

### Bluetooth – LE
- None


## Changes/Issues Fixed

### Wi-Fi
- None

### Bluetooth – Common
- Fixed STA disconnection issue in co-ex mode while giving `lescan` command.

### Bluetooth – Classic
- None

### Bluetooth – LE
- None

## Known Issues
- BT-HID may not inter-operate with Apple devices
- In Wi-Fi + BT/BLE co-ex mode, high Wi-Fi broadcast traffic may cause BT/BLE disconnections.
- Issues with BT PER continuous mode transmission
- Set rate can not be used for setting non-MCS and non-basic rates in kernels above 4.13.16.
- Enterprise security with WPA3 is not supported.
- Transmit rate update to use (iwconfig) is not supported.
- BT/BLE co-ex modes with WiFi AP are not supported.
- EAP-TLS 1.2 /1.1 are not supported in this release.
- Auto Channel Selection in AP with WORLD region will not work.


## Limitations and Unsupported Features

### Wi-Fi
- AMSDU TX is not supported
- Fragmentation is not supported.
- AMSDU's within AMPDU is not supported.
- Wi-Fi performance is reduced in dense environments.
- Digital Loopback is not supported.
- PUF is not supported.
- 11j not supported.
- 11p is not supported.
- AP mode more than 16 clients not supported, AP + BT + BLE has 4 clients support.
- CW mode is not supported.
- RF Loopback modes M2, M3 are not supported.
- CCX & 11k not supported.
- USB ULP is not supported.
- iAP Apple Wi-Fi Home kit is not supported.
- PUF IID is not supported.
- WAPI is not supported.
- WDS is not supported.
- WMM-Admission Control is not supported.
- External 3-wire co-existence is not supported.
- LP Power save using GPIO handshake is not supported.
- DFS Master in AP is not supported.
- Radar detection in STA mode is not supported.
- Enterprise security in AP mode is not supported.

### BT/BLE
- In BLE, recommended range of Connection Interval in
  - Power Save: 100 ms to 1.28 seconds
  - BT Classic + BLE Dual Mode: >= 200 ms.
  - Wi-Fi + BLE co-ex: 30 ms to 4 seconds
- In BLE, during Connection, same values of Scan Interval and Scan Window is not recommended.
- In BT Classic, recommended value of Sniff Interval during Power Save is limited to 100ms (<= 100).
- In Wi-Fi + BT Classic co-ex, for BT file transfer to work fine, recommended value of Wi-Fi protocol DTIM is more than 3 and Beacon Interval is more than 200 ms.
- In Wi-Fi + BLE, during Wi-Fi connection, we recommend using a lower BLE Scan Window and higher BLE Scan Interval.
- BT Sniff mode does not work if BT multiple slaves feature is enabled.
- When BT multiple slaves feature is enabled, Master to Slave role switch does not work.
- In BLE, if Advertising/Scanning are in progress, and the device moves to Slave/Master role, Advertising/Scanning will be stopped. Provide respective commands to start Advertising/Scanning while in Slave/Master role.
- In Wi-Fi + BLE co-ex, if BLE Connection is established with small Connection Interval (\< 15ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.
- Observed glitches in BT audio in co-ex mode when Wi-Fi is connected.
- In BLE, if device is acting as Master/Slave, Scan Window (in set_scan_params and create_connection command) shall be less than the existing Connection Interval.
- In BLE, if BLE Connection is established with small Connection Interval (< 15 ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.
- Simultaneous Slave & Master roles (Scatter-net) is not supported.

### Co-existence
  - In Wi-Fi + BLE, if the BLE Scan Interval and Scan Window are identical, Wi-Fi connection may be unsuccessful.
  - BLE disconnection might be observed with Wi-Fi + BLE configuration with Wi-Fi continuous data transfer for low BLE supervision timeout configured. For Supervision timeout configured with value 16 seconds no disconnections are observed
  - In WiFi+BT/BLE configuration if for some reason Wi-Fi disconnects then it is observed that BT/BLE might not reconnect on a disconnection. Can be recovered through Wireless denint followed by Wireless init.
  - In Wi-Fi+BT+BLE mode, if Wi-Fi connects first followed by BT/BLE, there is a high probability that all connections will be successfully established
  - If BT/BLE gets connected first with small connection interval, then Wi-Fi tries to connect, it will have a poor chance of connecting. In this scenario, a longer BLE connection interval, supervision timeout or BT with higher supervision timeout and sniff interval will have a higher probability of successfully connecting Wi-Fi

## Terminology
| Term             | Description |
| -----------------|---------------------------------------- |
| **New Features** | These items are new to this release |
| **Changes and Fixes** | Changes made to existing features found in previous releases of the software. |
|| Enhancements to existing product flow and feature set. |
||Bug fixes done in the Release |
| **Deprecated Items** | Features or functions or APIs that are removed from the distributed release to align with the software roadmap |
| **Known Issues** | Features or functions that do not work as planned at time of release. Workarounds may be offered to meet short term development goals, but longer-term solutions will be added in future software releases |
| **Limitations/Recommendations** | Describes the limitations on product usage and recommendations for optimal use cases |

