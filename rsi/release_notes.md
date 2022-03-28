# Introduction

This document offers users of Silicon Labs software with an insight to the changes found between different releases.

## Document reference
| Term             | Description |
|------------------|------------------------------------------|
| **New Features** | These items are new to this release. |
| **Changes/Issues fixed** | These are the changes made to existing features found in previous releases of the software. These are not considered bugs but enhancements to existing product flow and feature set. |
|**Known Issues**  | Features that do not or have not functioned as intended that are repaired as part of this release. |
| **Limitations/ Recommendations** | Describes what are the limitations on how we use the product and recommendations for optimal use cases. |

## Reference documentation for RS9116 nLink software

First time users of this software should consider reviewing the

- RS9116N_Open_Source_Driver_Technical_Reference_Manual_vX.X.pdf

- **UG452\_ RS9116N\_EVK\_Software\_User's\_Guide\_vX.X.pdf**


# RS9116.NB0.NL.GNU.LNX.OSD.2.5.1.11

## Index
- [Release Details](#release-details)
- [New Features](#new-features)
- [Changes/Issues Fixed](#changes/issues-fixed)
- [Known Issues](#known-issues)
- [Wi-Fi Limitations/Feature Not Supported](#wi-fi-limitations/features-not-supported)
- [BT/BLE Recommendations](#bt/ble-recommendations)
- [BT/BLE Limitations/Features Not Supported](#bt/ble-limitations/features-not-supported)
- [CoEx Limiatations](#coex-limitations)
- [CoEx Scenarios & Recommendations](#coex-scenarios-&-recommendations)
- [Folder Structure Changes](folder-structure-changes)


## Release Details
| Item                                                | Details                                |
|-----------------------------------------------------|----------------------------------------|
| Release date                                        | Friday 25th March 2022                      |
| Package Name                                        | RS9116.NB0.NL.GNU.LNX.OSD.2.5.1.11                       |
| Firmware Version                                    | 1610.2.5.1.0.11                  |
| Hardware Modules/Chipsets                           | Q7, B00, C00, CC1, AB0, AB1, AA0, AA1 |
| Supported Linux Kernel Versions                           | From v3.18. to v5.15     |
| Host interfaces supported | USB, SDIO         |

## New Features

### Wi-Fi

- Provision to Update Gain Table for certification

- Fast PSP support in power save profile

### Bluetooth - Common

   -None-

### Bluetooth – Classic

-None-

### Bluetooth – LE

-None-

## Changes/Issues Fixed

### Wi-Fi

- Fixed Tx Power issue in AA0/CC0 Modules.

- Added minimum supported rate for EAPOL packet transmission.

- Added a fix for SDIO suspend/Resume.

- Added fixes for  the Wi-Fi FragAttack Vulnerability.

- RS9116X-DB00-CCX-BXX certified TELEC gain tables included.

### Bluetooth – Common

- Added a fix to initiate AFH periodically to resolve the BT and Wi-Fi hang issue.

- Resolved the Wi-Fi ping issue that used to occur while connecting to BT in CoEx Mode in the CC0/AA0 modules based on chip version 1.3.

### Bluetooth – Classic

-None-

### Bluetooth – LE

-None-

## Known Issues:

- BT-HID might not inter-operate with Apple devices.

- In Wi-Fi + BT/BLE coex mode, high Wi-Fi broadcast traffic, might cause BT/BLE disconnections.

- Set rate can not be used for setting non MCS and non basic rates in kernels above 4.13.16.

- Enterprise security with WPA3 is not supported.

- Tx Rate information is not being updated in WLAN interfaces stats reported by iwconfig tool.

- Auto Channel Selection in AP with WORLD region will not work.

## Wi-Fi Limitations/Feature Not Supported

- Tx AMSDU is not supported.

- Fragmentation is not supported.

- AMSDU's within AMPDU is not supported.

- Wi-Fi Performance is less in dense environments.

- 11j not supported.

- In AP mode more than 16 clients are not supported, AP + BT + BLE has 4 clients support.

- USB ULP is not supported.

- External 3-wire coexistence is not supported.

- LP Power save using GPIO handshake is not supported.

- DFS Master is not supported.

- Radar detection in STA mode is not supported.

## BT/BLE Recommendations

- In BLE, the recommended range of Connection Interval in

  - Power Save - 100ms to 1.28sec.

  - BT Classic + BLE Dual Mode is \>= 200ms.

  - Wi-Fi + BLE coex - 30ms to 4sec

- In BLE, during Connection, using the same value for Scan Interval and Scan Window is not recommended.

- In BT Classic, the recommended value of Sniff Interval during Power Save is limited to 100ms (\<= 100).

- In Wi-Fi + BT Classic coex, for BT file transfer to work fine, the recommended value of Wi-Fi protocol DTIM is more than 3, and that of Beacon Interval is more than 200ms.

- In Wi-Fi + BLE, during Wi-Fi connection, using a lower BLE Scan Window and larger BLE Scan Interval is recommended.

## BT/BLE Limitations/Features Not Supported

-  In CC0/CC1 dual-band modules, the power consumption during transmission in 2.4 GHz is sometimes higher by up to 30% to 40%

- Power consumption in BLE connected sleep is higher by ~40%

- BT Sniff mode does not work if BT multiple slaves feature is enabled.

- When BT multiple slaves feature is enabled, Master to Slave role switch will not happen.

- In BLE, if Advertising/Scanning is in progress, and the device switches its Slave/Master role, Advertising/Scanning will be stopped. Commands to start Advertising/Scanning must be provided again.

- Glitches can be observed in BT audio if Wi-Fi is in connected state.

- In BLE, if device is acting as Master/Slave, Scan Window (in set_scan_params and create_connection command) shall be less than the existing Connection Interval.

- In BLE, if BLE Connection is established with small Connection Interval (\< 15ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.

- BT Classic Simultaneous Slave & Master roles (Scatter-net) is not supported.

## CoEx Limitations

- In Wi-Fi + BLE, if the BLE scan interval and scan window are identical, Wi-Fi connection may be unsuccessful.

- BLE disconnection might be observed with Wi-Fi + BLE configuration with Wi-Fi continuous data transfer for low BLE supervision timeout configured. For Supervision timeout configured with value 16 seconds or higher, no disconnections are observed

## CoEx Scenarios & Recommendations

- If BT/BLE gets connected first with small connection interval, then there is a likelihood of difficulty in Wi-Fi connection. It is recommended to use a longer connection interval and supervision timeout for BLE or a longer supervision timeout and sniff interval for  BT

## Folder Structure Changes

-None-

# RS9116.NB0.NL.GNU.LNX.OSD.2.3.2.0003

## Index
- [Release Details](#release-details)
- [New Features](#new-features)
- [Changes/Issues Fixed](#changes/issues-fixed)
- [Known Issues](#known-issues)
- [Wi-Fi Limitations/Feature Not Supported](#wi-fi-limitations/features-not-supported)
- [BT/BLE Recommendations](#bt/ble-recommendations)
- [BT/BLE Limitations/Features Not Supported](#bt/ble-limitations/features-not-supported)
- [CoEx Limiatations](#coex-limitations)
- [CoEx Scenarios & Recommendations](#coex-scenarios-&-recommendations)
- [Folder Structure Changes](folder-structure-changes)

## Release Details
| Item                                                | Details                                |
|-----------------------------------------------------|----------------------------------------|
| Release date                                        | Friday Monday 5th July 2021                      |
| Package Name                                        | RS9116.NB0.NL.GNU.LNX.OSD.2.3.2.0003                       |
| Firmware Version                                    | 1610.2.3.2.0003                  |
| Hardware Modules/Chipsets                           | Q7, B00, C00, CC1, AB0, AB1, AA0, AA1 |
| Supported Linux Kernel Versions                           | From v3.18. to v5.7     |
| Host interfaces supported | USB, SDIO         |


## New Features

### Wi-Fi

- Added Support for EN 300 328 V2.2.2.

### Bluetooth - Common

   -None-

### Bluetooth – Classic

-None-

### Bluetooth – LE

-None-

## Changes/Issues Fixed

### Wi-Fi

   -None-

### Bluetooth – Common

- Fixed STA disconnection issue in Coex mode while giving lescan command.

### Bluetooth – Classic

-None-

### Bluetooth – LE

-None-

## Known Issues:

- BT-HID might not inter-operate with Apple devices

- In Wi-Fi + BT/BLE coex mode, high Wi-Fi broadcast traffic, might cause BT/BLE disconnections.

- Issues with BT PER Continuous mode transmission

- Set rate can not be used for setting non MCS and non basic rates in kernels above 4.13.16.

- Enterprise security with WPA3 is not supported.

- TX rate update to the use (iwconfig) is not supported.

- BT/BLE coex modes with WiFi AP are not supported.

- EAP-TLS 1.2 /1.1 are not supported in this release.

- Auto Channel Selection in AP with WORLD region will not work.

## Wi-Fi Limitations/Feature Not Supported

- AMSDU TX is not supported

- Fragmentation is not supported.

- AMSDU's within AMPDU is not supported.

- Wi-Fi Performance is less in dense environments.

- Digital Loopback is not supported.

- PUF is not supported.

- 11j not supported.

- 11p is not supported.

- AP mode more than 16 clients not supported, AP + BT + BLE has 4 clients support.

- CW Mode is not supported.

- RF Loopback modes M2, M3 are not supported.

- CCX & 11k not supported.

- USB ULP is not supported.

- iAP Apple Wi-Fi Home kit is not supported.

- PUF IID is not supported.

- WAPI is not supported.

- WDS is not supported.

- WMM-Admission Control is not supported.

- External 3-wire coexistence is not supported.

- LP Power save using GPIO handshake is not supported.

- DFS Master in AP is not supported.

- Radar detection in STA mode is not supported.

- Enterprise security in AP mode is not supported.

## BT/BLE Recommendations

- In BLE, recommended range of Connection Interval in

  - Power Save - 100ms to 1.28sec.

  - BT Classic + BLE Dual Mode is \>= 200ms.

  - Wi-Fi + BLE coex - 30ms to 4sec

- In BLE, during Connection, same values of Scan Interval and Scan Window is not recommended.

- In BT Classic, recommended value of Sniff Interval during Power Save is limited to 100ms (\<= 100).

- In Wi-Fi + BT Classic coex, for BT file transfer to work fine, recommended value of Wi-Fi protocol DTIM is more than 3 and Beacon Interval is more than 200ms.

- In Wi-Fi + BLE, during Wi-Fi connection, recommending the lesser BLE scan Window and larger BLE scan Interval.

## BT/BLE Limitations/Features Not Supported

- BT Sniff mode does not work if BT multiple slaves feature is enabled.

- When BT multiple slaves feature is enabled, Master to slave role switch will not happen.

- In BLE, if Advertising/Scanning are in progress, and the device moves to Slave/Master role, Advertising/Scanning will be stopped. Provide respective commands to start Advertising/Scanning while being in Slave/Master role.

- In Wi-Fi + BLE coex, if BLE Connection is established with small Connection Interval (\< 15ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.

- Observed glitches in BT audio in coex mode when Wi-Fi is connected.

- In BLE, if device is acting as Master/Slave, Scan Window (in set_scan_params and create_connection command) shall be less than the existing Connection Interval.

- In BLE, if BLE Connection is established with small Connection Interval (\< 15ms), simultaneous roles (i.e. Master/Slave + Advertising/Scanning) are not supported.

- Simultaneous Slave & Master roles (Scatter-net) is not supported.

## CoEx Limitations

- In Wi-Fi + BLE, during Wi-Fi connection, if both BLE scan interval and window are same then there will be issue in successfully making the Wi-Fi connection.

- BLE disconnection might be observed with Wi-Fi + BLE configuration with Wi-Fi continuous data transfer for low BLE supervision timeout configured. For Supervision timeout configured with value 16 seconds no disconnections are observed

- In WiFi+BT/BLE configuration if for some reason Wi-Fi disconnects then it is observed that BT/BLE might not reconnect on a disconnection. Can be recovered through Wireless denint followed by Wireless init.

## CoEx Scenarios & Recommendations

- In Wi-Fi+BT+BLE if Wi-Fi connects first followed by BT/BLE there is a high probability of all connections to successfully establish

- If BT/BLE gets connected first with small connection interval, then Wi-Fi tries to connect, it will have a poor chance of connecting. In this scenario BLE longer connection interval, supervision timeout or BT with higher supervision timeout and sniff interval has a higher probability of Wi-Fi getting connected

## Folder Structure Changes

-None-


