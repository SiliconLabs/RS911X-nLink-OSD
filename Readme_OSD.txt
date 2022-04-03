Incompatible license headers
============================

The licnese headers on these files are incompatible with the GPL, but the code uses 2 macros that indicate the license is GPL.

MODULE_LICENSE("Dual BSD/GPL");
EXPORT_SYMBOL_GPL

The header on these source files claims some kind of Silicon labs license that is not compatible with the GPL. Please fix the headers on these files to make the header compatible with the GPL or remove the GPL exports.


This software package contains following folders/files :

Package
=======
RS9116.NB0.NL.GNU.LNX.OSD.X.X.X
	├── Readme_OSD.txt
	├── ReleaseNotes_OSD.pdf
	├── scripts
	├── Firmware
	└── rsi

Content Description
===================

	- ReleaseNotes_OSD.pdf
		This document contains supported features, hardware and software requirements, and known issues of this release.

	- Scripts 
		This folder contains configuration files of supplicant, hostapd and scripts for running dhcp server.

	- Firmware: 
		This folder contains firmware binaries for 9116.

	- rsi
		This folder contains source files Open source driver.
