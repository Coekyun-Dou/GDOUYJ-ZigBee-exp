
Texas Instruments, Inc.

ZStack-CC2530 Release Notes

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

ZigBee 2007 Release
Version 2.2.0-1.3.0
April 7, 2009


Notices:

 - This release has been certified for ZigBee and ZigBee-PRO compliance.

 - Z-Stack supports the ZigBee 2007 Specification, including new features
   such as PanID Conflict Resolution, Frequency Agility, and Fragmentation.
   The ZigBee 2007 Specification (www.zigbee.org) defines two ZigBee stack
   profiles, ZigBee and ZigBee-Pro. This release of Z-Stack provides support
   for both of these profiles. See the Z-Stack Developer's Guide for details.

 - The library files have been built/tested with EW8051 version 7.51A/W32
   (7.51.1.3) and may not work with previous versions of the IAR tools. You
   can obtain the 7.51A installer from the IAR website.

 - When programming a target for the first time with this release, make sure
   that you select "Erase Flash" in the "Debugger->Texas Instruments->
   Download" tab in the project options. When programming completes, it is
   recommended that the "Erase Flash" box gets un-checked so that NV items
   are retained during later re-programming.

 - Please review the document "Upgrading To Z-Stack v2.2.0" for information
   about moving existing v2.1.0 applications to v2.2.0.

Changes:

 - Z-Stack now includes support for the ZigBee Alliance Smart Energy Profile.
   This includes Smart Energy defined clusters, secure joining support, and
   InterPAN communications. Refer to the "Z-Stack Developer's Guide" and the
   "Z-Stack Smart Energy Developer's Guide" documents for details.

 - This release introduces the new SE SampleApp for support of Smart Energy
   applications. Refer to the "Smart Energy Sample Application User's Guide" for
   details on the use of this application. The SETestApp has been removed.

 - Z-Stack now provides InterPAN communications, which allows ZigBee devices
   to perform limited exchange of information without joining a network. See
   the "Z-Stack Developer's Guide" document for details.

 - In previous Z-Stack releases, the MAC timer and the OSAL timer each used a
   hardware timer. Z-Stack has been redesigned to provide user access to the
   timer formerly required for OSAL (which now derives timing from the MAC).

 - Z-Stack now permits the maximum MAC frame size to be configured from the
   command-line variable MAC_MAX_FRAME_SIZE. This value defaults to 102 if
   not overridden in the IDE or a configuration file (see f8wConfig.cfg).

 - ZCL logical cluster IDs and their mappings to actual cluster IDs have been
   removed. Elimination of this unnecessary feature saves code space.

 - This release can be used with Revisions 1.3 and 1.7 of the SmartRF05EB.
   The software defaults to the 1.7 hardware. To use with 1.3 hardware,
   add the compile option HAL_BOARD_CC2530EB_REV13 to the project.

 - Changed the default IDE setting for "Location for constants and strings"
   from "RAM memory" to "ROM mapped as data". This reduces typical RAM
   usage by about 400 bytes with minor impact on code size.

 - Optimized OSAL timer calculations in the "osalTimeUpdate()" function by
   changing from 32-bit to 16-bit arithmetic. This reduced the timer offset
   computation time on 8051-based devices from ~200 usec to ~10 usec.

 - Optimized processing of the OSAL Timer linked list to reduce interrupt
   latency when searching for or deleting a timer. Testing with 10 timers
   shows a reduction from 100-150 usec (variable) to 40 usec (fixed).

 - Re-organized the handling of IEEE addresses to eliminate the "write-once"
   policy, allowing developers freedom to change IEEE address when needed.
   When a "temporary" IEEE address is generated at start-up, Z-Stack no
   longer waits for user intervention (a flashing LED used to prompt for a
   button push). Refer to Section 7.2 of the "Z-Stack User's Guide" for details.

 - Made assignment of a Router address (tree addressing) to an End-Device
   an option (defaulted off). This permits an End-Device to occupy unused
   Router address in topologies with few Routers. In the past, this feature
   was always enabled and could "lock out" Routers when End-Devices
   and Routers were joining at the same time.

 - RSSI and correlation are now passed (along with LQI) from the MAC data
   indication, through the network layer (including InterPAN), APS, and AF
   to the application.


Bug Fixes:

 - Fixed a problem in networks with asymetric links, where routers could
   setup routes that were incorrect. In some instances, devices weren't
   taking proper actions to remove these incorrect routing table entries
   when they failed. [2800]

 - Fixed a MAC problem where a "lock-up" could rarely happen under high
   traffic device joining situations (such as network-wide reset). Problem
   was mis-handling of "reserved"/"not reserved" status of allocated queue
   buffers for MAC RX, MAC TX, or OSAL messages. [2763]

 - Fixed potential usage (read) of an uninitialized structure pointer in the
   function "zcl_SendReadReportCfgRspCmd()". [2754]

 - Fixed potential usage (memory allocation) of an uninitialized buffer length
   variable in the function"zclProcessInWriteUndividedCmd()". [2724]

 - Fixed a potential buffer overrun when processing ZCL "Configure Reporting
   Response" command. [2670]

 - Fixed a potential buffer overrun for more than 16 devices in DEVICE_INFO
   MT response message. [2652]

 - Fixed ZCL problems which returned invalid group.ID values when processing
   a received Groups Command, and invalid groupID and transTime values when
   processing a received Scenes Command. [2633]

 - Repaired a problem where an APS transport key message was sent to other
   than just the joining device. This would incorrectly reset the frame counter
   on the wrong device(s). [2601]

 - Fixed a MAC timer problem which could result in a failure if two timers were
   set to expire on the same backoff. [2478]

 - Added new parameter to the ZCL foundation validation callback function to
   pass the cluster ID and the attribute info up to the application. [2469]

 - Repaired a problem in which a parent device would deliver an encrypted
   transport key when there was no pre-configured key. [2460]

 - Fixed a problem where: after an address coflict was resolved, Link Status
   messages sent out by the device with the conflicted address would contain
   an extra entry for the old address. [2458]

 - Repaired a problem in which a parent device would deliver the dummy key
   to a joining device in the clear. The dummy key is now encrypted with the
   NWK key. [2451]

 - Changed the System Server Discovery Response to set the APS ACK bit to
   1 in the APS header. [2447]

 - Changed the Network Report command to include both Source and Destination
   IEEE addresses in the NWK header. [2445]

 - Repaired the Route Request and Route Reply: changed the broadcast address
   of the Route Request from 0xFFFF to 0xFFFD, and changed the NWK header of
   the Route Reply to include the Destination IEEE address. [2444]

 - Repaired the Network Leave command: the NWK header now includes Source
   IEEE address, and the broadcast address is now 0xFFFD instead of 0xFFFF.
   [2443]

 - Removed a time delay caused when a device is a member of a group that it
   is trying to send a message to. [2432]

 - Updated the DataRequest processing to set the ACK request bit in the APS
   frame for all fragmented packets. [2395]

 - Fixed a problem that caused a fragmentation failure when sending a long
   message (OTA length > 127 bytes) with APS security enabled. [2391]

 - Updated the ZCL callback processing functions to support more than one
   endpoint -- a pointer to callback function was only being returned when
   the specified endpoint was the first item in the linked-list. [2389]

 - Restored processing of IEEE_addr-req messages for sleeping End-Devices.
   [2352]

 - Repaired a route error reporting problem by routing to a device's parent
   when the route request was issued for the device's neighbor (only applies
   to "distributed address" networks). [2351]

 - The afRegister(..) function has been changed to check for a duplicate end
   point before adding an application's descriptor. [2341]

 - Return values have been added to the functions in hal_ccm.h, previously
   declared as void. In some instances, a memory allocation failure in one
   of these functions would not be returned to the caller. [2276]

 - Z-Tool has been updated to properly parse and display callback responses
   for ZDO_MGMT_NWK_DISC_REQ and ZDO_MGMT_LQI_REQ commands. [2248]


Known Issues:

 - OAD supports two modes, internal and external. Internal mode does not
   use an external flash memory, so the OAD image must fit within unused
   flash memory on the CC2530. In other words, two images must fit in the
   CC2530 memory. External mode uses and external flash memory, which
   is 128K on SmartRF05EB Rev 1.3 boards, and 256K on 1.7 boards. 

 - Z-Stack now provides support for Texas Instruments CC2591 PA/LNA range
   extender. The CC2591 increases the RF link budget by a PA for increased
   transmit output power and an LNA for improved receiver sensitivity. See
   the "HAL Driver API" document for details. Note that the CC2530 + CC2591
   "Combo" board is not yet available to assist in evaluation of the PA/LNA.

 - To disable security at build time, use the "SECURE=0" compile option. Do
   not attempt to disable security by setting the SECURITY_LEVEL to zero.

 - The ZDO Complex Descriptor is not supported.

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

For technical support please contact:

Texas Instruments, Inc.
Low Power RF
lpwsupport@ti.com
