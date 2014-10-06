ABOUT
-----
OMAPFlash is a console based program that supports the transfer of a binary image from a host PC to an OMAP based target platform.

See the OMAPFlash user guide for details on how to use OMAPFlash

See the OMAPFlash porting guide for details about porting the tool to a new platform

HISTORY
-------
Legend: * = fixed in later version

Version 5.2

    New features
        Generally changed many information messages from target to host to be handled as debug UART logs instead. 
        Added command "power_off" (only functional on OMAP4/5 and with Phoenix/Palmas PMICs).
        Changes to driver output format on debug UART.
        Added parameter 'c01d' to control CMD0-to-CMD1 delay in eMMC driver (defaults to zero). 
        Added parameter 'boot' to eMMC driver configuration to allow a partition to be marked as a boot partition.
        Added parameter 'rst_n' to allow permanent configuration of RST_N parameter for a boot partition in eMMC driver.
        Added additional decoding for several parameters in CID, CSD and EXT_CSD fields in eMMC driver.
        Added 'force_ack' flag to host application in order to allow specification of forced ACK for header and data part of all downlink messages (for host/target sync).
    Fixed issues
        Fixed problem with memory range check for address in case of branch/jump and download to DDR (would cause target to complain about good SDRAM offset address).
        Removed unnecessary code from branch and jump command handling in order to make these command work as intended.
        Fixed bug in DM check of MTIF memory test causing incorrect handling of initialization failure check.
        Fixed case sensitivity problem making HAL_CM_EnableModuleClocks and HAL_CTRL_ConfigurePads non-functional in v5.1.
        Fixed reverse-order problem and output format for EXT_CSD in eMMC driver.
        Fixed bug in expected output calculation for sparse format binaries in eMMC driver causing failure to download images with output larger than 4 GB.
        Corrected problem with missing STATUS CMD after SWITCH CMD in eMMC driver.
        Corrected issue with image size check for write functionality (image size allowed to be larger than device + offset).
    Known issues
        None
    Old known issues
        See 5.1

Version 5.1

    New features
        Addition of driver for TI fuel guage programming. Addition of configuration files for OMAP5 boards wo SDRAM and for uEMV5432. Addition
        of initial version of eMMC partition support to the eMMC driver. Addition of sparse file format support to the eMMC driver. Addition
        of support for large memory devices (address space extended to 64-bit). Addition of memory-test-only binaries for HS devices,
        allowing for provision of HS signed binaries that will support memory test but not allow for download or modification of memory
        device content. Addition of MTIF memory test. Restructuring of command parser on target side. Addition of list command to show
        supported commands in second. Addition of status command to enquire whether target is up. Update to documentation included in the
        installer. Board configuration files for 4430/4460 modified with new conditional skip command to allow for warm reset with SAR RAM
        ROM code directive to do USB peripheral boot. Android partition tables added to OMAP4 board configurations (tested on 4460 Tablet 2).
    Fixed issues
        Android partition table tested. Erase with zero length (all) in eMMC driver when using chip_erase command directly not functional.
    New known issues
        OMAP3 support in this version may have issues.
    Old known issues
        See 5.0

Version 5.0

    New features
        Added native support for partition definitions in the board configuration file. These partitions can be used to name memory
        offset and refer to these when downloading using the OMAPFlash download commands. Added support for Android partition table
        handling and Android Fastboot flashing commands in the native script handling of OMAPFlash, allowing a partition table to be
        defined in the board configuration file to be reflected in the formatting information for a memory (e.g. EMMC) - Note that at
        time of release of this version the table content has not been verified. Added support for OMAP5430 sEVM. Changed non-verbose 
        output format to produce very little in the way of output when flashing - this will be trimmed or enhanced moving forward.
    Fixed issues
        Fixed issue with erase functionality for eMMC not working properly. Correct issue causing overwrite of the data that should
        not be affected in the final partial 512 byte block of data for eMMC in one corner case. 
    New known issues
        None
    Old known issues
        See 4.20

Version 4.20

    New features
        No new features
    Fixed issues
        Fixed issue with chip_download command in case of OMAP3. Fixed issue with NAND reads for packet length greater than 64K. 
    New known issues
        None
    Old known issues
        See 4.19

Version 4.19

    New features
        No new features
    Fixed issues
        Fixed issue with emmc fails to write last partial block in some cases. Fixed issues with uploading emmc data of certain sizes. 
        Based on the errata update, CONTROL_EFUSE_2 overwrite is being removed in some configurations.
    New known issues
        None
    Old known issues
        See 4.18


Version 4.18

    New features
        Support for 4470 (tested on GP & HS).
    Fixed issues
        Fixed issue with support for OMAP3 devices in the host application peripheral boot code. Fixed problem with 4470 HS device support. Fixed
        issue in OMAP4460 EVM memory configuration for 8 Gb memory size with 2x2 Gb per channel (chip select not applied). Fixed issue in eMMC
        driver causing crash when driver relocated to 8 Gb address space.
    New known issues
        None
    Old known issues
        See 4.17


Version 4.17
    New features
        Support for OMAP4470 (untested as of release date). Addition of memory fill command. Change of memory testing algorithm so that they
        continue after failure detection. Update of eMMC driver to give better performance on data transfer. Change of dynamic memory handling
        to remove binary dependency on DDR memory size (now a board configuration parameter dependency).
    Fixed issues
        None.
    New known issues
        See 4.15.4
    Old known issues
        See 4.15.4


Version 4.16
    New features
        No new features - promotion of 4.15.4 to a main-line release.
    Fixed issues
        Correction for issue causing hang on reset commands to target.
    New known issues
        See 4.15.4
    Old known issues
        See 4.15.4

Version 4.15.4
    New features
        Addition of build option for Linux (host application) - In order to make use of this option the correct build environment including
        cross compilers must be installed (make file is a part of the package). Host application USB driver interface modified to allow for
        build on Linux - use of USB driver call back with listener thread removed.
    Fixed issues
        Modification of USB driver interface seems to have corrected BSOD crash issue on Windows 7 (looks like the listener thread in the
        USB driver was causing this and it is no longer used).
    New known issues
        See 4.15.3
    Old known issues
        See 4.15.3

Version 4.15.3
    New features
        Correction for memory handling error causing the LL FIFO buffers in the second to be deallocated and later corrupting other
        buffers allocated on the heap. Correction of error in handlig of data response from target causing incorrect call for download
        of additional data packets on failures (FF packets of FFFFFF size). Changed handling of chip_ commands so that drivers are
        only downloaded once (this new behavior can be suppressed using command line switch -renew_driver).
    Fixed issues
        The parameter rxtx_delay was incorrectly named rxtx_slowdown in the commandline handler.
    New known issues
        There have been modifications to OMAP3 specific code but the modifications have not been validated.
    Old known issues
        See 4.15.2

Version 4.15.2
    New features
        USB interface layer and link layer in second reimplemented. Added 8-byte header to packet format for target/host communication. 
        Added parameters for controlling timing for RX/TX turnaround and header-to-packet delay in host application. Reimplemented debug 
        UART functionality for second loader, configurable through the board configuration file (UART1/2/3 or USB). Added memory dump 
        commands to second loader (compile time inclusion required). Changed memory layout in order to increase heap size. Changed second 
        loader link layer so that heap and other DDR access is prevented until a download/upload is started - this enables memory testing 
        without relying on the memory for communication buffers. Made traceout of device information from eMMC driver dependent on optional
        parameter. Cleaned up configuration header file. Changed package size for data transfer to 128k. Changed package transfer for flash
        download to use a 16-packet train of data packets for each request for more data.
    Fixed issues
        None.
    New known issues
        UART download not usable - interface layer on target side needs reimplementation to match to new link layer.
    Old known issues
        See 4.15.1


Version 4.15.1
    New features
        Build system changed to use TMS470, MinGW/MSYS and MSVS2010. Dependency on CCSv3 removed.
    Fixed issues
        None.
    New known issues
        None.
    Old known issues
        See 4.15

Version 4.15
    New features
        Support for OMAP4460 ES1.1 based sEVM. Support for eMMC on MMC1. Support for memory testing using the commands:
		      mtquick <start address> <end address>
		      mtaddr <start address> <end address>
		      mtmarchx <start address> <end address>
    Fixed issues
        Wrong error report on binary image read if image size if exactly 100MB (or a multiplum of that).
    New known issues
        None.
    Old known issues
        See 4.14

Version 4.14
    New features
        Support for OMAP4430 ES2.3 based sEVM. Modifiction of Fastboot USB driver INF file to add Blaze platform PID. Additional 
        parameterization of the eMMC driver to allow control of timeout value for eMMC commands.
    Fixed issues
        Timing issue in eMMC driver timeout function observed on some memories - driver seems to time out while waiting for
        card to complete a command. Issue root caused to be due to inconsisten TO monitoring.
    New known issues
        Sligtly lower eMMC performance for download on OMAP4460 based sEVM - cause unknown.
    Old known issues
        See 4.13

Version 4.13
    New features
        Support for OMAP4460 based sEVM with 8 Gb Elpida memory configuration. Additional parameterization of the eMMC driver
        to allow control of the use of multi-block write access, maximum clock rate and normal/high density card. Additioal
        drivers for Numonyx and AMD NOR flash memories. New debug functionality for allowing register modifications to track
        activity (e.g. for configuration, LL, command interpreter and data transfer). Profiling functionality added, including
        new log command. Functions for debug and profiling added to driver API.
    Fixed issues
        File upload problem causing a halt of data transfer after 1-3 MB of data.
    New known issues
        Timing issue in eMMC driver timeout function observed on some memories - driver seems to time out while waiting for
        card to complete a command. Issue root caused to be due to inconsisten TO monitoring.
        Sligtly lower eMMC performance for download on OMAP4460 based sEVM - cause unknown.
    Old known issues
        See 4.12

Version 4.12
    New features
    Fixed issues
        Issue with i386 co-installers corrected. Binary installer was picking the wrong content for the folder in the user 
        installation and this would cause the driver installation to fail with a report of an inf file problem. Real issue
        was actually version of co-installers called.
    New known issues
        File upload problem observed - transfer may halt after 1-3 MB of data.
        Timing issue in eMMC driver timeout function observed on some memories - driver seems to time out while waiting for
        card to complete a command.
    Old known issues
        See 4.11

Version 4.11
    New features
        CSST USB driver was replaced by a WinUSB based driver, so the application can run on Windows7 (32 and 64 bit) platforms
    Fixed issues
    New known issues
    Old known issues
        See 4.10

Version 4.10
    New features
        Functionally equivalent to v4.9. Introduction of calls to new signing tool for HS/EMU second loader
        binaries.
    Fixed issues
    New known issues
    Old known issues
        See 4.9

Version 4.9
    New features
        Functionally equivalent to v4.8. Corrects a problem in the omapflash2nd.txt file that causes
        problems for correct download to OMAP4430 ES2.2 GP based sEVM.
    Fixed issues
    New known issues
    Old known issues
        See 4.8

Version 4.8
    New features
        Interrim support for OMAP4430 ES2.2 EMU/HS. This release is functionally equivalent to v4.7.
    Fixed issues
    New known issues
    Old known issues
        See 4.7

Version 4.7
    New features
        Addition of 256 Mb SDRAM support on OMAP3 based platforms.
    Fixed issues
    New known issues
    Old known issues
        See 4.6

Version 4.6
    New features
        Support for multiblock write access to eMMC devices in the eMMC driver.
    Fixed issues
	      Failure to allow access to com port above COM32. Now allows com ports from 1-64.
    New known issues
    Old known issues
        See 4.5

Version 4.5
    New features
        Support for OMAP4430 ES2.0. Addition of raw binaries for EMU/HS devices.
    Fixed issues
	  -
    New known issues
    Old known issues
        See 4.4

version 4.4
    New features
        -
    Fixed issues
        Removed modifications to the clock control registers for OMAP3 based platforms when executing 'branch' and 'jump' commands
        in order to prevent issues after SDRAM download of u-boot and branching to the executable.
    New known issues
    Old known issues
        See 4.3

version 4.3
    New features
        Additional parametrization of the OneNAND driver in order to make it more flexible. The driver can now accept a number
        of parameters from the board configuration file that will allow the user to specify the size parameters for a previously
        unknown compatible device. Driver remains 16-bit only.
    Fixed issues
        -
    New known issues
    Old known issues 
      See 4.2

version 4.2
    New features
        Addition of second loader builds and binaries for OMAP3 based platforms with 1 Gb and 512 Mb SDRAM sizes.
    Fixed issues
        -
    New known issues
    Old known issues 
      See 4.1

version 4.1
    New features
        Addition of board configuration file for Zoom2 using OMAP3430. Update of EVM37xx configuration to 200 MHz L3 clock and
        1 GHz CPU.
    Fixed issues
        -
    New known issues
    Old known issues 
      See 4.0

version 4.0
    New features
        Board and OMAP configurations no longer embedded in the second loader. All such configurations are applied through text
        based configuration files used by the host.
        Board configuration files added for SDP3630 4Gb/8Gb, Zoom3 4Gb/8Gb, SDP4430/Blaze 4Gb and EVM37xx
        Source code release available
    Fixed issues
        Peek/PeekPoke return values not displayed to user.
    New known issues
    Old known issues 
      See 3.4

version 3.4
    New features
        8G hynix support for 3630SDP and ZOOM3
    Fixed issues
        added Zoom3 emu
        fixed OMAP4 emmc not working properly when OMAPFlash running on certain machines
    New known issues
        downloading to emmc over uart on omap4 broken
	Old known issues
		See 3.3


version 3.3
    New features
        Some error messages improved
    Fixed issues
        default UART configuration fixed
        timeout crossing midnight did not occur
        Zoom3 now supported e.g. OMAPFlash.exe -omap3 -t 600 -p ZOOM3_MDDR_HYNIX_4G -2 chip_download nand test_data\pattern_1K.bin
    New known issues
        Zoom3 emu missing
        OMAP4 emmc not working properly when OMAPFlash running on certain machines
        downloading to emmc over uart on omap4 broken
	Old known issues
		See 3.2

version 3.2
    New features
        Some error messages improved
    Fixed issues
        AsicId now only printed in verbose mode (use -v)
        chip_upload SDRAM now supported
        Zoom3 now supported e.g. OMAPFlash.exe -omap3 -t 600 -p ZOOM3_MDDR_HYNIX_4G -2 chip_download nand test_data\pattern_1K.bin
    New known issues
        *default UART configuration broken use option -parity EVENPARITY
        *error handling in zoom3 did not work 
	Old known issues
		See 3.1

version 3.1
    New features
    Fixed issues
        Speed of download improved
            2.6MByte/s for 10MByte to SDRAM on OMAP4 (*5 improvement)
            3.5MByte/s for 10MByte to SDRAM on OMAP3630
            96Byte/s for 10MByte to EMMC on OMAP4 (20% improvement)
    Other changes
        Debug led pattern changed
    New known issues
        EMMC should be faster
        Error "load_file: could not open 'omapflash2nd.txt'" should be fatal
        OMAPFlash should try to locate omapflash2nd.txt in exe dir unless e.g. -config specify otherwise use -config . to obtain old behaviour
	Old known issues
		See 3.0

version 3.0
    New features
        Merged 3630 and 4430 support
        Added command extern_power_on, extern_power_off and extern_reset to run external an application that control power and reset of target when needed by the peripheral boot (for test automation)
        Added small command line application that can control ONTRAK ADU200 relay box
        Exit code set on communication failure (1: command line error, 2: peripheral boot error, 3: post boot error)
    Fixed issues
        Trace of 'eMMC CID PNM' was incorrect
        Connecting to downloaded 2nd though USB when dip switch selection included both USB and UART did not work
        Moved sample files to SampleScripts\3630 or SampleScripts\4430 directory depending on content
    New known issues
        Erase of complete eMMC by specifying size of zero is not implemented
        Giving a non hexdigit size argument to chip_erase is interpretted as 0 i.e. erase all (should give error message)
        When connecting to omap4 over uart proper default values should be used for boot package size and delay (must currently be specified manual as: -boot_package_delay 500 -max_boot_package_size 1024)
        *EMMC init error message is sometimes missing (print NULL, if deinit was called as part of cleanup) 
        Connecting through UART to 2nd running out of emmc do not work
        Missing small command line application that can control SDP mother board
        OMAPFlash should have timeout for waiting for power off/on
        No OMAP3430 support
        EMMC not supported for OMAP3630
	Old known issues
		See 2.8

version 2.8
    Fixed issues
        All versions now support 512MB (GP, GP-CH, HS/EMU)
        Heap moved to end of 512MB boundary i.e. 0x9FE50000 to 0x9FFFFFFF not available for ram download
    New known issues
        winusb.dll missing on some machines use OMAPFlashInstaller-2.7a-winusb patch to work around this problem (you dont have to do this for every time though)
        *Error in range check on available ram for download only 0x80000000 to 0x8FFFFFFF accepted
	Old known issues
		See 2.7        
        
version 2.7
    Fixed issues
		emmc_drv.bin now working
		Readme instruction updated
    New known issues
        winusb.dll missing on some machines use OMAPFlashInstaller-2.7a-winusb patch to work around this problem
        *Missing possibility to dump AsicId (needed for signing HS)
        UART disabled for HS/EMU version
        CH only supported for GP i.e. only 256MB support for HS/EMU
	Old known issues
		See 2.6

version 2.6
	Fixed issues
		emmc_drv.bin was missing
		Readme instruction updated
		Comments in uboot_emmc.txt and uboot_ram.txt updated
		Heap moved to end of 256MB boundary i.e. 0x8FE50000 to 0x8FFFFFFF not available for ram download
	New known issues
		*512MB RAM configuration only supported through CH and same heap as for 256MB
		*release emmc_drv.bin not compatible with 2.6
	Old known issues
		See 2.5
		
version 2.5
	Fixed issues
		Readme and various options files (*.txt) corrected with respec to minor errors
		eMMC erase now handle multiple 512KByte groups correctly
	New known issues
		*CH version not included
		*only 256MByte SDRAM configured (0x80000000 to 0x8FFFFFFF)
		*Heap not located at end of RAM i.e. 0x87E50000 to 0x87FFFFFF not available for ram download
		*emmc_drv.bin missing
	Old known issues
		*Upload may fail if file already exists (testcase usb_emmc_double_upload.bat)
		eMMC listed as 2GByte but is in reality 32GByte (for 4430 wakeup board) (area above 2GB not accessible through OMAPFlash)
		eMMC image size above 512MByte not tested 
		no -rxtx_trace for fastboot mode
		*No OMAP3XXX support (and installer do not support multiple version)
		Cannot poke value 0xFFFFFFFF
		MMC/SD cage not validated 
		OMAPFlash version should be listed by drivers
		Installer fails to install AdbWinApi.DLL (see INTALL section for proper action)

version 2.4
	Fixed issues
		eMMC write now work properly
		command enable_trace 5 works properly see trace.txt (except if tracing of comunication is enabled at compile time)
		Above instructions corrected
	New known issues
		Upload may fail if file already exists (testcase usb_emmc_double_upload.bat)
		eMMC listed as 2GByte but is in reality 32GByte	(area above 2GB not accessible through OMAPFlash)
		eMMC image size above 512MByte not tested 
		*eMMC erase will fail if more than one 512KByte group is to be erased
		no -rxtx_trace for fastboot mode
	Old known issues
		No OMAP3XXX support (and installer do not support multiple version)
		Cannot poke value 0xFFFFFFFF
		MMC/SD cage not validated 
		OMAPFlash version should be listed by drivers

version 2.3
	Added 
		eMMC
		Target commands for U-Boot support:
			HAL_CM_EnableModuleClocks <module> <instance>
			HAL_CTRL_ConfigurePads <module> <instance>
		Debug commands
			PEEK32 <address>
			POKE32 <address> <value>
	Known issue
		Cannot poke value 0xFFFFFFFF
		MMC/SD cage not validated 
		*only first 1024 bytes written correctly to eMMC
		*only first 1024 bytes erased correctly in eMMC
		*no progress information for eMMC erase
		erase of big areas in eMMC uneccesary slow
		command enable_trace 5 hangs 
		when using usb initial check for power-on may fail (missing request for power-off)
		eMMC CSD and Extended CSD registers assumed unchanged (erase groupsize assumed 512KB)
		non-whole erase groups are over written not erased 

version 2.2
	Fixed issues
		Fixed speed issue concerning USB download now 0.5 MByte / Second		

version 2.1
	Not released

version 2.0
	Initial version supporting OMAP4
	Known issue
		*DDR wirte only
		*No Flash support
		*No OMAP3XXX support
		*USB download slow (0.1 MByte / Second)
		