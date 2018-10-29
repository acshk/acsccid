/*
    ccid_ifdhandler.h: non-generic ifdhandler functions
    Copyright (C) 2004-2010   Ludovic Rousseau
    Copyright (C) 2010-2018   Advanced Card Systems Ltd.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#ifndef _ccid_ifd_handler_h_
#define _ccid_ifd_handler_h_

#define IOCTL_SMARTCARD_VENDOR_IFD_EXCHANGE	SCARD_CTL_CODE(1)

#define CLASS2_IOCTL_MAGIC 0x330000
#define IOCTL_FEATURE_VERIFY_PIN_DIRECT \
	SCARD_CTL_CODE(FEATURE_VERIFY_PIN_DIRECT + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_MODIFY_PIN_DIRECT \
	SCARD_CTL_CODE(FEATURE_MODIFY_PIN_DIRECT + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_MCT_READER_DIRECT \
	SCARD_CTL_CODE(FEATURE_MCT_READER_DIRECT + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_IFD_PIN_PROPERTIES \
	SCARD_CTL_CODE(FEATURE_IFD_PIN_PROPERTIES + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_WRITE_DISPLAY \
	SCARD_CTL_CODE(FEATURE_WRITE_DISPLAY + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_GET_KEY \
	SCARD_CTL_CODE(FEATURE_GET_KEY + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_IFD_DISPLAY_PROPERTIES \
	SCARD_CTL_CODE(FEATURE_IFD_DISPLAY_PROPERTIES + CLASS2_IOCTL_MAGIC)
#define IOCTL_FEATURE_GET_TLV_PROPERTIES \
	SCARD_CTL_CODE(FEATURE_GET_TLV_PROPERTIES + CLASS2_IOCTL_MAGIC)

// MS CCID I/O control code for escape command
#define IOCTL_CCID_ESCAPE	SCARD_CTL_CODE(3500)

// ACR83U, ACR85 and APG8201 specific I/O controls
#define IOCTL_SMARTCARD_GET_FIRMWARE_VERSION	SCARD_CTL_CODE(2078)
#define IOCTL_SMARTCARD_DISPLAY_LCD_MESSAGE		SCARD_CTL_CODE(2079)
#define IOCTL_SMARTCARD_READ_KEY				SCARD_CTL_CODE(2080)

// ACR38U, ACR38U-SAM and SCR21U specific I/O controls
#define IOCTL_SMARTCARD_SET_CARD_TYPE		SCARD_CTL_CODE(2060)
#define IOCTL_SMARTCARD_SET_CARD_VOLTAGE	SCARD_CTL_CODE(2074)

// ACR128 I/O control code for escape command
#define IOCTL_ACR128_READER_COMMAND	SCARD_CTL_CODE(2079)

/* Control code for toggling the card state in SAM slot */
#define IOCTL_SMARTCARD_TOGGLE_CARD_STATE	SCARD_CTL_CODE(2075)

/* Control codes for Windows compatibility. */
#define WINSCARD_CTL_CODE(code)					(0x310000 | (code << 2))
#define WINIOCTL_CCID_ESCAPE					WINSCARD_CTL_CODE(3500)
#define WINIOCTL_SMARTCARD_GET_FIRMWARE_VERSION	WINSCARD_CTL_CODE(2078)
#define WINIOCTL_SMARTCARD_DISPLAY_LCD_MESSAGE	WINSCARD_CTL_CODE(2079)
#define WINIOCTL_SMARTCARD_READ_KEY				WINSCARD_CTL_CODE(2080)
#define WINIOCTL_SMARTCARD_SET_CARD_TYPE		WINSCARD_CTL_CODE(2060)
#define WINIOCTL_SMARTCARD_SET_CARD_VOLTAGE		WINSCARD_CTL_CODE(2074)
#define WINIOCTL_ACR128_READER_COMMAND			WINSCARD_CTL_CODE(2079)
#define WINIOCTL_SMARTCARD_TOGGLE_CARD_STATE	WINSCARD_CTL_CODE(2075)

#define DRIVER_OPTION_CCID_EXCHANGE_AUTHORIZED 1
#define DRIVER_OPTION_GEMPC_TWIN_KEY_APDU 2
#define DRIVER_OPTION_USE_BOGUS_FIRMWARE 4
#define DRIVER_OPTION_DISABLE_PIN_RETRIES (1 << 6)

// ACS driver option
#define ACS_DRIVER_OPTION_REMOVE_PUPI_FROM_ATR	1
#define ACS_DRIVER_OPTION_DISABLE_PICC			2

extern int DriverOptions;

/*
 * Maximum number of CCID readers supported simultaneously
 *
 * The maximum number of readers is also limited in pcsc-lite (16 by default)
 * see the definition of PCSCLITE_MAX_READERS_CONTEXTS in src/PCSC/pcsclite.h
 */
#define CCID_DRIVER_MAX_READERS 16

/*
 * CCID driver specific functions
 */
CcidDesc *get_ccid_slot(unsigned int reader_index);

#endif

