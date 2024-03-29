/*
	ccid.c: CCID common code
	Copyright (C) 2003-2010   Ludovic Rousseau
	Copyright (C) 2009-2023   Advanced Card Systems Ltd.

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

#include <config.h>

#ifdef HAVE_STDIO_H
#include <stdio.h>
#endif
#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif
#ifdef HAVE_STRING_H
#include <string.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include <pcsclite.h>
#include <ifdhandler.h>

#include "debug.h"
#include "ccid.h"
#include "defs.h"
#include "ccid_ifdhandler.h"
#include "commands.h"
#include "utils.h"
#include "acr38cmd.h"

#ifdef __APPLE__
#include <CoreFoundation/CoreFoundation.h>
#endif

static int ACR83_GetFirmwareVersion(unsigned int reader_index, unsigned int *pVersion1, unsigned int *pVersion2);
static int ACR83_DisplayLcdMessage(unsigned int reader_index, const char *message);
static int ACR1222_GetFirmwareVersion(unsigned int reader_index, char *firmwareVersion, unsigned int *pFirmwareVersionLen);

/*****************************************************************************
 *
 *					ccid_open_hack_pre
 *
 ****************************************************************************/
int ccid_open_hack_pre(unsigned int reader_index)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
	int i;

	switch (ccid_descriptor->readerID)
	{
		case MYSMARTPAD:
			ccid_descriptor->dwMaxIFSD = 254;
			break;

		case CL1356D:
			/* the firmware needs some time to initialize */
			(void)sleep(1);
			ccid_descriptor->readTimeout = 60*1000; /* 60 seconds */
			break;

		case OZ776:
		case OZ776_7772:
			ccid_descriptor->dwMaxDataRate = 9600;
			break;

		case ElatecTWN4_CCID_CDC:
		case ElatecTWN4_CCID:
			/* Use a timeout of 1000 ms instead of 100 ms in
			 * CmdGetSlotStatus() used by CreateChannelByNameOrChannel()
			 * The reader answers after up to 1 s if no tag is present */
			ccid_descriptor->readTimeout = DEFAULT_COM_READ_TIMEOUT * 10;
			break;

		case SCM_SCL011:
		case IDENTIV_uTrust3700F:
		case IDENTIV_uTrust3701F:
		case IDENTIV_uTrust4701F:
			/* The SCM SCL011 reader needs 350 ms to answer */
			ccid_descriptor->readTimeout = DEFAULT_COM_READ_TIMEOUT * 4;
			break;

		case ACS_ACR122U:
		case ACS_AET62_PICC_READER:
		case ACS_AET62_1SAM_PICC_READER:
			// Enable polling mode (ACR122 v2.06)
			for (i = 0; i < 10; i++)
			{
				if (CmdPowerOff(reader_index) == IFD_SUCCESS)
				{
					(void)sleep(1);
					break;
				}
			}
			break;

		default:
			break;
	}

#ifndef __APPLE__
	/* CCID */
	if (((PROTOCOL_CCID == ccid_descriptor->bInterfaceProtocol)
		|| (PROTOCOL_ACR38 == ccid_descriptor->bInterfaceProtocol))
		&& (3 == ccid_descriptor -> bNumEndpoints))
	{
#ifndef TWIN_SERIAL
		/* just wait for 100ms in case a notification is in the pipe */
		(void)InterruptRead(reader_index, 100);
#endif
	}
#endif

	/* ICCD type A */
	if (PROTOCOL_ICCD_A == ccid_descriptor->bInterfaceProtocol)
	{
		unsigned char tmp[MAX_ATR_SIZE];
		unsigned int n = sizeof(tmp);

		DEBUG_COMM("ICCD type A");
		(void)CmdPowerOff(reader_index);
		(void)CmdPowerOn(reader_index, &n, tmp, VOLTAGE_AUTO);
		(void)CmdPowerOff(reader_index);
	}

	/* ICCD type B */
	if (PROTOCOL_ICCD_B == ccid_descriptor->bInterfaceProtocol)
	{
		unsigned char tmp[MAX_ATR_SIZE];
		unsigned int n = sizeof(tmp);

		DEBUG_COMM("ICCD type B");
		if (CCID_CLASS_SHORT_APDU ==
			(ccid_descriptor->dwFeatures & CCID_CLASS_EXCHANGE_MASK))
		{
			/* use the extended APDU comm alogorithm */
			ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
			ccid_descriptor->dwFeatures |= CCID_CLASS_EXTENDED_APDU;
		}

		(void)CmdPowerOff(reader_index);
		(void)CmdPowerOn(reader_index, &n, tmp, VOLTAGE_AUTO);
		(void)CmdPowerOff(reader_index);
	}

	return 0;
} /* ccid_open_hack_pre */

#ifndef NO_LOG
/*****************************************************************************
 *
 *					dump_gemalto_firmware_features
 *
 ****************************************************************************/
static void dump_gemalto_firmware_features(struct GEMALTO_FIRMWARE_FEATURES *gff)
{
	DEBUG_INFO2("Dumping Gemalto firmware features (%zd bytes):",
		sizeof(struct GEMALTO_FIRMWARE_FEATURES));

#define YESNO(x) (x) ? "yes" : "no"

	DEBUG_INFO2(" bLogicalLCDLineNumber: %d", gff->bLogicalLCDLineNumber);
	DEBUG_INFO2(" bLogicalLCDRowNumber: %d", gff->bLogicalLCDRowNumber);
	DEBUG_INFO2(" bLcdInfo: 0x%02X", gff->bLcdInfo);
	DEBUG_INFO2(" bEntryValidationCondition: 0x%02X",
		gff->bEntryValidationCondition);

	DEBUG_INFO1(" Reader supports PC/SCv2 features:");
	DEBUG_INFO2("  VerifyPinStart: %s", YESNO(gff->VerifyPinStart));
	DEBUG_INFO2("  VerifyPinFinish: %s", YESNO(gff->VerifyPinFinish));
	DEBUG_INFO2("  ModifyPinStart: %s", YESNO(gff->ModifyPinStart));
	DEBUG_INFO2("  ModifyPinFinish: %s", YESNO(gff->ModifyPinFinish));
	DEBUG_INFO2("  GetKeyPressed: %s", YESNO(gff->GetKeyPressed));
	DEBUG_INFO2("  VerifyPinDirect: %s", YESNO(gff->VerifyPinDirect));
	DEBUG_INFO2("  ModifyPinDirect: %s", YESNO(gff->ModifyPinDirect));
	DEBUG_INFO2("  Abort: %s", YESNO(gff->Abort));
	DEBUG_INFO2("  GetKey: %s", YESNO(gff->GetKey));
	DEBUG_INFO2("  WriteDisplay: %s", YESNO(gff->WriteDisplay));
	DEBUG_INFO2("  SetSpeMessage: %s", YESNO(gff->SetSpeMessage));
	DEBUG_INFO2("  bTimeOut2: %s", YESNO(gff->bTimeOut2));
	DEBUG_INFO2("  bPPDUSupportOverXferBlock: %s",
		YESNO(gff->bPPDUSupportOverXferBlock));
	DEBUG_INFO2("  bPPDUSupportOverEscape: %s",
		YESNO(gff->bPPDUSupportOverEscape));

	DEBUG_INFO2(" bListSupportedLanguages: %s",
		YESNO(gff->bListSupportedLanguages));
	DEBUG_INFO2(" bNumberMessageFix: %s", YESNO(gff->bNumberMessageFix));

	DEBUG_INFO2(" VersionNumber: 0x%02X", gff->VersionNumber);
	DEBUG_INFO2(" MinimumPINSize: %d", gff->MinimumPINSize);
	DEBUG_INFO2(" MaximumPINSize: %d", gff->MaximumPINSize);
	DEBUG_INFO2(" Firewall: %s", YESNO(gff->Firewall));
	if (gff->Firewall && gff->FirewalledCommand_SW1
		&& gff->FirewalledCommand_SW2)
	{
		DEBUG_INFO2("  FirewalledCommand_SW1: 0x%02X",
			gff->FirewalledCommand_SW1);
		DEBUG_INFO2("  FirewalledCommand_SW2: 0x%02X",
			gff->FirewalledCommand_SW2);
	}

} /* dump_gemalto_firmware_features */
#endif

/*****************************************************************************
 *
 *					set_gemalto_firmware_features
 *
 ****************************************************************************/
static void set_gemalto_firmware_features(unsigned int reader_index)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
	struct GEMALTO_FIRMWARE_FEATURES *gf_features;

	gf_features = malloc(sizeof(struct GEMALTO_FIRMWARE_FEATURES));
	if (gf_features)
	{
		unsigned char cmd[] = { 0x6A }; /* GET_FIRMWARE_FEATURES command id */
		unsigned int len_features = sizeof *gf_features;
		RESPONSECODE ret;

		ret = CmdEscapeCheck(reader_index, cmd, sizeof cmd,
			(unsigned char*)gf_features, &len_features, 0, TRUE);
		if ((IFD_SUCCESS == ret) &&
			(len_features == sizeof *gf_features))
		{
			/* Command is supported if it succeeds at CCID level */
			/* and returned size matches our expectation */
			ccid_descriptor->gemalto_firmware_features = gf_features;
#ifndef NO_LOG
			dump_gemalto_firmware_features(gf_features);
#endif
		}
		else
		{
			/* Command is not supported, let's free allocated memory */
			free(gf_features);
			DEBUG_INFO3("GET_FIRMWARE_FEATURES failed: " DWORD_D ", len=%d",
				ret, len_features);
		}
	}
} /* set_gemalto_firmware_features */

/*****************************************************************************
 *
 *					ccid_open_hack_post
 *
 ****************************************************************************/
int ccid_open_hack_post(unsigned int reader_index)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
	RESPONSECODE return_value = IFD_SUCCESS;

	switch (ccid_descriptor->readerID)
	{
		case GEMPCKEY:
		case GEMPCTWIN:
			/* Reader announces TPDU but can do APDU (EMV in fact) */
			if (DriverOptions & DRIVER_OPTION_GEMPC_TWIN_KEY_APDU)
			{
				unsigned char cmd[] = { 0x1F, 0x02 };
				unsigned char res[10];
				unsigned int length_res = sizeof(res);

				if (CmdEscape(reader_index, cmd, sizeof(cmd), res, &length_res, 0) == IFD_SUCCESS)
				{
					ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
					ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
				}
			}
			break;

		case VEGAALPHA:
		case GEMPCPINPAD:
			/* load the l10n strings in the pinpad memory */
			{
#define L10N_HEADER_SIZE 5
#define L10N_STRING_MAX_SIZE 16
#define L10N_NB_STRING 10

				unsigned char cmd[L10N_HEADER_SIZE + L10N_NB_STRING * L10N_STRING_MAX_SIZE];
				unsigned char res[20];
				unsigned int length_res = sizeof(res);
				int offset, i, j;

				const char *fr[] = {
					"Entrer PIN",
					"Nouveau PIN",
					"Confirmer PIN",
					"PIN correct",
					"PIN Incorrect !",
					"Delai depasse",
					"* essai restant",
					"Inserer carte",
					"Erreur carte",
					"PIN bloque" };

				const char *de[] = {
					"PIN eingeben",
					"Neue PIN",
					"PIN bestatigen",
					"PIN korrect",
					"Falsche PIN !",
					"Zeit abgelaufen",
					"* Versuche ubrig",
					"Karte einstecken",
					"Fehler Karte",
					"PIN blockiert" };

				const char *es[] = {
					"Introducir PIN",
					"Nuevo PIN",
					"Confirmar PIN",
					"PIN OK",
					"PIN Incorrecto !",
					"Tiempo Agotado",
					"* ensayos quedan",
					"Introducir Tarj.",
					"Error en Tarjeta",
					"PIN bloqueado" };

				const char *it[] = {
					"Inserire PIN",
					"Nuovo PIN",
					"Confermare PIN",
					"PIN Corretto",
					"PIN Errato !",
					"Tempo scaduto",
					"* prove rimaste",
					"Inserire Carta",
					"Errore Carta",
					"PIN ostruito"};

				const char *pt[] = {
					"Insira PIN",
					"Novo PIN",
					"Conf. novo PIN",
					"PIN OK",
					"PIN falhou!",
					"Tempo expirou",
					"* tentiv. restam",
					"Introduza cartao",
					"Erro cartao",
					"PIN bloqueado" };

				const char *nl[] = {
					"Inbrengen code",
					"Nieuwe code",
					"Bevestig code",
					"Code aanvaard",
					"Foute code",
					"Time out",
					"* Nog Pogingen",
					"Kaart inbrengen",
					"Kaart fout",
					"Kaart blok" };

				const char *tr[] = {
					"PIN Giriniz",
					"Yeni PIN",
					"PIN Onayala",
					"PIN OK",
					"Yanlis PIN",
					"Zaman Asimi",
					"* deneme kaldi",
					"Karti Takiniz",
					"Kart Hatasi",
					"Kart Kilitli" };

				const char *en[] = {
					"Enter PIN",
					"New PIN",
					"Confirm PIN",
					"PIN OK",
					"Incorrect PIN!",
					"Time Out",
					"* retries left",
					"Insert Card",
					"Card Error",
					"PIN blocked" };

				const char *lang;
				const char **l10n;

#ifdef __APPLE__
				CFArrayRef cfa;
				CFStringRef slang;

				/* Get the complete ordered list */
				cfa = CFLocaleCopyPreferredLanguages();

				/* Use the first/preferred language
				 * As the driver is run as root we get the language
				 * selected during install */
				slang = CFArrayGetValueAtIndex(cfa, 0);

				/* CFString -> C string */
				lang = CFStringGetCStringPtr(slang, kCFStringEncodingMacRoman);
#else
				/* The other Unixes just use the LANG env variable */
				lang = getenv("LANG");
#endif
				DEBUG_COMM2("Using lang: %s", lang);
				if (NULL == lang)
					l10n = en;
				else
				{
					if (0 == strncmp(lang, "fr", 2))
						l10n = fr;
					else if (0 == strncmp(lang, "de", 2))
						l10n = de;
					else if (0 == strncmp(lang, "es", 2))
						l10n = es;
					else if (0 == strncmp(lang, "it", 2))
						l10n = it;
					else if (0 == strncmp(lang, "pt", 2))
						l10n = pt;
					else if (0 == strncmp(lang, "nl", 2))
						l10n = nl;
					else if (0 == strncmp(lang, "tr", 2))
						l10n = tr;
					else
						l10n = en;
				}

#ifdef __APPLE__
				/* Release the allocated array */
				CFRelease(cfa);
#endif
				offset = 0;
				cmd[offset++] = 0xB2;	/* load strings */
				cmd[offset++] = 0xA0;	/* address of the memory */
				cmd[offset++] = 0x00;	/* address of the first byte */
				cmd[offset++] = 0x4D;	/* magic value */
				cmd[offset++] = 0x4C;	/* magic value */

				/* for each string */
				for (i=0; i<L10N_NB_STRING; i++)
				{
					/* copy the string */
					for (j=0; l10n[i][j]; j++)
						cmd[offset++] = l10n[i][j];

					/* pad with " " */
					for (; j<L10N_STRING_MAX_SIZE; j++)
						cmd[offset++] = ' ';
				}

				(void)sleep(1);
				if (IFD_SUCCESS == CmdEscape(reader_index, cmd, sizeof(cmd), res, &length_res, DEFAULT_COM_READ_TIMEOUT))
				{
					DEBUG_COMM("l10n string loaded successfully");
				}
				else
				{
					DEBUG_COMM("Failed to load l10n strings");
					return_value = IFD_COMMUNICATION_ERROR;
				}

				if (DriverOptions & DRIVER_OPTION_DISABLE_PIN_RETRIES)
				{
					/* disable VERIFY from reader */
					const unsigned char cmd2[] = {0xb5, 0x00};
					length_res = sizeof(res);
					if (IFD_SUCCESS == CmdEscape(reader_index, cmd2, sizeof(cmd2), res, &length_res, DEFAULT_COM_READ_TIMEOUT))
					{
						DEBUG_COMM("Disable SPE retry counter successful");
					}
					else
					{
						DEBUG_CRITICAL("Failed to disable SPE retry counter");
					}
				}
			}
			break;

		case HPSMARTCARDKEYBOARD:
		case HP_CCIDSMARTCARDKEYBOARD:
		case FUJITSUSMARTKEYB:
		case CHICONYHPSKYLABKEYBOARD:
			/* the Secure Pin Entry is bogus so disable it
			 * https://web.archive.org/web/20120320001756/http://martinpaljak.net/2011/03/19/insecure-hp-usb-smart-card-keyboard/
			 *
			 * The problem is that the PIN code entered using the Secure
			 * Pin Entry function is also sent to the host.
			 */

		case C3PO_LTC31_v2:
			ccid_descriptor->bPINSupport = 0;
			break;

		case HID_AVIATOR:      /* OMNIKEY Generic */
		case HID_OMNIKEY_3X21: /* OMNIKEY 3121 or 3021 or 1021 */
		case HID_OMNIKEY_6121: /* OMNIKEY 6121 */
		case CHERRY_XX44:      /* Cherry Smart Terminal xx44 */
		case FUJITSU_D323:     /* Fujitsu Smartcard Reader D323 */
			/* The chip advertises pinpad but actually doesn't have one */
			ccid_descriptor->bPINSupport = 0;
			/* Firmware uses chaining */
			ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
			ccid_descriptor->dwFeatures |= CCID_CLASS_EXTENDED_APDU;
			break;

		case KOBIL_TRIBANK:
			/* Firmware does NOT supported extended APDU */
			ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
			ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
			break;

#if 0
		/* SCM SCR331-DI contactless */
		case SCR331DI:
		/* SCM SCR331-DI-NTTCOM contactless */
		case SCR331DINTTCOM:
		/* SCM SDI010 contactless */
		case SDI010:
			/* the contactless reader is in the second slot */
			if (ccid_descriptor->bCurrentSlotIndex > 0)
			{
				unsigned char cmd1[] = { 0x00 };
				/*  command: 00 ??
				 * response: 06 10 03 03 00 00 00 01 FE FF FF FE 01 ?? */
				unsigned char cmd2[] = { 0x02 };
				/*  command: 02 ??
				 * response: 00 ?? */

				unsigned char res[20];
				unsigned int length_res = sizeof(res);

				if ((IFD_SUCCESS == CmdEscape(reader_index, cmd1, sizeof(cmd1), res, &length_res, 0))
					&& (IFD_SUCCESS == CmdEscape(reader_index, cmd2, sizeof(cmd2), res, &length_res, 0)))
				{
					DEBUG_COMM("SCM SCR331-DI contactless detected");
				}
				else
				{
					DEBUG_COMM("SCM SCR331-DI contactless init failed");
				}

				/* hack since the contactless reader do not share dwFeatures */
				ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
				ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;

				ccid_descriptor->dwFeatures |= CCID_CLASS_AUTO_IFSD;
			}
			break;
#endif
		case CHERRY_KC1000SC:
			if ((0x0100 == ccid_descriptor->IFD_bcdDevice)
				&& (ccid_descriptor->dwFeatures & CCID_CLASS_EXCHANGE_MASK) == CCID_CLASS_SHORT_APDU)
			{
				/* firmware 1.00 is bogus
				 * With a T=1 card and case 2 APDU (data from card to
				 * host) the maximum size returned by the reader is 128
				 * byes. The reader is then using chaining as with
				 * extended APDU.
				 */
				ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
				ccid_descriptor->dwFeatures |= CCID_CLASS_EXTENDED_APDU;
			}
			break;

		case ElatecTWN4_CCID_CDC:
		case ElatecTWN4_CCID:
		case SCM_SCL011:
			/* restore default timeout (modified in ccid_open_hack_pre()) */
			ccid_descriptor->readTimeout = DEFAULT_COM_READ_TIMEOUT;
			break;

		case BIT4ID_MINILECTOR:
			/* The firmware 1.11 advertises pinpad but actually doesn't
			 * have one */
			ccid_descriptor->bPINSupport = 0;
			break;

		case ACS_ACR33U_A1_3SAM_ICC_READER:
		case ACS_ACR33U_A2_3SAM_ICC_READER:
		case ACS_ACR33U_A3_3SAM_ICC_READER:
		case ACS_ACR33U_4SAM_ICC_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 1)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR83U:
			{
				unsigned int firmwareVersion;
				char *msg = "     ACR 83";

				ccid_descriptor->wLcdLayout = 0x0210;

				DEBUG_INFO1("Getting ACR83U firmware version...");
				if (ACR83_GetFirmwareVersion(reader_index, &firmwareVersion, NULL))
				{
					DEBUG_INFO2("ACR83U firmware version: 0x%04X", firmwareVersion);
					if (firmwareVersion >= 0x4500)
					{
						// ACR83U uses short APDU exchange
						ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
						ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
					}
					else
					{
						// Disable PIN support (firmware version < 0x4500)
						ccid_descriptor->bPINSupport = 0;
					}
				}

				// Display default message
				(void)ACR83_DisplayLcdMessage(reader_index, msg);
			}
			break;

		case ACS_APG8201:
			{
				unsigned int version1 = 0;
				unsigned int version2 = 0;

				DEBUG_INFO1("Getting APG8201 firmware version...");
				if (ACR83_GetFirmwareVersion(reader_index, &version1,
					&version2))
				{
					DEBUG_INFO3("APG8201 firmware version: 0x%04X%04X",
						version1, version2);

					/* Fix incorrect message length for 008I. */
					if ((version1 == 0x3030) && (version2 == 0x3849))
					{
						ccid_descriptor->dwMaxCCIDMessageLength = 320;
					}
				}

				/*
				 * Fix APG8201 which cannot receive the command properly.
				 * Set write delay to 10 ms.
				 */
				ccid_descriptor->writeDelay = 10;
			}
		case ACS_APG8201Z:
			ccid_descriptor->wLcdLayout = 0x0210;

			// APG8201 uses short APDU exchange
			if (ccid_descriptor->dwFeatures & CCID_CLASS_TPDU)
			{
				ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
				ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
			}
			break;

		case ACS_APG8201Z2:
			{
				unsigned int version1 = 0;
				unsigned int version2 = 0;

				ccid_descriptor->wLcdLayout = 0x0210;

				DEBUG_INFO1("Getting APG8201Z2 firmware version...");
				if (ACR83_GetFirmwareVersion(reader_index, &version1,
					&version2))
				{
					DEBUG_INFO3("APG8201Z2 firmware version: 0x%04X%04X",
						version1, version2);

					/* Disable SetParameters for 029Z. */
					if ((version1 == 0x3032) && (version2 == 0x395A))
					{
						ccid_descriptor->dwFeatures &= ~CCID_CLASS_AUTO_PPS_CUR;
						ccid_descriptor->dwFeatures |= CCID_CLASS_AUTO_PPS_PROP;
					}
				}
			}
			break;

		case ACS_ACR85_PINPAD_READER_ICC:
			{
				unsigned int firmwareVersion;

				// ACR85 ICC uses short APDU exchange
				ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
				ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;

				DEBUG_INFO1("Getting ACR85 ICC firmware version...");
				if (ACR83_GetFirmwareVersion(reader_index, &firmwareVersion, NULL))
				{
					DEBUG_INFO2("ACR85 ICC firmware version: 0x%04X", firmwareVersion);

					// Set firmware fix enabled
					ccid_descriptor->firmwareFixEnabled = (firmwareVersion == 0x0001);
					DEBUG_INFO2("Firmware fix enabled: %d", ccid_descriptor->firmwareFixEnabled);
				}

				/*
				 * Fix ACR85 ICC which cannot receive the command properly.
				 * Set write delay to 10 ms.
				 */
				ccid_descriptor->writeDelay = 10;
			}
			break;

		case ACS_ACR1222_1SAM_PICC_READER:
			if (ccid_descriptor->bCurrentSlotIndex == 1)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR1222_DUAL_READER:
		case ACS_ACR1222_1SAM_DUAL_READER:
			{
				char firmwareVersion[30];
				unsigned int firmwareVersionLen = sizeof(firmwareVersion);

				DEBUG_INFO1("Getting ACR1222 firmware version...");
				if (ACR1222_GetFirmwareVersion(reader_index, firmwareVersion, &firmwareVersionLen))
				{
					DEBUG_INFO2("ACR1222 firmware version: %s", firmwareVersion);

					ccid_descriptor->firmwareFixEnabled = (strcmp(firmwareVersion, "ACR1222U_V401") == 0);
					DEBUG_INFO2("Firmware fix enabled: %d", ccid_descriptor->firmwareFixEnabled);

					if ((ccid_descriptor->firmwareFixEnabled) &&
						(ccid_descriptor->bCurrentSlotIndex == 1))
					{
						DEBUG_INFO1("Enabling PICC...");
						EnablePicc(reader_index, 1);
					}
				}

				if (ccid_descriptor->bCurrentSlotIndex == 2)
					ccid_descriptor->isSamSlot = 1;	// SAM
			}
			break;

		case ACS_ACR1222_3S_PICC_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 0)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR85_PINPAD_READER_PICC:
			{
				char firmwareVersion[30];
				unsigned int firmwareVersionLen = sizeof(firmwareVersion);

				DEBUG_INFO1("Getting ACR85 PICC firmware version...");
				if (ACR1222_GetFirmwareVersion(reader_index, firmwareVersion, &firmwareVersionLen))
				{
					DEBUG_INFO2("ACR85 PICC firmware version: %s", firmwareVersion);

					ccid_descriptor->firmwareFixEnabled = (strcmp(firmwareVersion, "ACR1222U_V402") == 0);
					DEBUG_INFO2("Firmware fix enabled: %d", ccid_descriptor->firmwareFixEnabled);

					if (ccid_descriptor->firmwareFixEnabled)
					{
						DEBUG_INFO1("Enabling PICC...");
						EnablePicc(reader_index, 1);
					}
				}
			}
			break;

		case ACS_ACR38U:
		case ACS_ACR38U_SAM:
		case IRIS_SCR21U:
		case ACS_AET65_1SAM_ICC_READER:
			if (ccid_descriptor->bCurrentSlotIndex == 0)
			{
				char firmwareVersion[11];	// Firmware version

				// Get firmware version
				memset(firmwareVersion, 0, sizeof(firmwareVersion));
				if (ACR38_GetFirmwareVersion(reader_index, firmwareVersion) == IFD_SUCCESS)
				{
					DEBUG_INFO2("Firmware: %s", firmwareVersion);

					if (ccid_descriptor->readerID == IRIS_SCR21U)
					{
						// Adjust maximum data rate
						if (strcmp(firmwareVersion, "ACR38-1042") == 0)
							ccid_descriptor->dwMaxDataRate = 43010;	// MCU
					}
				}
			}
			else
			{
				// Adjust maximum data rate
				ccid_descriptor->dwMaxDataRate = 10752;	// SAM
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR88U:
			// Adjust maximum data rate
			if (ccid_descriptor->bCurrentSlotIndex < 2)
				ccid_descriptor->dwMaxDataRate = 116129;	// MCU
			else
			{
				ccid_descriptor->dwMaxDataRate = 9677;		// SAM
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR89_ICC_READER:
		case ACS_ACR89_FP_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 1)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR89_DUAL_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 2)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR128U:
			// Adjust features and maximum data rate
			if (ccid_descriptor->bCurrentSlotIndex == 0)
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// MCU
				ccid_descriptor->dwMaxDataRate = 116129;
			}
			else if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				ccid_descriptor->dwFeatures = 0x0002047A;	// Contactless
				ccid_descriptor->dwMaxDataRate = 116129;
			}
			else
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 9677;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR1251_1S_DUAL_READER:
		case ACS_ACR1261_1S_DUAL_READER:
			// Adjust features and maximum data rate
			if (ccid_descriptor->bCurrentSlotIndex == 0)
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// MCU
				ccid_descriptor->dwMaxDataRate = 344100;
			}
			else if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				ccid_descriptor->dwFeatures = 0x0004047A;	// Contactless
				ccid_descriptor->dwMaxDataRate = 344100;
			}
			else
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 125000;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR1281_1S_DUAL_READER:
			// Adjust features and maximum data rate
			if (ccid_descriptor->bCurrentSlotIndex == 0)
			{
				char firmwareVersion[30];
				unsigned int firmwareVersionLen = sizeof(firmwareVersion);

				ccid_descriptor->dwFeatures = 0x000204BA;	// MCU
				ccid_descriptor->dwMaxDataRate = 344100;

				/* ACR1281U-C1 >= v526 supports ICC extended APDU. */
				DEBUG_INFO1("Getting ACR1281U-C1 firmware version...");
				if (ACR1222_GetFirmwareVersion(reader_index, firmwareVersion,
					&firmwareVersionLen))
				{
					int version = 0;

					DEBUG_INFO2("ACR1281U-C1 firmware version: %s",
						firmwareVersion);
					if (sscanf(firmwareVersion, "ACR1281U_V%d", &version) > 0)
					{
						if (version >= 526)
						{
							ccid_descriptor->dwFeatures = 0x000404BA;
						}
					}
				}
			}
			else if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				ccid_descriptor->dwFeatures = 0x0004047A;	// Contactless
				ccid_descriptor->dwMaxDataRate = 344100;
			}
			else
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 688172;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR1281_2S_CL_READER:
			// Adjust features and maximum data rate
			if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				ccid_descriptor->dwFeatures = 0x0004047A;	// Contactless
				ccid_descriptor->dwMaxDataRate = 344100;
				ccid_descriptor->isSamSlot = 0;
			}
			else
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 125000;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR1283_4S_CL_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 0)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR39U_SAM_ICC_READER:
			if (ccid_descriptor->bCurrentSlotIndex == 1)
				ccid_descriptor->isSamSlot = 1;	// SAM
			break;

		case ACS_ACR1281U_K_DUAL_READER:
		case ACS_ACR1281U_K_1S_DUAL_READER:
		case ACS_ACR1281U_K_4S_DUAL_READER:
			if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// ICC
				ccid_descriptor->dwMaxDataRate = 600000;
			}
			else if (ccid_descriptor->bCurrentSlotIndex > 1)
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 172043;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR123_3S_READER:
			if (ccid_descriptor->bCurrentSlotIndex > 0)
			{
				ccid_descriptor->dwFeatures = 0x000204BA;	// SAM
				ccid_descriptor->dwMaxDataRate = 125000;
				ccid_descriptor->isSamSlot = 1;
			}
			break;

		case ACS_ACR40T_ICC_READER:
			ccid_descriptor->isSamSlot = 1;
			break;

		default:
			break;
	}

	/* Gemalto readers may report additional information */
	if (GET_VENDOR(ccid_descriptor->readerID) == VENDOR_GEMALTO)
		set_gemalto_firmware_features(reader_index);

	return return_value;
} /* ccid_open_hack_post */

/*****************************************************************************
 *
 *					ccid_error
 *
 ****************************************************************************/
void ccid_error(int log_level, int error, const char *file, int line,
	const char *function)
{
#ifndef NO_LOG
	const char *text;
	char var_text[30];

	switch (error)
	{
		case 0x00:
			text = "Command not supported or not allowed";
			break;

		case 0x01:
			text = "Wrong command length";
			break;

		case 0x05:
			text = "Invalid slot number";
			break;

		case 0xA2:
			text = "Card short-circuiting. Card powered off";
			break;

		case 0xA3:
			text = "ATR too long (> 33)";
			break;

		case 0xAB:
			text = "No data exchanged";
			break;

		case 0xB0:
			text = "Reader in EMV mode and T=1 message too long";
			break;

		case 0xBB:
			text = "Protocol error in EMV mode";
			break;

		case 0xBD:
			text = "Card error during T=1 exchange";
			break;

		case 0xBE:
			text = "Wrong APDU command length";
			break;

		case 0xE0:
			text = "Slot busy";
			break;

		case 0xEF:
			text = "PIN cancelled";
			break;

		case 0xF0:
			text = "PIN timeout";
			break;

		case 0xF2:
			text = "Busy with autosequence";
			break;

		case 0xF3:
			text = "Deactivated protocol";
			break;

		case 0xF4:
			text = "Procedure byte conflict";
			break;

		case 0xF5:
			text = "Class not supported";
			break;

		case 0xF6:
			text = "Protocol not supported";
			break;

		case 0xF7:
			text = "Invalid ATR checksum byte (TCK)";
			break;

		case 0xF8:
			text = "Invalid ATR first byte";
			break;

		case 0xFB:
			text = "Hardware error";
			break;

		case 0xFC:
			text = "Overrun error";
			break;

		case 0xFD:
			text = "Parity error during exchange";
			break;

		case 0xFE:
			text = "Card absent or mute";
			break;

		case 0xFF:
			text = "Activity aborted by Host";
			break;

		default:
			if ((error >= 1) && (error <= 127))
				(void)snprintf(var_text, sizeof(var_text), "error on byte %d",
					error);
			else
				(void)snprintf(var_text, sizeof(var_text),
					"Unknown CCID error: 0x%02X", error);

			text = var_text;
			break;
	}
	log_msg(log_level, "%s:%d:%s %s", file, line, function, text);
#endif

} /* ccid_error */

void acr38_error(int error, const char *file, int line, const char *function)
{
#ifndef NO_LOG
	const char *text;
	char var_text[30];

	switch (error)
	{
		case 0x00:
			text = "Success";
			break;

		case 0xF4:
			text = "Procedure byte conflict";
			break;

		case 0xF6:
			text = "Bad length";
			break;

		case 0xF7:
			text = "Bad Fi/Di";
			break;

		case 0xF8:
			text = "Bad ATR TS";
			break;

		case 0xF9:
			text = "ICC not powered up";
			break;

		case 0xFA:
			text = "ICC not inserted";
			break;

		case 0xFB:
			text = "Hardware error";
			break;

		case 0xFC:
			text = "XFE overrun";
			break;

		case 0xFD:
			text = "XFE parity error";
			break;

		case 0xFE:
			text = "ICC mute";
			break;

		case 0xFF:
			text = "Command aborted";
			break;

		default:
			(void)snprintf(var_text, sizeof(var_text),
				"Unknown ACR38 error: 0x%02X", error);
			text = var_text;
			break;
	}

	log_msg(PCSC_LOG_ERROR, "%s:%d:%s %s", file, line, function, text);
#endif
}

// Enable PICC
void EnablePicc(unsigned int reader_index, int enabled)
{
	unsigned char pollingOff[] = { 0xE0, 0x00, 0x00, 0x20, 0x01, 0x7F };
	unsigned char pollingOn[]  = { 0xE0, 0x00, 0x00, 0x20, 0x01, 0xFF };

	unsigned char antennaOff[] = { 0xFF, 0x00, 0x00, 0x00, 0x04, 0xD4, 0x32, 0x01, 0x02 };
	unsigned char antennaOn[]  = { 0xFF, 0x00, 0x00, 0x00, 0x04, 0xD4, 0x32, 0x01, 0x03 };

	unsigned char response[300];
	unsigned int responseLen;

	if (enabled)
	{
		// Turn ON polling
		responseLen = sizeof(response);
		if (CmdEscape(reader_index, pollingOn, sizeof(pollingOn), response, &responseLen, 0) != IFD_SUCCESS)
		{
			DEBUG_CRITICAL("Polling ON failed");
		}

		// Turn ON antenna
		responseLen = sizeof(response);
		if (CmdXfrBlock(reader_index, sizeof(antennaOn), antennaOn, &responseLen, response, T_0) != IFD_SUCCESS)
		{
			DEBUG_CRITICAL("Antenna ON failed");
		}
	}
	else
	{
		// Turn OFF polling
		responseLen = sizeof(response);
		if (CmdEscape(reader_index, pollingOff, sizeof(pollingOff), response, &responseLen, 0) != IFD_SUCCESS)
		{
			DEBUG_CRITICAL("Polling OFF failed");
		}

		// Turn OFF antenna
		responseLen = sizeof(response);
		if (CmdXfrBlock(reader_index, sizeof(antennaOff), antennaOff, &responseLen, response, T_0) != IFD_SUCCESS)
		{
			DEBUG_CRITICAL("Antenna OFF failed");
		}
	}
}

static int ACR83_GetFirmwareVersion(unsigned int reader_index, unsigned int *pVersion1, unsigned int *pVersion2)
{
	int ret = 0;
	unsigned char command[] = { 0x04, 0x00, 0x00, 0x00, 0x00 };
	unsigned int commandLen = sizeof(command);
	unsigned char response[3 + 6];
	unsigned int responseLen = sizeof(response);

	if (CmdEscape(reader_index, command, commandLen, response, &responseLen, 0) == IFD_SUCCESS)
	{
		if ((responseLen >= 9) && (response[0] == 0x84))
		{
			if (pVersion1 != NULL)
			{
				*pVersion1 = (response[5] << 8) | response[6];
			}

			if (pVersion2 != NULL)
			{
				*pVersion2 = (response[7] << 8) | response[8];
			}

			ret = 1;
		}
		else if ((responseLen >= 6) && (response[0] == 0x84))
		{
			if (pVersion1 != NULL)
			{
				*pVersion1 = (response[2] << 8) | response[3];
			}

			if (pVersion2 != NULL)
			{
				*pVersion2 = (response[4] << 8) | response[5];
			}

			ret = 1;
		}
	}

	return ret;
}

static int ACR83_DisplayLcdMessage(unsigned int reader_index, const char *message)
{
	int ret = 0;
	unsigned char command[5 + 32] = { 0x05, 0x00, 0x20, 0x00, 0x00 };
	unsigned int commandLen = sizeof(command);
	unsigned char response[3 + 2];
	unsigned int responseLen = sizeof(response);
	int messageLen = strlen(message);

	if (messageLen > 32)
		messageLen = 32;

	// Fill memory with spaces
	memset(command + 5, 0x20, 32);

	// Copy message to command
	memcpy(command + 5, message, messageLen);

	if (CmdEscape(reader_index, command, commandLen, response, &responseLen, 0) == IFD_SUCCESS)
	{
		if ((responseLen >= 5) && (response[0] == 0x85) &&
			(response[3] == 0) && (response[4] == 0))
		{
			ret = 1;
		}
	}

	return ret;
}

static int ACR1222_GetFirmwareVersion(unsigned int reader_index, char *firmwareVersion, unsigned int *pFirmwareVersionLen)
{
	int ret = 0;
	unsigned char command[] = { 0xE0, 0x00, 0x00, 0x18, 0x00 };
	unsigned char response[300];
	unsigned int responseLen = sizeof(response);

	if (CmdEscape(reader_index, command, sizeof(command), response, &responseLen, 0) == IFD_SUCCESS)
	{
		if (*pFirmwareVersionLen >= responseLen - 5 + 1)
		{
			*pFirmwareVersionLen = responseLen - 5;
			memcpy(firmwareVersion, response + 5, *pFirmwareVersionLen);
			firmwareVersion[*pFirmwareVersionLen] = '\0';
			ret = 1;
		}
	}

	return ret;
}
