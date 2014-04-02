/*
    ccid.c: CCID common code
    Copyright (C) 2003-2005   Ludovic Rousseau

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

/*
 * $Id: ccid.c 4346 2009-07-28 13:39:37Z rousseau $
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pcsclite.h>
#include <ifdhandler.h>

#include "config.h"
#include "debug.h"
#include "ccid.h"
#include "defs.h"
#include "ccid_ifdhandler.h"
#include "commands.h"
#include "ccid_usb.h"

static int GetFirmwareVersion(unsigned int reader_index);

/*****************************************************************************
 *
 *					ccid_open_hack_pre
 *
 ****************************************************************************/
int ccid_open_hack_pre(unsigned int reader_index)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
	int doInterruptRead = 1;
	int i;

	switch (ccid_descriptor->readerID)
	{
		case CARDMAN3121+1:
			/* Reader announces APDU but is in fact TPDU */
			ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
			ccid_descriptor->dwFeatures |= CCID_CLASS_TPDU;
			break;

		case MYSMARTPAD:
			ccid_descriptor->dwMaxIFSD = 254;
			break;

		case CL1356D:
			/* the firmware needs some time to initialize */
			(void)sleep(1);
			ccid_descriptor->readTimeout = 60; /* 60 seconds */
			break;

		case KOBIL_TRIBANK:
			/* the InterruptRead does not timeout (on Mac OS X) */
			doInterruptRead = 0;
			break;

		default:
			// Disable InterruptRead for all ACS CCID readers
			// ACR88U and ACR128U does not timeout on Mac OS X
			doInterruptRead = 0;

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
	}

	/* CCID */
	if (doInterruptRead && (0 == ccid_descriptor->bInterfaceProtocol))
	{
#ifndef TWIN_SERIAL
		/* just wait for 10ms in case a notification is in the pipe */
		(void)InterruptRead(reader_index, 10);
#endif
	}

	/* ICCD type A */
	if (ICCD_A == ccid_descriptor->bInterfaceProtocol)
	{
		unsigned char tmp[MAX_ATR_SIZE];
		unsigned int n = sizeof(tmp);

		DEBUG_COMM("ICCD type A");
		(void)CmdPowerOff(reader_index);
		(void)CmdPowerOn(reader_index, &n, tmp, CCID_CLASS_AUTO_VOLTAGE);
		(void)CmdPowerOff(reader_index);
	}

	/* ICCD type B */
	if (ICCD_B == ccid_descriptor->bInterfaceProtocol)
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
		(void)CmdPowerOn(reader_index, &n, tmp, CCID_CLASS_AUTO_VOLTAGE);
		(void)CmdPowerOff(reader_index);
	}

	return 0;
} /* ccid_open_hack_pre */

/*****************************************************************************
 *
 *					ccid_open_hack_post
 *
 ****************************************************************************/
int ccid_open_hack_post(unsigned int reader_index)
{
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);

	switch (ccid_descriptor->readerID)
	{
		case GEMPCKEY:
		case GEMPCTWIN:
			/* Reader announces TPDU but can do APDU */
			if (DriverOptions & DRIVER_OPTION_GEMPC_TWIN_KEY_APDU)
			{
				unsigned char cmd[] = { 0xA0, 0x02 };
				unsigned char res[10];
				unsigned int length_res = sizeof(res);

				if (CmdEscape(reader_index, cmd, sizeof(cmd), res, &length_res) == IFD_SUCCESS)
				{
					ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
					ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
				}
			}
			break;

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

				char *lang;
				const char **l10n;

				lang = getenv("LANG");
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
				if (IFD_SUCCESS == CmdEscape(reader_index, cmd, sizeof(cmd), res, &length_res))
				{
					DEBUG_COMM("l10n string loaded successfully");
				}
				else
				{
					DEBUG_COMM("Failed to load l10n strings");
				}
			}
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

				if ((IFD_SUCCESS == CmdEscape(reader_index, cmd1, sizeof(cmd1), res, &length_res))
					&& (IFD_SUCCESS == CmdEscape(reader_index, cmd2, sizeof(cmd2), res, &length_res)))
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

		case ACS_ACR83U:
			{
				unsigned char command[] = { 0x04, 0x00, 0x00, 0x00, 0x00 };
				unsigned int commandLen = sizeof(command);
				unsigned char response[3 + 6];
				unsigned int responseLen = sizeof(response);
				unsigned int firmwareVersion;

				ccid_descriptor->wLcdLayout = 0x0210;
				if (CmdEscape(reader_index, command, commandLen, response, &responseLen) == IFD_SUCCESS)
				{
					if ((responseLen >= 7) && (response[0] == 0x84))
					{
						firmwareVersion = (response[5] << 8) | response[6];
						DEBUG_INFO2("ACR83U firmware version: 0x%04X", firmwareVersion);
						if (firmwareVersion >= 0x4500)
						{
							ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
							ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
						}
						else
						{
							// Disable PIN support (firmware version < 0x4500)
							ccid_descriptor->bPINSupport = 0;
						}
					}
				}
			}
			break;

		case ACS_APG8201:
			ccid_descriptor->wLcdLayout = 0x0210;
		case ACS_ACR85_PINPAD_READER_ICC:
			// APG8201 uses short APDU exchange
			ccid_descriptor->dwFeatures &= ~CCID_CLASS_EXCHANGE_MASK;
			ccid_descriptor->dwFeatures |= CCID_CLASS_SHORT_APDU;
			break;

		// Enable PICC
		case ACS_ACR1222_DUAL_READER:
		case ACS_ACR1222_1SAM_DUAL_READER:
			DEBUG_INFO("Getting firmware version...");
			ccid_descriptor->firmwareVersion = GetFirmwareVersion(reader_index);
			DEBUG_INFO2("ACR1222: %d", ccid_descriptor->firmwareVersion);
			if (ccid_descriptor->bCurrentSlotIndex == 1)
			{
				DEBUG_INFO("Enabling PICC...");
				EnablePicc(reader_index, 1);
			}
			break;
		case ACS_ACR85_PINPAD_READER_PICC:
			DEBUG_INFO("Enabling PICC...");
			EnablePicc(reader_index, 1);
			break;

		default:
			break;
	}

	return 0;
} /* ccid_open_hack_post */

/*****************************************************************************
 *
 *					ccid_error
 *
 ****************************************************************************/
void ccid_error(int error, const char *file, int line, const char *function)
{
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
			{
				(void)snprintf(var_text, sizeof(var_text), "error on byte %d",
					error);
				text = var_text;
			}
			else
				(void)snprintf(var_text, sizeof(var_text),
					"Unknown CCID error: 0x%02X", error);
				text = var_text;
			break;
	}
	log_msg(PCSC_LOG_ERROR, "%s:%d:%s %s", file, line, function, text);

} /* ccid_error */

// Enable PICC
void EnablePicc(unsigned int reader_index, int enabled)
{
	unsigned char pollingOff[] = { 0xE0, 0x00, 0x00, 0x20, 0x01, 0x7F };
	unsigned char pollingOn[]  = { 0xE0, 0x00, 0x00, 0x20, 0x01, 0xFF };

	unsigned char antennaOff[] = { 0xFF, 0x00, 0x00, 0x00, 0x04, 0xD4, 0x32, 0x01, 0x02 };
	unsigned char antennaOn[]  = { 0xFF, 0x00, 0x00, 0x00, 0x04, 0xD4, 0x32, 0x01, 0x03 };

	unsigned char response[300];
	int responseLen;

	if (enabled)
	{
		// Turn ON polling
		responseLen = sizeof(response);
		if (CmdEscape(reader_index, pollingOn, sizeof(pollingOn), response, &responseLen) != IFD_SUCCESS)
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
		if (CmdEscape(reader_index, pollingOff, sizeof(pollingOff), response, &responseLen) != IFD_SUCCESS)
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

// Get firmware version (ACR1222)
static int GetFirmwareVersion(unsigned int reader_index)
{
	int firmwareVersion = -1;

	unsigned char getFirmwareVersion[] = { 0xE0, 0x00, 0x00, 0x18, 0x00 };
	unsigned char response[300];
	int responseLen;

	responseLen = sizeof(response);
	if (CmdEscape(reader_index, getFirmwareVersion, sizeof(getFirmwareVersion), response, &responseLen) == IFD_SUCCESS)
	{
		if (responseLen > 5)
			sscanf((char *) (response + 5), "ACR1222U_V%d", &firmwareVersion);
	}

	return firmwareVersion;
}
