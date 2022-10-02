/*******************************************************************************
 * Copyright (c) 2018 Dmitrii Kaleev (kaleev@org.miet.ru)                      *
 *                                                                             *
 * The MIT License (MIT):                                                      *
 * Permission is hereby granted, free of charge, to any person obtaining a     *
 * copy of this software and associated documentation files (the "Software"),  *
 * to deal in the Software without restriction, including without limitation   *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell   *
 * copies of the Software, and to permit persons to whom the Software is       *
 * furnished to do so, subject to the following conditions:                    *
 * The above copyright notice and this permission notice shall be included     *
 * in all copies or substantial portions of the Software.                      *
 *                                                                             *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,             *
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR       *
 * OTHER DEALINGS IN THE SOFTWARE.                                             *
 ******************************************************************************/

#include <unistd.h>
#include "MFRC522.h"
#include <string.h>

void delay(int ms)
{
	usleep(ms * 1000);
}

MFRC522::MIFARE_Key key;
byte blockcontent[] = { "Simple text" };
byte readbackblock[18];

// Write specific block
int writeBlock(MFRC522 mfrc522, int blockNumber, byte arrayAddress[])
{
	for (byte i = 0; i < 6; i++) {
		key.keyByte[i] =
			0xFF; // keyByte is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
	}
	// this makes sure that we only write into data blocks. Every 4th block is a trailer block for the access/security info.
	int largestModulo4Number = blockNumber / 4 * 4;
	int trailerBlock = largestModulo4Number +
			   3; // determine trailer block for the sector
	if (blockNumber > 2 && (blockNumber + 1) % 4 == 0) {
		printf("%d is a trailer block:", blockNumber);
		return 2;
	}
	printf("%d", blockNumber);
	printf(" is a data block:\n");

	// authentication of the desired block for access
	byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
					       trailerBlock, &key,
					       &(mfrc522.uid));

	if (status != MFRC522::STATUS_OK) {
		printf("PCD_Authenticate() failed: %d\n",
		       mfrc522.GetStatusCodeName(status));
		return 3; // return "3" as error message
	}

	// writing the block
	status = mfrc522.MIFARE_Write(blockNumber, arrayAddress, 16);
	if (status != MFRC522::STATUS_OK) {
		printf("MIFARE_Write() failed: %d\n",
		       mfrc522.GetStatusCodeName(status));
		return 4; // return "4" as error message
	}
	printf("block was written\n");
	return 0;
}

// Read specific block
int readBlock(MFRC522 mfrc522, int blockNumber, byte arrayAddress[])
{
	for (byte i = 0; i < 6; i++) {
		key.keyByte[i] =
			0xFF; // keyByte is defined in the "MIFARE_Key" 'struct' definition in the .h file of the library
	}

	int largestModulo4Number = blockNumber / 4 * 4;
	int trailerBlock = largestModulo4Number +
			   3; // determine trailer block for the sector

	// authentication of the desired block for access
	byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A,
					       trailerBlock, &key,
					       &(mfrc522.uid));

	if (status != MFRC522::STATUS_OK) {
		printf("PCD_Authenticate() failed (read): %d\n",
		       mfrc522.GetStatusCodeName(status));
		return 3; // return "3" as error message
	}

	// reading a block
	byte buffersize =
		18; // we need to define a variable with the read buffer size, since the MIFARE_Read method below needs a pointer to the variable that contains the size...
	status = mfrc522.MIFARE_Read(
		blockNumber, arrayAddress,
		&buffersize); //&buffersize is a pointer to the buffersize variable; MIFARE_Read requires a pointer instead of just a number
	if (status != MFRC522::STATUS_OK) {
		printf("MIFARE_Read() failed: %d\n",
		       mfrc522.GetStatusCodeName(status));
		return 4; // return "4" as error message
	}
	printf("block was read\n");
	return 0;
}

void PrintUID(MFRC522 mfrc)
{
	for (byte i = 0; i < mfrc.uid.size; ++i) {
		if (mfrc.uid.uidByte[i] < 0x10) {
			printf(" 0");
			printf("%X", mfrc.uid.uidByte[i]);
		} else {
			printf(" ");
			printf("%X", mfrc.uid.uidByte[i]);
		}
	}
	printf("\n");
}

void help()
{
	printf("    Use this application for work with RFID module\n");
	printf("    execute format: ./main [-h][-q]\n");
	printf("    -h - help\n");
	printf("    -q - quiet mode, return card ID\n");
}

int main(int argc, char *argv[])
{
	int quiet = 0;
	if (argc > 1) {
		if ((strcmp(argv[1], "-h") == 0)) {
			help();
			return 0;
		} else {
			if ((strcmp(argv[1], "-q") == 0)) {
				quiet = 1;
			} else {
				help();
				return 0;
			}
		}
	}

	MFRC522 mfrc;
	mfrc.PCD_Init();
	int var, nob;

	while (1) {
		if (!mfrc.PICC_IsNewCardPresent())
			continue;
		if (!mfrc.PICC_ReadCardSerial())
			continue;

		if (quiet) {
			printf("read ok!");
			usleep(1000000);
		} else {
			printf("Enter your choice:\n");
			printf("\t1: Print UID\n");
			printf("\t2: Write information\n");
			printf("\t3: Read information\n");
			scanf("%d", &var);
			switch (var) {
			case 1: {
				printf("UID: ");
				PrintUID(mfrc);
				break;
			}
			case 2: {
				printf("Enter number of block:\n");
				scanf("%d", &nob);
				printf("Enter text:\n");
				scanf("%s", blockcontent);
				writeBlock(mfrc, nob, blockcontent);
				return 0;
			}
			case 3: {
				printf("Enter number of block:\n");
				scanf("%d", &nob);
				readBlock(mfrc, nob, readbackblock);
				printf("String = %s\n", readbackblock);
				return 0;
			}
			default:
				printf("\t3: Error number\n");
			}
		}
	}
	return 0;
}
