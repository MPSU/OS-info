#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "MFRC522.h"

#define MAX_LEN 16

#define NRSTPD 13

#define DEV "/dev/spidev1.1"
extern int debug;

static uint32_t speed = 1000000;

int GPIO_setup(unsigned int pin, int value);
int openSPI(const char *device, uint32_t speed);
void spi_transfer(int fd, unsigned char *data, unsigned int length);

static int MFRC_fd = -1;

/*
 * NAME: blockaddr2sectorblock
 * PURPOSE: To split a linear block address into its sector and block numbers
 * ARGUMENTS: blockaddr: linear block address (0..0xff)
 * RETURNS: address of static buffer holding sector#/block#
 * NOTE: The first 32 sectors (0..31) have 4 blocks/sector
 *	The last 8 sectors (32..39) have 16 blocks/sector.
 *	This function is used for debugging only!
 */
static char *blockaddr2sectorblock(unsigned int blockaddr)
{
	static char sector_block[2 + 1 + 2 + 1];

	if (blockaddr < (32 * 4))
		snprintf(sector_block, sizeof(sector_block), "%u/%u",
			 blockaddr / 4, blockaddr % 4);
	else if (blockaddr < (32 * 4 + (39 - 32 + 1) * 16)) {
		blockaddr -= 32 * 4;
		snprintf(sector_block, sizeof(sector_block), "%u/%u",
			 32 + (blockaddr / 16), blockaddr % 16);
	} else
		snprintf(sector_block, sizeof(sector_block), "n/a");

	return sector_block;
}

/*
 * NAME: sectorblock2blockaddr
 * PURPOSE: To convert a sector # and a block # to a linear block address
 * ARGUMENTS: sector: sector # (0..39)
 *	block: block number within sector (0..3 or 0..15)
 * RETURNS: linear block address or -1 in case of error
 * NOTE: The first 32 sectors (0..31) have 4 blocks/sector
 *	The last 8 sectors (32..39) have 16 blocks/sector.
 */
int sectorblock2blockaddr(unsigned char sector, unsigned char block)
{
	int blockaddr;

	if (debug)
		fprintf(stderr,
			"sectorblock2blockaddr(sector=%#x, block=%#x)\n",
			sector, block);

	if (sector < 32) {
		if (block < 4)
			blockaddr = sector * 4 + block;
		else
			blockaddr = -1;
	} else if (sector < 40) {
		if (block < 16)
			blockaddr = 32 * 4 + (sector - 32) * 16 + block;
		else
			blockaddr = -1;
	}

	if (debug)
		fprintf(stderr, "    = %d\n", blockaddr);

	return blockaddr;
}

/*
 * NAME: Write_MFRC522_register
 * PURPOSE: To write a value into an MFRC522 register
 * ARGUMENTS: addr: register address
 *	val: value to write
 * RETURNS: Nothing
 */
static int Write_MFRC522_register(unsigned int addr, unsigned char val)
{
	unsigned char b[2];

	b[0] = (addr << 1) & 0x7E;
	b[1] = val;

	spi_transfer(MFRC_fd, b, 2);
}

/*
 * NAME: Read_MFRC522_register
 * PURPOSE: To read a register of the MFRC522
 * ARGUMENTS: addr: register address
 * RETURNS: register value
 */
static unsigned char Read_MFRC522_register(unsigned int addr)
{
	unsigned char b[2];

	b[0] = ((addr << 1) & 0x7E) | 0x80;
	b[1] = 0;

	spi_transfer(MFRC_fd, b, 2);

	return b[1];
}

/*
 * NAME: MFRC522_Reset
 * PURPOSE: To reset the reader
 * ARGUMENTS: None
 * RETURNS: Nothing
 */
static int MFRC522_Reset()
{
	Write_MFRC522_register(CommandReg, PCD_RESETPHASE);
}

/*
 * NAME: AntennaOn
 * PURPOSE: To switch the antenna on
 * ARGUMENTS: None
 * RETURNS: Nothing
 */
static void AntennaOn()
{
	unsigned char temp;

	temp = Read_MFRC522_register(TxControlReg);
	if ((temp & 0x03) != 0x03)
		Write_MFRC522_register(TxControlReg, temp | 0x03);
}

/*
 * NAME: ClearBitMask
 * PURPOSE: To clear bits in a register
 * ARGUMENTS: reg: register number
 *	mask: bits to set
 * RETURNS: Nothing
 */
static void ClearBitMask(unsigned int reg, unsigned char mask)
{
	unsigned char tmp;

	tmp = Read_MFRC522_register(reg);
	Write_MFRC522_register(reg, tmp & (~mask));
}

/*
 * NAME: SetBitMask
 * PURPOSE: To set bits in a register
 * ARGUMENTS: reg: register number
 *	mask: bits to set
 * RETURNS: Nothing
 */
static void SetBitMask(unsigned int reg, unsigned char mask)
{
	unsigned char tmp;

	tmp = Read_MFRC522_register(reg);
	Write_MFRC522_register(reg, tmp | mask);
}

/*
 * backLenp: pointer to unsigneed int where the length (in BITS!) is stored.
 * NAME: MFRC522_ToCard
 * PURPOSE: To send data (a command or the payload) to the card
 *	and receive the response
 * ARGUMENTS: command: command code (eg PCD_AUTHENT, PCD_TRANSCEIVE)
 *	sendData, nbytes: address and size of data to send
 *	backDatap: address of pointer where to store address of received data
 *	backLenp: address of unsigned int where to store length of received response IN BITS!
 * RETURNS: MI_XXX status
 */
static int MFRC522_ToCard(unsigned char command, unsigned char *sendData,
			  unsigned int nbytes, unsigned char **backDatap,
			  unsigned int *backLenp)
{
	unsigned char irqEn = 0x00;
	unsigned int waitIRq = 0x00;
	int status = MI_ERR;
	unsigned char *backData = NULL;
	unsigned int backLen = 0;
	int i;
	unsigned char n, lastBits;

	if (command == PCD_AUTHENT) {
		irqEn = 0x12;
		waitIRq = 0x10;
	} else if (command == PCD_TRANSCEIVE) {
		irqEn = 0x77;
		waitIRq = 0x30;
	}

	Write_MFRC522_register(CommIEnReg, irqEn | 0x80);
	ClearBitMask(CommIrqReg, 0x80);
	SetBitMask(FIFOLevelReg, 0x80);

	Write_MFRC522_register(CommandReg, PCD_IDLE);

	for (i = 0; i < nbytes; i++)
		Write_MFRC522_register(FIFODataReg, sendData[i]);

	Write_MFRC522_register(CommandReg, command);
	if (command == PCD_TRANSCEIVE)
		SetBitMask(BitFramingReg, 0x80);

	for (i = 2000;;) {
		n = Read_MFRC522_register(CommIrqReg);
		i--;
		if (!((i != 0) && !(n & 0x01) && !(n & waitIRq)))
			break;
	}

	ClearBitMask(BitFramingReg, 0x80);

	if (i != 0) {
		if ((Read_MFRC522_register(ErrorReg) & 0x1B) == 0x00) {
			status = MI_OK;
			if ((n & irqEn) & 0x01)
				status = MI_NOTAGERR;
			if (command == PCD_TRANSCEIVE) {
				n = Read_MFRC522_register(FIFOLevelReg);
				lastBits = Read_MFRC522_register(ControlReg) &
					   0x07;
				if (lastBits != 0)
					backLen = (n - 1) * 8 + lastBits;
				else
					backLen = n * 8;
				if (n == 0)
					n = 1;
				else if (n > MAX_LEN)
					n = MAX_LEN;

				backData = calloc(n, sizeof(backData[0]));
				for (i = 0; i < n; i++)
					backData[i] = Read_MFRC522_register(
						FIFODataReg);
			}
		} else
			status = MI_ERR;
	}
	if (backDatap != NULL)
		*backDatap = backData;
	if (backLenp != NULL)
		*backLenp = backLen;

	// return (status, backData, backLen)
	return status;
}

static void CalulateCRC(unsigned char *pIndata, size_t buflen,
			unsigned char *pOutData)
{
	int i;

	ClearBitMask(DivIrqReg, 0x04);
	SetBitMask(FIFOLevelReg, 0x80);
	for (i = 0; i < buflen; i++)
		Write_MFRC522_register(FIFODataReg, pIndata[i]);
	Write_MFRC522_register(CommandReg, PCD_CALCCRC);
	i = 0xff;
	while (1) {
		unsigned char n;

		n = Read_MFRC522_register(DivIrqReg);
		i--;
		if ((i == 0) || (n & 0x04))
			break;
	}

	pOutData[0] = Read_MFRC522_register(CRCResultRegL);
	pOutData[1] = Read_MFRC522_register(CRCResultRegM);
}

/*
 * EXPORTED
 */
int MFRC522_Init(unsigned int ce)
{
	char device[128];

	GPIO_setup(NRSTPD, 1);

	snprintf(device, sizeof(device), DEV, ce);

	MFRC_fd = openSPI(device, speed);

	if (MFRC_fd == -1)
		return 0;

	MFRC522_Reset();

	Write_MFRC522_register(TModeReg, 0x8D);
	Write_MFRC522_register(TPrescalerReg, 0x3E);
	Write_MFRC522_register(TReloadRegL, 30);
	Write_MFRC522_register(TReloadRegH, 0);

	Write_MFRC522_register(TxAutoReg, 0x40);
	Write_MFRC522_register(ModeReg, 0x3D);

	AntennaOn();

	return 1;
}

/*
 * EXPORTED
 */
int MFRC522_Anticoll(unsigned char **backDatap)
{
	unsigned char cmd[2];
	unsigned char *backData;
	unsigned int backLen;
	int status;

	Write_MFRC522_register(BitFramingReg, 0x00);

	/*
     * Assemble command
     */
	cmd[0] = PICC_ANTICOLL; /* 0x93 */
	cmd[1] = 0x20; /* 2 bytes (cmd and this byte) and 0 extra bits */

	/*
     * Send command and receive response
     */
	status = MFRC522_ToCard(PCD_TRANSCEIVE, cmd, sizeof(cmd), &backData,
				&backLen);

	if (status == MI_OK) {
		int i;

		if (debug)
			fprintf(stderr, "    backLen=%#x\n", backLen);
		if (((backLen + 7) / 8) == 5) {
			unsigned char serNumCheck = 0;

			for (i = 0; i < 5; i++)
				serNumCheck ^= backData[i];

			if (serNumCheck != 0)
				status = MI_ERR;
		} else
			status = MI_ERR;
	}

	*backDatap = backData;

	return status;
}

/*
 * EXPORTED
 */
int MFRC522_Request(unsigned char reqMode, unsigned char *backBitsp)
{
	int status = 0;
	unsigned char TagType[] = { reqMode };
	unsigned char *backData;
	unsigned int backBits;

	Write_MFRC522_register(BitFramingReg, 0x07);

	// TagType.append(reqMode);
	// python MFRC522_ToCard returns (status, backData, backLen)
	// (status, backData, backBits) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, TagType)
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, &backData,
				&backBits);

	if ((status != MI_OK) | (backBits != 0x10))
		status = MI_ERR;

	if (backBitsp != NULL)
		*backBitsp = backBits;

	return status;
}

static void my_dump(void *buf, size_t buflen)
{
	char *sep;
	unsigned char *dp = (unsigned char *)buf;

	for (sep = ""; buflen > 0; sep = ",", buflen--)
		fprintf(stderr, "%s%02x", sep, *dp++);

	return;
}

/*
 * EXPORTED
 */
int MFRC522_SelectTag(unsigned char *serNum)
{
	unsigned char buf[1 + 1 + 5 + 2];
	int i, status;
	unsigned char *backData;
	unsigned int backLen;

	if (debug) {
		fprintf(stderr, "MFRC522_SelectTag(");
		my_dump(serNum, 5);
		fprintf(stderr, ")\n");
	}

	/* Assemble command */
	buf[0] = PICC_SELECTTAG; /* command code */
	buf[1] =
		0x70; /* 7 bytes (command code, this byte and serNum) and 0 bits */
	memcpy(&buf[2], serNum, 5);
	CalulateCRC(buf, sizeof(buf) - 2, &buf[2 + 5]);

	status = MFRC522_ToCard(PCD_TRANSCEIVE, buf, sizeof(buf), &backData,
				&backLen);

	if ((status == MI_OK) && (backLen == 0x18)) {
		/*
	 * MF1S50yyX/V1 returns 08h
	 * MF1S70yyX/V1 returns 18h
	 */
		if (debug)
			fprintf(stderr, "SAK response: %#x\n", backData[0]);
		return backData[0];
	} else
		return 0;
}

/*
 * def MFRC522_Auth(self, authMode, BlockAddr, Sectorkey, serNum):
 */
int MFRC522_Auth(unsigned char authMode, int BlockAddr,
		 unsigned char *Sectorkey, size_t SectorkeyLen,
		 unsigned char *serNum)
{
	unsigned char *buff;
	size_t buffLen;
	unsigned char *backData;
	unsigned int backLen;
	int status;

	if (debug)
		fprintf(stderr,
			"MFRC522_Auth(authMode=%#x, BlockAddr=%#x, ...)\n",
			authMode, BlockAddr);

	buffLen = 2 + SectorkeyLen + 4;
	buff = alloca(buffLen);

	buff[0] = authMode; // First byte should be the authMode (A or B)
	buff[1] = BlockAddr; // Second byte is the trailerBlock (usually 7)

	memcpy(&buff[2], Sectorkey, SectorkeyLen);

	memcpy(&buff[2 + SectorkeyLen], serNum, 4);

	if (debug) {
		fprintf(stderr, "    buff=[");
		my_dump(buff, buffLen);
		fprintf(stderr, "]\n");
	}

	// Now we start the authentication itself
	status =
		MFRC522_ToCard(PCD_AUTHENT, buff, buffLen, &backData, &backLen);

	// Check if an error occurred
	if (status != MI_OK)
		fprintf(stderr, "AUTH ERROR!!\n");
	if ((Read_MFRC522_register(Status2Reg) & 0x08) == 0)
		fprintf(stderr, "AUTH ERROR(status2reg & 0x08) != 0\n");

	// Return the status
	return status;
}

/*
 * def MFRC522_StopCrypto1(self):
 */
void MFRC522_StopCrypto1()
{
	ClearBitMask(Status2Reg, 0x08);
}

/*
 * def MFRC522_Read(self, blockAddr):
 */
void MFRC522_Read(unsigned char blockAddr)
{
	unsigned char recvData[] = {
		PICC_READ, blockAddr, 0x00, 0x00 // CRC
	};
	int status;
	unsigned char *backData;
	unsigned int backLen;

	fprintf(stderr, "MFRC522_Read(%u)\n", blockAddr);
	CalulateCRC(recvData, 2, &recvData[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, sizeof(recvData),
				&backData, &backLen);

	/* if (status != MI_OK)
        fprintf(stderr, "Error while reading!\n");
    else
        fprintf(stderr, "backLen=%#x\n", backLen);
*/
	if (backLen == (16 + 2) * 8) /* 16 bytes data, 2 bytes CRC */
	{
		int i;

		fprintf(stderr, "Sector %s", blockaddr2sectorblock(blockAddr));
		for (i = 0; i < 16; i++)
			fprintf(stderr, " %02x", backData[i]);
		fprintf(stderr, " - %02x %02x", backData[16], backData[17]);

		fprintf(stderr, "\n");
	}

	return;
}

/*
 * def MFRC522_Write(self, blockAddr, writeData):
 */
void MFRC522_Write(unsigned char blockAddr, unsigned char *writeData,
		   size_t writeDataLen)
{
	unsigned char buff[] = { PICC_WRITE, blockAddr, 0x00, 0x00 };
	int status;
	unsigned char *backData;
	unsigned int backLen;

	CalulateCRC(buff, 2, &buff[2]);
	status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, sizeof(buff), &backData,
				&backLen);
	if ((status != MI_OK) || (backLen != 4) ||
	    ((backData[0] & 0x0f) != 0x0A))
		status = MI_ERR;
	fprintf(stderr, "%u backData &0x0F == 0x0A %02x\n", backLen,
		backData[0] & 0x0F);
	//if (status == MI_OK) {
		int i;
		unsigned char *buf;
		//int status;

		buf = alloca(writeDataLen + 2); // 2 extra for CRC

		memcpy(buf, writeData, writeDataLen);
		CalulateCRC(buf, writeDataLen, &buf[writeDataLen]);

		status = MFRC522_ToCard(PCD_TRANSCEIVE, buf, writeDataLen + 2,
					&backData, &backLen);
/*
		if ((status != MI_OK) || (backLen != 4) ||
		    ((backData[0] & 0x0F) != 0x0A))
			fprintf(stderr, "Error while writing\n");
		if (status == MI_OK)
			fprintf(stderr, "Data written\n");*/
	//}

	return;
}
