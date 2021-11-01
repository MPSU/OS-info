#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "include/lsm9ds1_reg.h"

/*
 *
 * 	 https://github.com/STMicroelectronics/LSM9DS1/
 *
 *   Sample build command:
 *   gcc -Wall -o ./main.c ./include/lsm9ds1_reg.c ./main -lwiringPi
 *
 *                   LSM9DS1  |  RaspPi
 *
 *   (Physical: 6)	     VCC ->  3.3    (Physical: 17)
 *   (Physical: 5)   	 GND ->  GND    (Physical: 25)
 *   (Physical: 2)       MOSI->  MOSI   (Physical: 19)
 *   (Physical: 3)       MISO->  MISO   (Physical: 21)
 *   (Physical: 4)       SCK ->  SCLK   (Physical: 23)
 *   (Physical: 1)    CS_A/G ->  13     (wiringPi: 2)
 * 	 (Physical: 9)      CS_M ->  15     (wiringPi: 3)
 *
*/

#define SS_ag	2
#define SS_ma	3

#define CHANNEL			0
#define CLOCK_SPEED		625000

int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t platform_write_imu(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);
int32_t platform_write_mag(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);

int main(int argc, char *argv[])
{

	stmdev_ctx_t dev_ctx_imu;
	stmdev_ctx_t dev_ctx_mag;

	int16_t data_raw_acceleration[3];
	int16_t data_raw_angular_rate[3];
	int16_t data_raw_magnetic_field[3];
	float acceleration_mg[3];
	float angular_rate_mdps[3];
	float magnetic_field_mgauss[3];
	lsm9ds1_status_t reg;
	uint8_t rst;

	lsm9ds1_id_t who_am_I = {0};

	struct {
		int imu : 1;
		int mag : 1;
	} exit_flag;

	exit_flag.imu = 0;
	exit_flag.mag = 0;

	if(wiringPiSetup() < 0)
	{
		fprintf(stderr, "Error occured while GPIO setup.\n");
		exit(-1);
	}

	pinMode(SS_ag, OUTPUT);
	pinMode(SS_ma, OUTPUT);

	digitalWrite(SS_ag, HIGH);
	digitalWrite(SS_ma, HIGH);

	if (wiringPiSPISetupMode(CHANNEL, CLOCK_SPEED, 3) < 0)
	{
		fprintf(stderr, "Failed to init SPI communication.\n");
		exit(-1);
	}
	printf("SPI communication successfully setup.\n");

	delay(20);

	dev_ctx_imu.write_reg = platform_write_imu;
	dev_ctx_imu.read_reg = platform_read_imu;
	dev_ctx_mag.write_reg = platform_write_mag;
	dev_ctx_mag.read_reg = platform_read_mag;

	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &who_am_I);
	printf("AG ID: 0x%x, MAG ID: 0x%x\n", who_am_I.imu, who_am_I.mag);

	if (who_am_I.imu != LSM9DS1_IMU_ID || who_am_I.mag != LSM9DS1_MAG_ID)
	{
		fprintf(stderr, "IMU initialization error.\n");
		exit(-1);
	}
	printf("IMU successfully initialized.\n");

	/* Restore default configuration */
	lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

	do
	{
		lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
	/* Set full scale */
	lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
	lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
	/* Configure filtering chain - See datasheet for filtering chain details */
	/* Accelerometer filtering chain */
	lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
	lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
	/* Gyroscope filtering chain */
	lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
	lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);
	/* Set Output Data Rate / Power mode */
	lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_14Hz9);
	lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

	/* Read samples*/
	while (!exit_flag.imu || !exit_flag.mag) {

		/* Read device status register */
		lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

		if (reg.status_imu.xlda && reg.status_imu.gda && !exit_flag.imu)
		{
			/* Read imu data */
			memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
			memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
			lsm9ds1_acceleration_raw_get(&dev_ctx_imu,data_raw_acceleration);
			lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,data_raw_angular_rate);
			acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[0]);
			acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[1]);
			acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[2]);
			angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
			angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
			angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
			printf("IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\n",
				  acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
				  angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			exit_flag.imu ^= 1;
		}

		if (reg.status_mag.zyxda && !exit_flag.mag)
		{
			/* Read magnetometer data */
			memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
			lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
			magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[0]);
			magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[1]);
			magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field[2]);
			printf("MAG - [mG]:%4.2f\t%4.2f\t%4.2f\n",
				  magnetic_field_mgauss[0], magnetic_field_mgauss[1],
				  magnetic_field_mgauss[2]);
			exit_flag.mag ^= 1;
		}
	}
}

int32_t platform_read_imu(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	unsigned char address = 0x80 | (reg & 0x7F);

	unsigned char* local_buf = (unsigned char*)calloc(len + 1, sizeof(unsigned char));
	local_buf[0] = address;

	digitalWrite(SS_ag, LOW);
	wiringPiSPIDataRW(CHANNEL, (unsigned char*)local_buf, len + 1);
	digitalWrite(SS_ag, HIGH);

	strcpy((char*)buf, (const char*)(&local_buf[1]));

	free(local_buf);

	return 0;
}

int32_t platform_read_mag(void* handle, uint8_t reg, uint8_t* buf, uint16_t len)
{
	unsigned char address = 0x80 | (reg & 0x3F);

	if (len > 1)
	{
		address |= 0x40;
	}

	unsigned char* local_buf = (unsigned char*)calloc(len + 1, sizeof(unsigned char));
	local_buf[0] = address;

	digitalWrite(SS_ma, LOW);
	wiringPiSPIDataRW(CHANNEL, (unsigned char*)local_buf, len + 1);
	digitalWrite(SS_ma, HIGH);

	strcpy((char*)buf, (const char*)(&local_buf[1]));

	free(local_buf);

	return 0;
}

int32_t platform_write_imu(void* handle, uint8_t reg, const uint8_t* buf, uint16_t len)
{
	unsigned char address = reg & 0x7F;
	unsigned char* local_buf = (unsigned char*)calloc(len + 1, sizeof(unsigned char));

	local_buf[0] = address;
	strcpy((char*)(&local_buf[1]), (const char*)buf);

	digitalWrite(SS_ag, LOW);
	wiringPiSPIDataRW(CHANNEL, local_buf, len + 1);
	digitalWrite(SS_ag, HIGH);

	free(local_buf);

	return 0;
}

int32_t platform_write_mag(void* handle, uint8_t reg, const uint8_t* buf, uint16_t len)
{
	unsigned char address = reg & 0x3F;

	if (len > 1)
	{
		address |= 0x40;
	}

	unsigned char* local_buf = (unsigned char*)calloc(len + 1, sizeof(unsigned char));

	local_buf[0] = address;
	strcpy((char*)(&local_buf[1]), (const char*)buf);

	digitalWrite(SS_ma, LOW);
	wiringPiSPIDataRW(CHANNEL, local_buf, len + 1);
	digitalWrite(SS_ma, HIGH);

	free(local_buf);

	return 0;
}
