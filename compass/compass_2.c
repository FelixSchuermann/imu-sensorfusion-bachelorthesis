
#include <stdint.h>
#include "LSM9DS0.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int file;

void writeMagReg(uint8_t reg, uint8_t value);
void readBlock(uint8_t command, uint8_t size, uint8_t *data);
void readMAG(int * m);


#define magXmax 0
#define magYmax 0
#define magZmax 0
#define magXmin 0
#define magYmin 0
#define magZmin 0

/*
#define magXmax 8447
#define magYmax 2047
#define magZmax 16383
#define magXmin -10753
#define magYmin -17665
#define magZmin 0
*/



int main(int argc, char *argv[])

{
	char filename[20];
	int magRaw[3];
	float scaledMag[3];
	//i2c Bus öffnen
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
        	printf("Unable to open I2C bus!");
                exit(1);
	}


	//Magnetometer anwählen
	if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0) {
                printf("Error: Could not select magnetometer\n");
        }


	//Enable the magnetometer
/*	writeMagReg( CTRL_REG5_XM, 0b11110000);   // Temp enable, M data rate = 50Hz
	writeMagReg( CTRL_REG6_XM, 0b01100000);   // +/-12gauss
	writeMagReg( CTRL_REG7_XM, 0b00000000);   // Continuous-conversion mode
*/	
	writeMagReg(LSM303DLHC_CRA_REG_M, 0b10011000); //data rate = 75 HZ
	writeMagReg(LSM303DLHC_CRB_REG_M, 0b11100000); // ? gauss range
	writeMagReg(LSM303DLHC_MR__REG_M, 0b00000000); // continuous conversation

		int magRawMax[3];
		int magRawMin[3];
		int	avgs[3];


	while(1)
	{
		readMAG(magRaw);

		//Apply hard iron calibration
		magRaw[0]-= (magXmin + magXmax) /2 ;
		magRaw[2] -= (magYmin + magYmax) /2 ;
		magRaw[1] -= (magZmin + magZmax) /2 ;

		//Apply soft iron calibration
	//	scaledMag[0]  = (float)(magRaw[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
	//	scaledMag[2]  = (float)(magRaw[2] - magYmin) / (magYmax - magYmin) * 2 - 1;
	//	scaledMag[1]  = (float)(magRaw[1] - magZmin) / (magZmax - magZmin) * 2 - 1;


		//printf("magRaw X %i    \tmagRaw Y %i \tMagRaw Z %i \n", magRaw[0],magRaw[2],magRaw[1]);

		printf("SCALED ::magRaw X: %f    \t magRaw Y:  %f \tMagRaw Z:  %f \n", scaledMag[0],scaledMag[2],scaledMag[1]);

		//Only needed if the heading value does not increase when the magnetometer is rotated clockwise
		scaledMag[2] = -scaledMag[2];

		//Compute heading
	        float heading = 180 * atan2(scaledMag[2],scaledMag[0])/M_PI;


		//Convert heading to 0 - 360
        	if(heading < 0)
	  	      heading += 360;


		//Local declination in mrads into radians
		float declination = 217.9 / 1000.0;

		//Add the declination correction to our current heading
		heading += declination * 180/M_PI;


		//Correct the heading if declination forces it over 360
		if ( heading > 360)
			heading -= 360;



		printf("heading %7.3f \t ", heading);

		//Sleep for 0.25ms
		usleep(25000);

	}

}



void writeMagReg(uint8_t reg, uint8_t value)
{
  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        printf ("Failed to write byte to I2C Mag.");
        exit(1);
    }
}

void  readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    if (result != size)
    {
       printf("Failed to read block from I2C.");
        exit(1);
    }
}

void readMAG(int  *m)
{
        uint8_t block[6];

        readBlock(0x80 | LSM303DLHC_OUT_X_L_M, sizeof(block), block);

        *m = (int16_t)(block[1] | block[0]<<8);
        *(m+1) = (int16_t)(block[3] | block[2]<<8);
        *(m+2) = (int16_t)(block[5] | block[4]<<8);

}





