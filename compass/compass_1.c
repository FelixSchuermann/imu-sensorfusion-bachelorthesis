
#include <stdint.h>
#include "LSM9DS0.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "adafruit9DoF.h"
/*
#define magXmax 0
#define magYmax 0
#define magZmax 0
#define magXmin 0
#define magYmin 0
#define magZmin 0
*/
#define magXmax 8702
#define magYmax 2303
#define magZmax 20480
#define magXmin -11266
#define magYmin -17665
#define magZmin 0


int file;

void writeMagReg(uint8_t reg, uint8_t value);
void writeAccReg(uint8_t reg, uint8_t value);
void readBlock(uint8_t command, uint8_t size, uint8_t *data);
void readMAG(int * m);
void readACC(int * a);




int main(int argc, char *argv[])

{
	char filename[20];
	int magRaw[3];
	int accRaw[3];
	float accXnorm,accYnorm,pitch,roll,magXcomp,magYcomp;

		float scaledMag[3];



	//Open the i2c bus
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
        	printf("Unable to open I2C bus!");
                exit(1);
	}


	//Select the magnetomoter
	if (ioctl(file, I2C_SLAVE, MAG_ADDRESS) < 0) {
                printf("Error: Could not select magnetometer\n");
        }



	// Enable accelerometer.
        writeAccReg(CTRL_REG1_XM, 0b01100111); //  z,y,x axis enabled, continuos update,  100Hz data rate
        writeAccReg(CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

        //Magnetmeter
    	writeMagReg(LSM303DLHC_CRA_REG_M, 0b00010000); //data rate = 75 HZ
    	writeMagReg(LSM303DLHC_CRB_REG_M, 0b11100000); // ? gauss range
    	writeMagReg(LSM303DLHC_MR__REG_M, 0b00000000); // continuous conversation



	while(1)
	{

/////////////////////////////////////OFFSETKORREKTUR///////////////////////////////////


		readMAG(magRaw);

				//Apply hard iron calibration
				magRaw[0]-= (magXmin + magXmax) /2 ;
				magRaw[2] -= (magYmin + magYmax) /2 ;
				magRaw[1] -= (magZmin + magZmax) /2 ;

				//Apply soft iron calibration
				scaledMag[0]  = (float)(magRaw[0] - magXmin) / (magXmax - magXmin) * 2 - 1;
				scaledMag[2]  = (float)(magRaw[2] - magYmin) / (magYmax - magYmin) * 2 - 1;
				scaledMag[1]  = (float)(magRaw[1] - magZmin) / (magZmax - magZmin) * 2 - 1;


				printf("magRaw X %i    \tmagRaw Y %i \tMagRaw Z %i\t", magRaw[0],magRaw[2],magRaw[1]);

			//	printf("magRaw X: %i    \t magRaw Y:  %i \tMagRaw Z:  %i \n", magRaw[0],magRaw[2],magRaw[1]);

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



			//	printf("heading %7.3f \t ", heading);

				//Sleep for 0.25ms
				usleep(25000);

//////////////////////////////////////////////////OFFSETKORREKTUR////////////////////////////



		//readMAG(magRaw);
		readACC(accRaw);

		//If your IMU is upside down, comment out the two lines below which we correct the tilt calculation
		//accRaw[0] = -accRaw[0];
		//accRaw[1] = -accRaw[1];

		//Compute heading

	        heading = 180 * atan2(scaledMag[2],scaledMag[0])/M_PI;

		//Convert heading to 0 - 360
        	if(heading < 0)
	  	      heading += 360;

		printf("heading %7.3f \t ", heading);

		//Normalize accelerometer raw values.
                accXnorm = accRaw[0]/sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);
                accYnorm = accRaw[1]/sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2]);

		//Calculate pitch and roll
		pitch = asin(accXnorm);
		roll = -asin(accYnorm/cos(pitch));

		/*printf("pitch:\t" ,pitch);
		printf("roll: \t", roll);
*/
		//printf("ACC ", accXnorm);

		//Calculate the new tilt compensated values
		magXcomp = scaledMag[0]*cos(pitch)+scaledMag[2]*sin(pitch);
		magYcomp = scaledMag[0]*sin(roll)*sin(pitch)+scaledMag[2]*cos(roll)-scaledMag[1]*sin(roll)*cos(pitch);


		//Calculate heading
		heading = 180*atan2(magYcomp,magXcomp)/M_PI;

                //Convert heading to 0 - 360
		if(heading < 0)
		      heading += 360;


		printf("Compensated  Heading %7.3f  \n", heading);


		//Sleep for 25ms
		//usleep(25000);

	}

}

void selectDevice(int file, int addr)
{
        if (ioctl(file, I2C_SLAVE, addr) < 0) {
		 printf("Failed to select I2C device.");
        }
}


void writeAccReg(uint8_t reg, uint8_t value)
{
  selectDevice(file,ACC_ADDRESS);

  int result = i2c_smbus_write_byte_data(file, reg, value);
    if (result == -1)
    {
        printf ("Failed to write byte to I2C Mag.");
        exit(1);
    }
}



void writeMagReg(uint8_t reg, uint8_t value)
{
  selectDevice(file,MAG_ADDRESS);
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


void readACC(int  *a)
{
        uint8_t block[6];
        selectDevice(file,ACC_ADDRESS);
                readBlock(0x80 | OUT_X_L_A, sizeof(block), block);

        *a = (int16_t)(block[0] | block[1] << 8);
        *(a+1) = (int16_t)(block[2] | block[3] << 8);
        *(a+2) = (int16_t)(block[4] | block[5] << 8);

}

void readMAG(int  *m)
{
        uint8_t block[6];
        selectDevice(file,MAG_ADDRESS);
        readBlock(0x80 | LSM303DLHC_OUT_X_L_M, sizeof(block), block);

        *m = (int16_t)(block[1] | block[0] << 8);
        *(m+1) = (int16_t)(block[3] | block[2] << 8);
        *(m+2) = (int16_t)(block[5] | block[4] << 8);

}


