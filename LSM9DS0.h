#define MAG_ADDRESS            0x1e
#define ACC_ADDRESS            0x19
#define GYR_ADDRESS            0x6B



/** LSM9DS0 Gyro Registers **/
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G			0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G			0x2E
#define FIFO_SRC_REG_G			0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G			0x32
#define INT1_THS_XL_G			0x33
#define INT1_THS_YH_G			0x34
#define INT1_THS_YL_G			0x35
#define INT1_THS_ZH_G			0x36
#define INT1_THS_ZL_G			0x37
#define INT1_DURATION_G			0x38

//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM			0x05
#define OUT_TEMP_H_XM			0x06
#define STATUS_REG_M			0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M			0x12
#define INT_SRC_REG_M			0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M			0x16
#define OFFSET_X_H_M			0x17
#define OFFSET_Y_L_M			0x18
#define OFFSET_Y_H_M			0x19
#define OFFSET_Z_L_M			0x1A
#define OFFSET_Z_H_M			0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM			0x1F
#define CTRL_REG1_XM			0x20
#define CTRL_REG2_XM			0x21
#define CTRL_REG3_XM			0x22
#define CTRL_REG4_XM			0x23
#define CTRL_REG5_XM			0x24
#define CTRL_REG6_XM			0x25
#define CTRL_REG7_XM			0x26
#define STATUS_REG_A			0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG			0x2E
#define FIFO_SRC_REG			0x2F
#define INT_GEN_1_REG			0x30
#define INT_GEN_1_SRC			0x31
#define INT_GEN_1_THS			0x32
#define INT_GEN_1_DURATION		0x33
#define INT_GEN_2_REG			0x34
#define INT_GEN_2_SRC			0x35
#define INT_GEN_2_THS			0x36
#define INT_GEN_2_DURATION		0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY			0x3C
#define TIME_WINDOW			0x3D


/*
//  LSM303DLHC Compass Register Map

#define LSM303DLHC_CRA_REG_M        0x00
#define LSM303DLHC_CRB_REG_M        0x01
#define LSM303DLHC_MR__REG_M        0x02
#define LSM303DLHC_OUT_X_H_M        0x03
#define LSM303DLHC_OUT_X_L_M        0x04
#define LSM303DLHC_OUT_Y_H_M        0x05
#define LSM303DLHC_OUT_Y_L_M        0x06
#define LSM303DLHC_OUT_Z_H_M        0x07
#define LSM303DLHC_OUT_Z_L_M        0x08
#define LSM303DLHC_STATUS_M         0x09
#define LSM303DLHC_TEMP_OUT_L_M     0x31
#define LSM303DLHC_TEMP_OUT_H_M     0x32


//  Compass Minimum  Data Output Rate ( LSM303DLHC_CRA_REG_M )
// temp sensor enabled
//				[Hz]
#define LSM303DLHC_COMPASS_MDOR_0_75      0b10000000
#define LSM303DLHC_COMPASS_MDOR_1_5       0b10000100
#define LSM303DLHC_COMPASS_MDOR_3         0b10001000
#define LSM303DLHC_COMPASS_MDOR_7_5       0b10001100
#define LSM303DLHC_COMPASS_MDOR_15        0b10010000
#define LSM303DLHC_COMPASS_MDOR_30        0b10010100		//take this
#define LSM303DLHC_COMPASS_MDOR_75        0b10011000
#define LSM303DLHC_COMPASS_MDOR_220       0b10011100

//  Compass Gain setting    (Sensor input field		gain xyz	gain z
// ( LSM303DLHC_CRB_REG_M)	range [Gauss]	)	[LSB/Gauss]	[LSB/Gauss]

#define LSM303DLHC_COMPASS_GS_1_3      	0b00100000	//1100		980
#define LSM303DLHC_COMPASS_GS_1_9      	0b01000000	//855		760
#define LSM303DLHC_COMPASS_GS_2_5      	0b01100000	//670		600
#define LSM303DLHC_COMPASS_GS_4        	0b10000000	//450		400
#define LSM303DLHC_COMPASS_GS_4_7      	0b10100000	//400		355
#define LSM303DLHC_COMPASS_GS_5_6      	0b11000000	//330		295
#define LSM303DLHC_COMPASS_GS_8_1      	0b11100000	//230		205	//take this

//  Compass Operating mode ( LSM303DLHC_MR__REG_M )
#define LSM303DLHC_COMPASS_OP_CC      	0b00000000	//continous conversion mode	//take this
#define LSM303DLHC_COMPASS_OP_SC      	0b00000001	//single conversion mode
#define LSM303DLHC_COMPASS_OP_SM      	0b00000010	//sleep mode
#define LSM303DLHC_COMPASS_OP_SM1      	0b00000011	//sleep mode
*/
