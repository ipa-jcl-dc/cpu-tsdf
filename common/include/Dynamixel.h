#ifndef __DYNAMIXEL_H__
#define __DYNAMIXEL_H__
#include <iostream>
using namespace std;
#define RET_OK 1
#define RET_FAILED 2
// Default settings
static int DYNAMIXEL_DEFAULT_PORTNUM =		0; // deafult COM port
static int DYNAMIXEL_DEFAULT_BAUDNUM =		1; // 1Mbps
static int DYNAMIXEL_DEFAULT_ID =			1;

// Control table address
static int DYNAMIXEL_GOAL_POSITION_L =		30;
static int DYNAMIXEL_GOAL_POSITION_H =		31;
static int DYNAMIXEL_ROTATION_SPEED_L =		32;
static int DYNAMIXEL_ROTATION_SPEED_H =		33;
static int DYNAMIXEL_PRESENT_POSITION_L	=	36;
static int DYNAMIXEL_PRESENT_POSITION_H	=	37;
static int DYNAMIXEL_MOVING	=				46;
static int DYNAMIXEL_TORQUE_L =             14;
static int DYNAMIXEL_TORQUE_H =             15;

// Constants
static int DYNAMIXEL_360_DEGREE =	4096;	// equivalent for 360 deg rotation (12bit = 4096)

/// Basic point struct/class to represent 3 double components.
class Dynamixel
{
public:
	Dynamixel();
	~Dynamixel();

	unsigned long Init();
	unsigned long Open();
	unsigned long Close();

	unsigned long DoHoming();
	unsigned long Rotate(double goalPositionDeg);
private:

	void printCommStatus(int commStatus);
	void printErrorCode();

	bool m_Initialized;
	bool m_Open;
};





#endif // __DYNAMIXEL_H__
