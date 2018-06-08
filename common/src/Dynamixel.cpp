#include "Dynamixel.h"
#include "dynamixel.h"
#include <cmath>
Dynamixel::Dynamixel()
{
	m_Initialized = false;
	m_Open = false;
}

Dynamixel::~Dynamixel()
{
	Close();
}

unsigned long Dynamixel::Close()
{
	if(m_Open)
	{
		// close device
		dxl_terminate();
	}

	m_Open = false;
	return RET_OK;
}

unsigned long Dynamixel::Init()
{
	m_Initialized = true;

	return RET_OK;
}

unsigned long Dynamixel::Open()
{
	if (!m_Initialized)
	{
        return RET_FAILED;
	}

	if (dxl_initialize(DYNAMIXEL_DEFAULT_PORTNUM, DYNAMIXEL_DEFAULT_BAUDNUM) == 0) 
    {
		std::cerr << "Dynamixel::Open: Failed in opening Dynamixel device" << std::endl;
		return RET_FAILED;
	}
        //set torque value
   // dxl_write_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_TORQUE_L,255);
   // dxl_write_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_TORQUE_H,0);
	// set rotation mode to multi-turn
	dxl_write_word(DYNAMIXEL_DEFAULT_ID, 6, 4095);
	if(dxl_get_result() != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Open: Failed in setting rotation mode of Dynamixel device" << std::endl;
        return RET_FAILED;
	}
	dxl_write_word(DYNAMIXEL_DEFAULT_ID, 8, 4095);
	if(dxl_get_result() != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Open: Failed in setting rotation mode of Dynamixel device" << std::endl;
        return RET_FAILED;
	}

	// set P gain
	dxl_write_word(DYNAMIXEL_DEFAULT_ID, 28, 64);
	if(dxl_get_result() != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Open: Failed in setting P gain of Dynamixel device" << std::endl;
        return RET_FAILED;
	}

	// set rotation speed
    dxl_write_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_ROTATION_SPEED_L, 16);
    dxl_write_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_ROTATION_SPEED_H, 0);

	if(dxl_get_result() != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Open: Failed in setting rotation speed of Dynamixel device" << std::endl;
        return RET_FAILED;
	}
	
    // set acceleration
    dxl_write_word(DYNAMIXEL_DEFAULT_ID, 73, 254);
    int commStatus = dxl_get_result();
    printCommStatus(commStatus);
    if(dxl_get_result() != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Open: Failed in setting maximum acceleration of Dynamixel device" << std::endl;
        return RET_FAILED;
	}

	return RET_OK;
}

unsigned long Dynamixel::DoHoming()
{
    if (Rotate(0) & RET_FAILED)
	{
		std::cerr << "Dynamixel::DoHoming: Could not home Dynamixel module" << std::endl;
        return RET_FAILED;
	}

	return RET_OK;
}


unsigned long Dynamixel::Rotate(double goalPositionDeg)
{
    int targetPosition = -(int)(DYNAMIXEL_360_DEGREE * goalPositionDeg/360.0) % DYNAMIXEL_360_DEGREE;
    std::cout << "Dynamixel::Rotate: Rotating to " << targetPosition << std::endl;

	// rotate
	dxl_write_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_GOAL_POSITION_L, targetPosition);
	int commStatus = dxl_get_result();
	if(commStatus != COMM_RXSUCCESS)
	{
		std::cerr << "Dynamixel::Rotate: Failed in moving Dynamixel device" << std::endl;
		printCommStatus(commStatus);
        return RET_FAILED;
	}
	int moving = 1;
	do
	{
		// Check moving done
		moving = dxl_read_byte(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_MOVING);
		commStatus = dxl_get_result();
		if(commStatus == COMM_RXSUCCESS)
		{
			printErrorCode();
		}
		else
		{
			printCommStatus(commStatus);
			break;
		}
			
	} while(moving == 1);
	
	// check whether target position reached
	int presentPosition = dxl_read_word(DYNAMIXEL_DEFAULT_ID, DYNAMIXEL_PRESENT_POSITION_L);
	commStatus = dxl_get_result();
	if(commStatus == COMM_RXSUCCESS)
	{
		printErrorCode();
		if (abs(presentPosition - targetPosition) < 3)
            return RET_OK;
	}
	else
	{
		printCommStatus(commStatus);
        return RET_FAILED;
	}

    return RET_OK;
}

// Print communication result
void Dynamixel::printCommStatus(int commStatus)
{
	switch(commStatus)
	{
	case COMM_TXFAIL:
		std::cerr << "COMM_TXFAIL: Failed transmit instruction packet!\n";
		break;

	case COMM_TXERROR:
		std::cerr << "COMM_TXERROR: Incorrect instruction packet!\n";
		break;

	case COMM_RXFAIL:
		std::cerr << "COMM_RXFAIL: Failed get status packet from device!\n";
		break;

	case COMM_RXWAITING:
		std::cerr << "COMM_RXWAITING: Now recieving status packet!\n";
		break;

	case COMM_RXTIMEOUT:
		std::cerr << "COMM_RXTIMEOUT: There is no status packet!\n";
		break;

	case COMM_RXCORRUPT:
		std::cerr << "COMM_RXCORRUPT: Incorrect status packet!\n";
		break;

	default:
		std::cerr << "This is unknown error code!\n";
		break;
	}
}

// print error bit of status packet if there is any
void Dynamixel::printErrorCode()
{
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		std::cerr << "Input voltage error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		std::cerr << "Angle limit error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		std::cerr << "Overheat error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		std::cerr << "Out of range error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		std::cerr << "Checksum error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		std::cerr << "Overload error!\n";

	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		std::cerr << "Instruction code error!\n";
}


