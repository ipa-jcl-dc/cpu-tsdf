#ifndef __WEBSERVER_
#define __WEBSERVER_
#include<string.h> //memset
#include<stdlib.h> //for exit(0);
#include<sys/socket.h>
#include<errno.h> //For errno - the error number
#include<netdb.h> //hostent
#include<arpa/inet.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <sstream>
#include <istream>
#include <fstream>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
struct webserver_signal {
    std::string scale_value;
    float motor_speed;
    float motor_position;
};

static char* host_name = "raspberrypi"; //name of web server
class WebServer
{
public:
    WebServer();
    ~WebServer();
    ////
    /// \brief hostnameToIP : scan IP address using webserver's name
    /// detail: http://www.binarytides.com/hostname-to-ip-address-c-sockets-linux/
    ///
    std::string hostnameToIP();

    ////create socket a connection between computer and station
    unsigned long createSocket();
    ////
    /// \brief pollStatus : poll for status of station
    /// \return struct contains signals of weight of object, motor speed and motor position
    ///
    webserver_signal pollStatus();
    ////
    /// \brief rotateDegRel : rotate relatively a degree
    /// \param degree : input degree
    ///
    void rotateDegRel(const double degree);
    ////
    /// \brief taraWeight Zeros the scale's weight.
    ///
    void taraWeight();
    ////Sets the maximum speed as (positive) float value in degree/second.
    //// Hardware limitation is around 14.5, default is 12
    void setAngularSpeed(const double speed);
    ////
    /// \brief getWeight : Get weight of an object
    /// \return weight in kilogam
    ///
    std::string getWeight();
    ////
    /// \brief resetMotorPos : reset motor position to zero
    ///
    void resetMotorPos();
    void testPollStatus();

private:
     boost::asio::ip::tcp::iostream socket_;
     std::string ip_address_;
     webserver_signal webserver_status_;

};


#endif //__WEBSERVER_
