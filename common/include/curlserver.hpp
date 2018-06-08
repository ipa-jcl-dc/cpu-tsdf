#ifndef __CURLSERVER_
#define __CURLSERVER_
#include <curl/curl.h>
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/json_parser.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <istream>
#include <string>

struct webserver_signal {
    std::string scale_value;
    float motor_speed;
    float motor_position;
};
class Webserver2{
public:
    Webserver2();
    ~Webserver2();
    ////
    /// \brief WriteCallback : callback function to get response from webserver using cURL
    ///
    static size_t WriteCallback(void *contents, size_t size, size_t nmemb, void *userp)
    {
        ((std::string*)userp)->append((char*)contents, size * nmemb);
            return size * nmemb;
    }
    ////
    /// \brief pollStatus : check status of motor(position, weight, speep)
    /// \return signal struct
    ///
    webserver_signal pollStatus();
    ////
    /// \brief rotateDegRel : Rotate motor with degree input
    /// \param degree : degree input
    /// \return signal struct
    ///
    webserver_signal rotateDegRel(const float degree);
    ////
    /// \brief getWeight : Get weight of object
    /// \return  : weight in string (0.123 kg)
    ///
    std::string getWeight();
    ////
    /// \brief taraWeight Zeros the scale's weight.
    /// \return return signal struct
    ///
    webserver_signal taraWeight();
    ////
    /// \brief resetMotorPos Reset motor position to 0
    /// \return return signal struct
    ///
    webserver_signal resetMotorPos();
    ////
    /// \brief setAngularSpeed : Set angular speed of motor(degree/second), max 14.5, default 12.0
    /// \param speed: Input speed
    ///
    void setAngularSpeed(const float speed);

private:
    //cURL object allow to send, receive reponse from webserver
    CURL *curl_;
    //Check error
    CURLcode res_;
    //Status of motor received from webser
    webserver_signal webserver_status_;
    // buffer to store reply from webserver
    std::string read_buffer_;

};




#endif //__CURLSERVER_
