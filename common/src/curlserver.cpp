#include "curlserver.hpp"

Webserver2::Webserver2()
{
    curl_ =  curl_easy_init();
}

Webserver2::~Webserver2()
{
    curl_easy_cleanup(curl_);
}

webserver_signal Webserver2::pollStatus()
{

      curl_easy_setopt(curl_, CURLOPT_URL, "http://raspberrypi:8080/?");
      curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, WriteCallback);
      curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
      res_ = curl_easy_perform(curl_);
      //std::cout << read_buffer_ << std::endl;
      //Create json file
      std::ofstream json_file("json_output.json");
      if (!json_file){
          std::cerr << "Error opening file\n";
      }
      //writing data to json file
      json_file<<read_buffer_;
      json_file.close();
      //reading data from json file
      boost::property_tree::ptree pt;
      boost::property_tree::read_json("json_output.json",pt);
      webserver_status_.motor_position = pt.get<float>("MotorPosition",0);
      webserver_status_.motor_speed = pt.get<float>("MotorSpeed",0);
      webserver_status_.scale_value = pt.get<std::string>("ScaleValue");
      read_buffer_.clear();
      return webserver_status_;
}

webserver_signal Webserver2::rotateDegRel(const float degree)
{
    std::ostringstream strs;
    strs << degree;
    std::string degree_str = strs.str();
    std::string command = "http://raspberrypi:8080/?RotateDegRel=" + degree_str;
    const char* command_char = command.c_str();
    curl_easy_setopt(curl_, CURLOPT_URL,command_char);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    //std::cout << read_buffer_ << std::endl;
    //Create json file
    std::ofstream json_file("json_output.json");
    if (!json_file){
        std::cerr << "Error opening file\n";
    }
    //writing data to json file
    json_file<<read_buffer_;
    json_file.close();
    //reading data from json file
    boost::property_tree::ptree pt;
    boost::property_tree::read_json("json_output.json",pt);
    webserver_status_.motor_position = pt.get<float>("MotorPosition",0);
    webserver_status_.motor_speed = pt.get<float>("MotorSpeed",0);
    webserver_status_.scale_value = pt.get<std::string>("ScaleValue");
    read_buffer_.clear();
    return webserver_status_;
}
std::string Webserver2::getWeight()
{
    return pollStatus().scale_value;
}
void Webserver2::setAngularSpeed(const float speed)
{
    std::ostringstream strs;
    strs << speed;
    std::string speed_str = strs.str();
    std::string command = "http://raspberrypi:8080/?AngularSpeed=" + speed_str;
    const char* command_char = command.c_str();
    curl_easy_setopt(curl_, CURLOPT_URL,command_char);
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    read_buffer_.clear();
}

webserver_signal Webserver2::taraWeight()
{
    curl_easy_setopt(curl_, CURLOPT_URL,"http://raspberrypi:8080/?Tara=1");
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);

    //Create json file
    std::ofstream json_file("json_output.json");
    if (!json_file){
        std::cerr << "Error opening file\n";
    }
    //writing data to json file
    json_file<<read_buffer_;
    json_file.close();
    //reading data from json file
    boost::property_tree::ptree pt;
    boost::property_tree::read_json("json_output.json",pt);
    webserver_status_.motor_position = pt.get<float>("MotorPosition",0);
    webserver_status_.motor_speed = pt.get<float>("MotorSpeed",0);
    webserver_status_.scale_value = pt.get<std::string>("ScaleValue");
    read_buffer_.clear();
    return webserver_status_;
}

webserver_signal Webserver2::resetMotorPos()
{
    curl_easy_setopt(curl_, CURLOPT_URL,"http://raspberrypi:8080/?ResetMotorPosition=1");
    curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION,WriteCallback);
    curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &read_buffer_);
    res_ = curl_easy_perform(curl_);
    //Create json file
    std::ofstream json_file("json_output.json");
    if (!json_file){
        std::cerr << "Error opening file\n";
    }
    //writing data to json file
    json_file<<read_buffer_;
    json_file.close();
    //reading data from json file
    boost::property_tree::ptree pt;
    boost::property_tree::read_json("json_output.json",pt);
    webserver_status_.motor_position = pt.get<float>("MotorPosition",0);
    webserver_status_.motor_speed = pt.get<float>("MotorSpeed",0);
    webserver_status_.scale_value = pt.get<std::string>("ScaleValue");
    read_buffer_.clear();
    return webserver_status_;
}
