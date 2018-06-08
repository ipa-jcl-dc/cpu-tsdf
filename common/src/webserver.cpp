#include "webserver.hpp"
using namespace std;
WebServer::WebServer()
{
    ip_address_ = hostnameToIP();
}
WebServer::~WebServer()
{
}
std::string WebServer::hostnameToIP()
{
   // const char* host_name = "raspberrypi"; //name of web server
    char ip_address[20];
    //Description of data base entry for a single host.
    struct hostent *he;
    // Internet address struct
    struct in_addr **addr_list;
    int i;

    if ( (he = gethostbyname( host_name ) ) == NULL)
        {
            // get the host info
            herror("gethostbyname is failed, no web server is connected or server name is incorrect");
            throw std::runtime_error("WebServer::hostnameToIP failed, check host_name in webserver.hpp file");
        }
        addr_list = (struct in_addr **) he->h_addr_list;
    for(i = 0; addr_list[i] != NULL; i++)
         {
             //Return the first one;
             strcpy(ip_address , inet_ntoa(*addr_list[i]) );
          }
    std::cout<< "Detect web server with ip address "<< ip_address<<std::endl;
    return std::string(ip_address);
}

unsigned long WebServer::createSocket()
{

    socket_.expires_from_now(boost::posix_time::seconds(60));
    socket_.connect(ip_address_,"8080");
    if (!socket_)
      {
        std::cout << "Unable to connect: " << socket_.error().message() << "\n";
            return EXIT_FAILURE;
    }
}
webserver_signal WebServer::pollStatus()
{
    createSocket();
    socket_<< "GET /?\r\n";
    socket_<< "Connection: close\r\n\r\n";
    std::string header;
    while (std::getline(socket_, header) && header != "\r")
            cout<<header<<endl;
    //Create json file
    std::ofstream json_file("json_output.json");
    if (!json_file){
        std::cerr << "Error opening file\n";
    }
    //writing data to json file
    json_file<<header;
    json_file.close();
    //reading data from json file
    boost::property_tree::ptree pt;
    boost::property_tree::read_json("json_output.json",pt);
    webserver_status_.motor_position = pt.get<float>("MotorPosition",0);
    webserver_status_.motor_speed = pt.get<float>("MotorSpeed",0);
    webserver_status_.scale_value = pt.get<std::string>("ScaleValue");
    return webserver_status_;
}
void WebServer::testPollStatus()
{
    createSocket();
    socket_<< "GET /?\r\n";
    socket_<< "Connection: close\r\n\r\n";
    std::string header;
   // while (std::getline(socket_, header) && header != "\r")
           // cout<<header<<endl;
}

void WebServer::rotateDegRel(const double degree)
{
    createSocket();
    std::ostringstream strs;
    strs << degree;
    std::string degree_str = strs.str();

    socket_<< "GET /?RotateDegRel="+degree_str+"\r\n";
    socket_<< "Connection: close\r\n\r\n";
}

void WebServer::taraWeight()
{
    createSocket();
    socket_<< "GET /?Tara=1\r\n";
    socket_<< "Connection: close\r\n\r\n";
}

std::string WebServer::getWeight()
{
    return pollStatus().scale_value;
}

void WebServer::setAngularSpeed(const double speed)
{
    createSocket();
    std::ostringstream strs;
    strs << speed;
    std::string speed_str = strs.str();
    socket_<< "GET /?AngularSpeed="+speed_str+"\r\n";
    socket_<< "Connection: close\r\n\r\n";
}
void WebServer::resetMotorPos()
{
    createSocket();
    socket_<< "GET /?ResetMotorPosition=1\r\n";
    socket_<< "Connection: close\r\n\r\n";
}
