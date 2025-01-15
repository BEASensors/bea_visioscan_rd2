// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <iostream>
#include <boost/asio.hpp>
#include "tcp_command_interface.h"
namespace bea_power {


TcpCommandInterface::TcpCommandInterface(const std::string hostname, const int tcp_port) : inbuf_(4096), instream_(&inbuf_)
{
    sys_cmd_realtime_ = SYS_CMD_REALTIME_NULL;
    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
    tcp_socket_ = 0;
//CONNECTING_TO_TCP:
    std::cout << "Connecting to TCP command channel at " << hostname << ":" << tcp_port << " ... "<<std::endl;
    try
    {
        // Resolve hostname/ip
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(hostname, std::to_string(tcp_port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::resolver::iterator end;

        tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
        boost::system::error_code error = boost::asio::error::host_not_found;

        // Iterate over endpoints and etablish connection
        while(error && endpoint_iterator != end)
        {
            tcp_socket_->close();
            tcp_socket_->connect(*endpoint_iterator++, error);
        }
        if(error)
        {
            throw boost::system::system_error(error);
        }

        // Start async reading
        tcp_socket_->async_receive(boost::asio::buffer(&tcp_buffer_[0],tcp_buffer_.size()),\
                boost::bind(&TcpCommandInterface::HandleTcpSocketRead, this,\
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

TcpCommandInterface::~TcpCommandInterface()
{
    disconnect();
    delete tcp_socket_;
}

void TcpCommandInterface::HandleTcpSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    int _size = (int)bytes_transferred;
    printf("@HandleTcpSocketRead bytes_transferred = %ld\n", bytes_transferred);
    printf("@tcp_buffer_ size = %d\n", int(tcp_buffer_.size()));
    char *src = &tcp_buffer_[0];
    printf("@HandleTcpSocketRead sys_cmd_realtime_ = %d\n", sys_cmd_realtime_);
    if(_size > 64) _size = 64;
    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_ANGULAR_RESOLUTION)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_RESOL;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: " << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if(position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr, ' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 3)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_OK;
                    printf("param : %d\n", std::stoi(_strVec[2]));
                    int _icode = std::stoi(_strVec[2]);
                    if(_icode == PARAM_ANGULAR_RESOLUTION_0)
                    {
                        parameterInfo_.angularResolution = 0.2;
                        parameterInfo_.freqHZ = 80;
                        parameterInfo_.scan_time = 1 / 80.0;
                    }
                    else if(_icode == PARAM_ANGULAR_RESOLUTION_1)
                    {
                        parameterInfo_.angularResolution = 0.1;
                        parameterInfo_.freqHZ = 40;
                        parameterInfo_.scan_time = 1 / 40.0;
                    }
                    else if(_icode == PARAM_ANGULAR_RESOLUTION_2)
                    {
                        parameterInfo_.angularResolution = 0.05;
                        parameterInfo_.freqHZ = 20;
                        parameterInfo_.scan_time = 1 / 20.0;
                    }
                    else if(_icode == PARAM_ANGULAR_RESOLUTION_3)
                    {
                        parameterInfo_.angularResolution = 0.025;
                        parameterInfo_.freqHZ = 10;
                        parameterInfo_.scan_time = 1 / 10.0;
                    }
                    else
                    {
                        //RCLCPP_ERROR(this->get_logger(), "ERROR");
                    }
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_ANGLE_RANGE)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_ANGLE_RANGE;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: " << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 4)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_OK;
                    printf("param1 : %d\n", std::stoi(_strVec[2]));
                    printf("param2 : %d\n", std::stoi(_strVec[3]));
                    parameterInfo_.startAngle = std::stoi(_strVec[2]) / 100.0;
                    parameterInfo_.stopAngle = std::stoi(_strVec[3]) / 100.0;
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_ANGLE_RANGE)

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_LIDAR_DATA_PACKET_TYPE;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: "<< _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 3)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_OK;
                    printf("param1 : %d\n", std::stoi(_strVec[2]));
                    parameterInfo_.lidarDataPacketType = std::stoi(_strVec[2]);
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE)

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_PROTOCOL_TYPE)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_PROTO_TYPE;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: " << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 3)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_OK;
                    printf("param1 : %d\n", std::stoi(_strVec[2]));
                    parameterInfo_.protocolType = std::stoi(_strVec[2]);
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_PROTOCOL_TYPE)

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_DATA_DIRECTION)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_DATA_DIRECTION;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get:" << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len:" << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 3)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK;
                    printf("param1 : %d\n", std::stoi(_strVec[2]));
                    parameterInfo_.scanDataDirection = std::stoi(_strVec[2]);
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_DATA_DIRECTION)

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_SKIP)
    {
        std::string _cmdStr = RESP_CMD_STR_GET_SKIP;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get:" << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len:" << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 3)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_GET_SKIP_OK;
                    printf("param1 : %d\n", std::stoi(_strVec[2]));
                    parameterInfo_.skipSpots = std::stoi(_strVec[2]);
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_GET_SKIP)

    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_SEND_MDI)
    {
        std::string _cmdStr = RESP_CMD_STR_SEND_MDI;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: " << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 2)
                {
                    sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_SEND_MDI_OK;
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_SEND_MDI)
    
    if(sys_cmd_realtime_ == SYS_CMD_REALTIME_STOP_MDI)
    {
        std::string _cmdStr = RESP_CMD_STR_STOP_MDI;
        ShowCharArray(src, _size);
        char _hexBuf[64];
        memset(_hexBuf, 0, 64);
        for(int i = 0; i < _size; i++)
        {
            sprintf(_hexBuf + strlen(_hexBuf), "%02X", src[i]);
        }
        char _ascii[64] = {0};
        memset(_ascii, 0, 64);
        Hex2Ascii(_hexBuf, _ascii);
        std::string _respStr = std::string(_ascii);
        std::cout << "@HandleTcpSocketRead get: " << _respStr << std::endl;
        std::cout << "@HandleTcpSocketRead get len: " << _respStr.length() << std::endl;
        if(_respStr.length() != 0)
        {
            std::string::size_type position;
            position = _respStr.find(_cmdStr);
            if (position != _respStr.npos)
            {
                printf("position is : %ld\n", position);
                std::vector<std::string> _strVec = YSplitString(_respStr,' ');
                printf("split size : %ld\n", _strVec.size());
                if(_strVec.size() == 2)
                {
                    sys_cmd_resp_=SYS_CMD_REALTIME_RESP_STOP_MDI_OK;
                }
            }
            else
            {
                printf("Not found the flag\n");
            }
        }
    }//end if(sys_cmd_realtime_ == SYS_CMD_REALTIME_STOP_MDI)

    if(!error)
    {
        // Read all received data and write it to the internal ring buffer
        instream_.clear();
        while(!instream_.eof())
        {
            char buf[4096];
            instream_.read(buf, 4096);
            int bytes_read = instream_.gcount();
            ShowCharArray(buf, bytes_read);
            printf("\n");
        }
        // Read data asynchronously
        tcp_socket_->async_receive(boost::asio::buffer(&tcp_buffer_[0], tcp_buffer_.size()), \
                boost::bind(&TcpCommandInterface::HandleTcpSocketRead, this, \
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if(error.value() != 995)
        {
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        }
        disconnect();
    }
}

void TcpCommandInterface::disconnect()
{
    try
    {
        if(tcp_socket_)
        {
            tcp_socket_->close();
        }

        io_service_.stop();
        if(boost::this_thread::get_id() != io_service_thread_.get_id())
        {
            io_service_thread_.join();
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

int TcpCommandInterface::GetAngularResolution()
{
    boost::system::error_code ec;
    
    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_ANGULAR_RESOLUTION;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "CMD_REALTIME_GET_ANGULAR_RESOLUTION OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetResol, sizeof(CMD_cRN_GetResol)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }
    
    return 0;
}

int TcpCommandInterface::GetAngleRange()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_ANGLE_RANGE;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "CMD_REALTIME_GET_ANGULAR_RANGE OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetRange, sizeof(CMD_cRN_GetRange)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }
    
    return 0;
}

int TcpCommandInterface::GetProtocolType()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_PROTOCOL_TYPE;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_OK OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetProto, sizeof(CMD_cRN_GetProto)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }

    return 0;
}

int TcpCommandInterface::GetLidarDataPacketType()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now()-_startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetPType, sizeof(CMD_cRN_GetPType)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }

    return 0;
}

int TcpCommandInterface::GetScanDataDirection()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_DATA_DIRECTION;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetDir, sizeof(CMD_cRN_GetDir)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }

    return 0;
}

int TcpCommandInterface::GetScanSkip()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_GET_SKIP;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    while(rclcpp::ok())
    {
        if((rclcpp::Clock().now() - _startTime).seconds() > SYS_TIMEOUT)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2;
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_GET_SKIP_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cRN_GetSkip, sizeof(CMD_cRN_GetSkip)), ec) << std::endl;
        //@IF ERROR
        if(ec)
        {
            std::cout << boost::system::system_error(ec).what() << std::endl;
            return -1;
        }
        usleep(10 * 1000);
    }

    return 0;
}

void TcpCommandInterface::SetCmdFactory(const std::string icmdSet)
{
    memset(setCmdBuf_, 0, 64);
    setCmdBuf_[0] = 0x02;
    for(auto i:icmdSet)
    {
        setCmdBuf_[strlen(setCmdBuf_)] = toascii(i);
    }
    setCmdBuf_[strlen(setCmdBuf_)] = 0x03;
    ShowCharArray(setCmdBuf_, strlen(setCmdBuf_));
}

void TcpCommandInterface::SetLidarDataPacketType(int param)
{
    boost::system::error_code ec;

    if(param == PARAM_DATA_PACKET_TYPE_DIST_INTEN)
    {
        SetCmdFactory(SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_INTEN);
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(setCmdBuf_, strlen(setCmdBuf_)), ec) << std::endl;
    }
    if(param == PARAM_DATA_PACKET_TYPE_DIST_ONLY)
    {
        SetCmdFactory(SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_ONLY);
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(setCmdBuf_, strlen(setCmdBuf_)), ec) << std::endl;
    }
}

void TcpCommandInterface::SetScanDataDirection(int param)
{
    boost::system::error_code ec;

    if(param == PARAM_DATA_DIRECTION_COUNTERCLOCKWISE)
    {
        SetCmdFactory(SET_CMD_STR_SPIN_DIRECTION_COUNTERCLOCKWISE);
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(setCmdBuf_, strlen(setCmdBuf_)), ec) << std::endl;
    }
    if(param == PARAM_DATA_DIRECTION_CLOCKWISE)
    {
        SetCmdFactory(SET_CMD_STR_SPIN_DIRECTION_CLOCKWISE);
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(setCmdBuf_, strlen(setCmdBuf_)), ec) << std::endl;
    }
}


int TcpCommandInterface::StartScanOutput()
{
    boost::system::error_code ec;

    SetCmdFactory(CTRL_CMD_STR_SEND_MDI);
    std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(setCmdBuf_, strlen(setCmdBuf_)), ec) << std::endl;

    return 1;
}

int TcpCommandInterface::StopScanOutput()
{
    boost::system::error_code ec;

    sys_cmd_realtime_ = SYS_CMD_REALTIME_STOP_MDI;
    rclcpp::Time _startTime = rclcpp::Clock().now();
    int _retry = 3;
    while(rclcpp::ok())
    {
        if(_retry-- == 0)
        {
            //RCLCPP_ERROR(this->get_logger(), "GET CMD TIMEOUT!");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return -2; 
        }
        if(sys_cmd_resp_ == SYS_CMD_REALTIME_RESP_STOP_MDI_OK)
        {
            //RCLCPP_WARN(this->get_logger(), "SYS_CMD_REALTIME_RESP_STOP_MDI_OK OK");
            sys_cmd_resp_ = SYS_CMD_REALTIME_RESP_NULL;
            return 1;
        }
        std::cout << "time consuming=" << (rclcpp::Clock().now() - _startTime).seconds() << std::endl;
        std::cout << DEBUG_POS << tcp_socket_->write_some(boost::asio::buffer(CMD_cWN_StopMDI, sizeof(CMD_cWN_StopMDI)), ec) << std::endl;
        usleep(1000*1000);
    }
    //@IF ERROR
    if(ec)
    {
        std::cout << boost::system::system_error(ec).what() << std::endl;
        return -1;
    }

    return 0;
}

}


















