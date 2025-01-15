// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.
#ifndef TCP_COMMAND_INTERFACE_H
#define TCP_COMMAND_INTERFACE_H
#include <string>
#include <boost/bind/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <array>
#include "protocol_info.h"
#include "packet_structure.h"
#include "common_libs.h"

using namespace boost::placeholders;

namespace bea_power {

class TcpCommandInterface
{
public:
    TcpCommandInterface(const std::string hostname, int tcp_port = 3050);
    ~TcpCommandInterface();
    void HandleTcpSocketRead(const boost::system::error_code& error, std::size_t bytes_transferred);
    void disconnect();
    int GetAngularResolution();
    int GetAngleRange();
    int GetLidarDataPacketType();
    int GetProtocolType();
    int GetScanDataDirection();
    int GetScanSkip();
    void SetCmdFactory(const std::string icmdSet);
    void SetLidarDataPacketType(int param);
    void SetScanDataDirection(int param);
    int StartScanOutput();
    int StopScanOutput();
    int sys_cmd_realtime_;
    int sys_cmd_resp_;

    const BEA_PARAMETER_INFO GetParameters() const { return parameterInfo_; }

private:
    boost::asio::ip::tcp::socket* tcp_socket_;
    boost::asio::streambuf inbuf_;
    std::istream instream_;
    boost::thread io_service_thread_;
    boost::asio::io_service io_service_;
    std::array< char, 65536 > tcp_buffer_;
    BEA_PARAMETER_INFO parameterInfo_;
    char setCmdBuf_[64];
};
}

#endif
