// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.


#include <ctime>
#include "visioscan_rd_driver.h"
#include "packet_structure.h"
#include "tcp_command_interface.h"
#include "scan_data_receiver.h"

namespace bea_power {

VISIOSCANDriver::VISIOSCANDriver()
{
    command_interface_ = 0;
    data_receiver_ = 0;
    is_connected_ = false;
    is_capturing_ = false;
    hostname_ = "";
    port_ = 0;
}

bool VISIOSCANDriver::connect(const std::string hostname, int port)
{
    hostname_ = hostname;
    port_ = port;
    printf("@connect start\n");
    command_interface_ = new TcpCommandInterface(hostname, port);
    // Stop MDI first
    if(command_interface_->StopScanOutput() != 1)
    {
        return false;
    }

    // Read device parameters. They are populated asynchronously by the Boost
    // ASIO I/O thread inside TcpCommandInterface (see HandleTcpSocketRead).
    // A transient timeout must NOT be silently accepted: it would leave
    // angularResolution / skipSpots at their 0.0 default, which makes
    // deltaAngle = 0 -> LaserScan.angle_increment = 0.0 and breaks every
    // downstream node. Retry a few times; only proceed if all reads succeed.
    // On failure return false so the caller (bea_node work_loop) can reconnect
    // cleanly instead of publishing a broken (0-angle) scan.
    const int kParamRetries = 3;
    bool params_ok = false;
    for(int attempt = 0; attempt < kParamRetries && !params_ok; ++attempt)
    {
        int r_dir   = command_interface_->GetScanDataDirection();
        int r_res   = command_interface_->GetAngularResolution();
        int r_rng   = command_interface_->GetAngleRange();
        int r_pkt   = command_interface_->GetLidarDataPacketType();
        int r_skip  = command_interface_->GetScanSkip();
        int r_proto = command_interface_->GetProtocolType();
        if(r_dir == 1 && r_res == 1 && r_rng == 1 && r_pkt == 1 && r_skip == 1 && r_proto == 1)
        {
            params_ok = true;
        }
    }
    if(!params_ok)
    {
        return false;
    }
    parameterInfo_ = command_interface_->GetParameters();
    is_connected_ = true;
    printf("@connect end\n");
    return true;
}

VISIOSCANDriver::~VISIOSCANDriver()
{
    disconnect();
}

bool VISIOSCANDriver::startCapturingTCP()
{
    if(!checkConnection())
    {
        return false;
    }
    
    // delete command interface
    delete command_interface_;
    command_interface_ = 0;

    data_receiver_ = new ScanDataReceiver(hostname_, port_);
    if(!data_receiver_->isConnected())
    {
        return false;
    }
    data_receiver_->sendMDI();
    is_capturing_ = true;
    return true;
}

bool VISIOSCANDriver::startCapturingUDP()
{
    if(!checkConnection())
    {
        return false;
    }

    command_interface_->StartScanOutput();
    data_receiver_ = new ScanDataReceiver(hostname_, port_, true);
    if(!data_receiver_->isConnected())
    {
        return false;
    }
    is_capturing_ = true;
    return true;
}

bool VISIOSCANDriver::stopCapturing()
{
    if(!is_capturing_) // || !command_interface_)
        return false;
    
    // Stop TCP capturing
    if(!command_interface_)
    {
        data_receiver_->stopMDI();
    }
    // Stop UDP capturing
    else
    {
        command_interface_->StopScanOutput();
    }
    bool return_val = checkConnection();
    // command_interface_->StopScanOutput();

    return return_val;
}

bool VISIOSCANDriver::checkConnection()
{
    if(/*!command_interface_ ||*/ !is_connected_)
    {
        std::cerr << "ERROR: No connection to scanner or connection lost!" << std::endl;
        return false;
    }

    return true;
}

ScanData VISIOSCANDriver::getScan()
{
    if(data_receiver_)
    {
        return data_receiver_->getScan();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return ScanData();
    }
}

ScanData VISIOSCANDriver::getFullScan()
{
    if(data_receiver_)
    {
        return data_receiver_->getFullScan();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return ScanData();
    }
}

std::size_t VISIOSCANDriver::getScansAvailable() const
{
    if(data_receiver_)
    {
        return data_receiver_->getScansAvailable();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return 0;
    }
}

std::size_t VISIOSCANDriver::getFullScansAvailable() const
{
    if(data_receiver_)
    {
        return data_receiver_->getFullScansAvailable();
    }
    else
    {
        std::cerr << "ERROR: No scan capturing started!" << std::endl;
        return 0;
    }
}

void VISIOSCANDriver::disconnect()
{
    if(isCapturing())
        stopCapturing();

    delete data_receiver_;
    delete command_interface_;
    data_receiver_ = 0;
    command_interface_ = 0;

    is_capturing_ = false;
    is_connected_ = false;
}

bool VISIOSCANDriver::isCapturing()
{
    //return is_capturing_ && data_receiver_->isConnected();
    return is_connected_ && data_receiver_->checkConnection();
}

}
