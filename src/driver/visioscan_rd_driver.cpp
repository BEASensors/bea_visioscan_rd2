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

    command_interface_->GetScanDataDirection();
    command_interface_->GetAngularResolution();
    command_interface_->GetAngleRange();
    command_interface_->GetLidarDataPacketType();
    command_interface_->GetScanSkip();
    if(command_interface_->GetProtocolType() != 1)
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
