// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.

#ifndef VISIOSCAN_DRIVER_H
#define VISIOSCAN_DRIVER_H

#include <string>
#include <vector>
#include <boost/optional.hpp>
#include "protocol_info.h"
#include "packet_structure.h"
#include "common_libs.h"

namespace bea_power {

class TcpCommandInterface;
class ScanDataReceiver;

class VISIOSCANDriver
{
public:
    VISIOSCANDriver();

    ~VISIOSCANDriver();

    bool connect(const std::string hostname, int port = 3050);
    void disconnect();
    bool isConnected() {return is_connected_; }
    bool startCapturingTCP();
    bool startCapturingUDP();
    bool stopCapturing();
    bool isCapturing();
    bool checkConnection();

    const BEA_PARAMETER_INFO GetParameters() const { return parameterInfo_; }
    
    ScanData getScan();
    ScanData getFullScan();
    std::size_t getScansAvailable() const;
    std::size_t getFullScansAvailable() const;
    bool setScanFrequency( unsigned int frequency );
    bool rebootDevice();
    bool resetParameters( const std::vector<std::string>& names );
    bool setParameter( const std::string& name, const std::string& value );

private:

    ScanDataReceiver* data_receiver_;
    TcpCommandInterface* command_interface_;
    BEA_PARAMETER_INFO parameterInfo_;
    bool is_connected_;
    bool is_capturing_;
    std::string hostname_;
    int port_;
};

} // end namespace bea_power

#endif // VISIOSCAN_DRIVER_H
