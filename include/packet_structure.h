// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.

#ifndef PACKET_STRUCTURE_H
#define PACKET_STRUCTURE_H
#include <cstdint>
#include <vector>

namespace bea_power {

#pragma pack(1)
//@ struct PacketHeader
//@ Header of a  UDP data packet from the scanner
struct PacketHeader
{
    std::uint32_t sync;
    std::uint8_t packetType;
    std::uint16_t packetSize;
    std::uint16_t reservedA;
    std::uint16_t reservedB;
    std::uint16_t reservedC;
    std::uint16_t packetNum;
    std::uint8_t totalNum;
    std::uint8_t subNum;
    std::uint16_t scanFreq;
    std::uint16_t scanPoints;
    std::int32_t firstAngle;
    std::int32_t deltaAngle;
    std::uint16_t timeStamp;
};

//@ \brief Structure of a UDP or TCP data packet from the laserscanner
struct PacketTypeC
{
    PacketHeader header;
    //std::uint32_t distance_amplitude_payload; // distance 20 bit, amplitude 12 bit
};
#pragma pack()

//@ \struct ScanData
//@ \brief Normally contains one complete laserscan (a full rotation of the scanner head)
struct ScanData
{
    //@ Distance data in polar form in millimeter
    std::vector<std::uint16_t> distance_data;

    //@ Amplitude data in the range 32-4095, values lower than 32 indicate an error or undefined values
    std::vector<std::uint16_t> amplitude_data;

    //@ Header received with the distance and amplitude data
    std::vector<PacketHeader> headers;
};

}

#endif // PACKET_STRUCTURE_H
