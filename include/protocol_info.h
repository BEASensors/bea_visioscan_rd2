// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.
#ifndef PROTOCOL_INFO_H
#define PROTOCOL_INFO_H
#include <vector>
#define ROS_RANGE_MAX 25.0

const unsigned char CMD_cWN_SendMDI[13]  = { 0x02, 0x63, 0x57, 0x4E, 0x20, 0x53, 0x65, 0x6E, 0x64, 0x4D, 0x44, 0x49, 0x03 };
const unsigned char CMD_cWN_StopMDI[13]  = { 0x02, 0x63, 0x57, 0x4E, 0x20, 0x53, 0x74, 0x6F, 0x70, 0x4D, 0x44, 0x49, 0x03 };
const unsigned char CMD_cRN_GetResol[14] = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x52, 0x65, 0x73, 0x6F, 0x6C, 0x03 };

const unsigned char CMD_cRN_GetPort[13]  = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x6F, 0x72, 0x74, 0x03 };
const unsigned char CMD_cRN_GetRange[14] = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x52, 0x61, 0x6E, 0x67, 0x65, 0x03 };
const unsigned char CMD_cRN_GetPType[14] = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x54, 0x79, 0x70, 0x65, 0x03 };
const unsigned char CMD_cRN_GetProto[14] = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x50, 0x72, 0x6F, 0x74, 0x6F, 0x03 };
const unsigned char CMD_cRN_GetDir[12]   = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x44, 0x69, 0x72, 0x03 };
const unsigned char CMD_cRN_GetSkip[13]  = { 0x02, 0x63, 0x52, 0x4E, 0x20, 0x47, 0x65, 0x74, 0x53, 0x6B, 0x69, 0x70, 0x03 };

#define TO_RADIAN                                              0.017453292222222222
#define SYS_TIMEOUT                                            1
#define SYS_CMD_REALTIME_NULL                                  0
#define SYS_CMD_REALTIME_RESP_NULL                             0
#define SYS_CMD_REALTIME_GET_ANGULAR_RESOLUTION                2001
#define SYS_CMD_REALTIME_GET_ANGLE_RANGE                       2002
#define SYS_CMD_REALTIME_GET_LIDAR_DATA_PACKET_TYPE            2003
#define SYS_CMD_REALTIME_GET_PROTOCOL_TYPE                     2004
#define SYS_CMD_REALTIME_GET_DATA_DIRECTION                    2005
#define SYS_CMD_REALTIME_SEND_MDI                              2006
#define SYS_CMD_REALTIME_STOP_MDI                              2007
#define SYS_CMD_REALTIME_GET_SKIP                              2008

#define SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_OK        3001
#define SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_OK               3002
#define SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_OK    3003
#define SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_OK             3004
#define SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_OK            3005
#define SYS_CMD_REALTIME_RESP_SEND_MDI_OK                      3006
#define SYS_CMD_REALTIME_RESP_STOP_MDI_OK                      3007
#define SYS_CMD_REALTIME_RESP_GET_SKIP_OK                      3008

#define SYS_CMD_REALTIME_RESP_GET_ANGULAR_RESOLUTION_ERROR     4001
#define SYS_CMD_REALTIME_RESP_GET_ANGLE_RANGE_ERROR            4002
#define SYS_CMD_REALTIME_RESP_GET_LIDAR_DATA_PACKET_TYPE_ERROR 4003
#define SYS_CMD_REALTIME_RESP_GET_PROTOCOL_TYPE_ERROR          4004
#define SYS_CMD_REALTIME_RESP_GET_DATA_DIRECTION_ERROR         4005
#define SYS_CMD_REALTIME_RESP_GET_SKIP_ERROR                   4008

// @Command Syntax  excluding the data
const std::string RESP_CMD_STR_GET_RESOL                  = "cRA GetResol";
const std::string RESP_CMD_STR_GET_ANGLE_RANGE            = "cRA GetRange";
const std::string RESP_CMD_STR_GET_LIDAR_DATA_PACKET_TYPE = "cRA GetPType";
const std::string RESP_CMD_STR_GET_PROTO_TYPE             = "cRA GetProto";
const std::string RESP_CMD_STR_GET_DATA_DIRECTION         = "cRA GetDir";
const std::string RESP_CMD_STR_GET_SKIP                   = "cRA GetSkip";

// @sets
const std::string SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_INTEN = "cWN SetPType 1";
const std::string SET_CMD_STR_LIDAR_DATA_PACKET_TYPE_DIS_ONLY  = "cWN SetPType 0";
const std::string SET_CMD_STR_SPIN_DIRECTION_CLOCKWISE         = "cWN SetDir 0";
const std::string SET_CMD_STR_SPIN_DIRECTION_COUNTERCLOCKWISE  = "cWN SetDir 1";

// @control
const std::string CTRL_CMD_STR_SEND_MDI = "cWN SendMDI";
const std::string CTRL_CMD_STR_STOP_MDI = "cWN StopMDI";
const std::string RESP_CMD_STR_SEND_MDI = "cWA SendMDI";
const std::string RESP_CMD_STR_STOP_MDI = "cWA StopMDI";

// PARAMETER: Angular resolution
#define PARAM_ANGULAR_RESOLUTION_0            0
#define PARAM_ANGULAR_RESOLUTION_1            1
#define PARAM_ANGULAR_RESOLUTION_2            2
#define PARAM_ANGULAR_RESOLUTION_3            3
#define PARAM_ANGULAR_RESOLUTION_4            4

// PARAMETER: Data packet type
#define PARAM_DATA_PACKET_TYPE_DIST_ONLY      0
#define PARAM_DATA_PACKET_TYPE_DIST_INTEN     1

// PARAMETER: Data output direction
#define PARAM_DATA_DIRECTION_CLOCKWISE        0
#define PARAM_DATA_DIRECTION_COUNTERCLOCKWISE 1

namespace bea_power
{
typedef struct tag_BEA_PARAMETER_INFO
{
    int protocolType;      // 0-UDP  1-TCP
    int lidarDataPacketType;
    int scanDataDirection; // 0-Clockwise  1-Countclockwise
    int skipSpots;
    double startAngle;
    double stopAngle;
    double angularResolution;
    double freqHZ;
    double scan_time;
    double deltaAngle;
} BEA_PARAMETER_INFO;

// @ \class ProtocolInfo
// @ \brief Information about the HTTP/JSON protocol
struct ProtocolInfo
{
    std::string protocol_name;
    int version_major;
    int version_minor;
    std::vector<std::string> commands;
};

// @ \class HandleInfo
// @ \brief Encapsulates data about an etablished data connection
struct HandleInfo
{
    static const int HANDLE_TYPE_TCP = 0;
    static const int HANDLE_TYPE_UDP = 1;
    int handle_type;
    std::string hostname;
    int port;
    std::string handle;
    char packet_type;
    int start_angle;
    bool watchdog_enabled;
    int watchdog_timeout;
};
}

#endif // PROTOCOL_INFO_H
