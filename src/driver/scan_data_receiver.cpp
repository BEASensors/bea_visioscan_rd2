// Copyright (c) 2024, BEA
// Copyright (c) 2024, BEA
// All rights reserved.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "scan_data_receiver.h"

namespace bea_power {

//@TCP
ScanDataReceiver::ScanDataReceiver(const std::string hostname, const int port) : inbuf_(4096), instream_(&inbuf_), ring_buffer_(65536), scan_data_()
{
    last_data_time_ = std::time(0);
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;
    std::cout << "Connecting to TCP data channel at " << hostname << ":" << port << " ... ";
    try
    {
        // Resolve hostname/ip
        boost::asio::ip::tcp::resolver resolver(io_service_);
        boost::asio::ip::tcp::resolver::query query(hostname, std::to_string(port));
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
        boost::asio::ip::tcp::resolver::iterator end;

        tcp_socket_ = new boost::asio::ip::tcp::socket(io_service_);
        boost::system::error_code error = boost::asio::error::host_not_found;

        // Iterate over endpoints and etablish connection
        while (error && endpoint_iterator != end)
        {
            tcp_socket_->close();
            tcp_socket_->connect(*endpoint_iterator++, error);
        }
        if (error)
        {
            throw boost::system::system_error(error);
        }

        // Start async reading
        boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&ScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;

    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

//@ udp
ScanDataReceiver::ScanDataReceiver(const std::string hostname, int port,bool use_udp) : inbuf_(4096), instream_(&inbuf_), ring_buffer_(65536), scan_data_()
{
    tcp_socket_ = 0;
    udp_socket_ = 0;
    udp_port_ = -1;
    is_connected_ = false;

    if(use_udp)
        std::cout << "Connecting to UDP data channel at " << hostname << ":" << port << " ... ";
    try
    {
        udp_socket_ = new boost::asio::ip::udp::socket(io_service_, boost::asio::ip::udp::v4());
        udp_socket_->bind(boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port));
        udp_port_ = udp_socket_->local_endpoint().port();
        // Start async reading
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0], udp_buffer_.size()), udp_endpoint_,
                boost::bind(&ScanDataReceiver::handleSocketRead, this,
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        io_service_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
        is_connected_ = true;
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " <<  e.what() << std::endl;
    }
}

ScanDataReceiver::~ScanDataReceiver()
{
    disconnect();
    delete udp_socket_;
    delete tcp_socket_;
}

void ScanDataReceiver::handleSocketRead(const boost::system::error_code &error)
{
    if(!error)
    {
        // Read all received data and write it to the internal ring buffer
        instream_.clear();
        while(!instream_.eof())
        {
            char buf[4096];
            instream_.read(buf, 4096);
            int bytes_read = instream_.gcount();
            writeBufferBack(buf,bytes_read);
        }
        
        // Handle (read and parse) packets stored in the internal ring buffer
        while(handleNextPacket()) {}

        boost::asio::async_read(*tcp_socket_, inbuf_, boost::bind(&ScanDataReceiver::handleSocketRead, this, boost::asio::placeholders::error));
    }
    else
    {
        if(error.value() != 995)
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        disconnect();
    }
    last_data_time_ = std::time(0);
}

void ScanDataReceiver::handleSocketRead(const boost::system::error_code &error, std::size_t bytes_transferred)
{
    if(!error)
    {
        // Read all received data and write it to the internal ring buffer
        writeBufferBack(&udp_buffer_[0], bytes_transferred);
        // Handle (read and parse) packets stored in the internal ring buffer
        while(handleNextPacket()) {}
        // Read data asynchronously
        udp_socket_->async_receive_from(boost::asio::buffer(&udp_buffer_[0], udp_buffer_.size()), udp_endpoint_,
                boost::bind(&ScanDataReceiver::handleSocketRead, this,
                            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        if(error.value() != 995)
            std::cerr << "ERROR: " << "data connection error: " << error.message() << "(" << error.value() << ")" << std::endl;
        disconnect();
    }
    last_data_time_ = std::time(0);
}

bool ScanDataReceiver::handleNextPacket()
{
    // Search for a packet
    int packet_start = findPacketStart();
    if(packet_start < 0)
    {
        return false;
    }

    // Try to retrieve packet
    char buf[65536];
    PacketTypeC* p = (PacketTypeC*) buf;
    if(!retrievePacket(packet_start,p))
    {
        return false;
    }
    // Lock internal outgoing data queue, automatically unlocks at end of function
    std::unique_lock<std::mutex> lock(data_mutex_);

    // Create new scan container if necessary
    if(scan_data_.empty()) //init
    {
        scan_data_.emplace_back();
    }
    else
    {
        if(scan_data_.size() > 100)
        {
            scan_data_.pop_front();
            std::cerr << "Too many scans in receiver queue: Dropping scans!" << std::endl;
        }
        scan_data_.emplace_back();
        data_notifier_.notify_one();
    }
    
    ScanData& myScanData = scan_data_.back();

    // Parse payload of packet
    std::uint16_t* p_scan_data = (std::uint16_t*) &buf[MDI_PACKET_HEADER_SIZE];
    int num_scan_points = p->header.scanPoints;
    if(p->header.subNum == 1)
    {
        scanPointsSum_ = 0;
        std::vector<unsigned short>().swap(distanceData_);
        std::vector<unsigned short>().swap(intensitiesData_);
    }
    scanPointsSum_ += p->header.scanPoints;
    //@Type of scan data packet, 0 = distance only, 1= distance & intensity ;
    if(p->header.packetType == 1)//1= distance & intensity
    {
        for(int i = 0; i < num_scan_points; i++)
        {
            unsigned short data = ntohs(p_scan_data[i]);
            distanceData_.push_back(data);
        }
        for(int i = num_scan_points; i < num_scan_points * 2; i++)
        {
            unsigned short data1 = ntohs(p_scan_data[i]);
            intensitiesData_.push_back(data1);
        }
    }
    else
    {
        for(int i = 0; i < num_scan_points; i++)
        {
            unsigned short data = ntohs(p_scan_data[i]);
            distanceData_.push_back(data);
        }
    }

    if(p->header.subNum == p->header.totalNum)
    {
        // Complete scan
        int _scanPointLen = distanceData_.size();
        
        if(p->header.packetType == 1)
        {
            for(int i = 0; i < _scanPointLen; i++)
            {
                myScanData.distance_data.push_back(distanceData_[i]);
                myScanData.amplitude_data.push_back(intensitiesData_[i]);
            }
        }
        else
        {
            for(int i = 0; i < _scanPointLen; i++)
            {
                myScanData.distance_data.push_back(distanceData_[i]);
            }
        }

        // Save header (of last packet)
        myScanData.headers.push_back(p->header);
        //std::cout << "There is " << _scanPointLen << " pts in 1 scan with distance[688]= " << myScanData.distance_data.at(_scanPointLen/2) << std::endl;
    }

    return true;
}


int ScanDataReceiver::findPacketStart()
{
    if(ring_buffer_.size() < MDI_PACKET_HEADER_SIZE)
    {
        return -1;
    }
    for(std::size_t i = 0; i < ring_buffer_.size() - 4; i++)
    {
        if(((unsigned char) ring_buffer_[i]) == MDI_PACKET_HEADER_SYNC_0
            && ((unsigned char) ring_buffer_[i + 1]) == MDI_PACKET_HEADER_SYNC_1
            && ((unsigned char) ring_buffer_[i + 2]) == MDI_PACKET_HEADER_SYNC_2
            && ((unsigned char) ring_buffer_[i + 3]) == MDI_PACKET_HEADER_SYNC_3)
        {
            return i;
        }
    }
    return -2;
}

void ScanDataReceiver::PacketHeaderToLittleEndian(PacketTypeC *pHead)
{
    pHead->header.packetSize = ntohs(pHead->header.packetSize);
    pHead->header.reservedA = ntohs(pHead->header.reservedA);
    pHead->header.reservedB = ntohs(pHead->header.reservedB);
    pHead->header.reservedC = ntohs(pHead->header.reservedC);
    pHead->header.packetNum = ntohs(pHead->header.packetNum);
    pHead->header.scanFreq = ntohs(pHead->header.scanFreq);
    pHead->header.scanPoints = ntohs(pHead->header.scanPoints);
    pHead->header.firstAngle = ntohl(pHead->header.firstAngle);
    pHead->header.deltaAngle = ntohl(pHead->header.deltaAngle);
    pHead->header.timeStamp = ntohs(pHead->header.timeStamp);
}

bool ScanDataReceiver::retrievePacket(std::size_t start, PacketTypeC *p)
{
    if(ring_buffer_.size() < MDI_PACKET_HEADER_SIZE)
    {
        printf("@retrievePacket error 1\n");
        return false;
    }

    //@从找到开始，开始之前全不要
    ring_buffer_.erase_begin(start);
    char* pp = (char*) p;
    // Read header
    readBufferFront(pp, MDI_PACKET_HEADER_SIZE);
    PacketHeaderToLittleEndian(p);

    if(ring_buffer_.size() < p->header.packetSize)
    {
#if 0
        ShowCharArray(pp, MDI_PACKET_HEADER_SIZE);
        printf("p->header.totalNum=%d\n",p->header.totalNum);
        printf("p->header.subNum=%d\n",p->header.subNum);
        printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!(%ld,%d)\n",ring_buffer_.size(),p->header.packetSize);
#endif
        return false;
    }

    // Read header+payload data
    readBufferFront(pp, p->header.packetSize);
    PacketHeaderToLittleEndian(p);

    //@erase
    ring_buffer_.erase_begin(p->header.packetSize);
    return true;
}

void ScanDataReceiver::disconnect()
{
    is_connected_ = false;
    try
    {
        if(tcp_socket_)
        {
            tcp_socket_->close();
        }
        if(udp_socket_)
        {
            udp_socket_->close();
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

bool ScanDataReceiver::checkConnection()
{
    if(!isConnected())
        return false;
    if((std::time(0) - last_data_time_) > 2)
    {
        std::cerr << "Error: receive distance data timeout, would disconnect scanner.." << std::endl;
        disconnect();
        return false;
    }
    return true;
}

ScanData ScanDataReceiver::getScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    ScanData data(std::move(scan_data_.front()));
    scan_data_.pop_front();
    return data;
}

ScanData ScanDataReceiver::getFullScan()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    //while ( checkConnection() && isConnected() && scan_data_.size() < 1)
    //{
    //    data_notifier_.wait_for(lock, std::chrono::seconds(1));
    //}
    ScanData data;
    if(scan_data_.size() >= 1 && isConnected())
    {
        data = ScanData(std::move(scan_data_.front()));
        scan_data_.pop_front();
    }
    return data;
}

std::size_t ScanDataReceiver::getFullScansAvailable() const
{
    if(scan_data_.size() == 0)
        return 0;
    else
        return scan_data_.size() - 1;
}

void ScanDataReceiver::writeBufferBack(char *src, std::size_t numbytes)
{
    if(ring_buffer_.size() + numbytes > ring_buffer_.capacity())
    {
        printf("@writeBufferBack error!");
        throw std::exception();
    }
    ring_buffer_.resize(ring_buffer_.size() + numbytes);
    char* pone = ring_buffer_.array_one().first;
    std::size_t pone_size = ring_buffer_.array_one().second;
    char* ptwo = ring_buffer_.array_two().first;
    std::size_t ptwo_size = ring_buffer_.array_two().second;
    if( ptwo_size >= numbytes )
    {
        std::memcpy(ptwo + ptwo_size - numbytes, src, numbytes);
    }
    else
    {
        std::memcpy(pone + pone_size + ptwo_size - numbytes,
                    src,
                    numbytes - ptwo_size);
        std::memcpy(ptwo,
                    src + numbytes - ptwo_size,
                    ptwo_size);
    }
}

void ScanDataReceiver::readBufferFront(char *dst, std::size_t numbytes)
{
    if(ring_buffer_.size() < numbytes)
    {
        printf("@readBufferFront error!");
        throw std::exception();
    }
    char* pone = ring_buffer_.array_one().first;
    std::size_t pone_size = ring_buffer_.array_one().second;

    char* ptwo = ring_buffer_.array_two().first;

    if(pone_size >= numbytes)
    {
        std::memcpy(dst, pone, numbytes);
    }
    else
    {
        std::memcpy(dst, pone, pone_size);
        std::memcpy(dst + pone_size, ptwo, numbytes - pone_size);
    }
}

bool ScanDataReceiver::sendMDI()
{
    boost::system::error_code ec;

    tcp_socket_->write_some(boost::asio::buffer(CMD_cWN_SendMDI, sizeof(CMD_cWN_SendMDI)), ec);
    if(ec)
    {
        std::cout << "SendMDI in ScanDataReceiver error:" << boost::system::system_error(ec).what() << std::endl;
        return false;
    }

    return true;
}

bool ScanDataReceiver::stopMDI()
{
    boost::system::error_code ec;

    tcp_socket_->write_some(boost::asio::buffer(CMD_cWN_StopMDI, sizeof(CMD_cWN_StopMDI)), ec);
    if(ec)
    {
        std::cout << "StopMDI in ScanDataReceiver error:" << boost::system::system_error(ec).what() << std::endl;
        return false;
    }

    return true;
}

}
