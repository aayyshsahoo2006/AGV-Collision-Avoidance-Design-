#ifndef SENSOR_LINUX_SENSOR_INTERFACE_HPP
#define SENSOR_LINUX_SENSOR_INTERFACE_HPP

#include "SensorData.hpp"
#include <vector>
#include <deque>
#include <mutex>
#include <cstdint>

namespace agv {
namespace sensor {

/**
 * @brief Binary packet format mimicking a hardware radar CAN/UART/char-device payload.
 */
#pragma pack(push, 1)
struct RadarHardwarePacket {
    uint32_t magic;
    uint64_t timestamp_us;
    int32_t  obstacle_id;
    int32_t  distance_mm;
    int16_t  angle_mrad;
    int16_t  rel_velocity_mms;
    uint8_t  detected_flag;
    uint8_t  checksum;
};
#pragma pack(pop)

/**
 * @brief Interface emulating a Linux character device driver (/dev/agv_radar)
 * with POSIX-like open, read, write, ioctl, and ring buffering.
 */
class LinuxSensorInterface {
public:
    LinuxSensorInterface();
    ~LinuxSensorInterface();

    int dev_open();
    int dev_close();
    ssize_t dev_read(uint8_t* buffer, size_t count);
    ssize_t dev_write(const uint8_t* buffer, size_t count);
    int dev_ioctl(unsigned long request, void* arg);

    void publishObservations(const std::vector<SensorObservation>& observations);
    std::vector<SensorObservation> readObservations();

    bool isOpened() const { return is_open_; }
    size_t getQueueSize() const;

private:
    uint8_t computeChecksum(const RadarHardwarePacket& pkt) const;
    bool is_open_{false};
    std::deque<RadarHardwarePacket> ring_buffer_;
    mutable std::mutex buffer_mutex_;
    static constexpr size_t MAX_BUFFER_CAPACITY = 1024;
};

}
}

#endif
