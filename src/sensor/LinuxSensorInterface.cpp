#include "../../include/sensor/LinuxSensorInterface.hpp"
#include <cstring>
#include <cmath>

namespace agv {
namespace sensor {

LinuxSensorInterface::LinuxSensorInterface() : is_open_(false) {}

LinuxSensorInterface::~LinuxSensorInterface() {
    dev_close();
}

int LinuxSensorInterface::dev_open() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    is_open_ = true;
    ring_buffer_.clear();
    return 0;
}

int LinuxSensorInterface::dev_close() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    is_open_ = false;
    ring_buffer_.clear();
    return 0;
}

uint8_t LinuxSensorInterface::computeChecksum(const RadarHardwarePacket& pkt) const {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t csum = 0;
    for (size_t i = 0; i < sizeof(RadarHardwarePacket) - 1; ++i) {
        csum ^= bytes[i];
    }
    return csum;
}

ssize_t LinuxSensorInterface::dev_write(const uint8_t* buffer, size_t count) {
    if (!is_open_ || !buffer || count < sizeof(RadarHardwarePacket)) {
        return -1;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    size_t packets_to_write = count / sizeof(RadarHardwarePacket);

    for (size_t i = 0; i < packets_to_write; ++i) {
        if (ring_buffer_.size() >= MAX_BUFFER_CAPACITY) {
            ring_buffer_.pop_front();
        }
        RadarHardwarePacket incoming_pkt;
        std::memcpy(&incoming_pkt, buffer + (i * sizeof(RadarHardwarePacket)), sizeof(RadarHardwarePacket));
        ring_buffer_.push_back(incoming_pkt);
    }

    return static_cast<ssize_t>(packets_to_write * sizeof(RadarHardwarePacket));
}

ssize_t LinuxSensorInterface::dev_read(uint8_t* buffer, size_t count) {
    if (!is_open_ || !buffer || count < sizeof(RadarHardwarePacket)) {
        return -1;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (ring_buffer_.empty()) {
        return 0;
    }

    size_t max_packets = count / sizeof(RadarHardwarePacket);
    size_t packets_to_read = std::min(max_packets, ring_buffer_.size());

    for (size_t i = 0; i < packets_to_read; ++i) {
        const RadarHardwarePacket& pkt = ring_buffer_.front();
        std::memcpy(buffer + (i * sizeof(RadarHardwarePacket)), &pkt, sizeof(RadarHardwarePacket));
        ring_buffer_.pop_front();
    }

    return static_cast<ssize_t>(packets_to_read * sizeof(RadarHardwarePacket));
}

int LinuxSensorInterface::dev_ioctl(unsigned long request, void* arg) {
    if (!is_open_) return -1;
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (request == 0x01) {
        ring_buffer_.clear();
        return 0;
    }
    if (request == 0x02 && arg) {
        *static_cast<size_t*>(arg) = ring_buffer_.size();
        return 0;
    }
    return -1;
}

void LinuxSensorInterface::publishObservations(const std::vector<SensorObservation>& observations) {
    if (!is_open_) return;

    std::vector<RadarHardwarePacket> packets;
    packets.reserve(observations.size());

    for (const auto& obs : observations) {
        RadarHardwarePacket pkt;
        pkt.magic = 0x52414452;
        pkt.timestamp_us = static_cast<uint64_t>(obs.timestamp * 1e6);
        pkt.obstacle_id = obs.obstacle_id;
        pkt.distance_mm = static_cast<int32_t>(obs.distance * 1000.0);
        pkt.angle_mrad = static_cast<int16_t>(obs.angle * 1000.0);
        pkt.rel_velocity_mms = static_cast<int16_t>(obs.relative_velocity * 1000.0);
        pkt.detected_flag = obs.detected ? 1 : 0;
        pkt.checksum = computeChecksum(pkt);

        packets.push_back(pkt);
    }

    if (!packets.empty()) {
        dev_write(reinterpret_cast<const uint8_t*>(packets.data()),
                  packets.size() * sizeof(RadarHardwarePacket));
    }
}

std::vector<SensorObservation> LinuxSensorInterface::readObservations() {
    std::vector<SensorObservation> observations;
    if (!is_open_) return observations;

    std::vector<RadarHardwarePacket> read_buf(64);
    while (true) {
        ssize_t bytes_read = dev_read(reinterpret_cast<uint8_t*>(read_buf.data()),
                                      read_buf.size() * sizeof(RadarHardwarePacket));
        if (bytes_read <= 0) break;

        size_t count = static_cast<size_t>(bytes_read) / sizeof(RadarHardwarePacket);
        for (size_t i = 0; i < count; ++i) {
            const auto& pkt = read_buf[i];
            if (pkt.magic != 0x52414452) continue;
            if (pkt.checksum != computeChecksum(pkt)) continue;

            SensorObservation obs;
            obs.timestamp = static_cast<double>(pkt.timestamp_us) / 1e6;
            obs.obstacle_id = pkt.obstacle_id;
            obs.distance = static_cast<double>(pkt.distance_mm) / 1000.0;
            obs.angle = static_cast<double>(pkt.angle_mrad) / 1000.0;
            obs.relative_velocity = static_cast<double>(pkt.rel_velocity_mms) / 1000.0;
            obs.detected = (pkt.detected_flag != 0);
            obs.relative_pos = {
                obs.distance * std::cos(obs.angle),
                obs.distance * std::sin(obs.angle)
            };
            observations.push_back(obs);
        }
    }

    return observations;
}

size_t LinuxSensorInterface::getQueueSize() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    return ring_buffer_.size();
}

}
}
