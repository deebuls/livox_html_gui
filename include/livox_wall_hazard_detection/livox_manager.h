#pragma once
#include <string>
#include <functional>
#include "livox_lidar_api.h"

class LivoxManager {
public:
    using PointCloudCallback = std::function<void(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data)>;
    using ImuCallback = std::function<void(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* data)>;
    using InfoCallback = std::function<void(uint32_t handle, uint8_t dev_type, const LivoxLidarInfo* info)>;

    explicit LivoxManager(const std::string& config_path) {
        if (!LivoxLidarSdkInit(config_path.c_str())) {
            initialized_ = false;
            return;
        }

        SetLivoxLidarPointCloudCallBack(PointCloudBridge, this);
        SetLivoxLidarImuDataCallback(ImuBridge, this);
        // SetLivoxLidarInfoCallback(InfoBridge, this);
        // SetLivoxLidarInfoChangeCallback(InfoChangeBridge, this);

        initialized_ = true;
        // sleep(10);
        // std::cout << "Initialized finished " <<std::endl;

    }

    ~LivoxManager() {
        if (initialized_) {
            std::cout << "uninit being called " << std::endl;
            SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
            SetLivoxLidarImuDataCallback(nullptr, nullptr);
            SetLivoxLidarInfoCallback(nullptr, nullptr);
            SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
            LivoxLidarSdkUninit();
        }
    }

    LivoxManager(const LivoxManager&) = delete;
    LivoxManager& operator=(const LivoxManager&) = delete;
    LivoxManager(LivoxManager&&) = delete;
    LivoxManager& operator=(LivoxManager&&) = delete;

    void SetPointCloudCallback(PointCloudCallback cb) { point_cloud_cb_ = std::move(cb); }
    void SetImuCallback(ImuCallback cb) { imu_cb_ = std::move(cb); }
    void SetInfoCallback(InfoCallback cb) { info_cb_ = std::move(cb); }
    void SetInfoChangeCallback(InfoCallback cb) { info_change_cb_ = std::move(cb); }

    bool IsInitialized() const { return initialized_; }


private:
    bool initialized_ = false;

    PointCloudCallback point_cloud_cb_;
    ImuCallback imu_cb_;
    InfoCallback info_cb_;
    InfoCallback info_change_cb_;

    static void PointCloudBridge(uint32_t h, uint8_t d, LivoxLidarEthernetPacket* pkt, void* user_data) {
        auto* self = static_cast<LivoxManager*>(user_data);
        if (self && self->point_cloud_cb_) self->point_cloud_cb_(h, d, pkt);
    }
    static void ImuBridge(uint32_t h, uint8_t d, LivoxLidarEthernetPacket* pkt, void* user_data) {
        auto* self = static_cast<LivoxManager*>(user_data);
        if (self && self->imu_cb_) self->imu_cb_(h, d, pkt);
    }
    static void InfoBridge(uint32_t h, uint8_t d, const LivoxLidarInfo* info, void* user_data) {
        auto* self = static_cast<LivoxManager*>(user_data);
        if (self && self->info_cb_) self->info_cb_(h, d, info);
    }
    static void InfoChangeBridge(uint32_t h, uint8_t d, const LivoxLidarInfo* info, void* user_data) {
        auto* self = static_cast<LivoxManager*>(user_data);
        if (self && self->info_change_cb_) self->info_change_cb_(h, d, info);
    }
};
