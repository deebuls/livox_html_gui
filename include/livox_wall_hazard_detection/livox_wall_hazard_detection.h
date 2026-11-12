#pragma once
#include <vector>
#include <memory>
#include <csignal>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include "pointcloud_server.hpp"
#include "livox_manager.h"
#include "circular_buffer.h"
#include <open3d/Open3D.h>
#include <open3d/visualization/webrtc_server/WebRTCWindowSystem.h>


class LivoxWallHazardDetection {
public:
    explicit LivoxWallHazardDetection(LivoxManager& manager);

    void ProcessPointCloudThread(std::stop_token stoken);

    std::shared_ptr<open3d::geometry::TriangleMesh> CreatePlaneMeshFromCloud(
        const Eigen::Vector4d& plane_eq,
        const std::shared_ptr<open3d::geometry::PointCloud>& plane_cloud,
        const Eigen::Vector3d& color = Eigen::Vector3d(1.0, 0.7, 0.7));

    void SpinMainLoop();
private:
    // std::vector<pcws::Point> flat_points_;
    CircularBuffer<pcws::Point, 20000> circular_points_; //TODO config
    pcws::Server server_{9002}; //TODO config
    std::unique_ptr<open3d::visualization::Visualizer> vis_;
    bool vis_initialized_{false};
    std::vector<std::shared_ptr<const open3d::geometry::Geometry>> geometries_;

    void PointCloudCallback(uint32_t handle,
            const uint8_t dev_type,
            LivoxLidarEthernetPacket* data);

    void IMUCloudCallback(uint32_t handle,
            const uint8_t dev_type,
            LivoxLidarEthernetPacket* data);
};
