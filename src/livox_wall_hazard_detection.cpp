#include <iostream>
#include <chrono>
#include <ranges>

#include "livox_wall_hazard_detection.h"


LivoxWallHazardDetection::LivoxWallHazardDetection(LivoxManager& manager){
        std::cout << "constructor started " << std::endl;
        if (!server_.start()) {
            std::cout << "server start failure " <<std::endl;
            return;
        }

        std::printf("ws://localhost:%d  (Ctrl+C to quit)\n", server_.port());
        manager.SetPointCloudCallback(
            [this](uint32_t h, uint8_t d, LivoxLidarEthernetPacket* pkt) {
                PointCloudCallback(h, d, pkt);
            }
        );

        manager.SetImuCallback(
            [this](uint32_t h, uint8_t d, LivoxLidarEthernetPacket* pkt) {
                IMUCloudCallback(h, d, pkt);
            }
        );

        vis_ = std::make_unique<open3d::visualization::Visualizer>();
        vis_->CreateVisualizerWindow("RANSAC Planes Live", 1280, 720);
        //open3d::visualization::webrtc_server::WebRTCWindowSystem::GetInstance()->EnableWebRTC();
};

std::shared_ptr<open3d::geometry::TriangleMesh> LivoxWallHazardDetection::CreatePlaneMeshFromCloud(
    const Eigen::Vector4d& plane_eq,
    const std::shared_ptr<open3d::geometry::PointCloud>& plane_cloud,
    const Eigen::Vector3d& color)
{
    Eigen::Vector3d n = plane_eq.head<3>();
    double D = plane_eq(3);
    Eigen::Vector3d normal = n.normalized();

    // 1. Find local coordinate axes on the plane
    Eigen::Vector3d u(1.0, 0.0, 0.0);
    if (std::abs(normal.dot(u)) > 0.9) {
        u = Eigen::Vector3d(0.0, 1.0, 0.0);
    }
    Eigen::Vector3d v1 = normal.cross(u).normalized();
    Eigen::Vector3d v2 = normal.cross(v1).normalized();

    // 2. Project point cloud onto v1 and v2 to find dynamic min/max bounds
    double min_v1 = std::numeric_limits<double>::max();
    double max_v1 = std::numeric_limits<double>::lowest();
    double min_v2 = std::numeric_limits<double>::max();
    double max_v2 = std::numeric_limits<double>::lowest();

    Eigen::Vector3d p_ref = plane_cloud->GetCenter();
    Eigen::Vector3d p0 = p_ref - ((n.dot(p_ref) + D) / n.squaredNorm()) * n;

    for (const auto& pt : plane_cloud->points_) {
        Eigen::Vector3d p_proj = pt - ((n.dot(pt) + D) / n.squaredNorm()) * n;
        double coord1 = (p_proj - p0).dot(v1);
        double coord2 = (p_proj - p0).dot(v2);

        min_v1 = std::min(min_v1, coord1);
        max_v1 = std::max(max_v1, coord1);
        min_v2 = std::min(min_v2, coord2);
        max_v2 = std::max(max_v2, coord2);
    }

    // 3. Build the quad mesh using the exact extents of the cloud
    std::vector<Eigen::Vector3d> vertices(4);
    vertices[0] = p0 + (min_v1 * v1 + min_v2 * v2);
    vertices[1] = p0 + (max_v1 * v1 + min_v2 * v2);
    vertices[2] = p0 + (max_v1 * v1 + max_v2 * v2);
    vertices[3] = p0 + (min_v1 * v1 + max_v2 * v2);

    auto plane_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    plane_mesh->vertices_ = vertices;
    plane_mesh->triangles_ = std::vector<Eigen::Vector3i>({
        Eigen::Vector3i(0, 1, 2),
        Eigen::Vector3i(0, 2, 3)
    });
    plane_mesh->ComputeVertexNormals();
    plane_mesh->PaintUniformColor(color);

    return plane_mesh;
}

void LivoxWallHazardDetection::PointCloudCallback(uint32_t handle,
        const uint8_t dev_type,
        LivoxLidarEthernetPacket* data)
{
    if (data == nullptr)  return;

    // printf("data callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
    //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
        LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
        for (uint32_t i = 0; i < data->dot_num; i++) {
            circular_points_.push({
                    p_point_data[i].x / 1000.0f,
                    p_point_data[i].y / 1000.0f,
                    p_point_data[i].z / 1000.0f
                    });
        }
    }

    // if (flat_points_.size() >= 20000) {
    //     // std::cout << "Sending size "<<flat_points_.sizen";
    //     server_.send(flat_points_);
    //     // flat_points_.clear();
    // }

}

void LivoxWallHazardDetection::IMUCloudCallback(uint32_t handle,
            const uint8_t dev_type,
            LivoxLidarEthernetPacket* data)
{

    if (data == nullptr)  return;

    // printf("IMU callback handle:%u, data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
    //     handle, data->dot_num, data->data_type, data->length, data->frame_cnt);
}

void LivoxWallHazardDetection::ProcessPointCloudThread(std::stop_token stoken)
{
    while(!stoken.stop_requested()) {
        std::cout << "PointCloudThread running...\n";
        geometries_.clear();
        auto active_data = circular_points_.get_buffer();
        // auto filtered = active_data | std::views::filter([](const T& val) {
        //         return val > 0; // your condition
        //         });
        //
        // for (const auto& val : filtered) {
        //     // use val safely
        // }
        std::cout << " active_data len " << active_data.size() << " \n";

        if (active_data.size() < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        auto cloud = std::make_shared<open3d::geometry::PointCloud>();
        cloud->points_.reserve(active_data.size());
        for (const auto& p : active_data) {
            cloud->points_.emplace_back(p.x, p.y, p.z);
        }

        double distance_threshold = 0.02; // 2 cm tolerance
        int ransac_n = 3;                 // 3 points to define a plane
        int num_iterations = 1000;

        Eigen::Vector4d plane1_eq;
        std::vector<size_t> inliers1;
        std::tie(plane1_eq, inliers1) = cloud->SegmentPlane(distance_threshold, ransac_n, num_iterations);

        auto plane1_cloud = cloud->SelectByIndex(inliers1);
        plane1_cloud->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0)); // Red for Plane 1

        auto remaining_cloud = cloud->SelectByIndex(inliers1, true); // invert selection
        Eigen::Vector4d plane2_eq;
        std::vector<size_t> inliers2;
        std::tie(plane2_eq, inliers2) = remaining_cloud->SegmentPlane(distance_threshold, ransac_n, num_iterations);

        auto plane2_cloud = remaining_cloud->SelectByIndex(inliers2);
        plane2_cloud->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0)); // Green for Plane 2
        //
        // server_.send(plane1_cloud);
        // server_.send(plane2_cloud);
        std::vector<pcws::Point> out_cloud;
        out_cloud.reserve(plane1_cloud->points_.size() + plane2_cloud->points_.size());
        for (const auto& pt : plane1_cloud->points_) {
            out_cloud.push_back({
                    static_cast<float>(pt.x()),
                    static_cast<float>(pt.y()),
                    static_cast<float>(pt.z())
                    });
        }
        for (const auto& pt : plane2_cloud->points_) {
            out_cloud.push_back({
                    static_cast<float>(pt.x()),
                    static_cast<float>(pt.y()),
                    static_cast<float>(pt.z())
                    });
        }

        server_.send(out_cloud);


        ///// Making the cube
        // Finding the normals
        Eigen::Vector3d n1 = plane1_eq.head<3>().normalized(); // Floor normal
        Eigen::Vector3d n2 = plane2_eq.head<3>().normalized(); // Wall normal
                                                               //
        double dot_val = std::abs(n1.dot(n2));
        bool is_perpendicular = (dot_val < 0.15); // Roughly 90 degrees (+/- ~8 degrees)
        bool is_floor_horizontal = (n1.z() > 0.7); // Floor normal points mostly up

        // geometries_.push_back(cloud);
        geometries_.push_back(plane1_cloud);
        geometries_.push_back(plane2_cloud);
        geometries_.push_back(open3d::geometry::TriangleMesh::CreateCoordinateFrame(0.5,
                                                                    Eigen::Vector3d(0, 0, 0)));

        std::cout << "is_perpendicular " << is_perpendicular << " is_floor_horizontal : " <<is_floor_horizontal <<std::endl;
        //if (is_perpendicular && is_floor_horizontal) {
        if (is_perpendicular ) {
        // Compute the third orthogonal axis (intersection line direction)
            Eigen::Vector3d n3 = n1.cross(n2).normalized();
        //Construct rotation matrix for the oriented bounding box
            Eigen::Matrix3d R;
            R.col(0) = n1;
            R.col(1) = n2;
            R.col(2) = n3;

            Eigen::Vector3d extent(1.0, 1.0, 1.0);
            Eigen::Vector3d center = plane1_cloud->GetCenter() + 0.5 * (n1 + n2);

            open3d::geometry::OrientedBoundingBox cube(center, R, extent);
            cube.color_ = Eigen::Vector3d(0.0, 0.0, 1.0);

            geometries_.push_back(std::make_shared<open3d::geometry::OrientedBoundingBox>(cube));
        }

        auto plane_mesh = CreatePlaneMeshFromCloud(plane1_eq, plane1_cloud);
        geometries_.push_back(plane_mesh);
        // Update existing geometry buffers cleanly
        //vis_->AddGeometry(cloud);
        // vis_->AddGeometry(plane1_cloud);
        // vis_->AddGeometry(plane2_cloud);
        // vis_->AddGeometry(geometries);
        if (vis_initialized_) {
            for (auto geom : geometries_){
                vis_->UpdateGeometry(geom);
            }
        } else {
            vis_->ClearGeometries();
            for (auto geom : geometries_){
                vis_->AddGeometry(geom);
            }
        }
        vis_->PollEvents();
        vis_->UpdateRender();

        std::cout << "Plane 1 points: " << inliers1.size() << " | Plane 2 points: " << inliers2.size() << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Worker stopped safely.\n";
}

void LivoxWallHazardDetection::SpinMainLoop() {
        while (vis_->PollEvents()) {
            {
                // std::lock_guard<std::mutex> lock(mutex_);
            }
            vis_->UpdateRender();
        }
}
