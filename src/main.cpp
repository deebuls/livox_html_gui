#include <thread>
#include <iostream>

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include "pointcloud_server.hpp"
#include "livox_wall_hazard_detection.h"
#include "livox_manager.h"

#include <atomic>
#include <csignal>
#include <chrono>

// Global flag to catch Ctrl+C
std::atomic<bool> KEEP_RUNNING{true};

void handle_sigint(int) {
    KEEP_RUNNING = false;
}

int main(int argc, char** argv) {

    if (argc != 2) {
        printf("Params Invalid, must input config path.\n");
        return -1;
    }
    const std::string path = argv[1];

    std::signal(SIGINT, handle_sigint);

    auto manager = LivoxManager(path);
    auto detector = LivoxWallHazardDetection(manager);
    // std::jthread automatically passes a std::stop_token as the first argument
    std::jthread t(&LivoxWallHazardDetection::ProcessPointCloudThread, &detector);

    //detector.SpinMainLoop();
    // // Keep main alive until Ctrl+C sets keep_running to false
    while (KEEP_RUNNING) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Signal received. Cleaning up...\n";
    t.request_stop();

    return 0;
}
