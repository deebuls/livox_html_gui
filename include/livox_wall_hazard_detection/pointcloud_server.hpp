// pointcloud_server.hpp  -  header-only point cloud WebSocket server
// (C++11, needs libwebsockets).
//
//   #include "pointcloud_server.hpp"
//
//   pcws::Server server(9002);
//   server.start();                              // runs in a background thread
//   std::vector<pcws::Point> cloud = {{0,0,0}, {1,2,3}};
//   server.send(cloud);                          // thread-safe, returns at once
//
// Wire format: ONE binary WebSocket message per send(), containing the points
// as interleaved little-endian float32: x0 y0 z0 x1 y1 z1 ...
// New clients automatically receive the most recent cloud when they connect.

#pragma once

#include <libwebsockets.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace pcws {

struct Point {
    float x, y, z;
};

static_assert(sizeof(Point) == 3 * sizeof(float), "Point must be tightly packed");
static_assert(std::is_standard_layout<Point>::value, "Point must be standard layout");

class Server {
public:
    explicit Server(int port = 9002) : port_(port) {
        std::memset(protocols_, 0, sizeof(protocols_));
        protocols_[0].name     = "pointcloud";
        protocols_[0].callback = &Server::callback;
        protocols_[0].per_session_data_size = sizeof(Session);
        // protocols_[1] stays zeroed = list terminator
    }

    ~Server() { stop(); }

    Server(const Server&)            = delete;
    Server& operator=(const Server&) = delete;

    // Starts listening and spawns the service thread. Returns false on failure.
    bool start() {
        if (running_) return true;

        lws_set_log_level(LLL_ERR | LLL_WARN, nullptr);

        lws_context_creation_info info;
        std::memset(&info, 0, sizeof(info));
        info.port      = port_;
        info.protocols = protocols_;
        info.user      = this;                    // reachable from the callback

        ctx_ = lws_create_context(&info);
        if (!ctx_) {
            std::fprintf(stderr, "pcws: could not start server on port %d\n", port_);
            return false;
        }

        running_ = true;
        thread_  = std::thread([this] {
            while (running_) lws_service(ctx_, 50);
        });
        return true;
    }

    // Stops the service thread and closes all connections.
    void stop() {
        if (!running_ && !ctx_) return;
        running_ = false;
        if (ctx_) lws_cancel_service(ctx_);       // wake the service loop
        if (thread_.joinable()) thread_.join();
        if (ctx_) { lws_context_destroy(ctx_); ctx_ = nullptr; }
    }

    // Broadcast a point cloud to every connected client. Thread-safe and
    // non-blocking: if a client is still busy with the previous cloud it simply
    // skips ahead to this newer one. May be called before start().
    void send(const std::vector<Point>& points) {
        auto frame = std::make_shared<std::vector<unsigned char>>(
            LWS_PRE + points.size() * sizeof(Point));          // LWS_PRE = header room
        if (!points.empty())
            std::memcpy(frame->data() + LWS_PRE, points.data(),
                        points.size() * sizeof(Point));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            frame_ = std::move(frame);
            ++version_;
        }
        if (ctx_) lws_cancel_service(ctx_);       // -> EVENT_WAIT_CANCELLED in service thread
    }

    int  port()        const { return port_; }
    int  clientCount() const { return clients_; }
    bool running()     const { return running_; }

private:
    struct Session { uint64_t sent_version; };

    static int callback(lws* wsi, lws_callback_reasons reason,
                        void* user, void* /*in*/, size_t /*len*/) {
        Server*  self = static_cast<Server*>(lws_context_user(lws_get_context(wsi)));
        Session* sess = static_cast<Session*>(user);
        if (!self) return 0;

        switch (reason) {

        case LWS_CALLBACK_ESTABLISHED:
            sess->sent_version = 0;
            ++self->clients_;
            lws_callback_on_writable(wsi);        // give newcomer the latest cloud
            break;

        case LWS_CALLBACK_CLOSED:
            --self->clients_;
            break;

        case LWS_CALLBACK_EVENT_WAIT_CANCELLED:   // raised by lws_cancel_service()
            lws_callback_on_writable_all_protocol(lws_get_context(wsi),
                                                  &self->protocols_[0]);
            break;

        case LWS_CALLBACK_SERVER_WRITEABLE: {
            std::shared_ptr<std::vector<unsigned char>> frame;
            uint64_t version;
            {
                std::lock_guard<std::mutex> lock(self->mutex_);
                frame   = self->frame_;
                version = self->version_;
            }
            if (!frame || version == sess->sent_version) break;   // nothing new

            int n = lws_write(wsi, frame->data() + LWS_PRE,
                              frame->size() - LWS_PRE, LWS_WRITE_BINARY);
            if (n < 0) return -1;                 // error: close this connection
            sess->sent_version = version;
            break;
        }

        default:
            break;
        }
        return 0;
    }

    int               port_;
    lws_context*      ctx_ = nullptr;
    lws_protocols     protocols_[2];
    std::thread       thread_;
    std::atomic<bool> running_{false};
    std::atomic<int>  clients_{0};

    std::mutex                                   mutex_;
    std::shared_ptr<std::vector<unsigned char>>  frame_;     // latest cloud (+LWS_PRE)
    uint64_t                                     version_ = 0;
};

} // namespace pcws
