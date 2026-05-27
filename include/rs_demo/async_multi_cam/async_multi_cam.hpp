#pragma once

#include <librealsense2/rs.hpp>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct CameraFrame
{
    std::string serial;
    rs2::frameset frames;
    unsigned long long frame_number = 0;
    double timestamp_ms = 0.0;
};

struct MultiCamFrameBundle
{
    std::size_t sequence_id = 0;
    std::vector<CameraFrame> cameras;
};

class AsyncMultiCamProducer
{
public:
    explicit AsyncMultiCamProducer(const std::vector<std::string>& serials);
    ~AsyncMultiCamProducer();

    AsyncMultiCamProducer(const AsyncMultiCamProducer&) = delete;
    AsyncMultiCamProducer& operator=(const AsyncMultiCamProducer&) = delete;

    void start();
    void stop();

    std::shared_ptr<const MultiCamFrameBundle> getLatest() const;

    std::shared_ptr<const MultiCamFrameBundle> waitForNewer(
        std::size_t last_sequence_id,
        int timeout_ms = 100
    );

private:
    struct CameraSlot;

    void onFrame(CameraSlot* cam, const rs2::frame& frame);
    void tryPublishLatestBundle();

private:
    rs2::context ctx_;
    std::vector<std::unique_ptr<CameraSlot>> cameras_;

    std::atomic<bool> running_{false};

    mutable std::mutex bundle_mtx_;
    std::condition_variable bundle_cv_;

    std::shared_ptr<const MultiCamFrameBundle> latest_bundle_;
    std::size_t sequence_counter_ = 0;
};