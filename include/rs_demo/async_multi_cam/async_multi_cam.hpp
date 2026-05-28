#pragma once

#include <librealsense2/rs.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

struct CameraFrame
{
    std::string serial;
    rs2::frameset frames;
    std::uint64_t frame_number = 0;
    double timestamp_ms = 0.0;
};

struct MultiCamFrameBundle
{
    std::uint64_t sequence_id = 0;
    std::vector<CameraFrame> cameras;
};

enum class WaitForNewerStatus
{
    NewFrame,
    Timeout,
    Stopped
};

struct WaitForNewerResult
{
    WaitForNewerStatus status = WaitForNewerStatus::Timeout;
    std::shared_ptr<const MultiCamFrameBundle> bundle;
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

    WaitForNewerResult waitForNewer(
        std::uint64_t last_sequence_id,
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
    std::uint64_t sequence_id_ = 0;
};
