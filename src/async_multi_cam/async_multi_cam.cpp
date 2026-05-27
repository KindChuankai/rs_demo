#include "rs_demo/async_multi_cam/async_multi_cam.hpp"

#include <iostream>

struct AsyncMultiCamProducer::CameraSlot
{
    CameraFrame latest_frame;
    rs2::pipeline pipe;

    std::shared_mutex mtx;
    bool has_frame = false;

    CameraSlot(rs2::context& ctx, const std::string& serial_number)
        : latest_frame{serial_number, rs2::frameset{}, 0, 0.0}, pipe(ctx)
    {
    }
};

AsyncMultiCamProducer::AsyncMultiCamProducer(
    const std::vector<std::string>& serials)
{
    for (const auto& serial : serials)
    {
        cameras_.push_back(std::make_unique<CameraSlot>(ctx_, serial));
    }
}

AsyncMultiCamProducer::~AsyncMultiCamProducer()
{
    stop();
}

void AsyncMultiCamProducer::start()
{
    if (running_)
        return;

    running_ = true;

    for (auto& cam : cameras_)
    {
        rs2::config cfg;
        cfg.enable_device(cam->latest_frame.serial);

        // D455 example: IR1 + depth.
        // Change this part according to your real network input.
        cfg.enable_stream(
            RS2_STREAM_INFRARED,
            1,
            640,
            480,
            RS2_FORMAT_Y8,
            30
        );

        cfg.enable_stream(
            RS2_STREAM_DEPTH,
            640,
            480,
            RS2_FORMAT_Z16,
            30
        );

        CameraSlot* cam_ptr = cam.get();

        cam->pipe.start(cfg, [this, cam_ptr](const rs2::frame& frame)
        {
            this->onFrame(cam_ptr, frame);
        });

        std::cout << "Started camera: " << cam->latest_frame.serial << std::endl;
    }
}

void AsyncMultiCamProducer::stop()
{
    if (!running_)
        return;

    running_ = false;

    for (auto& cam : cameras_)
    {
        try
        {
            cam->pipe.stop();
            std::cout << "Stopped camera: " << cam->latest_frame.serial << std::endl;
        }
        catch (const rs2::error& e)
        {
            std::cerr << "Failed to stop camera "
                      << cam->latest_frame.serial
                      << ": "
                      << e.what()
                      << std::endl;
        }
    }

    bundle_cv_.notify_all();
}

std::shared_ptr<const MultiCamFrameBundle>
AsyncMultiCamProducer::getLatest() const
{
    std::lock_guard<std::mutex> lock(bundle_mtx_);
    return latest_bundle_;
}

std::shared_ptr<const MultiCamFrameBundle>
AsyncMultiCamProducer::waitForNewer(
    std::uint64_t last_sequence_id,
    int timeout_ms)
{
    std::unique_lock<std::mutex> lock(bundle_mtx_);

    // !running_ is used as part of the condition to avoid potential deadlock when stopping the producer while waiting.
    bundle_cv_.wait_for(
        lock,
        std::chrono::milliseconds(timeout_ms),
        [&]()
        {
            return !running_ ||
                   (latest_bundle_ &&
                    latest_bundle_->sequence_id > last_sequence_id);
        }
    );

    if (!running_)
        return nullptr;

    if (!latest_bundle_)
        return nullptr;

    if (latest_bundle_->sequence_id <= last_sequence_id)
        return nullptr;

    // if (!running_ &&
    // (!latest_bundle_ ||
    // latest_bundle_->sequence_id <= last_sequence_id))
    // {
    //     return nullptr;
    // }

    return latest_bundle_;
}

void AsyncMultiCamProducer::onFrame(
    CameraSlot* cam,
    const rs2::frame& frame)
{
    #ifdef DEBUG
    std::cout << "[RealsenseCallback] camera "
              << cam->latest_frame.serial
              << ", stream = "
              << frame.get_profile().stream_name()
              << ", frame number = "
              << frame.get_frame_number()
              << std::endl;
    #endif

    // frame itself is a handle-like object, so we can just move it around without copying the underlying data.
    // the behavior of handle-like object is similar to shared_ptr, but with custom copy/move semantics.
    // handle-like object is a lightweight wrapper/object and you can think of it has a shared_ptr inside.
    // so is the frameset, which inherits from frame and also a handle-like object.
    auto fs = frame.as<rs2::frameset>();

    if (!fs)
    {
        std::cout << "[RealsenseCallback] not a frameset, stream = "
                  << frame.get_profile().stream_name()
                  << std::endl;
        return;
    }
    #ifdef DEBUG
    std::cout << "[RealsenseCallback] got frameset from camera "
              << cam->latest_frame.serial
              << std::endl;
    #endif

    auto ir = fs.get_infrared_frame(1);

    if (!ir)
    {
        std::cout << "[RealsenseCallback] frameset has no IR1 frame for camera "
                  << cam->latest_frame.serial
                  << std::endl;

        for (const auto& f : fs)
        {
            std::cout << "  contained stream: "
                      << f.get_profile().stream_name()
                      << ", index = "
                      << f.get_profile().stream_index()
                      << std::endl;
        }
        return;
    }

    {
        std::unique_lock<std::shared_mutex> lock(cam->mtx, std::try_to_lock);

        if (!lock.owns_lock())
        {
            std::cout << "[RealsenseCallback] camera buffer busy, drop frame "
                      << cam->latest_frame.serial
                      << std::endl;
            return;
        }

        cam->latest_frame.frames = fs;
        cam->has_frame = true;
        cam->latest_frame.frame_number = ir.get_frame_number();
        cam->latest_frame.timestamp_ms = ir.get_timestamp();
    }

    #ifdef DEBUG
    std::cout << "[RealsenseCallback] stored latest frame for camera "
              << cam->latest_frame.serial
              << ", frame number = "
              << cam->latest_frame.frame_number
              << ", timestamp = "
              << cam->latest_frame.timestamp_ms
              << std::endl;
    #endif

    tryPublishLatestBundle();
}

void AsyncMultiCamProducer::tryPublishLatestBundle()
{
    auto bundle = std::make_shared<MultiCamFrameBundle>();

    {
        std::vector<std::shared_lock<std::shared_mutex>> locks;
        locks.reserve(cameras_.size());

        for (auto& cam : cameras_)
        {
            locks.emplace_back(cam->mtx, std::try_to_lock);

            if (!locks.back().owns_lock())
            {
                std::cout << "[RealSensePublish] could not lock camera "
                          << cam->latest_frame.serial
                          << std::endl;
                return;
            }

            if (!cam->has_frame)
            {
                std::cout << "[RealSensePublish] camera has no frame yet: "
                          << cam->latest_frame.serial
                          << std::endl;
                return;
            }
        }

        bundle->cameras.reserve(cameras_.size());

        for (auto& cam : cameras_)
        {
            bundle->cameras.push_back(cam->latest_frame);
        }
    }

    {
        std::lock_guard<std::mutex> lock(bundle_mtx_);

        bundle->sequence_id = ++sequence_id_;
        latest_bundle_ = bundle;
    }

    #ifdef DEBUG
    std::cout << "[RealSensePublish] new bundle sequence "
              << bundle->sequence_id
              << std::endl;
    #endif

    bundle_cv_.notify_all();
}
