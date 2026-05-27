#include "rs_demo/async_multi_cam/async_multi_cam.hpp"

#include <iostream>

struct AsyncMultiCamProducer::CameraSlot
{
    std::string serial;
    rs2::pipeline pipe;

    std::mutex mtx;
    rs2::frameset latest_frames;
    bool has_frame = false;
    unsigned long long frame_number = 0;
    double timestamp_ms = 0.0;

    CameraSlot(rs2::context& ctx, const std::string& serial_number)
        : serial(serial_number), pipe(ctx)
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
        cfg.enable_device(cam->serial);

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

        std::cout << "Started camera: " << cam->serial << std::endl;
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
            std::cout << "Stopped camera: " << cam->serial << std::endl;
        }
        catch (const rs2::error& e)
        {
            std::cerr << "Failed to stop camera "
                      << cam->serial
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
    std::size_t last_sequence_id,
    int timeout_ms)
{
    std::unique_lock<std::mutex> lock(bundle_mtx_);

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

    if (!latest_bundle_)
        return nullptr;

    if (latest_bundle_->sequence_id <= last_sequence_id)
        return nullptr;

    return latest_bundle_;
}

void AsyncMultiCamProducer::onFrame(
    CameraSlot* cam,
    const rs2::frame& frame)
{
    std::cout << "[Callback] camera "
              << cam->serial
              << ", stream = "
              << frame.get_profile().stream_name()
              << ", frame number = "
              << frame.get_frame_number()
              << std::endl;

    auto fs = frame.as<rs2::frameset>();

    if (!fs)
    {
        std::cout << "[Callback] not a frameset, stream = "
                  << frame.get_profile().stream_name()
                  << std::endl;
        return;
    }

    std::cout << "[Callback] got frameset from camera "
              << cam->serial
              << std::endl;

    auto ir = fs.get_infrared_frame(1);

    if (!ir)
    {
        std::cout << "[Callback] frameset has no IR1 frame for camera "
                  << cam->serial
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
        std::unique_lock<std::mutex> lock(cam->mtx, std::try_to_lock);

        if (!lock.owns_lock())
        {
            std::cout << "[Callback] camera buffer busy, drop frame "
                      << cam->serial
                      << std::endl;
            return;
        }

        cam->latest_frames = fs;
        cam->has_frame = true;
        cam->frame_number = ir.get_frame_number();
        cam->timestamp_ms = ir.get_timestamp();
    }

    std::cout << "[Callback] stored latest frame for camera "
              << cam->serial
              << std::endl;

    tryPublishLatestBundle();
}

void AsyncMultiCamProducer::tryPublishLatestBundle()
{
    auto bundle = std::make_shared<MultiCamFrameBundle>();

    {
        std::vector<std::unique_lock<std::mutex>> locks;
        locks.reserve(cameras_.size());

        for (auto& cam : cameras_)
        {
            locks.emplace_back(cam->mtx, std::try_to_lock);

            if (!locks.back().owns_lock())
            {
                std::cout << "[Publish] could not lock camera "
                          << cam->serial
                          << std::endl;
                return;
            }
        }

        for (auto& cam : cameras_)
        {
            if (!cam->has_frame)
            {
                std::cout << "[Publish] camera has no frame yet: "
                          << cam->serial
                          << std::endl;
                return;
            }
        }

        bundle->cameras.reserve(cameras_.size());

        for (auto& cam : cameras_)
        {
            CameraFrame cf;
            cf.serial = cam->serial;
            cf.frames = cam->latest_frames;
            cf.frame_number = cam->frame_number;
            cf.timestamp_ms = cam->timestamp_ms;

            bundle->cameras.push_back(cf);
        }
    }

    {
        std::lock_guard<std::mutex> lock(bundle_mtx_);

        bundle->sequence_id = ++sequence_counter_;
        latest_bundle_ = bundle;
    }

    std::cout << "[Publish] new bundle sequence "
              << bundle->sequence_id
              << std::endl;

    bundle_cv_.notify_all();
}