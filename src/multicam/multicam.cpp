#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

// =======================================================
// Helper: downstream task placeholder
// =======================================================
void downstreamTask(const std::vector<cv::Mat>& ir_images)
{
    // ir_images[0] = newest image from camera 0
    // ir_images[1] = newest image from camera 1
    //
    // Replace this with your neural network / robot perception code.
    //
    // Example:
    // network.forward(ir_images);

    for (size_t i = 0; i < ir_images.size(); ++i)
    {
        cv::imshow("Camera " + std::to_string(i) + " IR", ir_images[i]);
    }
}

// =======================================================
// Latest frame buffer for one camera
// =======================================================
struct LatestFrameBuffer
{
    std::mutex mtx;
    rs2::frameset latest_frames;
    bool has_frame = false;
    unsigned long long frame_number = 0;
};

// =======================================================
// One camera node
// =======================================================
struct CameraNode
{
    std::string serial;
    rs2::pipeline pipe;
    LatestFrameBuffer buffer;

    CameraNode(rs2::context& ctx, const std::string& serial_number)
        : serial(serial_number), pipe(ctx)
    {
    }
};

// =======================================================
// Async multi-camera RealSense wrapper
// =======================================================
class AsyncMultiRealsense
{
public:
    AsyncMultiRealsense(const std::vector<std::string>& serials)
    {
        for (const auto& serial : serials)
        {
            cameras_.push_back(std::make_unique<CameraNode>(ctx_, serial));
            last_used_frame_numbers_.push_back(0);
        }
    }

    void start()
    {
        for (auto& cam : cameras_)
        {
            rs2::config cfg;

            cfg.enable_device(cam->serial);

            // For D455 infrared + depth.
            // If your downstream network only needs IR, you may remove depth.
            cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH,       640, 480, RS2_FORMAT_Z16, 30);

            CameraNode* cam_ptr = cam.get();

            cam->pipe.start(cfg, [cam_ptr](const rs2::frame& frame)
            {
                auto fs = frame.as<rs2::frameset>();
                if (!fs)
                    return;

                auto ir = fs.get_infrared_frame(1);
                if (!ir)
                    return;

                // Try to lock. If the main loop is reading, drop this frame.
                std::unique_lock<std::mutex> lock(
                    cam_ptr->buffer.mtx,
                    std::try_to_lock
                );

                if (!lock.owns_lock())
                    return;

                cam_ptr->buffer.latest_frames = fs;
                cam_ptr->buffer.frame_number = ir.get_frame_number();
                cam_ptr->buffer.has_frame = true;
            });

            std::cout << "Started camera: " << cam->serial << std::endl;
        }

        running_ = true;
    }

    bool getLatestIRImages(std::vector<cv::Mat>& ir_images)
    {
        ir_images.clear();

        std::vector<rs2::frameset> local_frames;
        std::vector<unsigned long long> current_frame_numbers;

        local_frames.reserve(cameras_.size());
        current_frame_numbers.reserve(cameras_.size());

        {
            std::vector<std::unique_lock<std::mutex>> locks;
            locks.reserve(cameras_.size());

            // Try to lock all camera buffers.
            // If any one is busy, skip this loop.
            for (auto& cam : cameras_)
            {
                locks.emplace_back(cam->buffer.mtx, std::try_to_lock);

                if (!locks.back().owns_lock())
                    return false;
            }

            // Check every camera has at least one frame.
            for (auto& cam : cameras_)
            {
                if (!cam->buffer.has_frame)
                    return false;
            }

            // Copy frame handles quickly.
            // RealSense frames are reference-counted, so this is cheap.
            for (auto& cam : cameras_)
            {
                local_frames.push_back(cam->buffer.latest_frames);
                current_frame_numbers.push_back(cam->buffer.frame_number);
            }
        }

        // Avoid processing exactly the same set repeatedly.
        bool has_new_frame = false;

        for (size_t i = 0; i < current_frame_numbers.size(); ++i)
        {
            if (current_frame_numbers[i] != last_used_frame_numbers_[i])
            {
                has_new_frame = true;
                break;
            }
        }

        if (!has_new_frame)
            return false;

        last_used_frame_numbers_ = current_frame_numbers;

        // Convert RealSense frames to OpenCV images.
        for (auto& fs : local_frames)
        {
            rs2::video_frame ir_frame = fs.get_infrared_frame(1);

            if (!ir_frame)
                return false;

            cv::Mat ir_raw(
                cv::Size(ir_frame.get_width(), ir_frame.get_height()),
                CV_8UC1,
                (void*)ir_frame.get_data(),
                cv::Mat::AUTO_STEP
            );

            // Clone so the image is safe for downstream processing.
            ir_images.push_back(ir_raw.clone());
        }

        return true;
    }

    void stop()
    {
        if (!running_)
            return;

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

        running_ = false;
    }

    ~AsyncMultiRealsense()
    {
        stop();
    }

private:
    rs2::context ctx_;
    std::vector<std::unique_ptr<CameraNode>> cameras_;
    std::vector<unsigned long long> last_used_frame_numbers_;
    bool running_ = false;
};

// =======================================================
// Main program
// =======================================================
int main()
{
    try
    {
        std::vector<std::string> serials = {
            // "213522251541",
            "213522252775"
        };

        AsyncMultiRealsense cameras(serials);

        cameras.start();

        while (true)
        {
            std::vector<cv::Mat> ir_images;

            if (cameras.getLatestIRImages(ir_images))
            {
                downstreamTask(ir_images);
            }

            int key = cv::waitKey(1);
            if (key == 27) // ESC
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        cameras.stop();
    }
    catch (const rs2::error& e)
    {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Standard exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}