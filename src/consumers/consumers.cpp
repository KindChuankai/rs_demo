#include "rs_demo/consumers/consumers.hpp"

#include <iostream>

cv::Mat irFrameToMatClone(const rs2::video_frame& ir_frame)
{
    cv::Mat ir_raw(
        cv::Size(ir_frame.get_width(), ir_frame.get_height()),
        CV_8UC1,
        (void*)ir_frame.get_data(),
        cv::Mat::AUTO_STEP
    );

    // Clone because cv::Mat above only points to RealSense memory.
    return ir_raw.clone();
}

void policyConsumer(
    AsyncMultiCamProducer& producer,
    std::atomic<bool>& running)
{
    std::size_t last_seq = 0;

    while (running)
    {
        auto bundle = producer.waitForNewer(last_seq, 100);

        if (!bundle)
            continue;

        last_seq = bundle->sequence_id;

        std::vector<cv::Mat> ir_images;
        ir_images.reserve(bundle->cameras.size());

        for (const auto& cam : bundle->cameras)
        {
            auto ir = cam.frames.get_infrared_frame(1);
            if (!ir)
                continue;

            ir_images.push_back(irFrameToMatClone(ir));
        }

        if (ir_images.size() != bundle->cameras.size())
            continue;

        // Your network / robot policy goes here.
        //
        // Example:
        // action = policy.forward(ir_images);
        // robot.sendAction(action);

        std::cout << "[Policy] sequence "
                  << bundle->sequence_id
                  << ", images = "
                  << ir_images.size()
                  << std::endl;
    }
}

void visualizationConsumer(
    AsyncMultiCamProducer& producer,
    std::atomic<bool>& running)
{
    std::size_t last_seq = 0;

    while (running)
    {
        auto bundle = producer.waitForNewer(last_seq, 100);

        if (!bundle)
            continue;

        last_seq = bundle->sequence_id;

        for (std::size_t i = 0; i < bundle->cameras.size(); ++i)
        {
            auto ir = bundle->cameras[i].frames.get_infrared_frame(1);
            if (!ir)
                continue;

            cv::Mat ir_img = irFrameToMatClone(ir);

            cv::Mat vis;
            cv::equalizeHist(ir_img, vis);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);

            cv::imshow("Camera " + std::to_string(i) + " IR", vis);
        }

        int key = cv::waitKey(1);

        if (key == 27)
        {
            running = false;
            break;
        }
    }
}