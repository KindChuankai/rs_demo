#pragma once

#include "rs_demo/async_multi_cam/async_multi_cam.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <string>

cv::Mat irFrameToMatClone(const rs2::video_frame& ir_frame);

void policyConsumer(
    AsyncMultiCamProducer& producer,
    std::atomic<bool>& running
);

void visualizationConsumer(
    AsyncMultiCamProducer& producer,
    std::atomic<bool>& running
);

void irVideoRecorderConsumer(
    AsyncMultiCamProducer& producer,
    std::atomic<bool>& running,
    const std::string& output_prefix,
    double fps = 30.0
);
