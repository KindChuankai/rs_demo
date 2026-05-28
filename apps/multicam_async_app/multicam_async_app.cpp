#include "rs_demo/async_multi_cam/async_multi_cam.hpp"
#include "rs_demo/consumers/consumers.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

int main()
{
    try
    {
        std::vector<std::string> serials = {
            // "213522251541",
            // "213522252775"
            "213522252775"
        };

        AsyncMultiCamProducer producer(serials);
        producer.start();

        std::atomic<bool> running{true};

        std::thread policy_thread(
            policyConsumer,
            std::ref(producer),
            std::ref(running)
        );

        std::thread vis_thread(
            visualizationConsumer,
            std::ref(producer),
            std::ref(running)
        );

        std::thread recorder_thread(
            irVideoRecorderConsumer,
            std::ref(producer),
            std::ref(running),
            std::string("realsense_ir"),
            30.0
        );

        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        producer.stop();

        if (policy_thread.joinable())
            policy_thread.join();

        if (vis_thread.joinable())
            vis_thread.join();

        if (recorder_thread.joinable())
            recorder_thread.join();
    }
    catch (const rs2::error& e)
    {
        std::cerr << "RealSense error: "
                  << e.what()
                  << std::endl;
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Standard exception: "
                  << e.what()
                  << std::endl;
        return 1;
    }

    return 0;
}
