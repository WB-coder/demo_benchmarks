#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include "lcm_demo/timestamp_t.hpp"

int main()
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    while(true)
    {
        lcm_demo::timestamp_t msg;
        auto now = std::chrono::system_clock::now();
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        
        msg.timestamp_ns = now_ns;

        lcm.publish("LATENCY_TEST", &msg);
        
        // Convert to seconds for display to match ROS 2 output style roughly
        double now_sec = now_ns / 1e9;
        printf("Publishing at: %.9f\n", now_sec);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
