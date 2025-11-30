#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <chrono>
#include "lcm_demo/timestamp_t.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const lcm_demo::timestamp_t* msg)
        {
            auto now = std::chrono::system_clock::now();
            auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            
            int64_t latency_ns = now_ns - msg->timestamp_ns;
            double latency_us = latency_ns / 1000.0;
            double now_sec = now_ns / 1e9;

            printf("Received at: %.6f, Latency: %.3f µs\n", now_sec, latency_us);
        }
};

int main()
{
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    Handler handlerObject;
    lcm.subscribe("LATENCY_TEST", &Handler::handleMessage, &handlerObject);

    while(0 == lcm.handle());

    return 0;
}
