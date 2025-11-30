#include "TopicData.hpp"
#include "iceoryx_posh/popo/publisher.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    iox::runtime::PoshRuntime::initRuntime("iox_publisher");

    iox::popo::Publisher<TopicData> publisher({"Latency", "Test", "Timestamp"});

    while(true)
    {
        publisher.loan()
            .and_then([&](auto& sample) {
                auto now = std::chrono::system_clock::now();
                auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                
                sample->timestamp_ns = now_ns;
                
                sample.publish();
                
                double now_sec = now_ns / 1e9;
                printf("Publishing at: %.9f\n", now_sec);
            })
            .or_else([](auto& error) {
                std::cerr << "Could not loan sample! Error: " << static_cast<uint64_t>(error) << std::endl;
            });

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
