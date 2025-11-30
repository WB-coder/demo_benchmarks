#include "TopicData.hpp"
#include "iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    iox::runtime::PoshRuntime::initRuntime("iox_subscriber");

    iox::popo::Subscriber<TopicData> subscriber({"Latency", "Test", "Timestamp"});

    while(true)
    {
        subscriber.take()
            .and_then([&](auto& sample) {
                auto now = std::chrono::system_clock::now();
                auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
                
                int64_t latency_ns = now_ns - sample->timestamp_ns;
                double latency_us = latency_ns / 1000.0;
                double now_sec = now_ns / 1e9;

                printf("Received at: %.6f, Latency: %.3f µs\n", now_sec, latency_us);
            })
            .or_else([](auto& error) {
                if (error != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                    std::cerr << "Error receiving chunk: " << static_cast<uint64_t>(error) << std::endl;
                }
            });

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
