#pragma once

#include <chrono>

class LoopTime
{
public:
    /// update & return the duration(span) of loop time
    double update();
    /// return the duration(span) of loop time
    double duration();
    std::chrono::steady_clock::time_point current();
private:
    double duration_ = 0;
    std::chrono::steady_clock::time_point preTime_;
    std::chrono::steady_clock::time_point currentTime_;
    bool b_loop_first = true;
};

inline double LoopTime::update()
{
    using namespace std::chrono;
    if (b_loop_first) {
        preTime_ = steady_clock::now();

        b_loop_first = false;
        return -1;
    }

    currentTime_ = steady_clock::now();
    duration_ = std::chrono::duration<double>(currentTime_ - preTime_).count(); // seconds
    preTime_ = currentTime_;

    return duration_;
}

inline double LoopTime::duration()
{
    return duration_;
}

inline std::chrono::steady_clock::time_point LoopTime::current()
{
    return currentTime_;
}
