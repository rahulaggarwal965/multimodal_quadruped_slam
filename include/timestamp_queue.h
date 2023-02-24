#pragma once

#include <vector>

#include <ros/ros.h>

struct TimestampQueue {

    std::vector<double> timestamps;

    const inline int find_closest(const double time) {
        // TODO(rahul): note that this is not exactly correct. This only finds the first element greater than `time`.
        // A proper implementation would also check the element right before the upper bound to see if that is closer
        // We do this for now to maintain some consistency (we always go to the upper bound for the factor graph)
        return std::distance(timestamps.cbegin(), std::upper_bound(timestamps.cbegin(), timestamps.cend(), time, [](double a, double b) {return a <= b;}));
    }

    const inline int find_closest(const ros::Time &time) {
        return find_closest(time.toSec());
    }

    inline void push(const double time) {
        auto index = find_closest(time);
        timestamps.insert(timestamps.cbegin() + index, time);
    }

    inline void push(const ros::Time &time) {
        push(time.toSec());
    }
};
