#include <ros/ros.h>
#include "optimizer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_estimation");


    Optimizer optimizer;

    // single-threaded
    ros::spin();
    return 0;
}
