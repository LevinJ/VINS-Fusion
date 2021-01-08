#pragma once
#include "modules/Common/include/easylogging++.h"
#include "lcm/lcm-cpp.hpp"
#include "modules/message/lcm/vslam/HEADER.hpp"
#include "modules/message/lcm/vslam/Imu.hpp"
#include "modules/message/lcm/vslam/Image.hpp"
#include "modules/BasicIpc/include/channelopt.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "../vslam_interface.h"

class Handler
{
private:
    /* data */
public:
    Handler(/* args */);
    ~Handler();

    void handleMessageImu(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vslam::Imu* msg);
    void handleMessageImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vslam::Image* msg);
};
