/*******************************************************
 * Copyright (C) 2025, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "vins_estimator_ros1/rosNode.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace vins_estimator_ros1
{
    class VinsNodeletClass : public nodelet::Nodelet, public vins_multi::VinsNodeBaseClass
    {
        public:
            VinsNodeletClass() {}
        private:
            virtual void onInit() override
            {
                ros::NodeHandle & n = getMTPrivateNodeHandle();
                Init(n);
            }
    };
    PLUGINLIB_EXPORT_CLASS(vins_estimator_ros1::VinsNodeletClass, nodelet::Nodelet);
}
