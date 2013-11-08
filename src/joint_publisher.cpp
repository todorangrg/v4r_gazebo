/*
 * Software License Agreement (BSD License)
 *
 *  Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <kdl_parser/kdl_parser.hpp>

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "state_publisher" );
    ros::NodeHandle n;
    ros::NodeHandle n_param("~");

    ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("joint_states",1000);

    tf::TransformBroadcaster broadcaster;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;

    double publish_frequency = 1;
    std::string joint_param;

    n_param.getParam("publish_frequency", publish_frequency);
    n_param.getParam("joints", joint_param);

    boost::erase_all(joint_param, " ");
    std::vector<std::string> joint_string_vector;
    boost::split(joint_string_vector, joint_param, boost::is_any_of(","));


    ros::Rate loop_rate(publish_frequency);

    std::vector<std::string> joints;
    std::vector<double> jointValues;
    for (int32_t i = 0; i < joint_string_vector.size(); i=i+2) {
        joints.push_back(joint_string_vector[i]);
        double value = boost::lexical_cast<double>(joint_string_vector[i+1]);
        jointValues.push_back(value);
        ROS_INFO("Goint to publish: %s %f", joint_string_vector[i].c_str(), value);

    }

    joint_state.name.resize(joints.size());
    joint_state.position.resize(jointValues.size());

    do{
        joint_state.header.stamp = ros::Time::now();

        for(unsigned int i = 0; i < joints.size(); i++){
            joint_state.name[i] = joints[i];
            joint_state.position[i] = jointValues[i];
            ROS_INFO("Publishing: %s %f", joints[i].c_str(), jointValues[i]);
        }



        joint_state_publisher.publish(joint_state);

        loop_rate.sleep();
    } while (ros::ok() && publish_frequency > 0.0);

    return 0;
}
