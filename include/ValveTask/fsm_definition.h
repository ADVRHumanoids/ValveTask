/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Pietro Balatti
 * email: pietro.balattis@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/
#include <XBotInterface/StateMachine.h>
#include<iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <ADVR_ROS/advr_segment_control.h>
#include <ADVR_ROS/advr_grasp_control_srv.h>
#include <ADVR_ROS/im_pose_msg.h>
#include<eigen_conversions/eigen_msg.h>

#include <trajectory_utils/segment.h>
#include <trajectory_utils/Cartesian.h>

#include <XBotCore-interfaces/XDomainCommunication.h>

#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <geometry_msgs/WrenchStamped.h>
#include <XCM/XBotPluginStatus.h>

#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>


#define TRAJ_DURATION 5
#define APPROCHING_SHIFT 0.1
#define RETREAT_SHIFT 0.1
#define VALVE_RADIUSE 0.2

#define CENTER_SHIFT 0.2


namespace myfsm {

    inline double RADTODEG(double x) { return x * 180.0 / M_PI; };

    inline double DEGTORAD(double x) { return x * M_PI / 180.0; };


    inline Eigen::Quaterniond zyx2quat(double x, double y, double z) {
        Eigen::Matrix3d rot;
        rot =
                Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX());
        return Eigen::Quaterniond(rot).normalized();
    }

/*Example how to define a custom Event*/
/*  class MyEvent : public XBot::FSM::Event {

    public:

      MyEvent(int id): id(id) {}

      int id;

    };
*/

/*Example how to define a custom Message*/
/*  class MyMessage : public XBot::FSM::Message {

    public:

      MyMessage (int id):id(id){};

      int id;

    };
*/

    class tfHandler {
    public:
        tfHandler() :
                _listener(), _gm_transform(), _transform() {

        }

        bool getTransformTf(const std::string &parent, const std::string &child, Eigen::Affine3d &world_T_bl) {
            try {
                ros::Time now = ros::Time::now();
                //if(_listener.waitForTransform(child, parent, now, ros::Duration(5.0)))
                //{
                _listener.lookupTransform(child, parent, ros::Time(0), _transform);

                tf::transformTFToMsg(_transform, _gm_transform);
                tf::transformMsgToEigen(_gm_transform, world_T_bl);

                return true;
                //}
// 	  else
// 	    return false;
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
                return false;
            }
        }

    private:
        tf::TransformListener _listener;
        geometry_msgs::Transform _gm_transform;
        tf::StampedTransform _transform;

    };

    struct SharedData {

        XBot::RobotInterface::Ptr _robot;
        std::shared_ptr<ros::NodeHandle> _nh;
        geometry_msgs::PoseStamped::ConstPtr _debris_pose;
        geometry_msgs::PoseStamped::Ptr _valve_pose;

        std_msgs::String::ConstPtr _debris_number;
        std_msgs::String::ConstPtr _hand_selection;

        bool no_hand_selection = true;

        ros::ServiceClient _client;
        XBot::SubscriberRT<XBot::Command> command;
        std::shared_ptr<XBot::Command> current_command;
        ros::Publisher _grasp_mag_pub_LSoftHand;
        ros::Publisher _grasp_mag_pub_RSoftHand;
        geometry_msgs::WrenchStamped::ConstPtr _ft_r_arm;
        double _w_F_ft_initial;
        bool _feedback;
        ros::Publisher _SoftHandPose_pub;
        geometry_msgs::PoseStamped::ConstPtr _last_pose;
        geometry_msgs::PoseStamped::ConstPtr _initial_pose_right_hand;
        geometry_msgs::PoseStamped::ConstPtr _initial_pose_left_hand;
        geometry_msgs::PoseStamped::ConstPtr _last_pose_left_hand;
        ros::ServiceClient _grasp_client;

        bool _hand_over_phase;
        std::shared_ptr<XBot::PluginStatus> plugin_status;



        //////////////////////////////////////////////////////////////////////////////////////////////////////
        //// valve task
        //////////////////////////////////////////////////////////////////////////////////////////////////////


        XBot::RobotInterface::Ptr robot_;
        tfHandler tf_;
        std::string selectedHand_;

        std::string floating_base_link_name_;
        Eigen::Affine3d world_T_floating_base_;


        Eigen::Affine3d left_hand_pose_Affine_, right_hand_pose_Affine_;
        geometry_msgs::PoseStamped left_hand_pose_PoseStamped_, right_hand_pose_PoseStamped_;


        Eigen::Affine3d left_hand_pose_home_Affine_, right_hand_pose_home_Affine_;
        geometry_msgs::PoseStamped left_hand_pose_home_PoseStamped_, right_hand_pose_home_PoseStamped_;


        void updateRobotStates() {
            // update robot states
            robot_->sense();
            robot_->model().getFloatingBaseLink(floating_base_link_name_);
            tf_.getTransformTf(floating_base_link_name_, "world_odom", world_T_floating_base_);
            robot_->model().setFloatingBasePose(world_T_floating_base_);
            robot_->model().update();

            // calculate needed information
            robot_->model().getPose("LSoftHand", left_hand_pose_Affine_);
            robot_->model().getPose("RSoftHand", right_hand_pose_Affine_);

            tf::poseEigenToMsg(left_hand_pose_Affine_, left_hand_pose_PoseStamped_.pose);
            tf::poseEigenToMsg(right_hand_pose_Affine_, right_hand_pose_PoseStamped_.pose);

        }


        bool home_recoreded_ = false;

        void recordHome() {
            std::cout << "Home Recorded!" << std::endl;
            left_hand_pose_home_Affine_ = left_hand_pose_Affine_;
            right_hand_pose_home_Affine_ = right_hand_pose_Affine_;

            left_hand_pose_home_PoseStamped_ = left_hand_pose_PoseStamped_;
            right_hand_pose_home_PoseStamped_ = right_hand_pose_PoseStamped_;

        }


        // define a bunch of key poses based on valve model pose
        geometry_msgs::PoseStamped valve_pose_;
        geometry_msgs::PoseStamped valve_approach_pose_;
        geometry_msgs::PoseStamped valve_turned_pose_;
        geometry_msgs::PoseStamped valve_retreat_pose_;

        geometry_msgs::PoseStamped last_pose_PoseStamped_;


        Eigen::Affine3d valve_Affine_;
        Eigen::Affine3d valve_approach_Affine_;
        Eigen::Affine3d valve_turned_Affine_;
        Eigen::Affine3d valve_retreat_Affine_;


        // valve model parameters needed to calculate those key poses
        double handel_length_ = 0.3;
        Eigen::Vector3d valve_center_position_wrt_base_ = Eigen::Vector3d(0.1, 0.0, 1.2);


        void calcValveKeyPoses() {


            // transform all pose to affine
            valve_Affine_ = Eigen::Affine3d::Identity();
            valve_approach_Affine_ = Eigen::Affine3d::Identity();
            valve_turned_Affine_ = Eigen::Affine3d::Identity();
            valve_retreat_Affine_ = Eigen::Affine3d::Identity();

            tf::poseMsgToEigen(valve_pose_.pose, valve_Affine_);
            tf::poseMsgToEigen(valve_approach_pose_.pose, valve_approach_Affine_);
            tf::poseMsgToEigen(valve_turned_pose_.pose, valve_turned_Affine_);
            tf::poseMsgToEigen(valve_retreat_pose_.pose, valve_retreat_Affine_);

            // calculate all affine
            // valve_approach
            Eigen::Affine3d approach_shift = Eigen::Affine3d::Identity();
            approach_shift.translation() = Eigen::Vector3d(0.0, -APPROCHING_SHIFT, 0.0);
            valve_approach_Affine_ = valve_Affine_ * approach_shift;

            // valve_turned
            Eigen::Affine3d turn = Eigen::Affine3d::Identity();
            turn.translate(Eigen::Vector3d(-VALVE_RADIUSE, VALVE_RADIUSE, 0.0));
            turn.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
            valve_turned_Affine_ = valve_Affine_ * turn;

            // valve_retreat
            Eigen::Affine3d retreat_shift = Eigen::Affine3d::Identity();
            retreat_shift.translation() = Eigen::Vector3d(0.0, -RETREAT_SHIFT, 0.0);
            valve_retreat_Affine_ = valve_turned_Affine_ * retreat_shift;


            // transfer data back
            tf::poseEigenToMsg(valve_approach_Affine_, valve_approach_pose_.pose);
            tf::poseEigenToMsg(valve_turned_Affine_, valve_turned_pose_.pose);
            tf::poseEigenToMsg(valve_retreat_Affine_, valve_retreat_pose_.pose);

        }


    };

    class MacroState : public XBot::FSM::State<MacroState, SharedData> {

    public:

        virtual void entry(const XBot::FSM::Message &msg) {};

        virtual void react(const XBot::FSM::Event &e) {};
        tfHandler tf;

    };

    class Homing : public MacroState {

        virtual std::string get_name() const { return "Homing"; }

        virtual void run(double time, double period);

        virtual void entry(const XBot::FSM::Message &msg);

        virtual void react(const XBot::FSM::Event &e);

        virtual void exit();

    private:


    };

    class ValveReach : public MacroState {

        virtual std::string get_name() const { return "ValveReach"; }

        virtual void run(double time, double period);

        virtual void entry(const XBot::FSM::Message &msg);

        virtual void react(const XBot::FSM::Event &e);

        virtual void exit();

    private:


    };

    class ValveTurn : public MacroState {

        virtual std::string get_name() const { return "ValveTurn"; }

        virtual void run(double time, double period);

        virtual void entry(const XBot::FSM::Message &msg);

        virtual void react(const XBot::FSM::Event &e);

        virtual void exit();

    private:


    };

    class ValveGoBack : public MacroState {

        virtual std::string get_name() const { return "ValveGoBack"; }

        virtual void run(double time, double period);

        virtual void entry(const XBot::FSM::Message &msg);

        virtual void react(const XBot::FSM::Event &e);

        virtual void exit();

    private:


    };

}
