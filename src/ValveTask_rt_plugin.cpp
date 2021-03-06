/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Pietro Balatti
 * email: pietro.balatti@iit.it
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

#include <ValveTask_rt_plugin.h>


/* Specify that the class XBotPlugin::ValveTask is a XBot RT plugin with name "ValveTask" */
REGISTER_XBOT_PLUGIN(ValveTask, XBotPlugin::ValveTask)

namespace XBotPlugin {

bool ValveTask::init_control_plugin(XBot::Handle::Ptr handle)
{
    /* This function is called outside the real time loop, so we can
     * allocate memory on the heap, print stuff, ...
     * The RT plugin will be executed only if this init function returns true. */


    /* Save robot to a private member. */
    _robot = handle->getRobotInterface();

    /* Initialize a logger which saves to the specified file. Remember that
     * the current date/time is always appended to the provided filename,
     * so that logs do not overwrite each other. */
    
    _logger = XBot::MatLogger::getLogger("/tmp/ValveTask_log");
    
    // ROS initialization
    int argc = 1;
    const char *arg = "dummy_arg";
    char* argg = const_cast<char*>(arg);
    char** argv = &argg;
    
    ros::init(argc, argv, "ValveTask");
    fsm.shared_data()._nh =  std::make_shared<ros::NodeHandle>();
    fsm.shared_data().current_command =  std::shared_ptr<XBot::Command>(&current_command);
    fsm.shared_data().plugin_status = _custom_status;
    
    fsm.shared_data()._client = fsm.shared_data()._nh->serviceClient<ADVR_ROS::advr_segment_control>("segment_control");    
    fsm.shared_data()._grasp_mag_pub_RSoftHand = fsm.shared_data()._nh->advertise<std_msgs::String>("/grasp/RWrMot3/goalGrasp",1);
    fsm.shared_data()._grasp_mag_pub_LSoftHand = fsm.shared_data()._nh->advertise<std_msgs::String>("/grasp/LWrMot3/goalGrasp",1);
     _feedBack = fsm.shared_data()._nh->subscribe("Manipulation_status",1,&ValveTask::on_manipulation_status,this);
    manipulation_status = true;
    fsm.shared_data()._SoftHandPose_pub = fsm.shared_data()._nh->advertise<geometry_msgs::PoseStamped>("/w_T_right_ee",1);
    fsm.shared_data()._grasp_client = fsm.shared_data()._nh->serviceClient<ADVR_ROS::advr_grasp_control_srv>("grasp_control");
    
    fsm.shared_data()._hand_over_phase = false;

    
    /*Saves robot as shared variable between states*/
    fsm.shared_data()._robot= _robot;
    fsm.shared_data().robot_ = _robot;


    /*Registers states*/
    fsm.register_state(std::make_shared<myfsm::Homing>());
    fsm.register_state(std::make_shared<myfsm::ValveReach>());
    fsm.register_state(std::make_shared<myfsm::ValveTurn>());
    fsm.register_state(std::make_shared<myfsm::ValveGoBack>());
    
    


    return true;


}

void ValveTask::on_manipulation_status(const std_msgs::Bool::ConstPtr& msg)
{
  
  manipulation_status = msg->data;
  fsm.shared_data()._feedback = manipulation_status;

}

void ValveTask::on_start(double time)
{
    /* This function is called on plugin start, i.e. when the start command
     * is sent over the plugin switch port (e.g. 'rosservice call /ValveTask_switch true').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */

    /* Save the plugin starting time to a class member */
    _robot->getMotorPosition(_q0);

    /* Save the robot starting config to a class member */
    _start_time = time;
    
        
    // Initialize the FSM with the initial state
    fsm.init("Homing");
}

void ValveTask::on_stop(double time)
{
    /* This function is called on plugin stop, i.e. when the stop command
     * is sent over the plugin switch port (e.g. 'rosservice call /ValveTask_switch false').
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */
}


void ValveTask::control_loop(double time, double period)
{
    /* This function is called on every control loop from when the plugin is start until
     * it is stopped.
     * Since this function is called within the real-time loop, you should not perform
     * operations that are not rt-safe. */


     fsm.run(time, 0.01);
     
}

bool ValveTask::close()
{
    /* This function is called exactly once, at the end of the experiment.
     * It can be used to do some clean-up, or to save logging data to disk. */

    /* Save logged data to disk */
    _logger->flush();

    return true;
}



}
