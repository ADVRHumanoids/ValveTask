#include "fsm_definition.h"



/******************************** BEGIN Homing *******************************/
///////////////////////////////////////////////////////////////////////////////

void myfsm::Homing::react(const XBot::FSM::Event &e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::entry(const XBot::FSM::Message &msg) {
    std::cout << "Homing::entry()" << std::endl;


    shared_data().plugin_status->setStatus("HOMING");


    shared_data().updateRobotStates();


    if (!shared_data().home_recoreded_) {
        shared_data().recordHome();
        shared_data().home_recoreded_ = true;
    }


    // define task home pose for both hands
    Eigen::Vector3d left_hand_task_home_position, right_hand_task_home_position;
    Eigen::Quaterniond left_hand_task_home_quaternion, right_hand_task_home_quaternion;
    left_hand_task_home_position = Eigen::Vector3d(0.3, 0.4, 1.0);
    left_hand_task_home_quaternion = zyx2quat(0.0, DEGTORAD(-45), 0.0);
    right_hand_task_home_position = Eigen::Vector3d(0.3, -0.4, 1.0);
    right_hand_task_home_quaternion = zyx2quat(0.0, DEGTORAD(-45), 0.0);


    geometry_msgs::PoseStamped left_hand_pose_stamped_task_home, right_hand_pose_stamped_task_home;
    left_hand_pose_stamped_task_home.pose.position.x = left_hand_task_home_position[0];
    left_hand_pose_stamped_task_home.pose.position.y = left_hand_task_home_position[1];
    left_hand_pose_stamped_task_home.pose.position.z = left_hand_task_home_position[2];
    left_hand_pose_stamped_task_home.pose.orientation.x = left_hand_task_home_quaternion.x();
    left_hand_pose_stamped_task_home.pose.orientation.y = left_hand_task_home_quaternion.y();
    left_hand_pose_stamped_task_home.pose.orientation.z = left_hand_task_home_quaternion.z();
    left_hand_pose_stamped_task_home.pose.orientation.w = left_hand_task_home_quaternion.w();

    right_hand_pose_stamped_task_home.pose.position.x = right_hand_task_home_position[0];
    right_hand_pose_stamped_task_home.pose.position.y = right_hand_task_home_position[1];
    right_hand_pose_stamped_task_home.pose.position.z = right_hand_task_home_position[2];
    right_hand_pose_stamped_task_home.pose.orientation.x = right_hand_task_home_quaternion.x();
    right_hand_pose_stamped_task_home.pose.orientation.y = right_hand_task_home_quaternion.y();
    right_hand_pose_stamped_task_home.pose.orientation.z = right_hand_task_home_quaternion.z();
    right_hand_pose_stamped_task_home.pose.orientation.w = right_hand_task_home_quaternion.w();

    shared_data().left_hand_pose_home_PoseStamped_ = left_hand_pose_stamped_task_home;
    shared_data().right_hand_pose_home_PoseStamped_ = right_hand_pose_stamped_task_home;



    // select the hand to use
    std::cout << "Please select the hand you want to move!" << std::endl;

    std::string selectedHand;
    selectedHand = "LSoftHand";
    selectedHand = "RSoftHand";
    shared_data().selectedHand_ = selectedHand;
    std::cout << "Hand selected: " << selectedHand << std::endl;

    trajectory_utils::Cartesian start, end;
    if (selectedHand == "LSoftHand") {
        start.distal_frame = "LSoftHand";
        start.frame = shared_data().left_hand_pose_PoseStamped_;
        end.distal_frame = "LSoftHand";
        end.frame = shared_data().left_hand_pose_home_PoseStamped_;
    } else if (selectedHand == "RSoftHand") {
        start.distal_frame = "RSoftHand";
        start.frame = shared_data().right_hand_pose_PoseStamped_;
        end.distal_frame = "RSoftHand";
        end.frame = shared_data().right_hand_pose_home_PoseStamped_;
    }

    // define one segment
    trajectory_utils::segment segment;
    segment.type.data = 0;        // min jerk traj
    segment.T.data = TRAJ_DURATION;         // traj duration 5 second
    segment.start = start;        // start pose
    segment.end = end;            // end pose

    // define segments
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(segment);

    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);
    std::cout << "Move to home pose!" << std::endl;


    std::cout << "----------State Machine----------" << std::endl;
    std::cout << "Current State:   Homing" << std::endl;
    std::cout << "success   ->   ValveReach" << std::endl;
    std::cout << "fail      ->   Homing" << std::endl;
    std::cout << "---------------------------------" << std::endl;

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::run(double time, double period) {


    // blocking reading: wait for a command
    if (!shared_data().current_command->str().empty()) {
        std::cout << "Command: " << shared_data().current_command->str() << std::endl;

        // Homing Succeeded
        if (!shared_data().current_command->str().compare("success"))
            transit("ValveReach");

        // Homing failed
        if (!shared_data().current_command->str().compare("fail"))
            transit("Homing");

    }

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::Homing::exit() {

}

/********************************* END Homing ********************************/




/****************************** BEGIN ValveReach *****************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::react(const XBot::FSM::Event &e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::entry(const XBot::FSM::Message &msg) {
    std::cout << "ValveReach::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVEREACH");

    // blocking call: wait for a pose on topic debris_pose
    std::cout << "Please define the pose of the valve!" << std::endl;
    geometry_msgs::PoseStampedConstPtr temp;
    temp = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("valve_pose");
    shared_data().valve_pose_ = *temp;


    shared_data().calcValveKeyPoses();



    //CALL SERVICE TO MOVE

    trajectory_utils::Cartesian start;
    start.distal_frame = shared_data().selectedHand_;
    start.frame = shared_data().right_hand_pose_home_PoseStamped_;


    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = shared_data().selectedHand_;
    intermediate.frame = shared_data().valve_approach_pose_;


    trajectory_utils::Cartesian end;
    end.distal_frame = shared_data().selectedHand_;
    end.frame = shared_data().valve_pose_;


    shared_data().last_pose_PoseStamped_ = end.frame;


    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second
    s1.start = start;        // start pose
    s1.end = intermediate;            // intermediate pose

    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose 

    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    segments.push_back(s2);

    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);

    std::cout << "----------State Machine----------" << std::endl;
    std::cout << "Current State:   ValveReach" << std::endl;
    std::cout << "success   ->   ValveTurn" << std::endl;
    std::cout << "fail      ->   Homing" << std::endl;
    std::cout << "---------------------------------" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::run(double time, double period) {

    // blocking reading: wait for a command
    if (!shared_data().current_command->str().empty()) {
        std::cout << "Command: " << shared_data().current_command->str() << std::endl;

        // ValveReach failed
        if (!shared_data().current_command->str().compare("fail"))
            transit("Homing");

        // ValveReach Succeeded
        if (!shared_data().current_command->str().compare("success"))
            transit("ValveTurn");
    }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveReach::exit() {

}

/****************************** END ValveReach *******************************/




/****************************** BEGIN ValveTurn ******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::react(const XBot::FSM::Event &e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::entry(const XBot::FSM::Message &msg) {
    std::cout << "ValveTurn::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVETURN");


    trajectory_utils::Cartesian start;
    start.distal_frame = shared_data().selectedHand_;
    start.frame = shared_data().valve_pose_;

    trajectory_utils::Cartesian end;
    end.distal_frame = shared_data().selectedHand_;
    end.frame = shared_data().valve_turned_pose_;


    geometry_msgs::Vector3 plane_normal;
    plane_normal.x = shared_data().valve_Affine_.rotation()(0, 2);
    plane_normal.y = shared_data().valve_Affine_.rotation()(1, 2);
    plane_normal.z = shared_data().valve_Affine_.rotation()(2, 2);


    geometry_msgs::Vector3 circle_center;
    circle_center.x = shared_data().valve_pose_.pose.position.x;
    circle_center.y = shared_data().valve_pose_.pose.position.y;
    circle_center.z = shared_data().valve_pose_.pose.position.z - VALVE_RADIUSE;


    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 1;        // arc traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    //s1.end = end;            // end pose 
    s1.end_rot = end.frame.pose.orientation;
    s1.angle_rot.data = M_PI_2;
    s1.circle_center = circle_center;
    s1.plane_normal = plane_normal;

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);

    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);


    shared_data().updateRobotStates();
    shared_data()._last_pose = boost::shared_ptr<geometry_msgs::PoseStamped>(
            new geometry_msgs::PoseStamped(shared_data().right_hand_pose_PoseStamped_));


    std::cout << "----------State Machine----------" << std::endl;
    std::cout << "Current State:   ValveTurn" << std::endl;
    std::cout << "success   ->   ValveGoBack" << std::endl;
    std::cout << "fail      ->   Homing" << std::endl;
    std::cout << "---------------------------------" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::run(double time, double period) {

    // blocking reading: wait for a command
    if (!shared_data().current_command->str().empty()) {
        std::cout << "Command: " << shared_data().current_command->str() << std::endl;

        // ValveTurn failed
        if (!shared_data().current_command->str().compare("fail"))
            transit("Homing");

        // ValveTurn Succeeded
        if (!shared_data().current_command->str().compare("success"))
            transit("ValveGoBack");
    }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveTurn::exit() {

}

/****************************** END ValveTurn *******************************/




/****************************** BEGIN ValveGoBack ******************************/

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::react(const XBot::FSM::Event &e) {

}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::entry(const XBot::FSM::Message &msg) {
    std::cout << "ValveGoBack::entry()" << std::endl;

    shared_data().plugin_status->setStatus("VALVEGOBACK");


    trajectory_utils::Cartesian start;
    start.distal_frame = shared_data().selectedHand_;
    start.frame = shared_data().valve_turned_pose_;


    trajectory_utils::Cartesian intermediate;
    intermediate.distal_frame = shared_data().selectedHand_;
    intermediate.frame = shared_data().valve_retreat_pose_;

    // define the first segment
    trajectory_utils::segment s1;
    s1.type.data = 0;        // min jerk traj
    s1.T.data = TRAJ_DURATION;         // traj duration 5 second      
    s1.start = start;        // start pose
    s1.end = intermediate;            // intermediate pose     



    trajectory_utils::Cartesian end;
    end.distal_frame = shared_data().selectedHand_;
    end.frame = shared_data().right_hand_pose_home_PoseStamped_;


    // define the second segment
    trajectory_utils::segment s2;
    s2.type.data = 0;        // min jerk traj
    s2.T.data = TRAJ_DURATION;         // traj duration 5 second
    s2.start = intermediate;        // start pose
    s2.end = end;            // end pose

    // only one segment in this example
    std::vector<trajectory_utils::segment> segments;
    segments.push_back(s1);
    segments.push_back(s2);

    // prepare the advr_segment_control
    ADVR_ROS::advr_segment_control srv;
    srv.request.segment_trj.header.frame_id = "world_odom";
    srv.request.segment_trj.header.stamp = ros::Time::now();
    srv.request.segment_trj.segments = segments;

    // call the service
    shared_data()._client.call(srv);


    std::cout << "----------State Machine----------" << std::endl;
    std::cout << "Current State:   ValveGoBack" << std::endl;
    std::cout << "success   ->   Homing" << std::endl;
    std::cout << "fail      ->   Homing" << std::endl;
    std::cout << "---------------------------------" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::run(double time, double period) {

    // blocking reading: wait for a command
    if (!shared_data().current_command->str().empty()) {
        std::cout << "Command: " << shared_data().current_command->str() << std::endl;

        // ValveGoBack failed
        if (!shared_data().current_command->str().compare("fail"))
            transit("Homing");

        // ValveGoBack Succeeded
        if (!shared_data().current_command->str().compare("success"))
            transit("Homing");
    }
}

///////////////////////////////////////////////////////////////////////////////
void myfsm::ValveGoBack::exit() {

}

/****************************** END ValveGoBack *******************************/


