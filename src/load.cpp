//#include <ros/ros.h>
//#include <fstream>

//using namespace std;

////bool saveExploration(fremen::SaveLoad::Request  &req, fremen::SaveLoad::Response &res)
////{
////    ROS_INFO("Saving Simulation!");

////    if(ac_ptr->getState() == actionlib::SimpleClientGoalState::ACTIVE)
////    {
////        ROS_INFO("Robot trying to move! Canceling goal...");
////        ac_ptr->cancelAllGoals();
////        ros::spinOnce();
////    }

////    saveFlag = true;

////    //get pose
////    tf::StampedTransform pose_tf;

////    tf_listener_ptr->waitForTransform("/map","/base_link",ros::Time::now(), ros::Duration(2));
////    tf_listener_ptr->lookupTransform("/map","/base_link",ros::Time(0),pose_tf);

////    geometry_msgs::Pose robot_pose;
////    robot_pose.position.x = pose_tf.getOrigin().x();
////    robot_pose.position.y = pose_tf.getOrigin().y();
////    robot_pose.position.z = pose_tf.getOrigin().z();
////    robot_pose.orientation.x = pose_tf.getRotation().getX();
////    robot_pose.orientation.y = pose_tf.getRotation().getY();
////    robot_pose.orientation.z = pose_tf.getRotation().getZ();
////    robot_pose.orientation.w = pose_tf.getRotation().getW();

////    //edit filename
////    string save_grid, save_pose;
////    save_grid = req.filename + ".grid";
////    save_pose = req.filename + ".pose";

////    //write to file
////    ofstream pose_file;

////    pose_file.open(save_pose.c_str(), ios::out);
////    pose_file << robot_pose.position.x << endl;
////    pose_file << robot_pose.position.y << endl;
////    pose_file << robot_pose.position.z << endl;
////    pose_file << robot_pose.orientation.x << endl;
////    pose_file << robot_pose.orientation.y << endl;
////    pose_file << robot_pose.orientation.z << endl;
////    pose_file << robot_pose.orientation.w << endl;
////    pose_file.close();

////    //call service to save grid!
////    fremen::SaveLoad save_srv;
////    save_srv.request.filename = save_grid;
////    save_srv.request.lossy = req.lossy;
////    save_srv.request.order = req.order;

////    if (save_client_ptr->call(save_srv))
////    {
////        ROS_INFO("Grid Saved");
////        saveFlag = false;
////    }
////    else
////    {
////        ROS_ERROR("Failed to save grid");
////        saveFlag = false;
////        return false;
////    }

////    //everything ok? return true else false
////    return true;
////}

//int main(int argc,char *argv[])
//{
//    ros::init(argc, argv, "FremenExploration");
//    ros::NodeHandle n;

//    ros::NodeHandle nh("~");
//    nh.param("interval", entropy_step, 2.0);

////    'id1 simulation set_object_pose ["robot", "[1.5, 3.2, 1.2]", "[1.0, 0.0, 0.1, 0.0]"]'


//    return 0;
//}


////'id1 simulation set_object_pose ["robot", "[1.5, 3.2, 1.2]", "[1.0, 0.0, 0.1, 0.0]"]'
