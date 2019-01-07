//moveit yonde robot wo move suru syori kaku
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_vs060");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();



    //ros::shutdown();
    return 0;
}


//  read pose  ---  uncomment below
/*
int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_vs060");
    ros::NodeHandle nh("~");
    tf::TransformListener tflistener;

    ros::Rate loop_rate(20);
    while (ros::ok()){
        geometry_msgs::PoseStamped source_pose;
        source_pose.header.frame_id="bucket_link";
        source_pose.pose.orientation.w=1.0;
        geometry_msgs::PoseStamped target_pose;
        try{
            tflistener.waitForTransform("base_link", "bucket_link", ros::Time(0), ros::Duration(8.0)); //Duration ha kensaku hani
            tflistener.transformPose("base_link",ros::Time(0),source_pose,"bucket_link",target_pose);
            ROS_INFO("x:%03f, y:%03f, z:%03f", target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);//pose.orientation.x  yzw mo 
            ROS_INFO("x:%03f, y:%03f, z:%03f, w:%03f", target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z, target_pose.pose.orientation.w);//pose.orientation.x  yzw mo 
        }
        catch(...){
            ROS_INFO("tf error");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
*/
