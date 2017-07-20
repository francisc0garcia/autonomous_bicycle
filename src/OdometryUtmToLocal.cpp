#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OdometryUtmToLocal{

public:
    OdometryUtmToLocal(ros::NodeHandle n, ros::NodeHandle pnode){
        //Read parameters from launch file
        pnode.getParam("input_topic_UTM", input_topic_gps);
        pnode.getParam("output_topic_odom", output_topic_odom);

        pnode.getParam("frame_id", frame_id);
        pnode.getParam("child_frame_id", child_frame_id);
        pnode.getParam("offset_x", offset_x);
        pnode.getParam("offset_y", offset_y);
        pnode.getParam("offset_z", offset_z);
        pnode.getParam("orientation", orientation);
        pnode.getParam("reference_model", reference_model);

        odom_pub =  n.advertise<nav_msgs::Odometry>(output_topic_odom, 1);

    }
    ~OdometryUtmToLocal(){};

    void UTMCallback(const nav_msgs::Odometry::ConstPtr &msg){

        if(reference_model == "gazebo"){
            // invert x and y
            global_x =  -msg->pose.pose.position.y;
            global_y =  msg->pose.pose.position.x;
            global_z =  msg->pose.pose.position.z;
        }

        if(reference_model == "real_data"){
            global_x =  -msg->pose.pose.position.x;
            global_y =  -msg->pose.pose.position.y;
            global_z =  msg->pose.pose.position.z;
            // global_z = 0.0; // TODO: Fix z
        }



        if(!is_init){
            initial_x = global_x;
            initial_y = global_y;
            initial_z = global_z;
            ROS_INFO("Frame: %s -> X=%f Y=%f Z=%f", child_frame_id.c_str(), initial_x, initial_y, initial_z);

            is_init = true;
        }else{
            current_x = initial_x - global_x + offset_x;
            current_y = initial_y - global_y + offset_y;
            current_z = initial_z - global_z + offset_z;

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(orientation);

            //first, we'll publish the transform over tf
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = msg->header.stamp;
            odom_trans.header.frame_id = frame_id;
            odom_trans.child_frame_id = child_frame_id;

            odom_trans.transform.translation.x = current_x;
            odom_trans.transform.translation.y = current_y;
            odom_trans.transform.translation.z = current_z;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);

            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = msg->header.stamp;
            odom.header.frame_id = frame_id;
            odom.child_frame_id = child_frame_id;

            //set the position
            odom.pose.pose.position.x = current_x;
            odom.pose.pose.position.y = current_y;
            odom.pose.pose.position.z = current_z;
            odom.pose.pose.orientation = odom_quat;

            //publish the message
            odom_pub.publish(odom);
        }
    }

    std::string input_topic_gps, output_topic_odom;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
private:
    double current_x, current_y, current_z, offset_x, offset_y, offset_z, orientation;
    double global_x, global_y, global_z;
    double initial_x, initial_y, initial_z;
    bool is_init = false;
    std::string frame_id, child_frame_id, reference_model = "gazebo";
};

int main(int argc, char **argv) {
    ROS_INFO("starting UtmToLocal");
    ros::init(argc, argv, "UtmToLocal");

    ros::NodeHandle n;
    ros::NodeHandle pnode("~");

    OdometryUtmToLocal odomToGlobal = OdometryUtmToLocal(n, pnode);
    ros::Subscriber  sub_odometry_utm = n.subscribe(odomToGlobal.input_topic_gps, 1,&OdometryUtmToLocal::UTMCallback, &odomToGlobal);
    ros::spin();

    ROS_INFO("Turning off UtmToLocal");

    return 0;
}

