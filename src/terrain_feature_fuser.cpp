#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "cloud_image_mapper.h"
#include "pcl_ros/point_cloud.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
 
using namespace std; 
tf::TransformListener* tfListener = NULL;
bool cloud_ready = false;
bool image_ready = false;

ros::Publisher  pub_cost;
image_transport::Publisher pub_img_color, pub_img_grey;

pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud;

Cloud_Image_Mapper *ci_mapper;

sensor_msgs::ImageConstPtr img_seg_;

void publish(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> cloud, int type = 2)
{
    sensor_msgs::PointCloud2 pointlcoud2;
    pcl::toROSMsg(cloud, pointlcoud2);

    if(type == 2)
    {
        pub.publish(pointlcoud2);
    }
    else
    {
        sensor_msgs::PointCloud pointlcoud;
        sensor_msgs::convertPointCloud2ToPointCloud(pointlcoud2, pointlcoud);

        pointlcoud.header = pointlcoud2.header;
        pub.publish(pointlcoud);
    }

}

sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2 cloud_in, string frame_target)
{
    ////////////////////////////////// transform ////////////////////////////////////////
    sensor_msgs::PointCloud2 cloud_out;
    tf::StampedTransform to_target;

    try 
    {
        // tf_listener_->waitForTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, ros::Duration(1.0));
        // tf_listener_->lookupTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, to_target);
        tfListener->lookupTransform(frame_target, cloud_in.header.frame_id, ros::Time(0), to_target);
    }
    catch (tf::TransformException& ex) 
    {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        // return cloud_in;
    }

    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (to_target, eigen_transform);
    pcl_ros::transformPointCloud (eigen_transform, cloud_in, cloud_out);

    cloud_out.header.frame_id = frame_target;
    return cloud_out;
}

void imageCallback_seg(const sensor_msgs::ImageConstPtr& image_msg)
{
    cout << "seg image recieved" << endl;
    if(image_ready)
        return;

    img_seg_ = image_msg;


    image_ready = true;
}


void imageCallback_raw(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if(!cloud_ready || !image_ready)
        return;

    cout << "raw image recieved" << endl;
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = ci_mapper->cloud_image_mapping(image_msg, info_msg, img_seg_, velodyne_cloud);
    pub_cost.publish(colored_cloud);

    // sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ci_mapper->img_label_color_).toImageMsg();
    // sensor_msgs::ImagePtr msg_grey  = cv_bridge::CvImage(std_msgs::Header(), "mono8", ci_mapper->img_label_grey_).toImageMsg();

    // pub_img_color.publish(msg_color);
    // pub_img_grey.publish(msg_grey);
    cloud_ready = false;
    image_ready = false;
}

void process_registered_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    cout << "cloud recieved" << endl;
    // sensor_msgs::PointCloud2 cloud_transformed = transform_cloud(*cloud_in, "world_corrected");
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);

    velodyne_cloud = pcl_cloud;
    cloud_ready = true; 
}

void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    process_registered_cloud(cloud_in);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terrain_feature_fuser");

    ros::NodeHandle node; 
    tfListener = new (tf::TransformListener);

    ci_mapper           = new Cloud_Image_Mapper(tfListener);

    ros::Subscriber sub_velodyne_left  = node.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, callback_velodyne);
    pub_cost = node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/label_fused_cloud", 1);
    //image_transport::Publisher pub = it.advertise("camera/image", 1);
    
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe("/image_seg", 1, imageCallback_seg);

    image_transport::CameraSubscriber sub_camera;
    sub_camera = it.subscribeCamera("/image_raw", 1, imageCallback_raw);

    pub_img_color  = it.advertise("geometry_color", 1);
    pub_img_grey   = it.advertise("geometry_grey", 1);
    ros::spin();

    return 0;
}
