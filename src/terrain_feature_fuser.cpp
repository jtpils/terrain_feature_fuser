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
bool image_raw_ready = false;

ros::Publisher  pub_fused, pub_geometric, pub_vision;
image_transport::Publisher pub_img_seg, pub_img_geo, pub_img_fused, pub_img_rgbd, pub_img_depth, pub_img_rgb;

pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud;

Cloud_Image_Mapper *ci_mapper;

sensor_msgs::ImageConstPtr img_seg_;
sensor_msgs::ImageConstPtr img_raw_;
ros::Time cloud_in_time_;

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


void imageCallback_raw(const sensor_msgs::ImageConstPtr& image_msg)
            //    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cout << "raw image recieved" << endl;
    img_raw_ = image_msg;
    image_raw_ready = true;

    if(!image_ready)
        return;

    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = ci_mapper->cloud_image_mapping(image_msg, img_seg_, velodyne_cloud, cloud_in_time_);

    pub_fused.publish(colored_cloud);
    pub_vision.publish(ci_mapper->cloud_v);
    pub_geometric.publish(ci_mapper->cloud_g);

    sensor_msgs::ImagePtr img_v = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ci_mapper->img_seg_).toImageMsg();
    sensor_msgs::ImagePtr img_g  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ci_mapper->img_geo_).toImageMsg();
    sensor_msgs::ImagePtr img_f  = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ci_mapper->img_fused_).toImageMsg();
    
    pub_img_seg.publish(img_v);
    pub_img_geo.publish(img_g);
    pub_img_fused.publish(img_f);

    cloud_ready = false;
    image_ready = false;
}

void process_registered_cloud(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    cout << "cloud recieved  " << cloud_in->header.frame_id << endl;
    cloud_in_time_ = cloud_in->header.stamp;
    // sensor_msgs::PointCloud2 cloud_transformed = transform_cloud(*cloud_in, "world_corrected");
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);

    velodyne_cloud = pcl_cloud;
    cloud_ready = true; 
}

void callback_rawcloud(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    if(!image_raw_ready)
        return;

    cout << "cloud recieved callback_rawcloud " << cloud_in->header.frame_id << endl;
    cloud_in_time_ = cloud_in->header.stamp;
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*cloud_in, pcl_cloud);

    Mat img_rgbd = ci_mapper->get_disparity(img_raw_, pcl_cloud);

    sensor_msgs::ImagePtr img_rgbd_msg = cv_bridge::CvImage(std_msgs::Header(), "rgba16", img_rgbd).toImageMsg();
    sensor_msgs::ImagePtr img_d        = cv_bridge::CvImage(std_msgs::Header(), "mono16", ci_mapper->img_depth_).toImageMsg();
    sensor_msgs::ImagePtr img_rgb      = cv_bridge::CvImage(std_msgs::Header(), "bgr8", ci_mapper->img_rgb_).toImageMsg();
    
    pub_img_depth.publish(img_d);
    pub_img_rgbd.publish(img_rgbd_msg);
    pub_img_rgb.publish(img_rgb);
}

void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    process_registered_cloud(cloud_in);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "terrain_feature_fuser");

    ros::NodeHandle node; 
    tfListener      = new (tf::TransformListener);
    ci_mapper       = new Cloud_Image_Mapper(tfListener);

    pub_fused       = node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/terrain_classifier/fused", 1);
    pub_geometric   = node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/terrain_classifier/geometric", 1);
    pub_vision      = node.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/terrain_classifier/vision", 1);

    ros::Subscriber sub_cloud_raw       = node.subscribe<sensor_msgs::PointCloud2>("/points_classified", 1, callback_rawcloud);
    ros::Subscriber sub_cloud           = node.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, callback_velodyne);
    ros::Subscriber sub_image_seg       = node.subscribe<sensor_msgs::Image>("/image_seg", 1, imageCallback_seg);
    //image_transport::Publisher pub = it.advertise("camera/image", 1);
    
    image_transport::ImageTransport it(node);
    // image_transport::Subscriber sub_seg = it.subscribe("/image_seg", 1, imageCallback_seg);
    image_transport::Subscriber sub_raw = it.subscribe("/image_raw", 1, imageCallback_raw);

    // image_transport::CameraSubscriber sub_camera;
    // sub_camera = it.subscribeCamera("/image_raw", 1, imageCallback_raw);

    pub_img_seg    = it.advertise("/terrain_classifier/label_seg", 1);
    pub_img_geo    = it.advertise("/terrain_classifier/label_geometric", 1);
    pub_img_fused  = it.advertise("/terrain_classifier/label_fused", 1);
    pub_img_rgbd   = it.advertise("/terrain_classifier/rgbd", 1);
    pub_img_depth  = it.advertise("/terrain_classifier/depth", 1);
    pub_img_rgb    = it.advertise("/terrain_classifier/rgb", 1);
    ros::spin();

    return 0;
}
