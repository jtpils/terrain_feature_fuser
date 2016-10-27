#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv;

class Cloud_Image_Mapper
{
  tf::TransformListener *tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;
  //sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in, string frame_target);

public:
  Mat img_ori, image_label;
  Mat img_label_grey_, img_label_color_;
  Cloud_Image_Mapper(tf::TransformListener *tf_listener)
  {
    tf_listener_ = tf_listener;
  }


  pcl::PointCloud<pcl::PointXYZRGB> transform_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, string frame_target)
  {
    ////////////////////////////////// transform ////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
    tf::StampedTransform to_target;
    
    try 
    {
      tf_listener_->lookupTransform(frame_target, cloud_in.header.frame_id, ros::Time(0), to_target);
    }
    catch (tf::TransformException& ex) 
    {
      ROS_WARN("[draw_frames] TF exception in Cloud_Image_Mapper:\n%s", ex.what());
    }
    
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (to_target, eigen_transform);
    pcl::transformPointCloud(cloud_in, cloud_out, eigen_transform);

    cloud_out.header.frame_id = frame_target;
    return cloud_out;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_image_mapping(
                const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg,
                sensor_msgs::ImageConstPtr image_seg_ptr,
                pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud)
  {   

    cout << "in mapping function" << endl;
    Mat image_raw, image_seg;
    try {
      image_raw = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      // image_seg = cv_bridge::toCvShare(image_seg_ptr, "8UC1")->image;
    }
    catch (cv_bridge::Exception& ex){
      cout << ex.what() << endl;
    }

  //   float scale_row = (float)image_seg.rows / (float)image_raw.rows; 
  //   float scale_col = (float)image_seg.cols / (float)image_raw.cols; 

  //  // read camera information
  //   cam_model_.fromCameraInfo(info_msg);

  //   pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, cam_model_.tfFrame());
    
  //   for(int i = 0; i < pcl_cloud.points.size(); i++)
  //   {
  //     pcl::PointXYZRGB point = pcl_cloud.points[i];
  //     if(point.z < 0 || abs(point.x) > 6)
  //       continue;
      
  //     cv::Point3d pt_cv(point.x, point.y, point.z);
      
  //     cv::Point2d uv;
  //     uv = cam_model_.project3dToPixel(pt_cv);

      
  //     static const int RADIUS = 7;
      
  //     if(uv.x >= 0 && uv.x < image_raw.cols && uv.y >= 0 && uv.y < image_raw.rows)
  //     {
  //       int seg_row         = uv.y * scale_row;
  //       int seg_col         = uv.x * scale_col;
  //       int vision_label    = (int)image_seg.at<uchar>(seg_row, seg_col);
  //       int geomegric_label = velodyne_cloud.points[i].r;

  //       // Vec3b label = image_raw.at<Vec3b>(uv.y, uv.x);

  //       velodyne_cloud.points[i].r = std::min(vision_label, geomegric_label);
  //       velodyne_cloud.points[i].g = std::min(vision_label, geomegric_label);
  //       velodyne_cloud.points[i].b = std::min(vision_label, geomegric_label);

  //       // cout << (int)label << endl;
  //     }
  //   }


  //  cv::imshow("image_resized", image_resized);
   cv::imshow("image", image_raw);
   cv::waitKey(50);

    return velodyne_cloud;
  }
};
