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

#define FLAT    0
#define ROUGH   100
#define OBS     200

class Cloud_Image_Mapper
{
  tf::TransformListener *tf_listener_;
  // image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;

public:
  ros::Time cloud_in_time_;

  Mat img_ori, image_label;
  Mat img_label_grey_, img_label_color_;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_g;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_v;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_f;

  Cloud_Image_Mapper(tf::TransformListener *tf_listener)
  {
    tf_listener_ = tf_listener;
  }

  cv::Point2d project3D_to_image(cv::Point3d& xyz, string frame_id )
  {
    double fx, fy, cx, cy; 
    if(frame_id == "kinect2_rgb_optical_frame")
    {
      fx = 529.9732789120519;
      fy = 526.9663404399863;
      cx = 477.4416333879422;
      cy = 261.8692914553029;
    }
    else
    {
      fx = 775.3905399535238;
      fy = 775.3925549639409;
      cx = 651.1391917338947;
      cy = 394.3686338123942;
    }
    cv::Point2d uv_rect;
    uv_rect.x = (fx*xyz.x) / xyz.z + cx;
    uv_rect.y = (fy*xyz.y) / xyz.z + cy;
    return uv_rect;
  }


  pcl::PointCloud<pcl::PointXYZRGB> transform_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_in, string frame_target)
  {
    ////////////////////////////////// transform ////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
    tf::StampedTransform to_target;
    
    try 
    {
      tf_listener_->lookupTransform(frame_target, cloud_in.header.frame_id, cloud_in_time_, to_target);
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

  void set_point_color(pcl::PointXYZRGB &point, int label)
  {
    if(label == FLAT)
    {
      point.r = 0;
      point.g = 0;
      point.b = 0;
    }
    else if(label == ROUGH)
    {
      point.r = 255;
      point.g = 255;
      point.b = 0;
    }
    else if(label == OBS)
    {
      point.r = 255;
      point.g = 0;
      point.b = 0;
    }      
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud_image_mapping(
                const sensor_msgs::ImageConstPtr& image_msg,
                // const sensor_msgs::CameraInfoConstPtr& info_msg,
                sensor_msgs::ImageConstPtr image_seg_ptr,
                pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud,
                ros::Time cloud_in_time)
  {   
    cout << "in mapping function" << endl;
    if(velodyne_cloud.points.size() == 0)
      return velodyne_cloud;

    cout << "in mapping function" << endl;
    Mat image_raw, image_seg;
    try {
      image_raw = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      image_seg = cv_bridge::toCvShare(image_seg_ptr, "8UC1")->image;
    }
    catch (cv_bridge::Exception& ex){
      cout << ex.what() << endl;
    }

    float scale_row = (float)image_seg.rows / (float)image_raw.rows; 
    float scale_col = (float)image_seg.cols / (float)image_raw.cols; 

   // read camera information
    // cam_model_.fromCameraInfo(info_msg);

    string image_frame_id;
    if(image_msg->header.frame_id == "kinect2_rgb_optical_frame")
      image_frame_id = image_msg->header.frame_id;
    else 
      image_frame_id = "overhead_camera_link";
    
    cloud_in_time_ = cloud_in_time;
    // init result cloud
    cloud_g.points.clear();
    cloud_v.points.clear();
    cloud_f.points.clear();

    cloud_g.header = velodyne_cloud.header;
    cloud_v.header = velodyne_cloud.header;
    cloud_f.header = velodyne_cloud.header;

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, image_msg->header.frame_id);

    for(int i = 0; i < pcl_cloud.points.size(); i++)
    {
      pcl::PointXYZRGB point = pcl_cloud.points[i];
      if(point.z < 0 || abs(point.x) > 6)
        continue;
      
      cv::Point3d pt_cv(point.x, point.y, point.z);
      
      cv::Point2d uv;
      // uv = cam_model_.project3dToPixel(pt_cv);
      uv = project3D_to_image(pt_cv, image_frame_id);
      // cout << cam_model_.tfFrame() << endl;;

      static const int RADIUS = 7;
      
      if(uv.x >= 0 && uv.x < image_raw.cols && uv.y >= 0 && uv.y < image_raw.rows)
      {
        pcl::PointXYZRGB point = velodyne_cloud.points[i];

        int seg_row         = uv.y * scale_row;
        int seg_col         = uv.x * scale_col;
        int vision_label    = (int)image_seg.at<uchar>(seg_row, seg_col);
        int geomegric_label = velodyne_cloud.points[i].r;
        // int fused_label     = std::min(vision_label, geomegric_label);
        int fused_label     = geomegric_label;
        if(vision_label == ROUGH)
          fused_label = vision_label;

        Vec3b raw_color = image_raw.at<Vec3b>(uv.y, uv.x);

        // for fused label
        set_point_color(point, fused_label);
        cloud_f.points.push_back(point);

        // for geomegric_label
        set_point_color(point, geomegric_label);
        point.z += 2;
        cloud_g.points.push_back(point);

        // for vision output
        set_point_color(point, vision_label);
        point.z += 2;
        cloud_v.points.push_back(point);

        velodyne_cloud.points[i].r = raw_color.val[0];
        velodyne_cloud.points[i].g = raw_color.val[1];
        velodyne_cloud.points[i].b = raw_color.val[2];
      }
    }


  //  cv::imshow("image_resized", image_resized);
  //  cv::imshow("image", image_raw);
  //  cv::waitKey(50);

    return cloud_f;
  }
};
