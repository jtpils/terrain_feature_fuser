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

#include <pcl/filters/passthrough.h>

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
  Mat img_seg_, img_geo_, img_fused_, img_depth_, img_all_, img_rgb_;

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
      cloud_out.points.clear();
      return cloud_out;
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
      point.b = 255;
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

  Scalar get_pixel_color(int label)
  {
    Scalar pix_color;
    if(label == FLAT)
    {
      pix_color.val[2] = 0;
      pix_color.val[1] = 0;
      pix_color.val[0] = 255;
    }
    else if(label == ROUGH)
    {
      pix_color.val[2] = 255;
      pix_color.val[1] = 255;
      pix_color.val[0] = 0;
    }
    else if(label == OBS)
    {
      pix_color.val[2] = 255;
      pix_color.val[1] = 0;
      pix_color.val[0] = 0;
    }      

    return pix_color;
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
      image_raw       = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      image_seg       = cv_bridge::toCvShare(image_seg_ptr, "8UC1")->image;
    }
    catch (cv_bridge::Exception& ex){
      cout << ex.what() << endl;
    }

    float scale_row   = (float)image_seg.rows / (float)image_raw.rows; 
    float scale_col   = (float)image_seg.cols / (float)image_raw.cols; 

   // read camera information
    // cam_model_.fromCameraInfo(info_msg);

    string image_frame_id;
    if(image_msg->header.frame_id == "kinect2_rgb_optical_frame")
      image_frame_id  = image_msg->header.frame_id;
    else 
      image_frame_id  = "overhead_camera_link";
    
    // cloud_in_time_    = cloud_in_time;
    cloud_in_time_ = image_msg->header.stamp;
    // init result cloud
    cloud_g.points.clear();
    cloud_v.points.clear();
    cloud_f.points.clear();

    cloud_g.header    = velodyne_cloud.header;
    cloud_v.header    = velodyne_cloud.header;
    cloud_f.header    = velodyne_cloud.header;

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, image_msg->header.frame_id);
    if(pcl_cloud.points.size() == 0)
    {
        cout << "transform error" << endl;
        return velodyne_cloud;
    }

    // init output image
    img_seg_          = image_raw.clone();
    img_geo_          = image_raw.clone();
    img_fused_        = image_raw.clone();
    img_depth_        = Mat(image_raw.rows, image_raw.cols, CV_32FC4, Scalar(0));

    for(int i = 0; i < pcl_cloud.points.size(); i++)
    {
      pcl::PointXYZRGB point_transd = pcl_cloud.points[i];
      if(point_transd.z < 0 || abs(point_transd.x) > 6)
        continue;
      
      cv::Point3d pt_cv(point_transd.x, point_transd.y, point_transd.z);
      
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
        int fused_label     = std::max(vision_label, geomegric_label);
        // int fused_label     = geomegric_label;
        // if(vision_label == ROUGH)
        //   fused_label = vision_label;

                // for output images
        Scalar color_f       = get_pixel_color(fused_label);
        Scalar color_v       = get_pixel_color(vision_label);
        Scalar color_g       = get_pixel_color(geomegric_label);

        cv::circle(img_seg_, uv, 5, color_v, -1);  
        cv::circle(img_geo_, uv, 5, color_g, -1);
        cv::circle(img_fused_, uv, 5, color_f, -1);  

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

        if(point_transd.z > 20)
          point_transd.z = 20;

        uchar depth = 255 / 20 * point_transd.z;
        cv::circle(img_depth_, uv, 5, depth, -1);  
        // img_depth_.at<uchar>(uv.y, uv.x) = depth;

        // // project raw image color to the point cloud
        // Vec3b raw_color = image_raw.at<Vec3b>(uv.y, uv.x);
        // velodyne_cloud.points[i].r = raw_color.val[0];
        // velodyne_cloud.points[i].g = raw_color.val[1];
        // velodyne_cloud.points[i].b = raw_color.val[2];

            // float alpht = 0.5;
    // addWeighted(image, alpht, image_display, 1-alpht, 0, output);
      }
    }


   cv::imshow("depth", img_depth_);
  //  cv::imshow("img_geo_", img_geo_);
  //  cv::imshow("img_fused_", img_fused_);
   cv::waitKey(50);

    return cloud_f;
  }



pcl::PointCloud<pcl::PointXYZRGB> cloud_filter(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  input_cloud       (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_passthrough (new pcl::PointCloud<pcl::PointXYZRGB>);

    cout << "before filter  " << input_cloud->points.size() << endl;

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (1, 200);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
    // cout << "after z filter  " << cloud_passthrough->points.size() << endl;

    pass.setInputCloud (cloud_passthrough);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-10, 10);
    pass.filter (*cloud_passthrough);
    // cout << "after x filter  " << cloud_passthrough->points.size() << endl;

    pass.setInputCloud (cloud_passthrough);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-5, 5);
    pass.filter (*cloud_passthrough);
    // cout << "after y filter  " << cloud_passthrough->points.size() << endl;

    return *cloud_passthrough;
}

  Mat get_disparity(const sensor_msgs::ImageConstPtr& image_msg,
                    pcl::PointCloud<pcl::PointXYZRGB> velodyne_cloud)
  {   
    Mat image_raw;
    // init output image
    if(velodyne_cloud.points.size() == 0)
      return image_raw;
      
    try {
      image_raw = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      img_rgb_ = image_raw.clone();
    }
    catch (cv_bridge::Exception& ex){
      cout << ex.what() << endl;
    }

    string image_frame_id;
    if(image_msg->header.frame_id == "kinect2_rgb_optical_frame")
      image_frame_id  = image_msg->header.frame_id;
    else 
      image_frame_id  = "overhead_camera_link";

    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud = transform_cloud (velodyne_cloud, image_msg->header.frame_id);
    if(pcl_cloud.points.size() == 0)
    {
        cout << "transform error" << endl;
        return image_raw;
    }

    img_depth_ = Mat(image_raw.rows, image_raw.cols, CV_16UC1, Scalar(0));
    img_all_   = Mat(image_raw.rows, image_raw.cols, CV_16UC4, Scalar(0));
    Mat img_depth_display_   = Mat(image_raw.rows, image_raw.cols, CV_16UC1, Scalar(0));

    cout << "in get_disparity function" << img_all_.rows << " " << img_all_.cols << endl;


    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_filtered = cloud_filter(pcl_cloud);
    cout << "after filting" << endl;
    float f = 2262.52;
    float base_line = 0.209313;
    float scale = f*base_line/1.7;

    for(int i = 0; i < pcl_cloud_filtered.points.size(); i++)
    {
      pcl::PointXYZRGB point_transd = pcl_cloud_filtered.points[i];
      if(point_transd.z < 0 || abs(point_transd.x) > 6)
        continue;
      
      cv::Point3d pt_cv(point_transd.x, point_transd.y, point_transd.z);
      
      cv::Point2d uv;
      uv = project3D_to_image(pt_cv, image_frame_id);

      static const int RADIUS = 7;
      
      if(uv.x >= 0 && uv.x < image_raw.cols && uv.y >= 0 && uv.y < image_raw.rows)
      {
        pcl::PointXYZRGB point = velodyne_cloud.points[i];
        unsigned short depth = scale/point_transd.z * 256 + 1;
        // img_depth_.at<unsigned short>(uv.y, uv.x) = depth;

        unsigned short depth_before = img_depth_.at<unsigned short>(uv.y, uv.x);

        if(depth_before < depth)
          cv::circle(img_depth_, uv, 5, depth, -1);  
      }
    }
    cout << "got depth image" << img_all_.rows << " " << img_all_.cols << endl;

    for(int row = 0; row < image_raw.rows; row ++)
    {
      for(int col = 0; col < image_raw.cols; col ++)
      {
        Vec4s u4_value;
        Vec3b raw_color = image_raw.ptr<Vec3b>(row)[col];
        unsigned short depth = img_depth_.ptr<unsigned short>(row)[col];

        u4_value.val[0] = (unsigned short) (raw_color.val[0]);
        u4_value.val[1] = (unsigned short) (raw_color.val[1]);
        u4_value.val[2] = (unsigned short) (raw_color.val[2]);
        u4_value.val[3] = depth;

        img_all_.ptr<Vec4s>(row)[col] = u4_value;
        img_depth_display_.ptr<unsigned short>(row)[col] = depth;
      }
    }
    cout << "got all image " << img_all_.rows << " " << img_all_.cols << endl;
//   imshow("depth", img_depth_display_);
//   waitKey(50);
  return img_all_;  
  }
};
