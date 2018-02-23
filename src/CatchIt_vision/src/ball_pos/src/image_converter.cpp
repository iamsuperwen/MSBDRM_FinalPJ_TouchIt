#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point32.h>
#include <tf/transform_broadcaster.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_d;
  image_transport::Publisher image_pub_;

  ros::Publisher vis_pub, goal_pub;
  tf::TransformBroadcaster *br;

public:
  ImageConverter(): it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_rgb = it_.subscribe("/kinect2/sd/image_color_rect", 1, &ImageConverter::rgbCb, this);
    ///image_sub_d = it_.subscribe("/kinect2/qhd/image_depth_rect", 1, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // *** rgbImg processing: find red ball ***
    cv::Mat hsvImg;   
    cv::cvtColor(cv_ptr->image, hsvImg, CV_BGR2HSV);

    //===== Hard Coded! =====
    //---Red---
    cv::Mat1b mask1, mask2;
    cv::inRange(hsvImg, cv::Scalar(0, 150, 100), cv::Scalar(5, 255, 255), mask1);
    cv::inRange(hsvImg, cv::Scalar(170, 150, 100), cv::Scalar(180, 255, 255), mask2);
    cv::Mat1b mask = mask1 | mask2; 

    //---Yellow---
    /*cv::Mat1b mask;
    cv::inRange(hsvImg, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask);*/


    cv::erode(mask, mask, cv::Mat());
    cv::dilate(mask, mask, cv::Mat());
    
    std::vector< std::vector< cv::Point > > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

    /// Get the moments:
    std::vector<cv::Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ ){
        mu[i] = cv::moments( contours[i], false );
    }    
    ///  Get the mass centers:
    std::vector<cv::Point2f> mc( contours.size() );
    std::vector<cv::Point2f> center( contours.size() );
    std::vector<float> radius( contours.size() );
    for( int i = 0; i < contours.size(); i++ ){
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        minEnclosingCircle(contours[i], center[i], radius[i]);
    }
    /// Draw circles:
    for( int i = 0; i< contours.size(); i++ ){
        if(radius[i] > 5){
            cv::circle( cv_ptr->image, mc[i], 3, CV_RGB(255,255,255) );
            cv::circle( cv_ptr->image, center[i], radius[i], CV_RGB(255,0,0), 2 );
            ///ROS_INFO("Radius: %f", radius[i]);

            //----- Get Interest point Depth(z) -----
            image_sub_d = it_.subscribe("/kinect2/sd/image_depth_rect", 1, boost::bind(&ImageConverter::depthCb, this, _1, center[i], radius[i]) );
        }
     }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);  //(ms)

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg, cv::Point2f pointOfInterest, float r)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);//now cv_ptr is the matrix, do not forget "TYPE_" before "16UC1"
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //*** Convert X,Y Pixel to meter ***
    //===== Hard Coded! =====
    // For IR camera: (/kinect2/sd/camera_info)
    // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1] 
    //     [365.04998779296875, 0.0, 257.3839111328125, 0.0, 365.04998779296875, 206.072998046875, 0.0, 0.0, 1.0]

    ///float depth = cv_ptr->image.at<ushort>(pointOfInterest);

    float tar_x[9], tar_y[9], tar_z[9];
    float x_d[9] = {-1, -1, -1, 0, 0, 0, 1, 1, 1};
    float y_d[9] = {-1, 0, 1, -1, 0, 1, -1, 0, 1};
    int valid_num = 0;
    float TarX=0, TarY=0, TarZ=0;  //averge
    visualization_msgs::Marker marker;
    geometry_msgs::Point32 goal_msg;

    vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    goal_pub = nh_.advertise<geometry_msgs::Point32>("target_pos_inKinect", 100);

    for(int i=0; i<9; i++)
    {
      tar_z[i] = cv_ptr->image.at<ushort>(pointOfInterest.y+y_d[i]*0.5*r, pointOfInterest.x+x_d[i]*0.5*r);
      if(tar_z[i]>0){
        tar_x[i] = calRealX(pointOfInterest.x+x_d[i]*0.5*r, tar_z[i]);
        tar_y[i] = calRealY(pointOfInterest.y+y_d[i]*0.5*r, tar_z[i]);
        valid_num++;
        TarX += tar_x[i];
        TarY += tar_y[i];
        TarZ += tar_z[i];     
      }
    }

    if(valid_num>0){
      TarX /= 1000*valid_num;
      TarY /= 1000*valid_num;
      TarZ /= 1000*valid_num;

      // --- Add Target Marker ---
      marker = createMarkerMesh(TarX, TarY, TarZ, r);
      goal_msg.x = TarX;
      goal_msg.y = TarY;
      goal_msg.z = TarZ;

      broadcastTarget( goal_msg );

      vis_pub.publish( marker );
      goal_pub.publish( goal_msg );
      

      ROS_INFO("(%f,%f)\tDepth: %f", TarX, TarY, TarZ);
    }else{
      ROS_INFO("(AAA");
    }    
  }

  float calRealX(float x_img, float depth)
  {
    //===== Hard Coded! =====
    const float fx = 365.04998779296875;
    const float cx = 257.3839111328125;    
    float x_real = (x_img - cx)*depth/fx;
    return x_real;
  }

  float calRealY(float y_img, float depth)
  {
    //===== Hard Coded! =====
    const float fy = 365.04998779296875;
    const float cy = 206.072998046875;   
    float y_real = (y_img - cy)*depth/fy;
    return y_real;
  }

  const visualization_msgs::Marker createMarkerMesh(float x, float y, float z, float r)
  {
      visualization_msgs::Marker marker;

      marker.header.frame_id = "kinect2_link";
      marker.header.stamp = ros::Time();
      marker.ns = "tracker_markers";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 0.0;
      marker.color.g = 0.8;
      marker.color.b = 0.0;
      marker.color.a= 0.8;
      marker.lifetime = ros::Duration();

      return marker;
  }

  void broadcastTarget(geometry_msgs::Point32 tar_msg){
    //tf::TransformBroadcaster br;
    br = new tf::TransformBroadcaster();
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(tar_msg.x, tar_msg.y, tar_msg.z) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br->sendTransform( tf::StampedTransform(transform, ros::Time::now(), "kinect2_link", "target_point") );
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
