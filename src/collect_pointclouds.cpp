/*********************************************************************
* written by Bertrand Le Saux bertrand.le_saux@onera.fr 2015/12
*
*********************************************************************/

#include <cstdio>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

// Messages
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

#include <tf/transform_listener.h>

// pcl conversions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// pcl filter
#include <pcl/filters/voxel_grid.h>

// pcl IO
#include <pcl/io/pcd_io.h>

// (if needed for display/test)
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV (if needed for display)
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


/***
 *
 */

using namespace ros;
using namespace pcl;
using namespace sensor_msgs;

class PointCloudCollector {
private:
  NodeHandle nh;
  Subscriber sub, subtf, subdv, subiv;
  // pcl::PointCloud<pcl::PointXYZ> *collectedCloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr collectedCloudPtr;
  int _cloudCount;

  float _leafSize;

  tf::TransformListener *tf_listener;

  tf::Vector3 firstTranslation;

public:
  /**
   * Constructor, subscribe to various needed topics
   */
  PointCloudCollector(bool flagViewDepth=false, bool flagViewImage=false) {
    // Container for original data
    collectedCloudPtr =  pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
    // std::cout << collectedCloudPtr->empty() << std::endl;

    // At first, no cloud is collected
    _cloudCount = 0;

    // default for size of the voxel grid subsampling
    _leafSize = 0.1f;

    // Subscribe to point clouds
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &PointCloudCollector::collectClouds, this);

    // A utility to listen published transforms and registrate point-clouds in /world reference system
    tf_listener    = new tf::TransformListener();

    // Subscribe to depth frames, if needed for visu
    if (flagViewDepth==true) {
      subdv = nh.subscribe("camera/depth/image_raw", 1000, &PointCloudCollector::displayView, this);
    }

    // Subscribe to image frames, if needed for visu
    if (flagViewImage==true) {
      subiv = nh.subscribe("camera/rgb/image_raw", 1000, &PointCloudCollector::displayView, this);
    }
  }

  ~PointCloudCollector() {
  }

  /**
   * Callback method for assembling point clouds.
   */
  void collectClouds( const sensor_msgs::PointCloud2ConstPtr& cloud_msg ) {
    ROS_INFO("****** Collect cloud...");

    // // Container for original data
   // pcl::PCLPointCloud2Ptr tmpCloudPtr( new pcl::PCLPointCloud2 );

   // // Convert to PCL2 data type
   // pcl_conversions::toPCL(*cloud_msg, *tmpCloudPtr);

   // // Perform the filtering, to keep voxel-grid-sampled points
   // pcl::PCLPointCloud2 cloud_filtered;
   // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
   // sor.setInputCloud (tmpCloudPtr);
   // sor.setLeafSize (0.1f, 0.1f, 0.1f);
   // sor.filter (cloud_filtered);
   // ROS_INFO("[PCLPointCloud2] after filtering:  = %ld / %ld data points (%s).", 
            // (long int)( cloud_filtered.width * cloud_filtered.height ) ,
            // (long int)(tmpCloudPtr->data.size()), 
            // pcl::getFieldsList (cloud_filtered).c_str() );


   // // Convert PCL2 to PCL true data type
   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
   // // pcl::fromPCLPointCloud2(*temp_cloud,*cloudPtr);
   // pcl::fromPCLPointCloud2(cloud_filtered,*cloudPtr);

    // Container for original data
   pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloudPtr( new pcl::PointCloud<pcl::PointXYZ> );

   // Convert ROS msg to PCL data type
   pcl::fromROSMsg(*cloud_msg, *tempCloudPtr);

   // Perform the filtering, to keep voxel-grid-sampled points
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::VoxelGrid< pcl::PointXYZ > voxFilter;
   voxFilter.setInputCloud (tempCloudPtr);
   voxFilter.setLeafSize ( _leafSize, _leafSize, _leafSize);
   voxFilter.filter ( *cloudPtr );
   ROS_INFO("After filtering:  = %ld / %ld data points (%s).", 
            // (long int)( cloudPtr->width * cloudPtr->height ) ,
            (long int)(cloudPtr->points.size()), 
            (long int)(tempCloudPtr->points.size()), 
            pcl::getFieldsList (*cloudPtr).c_str() );

   // project in world coordinates (using /odom as reference):
   // code snippets from: http://answers.ros.org/question/35084/transform-a-point-from-pointcloud/
   const std::string target_frame = "odom";// target ref. system
   std::string original_frame =  (*cloudPtr).header.frame_id;//"camera_depth_optical_frame";
   const ros::Time time =  pcl_conversions::fromPCL( (*cloudPtr).header.stamp );//ros::Time(0);
   ros::Rate rate(10.0);
   ROS_INFO("Can transform ? %d (point-cloud ref. = %s -> odom).", tf_listener->canTransform(target_frame, original_frame, time), (*cloudPtr).header.frame_id.c_str() );

   tf_listener->waitForTransform(target_frame, original_frame, time, ros::Duration(20.0));
   tf::StampedTransform transform;
   tf_listener->lookupTransform(target_frame, original_frame, time, transform);

   ROS_INFO("Transform origin =( %lf, %lf, %lf)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z() );
   ROS_INFO("Transform axis =( %lf, %lf, %lf) / angle = %lf", 
            transform.getRotation().getAxis()[0], transform.getRotation().getAxis()[1], 
            transform.getRotation().getAxis()[2], transform.getRotation().getAngle() );

   if (_cloudCount == 0) { // init collected cloud offset = 1st origin translation
     firstTranslation = transform.getOrigin();
   }
   transform.setOrigin( transform.getOrigin() - firstTranslation );

   pcl::PointCloud<pcl::PointXYZ>::Ptr registeredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
   // pcl_ros::transformPointCloud( target_frame, *cloudPtr, *registeredCloudPtr, *tf_listener);
   pcl_ros::transformPointCloud( *cloudPtr, *registeredCloudPtr, transform);

   // get better visualization (around the gravity center ??)
   // get one point (or transform origin ?) and substract it -> transform with identity rotation and one point translation


   // Concatenate new point cloud to the collected point-cloud
   cloudPtr = registeredCloudPtr;
   if ( collectedCloudPtr->empty() ) {
     copyPointCloud( *cloudPtr, *collectedCloudPtr);
   } else {
     *collectedCloudPtr += *cloudPtr;
   }
   _cloudCount++;

   std::cout << "[registered cloud] has " << registeredCloudPtr->points.size() << " points." << std::endl;
   std::cout << "[collected cloud] has " << collectedCloudPtr->points.size() << " points (" << _cloudCount << " clouds assembled)." << std::endl;
  }



  /**
   * display depth or rgb view frame.
   * example from: https://github.com/lucasb-eyer/ros-openni-example/blob/master/src/show_depth.cpp
   */
  void displayView(const sensor_msgs::Image::ConstPtr& msg)
  {
    // The message's data is a raw buffer. While the type is uint8_t*, the
    // data itself is an array of floats (for depth data), the value of
    // the float being the distance in meters.
    // std::cout << "Top-left corner: " << *reinterpret_cast<const float*>(&msg->data[0]) << "m" << std::endl;

    // get msg's frame_id to name the display window
    // std::cout << (*msg).header.frame_id << std::endl;
    std::string frameId = (*msg).header.frame_id;

    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      // cv_ptr = cv_bridge::toCvShare(msg);
      if (sensor_msgs::image_encodings::isColor(msg->encoding))
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      else
        cv_ptr = cv_bridge::toCvShare(msg);

      // imshow expects a float value to lie in [0,1], so we need to normalize
      // for visualization purposes.
      double max = 0.0;
      cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
      cv::Mat normalized;
      cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0)  ;

      // cv::imshow("depth view", normalized);
      cv::imshow(frameId, normalized);
      cv::waitKey(1);
    } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void demeanCloud() {
     Eigen::Vector4f centroid;
     pcl::compute3DCentroid( *collectedCloudPtr, centroid );
     std::cout << " centroid = (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")." << std::endl;
     tf::Vector3 meanCloud = tf::Vector3( centroid[0], centroid[1], centroid[2]);

     // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
     // pcl::demeanPointCloud<pcl::PointXYZ> (collectedCloudPtr, centroid,  cloud_xyz_demean);

     tf::StampedTransform transform;
     transform.setOrigin( - meanCloud );
     pcl_ros::transformPointCloud( *collectedCloudPtr, *collectedCloudPtr, transform);

  }


  /**
   * write collected point cloud to pcd
   */
  void saveCloud() {
    std::cout << "[collected cloud] saved with = " << collectedCloudPtr->points.size() << " points (" << _cloudCount << " clouds assembled)." << std::endl;
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *collectedCloudPtr);
  }

  /**
   * accessor for number of collected clouds
   */
  int getCloudCount() { return _cloudCount;}

  /**
   * accessor for size of voxel grid for subsampling
   */
  float getLeafSize() { return _leafSize;}

  /**
   * setter for size of voxel grid for subsampling
   */
  void setLeafSize(float lf) { _leafSize = lf; }
};


using namespace std ;

void helpUsage(int argc, char **argv) {
    // std::cout is capable of quasi-multi-line statements, like so:
    std::cout << "Usage: rosrun bag2data collect_pointclouds [options]\n"
        "Collect point-clouds from a ROS bag in a single cloud\n"
        "\nOptions:\n"
        "-h (--help)\tPrint this message and exit\n"
        "--view-image\tDisplay image frames while collecting clouds\n"
        "--view-depth\tDisplay depth frames while collecting clouds\n"
        "--n-frames\tNumber of clouds to collect (default: all)\n"
        "--voxel-size\tSize of voxel grid for subsampling points (default: 0.1)\n"
        "\n";
}


int main(int argc, char **argv)
{
  // program parameters set to default values
  // if true, visualize depth frame
  bool flagViewDepth=false;
  // if true, visualize image frame
  bool flagViewImage=false;
  // number of frame/clouds collected, exageratly long value for default means the whole bag
  int maxNbFrames = 100000;
  // voxel grid size for sub-sampling point-clouds
  float leafSize = 0.1f;

  // deal with arguments
  std::vector <std::string> sources;
  for (int i = 1; i < argc; ++i) {
    if ((std::string(argv[i]) == "--help")||(std::string(argv[i]) == "-h")) {
      helpUsage( argc, argv );
      return 2;
    } else    if (std::string(argv[i]) == "--view-depth") {
      flagViewDepth=true;
    } else if (std::string(argv[i]) == "--view-image") {
      flagViewImage=true;
    } else if (std::string(argv[i]) == "--n-frames") {
      if (i + 1 < argc) { // Make sure we aren't at the end of argv!
        i++;// Increment 'i' so we don't get the argument as the next argv[i].
        istringstream ss(argv[i]);
        if (!(ss >> maxNbFrames))
          cerr << "Invalid number " << argv[i] << '\n';
      } else { // Uh-oh, there was no argument to the destination option.
        std::cerr << "--n-frames option requires one argument." << std::endl;
        return 1;
      }
    } else if (std::string(argv[i]) == "--voxel-size") {
      if (i + 1 < argc) { // Make sure we aren't at the end of argv!
        i++;// Increment 'i' so we don't get the argument as the next argv[i].
        istringstream ss(argv[i]);
        if (!(ss >> leafSize))
          cerr << "Invalid number " << argv[i] << '\n';
      } else { // Uh-oh, there was no argument to the destination option.
        std::cerr << "--voxel-size option requires one argument." << std::endl;
        return 1;
      }
    } else {
      sources.push_back(argv[i]);
    }
  }

  // Alert user about arguments we did not deal with:
  if (sources.size() > 0 ) {
    std::cerr << "Some arguments were not handled." << std::endl;
    for (unsigned i=0; i<sources.size(); i++)
      std::cout << ' ' << sources.at(i);
    std::cout << '\n';
    helpUsage( argc, argv );
    return 2;
  }

  ros::init(argc, argv, "pointcloud_collector");

  // Object that subscribes to point cloud topic and collect them
  PointCloudCollector pointCloudCollector(flagViewDepth, flagViewImage);
  pointCloudCollector.setLeafSize( leafSize );
  Rate spin_rate(1);

  // if (flagViewDepth==true) {
    // cv::namedWindow("depth wiew");
  // }
  // if (flagViewImage==true) {
    // cv::namedWindow("image wiew");
  // }

  while( ok() && (pointCloudCollector.getCloudCount()<maxNbFrames) ) {
    spinOnce();

    spin_rate.sleep();
  }

  // save collected point cloud before exiting
  pointCloudCollector.demeanCloud();
  pointCloudCollector.saveCloud();

  // kill view windows if any
  if (flagViewDepth==true) {
    cv::destroyWindow("camera_depth_optical_frame");
  }
  if (flagViewImage==true) {
    cv::destroyWindow("camera_rgb_optical_frame");
  }


  return 0;
}
