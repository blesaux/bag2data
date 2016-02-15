/**
 * Written by Joris Guerry (joris.guerry@onera.fr) 2015/09
 * with snippets of code from:
 * http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 * published under license: CC BY 3.0 http://creativecommons.org/licenses/by/3.0/
 *
 * Modified:
 * 2015/09 timestamps in filenames / Bertrand le Saux (bertrand.le_saux@onera.fr)
 * 2015/10 image or depth saving / Bertrand le Saux (bertrand.le_saux@onera.fr)
 *
 */

// boost lib
#include <boost/thread/thread.hpp>
#include <boost/filesystem.hpp>

// Standard libs.
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdlib.h>

#include <stdio.h>
#include <dirent.h>
#include <ios>
#include <stdexcept>

//  ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

using namespace std;
using std::string;
using namespace boost::filesystem;

class ExportImage
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber sub_video_png_input;
  int counter;
  double timebase;
  string flux;
  string path;
  string format;

public:
  /**
   * Constructor which initializes an ImageTransport object with a ROS node
   */
  ExportImage( string _flux, string _path, string _fmt ): it( nh )
  {
    flux = _flux;
    path = _path;
    format = _fmt;
    boost::filesystem::path dir( _path.c_str() );
    if( boost::filesystem::create_directory( dir ) )
      {
        std::cerr << "Directory Created: " << _path << std::endl;
      }

    sub_video_png_input = it.subscribe( flux, 1, &ExportImage::video_png_input_Cb, this );

    counter = 0;
    timebase = 0;
  }

  ~ExportImage()
  {
    cout << counter << " saved images in " << path << endl;
  }

  void video_png_input_Cb( const sensor_msgs::ImageConstPtr& msg )
  {
    //cout << "msg->encoding :" << msg->encoding << endl;

    cv_bridge::CvImageConstPtr cv_ptr;// no modif of the image, so just sharing
    try
      {
        // encoding is deduced from the image message
        // cv_ptr = cv_bridge::toCvShare( msg ); //sensor_msgs::image_encodings::BGR8 );
        if (sensor_msgs::image_encodings::isColor(msg->encoding))
          cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        else
          cv_ptr = cv_bridge::toCvShare(msg);//, sensor_msgs::image_encodings::MONO8);
        // if ( !flux.compare("rgb") )
          // cv_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::BGR8 );
        // else
          // cv_ptr = cv_bridge::toCvShare( msg ); //sensor_msgs::image_encodings::BGR8 );
      }
    catch( cv_bridge::Exception& e )
      {
        ROS_ERROR( "cv_bridge exception: %s", e.what() );
        return;
      }
    counter++;

    /* with timebase, understandable timestamp */
    if (counter==1) {
      timebase=msg->header.stamp.toSec();
    }
    double timestampsec = msg->header.stamp.toSec() - timebase;
    // cout  << "timestampsec = " << timestampsec << endl;
    unsigned int timestamp = static_cast<unsigned int> ( timestampsec * 100000);
    // cout  << "timestamp = " << timestamp << endl;

    /* timestamp based on micro-seconds timestamp since beginning */
    // unsigned int timestamp = static_cast<unsigned int> ( msg->header.stamp.nsec - msg->header.stamp.sec ) / 1000;
    // cout  << "timestamp = " << timestamp << endl;

    std::stringstream saveLocationFlux;
    //			saveLocationFlux << "/home/guerry/catkin_ws/output/frames/frame_" << counter << ".png";
    cout << path << "/" << std::setw( 7 ) << std::setfill( '0' ) << counter << "-" << std::setw( 12 ) << std::setfill( '0' ) << timestamp << "." << format << "\xd";
    saveLocationFlux << path << "/" << std::setw( 7 ) << std::setfill( '0' ) << counter << "-" << std::setw( 12 ) << std::setfill( '0' ) << timestamp << "." << format;

    //	saveLocationFlux << path << "/frame_" << std::setw( 7 ) << std::setfill( '0' ) << counter << ".png";
    std::string saveLocation = saveLocationFlux.str();

    // cv::imwrite( saveLocation , cv_ptr->image ); //, CV_IMWRITE_PNG_COMPRESSION );
    // cout << "format (" << format << ") ?= png  -> " << format.compare(string("png")) << endl;
    if ( (!format.compare(string("png"))) || (!format.compare(string("jpg"))) || (!format.compare(string("jpeg"))) ) {
      cv::imwrite( saveLocation, cv_ptr->image );
    } else {
      ROS_ERROR( "cv::imwrite error: unknown image format" );
      return;
    }

  }

};

int main( int argc, char** argv )
{
  std::cout << "###########################################################################" << std::endl << std::endl ;
  std::cout << "Extract RGB/D image" <<  std::endl << std::endl;
  std::cout << "###########################################################################" << std::endl << std::endl ;

  std::string flux;
  std::string path;
  std::string format;

  ros::init( argc, argv, "Extract_RGBD" );
  if( argc > 2 )
    {
      format = argv[1];
      flux = argv[2];
      path = argv[3];
    }
  else
    {
      cout << "Usage:\nrosrun bag2data extract_rgbd <image format> <topic> <path to save>\n Default is:\nrosrun bag2data extract_rgbd  png /camera/rgb/image_raw /tmp/image/" << endl;
      format = "png";
      flux = "/camera/rgb/image_raw";
      path = "/tmp/images";
    }

  std::cout <<"Save " << flux << " to " << path<<"."<< std::endl;
  ExportImage toImage( flux, path, format );
  ros::spin();

  return 0;
}
