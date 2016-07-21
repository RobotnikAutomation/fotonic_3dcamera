/*
 * Software License Agreement (BSD License)
 *
 *  fotonic_3dcamera_node
 *  Copyright (c) 2012, Robotnik Automation, SLL
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <assert.h>
#include <boost/format.hpp>
#include <iostream>
#include "fotonic_3dcamera/camera.h"
#include <stdio.h>

#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"

#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/Float32.h>

#define FOTONIC_DESIRED_FREQ	10.0

using namespace std;


class fotonicNode
{
public:
	// Config params
	string ip_address_;
	string base_frame_id_;
	string fotonic_frame_id_;
    string name_;
    int startup_delay_;

	self_test::TestRunner self_test_;
	diagnostic_updater::Updater diagnostic_;

	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	ros::Publisher fotonic_3Ddata_pub_;
	ros::Publisher fotonic_3Ddata_pub2_;
	//ros::Publisher image_pub_;
	//ros::Publisher image_depth_pub_;
	ros::Publisher temperature_pub_;
	ros::Publisher camera_info_pub_;
	ros::Publisher camera_info_depth_pub_;	
    sensor_msgs::CameraInfo camera_info_msg_;
    image_transport::Publisher pub_;
    image_transport::Publisher pub_depth_;

	

	bool running;

	// Error counters and flags
	int error_count_;
	int slow_count_;
	std::string was_slow_;
	std::string error_status_;

	double desired_freq_;
	diagnostic_updater::FrequencyStatus freq_diag_;

    // Camera and filter variables
    C_FZCamera *camera_;
    // Sensor parameters
    double sensor_shutter_ms_;
    int sensor_framerate_;
    int sensor_framerate_divisor_;
    // Hardware filters
    bool lerp_filter_enable_;   // true
    bool edge_filter_enable_;   // false
    int edge_filter_minB_;      // 1
    int edge_filter_diff1_;     // 10
    int edge_filter_diff2_;     // 10
    int edge_filter_diff3_;     // 20
    // Software filters
    int active_brightness_min_;	// 20
    int active_brightness_max_;	// 4095
    int z_range_min_;           // 0
    int z_range_max_;           // 7000
    bool pre_zfilterx_enable_;	// false
    bool pre_zfiltery_enable_;	// false
    int  pre_zfilter_depth_;	// 150
    bool noise_filter_enable_;	// false
    int  noise_filter_radius_;
    int  noise_filter_depth_;
    int  noise_filter_pixels_;
		
	float max_delay;

	fotonicNode(ros::NodeHandle h) : self_test_(), diagnostic_(),
	 node_handle_(h), private_node_handle_("~"),
	 error_count_(0),
	 slow_count_(0),
	 desired_freq_(10),
     freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05))
	{
		running = false;

        // READ PARAMS
        private_node_handle_.param("name", name_, string("fotonic_3dcamera"));
        ros::NodeHandle fotonic_node_handle(node_handle_, name_);
        
        private_node_handle_.param("startup_delay", startup_delay_, 0);

        private_node_handle_.param("ip_address", ip_address_, string("192.168.1.10"));
        private_node_handle_.param("base_frame_id", base_frame_id_, string("base_link"));
        private_node_handle_.param("fotonic_frame_id", fotonic_frame_id_, string("fotonic_link_optical"));
        // Sensor parameters
        private_node_handle_.param("sensor_shutter_ms", sensor_shutter_ms_, 10.0 );
        private_node_handle_.param("sensor_framerate", sensor_framerate_, 40 );
        private_node_handle_.param("sensor_framerate_divisor", sensor_framerate_divisor_, 1 );
        // Hardware filters
        private_node_handle_.param("lerp_filter_enable", lerp_filter_enable_, true );
        private_node_handle_.param("edge_filter_enable", edge_filter_enable_, false );
        private_node_handle_.param("edge_filter_minB", edge_filter_minB_, 1 );
        private_node_handle_.param("edge_filter_diff1", edge_filter_diff1_, 10 );
        private_node_handle_.param("edge_filter_diff2", edge_filter_diff2_, 10 );
        private_node_handle_.param("edge_filter_diff3", edge_filter_diff3_, 20 );
        // Software filters
        private_node_handle_.param("active_brightness_min", active_brightness_min_, 20 );
        private_node_handle_.param("active_brightness_max", active_brightness_max_, 4095 );
        private_node_handle_.param("z_range_min", z_range_min_, 0 );
        private_node_handle_.param("z_range_max", z_range_max_, 7000);
        private_node_handle_.param("pre_zfilterx_enable", pre_zfilterx_enable_, false );
        private_node_handle_.param("pre_zfiltery_enable", pre_zfiltery_enable_, false );
        private_node_handle_.param("pre_zfilter_depth", pre_zfilter_depth_, 150 );
        private_node_handle_.param("noise_filter_enable", noise_filter_enable_, false );
        private_node_handle_.param("noise_filter_radius", noise_filter_radius_, 1 );
        private_node_handle_.param("noise_filter_depth", noise_filter_depth_, 10 );
        private_node_handle_.param("noise_filter_pixels", noise_filter_pixels_, 20 );

        std::string sNodeNamespace = private_node_handle_.getNamespace();
        ROS_INFO("%s: Initializing node", sNodeNamespace.c_str());
        ROS_INFO("%s: Brightness [%d - %d]", sNodeNamespace.c_str(), active_brightness_min_, active_brightness_max_);
        ROS_INFO("%s: Z Range: [%d - %d]", sNodeNamespace.c_str(), z_range_min_, z_range_max_);
        ROS_INFO("%s: Pre Z Filter [X]: %d Pre Z Filter [Y]:%d", sNodeNamespace.c_str(), pre_zfilterx_enable_, pre_zfiltery_enable_);
        ROS_INFO("%s: Noise Filter: %d", sNodeNamespace.c_str(), noise_filter_enable_);
        ROS_INFO("%s: Frame: %s", sNodeNamespace.c_str(), fotonic_frame_id_.c_str());

        fotonic_3Ddata_pub_ = private_node_handle_.advertise<sensor_msgs::PointCloud>("point_cloud", 1);
        fotonic_3Ddata_pub2_ = private_node_handle_.advertise<sensor_msgs::PointCloud2>("point_cloud2", 1);
		//image_pub_ = fotonic_node_handle.advertise<sensor_msgs::Image>("image", 1);
		//image_depth_pub_ = fotonic_node_handle.advertise<sensor_msgs::Image>("image_depth", 1);
        camera_info_pub_ = private_node_handle_.advertise<sensor_msgs::CameraInfo>("ir/camera_info", 1);
        camera_info_depth_pub_ = private_node_handle_.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);
        temperature_pub_ = private_node_handle_.advertise<std_msgs::Float32>("temperature", 1);
        image_transport::ImageTransport it(private_node_handle_);
        pub_ = it.advertise("ir/image_raw", 1);
        pub_depth_ = it.advertise("depth/image_raw", 1);
	   
        camera_info_manager::CameraInfoManager camera_info_manager_fotonic_(fotonic_node_handle, name_);
		camera_info_manager_fotonic_.loadCameraInfo("package://fotonic_3dcamera/fotonic_calibration.yaml");
		camera_info_msg_ = camera_info_manager_fotonic_.getCameraInfo();
		
		self_test_.add( "Connect Test", this, &fotonicNode::ConnectTest );
		diagnostic_.add( freq_diag_ );
		diagnostic_.add( "Device Status", this, &fotonicNode::deviceStatus );

		max_delay = 1.0 / FOTONIC_DESIRED_FREQ;
	}

	~fotonicNode()
	{
		stop();
	}


    void InitSensor()
    {
        std::string sNodeNamespace = private_node_handle_.getNamespace();

        //ROS_INFO("%s: Initializing sensor", sNodeNamespace.c_str());

        double dMSShutter2 = 5.0; // must be <= dShutter
        int iMSSaturation = 800;

        camera_->SetShutter( (unsigned short)( sensor_shutter_ms_*10 ) );
        camera_->SetFrameRate( (unsigned short) sensor_framerate_, (unsigned short) sensor_framerate_divisor_ );
        camera_->SetMSShutter2( (unsigned short)( dMSShutter2*10 ) );
        camera_->SetMSSaturation( (unsigned short) iMSSaturation );

        camera_->SetFiltering( (unsigned short) lerp_filter_enable_, (unsigned short) edge_filter_enable_,
                 edge_filter_minB_, edge_filter_diff1_, edge_filter_diff2_, edge_filter_diff3_);

        ROS_INFO("%s: Sensor Shutter: %f", sNodeNamespace.c_str(), sensor_shutter_ms_);
        ROS_INFO("%s: Sensor Frame Rate: %d", sNodeNamespace.c_str(), sensor_framerate_);
        ROS_INFO("%s: Sensor Frame Rate Divisor: %d", sNodeNamespace.c_str(), sensor_framerate_divisor_);
        ROS_INFO("%s: Lerp Filter: %d", sNodeNamespace.c_str(), lerp_filter_enable_);
        ROS_INFO("%s: Edge Filter: %d", sNodeNamespace.c_str(), edge_filter_enable_);
    }


	int start()
	{	
        std::string sNodeNamespace = private_node_handle_.getNamespace();

        stop();
        
        // Perform a configurable stop in order to delay the start up and don't saturate the Ethernet comunication
        if (startup_delay_){
			ROS_WARN("%s: Delaying startup %d seconds...", sNodeNamespace.c_str(), startup_delay_);
			sleep(startup_delay_);
		}

		// Create camera object
		camera_ = new C_FZCamera();

		// Initialize library 
		FZ_Result res = camera_->Init();
        //ROS_INFO("%s: FZ_Init returned %d", sNodeNamespace.c_str(), (int) res);

		// Initializes the camera with the given address
		char buf[50] = "/0"; 
		sprintf(buf, "%s", ip_address_.c_str() );

		if (!camera_->SetActive( buf ) ) {
            ROS_ERROR("%s: SetActive failed", sNodeNamespace.c_str());
			return -1;
        }else{
            ROS_INFO("%s: Set active camera at %s", sNodeNamespace.c_str(), ip_address_.c_str());
        }

		// Sets camera measurement mode
		if (!camera_->SetMode( DE_MODE_PA_Z ) ) {
            ROS_ERROR("%s: SetMode failed", sNodeNamespace.c_str());
			return -1;
        }

		InitSensor();
		camera_->Start();
        if (camera_->IsStarted() ) ROS_INFO("%s: Camera Started", sNodeNamespace.c_str());

        // Enumerate 3dcamera devices (This service is not working in the library, it also does not work in Windows)
        /*if (camera_->Enumerate() == 0){
            ROS_ERROR ("fotonicNode::start - No device found!");
            return -1;
        }*/

        ROS_INFO("%s: Connected to 3D CAMERA", sNodeNamespace.c_str());
		freq_diag_.clear();
		
		// Get and print some important data to the connected camera
		char cameraData [100];
		
		camera_->GetInfoString(API_VERSION, cameraData, 50); 
		ROS_INFO("%s: API VERSION: %s", sNodeNamespace.c_str(), cameraData);
		
		camera_->GetInfoString(DE_VERSION, cameraData, 50); 
		ROS_INFO("%s: DE VERSION: %s", sNodeNamespace.c_str(), cameraData);
		
		camera_->GetInfoString(CA_VERSION, cameraData, 50); 
		ROS_INFO("%s: CA VERSION: %s", sNodeNamespace.c_str(), cameraData);

		running = true;

		return(0);
	}

	int stop()
	{
		if(running)
		{
			// Closing actions
			delete camera_;
			running = false;
		}
		return(0);
	}

	// determines if a given pixel shall be filtered out or not
	// X: if Z value of 1 horizontal neighbour differ more than "depth" the pixel is filtered
	// Y: if Z value of 1 vertical neighbour differ more than "depth" the pixel is filtered
	bool PreZFilterPixel(short *pPixels, int iNumX, int iNumY, int x, int y)
	{
		// filtering enabled?
        if(!pre_zfilterx_enable_ && !pre_zfiltery_enable_) return false;

		// remove pixels where the filter radius is outside of image
		if(x<=0 || x>=(iNumX-1) || y<=0 || y>=(iNumY-1)) return true;

		// current position and pixel
		int iRowPosZ = (y*iNumX)*4+(iNumX*1);
		short iZ = *(pPixels+iRowPosZ+x);

		if(pre_zfilterx_enable_) {
			// get surounding pixels
			short iZLeft  = *(pPixels+iRowPosZ+x-1);
			short iZRight = *(pPixels+iRowPosZ+x+1);
			// check pixels to left and right
			if( iZ < (iZLeft  - pre_zfilter_depth_) || iZ > (iZLeft  + pre_zfilter_depth_) || 
				iZ < (iZRight - pre_zfilter_depth_) || iZ > (iZRight + pre_zfilter_depth_)) return true;
        }

		if(pre_zfiltery_enable_) {
			int iRowPosZAbove = ((y-1)*iNumX)*4+(iNumX*1);
			int iRowPosZBelow = ((y+1)*iNumX)*4+(iNumX*1);
			// get surounding pixels
			short iZAbove = *(pPixels+iRowPosZAbove+x);
			short iZBelow = *(pPixels+iRowPosZBelow+x);
			// check pixels above and below
			if( iZ < (iZAbove - pre_zfilter_depth_) || iZ > (iZAbove + pre_zfilter_depth_) || 
				iZ < (iZBelow - pre_zfilter_depth_) || iZ > (iZBelow + pre_zfilter_depth_)) return true;
        }

        return false;
	}

	// determines if a given pixel shall be filtered out or not
	// if Z difference of neighbouring pixels within "radius" is more than "depth" a counter is increased
	//  if the counter reaches "pixels" the pixel is filtered
	bool NoiseFilterPixel(short *pPixels, int iNumX, int iNumY, int x, int y)
	{
		if(!noise_filter_enable_) return false;

		// remove pixels where the filter radius is outside of image
		if( x-noise_filter_radius_<0 || x+noise_filter_radius_>=(iNumX-1) ||
			y-noise_filter_radius_<0 || y+noise_filter_radius_>=(iNumY-1)) return true;

		// current position and pixel
		int iRowPosZ = (y*iNumX)*4+(iNumX*1);
		short iZ = *(pPixels+iRowPosZ+x);

		int iPixelsFound = 0;
		int iNumPixels = noise_filter_radius_*2+1;
		iNumPixels = iNumPixels*iNumPixels-1;

		// each row in filter
		for(int iFilterY=-noise_filter_radius_; iFilterY<=noise_filter_radius_; iFilterY++) {
			// pointer to current row in filter
			int iFilterRowPosZ = ((y+iFilterY)*iNumX)*4+(iNumX*1);

			for(int iFilterX=-noise_filter_radius_; iFilterX<=noise_filter_radius_; iFilterX++){
				// dont use current pixel
				if( iFilterY==0 && iFilterX==0 ) continue;
				// get the neighbour pixel
				int iNeighbourPixel = *(pPixels+iFilterRowPosZ+x+iFilterX);
				// compare if current pixel is within depth of neighbour
				if( (iNeighbourPixel - noise_filter_depth_) <= iZ && iZ <= (iNeighbourPixel + noise_filter_depth_) ) {
					iPixelsFound++;
					if(iPixelsFound>=noise_filter_pixels_) {
						// pixel will not be filtered
						return false;
					}
                }
				iNumPixels--; //pixels left to test
				// no way for the pixels needed to be found, exit early (pixel will be filtered)
				if(iNumPixels+iPixelsFound < noise_filter_pixels_) return true; 
			}
		}
		// dont draw
		return true; //this shall never happen
    }


	void getData()
	{

		// Get pointclouds and intensities as image 
		sensor_msgs::PointCloud2 point_cloud2;
		sensor_msgs::PointCloud point_cloud;
		short *pImage = NULL;
        short *pImageLastOK = NULL;

		int iRows = 120;  // finally working !
		int iCols = 160;  // 

		if (!camera_->IsStarted()) return;

		// Get Image and Image Header 
		pImage = camera_->GetFrame();

		if (pImage) {
			FZ_FRAME_HEADER *pHeader = camera_->GetFrameHeader();	
			// ROS_INFO("version=%d bytesperpixel=%d nrows=%d ncols=%d framecounter=%d", 
            // pHeader->version, pHeader->bytesperpixel, pHeader->nrows, pHeader->ncols, pHeader->framecounter);
            // ROS_INFO("lasterrorframe=%d shutter=%d mode=%d reportedframerate=%d",
            // pHeader->lasterrorframe, pHeader->shutter, pHeader->mode, pHeader->reportedframerate);
            // ROS_INFO("measuredframerate=%d, camera chip temperature=%3.2f[C]",
            // pHeader->measuredframerate, (float)pHeader->temperature * 0.1);
            //pHeader->timestamp[0],pHeader->timestamp[1],pHeader->timestamp[2],pHeader->timestamp[3]
			// timestamp[0] Contains start of exposure in POSIX time (seconds)
			// timestamp[1] Contains the milliseconds part of start of exposure.
			// timestamp[2] Contains the duration of the complete exposure in milliseconds.
			// timestamp[3] Not used, will contain zero.

			// copy to local 			
			int iSize = pHeader->bytesperpixel*pHeader->ncols*pHeader->nrows;
			pImageLastOK = new short[iSize/2];
			memcpy(pImageLastOK, pImage, iSize);

			// PointCloud type 
			// Prepare the PointCloud message 
			ros::Time current_time = ros::Time::now();					
			point_cloud.header.stamp = current_time;
			point_cloud.header.frame_id = fotonic_frame_id_;
			point_cloud.channels.resize(1);
			point_cloud.channels[0].name = "intensities";
			//int num_points = iCols * iRows;
			//point_cloud.points.resize( num_points );
			//point_cloud.channels[0].values.resize(num_points);

			// PointCloud2 type 
			// Prepare the message to publish		
			point_cloud2.header.stamp = current_time;
			point_cloud2.header.frame_id = fotonic_frame_id_;
			point_cloud2.height = iRows;
			point_cloud2.width = iCols;

            // size of field is 3 for x, y, z and 3 for rgb and u,v
			point_cloud2.fields.resize( 3 + 3 );
			point_cloud2.fields[0].name = "x"; point_cloud2.fields[1].name = "y"; point_cloud2.fields[2].name = "z";
			point_cloud2.fields[3].name = "rgb"; point_cloud2.fields[4].name = "u"; point_cloud2.fields[5].name = "v";
			int offset = 0;
			for (size_t d = 0; d < point_cloud2.fields.size (); ++d, offset += 4) {
				point_cloud2.fields[d].offset = offset;
				point_cloud2.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
				point_cloud2.fields[d].count  = 1;
            }

			point_cloud2.point_step = offset;
			point_cloud2.row_step   = point_cloud2.point_step * iCols;
			point_cloud2.is_bigendian = false;
			point_cloud2.data.resize ( iCols * iRows * point_cloud2.point_step );//act.point data, size is (row_step*height)

			// Image type 
            cv::Mat* cv_float_image = new cv::Mat(iRows,iCols,CV_16UC1);
            cv::Mat* cv_float_image_depth = new cv::Mat(iRows,iCols,CV_16UC1);  // CV_32FC1);

            int iNumY = iRows;
            int iNumX = iCols;

            for(int y=0; y<iNumY; y++) {

                int iRowPos = y*(iNumX*4);

                //image
                // first lib versions E70P
                for(int x=0; x<iNumX; x++) {

                    // new E70P version 3D image is inverted left-right

                    int16_t iB = pImageLastOK[iRowPos+iNumX*0+x];       // B
                    int16_t iZ = pImageLastOK[iRowPos+iNumX*1+x];       // Z
                    int16_t iX = pImageLastOK[iRowPos+iNumX*2+x];       // X
                    int16_t iY = pImageLastOK[iRowPos+iNumX*3+x];       // Y

                    // pre Z filter
                    if( PreZFilterPixel(pImageLastOK, iNumX, iNumY, x, y) ) continue;

                    // noise filter
                    if( NoiseFilterPixel(pImageLastOK, iNumX, iNumY, x, y) ) continue;

                    // Confidence level
                    if ((iB<active_brightness_min_) || (iB>active_brightness_max_) || (iZ<50) ) continue;

                    // Z Range
                    if ((iZ < z_range_min_) || (iZ > z_range_max_)) continue;

                    // scale from [mm] to [m]
                    float fZ = (float)(iZ) * 0.001f;
                    float fX = (float)(iX) * 0.001f;
                    float fY = (float)(iY) * 0.001f;
                    float fB = (float)(iB);

                    // locate point in meters
                    geometry_msgs::Point32 pt;
                    pt.z = fZ;
                    // first lib
                    pt.y = fX;
                    pt.x = fY;
                    //pt.x = fX;
                    //pt.y = fY;

                    // Fill PointCloud data
                    point_cloud.points.push_back( pt );
                    point_cloud.channels[0].values.push_back( fB );

                    // Fill PointCloud2 data
                    // first version of E70P lib
                    int adr = y * iNumX + x;

                    // new E70P lib 3d image is inverted left-right
                    //memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[0].offset], &fX, sizeof (float));
                    //memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[1].offset], &fY, sizeof (float));
                    memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[0].offset], &fY, sizeof (float));
                    memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[1].offset], &fX, sizeof (float));
                    memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[2].offset], &fZ, sizeof (float));
                    memcpy(&point_cloud2.data[adr*point_cloud2.point_step + point_cloud2.fields[3].offset], &fB, sizeof (float));

                    // Fill image data
                    //cv_float_image->at<float>(y, iCols - x) = fB;   // pImageLastOK[iRowPos+iNumX*0+x];  //B
                    cv_float_image->at<int16_t>(y, iCols - x) = iB;

                    cv_float_image_depth->at<uint16_t>(y, iCols - x) = -iZ; // in [mm]

                }

            }

            // Publish pointclouds
            fotonic_3Ddata_pub_.publish( point_cloud );
            fotonic_3Ddata_pub2_.publish( point_cloud2 );

            // Publish image
            cv_bridge::CvImage out_msg;
            out_msg.header.stamp = current_time;
            out_msg.header.frame_id = fotonic_frame_id_;
            out_msg.encoding = sensor_msgs::image_encodings::MONO16; // Or whatever
            out_msg.image = *cv_float_image; // the cv::Mat
            //image_pub_.publish(out_msg.toImageMsg());
            pub_.publish(out_msg.toImageMsg());


            // Publish image depth
            out_msg.image = *cv_float_image_depth; // the cv::Mat
            out_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            //image_depth_pub_.publish(out_msg.toImageMsg());
            pub_depth_.publish(out_msg.toImageMsg());

            //Publish Camera Info

            camera_info_msg_.header = out_msg.header;
            camera_info_pub_.publish(camera_info_msg_);
            camera_info_depth_pub_.publish(camera_info_msg_);

            // Publish the camera temperature (could be moved to diagnostics)
            std_msgs::Float32 temperature_msg;
            temperature_msg.data = (float)pHeader->temperature * 0.1;
            temperature_pub_.publish( temperature_msg );
            delete cv_float_image;
            delete cv_float_image_depth;
            delete pImageLastOK;

        }
	}

	int read_and_publish()
	{
        static double prevtime = 0;

		double starttime = ros::Time::now().toSec();
		if (prevtime && prevtime - starttime > 0.05)
		{
			ROS_WARN("Full fotonic_3dcamera loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
			was_slow_ = "Full fotonic_3dcamera loop was slow.";
			slow_count_++;
		}

		getData( );

		double endtime = ros::Time::now().toSec();
		if (endtime - starttime > max_delay)
		{
			ROS_WARN("Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full fotonic_3dcamera loop was slow.";
			slow_count_++;
		}

		prevtime = starttime;
		starttime = ros::Time::now().toSec();
		endtime = ros::Time::now().toSec();

		if (endtime - starttime > max_delay)
		{
			ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full fotonic_3dcamera loop was slow.";
			slow_count_++;
		}

		freq_diag_.tick();
		return(0);
	}

	bool spin()
	{
		ros::Rate r(FOTONIC_DESIRED_FREQ);
		while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
		{
			if (start() == 0)
			{
				while(node_handle_.ok()) {
					if(read_and_publish() < 0)
						break;
					self_test_.checkTest();
					diagnostic_.update();
					ros::spinOnce();
					r.sleep();
				}
			} else {
				// No need for diagnostic here since a broadcast occurs in start
				// when there is an error.
				sleep(1);
				self_test_.checkTest();
				ros::spinOnce();
			}
		}

		ROS_INFO("fotonicNode::spin - calling stop !");
		stop();
		return true;
	}

	void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
	{
		// connection test
		// TBC
		status.summary(0, "Connected successfully.");
	}

	void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
	{
		if (!running)
			status.summary(2, "fotonic_3dcamera is stopped");
		else if (!was_slow_.empty())
		{
			status.summary(1, "Excessive delay");
			was_slow_.clear();
		}
		else
			status.summary(0, "fotonic_3dcamera is running");

		status.add("Error count", error_count_);
		status.add("Excessive delay", slow_count_);
	}
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "fotonic_3dcamera_node");

  ros::NodeHandle nh;

  fotonicNode fn(nh);
  fn.spin();

  return(0);
}
