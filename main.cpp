#include <iostream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

using namespace std;
using namespace cv;
using namespace zbar;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s){
	protonect_shutdown = true;
}

int main(){
	std::cout << "Streaming from Kinect One sensor!" << std::endl;

	//! [context]
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	//! [context]

	//! [discovery]
	if(freenect2.enumerateDevices() == 0){
		std::cout << "no device connected!" << std::endl;
		return -1;
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "SERIAL: " << serial << std::endl;

	if(pipeline){
		dev = freenect2.openDevice(serial, pipeline);
	} else {
		dev = freenect2.openDevice(serial);
	}

	if(dev == 0){
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;

	//! [listeners]
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
			libfreenect2::Frame::Depth |
			libfreenect2::Frame::Ir);
	libfreenect2::FrameMap frames;

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	//! [listeners]
	
	zbar::ImageScanner scanner;
	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
	//! [start]
	dev->start();

	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	//! [start]
	Mat rgbmat, depthmat, depthmatUndistorted, irmat, rgbd, rgbd2;
	//! [loop start]
	while(!protonect_shutdown){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		// libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		// libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		//! [loop start]

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		// cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		// cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		cv::Mat res(rgbmat,cv::Rect(420,0,1000,1000));
		cv::flip(res,res,1);	//reverce

		// cv::imshow("rgb", rgbmat);
		// cv::imshow("ir", irmat / 4096.0f);
		// cv::imshow("depth", depthmat / 4096.0f);

		Mat gray;
		cvtColor(res,gray,CV_BGR2GRAY);
		int width = res.cols;  
		int height = res.rows; 
		uchar *raw = (uchar *)gray.data;
		// wrap image data  
		zbar::Image image(width, height, "Y800", raw, width*height);  
		cout << width << ":" << height << endl;
		// scan the image for barcodes  
		int n = scanner.scan(image); 
		for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  
				symbol != image.symbol_end();++symbol) {  
			vector<Point> vp;  
			// do something useful with results  
			cout << "symbol \"" << symbol->get_data() <<'"'<< endl;  
			int n = symbol->get_location_size();  
			for(int i=0;i<n;i++){  
				vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i))); 
			}  
			RotatedRect r = minAreaRect(vp);  
			Point2f pts[4];  
			r.points(pts);  
			for(int i=0;i<4;i++){  
				line(res,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);  
			}  
			//cout<<"Angle: "<<r.angle<<endl;  
		}  

		cv::imshow("res", res);
		protonect_shutdown = protonect_shutdown || (cv::waitKey(1) == 27); // shutdown on escape

		//! [loop end]
		listener.release(frames);
	}
	//! [loop end]

	//! [stop]
	dev->stop();
	dev->close();
	//! [stop]

	std::cout << "Streaming Ends!" << std::endl;
	return 0;
}

