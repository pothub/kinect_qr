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

#define err -1
using namespace cv;

bool protonect_shutdown = false; // Whether the running application should shut down.

void sigint_handler(int s){
	protonect_shutdown = true;
}
int errMes(string mes){
	std::cout << mes << std::endl;
	return err;
}

libfreenect2::Freenect2Device *dev = 0;
libfreenect2::Freenect2 freenect2;
libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color |
		libfreenect2::Frame::Depth | libfreenect2::Frame::Ir);
zbar::ImageScanner scanner;
int locationAveX=0,locationAveY=0;

int setupKinect();
void startKinect();
void markQr(const Mat& src, Mat& dst){
	Mat gray;
	dst = src.clone();
	cvtColor(src,gray,CV_BGR2GRAY);
	int width = src.cols;  
	int height = src.rows; 
	uchar *raw = (uchar *)gray.data;
	// wrap image data  
	zbar::Image image(width, height, "Y800", raw, width*height);  
	// scan the image for barcodes  
	int n = scanner.scan(image); 
	for(zbar::Image::SymbolIterator symbol = image.symbol_begin();  
			symbol != image.symbol_end();++symbol) {  
		vector<Point> vp;  
		// do something useful with results  
		// std::cout << "symbol \"" << symbol->get_data() <<'"'<< std::endl;  
		locationAveX=0;
		locationAveY=0;
		int n = symbol->get_location_size();  
		for(int i=0;i<n;i++){  
			vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i))); 
			locationAveX += symbol->get_location_x(i);
			locationAveY += symbol->get_location_y(i);
		}  
		locationAveX /= 4;
		locationAveY /= 4;
		// std::cout<<"x"<<locationAveX<<"y"<<locationAveY<<std::endl;
		RotatedRect r = minAreaRect(vp);  
		Point2f pts[4];  
		r.points(pts);  
		for(int i=0;i<4;i++){  
			line(dst,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);  
		}  
		cv::circle(dst,Point(locationAveX,locationAveY),10,cv::Scalar(255,0,0),3,3);
		//cout<<"Angle: "<<r.angle<<endl;  
	}  
}

int main(){
	std::cout << "Streaming from Kinect One sensor!" << std::endl;
	if(setupKinect() != true){
		return errMes("failure discoverKinect");
	}
	startKinect();

	libfreenect2::Registration* registration = 
		new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

	scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

	libfreenect2::FrameMap frames;
	Mat rgbmat, depthmat, rgbMatRegistered, irmat;
	//! [loop start]
	while(!protonect_shutdown){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		// libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		//! [loop start]

		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		// cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);

		cv::Mat rgbMatRegistered=cv::Mat(registered.height,registered.width,CV_8UC4,
				registered.data);

		cv::Mat res;
		// cv::Mat res(rgbmat,cv::Rect(420,0,1000,1000));
		cv::resize(rgbmat,res,cv::Size(),0.5,0.5);
		cv::flip(res,res,1);	//reverce
		cv::flip(rgbMatRegistered,rgbMatRegistered,1);	//reverce

		// cv::imshow("ir", irmat / 4096.0f);
		// cv::imshow("depth", depthmat / 4096.0f);

		// markQr(res,res);
		markQr(rgbMatRegistered,rgbMatRegistered);
		registration->apply(rgb, depth, &undistorted, &registered);
		float x,y,z;
		registration->getPointXYZ(&undistorted,locationAveX,locationAveY,x,y,z);
		std::cout<<"x:"<<x<<"y:"<<y<<"z:"<<z<<std::endl;

		cv::flip(res,res,1);	//reverce
		cv::flip(rgbMatRegistered,rgbMatRegistered,1);	//reverce
		// cv::imshow("res", res);
		cv::imshow("rgbMatRegistered", rgbMatRegistered);
		protonect_shutdown = protonect_shutdown || (cv::waitKey(1) == 27); // shutdown on escape

		listener.release(frames);
	}

	//! [stop]
	dev->stop();
	dev->close();
	//! [stop]

	std::cout << "Streaming Ends!" << std::endl;
	return 0;
}

int setupKinect(){
	libfreenect2::PacketPipeline *pipeline = 0;
	if(freenect2.enumerateDevices() == 0){
		return errMes("no device connected!");
	}
	string serial = freenect2.getDefaultDeviceSerialNumber();
	std::cout << "SERIAL: " << serial << std::endl;

	if(pipeline){
		dev = freenect2.openDevice(serial, pipeline);
	} else {
		dev = freenect2.openDevice(serial);
	}
	if(dev == 0){
		errMes("failure opening device!");
	}

	// set emergency stop handler
	signal(SIGINT, sigint_handler);
	protonect_shutdown = false;
	// set listener color,ir,depth
	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	return true;
}
void startKinect(){
	dev->start();
	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
}
