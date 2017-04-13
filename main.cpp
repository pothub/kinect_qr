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
// int locationAveX=0,locationAveY=0;

int setupKinect();
void startKinect();
void markQr(const Mat& src, Mat& dst,int *qrx, int *qry){
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
		*qrx=0;
		*qry=0;
		int n = symbol->get_location_size();  
		for(int i=0;i<n;i++){  
			vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i))); 
			*qrx += symbol->get_location_x(i);
			*qry += symbol->get_location_y(i);
		}  
		*qrx /= 4;
		*qry /= 4;
		// std::cout<<"x"<<locationAveX<<"y"<<locationAveY<<std::endl;
		RotatedRect r = minAreaRect(vp);  
		Point2f pts[4];  
		r.points(pts);  
		for(int i=0;i<4;i++){  
			line(dst,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);  
		}  
		cv::circle(dst,Point(*qrx,*qry),10,cv::Scalar(255,0,0),3,3);
		*qrx = 512-*qrx;
		// *qrx *= -1;		//to reverce
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

	FILE *gp;
	// gp = popen("gnuplot","w");
	gp = popen("gnuplot -persist","w");
	fprintf(gp,"splot [-2:2][-2:2][-2:2] 0\n");
	// fprintf(gp,"set label 1 point pt 4 at %f,%f,%f\n",0,0,0);
	// fprintf(gp,"set label 1 point pt 4 at 0,0,0\n");
	// fprintf(gp,"replot\n");
	// pclose(gp);
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

		int cutX=260;
		// cv::Mat res;
		cv::Mat resRgb(rgbmat,cv::Rect(cutX,0,1920-cutX-197,1080));
		cv::resize(resRgb,resRgb,cv::Size(),0.35,0.35);
		cv::Mat resDepth(depthmat/4096.0f,cv::Rect(0,0,512,378));
		// cv::flip(res,res,1);	//reverce
		cv::flip(rgbMatRegistered,rgbMatRegistered,1);	//reverce
		int qrx=0,qry=0;
		markQr(rgbMatRegistered,rgbMatRegistered,&qrx,&qry);
		cv::flip(rgbMatRegistered,rgbMatRegistered,1);	//reverce
		// cv::imshow("ir", irmat / 4096.0f);
		// cv::imshow("depth", depthmat / 4096.0f);

		registration->apply(rgb, depth, &undistorted, &registered);
		float x,y,z;
		registration->getPointXYZ(&undistorted,qry,qrx,x,y,z);

		static int cnt=0;
		if(cnt++>5 && z < 1.5){
			pclose(gp);
			// gp = popen("gnuplot","w");
			gp = popen("gnuplot -persist","w");
			fprintf(gp,"splot [-0.5:0.5][0.5:-0.5][0.7:1.5] 0.7\n");
			fprintf(gp,"set xlabel 'x[m]'\n");
			fprintf(gp,"set ylabel 'y[m]'\n");
			fprintf(gp,"set zlabel 'z[m]'\n");
			fprintf(gp,"set label 1 point pt 4 at %f,%f,%f\n",x,y,z);
			fprintf(gp,"set label 2 point pt 3 at %f,%f,%f\n",x,0.5,0.7);
			fprintf(gp,"set label 3 point pt 3 at %f,%f,%f\n",0.5,y,0.7);
			fprintf(gp,"set label 4 point pt 3 at %f,%f,%f\n",-0.5,0.5,z);
			// fprintf(gp,"set label 1 point pt 4 at 0,0,0\n");
			fprintf(gp,"replot\n");
			cnt=0;
		}
		std::cout<<x<<":"<<y<<":"<<z<<":"<<resDepth.at<float>(qry,qrx)*4<<":"<<qrx<<std::endl;
		cv::circle(resRgb,Point(qrx,qry),10,cv::Scalar(0,255,0),3,3);
		// cv::flip(res,res,1);	//reverce
		// cv::flip(rgbMatRegistered,rgbMatRegistered,1);	//reverce
		cv::imshow("rgbMatRegistered", rgbMatRegistered);
		cv::imshow("resRgb", resRgb);
		cv::imshow("depthmat", resDepth);
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
