#include "aruco/aruco.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <aruco/cvdrawingutils.h>


using namespace std;


int main(int argc, char** argv){
//    if (argc != 2) {cerr<<"usage: inimage"<<endl; return -1;}
    aruco::MarkerDetector mDetector;
    mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
//    mDetector.enableErosion(true);

    mDetector.setThresholdParams(5,5);


    vector<aruco::Marker> markers;
    double marker_size =0.15;
    aruco::CameraParameters camParam;
    camParam.readFromFile("/home/waxz/github/opencv_cam_calibration/build/bin/out_camera_logitech.xml");
    cout<<camParam.CamSize;
//    camParam = aruco::CameraParameters();

    //Print parameters of aruco marker detector:
    cout<<"Corner refinement method: " << mDetector.getCornerRefinementMethod();
    cout<<"Threshold method: " << mDetector.getThresholdMethod();
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    cout<<"Threshold method: " << " th1: " << th1 << " th2: " << th2;
    float mins, maxs;
    mDetector.setMinMaxSize(0.1,1);
    mDetector.getMinMaxSize(mins, maxs);
    cout<<"Marker size min: " << mins << "  max: " << maxs;
    cout<<"Desired speed: " << mDetector.getDesiredSpeed();





    cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("gray", CV_WINDOW_AUTOSIZE);

    cv::VideoCapture cap(CV_CAP_ANY); // open any camera
    while (1) {
        cv::Mat frame;
        string fn ="/home/waxz/Pictures/marker582_5cm_margin_2cm.jpg";
        ///home/waxz/Pictures/aruco_mip_36h12_dict/aruco_mip_36h12_00000.png
//        frame = cv::imread(fn);
        if (cap.read(frame)) {
//        if (1) {

//            camParam.resize(frame.size());

            markers.clear();
//            mDetector.detect(frame,markers);
            mDetector.detect(frame, markers, camParam, marker_size, false);
            cout<<"get markers"<<markers.size();
            cv::Mat thrashimage = mDetector.getThresholdedImage();
            vector<vector <cv::Point2f> > P;
            mDetector.detectRectangles(thrashimage,P);
            cout<<"get rect"<<P.size()<<endl;
            vector<vector <cv::Point2f> > Pc = mDetector.getCandidates();


            if (!markers.empty())
                cout<<"find marker!"<<endl;
            else
                cout<<"fail!!"<<endl;

            if (!markers.empty()){
                for (size_t i=0;i<markers.size();i++){
                    markers[i].draw(thrashimage,cv::Scalar(110,255,255));
                    aruco::CvDrawingUtils::draw3dAxis(frame, markers[i], camParam);
                    aruco::CvDrawingUtils::draw3dCube(frame, markers[i], camParam);
                    tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
                    char d[100];
                    sprintf(d,"%.3f,%.3f,%.3f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
                    cv::putText(frame,d,cvPoint(30,30),cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(255,0,0), 1, CV_AA);
//                    cv::putText(frame, "Differencing the two images.", cvPoint(30,30),
//                            cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

                    cout<<"draw marker;";
                }
            }

            if (!P.empty()) {
                for (int i = 0; i < P.size(); i++) {
                    for (int j = 0; j < P[i].size(); j++) {
                        cv::circle(thrashimage,(cv::Point)P[i][j], 5, cv::Scalar(255, 0, 0));

                    }
                }
            }

            if (!Pc.empty()) {
                for (int i = 0; i < Pc.size(); i++) {
                    for (int j = 0; j < Pc[i].size(); j++) {
                        cv::circle(frame,(cv::Point)Pc[i][j], 5, cv::Scalar(110, 220, 0));

                    }
                }
            }




//            for(size_t i=0;i<markers.size();i++){
//                cout<<markers[i]<<endl;
//                markers[i].draw(frame,cv::Scalar(0,0,255),2);
//            }
            cv::imshow("Webcam", frame);
            cv::imshow("gray", thrashimage);



        }
        // process callback
        if (cv::waitKey(30) == 27) {
            // if "esc" is pressed end the program
            std::cout << "Closing the program because esc pressed";
            break;
        }
    }







}