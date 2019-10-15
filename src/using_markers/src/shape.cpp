#include <eigen3/Eigen/Dense> //先用，否则报错
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

using namespace Eigen;
using namespace cv;
using namespace std;


int main(int argc, char **argv) {

  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(0.1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x=marker.pose.position.y=marker.pose.position.z=0;

    VideoCapture capture("/home/linyihong/rosopencv/src/rosopencv/src/water.avi");
    Mat src0;
    Mat erzhihua;
    float angle;

    while(capture.isOpened())
    {
        capture>>src0;
        //imshow("corner_img",corner_img);
        inRange(src0,Scalar(0,50,60),Scalar(200,255,255),erzhihua);
         Mat element=getStructuringElement(MORPH_RECT,Size(9,9));
        erode(erzhihua,erzhihua,element);
        dilate(erzhihua,erzhihua,element);
        //imshow("1",erzhihua);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(erzhihua,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);

            Mat drawing1 = Mat::zeros( erzhihua.size(), CV_8UC3 );
            for( int i = 0; i< contours.size(); i++ )
            {if(contourArea(contours[i])>500)
                 drawContours( drawing1, contours, i, Scalar( 80, 80, 80 ), 1, LINE_8, hierarchy, 0 );
            }
            
            vector<RotatedRect> minRect( contours.size() );
            Point2f aimpoint[4];
            for( int i = 0; i <2 ;i++ )
            {
                Point2f point[4];
                minRect[i] = minAreaRect( contours[i] );
                Point2f rect_points[4];
                minRect[i].points( rect_points );
                for ( int j = 0; j < 4; j++ )
                {
                    line( drawing1, rect_points[j], rect_points[(j+1)%4], Scalar( 120, 120, 120 ) );
                    point[j]=Point2f((rect_points[j].x+rect_points[(j+1)%4].x)/2,
                                            (rect_points[j].y+rect_points[(j+1)%4].y)/2);
                    if(j==0) {aimpoint[i]=point[j];aimpoint[i+2]=point[j];continue;}
                    if(j!=0&&point[j].y>aimpoint[i].y) aimpoint[i]=point[j];
                    if(j!=0&&point[j].y<aimpoint[i+2].y) aimpoint[i+2]=point[j];
                }
            }
            imshow("showing",drawing1); 
            
            circle(src0,aimpoint[0],1,Scalar(20,20,20),2);
             circle(src0,aimpoint[1],1,Scalar(20,20,20),2);
              circle(src0,aimpoint[2],1,Scalar(20,20,20),2);
               circle(src0,aimpoint[3],1,Scalar(20,20,20),2);

        float length,width;
            
        line( drawing1, aimpoint[0], aimpoint[1], Scalar( 200, 200, 120 ) );
        line( drawing1, aimpoint[1], aimpoint[3], Scalar( 200, 200, 120 ) );
        line( drawing1, aimpoint[3], aimpoint[2], Scalar( 200, 200, 120 ) );
        line( drawing1, aimpoint[2], aimpoint[0], Scalar( 200, 200, 120 ) );//顺时针方向

        imshow("showing2",drawing1);
        imshow("src0",src0);
        vector<Point2f> corners;

        corners.push_back(aimpoint[1]);
        corners.push_back(aimpoint[3]);
        corners.push_back(aimpoint[2]);
        corners.push_back(aimpoint[0]);


        vector<Point3f> objectPoints;
        //float Length,Width;
        //Length=(corners[1].x-corners[0].x)/2;
        //Width=(corners[1].y-corners[2].y)/2;
        
        objectPoints.push_back((Point3f(-6.7, 2.7, 0)));
        objectPoints.push_back((Point3f(-6.7, -2.7, 0)));
        objectPoints.push_back((Point3f(6.7, -2.7, 0)));
        objectPoints.push_back((Point3f(6.7, 2.7, 0)));
        //cout<<objectPoints<<endl;

        //Mat cameraMatrix(3,3,CV_32F);
        Mat cameraMatrix =(Mat_<double>(3,3)<<1128.048344,0 ,339.421769 ,  0, 1127.052190,236.535242 ,  0, 0 ,1);
        Mat distCoeffs=(Mat_<double>(5,1) <<-0.568429,0.514592,-0.000126,0.000500,0.000000);
    
        //vector<float>  distCoeffs= { -0.568429,0.514592,-0.000126,0.000500,0.000000};
        //float tempMatrix[3][3] = { { 1128.048344,0 ,339.421769 }, { 0, 1127.052190,236.535242 }, { 0, 0 ,1} };
        
        Mat tvec;//(3, 1, DataType<double>::type);
        Mat rvec;//(3, 3, DataType<double>::type);
        //rvec:输出旋转矢量，与tvec一起，将点从模型坐标系带到相机坐标系
        //tvec:输出转换向量
        solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
        cout<<tvec<<endl;
        Rodrigues(rvec, rvec);//由于solvePnP返回的是旋转向量，故用罗德里格斯变换变成旋转矩阵
        //Rodrigues(tvec, tvec);
        Eigen::Matrix<double, 1, 3> t;
        Eigen::Matrix<double, 3, 3> r;
        cv2eigen(rvec,r);

        //cout<< tvec.at<double>(0,0)<<' '<<tvec.at<double>(0,1)<<' '<<tvec.at<double>(0,2)<<endl;

        Eigen::MatrixXd eigenMat(1,3);
        eigenMat(0,0)=tvec.at<double>(0,0);eigenMat(0,1)=tvec.at<double>(0,1);eigenMat(0,2)=tvec.at<double>(0,2);
        Quaterniond q(r);

        cout<<eigenMat(0,0)<<' '<<eigenMat(0,1)<<' '<<eigenMat(0,2)<<endl<<endl;
        cout<<r(0,0)<<' '<<r(1,0)<<' '<<r(2,0)<<endl;
        cout<<r(0,1)<<' '<<r(1,1)<<' '<<r(2,1)<<endl;
        cout<<r(0,2)<<' '<<r(1,2)<<' '<<r(2,2)<<endl;
        //cv2eigen(tvec,t);

        //q.coeffs(); //输出系数 
        //cout<<r<<endl;

    //float sy = sqrt(rvec.at<double>(2,1) * rvec.at<double>(2,1) +  rvec.at<double>(2,2) * rvec.at<double>(2,2) );
  
        //float x,y,z;
        //x = atan2(rvec.at<double>(2,1) , rvec.at<double>(2,2));
        //y = atan2(-rvec.at<double>(2,0), sy);
        //z = atan2(rvec.at<double>(1,0), rvec.at<double>(0,0));

    //cout<<x<<" "<<y<<" "<<z<<endl;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        
        marker.pose.position.x = eigenMat(0,0)/20;
        marker.pose.position.y = eigenMat(0,2)/20;
        marker.pose.position.z = -eigenMat(0,1)/20;
double pi=3.14159;
        marker.pose.orientation.x =q.x(); //x;
        marker.pose.orientation.y =-q.y();// z;
        marker.pose.orientation.z =q.z(); //-y;
        marker.pose.orientation.w =q.w();//= 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side

        marker.scale.x = 0.675;//y

        marker.scale.y = 0.075;//z

        marker.scale.z = 0.275;//x

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    waitKey(100);
    }
  r.sleep();
    waitKey(0);
}
