//
//  ViewController.m
//  IMU_OpticalFlow
//
//  Created by Priya Ganadas on 11/21/15.
//  Copyright © 2015 Priya Ganadas. All rights reserved.
//

#import "ViewController.h"

#ifdef __cplusplus
#import <opencv2/opencv.hpp>
#import "opencv2/highgui/ios.h"
#endif

#ifdef __OBJC__
#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>
#endif

// Include iostream and std namespace so we can mix C++ code in here
#include <iostream>
using namespace std;

@interface ViewController () {
    
    UIImageView *liveView_; // Live output from the camera
    CvVideoCamera *videoCamera_;
    
}
@end


@implementation ViewController


- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    
    // 1. Setup the your OpenCV view, so it takes up the entire App screen......
    int view_width = self.view.frame.size.width;
    int view_height = (640*view_width)/480; // Work out the view-height assuming 640x480 input
    int view_offset = (self.view.frame.size.height - view_height)/2;
    liveView_ = [[UIImageView alloc] initWithFrame:CGRectMake(0.0, view_offset, view_width, view_height)];
    [self.view addSubview:liveView_]; // Important: add liveView_ as a subview
    UIView *myBox  = [[UIView alloc] initWithFrame:CGRectMake(130, 150, 500, 700)];
    myBox.backgroundColor = [UIColor colorWithWhite:0.7 alpha:0.3];
    [self.view addSubview:myBox];
    
    
    // 2. Initialize the camera parameters and start the camera (inside the App)
    videoCamera_ = [[CvVideoCamera alloc] initWithParentView:liveView_];
    videoCamera_.delegate = self;
    
    // This chooses whether we use the front or rear facing camera
    videoCamera_.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    
    // This is used to determine the device orientation
    videoCamera_.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    
    //orientation shows sideways even if it is portait, these two lines are added to fix this problem
   // CGAffineTransform xform = CGAffineTransformMakeRotation(-M_PI / 2);
   // liveView_.transform = xform;
    self-> videoCamera_.rotateVideo = YES;
    
    // This starts the camera capture
    [videoCamera_ start];

}

void draw_motion_comp(Mat& img, int x_coordinate, int y_coordinate, int width, int height, double angle,Mat& result)
{
    rectangle(img,cv::Point(x_coordinate,y_coordinate), cv::Point(x_coordinate+width,y_coordinate+width), Scalar(255,0,0), 1, 8, 0);
    int r,cx,cy;
    if(height/2 <= width/2)
        r = height/2;
    else
        r = width/2;
    cx = x_coordinate + width/2;
    cy = y_coordinate + height/2;
    angle = angle*M_PI/180;
    circle(img, cv::Point(cx,cy), r, Scalar(255,0,0),1, 8, 0);
    line(img, cv::Point(cx,cy), cv::Point(int(cx+cos(angle)*r), int(cy+sin(angle)*r)), Scalar(255,0,0), 1, 8, 0);
    result = img.clone();
}

void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,double, const cv::Scalar& color)
{
    for(int y = 0; y < cflowmap.rows; y += step)
        for(int x = 0; x < cflowmap.cols; x += step)
        {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
                 color);
            circle(cflowmap, cv::Point(x,y), 1, CV_RGB(200, 0, 0), -1);
        }
    
}
static void drawArrows(cv::Mat frame, const cv::vector<cv::Point2f>& prevPts, const cv::vector<cv::Point2f>& nextPts, const cv::vector<uchar>& status,
                       cv::Scalar line_color = cv::Scalar(0, 0, 255))

{
    //cv:: Scalar RED = cv::Scalar(255,0,0);
    for (size_t i = 0; i < prevPts.size(); ++i)
    {
        if (status[i])
        {
            int line_thickness = 1;
            
            cv::Point p = prevPts[i];
            cv::Point q = nextPts[i];
            
            double angle = atan2((double) p.y - q.y, (double) p.x - q.x);
            
            double hypotenuse = sqrt( (double)(p.y - q.y)*(p.y - q.y) + (double)(p.x - q.x)*(p.x - q.x) );
            
            if (hypotenuse < 1.0)
                continue;
            
            // Here we lengthen the arrow by a factor of three.
            q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
            q.y = (int) (p.y - 3 * hypotenuse * sin(angle));
            
            // Now we draw the main line of the arrow.
            cv:: Scalar RED = cv::Scalar(255,0,0);
            cv:: Scalar BLUE = cv::Scalar(0,0,255);
            line(frame, p, q, line_color, line_thickness);
            circle(frame, q, 5,RED);
            circle(frame, p, 5,BLUE);
            
            // Now draw the tips of the arrow. I do some scaling so that the
            // tips look proportional to the main line of the arrow.
            
            p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);
            
            p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
            line(frame, p, q, line_color, line_thickness);
        }
    }
}


cv::Mat prev_image;
cv::Mat prev_features;
float MHI_DURATION = 0.05;
int DEFAULT_THRESHOLD = 32;
float MAX_TIME_DELTA = 12500.0;
float MIN_TIME_DELTA = 5;
//int visual_trackbar = 2;
void draw_motion_comp(Mat& img, int x_coordinate, int y_coordinate, int width, int height, double angle,Mat& result);

#pragma mark - Protocol CvVideoCameraDelegate

#ifdef __cplusplus
- (void)processImage:(Mat&)image;
{
    //------------------------------optical flow---------------------------------------------------
    cv::Mat grey_image;
    cv::Mat flow;
    cv::vector<cv::Mat> channels;
    split(image, channels);
    cv::vector<uchar> status;
    cv::vector<cv::Point2f> features;
    cv::vector<cv::Point2f> nextPoints;
    cv::Mat downsampled_grey( image.rows, image.cols, CV_8UC1 );;
    cv::Mat output[] = { downsampled_grey };
    int from_to[] = { 0,0 };
    
    cv::mixChannels(&image, 1, output, 1, from_to, 1);
    
    cv::cvtColor(image, downsampled_grey, CV_BGR2GRAY);
    
    cv::Mat err;
    
    if (prev_image.empty()) {
        downsampled_grey.copyTo(prev_image);
    }
    cv::goodFeaturesToTrack(downsampled_grey, features, 20, 0.1, 0.2);
    cv::calcOpticalFlowPyrLK(prev_image, downsampled_grey, features, nextPoints, status, err);
    //cout << nextPoints << endl;
    drawArrows(image, features, nextPoints, status);
    //cout<< features.size() << endl;
   
    //This is alternative dense optical flow implementation
   // cv::calcOpticalFlowFarneback(prev_image, downsampled_grey, flow, 0.2, 3, 14, 3, 5, 1.2, 0);
    //drawOptFlowMap(flow, image, 12, 1.6, CV_RGB(0, 255, 0));
    
    //Computing the motion by estimating the difference between the next and previous points
    cv::Mat samples(features.size(), 2, CV_32F);
    for(unsigned int n = 0; n < features.size(); n++)
    {
        samples.at<float>(n,0) = nextPoints[n].x - features[n].x;
        samples.at<float>(n,1) = nextPoints[n].y - features[n].y;
    }
    
   // cout<< samples.size() << endl;
    
    
    //------------------------------------Motion segmentation -----------------------------------
    /*cv::Mat frame,ret,frame_diff,gray_diff,motion_mask;
    frame = image.clone();
    ret = frame.clone();
    cv::Size frame_size = frame.size();
    int h = frame_size.height;
    int w = frame_size.width;
 
    double timestamp = 1000.0*clock()/CLOCKS_PER_SEC;
    //cout<< timestamp << endl;
    Mat prev_frame = frame.clone();
    Mat motion_history(h,w, CV_32FC1,Scalar(0,0,0));
    Mat hsv(h,w, CV_8UC3,Scalar(0,255,0));
    Mat mg_mask(h,w, CV_8UC1,Scalar(0,0,0));
    Mat mg_orient(h,w, CV_32FC1,Scalar(0,0,0));
    Mat seg_mask(h,w, CV_32FC1,Scalar(0,0,0));
    vector<cv::Rect> seg_bounds;
    String visual_name;
    cv::Mat vis(h,w,CV_32FC3);
    cv::Mat vis1(h,w,CV_8UC1);
    cv::Mat silh_roi,orient_roi,mask_roi,mhi_roi;
    
   
    absdiff(frame, prev_frame, frame_diff);
    cvtColor(frame_diff,gray_diff, CV_BGR2GRAY );
    threshold(gray_diff,ret,DEFAULT_THRESHOLD,255,0);
    motion_mask = ret.clone();
    
    updateMotionHistory(motion_mask, motion_history, timestamp, MHI_DURATION);
    calcMotionGradient(motion_history, mg_mask, mg_orient, MIN_TIME_DELTA, MAX_TIME_DELTA, 3);
    segmentMotion(motion_history, seg_mask, seg_bounds, timestamp, 32);
    //cout<< motion_history <<endl;
    
    for(unsigned int h = 0; h < seg_bounds.size(); h++)
    {
        cv::Rect rec = seg_bounds[h];
        if(rec.area() > 5000 && rec.area() < 70000)
        {
            rectangle(vis, rec,Scalar(0,0,255),3);
            silh_roi = motion_mask(rec);
            orient_roi = mg_orient(rec);
            mask_roi = mg_mask(rec);
            mhi_roi = motion_history(rec);
            if(norm(silh_roi, NORM_L2, noArray()) > rec.area()*0.5)
            {
                double angle = calcGlobalOrientation(orient_roi, mask_roi, mhi_roi,timestamp, MHI_DURATION);
                cout << rec.width << endl;
                draw_motion_comp(vis, rec.x, rec.y, rec.width, rec.height,angle,image);
            }			
        }            	
    }
    prev_frame = frame.clone();
    */
    
    //---------------------------------------clustering the optical flow to find centroids----------------------------
    // Performing kmeans clustering
    Mat labels;
    int clusterCount = 2;
    Mat centers;
    
    kmeans(samples, clusterCount, labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 3, KMEANS_PP_CENTERS, centers);
    
    //cout<< centers << endl;
    //cout<< centers.at<float>(0, 1) << endl;
    cv:: Scalar RED = cv::Scalar(255,0,0);
    for (int i=0; i<= clusterCount; i++)
    {
        cv::Point centroid;
        centroid.x = centers.at<float>(i, 0)+640;
        centroid.y = centers.at<float>(i, 1)+480;
        
        circle(image, centroid, 5,RED);
    }

    downsampled_grey.copyTo(prev_image);
    
}
#endif


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
