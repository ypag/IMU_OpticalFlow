//
//  ViewController.h
//  IMU_OpticalFlow
//
//  Created by Priya Ganadas on 11/21/15.
//  Copyright © 2015 Priya Ganadas. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <opencv2/highgui/cap_ios.h>
#import <opencv2/objdetect/objdetect.hpp>
#import <opencv2/imgproc/imgproc_c.h>

using namespace cv;

@interface ViewController : UIViewController<CvVideoCameraDelegate>
{
    IBOutlet UIImageView* imageView;
    
   
   }


@end

