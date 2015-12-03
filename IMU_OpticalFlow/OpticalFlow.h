//
//  OpticalFlow.h
//  IMU_OpticalFlow
//
//  Created by Priya Ganadas on 11/22/15.
//  Copyright © 2015 Priya Ganadas. All rights reserved.
//

#ifndef OpticalFlow_h
#define OpticalFlow_h

#import <Foundation/Foundation.h>
#import <opencv2/opencv.hpp>
#import <opencv2/highgui/cap_ios.h>

//#define FARNEBECK_DENSE 0
//#define LUCAS_KANADE_SPARSE 1


@interface OpticalFlow : NSObject<CvVideoCameraDelegate>
@property(nonatomic, readwrite) int type;

@end


#endif /* OpticalFlow_h */
