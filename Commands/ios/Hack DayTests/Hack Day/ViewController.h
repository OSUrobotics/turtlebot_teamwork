//
//  ViewController.h
//  Hack Day
//
//  Created by Patrick on 8/12/15.
//  Copyright (c) 2015 Personal Robotics Group. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface ViewController : UIViewController <NSStreamDelegate, UIGestureRecognizerDelegate>

@property NSInputStream *inputStream;
@property NSOutputStream *outputStream;
@property float theta;
@property CGPoint startPoint;
@property CGPoint endPoint;

@property (nonatomic, weak) IBOutlet UIImageView *imageView;

- (IBAction)emergencyStop:(id)sender;

@end

