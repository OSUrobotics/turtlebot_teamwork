//
//  ViewController.m
//  Hack Day
//
//  Created by Patrick on 8/12/15.
//  Copyright (c) 2015 Personal Robotics Group. All rights reserved.
//
//  Patrick Hansen

#import "ViewController.h"

@interface ViewController () {
    NSMutableArray *messages;
    NSMutableArray *turtles;
}

@end

@implementation ViewController

@synthesize inputStream;
@synthesize outputStream;
@synthesize theta;
@synthesize startPoint;
@synthesize endPoint;

@synthesize imageView;


- (void)viewDidLoad {
    [super viewDidLoad];
    
    [self initNetworkCommunication];
    [self setUpGestures];
    
    // Grab the map image from github
    UIImage *image = [UIImage imageWithData:[NSData dataWithContentsOfURL:[NSURL URLWithString:@"https://raw.githubusercontent.com/OSUrobotics/turtlebot_teamwork/master/Maps/map_lab_08_12.png"]]];
    [imageView setImage:image];
    
    // Keep a log of messages from the server
    messages = [[NSMutableArray alloc] init];
    
    // Keep a tab on the current TurtleBots that need to be displayed on the map
    turtles = [[NSMutableArray alloc] init];
}

- (void)stream:(NSStream *)theStream handleEvent:(NSStreamEvent)streamEvent {
    
    switch (streamEvent) {
            
        case NSStreamEventOpenCompleted:
            NSLog(@"Stream opened");
            break;
            
        case NSStreamEventHasBytesAvailable:
            
            if (theStream == inputStream) {
                
                uint8_t buffer[1024];
                int len;
                
                while ([inputStream hasBytesAvailable]) {
                    len = [inputStream read:buffer maxLength:sizeof(buffer)];
                    if (len > 0) {
                        
                        NSString *output = [[NSString alloc] initWithBytes:buffer length:len encoding:NSASCIIStringEncoding];
                        
                        if (nil != output) {
                            NSLog(@"server said: %@", output);
                            [self messageReceived:output];
                        }
                    }
                }
            }
            break;
        case NSStreamEventErrorOccurred:
            NSLog(@"Can not connect to the host!");
            break;
            
        case NSStreamEventEndEncountered:
            break;
            
        default:
            NSLog(@"Unknown event");
    }
    
}

// Called when a messaged is received from the server (will contain TurtleBot poses)
- (void) messageReceived:(NSString *)message {
    
    [messages addObject:message];
    
    NSArray *positions = [message componentsSeparatedByString:@","];
    for (int i = 1; i < 3*[positions[0] intValue] + 1; i += 3) {
        float x = [positions[i] floatValue];
        float y = [positions[i + 1] floatValue];
        float t = [positions[i + 2] floatValue];
        
        // Transform from map to screen coordinates
        x = (x + 10.860755)/0.01976356368;
        y = (y - 13.07315675)/-0.01965999352;
        
        // Create a red dot TurtleBot to populate the map
        UIImageView *turtle = [[UIImageView alloc] initWithImage:[UIImage imageNamed:@"red-dot.png"]];
        turtle.center = CGPointMake(x, y);
        
        [turtles addObject:turtle];
        
        // Put all of the active TurtleBots on the map
        for (UIImageView *turt in turtles) {
            [self.view insertSubview:turt aboveSubview:imageView];
        }
    }
}

// Add tap, pan, and pinch gestures
- (void)setUpGestures {
    UITapGestureRecognizer *tapRecognizer = [[UITapGestureRecognizer alloc] initWithTarget:self action:@selector(mapTouched:)];
    [tapRecognizer setNumberOfTapsRequired:2];
    [self.view addGestureRecognizer:tapRecognizer];
    
    UIPanGestureRecognizer *panRecognizer = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(panned:)];
    [self.view addGestureRecognizer:panRecognizer];
    
    UIPinchGestureRecognizer *pinchRecognizer = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(pinched:)];
    [self.view addGestureRecognizer:pinchRecognizer];
}


// Set up communication with the server
- (void)initNetworkCommunication {
    CFReadStreamRef readStream;
    CFWriteStreamRef writeStream;
    
    // IP and port number
    CFStreamCreatePairWithSocketToHost(NULL, (CFStringRef)@"10.214.152.18", 11411, &readStream, &writeStream);
    inputStream = (__bridge NSInputStream *)readStream;
    outputStream = (__bridge NSOutputStream *)writeStream;
    
    [inputStream setDelegate:self];
    [outputStream setDelegate:self];
    
    [inputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    [outputStream scheduleInRunLoop:[NSRunLoop currentRunLoop] forMode:NSDefaultRunLoopMode];
    
    [inputStream open];
    [outputStream open];
    
}

- (IBAction)emergencyStop:(id)sender {

    NSLog(@"KILL");
    NSString *response = [NSString stringWithFormat:@"KILL,%f,%f,0.0,%f", 0.0, 0.0, 0.0];
    [self sendMessage:response];
    
}

#pragma mark - Gesture Recognizers
- (void)mapTouched:(UIGestureRecognizer *)sender {

    CGPoint tapPoint = [sender locationInView:self.view];
    float tapX = tapPoint.x;
    float tapY = tapPoint.y;
    NSLog(@"TAPPED X:%f Y:%f", tapX, tapY);
    
    // Transform from screen to map coordinates
    float mapX = 0.01976356368*tapX - 10.860755;
    float mapY = -0.01965999352*tapY + 13.07315675;
    theta = 0.0;
    NSLog(@"TAPPED: ");
    NSLog(@"MAP X:%f Y:%f", mapX, mapY);
    
    NSString *response  = [NSString stringWithFormat:@"CIRCLE,%f,%f,0.0,%f", mapX, mapY, theta];
    [self sendMessage:response];
    
}

// Send string to server
- (void)sendMessage:(NSString *)message {
    NSData *data = [[NSData alloc] initWithData:[message dataUsingEncoding:NSASCIIStringEncoding]];
    [outputStream write:[data bytes] maxLength:[data length]];
}

- (void)panned:(UIPanGestureRecognizer *)gestureRecognizer {
    CGPoint locationPoint = [gestureRecognizer locationInView:self.view];
    if (gestureRecognizer.state == UIGestureRecognizerStateBegan) {
        startPoint = locationPoint;
    } else if (gestureRecognizer.state == UIGestureRecognizerStateEnded) {
        endPoint = locationPoint;
        NSLog(@"LINED: ");
        NSLog(@"started: %f %f ended: %f %f", startPoint.x, startPoint.y, endPoint.x, endPoint.y);
        theta = atan2f(endPoint.y - startPoint.y, endPoint.x - startPoint.x);
        NSLog(@"theta: %f", theta*180/3.1415926535);
        NSString *response  = [NSString stringWithFormat:@"DIRECTION,%f,%f,0.0,%f", 0.0, 0.0, theta];
        [self sendMessage:response];
    }
}

- (void)pinched:(UIPinchGestureRecognizer *)gestureRecognizer {
    CGPoint locationPoint = [gestureRecognizer locationInView:self.view];
    if (gestureRecognizer.state == UIGestureRecognizerStateBegan) {
        startPoint = locationPoint;
    } else if (gestureRecognizer.state == UIGestureRecognizerStateEnded) {
        endPoint = locationPoint;
        NSLog(@"PINCHED: ");
        NSLog(@"started: %f %f ended: %f %f", startPoint.x, startPoint.y, endPoint.x, endPoint.y);
        theta = atan2f(endPoint.y - startPoint.y, endPoint.x - startPoint.x);
        NSLog(@"theta: %f", theta*180/3.1415926535);
        NSString *response  = [NSString stringWithFormat:@"LINE,%f,%f,0.0,%f", 0.0, 0.0, -theta];
        [self sendMessage:response];
    }
}


@end
