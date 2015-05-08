/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads camera images using Apples QTKit Framework.
 *
 * @author Ulrich Eck <ueck@net-labs.de>
 *
 * implementation taken from opencv/modules/videoio/cap_qtkit.mm
 *
 */

#include <string>
#include <list>
#include <iostream>
#include <strstream>
#include <log4cpp/Category.hh>

#import <QTKit/QTKit.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/Image.h>
#include <utVision/Undistortion.h>
#include <opencv/cv.h>

#ifndef QTKIT_VERSION_7_6_3
#define QTKIT_VERSION_7_6_3         70603
#define QTKIT_VERSION_7_0           70000
#endif

#ifndef QTKIT_VERSION_MAX_ALLOWED
#define QTKIT_VERSION_MAX_ALLOWED QTKIT_VERSION_7_0
#endif

#define DISABLE_AUTO_RESTART 999

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Vision.QTKitCapture" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;





/*****************************************************************************
 *
 * UTCaptureDelegate Implementation.
 *
 * UTCaptureDelegate is notified on a separate thread by the OS whenever there
 *   is a new frame. When "updateImage" is called from the main thread, it
 *   copies this new frame into an IplImage, but only if this frame has not
 *   been copied before. When "getOutput" is called from the main thread,
 *   it gives the last copied IplImage.
 *
 *****************************************************************************/

namespace Ubitrack {
    namespace Drivers {
        class QTKitCapture;
    }
}


@interface UTCaptureDelegate : NSObject
{
    Ubitrack::Drivers::QTKitCapture* _owner;
    NSLock* _lock;
}

- (void)registerOwner:(Ubitrack::Drivers::QTKitCapture*)owner;

@end



namespace Ubitrack { namespace Drivers {

/**
 * @ingroup vision_components
 * Reads camera images using Apple QTKit
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Vision::ImageMeasurement.
 *
 * @par Configuration
 */
class QTKitCapture
	: public Dataflow::Component
{
public:

	/** constructor */
    QTKitCapture( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

	/** destructor, waits until thread stops */
	~QTKitCapture();

    /** starts the camera */
    void start();

    /** stops the camera */
    void stop();
    
    /** handler method for incoming pull requests */
	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getMatrix() );
		} else {
			UBITRACK_THROW( "No undistortion configured for QTKitCapture" );
		}
	}

    void receiveFrame (void*pixelBufferBase, size_t width, size_t height, size_t size, Ubitrack::Measurement::Timestamp timestamp);

protected:
	// qtkit stuff
    QTCaptureSession            *mCaptureSession;
    QTCaptureDeviceInput        *mCaptureDeviceInput;
    QTCaptureDecompressedVideoOutput    *mCaptureDecompressedVideoOutput;
    QTCaptureDevice             *mCaptureDevice;
    UTCaptureDelegate           *m_CaptureDelegate;

    unsigned char* m_imageBuffer;
    size_t         m_imageBufferSize;

    void initializeCamera();
    QTCaptureDevice* defaultCamDevice();
    QTCaptureDevice * camDevice(const char* uid);
    void configureOutput ();
    void destroySession();

    // camera UUID
    std::string m_cameraUUID;
    
	// the image width
	int m_width;

	// the image height
	int m_height;

	// trigger flash
	bool m_disable_autostart;

	// shift timestamps (ms)
	int m_timeOffset;

    // thread main loop
    void ThreadProc();

    // the thread
    boost::scoped_ptr< boost::thread > m_Thread;

    // stop the thread?
    volatile bool m_bStop;
    volatile bool m_bCaptureThreadReady;


	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;
};


QTKitCapture::QTKitCapture( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_cameraUUID( "" )
	, m_width( 0 )
	, m_height( 0 )
    , m_disable_autostart( false )
    , m_bStop( false )
    , m_bCaptureThreadReady( false )
	, m_timeOffset( 0 )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &QTKitCapture::getIntrinsic, this, _1 ) )
    , m_imageBufferSize(0)
    , mCaptureSession(NULL)
    , mCaptureDeviceInput(NULL)
    , mCaptureDecompressedVideoOutput(NULL)
    , mCaptureDevice(NULL)
    , m_CaptureDelegate(NULL)
    , m_imageBuffer(NULL)
{
    m_cameraUUID = subgraph->m_DataflowAttributes.getAttributeString( "cameraUUID" );
	subgraph->m_DataflowAttributes.getAttributeData( "width", m_width );
	subgraph->m_DataflowAttributes.getAttributeData( "height", m_height );

	if ( subgraph->m_DataflowAttributes.getAttributeString( "disableAutostart" ) == "true")
	{
        m_disable_autostart = true;
	}

	subgraph->m_DataflowAttributes.getAttributeData( "timeOffset", m_timeOffset );

	std::string intrinsicFile = subgraph->m_DataflowAttributes.getAttributeString( "intrinsicMatrixFile" );
	std::string distortionFile = subgraph->m_DataflowAttributes.getAttributeString( "distortionFile" );

	m_undistorter.reset(new Vision::Undistortion(intrinsicFile, distortionFile));
}

QTKitCapture::~QTKitCapture()
{
    destroySession();
}

void QTKitCapture::initializeCamera() {
    // initialize qtkit m_CaptureDelegate
    LOG4CPP_INFO(logger, "Initialize QTKit.");

    mCaptureSession = nil;
    mCaptureDeviceInput = nil;
    mCaptureDecompressedVideoOutput = nil;
    m_CaptureDelegate = nil;

    // need uid - somethimes parts of uuid change, need to strmatch the uuid
//    const char* uid = NULL;
//    if (m_cameraUUID != "") {
//        uid = m_cameraUUID.c_str();
//    }

    NSArray* devices = [[[QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeVideo]
                          arrayByAddingObjectsFromArray:[QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeMuxed]] retain];

    if ([devices count] == 0) {
        LOG4CPP_ERROR( logger, "QTKit didn't find any attached Video Input Devices!" );
        return;
    }

    QTCaptureDevice *device = NULL;
    NSUInteger nCameras = [devices count];
    NSString* uid = [NSString stringWithCString:m_cameraUUID.c_str()];
    for (int i = 0; i < nCameras; ++i) {
        device = [devices objectAtIndex:i];

        LOG4CPP_INFO(logger, "Found camera: "
                << ([[device localizedDisplayName] UTF8String])
                << " UUID "<< ([[device uniqueID] UTF8String]));

        if ([[device uniqueID] rangeOfString:uid].location != NSNotFound) {
            uid = [device uniqueID];
            break;
        }
    }

    [devices release];
    [device release];

    mCaptureSession = [[QTCaptureSession alloc] init];
    mCaptureDevice = camDevice([uid UTF8String]);
    if (! mCaptureDevice ) {
        [mCaptureSession release];
        mCaptureSession = nil;
        return;
    }
    [mCaptureDevice retain];

    LOG4CPP_INFO(logger, "Selected camera: "
            << ([[device localizedDisplayName] UTF8String])
            << " UUID "<< ([[device uniqueID] UTF8String]));


    int success;
    NSError* error;

    if (mCaptureDevice) {
        LOG4CPP_INFO(logger, "QTKit - Found device.");

        success = [mCaptureDevice open: &error];
        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to open a Video Capture Device");
            // destroySession();
            return;
        }

        mCaptureDeviceInput = [[QTCaptureDeviceInput alloc] initWithDevice: mCaptureDevice];
        if (! mCaptureDeviceInput ) {
            LOG4CPP_ERROR(logger, "QTKit input not received");
            // destroySession();
            return;
        }
        [mCaptureDeviceInput retain];


        success = [mCaptureSession addInput: mCaptureDeviceInput error: &error];
        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to start capture session with opened Capture Device");
            // destroySession();
            return;
        }

        m_CaptureDelegate = [[UTCaptureDelegate alloc] init];
        [m_CaptureDelegate registerOwner:this];
        mCaptureDecompressedVideoOutput = [[QTCaptureDecompressedVideoOutput alloc] init];

        configureOutput();

        [mCaptureDecompressedVideoOutput setDelegate: m_CaptureDelegate];

        success = [mCaptureSession addOutput: mCaptureDecompressedVideoOutput error: &error];
        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to add Output to Capture Session");
            return;
        }
        LOG4CPP_INFO(logger, "QTKit - device setup complete.");

    }}

QTCaptureDevice * QTKitCapture::defaultCamDevice()
{
    LOG4CPP_INFO(logger, "Requesting default device ");
    QTCaptureDevice * cam = [QTCaptureDevice defaultInputDeviceWithMediaType: QTMediaTypeVideo];
    if (! cam) {
        cam = [QTCaptureDevice defaultInputDeviceWithMediaType: QTMediaTypeMuxed];
    }
    return cam;
}

QTCaptureDevice * QTKitCapture::camDevice(const char* uid)
{
    if (! uid) {
        return defaultCamDevice();
    }
    LOG4CPP_INFO(logger, "Requesting uid " << uid);

    // then find the rest
    NSArray* devices = [QTCaptureDevice inputDevicesWithMediaType: QTMediaTypeVideo];
    NSEnumerator *enumerator = [devices objectEnumerator];

    QTCaptureDevice* value;
    while (value = ((QTCaptureDevice*) [enumerator nextObject])) {	// while not nil
        const char * cuid = [[ value uniqueID ] UTF8String ];
        if (cuid && ( strcmp( uid, cuid ) == 0)) {
            return value;
        }}

    devices = [QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeMuxed];
    enumerator = [devices objectEnumerator];
    while (value = ((QTCaptureDevice*) [enumerator nextObject])) {	// while not nil
        const char * cuid = [[ value uniqueID ] UTF8String ];
        if (cuid && ( strcmp( uid, cuid ) == 0)) {
            return value;
        }}
    LOG4CPP_ERROR( logger, "QTKit didn't find Video Input Device!" );
    return NULL;
}

void QTKitCapture::configureOutput ()
{

    NSDictionary *pixelBufferOptions;
    if (m_width > 0 && m_height > 0) {
        pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys: [NSNumber numberWithDouble: 1.0 * m_width], (id) kCVPixelBufferWidthKey,
        [NSNumber numberWithDouble: 1.0 * m_height], (id) kCVPixelBufferHeightKey,
        //[NSNumber numberWithUnsignedInt:k32BGRAPixelFormat], (id)kCVPixelBufferPixelFormatTypeKey,
        [NSNumber numberWithUnsignedInt: kCVPixelFormatType_32BGRA], (id) kCVPixelBufferPixelFormatTypeKey, nil];
    } else {
        pixelBufferOptions = [NSDictionary dictionaryWithObjectsAndKeys: [NSNumber numberWithUnsignedInt: kCVPixelFormatType_32BGRA], (id) kCVPixelBufferPixelFormatTypeKey, nil];
    }
    [mCaptureDecompressedVideoOutput setPixelBufferAttributes: pixelBufferOptions];

#if QTKIT_VERSION_MAX_ALLOWED >= QTKIT_VERSION_7_6_3
    [mCaptureDecompressedVideoOutput setAutomaticallyDropsLateVideoFrames: YES];
#endif


    // These require > 10.5, and a QTCaptureDecompressedVideoOutput
    @try {
//  no attribute for fps yet
//        [mCaptureDecompressedVideoOutput setMinimumVideoFrameInterval: (targetFrameInterval * 0.999) ];
        [mCaptureDecompressedVideoOutput setAutomaticallyDropsLateVideoFrames: YES];
    } @catch (id theException) {
        LOG4CPP_ERROR(logger, "No video-source rate control here.");
    }
}

void QTKitCapture::destroySession ()
{
    LOG4CPP_INFO( logger, "Camera session ends.");
    if (mCaptureSession) {
        if ([mCaptureSession isRunning]) [mCaptureSession stopRunning ];
        if (mCaptureDecompressedVideoOutput) [mCaptureSession removeOutput: mCaptureDecompressedVideoOutput ];
        if (mCaptureDeviceInput) [ mCaptureSession removeInput: mCaptureDeviceInput ];

        [mCaptureSession release];
        mCaptureSession = nil;
    }
    if (mCaptureDeviceInput) {
        [mCaptureDeviceInput release];
        mCaptureDeviceInput = nil;
    }
    if (mCaptureDecompressedVideoOutput) {
        [mCaptureDecompressedVideoOutput release];
        mCaptureDecompressedVideoOutput = nil;
    }
    if (m_CaptureDelegate) {
        [m_CaptureDelegate
        release];
        m_CaptureDelegate = nil;
    }

    if (mCaptureDevice) {
        LOG4CPP_INFO( logger, "Closing camera " << [[mCaptureDevice localizedDisplayName] UTF8String]);
        if ([mCaptureDevice isOpen]) [mCaptureDevice close];
        [mCaptureDevice release];
        mCaptureDevice = nil;
    }
}

void QTKitCapture::start()
{
    if ( !m_running ) {
        m_Thread.reset( new boost::thread( boost::bind( &QTKitCapture::ThreadProc, this ) ) );

        // run the main-loop until the capture thread is ready
        // then he'll maintain the loop until stop is requested
        double sleepTime = 0.005;

        // If the capture is launched in a separate thread, then
        // [NSRunLoop currentRunLoop] is not the same as in the main thread, and has no timer.
        //see https://developer.apple.com/library/mac/#documentation/Cocoa/Reference/Foundation/Classes/nsrunloop_Class/Reference/Reference.html
        // "If no input sources or timers are attached to the run loop, this
        // method exits immediately"
        // using usleep() is not a good alternative, because it may block the GUI.
        // Create a dummy timer so that runUntilDate does not exit immediately:
        [NSTimer scheduledTimerWithTimeInterval:100 target:nil selector:@selector(doFireTimer:) userInfo:nil repeats:YES];
        while (!m_bCaptureThreadReady) {
            [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:sleepTime]];
        }

    }
    Component::start();
}


void QTKitCapture::stop()
{
    if ( m_running ) {
        if ( m_Thread )
        {
            m_bStop = true;
            m_Thread->join();
        }
    }
}


void QTKitCapture::receiveFrame(void *pixelBufferBase, size_t width, size_t height, size_t size, Ubitrack::Measurement::Timestamp timestamp) {

    if (size != 0) {
        Vision::Image bufferImage( width, height, 4, pixelBufferBase, IPL_DEPTH_8U, 0 );
//        Measurement::Timestamp utTime = m_syncer.convertNativeToLocal( timestamp );

        boost::shared_ptr<Vision::Image> pColorImage = bufferImage.CvtColor(CV_BGRA2BGR, 3);
        pColorImage->channelSeq[0] = 'B';
        pColorImage->channelSeq[1] = 'G';
        pColorImage->channelSeq[2] = 'R';

        pColorImage = m_undistorter->undistort( pColorImage );

        if ( m_colorOutPort.isConnected() )
            m_colorOutPort.send( Measurement::ImageMeasurement( timestamp, pColorImage ) );
        if ( m_outPort.isConnected() )
            m_outPort.send( Measurement::ImageMeasurement( timestamp, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );
    }
}

void QTKitCapture::ThreadProc() {

    initializeCamera();

    if (mCaptureSession) {
        if (! [mCaptureSession isRunning]) {
            LOG4CPP_INFO(logger, "Start QTKit Capturing.");
            configureOutput();
            [ mCaptureSession startRunning ];
        }
    } else {
        LOG4CPP_ERROR( logger, "Run: no session to run.");
        return;
    }

    m_bCaptureThreadReady = true;

    double sleepTime = 0.005;

    // If the capture is launched in a separate thread, then
    // [NSRunLoop currentRunLoop] is not the same as in the main thread, and has no timer.
    //see https://developer.apple.com/library/mac/#documentation/Cocoa/Reference/Foundation/Classes/nsrunloop_Class/Reference/Reference.html
    // "If no input sources or timers are attached to the run loop, this
    // method exits immediately"
    // using usleep() is not a good alternative, because it may block the GUI.
    // Create a dummy timer so that runUntilDate does not exit immediately:
    [NSTimer scheduledTimerWithTimeInterval:100 target:nil selector:@selector(doFireTimer:) userInfo:nil repeats:YES];
    while (!m_bStop) {
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:sleepTime]];
    }

    LOG4CPP_INFO(logger, "Stop QTKit Capturing.");
    [mCaptureSession stopRunning];

}

} } // namespace Ubitrack::Driver


// implementation of delegate

@implementation UTCaptureDelegate

- (id)init {
    LOG4CPP_INFO(logger, "Initialize QTKit delegate.");
    self = [super init];
    if (self) {
        _owner = NULL;
    }
    return self;
}


-(void)dealloc {
    _owner = NULL;
    [super dealloc];
}

- (void)captureOutput:(QTCaptureOutput *)captureOutput
        didOutputVideoFrame:(CVImageBufferRef)videoFrame
        withSampleBuffer:(QTSampleBuffer *)sampleBuffer
        fromConnection:(QTCaptureConnection *)connection {

    LOG4CPP_TRACE(logger, "QTKit received image.");

    @synchronized (self) {
        // get timestamp early
        Ubitrack::Measurement::Timestamp timestamp = Ubitrack::Measurement::now();

        void* base;
        size_t size, width, height;
        if (CVPixelBufferLockBaseAddress(videoFrame, 0)) {
            LOG4CPP_ERROR(logger, "Cannot lock frame buffer.");
            return;
        }

        // Get info about the raw pixel-buffer data.
        base = (void*) CVPixelBufferGetBaseAddress(videoFrame);
        width = CVPixelBufferGetWidth(videoFrame);
        height = CVPixelBufferGetHeight(videoFrame);
        size = height * CVPixelBufferGetBytesPerRow(videoFrame);

        double nsinterval;
        QTGetTimeInterval ([sampleBuffer decodeTime], & nsinterval);

        if (_owner != NULL) {
            _owner->receiveFrame (base, width, height, size, timestamp);
        }

        // We're done with the pixel-buffer
        CVPixelBufferUnlockBaseAddress(videoFrame, 0);
    }

}

- (void)captureOutput:(QTCaptureOutput *)captureOutput
        didDropVideoFrameWithSampleBuffer:(QTSampleBuffer *)sampleBuffer
        fromConnection:(QTCaptureConnection *)connection {
//    (void)captureOutput;
//    (void)sampleBuffer;
//    (void)connection;
    LOG4CPP_ERROR( logger, "Camera dropped frame!" );
}


#pragma mark Public methods

- (void)registerOwner:(Ubitrack::Drivers::QTKitCapture*)owner {
    @synchronized (self) {
        _owner = owner;
    }
}


@end





UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::QTKitCapture > ( "QTKitCapture" );
}

