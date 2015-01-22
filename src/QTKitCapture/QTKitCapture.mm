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
 * CaptureDelegate Implementation.
 *
 * CaptureDelegate is notified on a separate thread by the OS whenever there
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


@interface CaptureDelegate : NSObject
{
    int newFrame;
    CVImageBufferRef  mCurrentImageBuffer;
    char* imagedata;
    IplImage* image;
    size_t currSize;
    Ubitrack::Drivers::QTKitCapture* _owner;
    NSLock* _lock;
}

- (void)captureOutput:(QTCaptureOutput *)captureOutput
  didOutputVideoFrame:(CVImageBufferRef)videoFrame
     withSampleBuffer:(QTSampleBuffer *)sampleBuffer
       fromConnection:(QTCaptureConnection *)connection;

- (void)captureOutput:(QTCaptureOutput *)captureOutput
didDropVideoFrameWithSampleBuffer:(QTSampleBuffer *)sampleBuffer
       fromConnection:(QTCaptureConnection *)connection;

- (void)registerOwner:(Ubitrack::Drivers::QTKitCapture*)owner;

@end



@implementation CaptureDelegate

- (id)init {
    self = [super init];
    if (self) {
        imagedata = NULL;
        currSize = 0;
        image = NULL;
        _owner = NULL;
    }
    return self;
}


-(void)dealloc {
    if (imagedata != NULL) free(imagedata);
    cvReleaseImage(&image);
    [super dealloc];
}

- (void)captureOutput:(QTCaptureOutput *)captureOutput
  didOutputVideoFrame:(CVImageBufferRef)videoFrame
     withSampleBuffer:(QTSampleBuffer *)sampleBuffer
       fromConnection:(QTCaptureConnection *)connection {
    (void)captureOutput;
    (void)sampleBuffer;
    (void)connection;

    CVBufferRetain(videoFrame);
    CVImageBufferRef imageBufferToRelease  = mCurrentImageBuffer;

    
    // optimize START
    
    @synchronized (self) {
        mCurrentImageBuffer = videoFrame;
    }

    CVBufferRelease(imageBufferToRelease);

    // send measurement here
    CVPixelBufferRef pixels;

    @synchronized (self){
        pixels = CVBufferRetain(mCurrentImageBuffer);
    }
    
    // optimize Stop
    
    
    // get timestamp early
    Ubitrack::Measurement::Timestamp timeStamp = Ubitrack::Measurement::now();

    CVPixelBufferLockBaseAddress(pixels, 0);
    uint32_t* baseaddress = (uint32_t*)CVPixelBufferGetBaseAddress(pixels);

    size_t width = CVPixelBufferGetWidth(pixels);
    size_t height = CVPixelBufferGetHeight(pixels);
    size_t rowBytes = CVPixelBufferGetBytesPerRow(pixels);

    if (rowBytes != 0) {
        
        if (currSize != rowBytes*height*sizeof(char)) {
            currSize = rowBytes*height*sizeof(char);
            if (imagedata != NULL) free(imagedata);
            imagedata = (char*)malloc(currSize);
        }

        memcpy(imagedata, baseaddress, currSize);

        if (image == NULL) {
            image = cvCreateImageHeader(cvSize((int)width,(int)height), IPL_DEPTH_8U, 4);
        }
        image->width = (int)width;
        image->height = (int)height;
        image->nChannels = 4;
        image->depth = IPL_DEPTH_8U;
        image->widthStep = (int)rowBytes;
        image->imageData = imagedata;
        image->imageSize = (int)currSize;

        // here we need to send the image
        if (_owner != NULL) {
            _owner->sendImage(timestamp, image);
        }

    }

    CVPixelBufferUnlockBaseAddress(pixels, 0);
    CVBufferRelease(pixels);


}
- (void)captureOutput:(QTCaptureOutput *)captureOutput
  didDropVideoFrameWithSampleBuffer:(QTSampleBuffer *)sampleBuffer
       fromConnection:(QTCaptureConnection *)connection {
    (void)captureOutput;
    (void)sampleBuffer;
    (void)connection;
    LOG4CPP_ERROR( logger, "Camera dropped frame!" );
}


#pragma mark Public methods

- (void)registerOwner:(Ubitrack::Drivers::QTKitCapture*)owner {
    [_lock lock];
    _owner = owner;
    [_lock unlock];
}


@end



namespace {


} // anonymous namespace

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
    
    void getImage(Ubitrack::Measurement::Timestamp timeStamp, IplImage* bgr_image);

    /** handler method for incoming pull requests */
	Measurement::Matrix3x3 getIntrinsic( Measurement::Timestamp t )
	{
		if (m_undistorter) {
			return Measurement::Matrix3x3( t, m_undistorter->getIntrinsics() );
		} else {
			UBITRACK_THROW( "No undistortion configured for QTKitCapture" );
		}
	}

protected:
	// qtkit stuff
    QTCaptureSession            *mCaptureSession;
    QTCaptureDeviceInput        *mCaptureDeviceInput;
    QTCaptureDecompressedVideoOutput    *mCaptureDecompressedVideoOutput;
    CaptureDelegate* capture;

    // camera number
    int m_cameraBusIndex;
    
	// the image width
	int m_width;

	// the image height
	int m_height;


	// trigger flash
	bool m_disable_autostart;

	// shift timestamps (ms)
	int m_timeOffset;

	/** undistorter */
	boost::shared_ptr<Vision::Undistortion> m_undistorter;

	// the ports
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_colorOutPort;
	Dataflow::PullSupplier< Measurement::Matrix3x3 > m_intrinsicsPort;
};


QTKitCapture::QTKitCapture( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph )
	: Dataflow::Component( sName )
	, m_cameraBusIndex( -1 )
	, m_width( 0 )
	, m_height( 0 )
    , m_disable_autostart( false )
	, m_timeOffset( 0 )
	, m_outPort( "Output", *this )
	, m_colorOutPort( "ColorOutput", *this )
	, m_intrinsicsPort( "Intrinsics", *this, boost::bind( &QTKitCapture::getIntrinsic, this, _1 ) )
{
	subgraph->m_DataflowAttributes.getAttributeData( "cameraBusIndex", m_cameraBusIndex );
	if (m_cameraBusIndex == -1)
		UBITRACK_THROW( "Need to specify either cameraBusIndex" );

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

    // QTKit setup
    mCaptureSession = nil;
    mCaptureDeviceInput = nil;
    mCaptureDecompressedVideoOutput = nil;
    capture = nil;
    

    NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];

    capture = [[CaptureDelegate alloc] init];
    [capture registerOwner:this];

    QTCaptureDevice *device;
    NSArray* devices = [[[QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeVideo]
    arrayByAddingObjectsFromArray:[QTCaptureDevice inputDevicesWithMediaType:QTMediaTypeMuxed]] retain];

    if ([devices count] == 0) {
        LOG4CPP_ERROR( logger, "QTKit didn't find any attached Video Input Devices!" );
        [localpool drain];
        return;
    }

    if (m_cameraBusIndex >= 0) {
        NSUInteger nCameras = [devices count];
        if( (NSUInteger)m_cameraBusIndex >= nCameras ) {
            LOG4CPP_ERROR( logger, "QTKit invalid cameraBusIndex" );
            [localpool drain];
            return;
        }
        device = [devices objectAtIndex:m_cameraBusIndex] ;
    } else {
        device = [QTCaptureDevice defaultInputDeviceWithMediaType:QTMediaTypeVideo]  ;
    }
    int success;
    NSError* error;

    if (device) {

        success = [device open: &error];
        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to open a Video Capture Device");
            [localpool drain];
            return;
        }

        mCaptureDeviceInput = [[QTCaptureDeviceInput alloc] initWithDevice: device];
        mCaptureSession = [[QTCaptureSession alloc] init];

        success = [mCaptureSession addInput: mCaptureDeviceInput error: &error];

        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to start capture session with opened Capture Device");
            [localpool drain];
            return;
        }


        mCaptureDecompressedVideoOutput = [[QTCaptureDecompressedVideoOutput alloc] init];
        [mCaptureDecompressedVideoOutput setDelegate: capture];
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


        success = [mCaptureSession addOutput: mCaptureDecompressedVideoOutput error: &error];
        if (!success) {
            LOG4CPP_ERROR(logger, "QTKit failed to add Output to Capture Session");
            [localpool drain];
            return;
        }

        [localpool drain];
    }

    } else {
        LOG4CPP_ERROR( logger, "QTKit no device." );
        [localpool drain];
        return;
    }
}


}


QTKitCapture::~QTKitCapture()
{
    NSAutoreleasePool* localpool = [[NSAutoreleasePool alloc] init];

    if (m_isrunning) {
        [mCaptureSession stopRunning];
    }

    QTCaptureDevice *device = [mCaptureDeviceInput device];
    if ([device isOpen])  [device close];

    [mCaptureSession release];
    [mCaptureDeviceInput release];

    [mCaptureDecompressedVideoOutput setDelegate:mCaptureDecompressedVideoOutput];
    [mCaptureDecompressedVideoOutput release];
    [capture release];
    [localpool drain];
}

void QTKitCapture::start()
{
    if ( !m_running ) {
        NSAutoreleasePool * localpool = [[NSAutoreleasePool alloc] init];
        [mCaptureSession startRunning];
        [localpool drain];
    }
    Component::start();
}


void QTKitCapture::stop()
{
    if ( !m_running ) {
        NSAutoreleasePool * localpool = [[NSAutoreleasePool alloc] init];
        [mCaptureSession stopRunning];
        [localpool drain];
    }
}

void QTKitCapture::getImage(Ubitrack::Measurement::Timestamp timeStamp, IplImage* image) {
    
    boost::shared_ptr< Vision::Image > pColorImage;
    pColorImage.reset(new Vision::Image(image->width, image->height, 3 ) );

    cvCvtColor(image, *pColorImage, CV_BGRA2BGR);
    pColorImage->channelSeq[0] = 'B';
    pColorImage->channelSeq[1] = 'G';
    pColorImage->channelSeq[2] = 'R';
    
    pColorImage = m_undistorter->undistort( pColorImage );
    
    if ( m_colorOutPort.isConnected() )
        m_colorOutPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage ) );
    if ( m_outPort.isConnected() )
        m_outPort.send( Measurement::ImageMeasurement( timeStamp, pColorImage->CvtColor( CV_RGB2GRAY, 1 ) ) );

}

} } // namespace Ubitrack::Driver

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::QTKitCapture > ( "QTKitCapture" );
}

