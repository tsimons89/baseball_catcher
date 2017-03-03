#include "stdafx.h"
#include "time.h"
#include "math.h"
#include "Hardware.h"
#include "ball_tracker.h"


#define NUM_FIRST_FRAMES 19
#define NUM_SECOND_FRAMES 21
#define INITAL_WAIT_FRAMES 2
#define NUM_FLIGHT_IMAGES 33
#define Z_OFFSET 5
#define RESET_CATCHER_COUNT 50
#define PLOT_POINTS false
#define SHOW_TRACKING_IMAGES false
#define CLICK_FLIGHT_IMAGES false
#define X_POLY 1
#define Y_POLY -1

enum CatcherState{  RESET, FIRST, SECOND };
CatcherState catcher_state = FIRST;

ImagingResources	CTCSys::IR;

ball_tracker * tracker;
bool first_move_done;
bool second_move_done;
Point2d prediction(0,0);
Point2d prediction_1(0, 0);
Point2d prediction_2(0, 0);

int reset_count;



CTCSys::CTCSys()
{
	EventEndProcess = TRUE;
	EventEndMove = TRUE;
	IR.Acquisition = TRUE;
	IR.UpdateImage = TRUE;
	IR.CaptureSequence = FALSE;
	IR.DisplaySequence = FALSE;
	IR.PlayDelay = 30;
	IR.CaptureDelay = 30;
	IR.FrameID = 0;
	IR.CatchBall = FALSE;
	OPENF("c:\\Projects\\RunTest.txt");
	tracker = new ball_tracker("./source/left_calib.xml", "./source/right_calib.xml", "./source/stereo_calib.xml", "./source/rect.xml");
	tracker->set_catcher_z(Z_OFFSET);
	tracker->set_num_frames_to_track(NUM_FIRST_FRAMES);
	tracker->set_wait_after_motion(INITAL_WAIT_FRAMES);
	tracker->set_show_tracking_images_flag(SHOW_TRACKING_IMAGES);
	tracker->set_click_flight_images_flag(CLICK_FLIGHT_IMAGES);
	tracker->set_plot_points_flag(PLOT_POINTS);
	tracker->set_poly_order(X_POLY, Y_POLY);
	tracker->set_num_flight_frames(NUM_FLIGHT_IMAGES);
}

CTCSys::~CTCSys()
{
	CLOSEF;
}

void CTCSys::QSStartThread()
{
	EventEndProcess = FALSE;
	EventEndMove = FALSE;
	QSMoveEvent = CreateEvent(NULL, TRUE, FALSE, NULL);		// Create a manual-reset and initially nosignaled event handler to control move event
	ASSERT(QSMoveEvent != NULL);

	// Image Processing Thread
	QSProcessThreadHandle = CreateThread(NULL, 0L,
		(LPTHREAD_START_ROUTINE)QSProcessThreadFunc,
		this, NULL, (LPDWORD) &QSProcessThreadHandleID);
	ASSERT(QSProcessThreadHandle != NULL);
	SetThreadPriority(QSProcessThreadHandle, THREAD_PRIORITY_HIGHEST);

	QSMoveThreadHandle = CreateThread(NULL, 0L,
		(LPTHREAD_START_ROUTINE)QSMoveThreadFunc,
		this, NULL, (LPDWORD) &QSMoveThreadHandleID);
	ASSERT(QSMoveThreadHandle != NULL);
	SetThreadPriority(QSMoveThreadHandle, THREAD_PRIORITY_HIGHEST);
}

void CTCSys::QSStopThread()
{
	// Must close the move event first
	EventEndMove = TRUE;				// Set the falg to true first
	SetEvent(QSMoveEvent);				// must set event to complete the while loop so the flag can be checked
	do { 
		Sleep(100);
		// SetEvent(QSProcessEvent);
	} while(EventEndProcess == TRUE);
	CloseHandle(QSMoveThreadHandle);

	// need to make sure camera acquisiton has stopped
	EventEndProcess = TRUE;
	do { 
		Sleep(100);
		// SetEvent(QSProcessEvent);
	} while(EventEndProcess == TRUE);
	CloseHandle(QSProcessThreadHandle);
}

long QSMoveThreadFunc(CTCSys *QS)
{
	while (QS->EventEndMove == FALSE) {
		WaitForSingleObject(QS->QSMoveEvent, INFINITE);
		if (QS->EventEndMove == FALSE) QS->Move(QS->Move_X, QS->Move_Y);
		ResetEvent(QS->QSMoveEvent);
	}
	QS->EventEndMove = FALSE;
	return 0;
}


long QSProcessThreadFunc(CTCSys *QS)
{
	int     i;
	int     BufID = 0;
	char    str[32];
    long	FrameStamp;
    
    FrameStamp = 0;
	while (QS->EventEndProcess == FALSE) {
#ifdef PTGREY		// Image Acquisition
		if (QS->IR.Acquisition == TRUE) {
			for(i=0; i < QS->IR.NumCameras; i++) {
				QS->IR.PGRError = QS->IR.pgrCamera[i]->RetrieveBuffer(&QS->IR.PtGBuf[i]);
				// Get frame timestamp if exact frame time is needed.  Divide FrameStamp by 32768 to get frame time stamp in mSec
                QS->IR.metaData[i] = QS->IR.PtGBuf[i].GetMetadata();
				FrameStamp = QS->IR.metaData[i].embeddedTimeStamp;               
				if(QS->IR.PGRError == PGRERROR_OK){
					QS->QSSysConvertToOpenCV(&QS->IR.AcqBuf[i], QS->IR.PtGBuf[i]);		// copy image data pointer to OpenCV Mat structure
				}
			}
			for(i=0; i < QS->IR.NumCameras; i++) {
				if (QS->IR.CaptureSequence) {
#ifdef PTG_COLOR
					mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.SaveBuf[i][QS->IR.FrameID], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
					QS->IR.AcqBuf[i].copyTo(QS->IR.SaveBuf[i][QS->IR.FrameID]);
#endif
				} else {
#ifdef PTG_COLOR
					mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.ProcBuf[i][BufID], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
					QS->IR.AcqBuf[i].copyTo(QS->IR.ProcBuf[i]);	// Has to be copied out of acquisition buffer before processing
#endif
				}
			}
		}
#else
		Sleep (100);
#endif
		// Process Image ProcBuf
		if (QS->IR.CatchBall) {  	// Click on "Catch" button to toggle the CatchBall flag when done catching
			// Images are acquired into ProcBuf[0] for left and ProcBuf[1] for right camera
			// Need to create child image or small region of interest for processing to exclude background and speed up processing
			// Mat child = QS->IR.ProcBuf[i](Rect(x, y, width, height));
			Mat left_image = QS->IR.ProcBuf[0];
			Mat right_image = QS->IR.ProcBuf[1];
			tracker->feed_next_images(left_image, right_image);
			if (tracker->is_tracking_done()){
				prediction = tracker->get_prediction();

				switch (catcher_state)
				{
				case RESET:
					reset_count++;
					if (reset_count >= RESET_CATCHER_COUNT){
						tracker->reset();
						tracker->set_catcher_z(Z_OFFSET);
						tracker->set_num_frames_to_track(NUM_FIRST_FRAMES);
						tracker->set_wait_after_motion(INITAL_WAIT_FRAMES);
						QS->Move_X = 0;					// replace 0 with your x coordinate
						QS->Move_Y = 0;					// replace 0 with your y coordinate
						SetEvent(QS->QSMoveEvent);		// Signal the move event to move catcher. The event will be reset in the move thread.
						catcher_state = FIRST;
						reset_count = 0;
					}
					break;
				case FIRST:
					QS->Move_X = prediction.x;					// replace 0 with your x coordinate
					QS->Move_Y = prediction.y;					// replace 0 with your y coordinate
					SetEvent(QS->QSMoveEvent);		// Signal the move event to move catcher. The event will be reset in the move thread.
					catcher_state = SECOND;
					tracker->set_num_frames_to_track(NUM_SECOND_FRAMES);
					prediction_1 = prediction;
					break;
				case SECOND:
					QS->Move_X = prediction.x;					// replace 0 with your x coordinate
					QS->Move_Y = prediction.y;					// replace 0 with your y coordinate
					SetEvent(QS->QSMoveEvent);		// Signal the move event to move catcher. The event will be reset in the move thread.
					reset_count = 0;
					if (tracker->is_flight_done()){
						catcher_state = RESET;
						tracker->show_results();
					}
					prediction_2 = prediction;
					break;
				default:
					break;
				}
			}
//			for(i=0; i < QS->IR.NumCameras; i++) {
//#ifdef PTG_COLOR
//				cvtColor(QS->IR.ProcBuf[i][BufID], QS->IR.OutBuf1[i], CV_RGB2GRAY, 0);
//#else			// Example using Canny.  Input is ProcBuf.  Output is OutBuf1
//				//Canny(QS->IR.ProcBuf[i], QS->IR.OutBuf1[i], 70, 100);
//#endif			
//
//				// remove the Canny function above and add your ball detection and trajectory estimation code here
//				// calculate your estimated ball x, y location in inches and assigned them to moveX, and moveY below
//			}
			// This is how you move the catcher.  QS->moveX and QS->moveY (both in inches) must be calculated and set first.
		}
		else{
			reset_count = 0;
			tracker->reset();
			tracker->set_catcher_z(Z_OFFSET);
			tracker->set_num_frames_to_track(NUM_FIRST_FRAMES);
			tracker->set_wait_after_motion(INITAL_WAIT_FRAMES);
			QS->Move_X = 0;					// replace 0 with your x coordinate
			QS->Move_Y = 0;					// replace 0 with your y coordinate
			SetEvent(QS->QSMoveEvent);		// Signal the move event to move catcher. The event will be reset in the move thread.
		}
		// Display Image
		if (QS->IR.UpdateImage) {
			putText(QS->IR.ProcBuf[0], "pred1(" + to_string(prediction_1.x) + "," + to_string(prediction_1.y) + ")", Point(120, 20), FONT_HERSHEY_SIMPLEX, .5, Scalar(255, 0, 0));
			putText(QS->IR.ProcBuf[0], "pred2(" + to_string(prediction_2.x) + "," + to_string(prediction_2.y) + ")", Point(120, 35), FONT_HERSHEY_SIMPLEX, .5, Scalar(255, 0, 0));
			for (i=0; i<QS->IR.NumCameras; i++) {
				if (QS->IR.CaptureSequence || QS->IR.DisplaySequence) {
#ifdef PTG_COLOR
					QS->IR.SaveBuf[i][QS->IR.FrameID].copyTo(QS->IR.DispBuf[i]);
#else
					QS->IR.OutBuf[0] = QS->IR.OutBuf[1] = QS->IR.OutBuf[2] = QS->IR.SaveBuf[i][QS->IR.FrameID];
					merge(QS->IR.OutBuf, 3, QS->IR.DispBuf[i]);
#endif
					sprintf_s(str,"%d",QS->IR.FrameID);
					putText(QS->IR.DispBuf[0], str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 0), 2);
					if (QS->IR.PlayDelay) Sleep(QS->IR.PlayDelay);
				} else {
#ifdef PTG_COLOR
					QS->IR.ProcBuf[i][BufID].copyTo(QS->IR.DispBuf[i]);
#else
					// Display OutBuf1 when Catch Ball, otherwise display the input image
					QS->IR.OutBuf[0] = QS->IR.OutBuf[1] = QS->IR.OutBuf[2] = (QS->IR.CatchBall) ? QS->IR.ProcBuf[i] : QS->IR.ProcBuf[i];
					merge(QS->IR.OutBuf, 3, QS->IR.DispBuf[i]);
					line(QS->IR.DispBuf[i], Point(320, 0), Point(320, 480), Scalar(0, 255, 0), 1, 8, 0);
					line(QS->IR.DispBuf[i], Point(0, 240), Point(640, 240), Scalar(0, 255, 0), 1, 8, 0);
#endif
				}
				QS->QSSysDisplayImage();
			}
		}
		if (QS->IR.CaptureSequence || QS->IR.DisplaySequence) {
			QS->IR.FrameID++;
			if (QS->IR.FrameID == MAX_BUFFER) {				// Sequence if filled
				QS->IR.CaptureSequence = FALSE;
				QS->IR.DisplaySequence = FALSE;
			} else {
				QS->IR.FrameID %= MAX_BUFFER;
			}
		}
		BufID = 1 - BufID;
	} 
	QS->EventEndProcess = FALSE;
	return 0;
}

void CTCSys::QSSysInit()
{
	long i, j;
	IR.DigSizeX = 640;
	IR.DigSizeY = 480;
	initBitmapStruct(IR.DigSizeX, IR.DigSizeY);

	// Camera Initialization
#ifdef PTGREY
	IR.cameraConfig.asyncBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.isochBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.grabMode = DROP_FRAMES;			// take the last one, block grabbing, same as flycaptureLockLatest
	IR.cameraConfig.grabTimeout = TIMEOUT_INFINITE;	// wait indefinitely
	IR.cameraConfig.numBuffers = 4;					// really does not matter since DROP_FRAMES is set not to accumulate buffers
	char ErrorMsg[64];

	// How many cameras are on the bus?
	if(IR.busMgr.GetNumOfCameras((unsigned int *)&IR.NumCameras) != PGRERROR_OK){	// something didn't work correctly - print error message
        sprintf_s(ErrorMsg, "Connect Failure: %s", IR.PGRError.GetDescription());
        AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP );
	} else {
		IR.NumCameras = (IR.NumCameras > MAX_CAMERA) ? MAX_CAMERA : IR.NumCameras;
		for(i = 0; i < IR.NumCameras; i++) {		
			// Get PGRGuid
			if (IR.busMgr.GetCameraFromIndex(i, &IR.prgGuid[i]) != PGRERROR_OK) {    // change to 1-i is cameras are swapped after powered up
				sprintf_s(ErrorMsg, "PGRGuID Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			IR.pgrCamera[i] = new Camera;
			if (IR.pgrCamera[i]->Connect(&IR.prgGuid[i]) != PGRERROR_OK) { 
				sprintf_s(ErrorMsg, "PConnect Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set video mode and frame rate
			if (IR.pgrCamera[i]->SetVideoModeAndFrameRate(VIDEO_FORMAT, CAMERA_FPS) != PGRERROR_OK) { 
				sprintf_s(ErrorMsg, "Video Format Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set all camera configuration parameters
			if (IR.pgrCamera[i]->SetConfiguration(&IR.cameraConfig) != PGRERROR_OK) { 
				sprintf_s(ErrorMsg, "Set Configuration Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Sets the onePush option off, Turns the control on/off on, disables auto control.  These are applied to all properties.
			IR.cameraProperty.onePush = false;
			IR.cameraProperty.autoManualMode = false;
			IR.cameraProperty.absControl = true;
			IR.cameraProperty.onOff = true;
			// Set shutter sppeed
			IR.cameraProperty.type = SHUTTER;
			IR.cameraProperty.absValue = SHUTTER_SPEED;
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				sprintf_s(ErrorMsg, "Shutter Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set gamma value
			IR.cameraProperty.type = GAMMA;
			IR.cameraProperty.absValue = 1.0;
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				sprintf_s(ErrorMsg, "Gamma Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set sharpness value
			IR.cameraProperty.type = SHARPNESS;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.valueA = 2000;
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				sprintf_s(ErrorMsg, "Sharpness Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
#ifdef  PTG_COLOR
			// Set white balance (R and B values)
			IR.cameraProperty = WHITE_BALANCE;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = WHITE_BALANCE_R;
			IR.cameraProperty.valueB = WHITE_BALANCE_B;
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				ErrorMsg.Format("White Balance Failure: %s",IR.PGRError.GetDescription());
				AfxMessageBox( ErrorMsg, MB_ICONSTOP );
			}
#endif
			// Set gain values (350 here gives 12.32dB, varies linearly)
			IR.cameraProperty = GAIN;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = GAIN_VALUE_A;
			IR.cameraProperty.valueB = GAIN_VALUE_B;
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				sprintf_s(ErrorMsg, "Gain Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
			// Set trigger state
			IR.cameraTrigger.mode = 0;
			IR.cameraTrigger.onOff = TRIGGER_ON;
			IR.cameraTrigger.polarity = 0;
			IR.cameraTrigger.source = 0;
			IR.cameraTrigger.parameter = 0;
			if(IR.pgrCamera[i]->SetTriggerMode(&IR.cameraTrigger, false) != PGRERROR_OK){	
				sprintf_s(ErrorMsg, "Trigger Failure: %s", IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
            IR.embeddedInfo[i].frameCounter.onOff = true;
            IR.embeddedInfo[i].timestamp.onOff = true;
            IR.pgrCamera[i]->SetEmbeddedImageInfo(&IR.embeddedInfo[i]);
			// Start Capture Individually
			if (IR.pgrCamera[i]->StartCapture() != PGRERROR_OK) {
				sprintf_s(ErrorMsg, "Start Capture Camera %d Failure: %s", i, IR.PGRError.GetDescription());
				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP);
			}
		}
		// Start Sync Capture (only need to do it with one camera)
//		if (IR.pgrCamera[0]->StartSyncCapture(IR.NumCameras, (const Camera**)IR.pgrCamera, NULL, NULL) != PGRERROR_OK) {
//				sprintf_s(ErrorMsg, "Start Sync Capture Failure: %s", IR.PGRError.GetDescription());
//				AfxMessageBox(CA2W(ErrorMsg), MB_ICONSTOP );
//		}
	}

#else
	IR.NumCameras = MAX_CAMERA;
#endif
	Rect R = Rect(0, 0, 640, 480);
	// create openCV image
	for(i=0; i<IR.NumCameras; i++) {
#ifdef PTG_COLOR
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.ProcBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		for (j=0; j<MAX_BUFFER; j++) 
			IR.SaveBuf[i][j].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
#else
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.ProcBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		for (j=0; j<MAX_BUFFER; j++) 
			IR.SaveBuf[i][j].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
#endif
		IR.AcqPtr[i] = IR.AcqBuf[i].data;
		IR.DispROI[i] = IR.DispBuf[i](R); 
		IR.ProcROI[i] = IR.ProcBuf[i](R); 

		IR.OutBuf1[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutBuf2[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutROI1[i] = IR.OutBuf1[i](R); 
		IR.OutROI2[i] = IR.OutBuf2[i](R); 
		IR.DispBuf[i] = Scalar(0);
		IR.ProcBuf[i] = Scalar(0);
	}
	IR.from_to[0] = 0;
	IR.from_to[1] = 2;
	IR.from_to[2] = 1;
	IR.from_to[3] = 1;
	IR.from_to[4] = 2;
	IR.from_to[5] = 0;

	
	QSStartThread();
}

void CTCSys::QSSysFree()
{
	QSStopThread(); // Move to below PTGREY if on Windows Vista
#ifdef PTGREY
	for(int i=0; i<IR.NumCameras; i++) {
		if (IR.pgrCamera[i]) {
			IR.pgrCamera[i]->StopCapture();
			IR.pgrCamera[i]->Disconnect();
			delete IR.pgrCamera[i];
		}
	}
#endif
}

void CTCSys::initBitmapStruct(long iCols, long iRows)
{
	m_bitmapInfo.bmiHeader.biSize			= sizeof( BITMAPINFOHEADER );
	m_bitmapInfo.bmiHeader.biPlanes			= 1;
	m_bitmapInfo.bmiHeader.biCompression	= BI_RGB;
	m_bitmapInfo.bmiHeader.biXPelsPerMeter	= 120;
	m_bitmapInfo.bmiHeader.biYPelsPerMeter	= 120;
    m_bitmapInfo.bmiHeader.biClrUsed		= 0;
    m_bitmapInfo.bmiHeader.biClrImportant	= 0;
    m_bitmapInfo.bmiHeader.biWidth			= iCols;
    m_bitmapInfo.bmiHeader.biHeight			= -iRows;
    m_bitmapInfo.bmiHeader.biBitCount		= 24;
	m_bitmapInfo.bmiHeader.biSizeImage = 
      m_bitmapInfo.bmiHeader.biWidth * m_bitmapInfo.bmiHeader.biHeight * (m_bitmapInfo.bmiHeader.biBitCount / 8 );
}

void CTCSys::QSSysDisplayImage()
{
	for (int i = 0; i < 2; i++) {
		::SetDIBitsToDevice(
			ImageDC[i]->GetSafeHdc(), 1, 1,
			m_bitmapInfo.bmiHeader.biWidth,
			::abs(m_bitmapInfo.bmiHeader.biHeight),
			0, 0, 0,
			::abs(m_bitmapInfo.bmiHeader.biHeight),
			IR.DispBuf[i].data,
			&m_bitmapInfo, DIB_RGB_COLORS);
	}
}

#ifdef PTGREY
void CTCSys::QSSysConvertToOpenCV(Mat* openCV_image, Image PGR_image)
{
	openCV_image->data = PGR_image.GetData();	// Pointer to image data
	openCV_image->cols = PGR_image.GetCols();	// Image width in pixels
	openCV_image->rows = PGR_image.GetRows();	// Image height in pixels
	openCV_image->step = PGR_image.GetStride(); // Size of aligned image row in bytes
}
#endif