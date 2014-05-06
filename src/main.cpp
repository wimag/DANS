#include "ardrone/ardrone.h"

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------

using namespace cv;

int main(int argc, char **argv)
{
    // AR.Drone class
    ARDrone ardrone;

    // Initialize
    if (!ardrone.open()) {
        printf("Failed to initialize.\n");
        return -1;
    }
	ardrone.setCamera(1);

    // Battery
    printf("Battery = %d%%\n", ardrone.getBatteryPercentage());

    // Instructions
    printf("***************************************\n");
    printf("*       CV Drone sample program       *\n");
    printf("*           - How to Play -           *\n");
    printf("***************************************\n");
    printf("*                                     *\n");
    printf("* - Controls -                        *\n");
    printf("*    'Space' -- Takeoff/Landing       *\n");
    printf("*    'W'     -- Move forward          *\n");
    printf("*    'S'     -- Move backward         *\n");
	printf("*    'A'     -- Move leftward         *\n");
	printf("*    'D'     -- Move rightward        *\n");
    printf("*    'Q'     -- Turn left             *\n");
    printf("*    'E'     -- Turn right            *\n");
    printf("*    'R'     -- Move upward           *\n");
    printf("*    'F'     -- Move downward         *\n");
	printf("*    'M'     -- Manual mode           *\n");
    printf("*                                     *\n");
    printf("* - Others -                          *\n");
    printf("*    'C'     -- Change camera         *\n");
    printf("*    'Esc'   -- Exit                  *\n");
    printf("*                                     *\n");
    printf("***************************************\n\n");

     // Thresholds
    int minH = 0, maxH = 30;
    int minS = 0, maxS = 70;
    int minV = 0, maxV = 30;

    // Create a window
	IplImage *path = cvCreateImage(cvSize(500, 500), IPL_DEPTH_8U, 3);
    cvSet(path, CV_RGB(255, 255, 255));
    cvNamedWindow("path");
    cvResizeWindow("path", 500, 500);
    cvNamedWindow("binalized");
	cvResizeWindow("binalized", 0, 0);

	// Different modes of flight
	bool manualMode = true,
		 searchObject = false,
		 regulator = false,
		 returnToInitial = false;

	// Global coordinates
	double globalx = 0,
		   globaly = 0;
	double actualvx, actualvy, actualvz;
	double movetox = 0,
		   movetoy = 0;
	const double C_PROPORT = 0.001, 
				 C_DIFF = 1, 
				 STEPWIDTH = 0.07,
				 C_SPIRAL = 1;
	double phi = 0;
    int64 lastTime = getTickCount();
	 
    while (1) {
        // Key input
        int key = cvWaitKey(33);
        if (key == 0x1b) break;

        // Update
        if (!ardrone.update()) break;

        // Get an image
        IplImage *image = ardrone.getImage();

        // Take off / Landing
        if (key == ' ') {
            if (ardrone.onGround()) ardrone.takeoff();
            else                    ardrone.landing();
        }

		// Switch between modes
		if (key == 'm') {
			if (manualMode) {
				searchObject = true;
				globalx = 0;
				globaly = 0;
				phi = 0;
				printf ("Searching object\n");
			} 
		}

		// HSV image
        IplImage *hsv = cvCloneImage(image);
        cvCvtColor(image, hsv, CV_RGB2HSV_FULL);

        // Binarized image
        IplImage *binalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

        // Binarize
        CvScalar lower = cvScalar(minH, minS, minV);
        CvScalar upper = cvScalar(maxH, maxS, maxV);
        cvInRangeS(image, lower, upper, binalized);

        // Show result
        cvShowImage("binalized", binalized);

        // De-noising
        cvMorphologyEx(binalized, binalized, NULL, NULL, CV_MOP_CLOSE);

        // Detect contours
        CvSeq *contour = NULL, *maxContour = NULL;
        CvMemStorage *contourStorage = cvCreateMemStorage();
        cvFindContours(binalized, contourStorage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// Initial velocities
        double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;

        // Find largest contour
        CvPoint minPoint, maxPoint, centerRect, centerWindow;
        centerWindow.x = image->width / 2;
        centerWindow.y = image->height / 2;
        cvCircle(image, centerWindow, 5, CV_RGB(0,0,255), 3);

        CvRect rect;
        double max_area = 0.0;
        while (contour) {
            rect = cvBoundingRect(contour);
            double area = fabs(cvContourArea(contour));
            if (area > max_area) {
                maxContour = contour;
                max_area = area;
            }
            contour = contour->h_next;
        }

        // Object detected
        if (maxContour) {
            rect = cvBoundingRect(maxContour);
            minPoint.x = rect.x;
            minPoint.y = rect.y;
            maxPoint.x = rect.x + rect.width;
            maxPoint.y = rect.y + rect.height;
            centerRect.x = rect.x + rect.width / 2;
            centerRect.y = rect.y + rect.height / 2;
            int diffx = centerWindow.x - centerRect.x;
            int diffy = centerWindow.y - centerRect.y;
            if (key == 'x') {
				printf("%d %d\n", diffx, diffy);
			}
            cvCircle(image, centerRect, 5, CV_RGB(255, 0, 0), 3);
            cvRectangle(image, minPoint, maxPoint, CV_RGB(0,255,0));
            double area = fabs(cvContourArea(maxContour));
            if (area > 3000 && searchObject) {
				searchObject = false;
				regulator = true;
				printf ("Found object, regulating");
			}
			if (regulator) {
				if (key == 'r')
					printf ("regulating\n");
				vx = C_PROPORT * diffy - C_DIFF * actualvx;
				if (abs(vx) > 1)
					vx = vx / abs(vx);
				vy = C_PROPORT * diffx - C_DIFF * actualvy;
				if (abs(vy) > 1)
					vy = vy / abs(vy);
			}
        }
		       
		if (searchObject) {
			movetox = STEPWIDTH * cos(phi) * phi,
			movetoy = STEPWIDTH * sin(phi) * phi;
			vx = C_SPIRAL * (movetox - globalx) - C_DIFF * actualvx;
			vy = C_SPIRAL * (movetoy - globaly) - C_DIFF * actualvy;
			if (abs(movetoy - globaly) < 0.2 && abs(movetox - globalx) < 0.2)
				phi += 0.1;
			if (key == 'p')
				printf ("%f %f\n", movetox, movetoy);
		}

		if (returnToInitial) {
			if (abs(globalx) < 0.1 && abs(globaly) < 0.1) {
				returnToInitial = false;
				ardrone.landing();
			}
			vx = - C_SPIRAL * globalx - C_DIFF * actualvx;
			vy = - C_SPIRAL * globaly - C_DIFF * actualvy;
		}

		
        // Move
		if (manualMode) {
			if (key == 'w') vx =  1.0;
			if (key == 's') vx = -1.0;
			if (key == 'a') vy =  1.0;
			if (key == 'd') vy = -1.0;
			if (key == 'q') vr =  1.0;
			if (key == 'e') vr = -1.0;
			if (key == 'r') vz =  1.0;
			if (key == 'f') vz = -1.0;
		}
		if (key == 'o')
			printf("%f %f\n", vx, vy);

		double dt = (getTickCount() - lastTime) / getTickFrequency();
        lastTime = getTickCount();

		double velocity = ardrone.getVelocity(&actualvx, &actualvy, &actualvz);
		globalx += actualvx * dt;
		globaly += actualvy * dt;

		if (abs(vx) > 0.5)
			vx = vx / (2 * abs(vx));
		if (abs(vy) > 0.5)
			vy = vy / (2 * abs(vy));
        ardrone.move3D(vx, vy, vz, vr);


        // Change camera
        static int mode = 1;

        //if (key == 'c') ardrone.setCamera(++mode%4);

        // Display the image
        cvShowImage("camera", image);
		cvCircle(path, cvPoint(path->height / 2 - globaly * 100.0, path->width / 2 - globalx * 100.0), 2, CV_RGB(0, 0, 255));
		cvCircle(path, cvPoint(path->height / 2 - movetoy * 100.0, path->width / 2 - movetox * 100.0), 2, CV_RGB(255, 0, 255));
        cvShowImage("path", path);

        // Release memory
        cvReleaseMemStorage(&contourStorage);
    }

    // See you
    ardrone.close();

    return 0;
}
