#include <cv.h> /* required to use OpenCV */
#include <highgui.h> /* required to use OpenCV's highgui */
#include <stdio.h>
 
IplImage* GetThresholdedImage(IplImage* imgHSV){       
       IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
       cvInRangeS(imgHSV, cvScalar(0,0,0), cvScalar(50,50,250), imgThresh);
       return imgThresh;
} 
 


int main(int argc, char *argv[]) {
 IplImage* img=0; /* pointer to an image */
 printf("Hello\n");
 img = cvLoadImage("tomato.jpeg", 1);
 IplImage* frame = GetThresholdedImage(img);
 if(img != 0) {
  cvNamedWindow("Display", CV_WINDOW_AUTOSIZE); // create a window
  cvShowImage("Display", frame); // show image in window
  cvWaitKey(0); // wait until user hits a key
  cvDestroyWindow("Display");
 }
 else
  printf("File not found\n");
return 0;
}
