#include <cv.h> /* required to use OpenCV */
#include <highgui.h> /* required to use OpenCV's highgui */
#include <stdio.h>
 
int main(int argc, char *argv[]) {
 IplImage* img=0; /* pointer to an image */
 printf("Hello\n");
 img = cvLoadImage("tomato.jpeg", 1);
 //if(argv[1] != 0)
 // img = cvLoadImage(argv[1], 1); // 1 for color
 //else
 // printf("File not found");
 if(img != 0) {
  cvNamedWindow("Display", CV_WINDOW_AUTOSIZE); // create a window
  cvShowImage("Display", img); // show image in window
  cvWaitKey(0); // wait until user hits a key
  cvDestroyWindow("Display");
 }
 else
  printf("File not found\n");
return 0;
}
