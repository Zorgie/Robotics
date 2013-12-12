#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class Color {
public:
	Color(int r, int g, int b) {
		c[0] = r;
		c[1] = g;
		c[2] = b;
	}
	double c[3];
};

class ImageConverter {
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	ImageConverter() :
			it_(nh_) {
		image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
				&ImageConverter::imageCb, this);

		cv::namedWindow(WINDOW);

	}

	~ImageConverter() {
		cv::destroyWindow(WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg) {

		Color tomato(120, 10, 0);
		Color lemon(200, 175, 0);

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		int kk = 430 * 640 + 350;
		int rr = cv_ptr->image.data[3 * kk + 2];
		int gg = cv_ptr->image.data[3 * kk + 1];
		int bb = cv_ptr->image.data[3 * kk];
		cv::circle(cv_ptr->image, cv::Point(350, 430), 10, CV_RGB(rr,gg,bb));
		printf("Circle color: %d, %d, %d\n", rr, gg, bb);

		double minDist = 3 * 255 * 255;
		int pointX, pointY;
		int rgbSaved[3];
		for (int y = 0; y < cv_ptr->image.rows; y++) {
			for (int x = 0; x < cv_ptr->image.cols; x++) {
				int k = x + y * cv_ptr->image.cols;
				char b = cv_ptr->image.data[3 * k];
				char g = cv_ptr->image.data[3 * k + 1];
				char r = cv_ptr->image.data[3 * k + 2];
				//double dist = colorDistance(b, g, r, 0, 10, 110);
				double tomatoDist = colorDistance(Color(r, g, b), tomato);
				double lemonDist = colorDistance(Color(r, g, b), lemon);
				if (tomatoDist < 20) {
					cv_ptr->image.data[3 * k] = 0;
					cv_ptr->image.data[3 * k + 1] = 0;
					cv_ptr->image.data[3 * k + 2] = 255;
				}
				if (lemonDist < 20) {
					cv_ptr->image.data[3 * k] = 0;
					cv_ptr->image.data[3 * k + 1] = 255;
					cv_ptr->image.data[3 * k + 2] = 255;
				}

			}
		}
		cv::imshow(WINDOW, cv_ptr->image);
		cv::waitKey(3);

		image_pub_.publish(cv_ptr->toImageMsg());
	}

	double colorDistance(Color col1, Color col2) {
		return sqrt(
				pow(col1.c[0] - col2.c[0], 2) + pow(col1.c[1] - col2.c[1], 2)
						+ pow(col1.c[2] - col2.c[2], 2));
	}

	double colorDistance(int b1, int g1, int r1, int b2, int g2, int r2) {
		double dist = sqrt(
				(r1 - r2) * (r1 - r2) + (g1 - g2) * (g1 - g2)
						+ (b1 - b2) * (b1 - b2));
		return dist;
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}

