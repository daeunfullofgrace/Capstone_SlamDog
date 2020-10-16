#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//Filtering
#include <math.h>

#define WIDTH 640
#define HEIGHT 480
#define DEPTH_MAX 10000
#define DEPTH_MIN 0
#define THRESHOLD 200
#define OPENCV_WINDOW1 "ORIGIN IMAGE"
#define OPENCV_WINDOW2 "RESULT"
#define OPENCV_WINDOW3 "BINARY IMAGE"

using namespace cv;

class VideoStreaming {
public:
    VideoStreaming()
        :_it(_nh) {
        sub_depth_info = _nh.subscribe("/camera/depth/image_rect_raw", 1, &VideoStreaming::stream, this);
        cv::namedWindow(OPENCV_WINDOW1);
        cv::namedWindow(OPENCV_WINDOW2);
    }

    ~VideoStreaming() {
        cv::destroyWindow(OPENCV_WINDOW1);
    }

    void stream(const sensor_msgs::Image::ConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat original_image, cropped_img, converted_image, binary_image;
        // cv::Rect bound(0, 0, WIDTH, HEIGHT);
        // cv::Rect new_size(WIDTH/2-RANGE, HEIGHT/2-RANGE, RANGE*2, RANGE*2);

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch(cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        
        original_image = cv_ptr -> image;
        binary_image = thresholding(original_image);
        converted_image = morphology_filtering(binary_image);

        cv::imshow(OPENCV_WINDOW1, convert_scale(original_image));
        cv::imshow(OPENCV_WINDOW2, converted_image);
        cv::imshow(OPENCV_WINDOW3, binary_image);
        
        cv::waitKey(30);
    }

    cv::Mat morphology_filtering(cv::Mat original_image) {
        cv::Mat converted_image;

        //image closing
        cv::dilate(original_image, converted_image, cv::Mat());
        cv::erode(converted_image, converted_image, cv::Mat());
        
        //image opening
        cv::erode(converted_image, converted_image, cv::Mat());
        cv::dilate(converted_image, converted_image, cv::Mat());

        // cv::dilate(converted_image, converted_image, cv::Mat());
        // cv::erode(converted_image, converted_image, cv::Mat());

        return converted_image;
    }

    cv::Mat convert_scale(cv::Mat original_image) {
        cv::Mat img_scaled_8u;

        cv::Mat(original_image-DEPTH_MIN).convertTo(img_scaled_8u, CV_8UC1, 255. / (DEPTH_MAX - DEPTH_MIN));
        // cv::cvtColor(img_scaled_8u, original_image, CV_GRAY2RGB);

        return img_scaled_8u;
    }

    cv::Mat thresholding(cv::Mat original_image) {
        cv::Mat binary_image, scaled_image;

        int range = 255 * THRESHOLD / DEPTH_MAX;
        scaled_image = convert_scale(original_image);
        
        cv::threshold(scaled_image, binary_image, range, 255, THRESH_BINARY);

        return binary_image;
    }

private:
    cv::Mat frame;

    ros::Subscriber sub_depth_info;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;

    double distance;
    double xr = 0.0;
    double yr = 0.0;
};


int main (int argc, char** argv){
    ros::init(argc, argv, "realsense_camera_test");
    ROS_INFO("Start Stream Depth Image");
    VideoStreaming video_streaming;
    ros::spin();

    return 0;
}