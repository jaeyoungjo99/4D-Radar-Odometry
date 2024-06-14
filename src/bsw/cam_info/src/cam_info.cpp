#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

class ImageInfoPublisher {
public:
    ImageInfoPublisher() {

        if (!nh.getParam("/common_variable/camera_topic_name", s_camera_topic_name_)) {
            s_camera_topic_name_ = "";
        }  
    
        // 이미지 메시지를 구독하고 카메라 정보 메시지를 발행하는 publisher를 설정
        image_sub = nh.subscribe(s_camera_topic_name_, 10, &ImageInfoPublisher::imageCallback, this);
        info_pub = nh.advertise<sensor_msgs::CameraInfo>("/a2A1920/camera_info", 10);
        
    }

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        sensor_msgs::CameraInfo info;

        // CameraInfo 메시지 설정
        info.header = msg->header; // CompressedImage 메시지의 header를 사용
        info.height = 1200;
        info.width = 1920;
        info.distortion_model = "plumb_bob";
        info.D = {-0.272698, 0.080834, -0.001172, -0.00087, 0.0};
        // info.K = {1090.96952, 0.0, 971.030675, 0.0, 1091.96603, 573.895593, 0.0, 0.0, 1.0}; // FOV 100
        info.K = {1662.77, 0.0, 971.030675, 0.0, 1662.77, 573.895593, 0.0, 0.0, 1.0}; // FOV 60
        info.K = {2317, 0.0, 971.030675, 0.0, 2317, 573.895593, 0.0, 0.0, 1.0}; // FOV 45
        // info.K = {3582.77, 0.0, 971.030675, 0.0, 3582.77, 573.895593, 0.0, 0.0, 1.0}; // FOV 30
                        
        info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        info.P = {856.62915, 0.0, 969.627583, 0.0, 
                    0.0, 1013.993286, 578.051775, 0.0, 
                    0.0, 0.0, 1.0, 0.0}; // FOV 100
        info.P = {1662.77, 0.0, 969.627583, 0.0, 
                    0.0, 1662.77, 578.051775, 0.0, 
                    0.0, 0.0, 1.0, 0.0};   
        info.P = {2317, 0.0, 969.627583, 0.0, 
                    0.0, 2317, 578.051775, 0.0, 
                    0.0, 0.0, 1.0, 0.0}; // 45        
        // info.P = {3582.77, 0.0, 969.627583, 0.0, 
        //             0.0, 3582.77, 578.051775, 0.0, 
        //             0.0, 0.0, 1.0, 0.0};
        info.binning_x = 0;
        info.binning_y = 0;
        info.roi.x_offset = 0;
        info.roi.y_offset = 0;
        info.roi.height = 0;
        info.roi.width = 0;
        info.roi.do_rectify = false;

        // CameraInfo 메시지 발행
        info_pub.publish(info);

        std::cout<<"Publish Cam info"<<std::endl;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher info_pub;
    
    std::string s_camera_topic_name_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_info_publisher");
    ImageInfoPublisher image_info_publisher;
    ros::spin();
    return 0;
}