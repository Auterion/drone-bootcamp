#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <fstream>
#include <auterion_sdk/camera/camera.hpp>
#include <auterion_sdk/settings.hpp>

#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp> 

#include <sensor_msgs/msg/camera_info.hpp>

#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>


class MyMode {
   private:
    /// Don't change this!!!
    auterion::Mode _mode;
    auterion::SystemState _system_state;

    std::optional<auterion::Camera> _camera;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;


    float _ground_distance = 0.F;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _position_subscriber;
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition& msg) {
        _ground_distance = msg.dist_bottom;
    }


    // Start coding here 
    bool _sim = false;

    // Read parameters from the settings.default.env file (WebUI settings)
    // double _P_XY = auterion::getSetting("P_XY", 0.001); 


   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits)
        : _mode(sdk, "E.g. Precision Landing", {auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
        _system_state(sdk){


        /// Don't change this!!!
        _position_subscriber = sdk.defaultRosHandle()->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", 
            rclcpp::QoS(1).best_effort(), 
            std::bind(&MyMode::localPositionCallback, this, std::placeholders::_1)
        );

        _image_publisher = sdk.defaultRosHandle()->create_publisher<sensor_msgs::msg::Image>("/camera_image", 10);

        _system_state.subscribeLocalPosition();

        _camera = auterion::Camera::openFirst(sdk);

        if (_camera) {
            printf("Got camera : %s, model=%s\n", _camera->descriptor().unique_name.c_str(),
                _camera->descriptor().camera_model.c_str());

            // Image callback
            _camera->subscribeImage([this](const auterion::Image& image) {
                // Convert the image to OpenCV Mat format
                cv::Mat cv_mat = image.asOpenCVMat();
                cv::Mat bgr_img;
                if (!_sim){
                    cv::cvtColor(cv_mat, bgr_img, cv::COLOR_YUV2BGR_YUY2);
                    cv_mat = bgr_img.clone();
                }

                // Implement you detection algorithm here


                // DEBUGGING: Use this to publish an image to visualize processing
                // sensor_msgs::msg::Image::SharedPtr msg;
                // if (_sim){
                //     msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", viz_image).toImageMsg();
                // }
                // else{
                //     msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz_image).toImageMsg();
                // }
                // // Publish the image
                // _image_publisher->publish(*msg);

            });
        }

        _mode.onActivate([this]() {
            std::cout << "Precision Landing Activated!" << std::endl;
        });

        _mode.onDeactivate([]() { std::cout << "Precision Landing Deactivated!" << std::endl; });

        // runs at 50 Hz (dt_s = 20 ms)
        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {       
            // Dummy setpoint
            Eigen::Vector3f pos = {0.F , 0.F, 5.F};   
            return auterion::multicopter::LocalFrameGotoSetpoint{}.withPosition(pos);

            // Implement you control algorithm here

        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "precision_landing");

    auto ret = rcutils_logging_set_logger_level(sdk.defaultRosHandle()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    auterion::multicopter::GotoControlLimits goto_limits{};
    goto_limits.withMaxHorizontalSpeed(1.F).withMaxVerticalSpeed(1.F);

    MyMode my_mode(sdk, goto_limits);

    sdk.run();

    return 0;
}
