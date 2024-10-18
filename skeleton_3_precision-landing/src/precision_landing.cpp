#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <fstream>
#include <auterion_sdk/camera/camera.hpp>

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
    float _width = 0.F;
    float _height = 0.F;

    float _dx = 0.F;
    float _dy = 0.F;

    int _counter = 0;
    int _timeout_counter = 0;
    int _invalid_counter = 0;
    bool _valid_detection = true;

    




   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits)
        : _mode(sdk, "Challenge 1", {auterion::multicopter::BodyFrameDynamicsSetpoint::Config{}, auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
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
                if(!(_sim)) {
                    cv::Mat init_image = cv_mat.clone();
                    cv::cvtColor(init_image, cv_mat, cv::COLOR_YUV2RGB_YUY2);   
                }

                // extract the image width & height
                _width = cv_mat.cols;
                _height = cv_mat.rows;

                // filter white pixels from the cv::Mat
                cv::Mat mask;
                cv::inRange(cv_mat, cv::Scalar(200, 200, 220), cv::Scalar(255, 255, 255), mask);

                // make sure the detected pixels form a connected square and declare contours first
                std::vector<std::vector<cv::Point>> contours;
                cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::drawContours(mask, contours, -1, cv::Scalar(0, 0, 255), 10);

                // Find the rotated rectangle of the minimum area enclosing the contours
                std::vector<cv::RotatedRect> min_rects(contours.size());
                for (size_t i = 0; i < contours.size(); i++) {
                    min_rects[i] = cv::minAreaRect(contours[i]);
                }

                // Calculate the angle of the rectangle with respect to the image edges
                std::vector<float> angles;
                for (const auto& rect : min_rects) {
                    float angle = rect.angle;
                    if (rect.size.width < rect.size.height) {
                        angle = 90 + angle;
                    }
                    angles.push_back(angle);
                }

                // Calculate the average angle
                float avg_angle = std::accumulate(angles.begin(), angles.end(), 0.0f) / angles.size();
                // std::cout << "Average angle: " << avg_angle << " degrees" << std::endl;

                // create a visualization image
                cv::Mat viz_image;
                cv_mat.copyTo(viz_image, mask);

                // extract the location of the center of the white pixels
                cv::Moments M = cv::moments(mask, true);
                cv::Point center(M.m10 / M.m00, M.m01 / M.m00);

                // find the distance between the center of the image and the center of the white pixels
                if((center.x >= 0) && (center.y >= 0) && (center.x <= _width) && (center.y <= _height)) {
                    _dx = center.x - _width / 2;
                    _dy = center.y - _height / 2;
                    _valid_detection = true;

                } else {
                    _dx = 0.F;
                    _dy = 0.F;
                    _valid_detection = false;
                }

                std::cout << "Center: " << center << std::endl;

                // float distance = sqrt(dx * dx + dy * dy);


                // reject any pixels that are not part of a meaningful object
                // if (M.m00 < 1000) {
                //     return;
                // }


                // add a red dot to the extracted center location on the viz_image
                cv::circle(viz_image, center, 5, cv::Scalar(0, 0, 255), -1);
                cv::drawContours(mask, contours, -1, cv::Scalar(0, 0, 255), 10);

                      
                // Implement your detection algorithm here


                // DEBUGGING: Use this to publish an image to visualize processing
                sensor_msgs::msg::Image::SharedPtr msg;
                if (_sim){
                    msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", viz_image).toImageMsg();
                }
                else{
                    msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", viz_image).toImageMsg();
                }
                // Publish the image
                _image_publisher->publish(*msg);

            });
        }

        _mode.onActivate([this]() {
            _counter = 0;
            _timeout_counter = 0;
            _invalid_counter = 0;
            std::cout << "Precision Landing Activated!" << std::endl;
        });

        _mode.onDeactivate([]() { std::cout << "Precision Landing Deactivated!" << std::endl; });

        // runs at 50 Hz (dt_s = 20 ms)
        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {                 

            // Implement your control algorithm here
            // write a simple proportional controller to move the drone to the center of the white pixels
            // Proportional control gains
            float Kp_x = 0.005F;
            float Kp_y = 0.005F;
            float Kp_z = 1.F;

            // Integral control gains
            float Ki_x = 0.0005F;
            float Ki_y = 0.0005F;
            float Ki_z = 0.0008F;

            // Derivative control gains

            float Kd_x = 0.F;
            float Kd_y = 0.F;
            float Kd_z = 0.01F;

            // Error accumulation for integral control
            static float integral_x = 0.F;
            static float integral_y = 0.F;
            static float integral_z = 0.F;

            // Previous errors for derivative control
            static float prev_error_x = 0.F;
            static float prev_error_y = 0.F;
            static float prev_error_z = 0.F;

            // Calculate errors
            float error_x = -_dx;
            float error_y = -_dy;
            

            // Update integral terms
            integral_x += error_x * dt_s;
            integral_y += error_y * dt_s;
            

            // Calculate derivative terms
            float derivative_x = (error_x - prev_error_x) / dt_s;
            float derivative_y = (error_y - prev_error_y) / dt_s;

            // Update previous errors
            prev_error_x = error_x;
            prev_error_y = error_y;

            // Calculate the control signals
            float control_x = Kp_x * error_x + Ki_x * integral_x + Kd_x * derivative_x;
            float control_y = Kp_y * error_y + Ki_y * integral_y + Kd_y * derivative_y;

            // std::cout << "dx: " << _dx << " dy: " << _dy << std::endl;
            // std::cout << "Ground Distance: " << _ground_distance << std::endl;
            // std::cout << "Width: " << _width << " Height: " << _height << std::endl; //Width: 1280 Height: 960
            // std::cout << "Valid Detection: " << _valid_detection << std::endl;

            if (!(_valid_detection)) {
                _invalid_counter++;
                if (_invalid_counter > 20 && _counter <= 150) {
                    _counter = 0;
                    _invalid_counter = 0;
                    RCLCPP_WARN(rclcpp::get_logger(""), "Re-starting the landing");
                    return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({control_y, control_x}).withVerticalVelocity(0.F);
                }
            }
            if (_counter <= 50) {
                float altitude_setpoint = 5.F;
                float error_z = _ground_distance - altitude_setpoint;
                integral_z += error_z * dt_s;
                float derivative_z = (error_z - prev_error_z) / dt_s;
                prev_error_z = error_z;
                float control_z = Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
                if (std::abs(_dx) <= 100 && std::abs(_dy) <= 100 && std::abs(error_z) <= 0.4 && _valid_detection) {
                    _counter++;
                    _timeout_counter = 0;
                } else {
                    if (_timeout_counter < 15) {
                        _timeout_counter++;
                        std::cout << "Timeout counter: " << _timeout_counter << std::endl;
                    } else {
                        _counter = 0;
                    } 
                }
                std::cout << "Going to start position with counter: " << _counter << std::endl;
                return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({control_y, control_x}).withVerticalVelocity(-control_z/2);
            } else if (_counter <=100) {
                float altitude_setpoint = 2.F;
                float error_z = _ground_distance - altitude_setpoint;
                integral_z += error_z * dt_s;
                float derivative_z = (error_z - prev_error_z) / dt_s;
                prev_error_z = error_z;
                float control_z = Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
                if (std::abs(_dx) <= 50 && std::abs(_dy) <= 50 && std::abs(error_z) <= 0.5 && _valid_detection) {
                    _counter++;
                    _timeout_counter = 0;
                } else {
                    if (_timeout_counter < 15) {
                        _timeout_counter++;
                        std::cout << "Timeout counter: " << _timeout_counter << std::endl;
                    } else {
                        _counter = 51;
                    }
                }
                std::cout << "Pre-landing with counter: " << _counter << std::endl;
                return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({control_y/2, control_x/2}).withVerticalVelocity(-control_z/2);
            } else if ( _counter <=150) {
                float altitude_setpoint = 0.8F;
                float error_z = _ground_distance - altitude_setpoint;
                integral_z += error_z * dt_s;
                float derivative_z = (error_z - prev_error_z) / dt_s;
                prev_error_z = error_z;
                float control_z = Kp_z * error_z + Ki_z * integral_z + Kd_z * derivative_z;
                if (std::abs(_dx) <= 30 && std::abs(_dy) <= 30 && std::abs(error_z) <= 0.3 && _valid_detection) {
                    _counter++;
                    _timeout_counter = 0;
                } else {
                    if (_timeout_counter < 15) {
                        _timeout_counter++;
                        std::cout << "Timeout counter: " << _timeout_counter << std::endl;
                    } else {
                        _counter = 101;
                    }
                }
                std::cout << "Landing with counter: " << _counter << std::endl;
                return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({control_y/4, control_x/4}).withVerticalVelocity(-control_z/4);
            } else if (_counter > 150) {
                std::cout << "Final touchdown" << std::endl;
                return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({0, 0}).withVerticalVelocity(-0.5F);
            } else {
                _counter = 0;
                RCLCPP_WARN(rclcpp::get_logger(""), "Re-starting the landing");
                return auterion::multicopter::BodyFrameDynamicsSetpoint{}.withHorizontalVelocity({control_y, control_x}).withVerticalVelocity(0.F);
            }

            // if (_system_state.localPosition().isLastValid()) {
            //     Eigen::Vector3f current_position = _system_state.localPosition().last().position_enu_m;
            //     // std::cout << "Local Position: " << local_position << std::endl;
            //     // Current position
            //     // New setpoint position
            //     // Eigen::Vector3f new_position = current_position + Eigen::Vector3f(control_y, control_x, control_z);

            //     // Return the new setpoint
            //     // return auterion::multicopter::LocalFrameGotoSetpoint{}.withPosition(new_position);
                
            // } else {
            //     Eigen::Vector3f pos = {0.F, 0.F, 5.F};   
            //     return auterion::multicopter::LocalFrameGotoSetpoint{}.withPosition(pos);
                
            // }

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
