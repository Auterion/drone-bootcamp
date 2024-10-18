

#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <fstream>
#include <Eigen/Dense>  // For Eigen::Vector2f/Vector3f
#include <cmath>        // For cos, sin

class MyMode {
   private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    // Circle parameters
    const float _radius_m = 10.0F;   // 10 meters radius
    float _angle_rad = 0.0F;         // Current angle in radians
    const float _angular_velocity_rad_s = 0.1F;  // Speed of rotation (radians per second)

    Eigen::Vector3f _center_position;  // Center of the circle (x, y, z coordinates)
    Eigen::Vector3f _takeoff_position{10.F, 0.F, 5.F};  // Takeoff position (altitude 5m)

    bool _start_position_set = false;
    bool _circle_completed = false;  // Flag to indicate if one circle is completed


    enum class State { TakingOff = 0, FlyingCircle } _state = State::TakingOff;

   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Circle Mode", {auterion::multicopter::LocalFrameDynamicsSetpoint::Config{},
                                     auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config) {

        _system_state.subscribeLocalPosition();

        _mode.onActivate([this]() {
            std::cout << "Circle Mode activated" << std::endl;

            // Initialize variables
            _angle_rad = 0.0F;  // Start at angle 0 (east of center)
            _center_position = _system_state.localPosition().last().position_enu_m;
            _state = State::TakingOff;
        });

        _mode.onDeactivate([]() { std::cout << "Circle Mode deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            auterion::multicopter::LocalFrameGotoSetpoint setpoint;

            switch (_state) {
                case State::TakingOff: {
                    if (!_start_position_set) {
                        _center_position = _system_state.localPosition().last().position_enu_m;
                        _start_position_set = true;
                        std::cout << "Start position set. Taking off..." << std::endl;
                    }

                    // Check if takeoff is complete
                    if (_local_position_assessor.isPositionWithinThreshold(_takeoff_position)) {
                        _state = State::FlyingCircle;
                        std::cout << "Takeoff complete. Now flying in a circle." << std::endl;
                    }

                    setpoint.withPosition(_takeoff_position).withHeading(0.0F);  // Takeoff setpoint
                    break;
                }

                case State::FlyingCircle: {
                    // Update angle based on time step and angular velocity
                    _angle_rad += _angular_velocity_rad_s * dt_s;

                    // Calculate new target position on the circle
                    float target_x = _center_position.x() + _radius_m * std::cos(_angle_rad);
                    float target_y = _center_position.y() + _radius_m * std::sin(_angle_rad);
                    float target_z = 5.F;  // Maintain altitude

                    Eigen::Vector3f target_position(target_x, target_y, target_z);
                    setpoint.withPosition(target_position).withHeading(_angle_rad + M_PI_2);  // Heading along the circle path

                    break;
                }
            }

            return setpoint;
        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "circle_example");

   
    // Define speed limits for Goto setpoints
    const float max_horizontal_speed_m_s{2.F};
    const float max_vertical_speed_m_s{1.F};
    const float max_heading_rate_rad_s{0.5F};
    auterion::multicopter::GotoControlLimits goto_limits{};
    goto_limits.withMaxHorizontalSpeed(max_horizontal_speed_m_s)
               .withMaxVerticalSpeed(max_vertical_speed_m_s)
               .withMaxHeadingRate(max_heading_rate_rad_s);

    // Define error thresholds for assessing position/velocity accuracy
    const float position_error_threshold_m{0.5F};
    const float velocity_error_threshold_m_s{1.0F};
    const float heading_error_threshold_rad{5.F * M_PI / 180.F};
    auterion::LocalPositionAssessorConfig position_assessor_config{};
    position_assessor_config.withPositionErrorThreshold(position_error_threshold_m)
                            .withVelocityThreshold(velocity_error_threshold_m_s)
                            .withHeadingErrorThreshold(heading_error_threshold_rad);

    MyMode my_mode(sdk, goto_limits, position_assessor_config);

    sdk.run();

    return 0;
}
