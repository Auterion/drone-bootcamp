#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <fstream>
#include <cmath> // Include for math functions

// Fly to three Goto setpoints in a loop
class MyMode {
   private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    Eigen::Vector3f _start_position_enu_m;
    bool _start_position_set;
    float pos; // Initialize pos correctly in constructor
    enum class State { GoingToStart = 0, GoingPos1} _state;

   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Circle 2b Mode", {auterion::multicopter::LocalFrameDynamicsSetpoint::Config{},
                                    auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config),
          pos(0) // Initialize position angle
    {

        _system_state.subscribeLocalPosition();

        _mode.onActivate([this]() {
            std::cout << "Circle 2b Mode activated" << std::endl;
            _state = State::GoingToStart;
            _start_position_set = false;
        });

        _mode.onDeactivate([]() { std::cout << "Circle 2b Mode deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            if (!_start_position_set) {
                _start_position_enu_m = _system_state.localPosition().last().position_enu_m;
                _start_position_set = true;
                std::cout << "Established start position." << std::endl;
            }

            switch (_state) {
                case State::GoingToStart: {
                    if (_local_position_assessor.isPositionWithinThreshold(_start_position_enu_m)) {
                        _state = State::GoingPos1;
                        std::cout << "Going to position 1..." << std::endl;
                    }

                    auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                    setpoint.withPosition(_start_position_enu_m);
                    return setpoint;
                }

                case State::GoingPos1: {
                    Eigen::Vector3f goal_position_enu_m{10 * cos(pos), 10 * sin(pos), 5.0F}; // Use standard floating-point values
                    float goal_heading_enu_rad = pos + M_PI;

                    if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m, 0.5F) &&
                        _local_position_assessor.isVelocityUnderThreshold(0.2F) &&
                        _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad)) {
                       
                        pos += 0.174533; // Increment position

                        if (pos <= 6.28319) {
                            _state = State::GoingPos1;
                         //} else {
                           // _state = State::GoingToStart;
                            std::cout << "Going to position 2..." << std::endl;
                        }
                    }

                    auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                    setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);
                    setpoint.withMaxHorizontalSpeed(2.F).withMaxVerticalSpeed(1.F).withMaxHeadingRate(0.5F);
                    return setpoint;
                }

                // case State::GoingPos2: {
                //     pos += 0.174533;
                //     Eigen::Vector3f goal_position_enu_m{-10 * cos(pos), 10 * sin(pos), 5.0F};
                //     float goal_heading_enu_rad = M_PI / 2.0F;

                //     if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m, 0.5F) &&
                //         _local_position_assessor.isVelocityUnderThreshold(0.2F) &&
                //         _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad)) {
                       
                //         if (pos <= 3.14159) {
                //             _state = State::GoingPos2;
                //         } else {
                //             _state = State::GoingPos3;
                //             std::cout << "Going to position 3..." << std::endl;
                //         }
                //     }

                //     auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                //     setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);
                //     return setpoint;
                // }

                // case State::GoingPos3: {
                //     pos += 0.174533;
                //     Eigen::Vector3f goal_position_enu_m{-10 * cos(pos), -10 * sin(pos), 5.0F};
                //     float goal_heading_enu_rad = M_PI / 2.0F;

                //     if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m, 0.5F) &&
                //         _local_position_assessor.isVelocityUnderThreshold(0.2F) &&
                //         _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad)) {
                       
                //         if (pos <= 4.71239) {
                //             _state = State::GoingPos3;
                //         } else {
                //             _state = State::GoingPos4;
                //             std::cout << "Going to position 4..." << std::endl;
                //         }
                //     }

                //     auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                //     setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);
                //     return setpoint;
                // }

                // case State::GoingPos4: {
                //     pos += 0.174533;
                //     Eigen::Vector3f goal_position_enu_m{-10 * cos(pos), -10 * sin(pos), 5.0F};
                //     float goal_heading_enu_rad = M_PI / 2.0F;

                //     if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m, 0.5F) &&
                //         _local_position_assessor.isVelocityUnderThreshold(0.2F) &&
                //         _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad)) {
                       
                //         if (pos <= 6.28319) {
                //             _state = State::GoingPos4;
                //         } else {
                //             _state = State::GoingPos4;
                //             std::cout << "Completing circle..." << std::endl;
                //         }
                //     }

                //     auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                //     setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);
                //     return setpoint;
                // }
            }
        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "circle_2b_example");

    auto ret = rcutils_logging_set_logger_level(sdk.defaultRosHandle()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    const float max_horizontal_speed_m_s{1.0F};
    const float max_vertical_speed_m_s{1.0F};
    const float max_heading_rate_rad_s{0.5F};
    auterion::multicopter::GotoControlLimits goto_limits{};
    goto_limits.withMaxHorizontalSpeed(max_horizontal_speed_m_s)
        .withMaxVerticalSpeed(max_vertical_speed_m_s)
        .withMaxHeadingRate(max_heading_rate_rad_s);

    const float position_error_threshold_m{0.5F};
    const float velocity_error_threshold_m_s{1.0F};
    const float heading_error_threshold_rad{5.0F * M_PI / 180.0F};
    auterion::LocalPositionAssessorConfig position_assessor_config{};
    position_assessor_config.withPositionErrorThreshold(position_error_threshold_m)
        .withVelocityThreshold(velocity_error_threshold_m_s)
        .withHeadingErrorThreshold(heading_error_threshold_rad);

    MyMode my_mode(sdk, goto_limits, position_assessor_config);

    sdk.run();

    return 0;
}