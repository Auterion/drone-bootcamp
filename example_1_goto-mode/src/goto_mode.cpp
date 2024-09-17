#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <fstream>

// Fly to three Goto setpoints in a loop
class MyMode {
   private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    Eigen::Vector3f _start_position_enu_m;
    bool _start_position_set;

    enum class State { GoingToStart = 0, GoingPos1, GoingPos2 } _state;

   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Goto Example", {auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config) {
        _system_state.subscribeLocalPosition();

        _mode.onActivate([this]() {
            std::cout << "Goto Example activated" << std::endl;
            _state = State::GoingToStart;
            _start_position_set = false;
        });

        _mode.onDeactivate([]() { std::cout << "Goto Example deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            if (!_start_position_set) {
                _start_position_enu_m = _system_state.localPosition().last().position_enu_m;
                _start_position_set = true;
                std::cout << "Established start position." << std::endl;
            }

            switch (_state) {
                case State::GoingToStart: {
                    // Check target position reached
                    // uses default position threshold
                    if (_local_position_assessor.isPositionWithinThreshold(_start_position_enu_m)) {
                        _state = State::GoingPos1;
                        std::cout << "Going to position 1..." << std::endl;
                    }

                    auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                    setpoint.withPosition(_start_position_enu_m);

                    return setpoint;
                }

                case State::GoingPos1: {
                    const Eigen::Vector3f goal_position_enu_m{15.F, 4.F, 10.F};
                    const float goal_heading_enu_rad = M_PI / 2.F;  // North

                    // Define override values for
                    // goto completion acceptance criteria
                    const float position_error_threshold_m = 0.5F;
                    const float velocity_error_threshold_m_s = 0.2F;

                    // Check target position, velocity, and heading reached
                    // overrides position error and velocity thresholds
                    // uses default heading threshold
                    if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m,
                                                                position_error_threshold_m) &&
                        _local_position_assessor.isVelocityUnderThreshold(velocity_error_threshold_m_s) &&
                        _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad)) {
                        _state = State::GoingPos2;
                        std::cout << "Going to position 2..." << std::endl;
                    }

                    auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                    setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);

                    // Override default goto speed limits
                    setpoint.withMaxHorizontalSpeed(10.F).withMaxVerticalSpeed(5.F).withMaxHeadingRate(1.5F);

                    return setpoint;
                }

                case State::GoingPos2: {
                    const Eigen::Vector3f goal_position_enu_m{-3.F, 14.F, 5.F};
                    const float goal_heading_enu_rad = -3.F * M_PI / 4.F;  // South-west

                    // Define override values for
                    // goto completion acceptance criteria
                    const float heading_error_threshold_rad = 2.F * M_PI / 180.F;

                    // Check target position, velocity, and heading reached
                    // overrides heading error threshold
                    // uses default velocity and position thresholds
                    if (_local_position_assessor.isPositionWithinThreshold(goal_position_enu_m) &&
                        _local_position_assessor.isVelocityUnderThreshold() &&
                        _local_position_assessor.isHeadingWithinThreshold(goal_heading_enu_rad,
                                                               heading_error_threshold_rad)) {
                        _state = State::GoingToStart;
                        std::cout << "Going to start position..." << std::endl;
                    }

                    auterion::multicopter::LocalFrameGotoSetpoint setpoint;
                    setpoint.withPosition(goal_position_enu_m).withHeading(goal_heading_enu_rad);

                    return setpoint;
                }
            }

            throw std::runtime_error("Invalid state");
        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "goto_example");

    // Define speed limits while reaching goto setpoints
    const float max_horizontal_speed_m_s{2.F};
    const float max_vertical_speed_m_s{1.F};
    const float max_heading_rate_rad_s{0.5F};
    auterion::multicopter::GotoControlLimits goto_limits{};
    goto_limits.withMaxHorizontalSpeed(max_horizontal_speed_m_s)
        .withMaxVerticalSpeed(max_vertical_speed_m_s)
        .withMaxHeadingRate(max_heading_rate_rad_s);

    // Define default error thresholds used to check completion of goto setpoints
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
