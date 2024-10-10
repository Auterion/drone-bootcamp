#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <fstream>

// Fly to three Goto setpoints in a loop
class MyMode {
   private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    // TODO #1: define your global variables here



   public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Circle Mode", {
                                    auterion::multicopter::LocalFrameDynamicsSetpoint::Config{},
                                    auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config) {

        _system_state.subscribeLocalPosition();


        _mode.onActivate([this]() {
            std::cout << "Circle Mode activated" << std::endl;

            // TODO #2: Add your variables initializations here


        });

        _mode.onDeactivate([]() { std::cout << "Circle Mode deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            // TODO #4: Implement your circle movement algorithm here



            auterion::multicopter::LocalFrameDynamicsSetpoint setpoint;
            return setpoint;
        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "circle_example");

    auto ret = rcutils_logging_set_logger_level(sdk.defaultRosHandle()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    // TODO #3: Adapt your speed limits and position assessor thresholds here if needed

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
