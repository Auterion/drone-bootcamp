#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <fstream>

class MyMode {
private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    // TODO #1: define your global variables here



public:
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Cargo Mode", {auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config){

        _system_state.subscribeLocalPosition();

        _mode.onActivate([this]() {
            std::cout << "Cargo Mode activated. Hello!!!" << std::endl;

            // TODO #2: Add your variables initializations here


        });

        _mode.onDeactivate([]() { std::cout << "Cargo Mode deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            // TODO #4: Implement your cargo waypoint algorithm here


            auterion::multicopter::LocalFrameGotoSetpoint setpoint;
            return setpoint;
        });
    }
};


int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "cargo_example");

    auto ret = rcutils_logging_set_logger_level(sdk.defaultRosHandle()->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    
    auterion::multicopter::GotoControlLimits goto_limits{};
    auterion::LocalPositionAssessorConfig position_assessor_config{};

    // TODO #3: Define your speed limits and position assessor thresholds here (see example_1_goto-mode for inspiration)



    MyMode my_mode(sdk, goto_limits, position_assessor_config);

    sdk.run();

    return 0;
}
