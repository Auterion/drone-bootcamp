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
   public:
    explicit MyMode(auterion::SDK& sdk)
        : _mode(sdk, "Circle Mode", {auterion::multicopter::LocalFrameDynamicsSetpoint::Config{}}),
          _system_state(sdk){
        _system_state.subscribeLocalPosition();


        _mode.onActivate([this]() {
            std::cout << "Circle Mode activated" << std::endl;
        });

        _mode.onDeactivate([]() { std::cout << "Circle Mode deactivated" << std::endl; });

        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            auterion::multicopter::LocalFrameDynamicsSetpoint setpoint;
            return setpoint;
        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "circle_example");

    MyMode my_mode(sdk);

    sdk.run();

    return 0;
}
