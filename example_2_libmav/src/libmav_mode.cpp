#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <fstream>
#include <mav/MessageSet.h>
#include <mav/Network.h>
#include <mav/TCPClient.h>

class MyMode {
   private:
    auterion::Mode _mode;
    
    float _time_acc = 0.F;
    int message_idx = 0;

    mav::MessageSet _message_set{"mavlink/common.xml"};

    std::shared_ptr<mav::TCPClient> _phy = std::make_shared<mav::TCPClient>("172.17.0.1", 5790);

    mav::Message own_heartbeat = _message_set.create("HEARTBEAT").set({
        {"type", _message_set.e("MAV_TYPE_LOG")},
        {"autopilot", _message_set.e("MAV_AUTOPILOT_INVALID")},
        {"base_mode", 0},
        {"custom_mode", 0},
        {"system_status", _message_set.e("MAV_STATE_ACTIVE")}
    });

    mav::Identifier _ident{1, 191};

    std::shared_ptr<mav::NetworkRuntime> _runtime = std::make_shared<mav::NetworkRuntime>(_ident, _message_set, own_heartbeat, *_phy);
    std::shared_ptr<mav::Connection> _connection = _runtime->awaitConnection(1000);

    float roundToTwoDecimalPlaces(float value) {
        return std::round(value * 100.0f) / 100.0f;
    };

   public:
    explicit MyMode(auterion::SDK& sdk)
        : _mode(sdk, "Libmav Example", {auterion::multicopter::LocalFrameGotoSetpoint::Config{}}){

        _mode.onActivate([this]() {
            std::cout << "Libmav Example Activated!" << std::endl;


            // Request SYS_AUTOSTART param from PX4
            auto expectation = _connection->expect("PARAM_VALUE");

            auto request = _message_set.create("PARAM_REQUEST_READ");
            request["param_index"] = -1;
            request["param_id"] = "SYS_AUTOSTART";
            request["target_system"] = 1;
            request["target_component"] = 1;
            _connection->send(request);

            auto response = _connection->receive(expectation, 1000);

            std::cout << "Received PARAM_VALUE: SYS_AUTOSTART" << std::endl;
            int param_value = response["param_value"].floatUnpack<int>();
            std::cout << "Param Value: " << param_value << std::endl;

        });

        _mode.onDeactivate([]() { std::cout << "Libmav Example Deactivated!" << std::endl; });

        // runs at 50 Hz
        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            if (_time_acc > 5.F){
                _time_acc = 0.F;

                // Receive the local position in NED using MAVLINK
                auto local_pos_response = _connection->receive("LOCAL_POSITION_NED");

                float x = local_pos_response["x"];
                float y = local_pos_response["y"];
                float z = local_pos_response["z"];

                std::string log_msg = "Pos is: x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", z=" + std::to_string(z);

                // Send a STATUSTEXT message that appears in the AMC console/UI
                // Severity level 1-3: yellow banner in AMC - CRITICAL
                // Severity level 4-5: yellow banner in AMC - WARNING
                // Severity level 6-8: no banner in AMC - INFO
                auto hello_message = _message_set.create("STATUSTEXT") ({
                        {"severity", 3},
                        {"text", "HELLO MAVLINK HERE!!!"},
                        {"id", 0}
                });

                auto pos_message = _message_set.create("STATUSTEXT") ({
                        {"severity", 6},
                        {"text", log_msg},
                        {"id", 0}
                });

                // Send the message
                _connection->send(hello_message);

                _connection->send(pos_message);
            }


            _time_acc += dt_s;


            // Dummy go-to setpoint
            auterion::multicopter::LocalFrameGotoSetpoint setpoint;

            setpoint.withPosition(Eigen::Vector3f(0.F, 0.F, 5.F));

            return setpoint;

        });
    }
};

int main(int argc, char* argv[]) {
    auterion::SDK sdk(argc, argv, "libmav_example");

    MyMode my_mode(sdk);

    sdk.run();

    return 0;
}
