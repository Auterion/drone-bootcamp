#include <algorithm>
#include <auterion_sdk/auterion.hpp>
#include <auterion_sdk/control/control.hpp>
#include <auterion_sdk/control/mode.hpp>
#include <auterion_sdk/system_state/system_state.hpp>
#include <auterion_sdk/system_state/local_position_assessor.hpp>
#include <Eigen/Dense>
#include <fstream>

// Class definition
class MyMode {
private:
    auterion::Mode _mode;
    auterion::SystemState _system_state;
    auterion::LocalPositionAssessor _local_position_assessor;

    Eigen::Vector3f _start_position_swu_m;
    bool _start_position_set;
    
    // Enum for drone states (moving between waypoints)
    enum class State { TakingOff = 0, GoingToStart, GoingPos1, GoingPos2, GoingPos3 , GoingPos4} _state;

public:
    // Constructor
    explicit MyMode(auterion::SDK& sdk, auterion::multicopter::GotoControlLimits& limits,
                    auterion::LocalPositionAssessorConfig& position_assessor_config)
        : _mode(sdk, "Survey Mode", {auterion::multicopter::LocalFrameGotoSetpoint::Config(limits)}),
          _system_state(sdk),
          _local_position_assessor(_system_state, position_assessor_config),
          _start_position_set(false) {

        // Subscribe to local position updates
        _system_state.subscribeLocalPosition();

        // On activate mode
        _mode.onActivate([this]() {
            std::cout << "Survey Mode activated. Let's start surveying!" << std::endl;
            _state = State::TakingOff;  // Begin with takeoff state
            _start_position_set = false;
        });

        // On deactivate mode
        _mode.onDeactivate([]() { std::cout << "Survey Mode deactivated" << std::endl; });

        // On update setpoint, modify the setpoint logic to survey the area
        _mode.onUpdateSetpoint([this](float dt_s) -> auterion::Setpoint {
            (void)dt_s;

            // Initialize the setpoint object
            auterion::multicopter::LocalFrameGotoSetpoint setpoint;

            // Define survey waypoints
            Eigen::Vector3f takeoff_position{0.F, 0.F, 5.F};  // Takeoff to 5 meters
            Eigen::Vector3f start_position{0.F, 0.F, 5.F};    // South-West corner
            Eigen::Vector3f pos1{0.F, 20.F, 5.F};             // North-West corner
            Eigen::Vector3f pos2{20.F, 20.F, 5.F};            // North-East corner
            Eigen::Vector3f pos3{20.F, 0.F, 5.F}; 
            Eigen::Vector3f pos4{20.F, 0.F, -1.F};             // South-East corner

            // Define the survey pattern (moving between waypoints)
            switch (_state) {
                case State::TakingOff: {
                    if (!_start_position_set) {
                        _start_position_swu_m = _system_state.localPosition().last().position_enu_m;
                        _start_position_set = true;
                        std::cout << "Start position set. Taking off..." << std::endl;
                    }

                    // Takeoff to the defined height
                    if (_local_position_assessor.isPositionWithinThreshold(takeoff_position)) {
                        _state = State::GoingToStart;
                        std::cout << "Takeoff complete. Moving to Start Position (South-West corner)" << std::endl;
                    }

                    setpoint.withPosition(takeoff_position).withHeading(0.0F);  // Takeoff vertically
                    break;
                }

                case State::GoingToStart: {
                    if (_local_position_assessor.isPositionWithinThreshold(start_position)) {
                        _state = State::GoingPos1;
                        std::cout << "Reached Start Position. Moving to Position 1 (North-West corner)" << std::endl;
                    }

                    setpoint.withPosition(start_position).withHeading(0.0F);  // Facing North
                    break;
                }

                case State::GoingPos1: {
                    if (_local_position_assessor.isPositionWithinThreshold(pos1)) {
                        _state = State::GoingPos2;
                        std::cout << "Reached Position 1. Moving to Position 2 (North-East corner)" << std::endl;
                    }

                    setpoint.withPosition(pos1).withHeading(0.0F);  // Still facing North
                    break;
                }

                case State::GoingPos2: {
                    if (_local_position_assessor.isPositionWithinThreshold(pos2)) {
                        _state = State::GoingPos3;
                        std::cout << "Reached Position 2. Moving to Position 3 (South-East corner)" << std::endl;
                    }

                    setpoint.withPosition(pos2).withHeading(M_PI / 2.F);  // Facing East
                    break;
                }

                case State::GoingPos3: {
                    if (_local_position_assessor.isPositionWithinThreshold(pos3)) {
                        _state = State::GoingPos4;
                        std::cout << "Reached Position 3. Returning to Start Position (South-West corner)" << std::endl;
                    }

                    setpoint.withPosition(pos3).withHeading(M_PI);  // Facing South
                    break;
                }

                case State::GoingPos4: {
                    if (_local_position_assessor.isPositionWithinThreshold(pos4)) {
                        _state = State::GoingToStart;
                        std::cout << "Reached Position 4. Returning to Start Position (South-West corner)" << std::endl;
                    }

                    setpoint.withPosition(pos4).withHeading(M_PI);  // Facing South
                    break;
                }
            }

            return setpoint;
        });
    }
};

// Main function
int main(int argc, char* argv[]) {
    // Initialize the SDK with "survey_example"
    auterion::SDK sdk(argc, argv, "survey_example");

    // Define the limits for drone movement (speeds)
    auterion::multicopter::GotoControlLimits goto_limits{};
    goto_limits.withMaxHorizontalSpeed(2.F)
               .withMaxVerticalSpeed(1.F)
               .withMaxHeadingRate(0.5F);

    // Define the position assessor configuration (thresholds for accuracy)
    auterion::LocalPositionAssessorConfig position_assessor_config{};
    position_assessor_config.withPositionErrorThreshold(0.5F)
                            .withVelocityThreshold(1.0F)
                            .withHeadingErrorThreshold(5.F * M_PI / 180.F);

    // Instantiate MyMode class
    MyMode my_mode(sdk, goto_limits, position_assessor_config);

    // Start the SDK run loop
    sdk.run();

    return 0;
}

