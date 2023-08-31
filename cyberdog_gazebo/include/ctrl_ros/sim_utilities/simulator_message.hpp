/*! @file simulator_message.hpp
 *  @brief Messages sent to/from the development simulator
 *
 *  These messsages contain all data that is exchanged between the robot program
 * and the simulator using shared memory.   This is basically everything except
 * for debugging logs, which are handled by LCM instead
 */

#ifndef PROJECT_SIMULATORTOROBOTMESSAGE_H
#define PROJECT_SIMULATORTOROBOTMESSAGE_H

#include "control_parameters/control_parameter_interface.hpp"
#include "sim_utilities/gamepad_command.hpp"
#include "sim_utilities/imu_types.hpp"
#include "sim_utilities/spine_board.hpp"
#include "sim_utilities/visualization_data.hpp"
#include "utilities/shared_memory.hpp"

/*!
 * The mode for the simulator
 */
enum class SimulatorMode {
  RUN_CONTROL_PARAMETERS,  // don't run the robot controller, just process
                           // Control Parameters
  RUN_CONTROLLER,          // run the robot controller
  DO_NOTHING,              // just to check connection
  EXIT                     // quit!
};

/*!
 * A plain message from the simulator to the robot
 */
struct SimulatorToRobotMessage {
  int value0[2];//value*: tmp for struct 8bytes alignment
  
  int value1;
  
  int value2;
  GamepadCommand gamepadCommand;// joystick
  RobotType robotType;// which robot the simulator thinks we are simulating
  
  
  // imu data
  VectorNavData vectorNav;
  CheaterState<double> cheaterState;

  // leg data
  SpiData spiData;
  // TODO: remove tiboard related later
  // TiBoardData tiBoardData[4];
  ControlParameterRequest controlParameterRequest;

  SimulatorMode mode;
  //int value2;
};

/*!
 * A plain message from the robot to the simulator
 */
struct RobotToSimulatorMessage {
  
  RobotType robotType;
  SpiCommand spiCommand;
  // TiBoardCommand tiBoardCommand[4];

  VisualizationData visualizationData;
  CheetahVisualization mainCheetahVisualization;
  ControlParameterResponse controlParameterResponse;

  char errorMessage[2056];
};

/*!
 * All the data shared between the robot and the simulator
 */
struct SimulatorMessage {
  RobotToSimulatorMessage robotToSim;
  SimulatorToRobotMessage simToRobot;
};

#endif  // PROJECT_SIMULATORTOROBOTMESSAGE_H
