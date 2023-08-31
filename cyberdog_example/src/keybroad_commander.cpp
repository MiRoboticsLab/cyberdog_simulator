#include <iostream>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cyberdog_msg/msg/apply_force.hpp>
#include <lcm/lcm-cpp.hpp>
#include "cyberdog_example/gamepad_lcmt.hpp"

enum class ControlParameterValueKind : uint64_t {
  kDOUBLE = 1,
  kS64 = 2,
  kVEC_X_DOUBLE = 3,  // for template type derivation
  kMAT_X_DOUBLE = 4   // for template type derivation
};

class ExampleNode: public rclcpp::Node
    {
    public:
        ExampleNode(std::string node_name):Node(node_name){};
        ~ExampleNode(){};
    };

int main(int argc, char** argv){
    //initialize rclcpp
    rclcpp::init(argc, argv);
    auto example_node_=std::make_shared<ExampleNode>("kdcommand_node");
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr para_pub_;
    para_pub_=example_node_->create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter",10);
    
    auto param_message_ = cyberdog_msg::msg::YamlParam();

    sleep(1);

    //send control mode to Gamepad_controller mode which can enable control from simulator program
    param_message_.name = "use_rc";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(0);
    param_message_.is_user = 0;
    para_pub_->publish(param_message_);
    std::cout<<"switch to gamepad control model..."<<std::endl;

    //shut down rclcpp
    rclcpp::shutdown();

    //initial lcm
    lcm::LCM lcm_;
    if(!lcm_.good())
        return 1;

    gamepad_lcmt gamepad_lcm_type;
    char key;
    
    while(1){
        std::cout << "Type command: ";
        std::cin >> key;
        switch(key){
            case 'w':   //x direction speed
                if(gamepad_lcm_type.leftStickAnalog[1]<0.9)
                    gamepad_lcm_type.leftStickAnalog[1] += 0.1;
            break;
            case 's':  //x direction speed
                if(gamepad_lcm_type.leftStickAnalog[1]>-0.9)
                    gamepad_lcm_type.leftStickAnalog[1] += -0.1;
            break;
            case 'd': //y direction speed
                if(gamepad_lcm_type.leftStickAnalog[0]<0.9)
                    gamepad_lcm_type.leftStickAnalog[0] += 0.1;
            break;
            case 'a': //y direction speed
                if(gamepad_lcm_type.leftStickAnalog[0]<0.9)
                    gamepad_lcm_type.leftStickAnalog[0] += -0.1;
            break;
            case 'i': //pitch direction speed
                if(gamepad_lcm_type.rightStickAnalog[1]<0.9)
                    gamepad_lcm_type.rightStickAnalog[1] += 0.1;
            break;
            case 'k': //pitch direction speed
                if(gamepad_lcm_type.rightStickAnalog[1]>-0.9)
                    gamepad_lcm_type.rightStickAnalog[1] += -0.1;
            break;
            case 'j': //yaw direction speed
                if(gamepad_lcm_type.rightStickAnalog[0]<0.9)
                    gamepad_lcm_type.rightStickAnalog[0] += 0.1;
            break;
            case 'l': //yaw direction speed
                if(gamepad_lcm_type.rightStickAnalog[0]>-0.9)
                    gamepad_lcm_type.rightStickAnalog[0] += -0.1;
            break;
            case 'e': //QP stand
                gamepad_lcm_type.x = 1;
            break;
            case 'r': //locomotion
                gamepad_lcm_type.y = 1;
            break;
            case 't': //pure damper
                gamepad_lcm_type.a = 1;
            break;
            case 'y': //recoverystand
                gamepad_lcm_type.b = 1;
            break;
            case 'c': //clear speed of all direction
                gamepad_lcm_type.leftStickAnalog[0] = 0;
                gamepad_lcm_type.leftStickAnalog[1] = 0;
                gamepad_lcm_type.rightStickAnalog[0] = 0;
                gamepad_lcm_type.rightStickAnalog[1] = 0;
            break;
        }
        
        //publish lcm
        lcm_.publish("gamepad_lcmt",&gamepad_lcm_type);

        //clear button states
        gamepad_lcm_type.x = 0;
        gamepad_lcm_type.y = 0;
        gamepad_lcm_type.a = 0;
        gamepad_lcm_type.b = 0;
    }

    
}