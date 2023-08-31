#include <iostream>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <cyberdog_msg/msg/yaml_param.hpp>
#include <cyberdog_msg/msg/apply_force.hpp>

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
    rclcpp::init(argc, argv);
    auto example_node_=std::make_shared<ExampleNode>("cyberdogmsg_node");
    rclcpp::Publisher<cyberdog_msg::msg::YamlParam>::SharedPtr para_pub_;
    rclcpp::Publisher<cyberdog_msg::msg::ApplyForce>::SharedPtr force_pub_;
    
    para_pub_=example_node_->create_publisher<cyberdog_msg::msg::YamlParam>("yaml_parameter",10);
    force_pub_=example_node_->create_publisher<cyberdog_msg::msg::ApplyForce>("apply_force",10);

    auto param_message_ = cyberdog_msg::msg::YamlParam();
    auto force_message_ = cyberdog_msg::msg::ApplyForce();

    sleep(1);

    //send control mode to Gamepad_controller mode which can enable control from simulator program
    param_message_.name = "use_rc";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(0);
    param_message_.is_user = 0;
    para_pub_->publish(param_message_);
    std::cout<<"switch to gamepad control model..."<<std::endl;

    sleep(1);

    //switch to recvorystand command control mode
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(12);
    param_message_.is_user = 0;

    para_pub_->publish(param_message_);
    std::cout<<"recovery stand ..."<<std::endl;

    sleep(5);

    //switch to locomotion control mode
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(11);
    param_message_.is_user = 0;
    para_pub_->publish(param_message_);
    std::cout<<"locomotion ..."<<std::endl;

    sleep(1);

    //apply force on FrontLeft leg
    std::array<double, 3> rel_pos_;
    std::array<double, 3> force_;
    force_[2] = 20;

    force_message_.link_name = "FL_knee";
    force_message_.rel_pos = rel_pos_;
    force_message_.force = force_;
    force_message_.time = 2;
    force_pub_->publish(force_message_);
    std::cout<<"applying force on FL_knee ..."<<std::endl;

    sleep(5);

    //change roll angle of body in locomtion
    std::array<double, 12> vecxd_value_;
    vecxd_value_[0] = 0.2;
    vecxd_value_[2] = 0.25;

    param_message_.name = "des_roll_pitch_height";
    param_message_.kind = uint64_t(ControlParameterValueKind::kVEC_X_DOUBLE);
    param_message_.vecxd_value = vecxd_value_;
    param_message_.is_user = 1;
    para_pub_->publish(param_message_);
    std::cout<<"set roll angle to 0.2 ..."<<std::endl;

    sleep(7);

    //switch to recvorystand command control mode
    param_message_.name = "control_mode";
    param_message_.kind = uint64_t(ControlParameterValueKind::kS64);
    param_message_.s64_value = int64_t(12);
    param_message_.is_user = 0;

    para_pub_->publish(param_message_);
    std::cout<<"recovery stand ..."<<std::endl;

    rclcpp::shutdown();
    return 0;
}