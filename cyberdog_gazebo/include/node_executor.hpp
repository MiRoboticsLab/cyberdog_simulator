// Copyright (c) 2023-2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef _NODE_EXCUTOR_HPP__
#define _NODE_EXCUTOR_HPP__

#include "rclcpp/rclcpp.hpp"

namespace gazebo
{
    class GazeboNode: public rclcpp::Node
    {
    public:
        /**
         * @brief Construct a new GazeboNode object to subscribe topic messages
         * 
         * @param node_name 
         */
        GazeboNode(std::string node_name):Node(node_name){};
        ~GazeboNode(){};
    };

    class NodeExc
    {
        public:
            /**
             * @brief Construct a new NodeExecutor object
             * 
             */
            NodeExc(){}

            /**
             * @brief Add node into node executor
             * 
             * @param node 
             */
            void AddNode(std::shared_ptr<GazeboNode> node)
            {
                executor_.add_node(node);
            }

            /**
             * @brief Receive topics
             * 
             */
            void ReceiveTopic()
            {
                using namespace std::chrono_literals;
                auto wait_time = 100ns;
                executor_.spin_once(wait_time);
            }


        private:
        rclcpp::executors::SingleThreadedExecutor executor_;
    };


}

#endif //_NODE_EXCUTOR_HPP__