/*
 *  This file is part of ROS2ISOBUS
 *
 *  Copyright 2026 Juha Backman / Natural Resources Institute Finland
 *
 *  ROS2ISOBUS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  ROS2ISOBUS is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with ROS2ISOBUS.
 *  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "TECUClass2ClientNode.hpp"
#include "TECUClass3ClientNode.hpp"

/*
 *
 * TECU client entry point (ISO 11783-7/9 Class 2 or legacy Class 3).
 *  - Select with CLI args: --class2 / --class3 (default Class2)
 *  - Node options can still override parameters (e.g., esp_name_hex, esp_sa)
 *
 */
int main(int argc, char *argv[])
{
    bool use_class3 = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--class3" || arg == "class3")
        {
            use_class3 = true;
        }
        else if (arg == "--class2" || arg == "class2")
        {
            use_class3 = false;
        }
    }

    rclcpp::init(argc, argv);

    auto node = use_class3
        ? std::static_pointer_cast<rclcpp::Node>(std::make_shared<ros2_isobus::TECUClass3ClientROS2>())
        : std::static_pointer_cast<rclcpp::Node>(std::make_shared<ros2_isobus::TECUClass2ClientROS2>());

    RCLCPP_INFO(node->get_logger(), "Starting TECU as Class%d", use_class3 ? 3 : 2);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

