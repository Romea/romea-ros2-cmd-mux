# romea_ros2_cmd_mux

## Overview

`romea_ros2_cmd_mux` groups the ROS2 packages used to multiplex command topics in the ROMEA ecosystem.

The command mux receives commands from several input topics and republishes only one command stream. The selected input is the active source with the highest priority. The mux is message-type agnostic: it uses ROS2 generic publishers and subscriptions, so it can multiplex mobile base commands, velocity commands or other command message types selected at runtime.

This repository-level README gives a map of the stack. Detailed behavior, node interfaces, service definitions and C++ helper APIs are documented in the README of each package listed below.

## Packages

| Package | Role |
| --- | --- |
| `romea_cmd_mux` | Runtime command multiplexer node and component. |
| `romea_cmd_mux_msgs` | Service definitions used to register and unregister command input topics. |
| `romea_cmd_mux_utils` | C++ helper classes used by command-producing nodes to connect to a command mux. |

## Usage

This stack is usually consumed by command-producing nodes such as teleoperation, autonomous navigation, safety controllers or robot-specific bridges.

In most cases, the runtime entry point is `romea_cmd_mux`: it starts the mux node, exposes `subscribe` and `unsubscribe` services, and publishes the selected command stream. Command-producing nodes usually use `romea_cmd_mux_utils` to register their output topic with the mux instead of calling the services directly.

Use the specialized package README files when you need to inspect or extend a specific part of the stack:

* `romea_cmd_mux` to configure and run the multiplexer node;
* `romea_cmd_mux_msgs` to inspect the subscribe and unsubscribe service API;
* `romea_cmd_mux_utils` to use the C++ client helpers from command-producing nodes.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `romea_ros2_cmd_mux` stack was developed by Jean Laneurit in the context of research projects carried out at INRAE.

## Contact

For questions or comments about this stack, contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
