# romea_cmd_mux

## 1) Overview

`romea_cmd_mux` provides a generic ROS2 command multiplexer.

It receives commands from several input topics and republishes only one command stream on its output topic. The selected input is the active input with the highest priority.

The node is message-type agnostic. It uses ROS2 generic subscriptions and publishers, so the message type is provided at runtime through the `topics_type` parameter.

Typical uses in the ROMEA stack include selecting which source is allowed to command a mobile base:

* joystick teleoperation
* autonomous navigation
* safety or recovery command source
* robot-specific command bridges

The input topics are not configured statically. Command sources register and unregister themselves through services.

---

## 2) Selection concept

Each command source registers an input topic in the mux with:

| Field | Description |
|:------|:------------|
| `topic` | command input topic published by the command source |
| `priority` | priority of the command source, from `0` to `255` |
| `timeout` | maximum delay, in seconds, after which the source is considered inactive |

Higher priority values have precedence over lower priority values.

When a command is received from an input topic, the mux republishes it only if no higher-priority source is currently active. A source is active while its last received message is newer than its configured timeout.

Two subscriptions cannot use the same topic name or the same priority.

---

## 3) Node interface

The package exports the `romea::CmdMux` component and the `cmd_mux_node` executable.

### Parameters

| Parameter | Type | Description |
|:----------|:-----|:------------|
| `topics_type` | string | message type multiplexed by the node, for example `geometry_msgs/msg/Twist` or `romea_mobile_base_msgs/msg/OneAxleSteeringCommand` |

The node cannot initialize correctly if `topics_type` is not provided.

### Services

Services are private to the mux node.

| Service | Type | Role |
|:--------|:-----|:-----|
| `~/subscribe` | `romea_cmd_mux_msgs/srv/Subscribe` | register a new input topic |
| `~/unsubscribe` | `romea_cmd_mux_msgs/srv/Unsubscribe` | unregister an input topic |

The `Subscribe` service can return:

| Result | Meaning |
|:-------|:--------|
| `ACCEPTED` | the input topic has been registered |
| `REJECTED_TOPIC_ALREADY_SUBSCRIBED` | another source already uses this topic |
| `REJECTED_PRIORITY_ALREADY_USED` | another source already uses this priority |

The `Unsubscribe` service returns `ACCEPTED` when the topic was registered and removed, otherwise `REJECTED`.

### Topics

| Topic | Direction | Type | Description |
|:------|:----------|:-----|:------------|
| `~/out` | published | `topics_type` | selected command stream |
| registered input topic | subscribed | `topics_type` | command stream registered through `~/subscribe` |
| `/diagnostics` | published | `diagnostic_msgs/msg/DiagnosticArray` | list of registered inputs and their active/inactive state |

The output topic is private to the node. For a node named `cmd_mux`, it is usually exposed as `cmd_mux/out`.

---

## 4) Example

Start a mux for a mobile base command type:

```bash
ros2 run romea_cmd_mux cmd_mux_node \
  --ros-args \
  -p topics_type:=romea_mobile_base_msgs/msg/OneAxleSteeringCommand
```

A command source can then register an input topic through the service API:

```bash
ros2 service call /cmd_mux/subscribe romea_cmd_mux_msgs/srv/Subscribe \
  "{topic: /teleop/cmd, timeout: 0.5, priority: 10}"
```

The selected command is published on:

```text
/cmd_mux/out
```

In ROMEA packages, command sources usually use `romea_cmd_mux_utils` instead of calling the services directly.
