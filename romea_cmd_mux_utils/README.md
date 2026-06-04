# romea_cmd_mux_utils

## 1) Overview

`romea_cmd_mux_utils` provides C++ helper classes used by command-producing nodes to connect to a `romea_cmd_mux` node.

It wraps the `romea_cmd_mux_msgs` service API and gives clients a small interface to:

* register a command input topic in the command multiplexer
* unregister the command input topic when it is no longer used
* report service errors as C++ exceptions

This package does not publish commands itself. The application node remains responsible for publishing messages on its own command topic. `romea_cmd_mux_utils` only tells `romea_cmd_mux` that this topic should be considered as a possible command source.

Typical users are teleoperation nodes, autonomous controllers or robot-specific bridges that must publish commands through `romea_cmd_mux`.

---

## 2) Provided classes

| Class | Role |
|:------|:-----|
| `CmdMuxSubscriptionClient` | calls `cmd_mux/subscribe` |
| `CmdMuxUnsubscriptionClient` | calls `cmd_mux/unsubscribe` |
| `CmdMuxInterface` | convenience wrapper that tracks registered topics and exposes `subscribe()` / `unsubscribe()` |

The clients wait up to five seconds for the corresponding service. If the service is unavailable, the request fails, or the mux rejects the request, the helper throws a `std::runtime_error`.

---

## 3) CmdMuxInterface

`CmdMuxInterface` is the class most applications should use.

```cpp
#include "romea_cmd_mux_utils/cmd_mux_interface.hpp"

auto cmd_mux = std::make_unique<romea::CmdMuxInterface>(node);

cmd_mux->subscribe(
  "controller/cmd_one_axle_steering",
  10,
  0.5);

// Publish commands on controller/cmd_one_axle_steering.

cmd_mux->unsubscribe("controller/cmd_one_axle_steering");
```

The `subscribe()` arguments are:

| Argument | Description |
|:---------|:------------|
| `topic` | command input topic published by the client node |
| `priority` | priority requested in the mux, from `0` to `255` |
| `timeout` | activity timeout in seconds |

Higher priority values have precedence over lower priority values in `romea_cmd_mux`.

`CmdMuxInterface` keeps the list of topics it successfully registered. Calling `unsubscribe()` on a topic that was not registered through this interface is ignored locally.

---

## 4) Service names and namespaces

The helper clients call:

| Service | Type |
|:--------|:-----|
| `cmd_mux/subscribe` | `romea_cmd_mux_msgs/srv/Subscribe` |
| `cmd_mux/unsubscribe` | `romea_cmd_mux_msgs/srv/Unsubscribe` |

These names are relative to the client node namespace. Therefore, if a teleoperation node runs in a mobile base namespace, it connects to the `cmd_mux` node in that same namespace.

For example, a client node in `/robot/base` calls:

```text
/robot/base/cmd_mux/subscribe
/robot/base/cmd_mux/unsubscribe
```

This is the convention used by mobile base teleoperation and hardware-related nodes that publish commands through a local command mux.

---

## 5) Error handling

The subscription helper throws when:

* the `cmd_mux/subscribe` service is not available
* the service call fails
* the topic is already registered
* another topic already uses the requested priority

The unsubscription helper throws when:

* the `cmd_mux/unsubscribe` service is not available
* the service call fails
* the mux rejects the unsubscription request

Applications should catch these exceptions when command mux connectivity is optional. For example, some hardware or teleoperation nodes can skip command mux registration when configured with a disabled priority value.
