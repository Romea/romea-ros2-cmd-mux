# romea_cmd_mux_msgs

## 1) Overview

`romea_cmd_mux_msgs` provides the service definitions used to register and unregister command sources in `romea_cmd_mux`.

The package contains no runtime node. It defines the service API shared by `romea_cmd_mux`, client libraries such as `romea_cmd_mux_utils`, and command-producing nodes that want to connect to a command multiplexer.

`romea_cmd_mux` exposes these services as `~/subscribe` and `~/unsubscribe`. Client libraries usually call them through names such as `cmd_mux/subscribe` and `cmd_mux/unsubscribe` in their local namespace.

---

## 2) Services

| Service | Role |
|:--------|:-----|
| `Subscribe` | register a command input topic with a timeout and a priority |
| `Unsubscribe` | remove a previously registered command input topic |

---

## 3) Subscribe

Request:

| Field | Type | Description |
|:------|:-----|:------------|
| `topic` | `string` | command input topic published by the command source |
| `timeout` | `float64` | activity timeout in seconds |
| `priority` | `uint8` | command priority from `0` to `255` |

Response:

| Constant | Value | Meaning |
|:---------|:-----:|:--------|
| `ACCEPTED` | `0` | subscription accepted |
| `REJECTED_TOPIC_ALREADY_SUBSCRIBED` | `1` | topic already registered |
| `REJECTED_PRIORITY_ALREADY_USED` | `2` | priority already used |

---

## 4) Unsubscribe

Request:

| Field | Type | Description |
|:------|:-----|:------------|
| `topic` | `string` | input topic to remove |

Response:

| Constant | Value | Meaning |
|:---------|:-----:|:--------|
| `ACCEPTED` | `0` | topic removed |
| `REJECTED` | `1` | topic was not registered |
