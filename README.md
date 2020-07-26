ESP-IDF Based Lora Mesh Networking System
====================

The goal of this project is to create a system to enable creation of self healing self learning mesh networks that enable relativly high network capacity over a small area.

To do this requires a medium access control protocol, and a routing protocol.

The first step is the medium access control protocol. Many other lora projects rely on nodes not sending data very often to keep the collision probability low, however if we want a useful throughput for streaming realtime data, we are going to need a real MAC layer.

A common MAC protocol for ad-hoc networks of this type is Aloha, which is a good choice for a network with high mobility and a dynamic number of nodes. However, to implement Aloha, the transmitting node needs to be able to detect when a colision has occured. To my knowledge, this is not possible with the SX1276 or similar lora chips. (If anyone knows how please tell me!)

So instead I have written a TDM (time division muliplexing system), where there is a defined number of nodes in the network, and each node has a fixed time allocation.

TDM compared to Aloha:

Pros:
- Collisions are extremely unlikely and only occur in obscure network conditions
- Very good air time utilization
- Simple to implement without needing the transmitter to be able to detect a collision

Cons:
- Nodes can't join the network dynamicly, unless they have already been asigned a slot in the schedule
- If not all assigned nodes are active, airtime will be wasted on the dead nodes
- If an active node does not use all its airtime, the remaining airtime in the slot is wasted

How the TDM works:
- The TDM layer has a state machine which can be in LISTEN, TRANSMIT, or receive.
- The TDM is maintained by "sync packets" which contain the slot number and the time till the end of the slot (adjusted for airtime by the transmitter).
- When a node starts up, it is first in LISTEN state for TDM_LISTEN_MS to see if it picks up a sync packet. If it picks up a sync packet, it sets its internal timing based on the slot number and timing in that packet, and goes to RECEIVE. If it doesn't pick up a packet, it sets its own timing starting from slot 0 and goes to RECEIVE or TRANSMIT as appropriate.
- When a node is in RECEIVE and gets a sync packet, it calculates the error between its own timing and the timing in the packet. It will then shift its own timing by a percentage of the error. If the error is too large, we assume a network malfanction and the node goes back to LISTEN state to try and sync up properly with the network.

Using this system of sync packets the nodes tend to achieve quite good syncronization, with an error less than 50 microseconds.

Written in C++ for ESP-IDF V4.0

Working so far:
- Basic C++ Lora Driver tested on SX1276 written to allow sharing of SPI bus with other devices
- TDM Layer to manage airtime based on fixed slot timing and allocations
- Protocol/packet structure written to allow definition of multiple packet types
- Sync packet to share timing information between nodes, to keep the TDM in sync
