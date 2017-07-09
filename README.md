# legoswarm
Swarm robotics with LEGO: Multiple LEGO robot vehicles collaborating to harvest and sort LEGO balls spread out in a field.

# General Concept



- Decentralized Control: Each agent (Skid steering rover) has simple local rules for emerging "intelligent" behavior
  - Driving around while collecting any balls that are "accidentally" in its path
  - Control by virtual springs
    - Virtual springs with repulsive spring force between neighbors
    - Virtual springs with attractive spring force to valuable locations (see below)
    - The forces are based on information broadcast by other agents
    - The springs are attached to a virtual point ahead of the wheel axles
      - This way, an attractive or repulsive force "drags" the robot in the right direction, thus automatically steering it
      - Spring then controls both heading and position (almost) without singularities
  - Each robot is completely oblivious to what it is doing (it doesn't even detect the resource until actually finding it), yet is part of a useful network
- Agents communicate only with neighboring agents, broadcasting:
  - 1) Its approximate location
    - Repulsive forces prevent most collisions
  - 2) Its estimate of the local value/resources
    - Each time an agent finds a ball (by accident), it increases its localized value
    - The value slowly decays as time passes without finding any new balls
    - An agent in an resource rich area broadcasts a high value, thus attracting other agents through the virtual springs
  - Robots have an (artificially) limited line of sight.
    - Only close neighbors can communicate
    - Information can be re-broadcast to hop through each node (agent) in the network
- The swarm is robust
  - Continues operation when agents die or new agents are added.
    - Repulsive virtual forces ensure repositioning when the network changes
  - Adapts when the network is disturbed
    - When a robot is pushed away (e.g. visitor interaction, or a rogue robot controlled by the public)
    - When new resources are added the network dynamically approaches the new resources


# Implementation Aspects
- A way for each agent to know its approximate location and orientation
  - Heading:
    - Gyroscope
    - Compass
  - Positioning
    - Beacons "indoor GPS"
      - Simple least squares to merge redundant beacon data
    - Precise positioning is not necessary due to the simple move rules
    - Local QR code stickers on field
      - Could be a LEGO mosaic with unique patterns
      - So many 1x1 tiles....
- A way for communication between the agents, without central network
  - Bluetooth/IR?
  - Publicly broadcast location and resource values
- A simple robot that collects and stores LEGO balls. (And then build several)
  - Transport:
    - Skid/Tank steering
    - Omniwheel probably too expensive, and above steering is actually fun to implement
  - Collecting balls:
    - Somewhat like vacuum cleaner, collecting anything in its path without sensing
    - Color is detected upon collection
    - Stores the ball in order, into zamor shooter storage
    - That gives a FIFO array of colors
    - Can travel towards drop off point and then expel each one by one
    - Generation 2.0 could do sorting on board, for less visits to the dump site  
  - Returning to charge point would be nice
