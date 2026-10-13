# XLeRobot ROS control adapter

`SimulatorSystem` implements the ROS 2 `hardware_interface::SystemInterface`
for XLeRobot's original MuJoCo simulation. The canonical robot description
owns all joint names, interfaces and limits. LeKiwi's upstream controller defaults
are overlaid by `xlerobot_sim.control_setup`; standard controllers own joint
states, omni base commands and arm/head trajectory actions.

One physics process owns MuJoCo. The adapter exchanges measured position/velocity
and bounded commands with it through a fixed 1,592-byte shared mmap packet.
Sequence counters guard snapshots, a state-age check detects a stopped physics
process, and the Python command watchdog stops base motion on stale commands.
Rendering is a separate read-only process to keep GPU work out of feedback.

The chassis floats freely; three driven omniwheels and 48 passive rollers
produce motion through ground contact. The roller topology comes from LeKiwi;
the wheel frames and full robot assembly come from XLeRobot.
`config/base.yaml` supplies the same simulation geometry to the
controller and physics adapter; these values are not physical calibration.
The upstream STS driver is built separately for future hardware configuration.
