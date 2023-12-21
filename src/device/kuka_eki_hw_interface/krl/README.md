# Configuring EKI on the controller

This guide highlights the steps needed in order to successfully configure the **EKI interface** on the controller to
work with the **kuka_eki_hw_interface** on your PC with ROS.

## 1. Controller network configuration

Windows runs behind the SmartHMI on the teach pad. Make sure that the **Windows interface** of the controller and the *
*PC with ROS** is connected to the same subnet.

1. Log in as **Expert** or **Administrator** on the teach pad and navigate to **Network configuration** (**Connect-up >
   Network configuration > Activate advanced configuration**).
2. There should already be an interface checked out as the **Windows interface**. For example:
    * **IP**: `192.168.1.121`
    * **Subnet mask**: `255.255.255.0`
    * **Default gateway**: `192.168.1.121`
    * **Windows interface checkbox** should be checked.
3. Make note of the above IP address as you will need it later.
4. (Optional) Run **cmd.exe** and ping the PC you want to communicate with on the same subnet (e.g. 192.168.1.10). If
   your **PC** has an IP address on the same subnet as the **Windows interface** on the controller, the controller
   should receive answers from the PC.

## 2. KRL Files

The files included in the `kuka_eki_hw_interface/krl` folder provide the KRL interface and Ethernet packet
configurations. The XML files need to be modified to work for your specific configuration:

##### EkiHwInterface.xml

1. Edit the `IP` tag so that it corresponds to the IP address (`address.of.robot.controller`) corresponding to the *
   *Windows interface** of the controller (noted earlier).
2. Keep the `PORT` tag as it is (`54600`) or change it if you want to use another port (must be in the range of `54600`
   to `54615`).

Note that the `eki/robot_address` and `eki/robot_port` parameters of the `kuka_eki_hw_interface` must correspond to
the `IP`and `PORT` set in this XML file.

##### Copy files to controller

The files `kuka_eki_hw_interface.dat` and `kuke_eki_hw_interface.src` should not be edited. All files are now ready_ to
be copied to the Kuka controller. Using WorkVisual or a USB drive (with appropriate privleges):

1. Copy `kuka_eki_hw_interface.dat` and `kuka_eki_hw_interface.src` files to `KRC:\R1\Program`.
2. Copy `EkiHwInterface.xml` to `C:\KRC\ROBOTER\Config\User\Common\EthernetKRL\`.

##### Command Buffering Setup

The driver currently limits the number of queued joint targets to prevent unbounded trajectory lag (error) that can
occur when sufficiently dense target trajectories (temporal and/or spatial) are requested. The size of the buffer is set
via the parameter `eki/max_cmd_buf_len`. When not present, the buffer defaults to five (5), which is the maximum number
of instructions allowed in the controller's advance_ run (i.e., KRC internal interpolator lookahead).

With the buffering active, overly dense trajectory points are dropped with the result being small and bounded trajectory
lag at the expense of some positional deviation (dropped waypoints are not strictly enforced). For paths that require
exact positional accuracy buffering can be disabled by setting the limit to a sufficiently large number (e.g., 512) with
the caveat that dense trajectories might result in low-than-requested velocity and subsequent time lag. In other words,
the buffering limit can be seen as a tradeoff between positional accuracy and time accuracy.

## 4. Testing

At this point you are ready_ to test the EKI interface. Before the test, make sure that:

* You have specified the `eki/robot_address` and `eki/robot_port` of the **kuka_eki_hw_interface** to correspond with
  the KRL files on the controller.
* You have a launch-file loading the network parameters, robot description, kuka_hw_interface, hardware controller and
  controller joint names.

The next steps describe how to launch the test file:

* On the robot, Connect the servers

1. On the teach pad, enter mode **T1** for testing purposes.
2. Navigate to `KRC:\R1\Program` and select `kuka_eki_hw_interface.src`.
3. Press and hold an enabling switch and the run/play-button. The robot will release its breaks and a message like *
   *Programmed path reached (BCO)** will be shown at the teach pad.
4. Press and hold again. The robot is now waiting for clients to connect.

* Note that T1-mode limits the robot movement velocity and is intended for testing purposes.

