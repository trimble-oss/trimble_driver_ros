*Table of Contents*

- [Trimble Driver ROS](#trimble-driver-ros)
- [ROS version compatibility](#ros-version-compatibility)
- [Product compatibility](#product-compatibility)
- [Usage](#usage)
- [Node Documentation](#node-documentation)
    - [LvxClientNode](#lvxclientnode)
        - [Parameters](#parameters-1)
        - [Topics](#topics-1)
            - [Standard Topics](#standard-topics-1)
            - [GSOF Topics](#gsof-topics)
        - [Services](#services-1)
- [Disclaimer](#disclaimer)

# Trimble Driver ROS

This package allows parsing of a subset of
Trimble's [General Serial Output Format](https://receiverhelp.trimble.com/oem-gnss/index.html#gsof-messages-overview.html) and converting the data to 
the Robot Operating System format.

# Product compatibility

The `trimble_driver_ros` packages supports Trimble GSOF. While the nodes and
libraries provided in the package may work with a variety of products they have only been tested with a subset of them,
namely:

* **Trimble Applanix**: Airborne (APX), Land (POS LVX), AP+ Air, AP+ Land products
* **Trimble OEM Products**: BDXXX[-INS] and BXXXX[-INS]

Other Trimble products supporting GSOF output may also be compatible.

# Usage
## Product configuration prerequisite
**Before you launch**, you must enable GSOF output on your product.

1. Access your device's web interface by entering its IP address into your web browser.
2. In the left hand side menu open the left hand side menu `IO Configuration > Port Summary`.
3. Click on one of the TCP/IP ports (e.g. 5017).
4. Select `GSOF` in the second drop down menu.
5. Set the frequency of the required GSOF messages.
    - For Inertial Navigation Systems, we typically at least enable
      - #49 INS Full Navigation Info
      - #50 INS RMS Info
    - For GNSS Receiver-only systems, we typically at least enable
      - #1  Position Time Info
      - #2  Lat Long Height
      - #12 Position Sigma Info
6. Click `ok` to save your settings

Back at the `IO Configuration > Port Summary` page, you should see your port with the word "GSOF" in the output column. Once you start the node, you should see the port's row turn green to indicate a connection is active.

## Building
Building can be done through the typical ROS workflow of using `rosdep` and `colcon`.

1. Clone this repository into your colcon workspace
   ```
   cd ~/colcon_ws/src
   git clone ...
   ```
   [Optional] ownload the test files to run the unit tests.
   ```
   git lfs pull
   ```
2. Install dependencies through `rosdep`
   ```
   apt update
   rosdep init
   rosdep update
   rosdep install --from-paths src -i -r -y
   ```
3. Build 
   ```
   colcon build --symlink-install --mixin release
   ```
4. [Optional] Run unit tests
   ```
   colcon test --event-handlers console_direct+
   ```

## Running
First edit the `gsof_client_params.yaml` file to fill in the IP address and GSOF port of your device.

### ros2 launch
Launch the gsof client node using
```
ros2 launch trimble_driver_ros gsof_client.py
```

### ros2 run
Alternatively you can run the node directly while specifying the parameters at the command line.
```
ros2 run trimble_driver_ros gsof_client_node --ros-args -p ip:=0.0.0.0 -p port:=5017
```

# Node Documentation

## LvxClientNode

ROS node to be used with Trimble products using the General Serial Output Format (GSOF), a type of packet part of the "
Data Collector" (DCOL) protocol.

### Parameters

| Parameter (All in private namespace ~/) | Type           | Description                                                                                                                                                                                                                                                                                                                                                                                                                   |
|-----------------------------------------|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `ip`                                    | `string`       | IP Address of the GSOF-speaking product to connect to.                                                                                                                                                                                                                                                                                                                                                                        |
| `port`                                  | `unsigned int` | TCP port of the GSOF-speaking product configured in the I/O configuration page of its web interface.                                                                                                                                                                                                                                                                                                                          |
| `parent_frame`                          | `string`       | When publishing a ROS TF, the node will use this parameter as the `parent_frame` for the published transform. The `parent_frame` will be initialized at the first Latitude-Longitude-Altitude data received by the node.                                                                                                                                                                                                      |
| `child_frame`                           | `string`       | When publishing a ROS TF, the node will use this parameter as the `child_frame` for the published transform.                                                                                                                                                                                                                                                                                                                  |
| `publish_tf`                            | `bool`         | Enable publishing of standard ROS TF.                                                                                                                                                                                                                                                                                                                                                                                         |
| `publish_gsof_msgs`                     | `bool`         | Enable publishing of the ROS version of GSOF messages. These messages have almost exactly the same format as those defined in the GSOF protocol.                                                                                                                                                                                                                                                                              |
| `publish_ros_msgs`                      | `bool`         | Enable publishing of "standard" ROS messages such as `nav_msgs::msg::Odometry` and `sensor_msgs::msg::NavSatFix`.                                                                                                                                                                                                                                                                                                             |
| `time_source`                           | `string`       | Valid values are `gps_time_of_week`, `now`, `gps`. When publishing a ROS message, the time stamp in `std_msgs::msg::Header` will be set according to this parameter. Valid values are &nbsp;<ul><li>`gps_time_of_week` : Time of the week according to the GPS clock.</li><li>`now` : System time sampled as the message was published.</li><li>`gps` : GPS clock time since the GPS epoch January 5th to 6th 1980.</li></ul> |

### Topics

#### Standard Topics

| **Topic (All in private namespace ~/)** | Type                             | Description                                                                                                                                                                                                                                                                                                                                            |
|-----------------------------------------|----------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `odom`                                  | `navigation_msgs::msg::Odometry` | INS pose in the local tangent plane centered either on the first Latitude-Longitude-Altitude message of GSOF49 or the LLA coordinates set through the `set_origin` service.                                                                                                                                                                            |
| `navsat`                                | `sensor_msgs::msg::NavSatFix`    | Satellite Fix information. If you are using a GNSS receiver with INS capabilities please use GSOF 49. If your receiver does not have INS capabilities please enable GSOF 1, 2 and 12. If using GSOF 49: will include position uncertainty (covariance) if GSOF50 has been received, otherwise the first element of the covariance vector will be `-1`. |

#### GSOF Topics

Note, to avoid polluting the topic list, the `GsofClientRos` implements lazy initialization of ROS publishers. The
following topics will only appear if the corresponding GSOF message is received at least once by the ROS node.

| **Topic (All in private namespace ~/)** | Type                                        | Description                                                                                                                                                                                                                                                                                   |
|-----------------------------------------|---------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `gsof/position_time_info_1`             | `gsof_msgs::msg::PositionTimeInfo1`         | GPS time and status bits for the positioning solution.                                                                                                                                                                                                                                        |
| `gsof/lat_long_height_2`                | `gsof_msgs::msg::LatLongHeight2`            | Latitude, Longitude, Height in WGS84 radians/meters.                                                                                                                                                                                                                                          |
| `gsof/ecef_position_3`                  | `gsof_msgs::msg::EcefPosition3`             | Earth-centered Earth-fixed (ECEF) WGS84 coordinate of the position in meters.                                                                                                                                                                                                                 |
| `gsof/ecef_delta_6`                     | `gsof_msgs::msg::EcefDelta6`                | ECEF delta between the rover and base positions (rover - base) in meters.                                                                                                                                                                                                                     |
| `gsof/tangent_plane_delta_7`            | `gsof_msgs::msg::TangentPlaneDelta7`        | Vector from the base to the rover projected onto a plane tangent to the WGS84 ellipsoid at the base in meters. The `up` component is the difference between the ellipsoidal height of the tangent plane at the base and a plane parallel to this passing through the rover point.             |
| `gsof/velocity_8`                       | `gsof_msgs::msg::Velocity8`                 | Velocity in meters per second where the heading is the WGS84 referenced true north heading in radians and the `local heading` is the local coordinate referenced coordinate system north heading in radians. It is only present in the message if a planar local coordinate system is loaded. |
| `gsof/pdop_info_9`                      | `gsof_msgs::msg::PdopInfo9`                 | Position dilution of precision.                                                                                                                                                                                                                                                               |
| `gsof/clock_info_10`                    | `gsof_msgs::msg::ClockInfo10`               | Clock offset flags, frequency offset flags and indicators.                                                                                                                                                                                                                                    |
| `gsof/position_vcv_info_11`             | `gsof_msgs::msg::PositionVcvInfo11`         | Variance-covariance matrix showing the uncertainty in the positioning solution.                                                                                                                                                                                                               |
| `gsof/position_sigma_info_12`           | `gsof_msgs::msg::PositionSigmaInfo12`       | 1-𝜎 of the position solution in meters. The orientation is of the semi-major axis is in degrees from clockwise from True North.                                                                                                                                                              |
| `gsof/receiver_serial_number_15`        | `gsof_msgs::msg::ReceiverSerialNumber15`    | Full serial number of the receiver.                                                                                                                                                                                                                                                           |
| `gsof/current_time_16`                  | `gsof_msgs::msg::CurrentTime16`             | GPS time and UTC offset.                                                                                                                                                                                                                                                                      |
| `gsof/attitude_info_27`                 | `gsof_msgs::msg::AttitudeInfo27`            | Attitude in radians and for firmware versions above GNSS v4.20 expected variance of the error on the estimate. If your product was purchased or updated after 2010, in all likelihood this information will be present.                                                                       |
| `gsof/all_sv_brief_info_33`             | `gsof_msgs::msg::AllSvBriefInfo33`          | Brief information about all the space vehicles (a.k.a. satellites) being tracked. e.g. PRN number and constellation                                                                                                                                                                           |
| `gsof/all_sv_detailed_info_34`          | `gsof_msgs::msg::AllSvDetailedInfo34`       | Detailed version of GSOF33 which also includes frequency bands and SNR indicators.                                                                                                                                                                                                            |
| `gsof/received_base_info_35`            | `gsof_msgs::msg::ReceivedBaseInfo35`        | Base station ID and latitude-longitude-height coordinates.                                                                                                                                                                                                                                    |
| `gsof/battery_memory_info_37`           | `gsof_msgs::msg::BatteryMemoryInfo37`       | Battery capacity and remaining logging time.                                                                                                                                                                                                                                                  |
| `gsof/position_type_information_38`     | `gsof_msgs::msg::PositionTypeInformation38` | Bit flags indicating RTK condition, network conditions with respect to the base station, RTCM conditions, etc.                                                                                                                                                                                |
| `gsof/lband_status_info_40`             | `gsof_msgs::msg::LbandStatusInfo40`         | Information about tracked satellite LBAND and the engine being used to track it.                                                                                                                                                                                                              |
| `gsof/base_position_and_quality_41`     | `gsof_msgs::msg::BasePositionAndQuality41`  | WGS84 coordinates in radians-radians-meters of the base station.                                                                                                                                                                                                                              |
| `gsof/ins_solution_49`                  | `gsof_msgs::msg::InsSolution49`             | Latitude-longitude-altitude (degrees, degrees, meters), velocity (meters per second), attitude (degrees), track angle (degrees), angular rates (degrees per second), body acceleration (meters per seconds squared).                                                                          |
| `gsof/ins_solution_rms_50`              | `gsof_msgs::msg::InsSolutionRms50`          | RMS error of position (meters), velocity (meters per second) and attitude (degrees).                                                                                                                                                                                                          |
| `gsof/ins_vnav_full_nav_info_63`        | `gsof_msgs::msg::InsVnavFullNavInfo63`      | Same as GSOF 49 but with heave (meters) for marine vehicles.                                                                                                                                                                                                                                  |
| `gsof/ins_vnav_rms_info_64`             | `gsof_msgs::msg::InsVnavRmsInfo64`          | Same as GSOF 50 but with heave RMS error (meters) for marine vehicles.                                                                                                                                                                                                                        |

### Services

| **Service (All in private namespace ~/)** | Argument                                                                                           | Return                                    | Description                                                                                                                                                                                                                                                                                                                                                                    |
|-------------------------------------------|----------------------------------------------------------------------------------------------------|-------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `get_origin`                              | `void`                                                                                             | `trimble_interfaces::srv::GetOrigin::Response` | Returns a boolean value and a set of LLA coordinates. A boolean value of `True` indicates that the origin of the local tangent plane coordinate system has been set by a valid GSOF message (either #2 or #49) and the LLA values in the response are valid. `False` indicates that the tangent-plane origin has NOT been set, and the response LLA values should not be used. |
| `set_origin`                              | `trimble_interfaces::srv::SetOrigin::Request`: Tangent-plane origin in Latitude-Longitude-Altitude | `void`                                    | Set the LLA coordinates used by the node to convert from LLA to NED.                                                                                                                                                                                                                                                                                                           |
| `reset_origin`                            | `std_srvs::srv::Empty::Request`                                                                    | `void`                                    | Clear the current coordinates being used as NED origin. The next INS solution received will be used as NED origin.                                                                                                                                                                                                                                                             |

# Disclaimer

This software package is provided for your convenience and is not an official product of Trimble Inc. While we have tested it and it works for our use cases, we cannot guarantee it will work for everyone. We will do our best to help with any issues, but please understand that we cannot promise any specific support or fixes.

# Product Support

For receiver or inertial navigation system support please refer to your corresponding support contact:

* [Trimble Applanix Support](https://www.applanix.com/contact.htm#support) for all Trimble Applanix brand products such as AP+ Air/Land, POS LVX, APX, etc.
* [Trimble OEM GNSS Receiver Support](https://oemgnss.trimble.com/en/contact-support) for all Trimble OEM receivers such as BD9xx receiver modules, BX9xx receiver enclosures, etc.
