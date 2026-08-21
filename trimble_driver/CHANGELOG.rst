^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trimble_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-08-21)
------------------
* fix: Add buffer check to StreamPageParser
  can cause segfault in certain cases
* feat: jazzy: Port `#33 <https://github.com/trimble-oss/trimble_driver_ros/issues/33>`_ and `#36 <https://github.com/trimble-oss/trimble_driver_ros/issues/36>`_ (`#37 <https://github.com/trimble-oss/trimble_driver_ros/issues/37>`_)
  * fix(gsof_client): Remove unique_ptr wrapper on std::thread (`#33 <https://github.com/trimble-oss/trimble_driver_ros/issues/33>`_)
  Easy fix to prevent segfault when the thread is not created
  * refactor: make unit tests use synthetic data instead of pcaps (`#36 <https://github.com/trimble-oss/trimble_driver_ros/issues/36>`_)
  The ROS buildfarm doesn't support git lfs and thus all the unit tests fail. Made the main tests use synthetic data but also kept the old unit tests using recorded pcaps. These old tests can still be run and will run in github actions CI.
* fix: avoid pointer arithmetic
  This triggered CWE-468
* style: clang-format
* refactor: rename gsof_msgs to trimble_gsof_msgs
* fix: rename folder to match new package name
* Contributors: Andre Nguyen
