^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robotont_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2024-05-19)
------------------
* Changes to parameter handling
* Contributors: Zhven

0.1.2 (2024-03-18)
------------------

0.1.1 (2024-03-18)
------------------
* Solved build warnings
* Odom fix
* fixed timeout
* added timeout to the fake_driver and descriprion launch to the launch file
* Merge pull request `#16 <https://github.com/robotont/robotont_driver/issues/16>`_ from robotont/humble-devel-raimo
  Humble devel raimo
* removed redundant dependency
* resolving minor merge conflict from led plugin
* Add license file
* generic commenting
* commented out some code
* Added ledmodule code
* Smalle changes to README
* Reworked logging and odom namespace
* Foundation for led and range plugins
* First implementation of fake_driver
* Added plugin launch arguments
* Launch file + parameters
* Tested and validated current functionality. Added some comments.
* Working odom + cmd_vel
* Functional Odom and TF messages
* Merge branch 'ros2-foxy-devel' into devel-ros2-rolling
* Groundwork for odom plugin layed out
* first attempt at plugins
* Create README.md
* Ubuntu 22 + Ros2 rolling
* First rolling commit
* reverted async_receive_handler; added watchdog timer for serial port monitoring
* Serial reading working without correct error handling.
* End of file error
* Attempted serial echo
* New beginnings; compiling code with working reading from serial
* Changes to packet polling
* Serial data packet passing between nodes is functional
* Serial read improvements
* Merge branch 'ros2-foxy-devel' of https://github.com/robotont/robotont_driver into ros2-foxy-devel
* Incomplete mess
* template with async read working, moved to robotont namespace
* passing node shared pointer to the hardware class, still have to resolve a weak ptr exception due to shared_from_this() being called from constructor
* Subscriber not working
* Reading from serial working
* First working compile
* First successful compile
* First commit. Include errors.
* Initial ROS2 commit
* Merge pull request `#13 <https://github.com/robotont/robotont_driver/issues/13>`_ from robotont/melodic-devel-maarika
  stopping mechanism
* velocities to 0
* stop
* Merge pull request `#12 <https://github.com/robotont/robotont_driver/issues/12>`_ from robotont/melodic-devel-ns-slash
  Melodic devel ns slash
* improved comment layout
* ns with slash
* Merge pull request `#11 <https://github.com/robotont/robotont_driver/issues/11>`_ from robotont/melodic-devel-ns
  Namespaced
* removed instructions about remapping
* Merge branch 'melodic-devel' into melodic-devel-ns
* Merge pull request `#9 <https://github.com/robotont/robotont_driver/issues/9>`_ from robotont/melodic-devel-veiko
  Travis fix
* turning debug off again
* converted run_depend tags for package format 2
* travis:enable debugging
* removed cmake_modules dependency, package.xml format 2
* changed arg name
* ns
* ns
* driver_basic ns
* ns
* Merge pull request `#6 <https://github.com/robotont/robotont_driver/issues/6>`_ from robotont/melodic-devel-veiko
  Melodic devel veiko
* travis from .org to .com
* renaming RobotontHW->Hardware and RobotontDriver->Driver
* Merge branch 'melodic-devel' into melodic-devel-veiko
* Merge pull request `#5 <https://github.com/robotont/robotont_driver/issues/5>`_ from robotont/melodic-devel-maarika
  removed gazebo_driver and gazebo_odom
* removed gazebo_driver and gazebo_odom
* changed loglevel of loading messages and removed text formatting dependency from launch file
* control a single led or the whole segment with a same keyword
* Merge pull request `#3 <https://github.com/robotont/robotont_driver/issues/3>`_ from robotont/melodic-devel-veiko
  cleaned up deprecated stuff
* improvements based on Karl's review
* frame changed to base_link
* namespaces for android app instructions in README
* minor improvements to README
* specifying rosinstall file in .travis.yml
* moved travis upstream workspace packages to a separate .rosinstall file
* added robotont_msgs to travis script
* added install tags to CmakeLists and removed old .rviz file
* Namespace explanations to readme, added travis
* put fake driver under robotont ns, reusing launch files from robotont_description
* added dependency to robotont_description, fixed a typo in authors name
* restructuring the main README.md, merged stuff from src/README.md and removed it
* Updated authors, maintainers, and other meta info
* Need to specify namespace when running keyboard teleop node with rosrun.
* cleaned up deprecated stuff
* removed deprecated fake_odom.launch
* Merge remote-tracking branch 'origin/melodic-devel-maarika' into melodic-devel-veiko
* Merge remote-tracking branch 'origin/melodic-devel-ranno' and 'origin/melodic-devel-rvalner into melodic-devel-veiko
* removed typo from readme
* removed typo from readme
* modified readme
* changed fake_odom_publisher.ccp -> fake_driver_node.cpp and added fake_driver.launch
* C++ 11 standard added
* Added power supply status plugin
* led module plugin, range plugin
* fixed duplicate iterator increase bug in writePacket function
* modular plugin based architecture - a complete rewrite
* implementation template for RangeSensor
* sensors node
* Merge pull request `#2 <https://github.com/robotont/robotont_driver/issues/2>`_ from robotont/melodic-devel-veiko
  removed deprecated notification from readme
* removed deprecated notification from readme
* Merge pull request `#1 <https://github.com/robotont/robotont_driver/issues/1>`_ from robotont/ihar
  fake odom added
* joy is removed
* fake_odom_publisher created
* copy from the old repository
* Initial commit
* Contributors: Ihar Suvorau, M, RValner, Raimo Köidam, Veiko, Veiko Vunder, Veix123, Zhven, kasutaja, kruusamae, m, patsyuk03, peko, rannomspp, sven-ervin.paap
