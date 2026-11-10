{
  description = "TP4 INSA : asservissement visuel (tête TIAGo, Gazebo Harmonic)";

  inputs.nix.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.nix.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        rosDistros = [ "jazzy" ];
        rosShellDistro = "jazzy";

        extraPackages = [
          "vcs2l"
        ];

        extraDevPackages = [
          "boost"
          "foonathan-memory"
          "tinyxml-2"
          "fmt"
          "fcl"
        ];

        extraPyPackages = [
          "setuptools"
        ];

        extraRosPackages = [
          "xacro"
          "robot-state-publisher"
          "launch"
          "launch-ros"
          "ros-gz-sim"
          "ros-gz-bridge"
          "ros-gz-image"
          "ament-cmake-auto"
          "ament-lint-auto"
          "moveit"
          "nav2-bringup"
          "ros2-control"
          "ros2-controllers"
          "controller-manager"
          "joint-state-broadcaster"
          "joint-trajectory-controller"
          "diff-drive-controller"
          "gz-ros2-control"
          "key-teleop"
          "teleop-twist-keyboard"
          "rviz2"
          "twist-stamper"
          "twist-mux"
          "moveit-ros-control-interface"
          "moveit-py"
          "ros2interface"
          "cv-bridge"
          "image-transport"
          "vision-opencv"
          "rqt-image-view"
        ];
      }
    );
}
