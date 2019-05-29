# ur_calibration

Package for extracting the factory calibration from a UR robot and change it such that it can be used by `ur_description` to gain a correct URDF

## Nodes
### calibration_correction
This node extracts calibration information directly from a robot, calculates the URDF correction and
saves it into a yaml file.

With the parameters explained below calibration will be saved inside
```bash
<output_package_name>/<subfolder>/<robot_name>_calibration.yaml
```

#### Example usage
```bash
rosrun ur_calibration calibration_correction _robot_ip:=192.168.56.101 _robot_name:=ur10_ideal _output_package_name:=ur_calibration
```

#### Parameters
 * **"~subfolder_name"** (default: "etc")

     Given a package where the output should be saved, the calibration file will be saved in this
     subfolder relative to the package root.

 * **"~robot_ip"** (required)

    IP address of the robot. The robot has to be reachable with this ip from the machine running
    this node.

 * **"~robot_name"** (required)

    Arbitrary name that will be used for generating the calibration file's filename (see node
    description).

 * **"~output_package_name"** (required)

    Package inside which the calibration data will be stored in. This package has to exist and has
    to be writable. Otherwise execution will fail and calibration data won't be saved.
