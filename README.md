# The ros_fmu repository

This repository provides the ros_fmu package for wrapping *functional mockup units (FMUs)* for co-simulation of physical models into ROS nodes. FMUs are defined in the [FMI standard](http://fmi-standard.org/) and can be created with a variety of modeling and simulation tools, including [Dymola](http://www.3ds.com/products-services/catia/products/dymola), [MATLAB/Simulink](https://www.mathworks.com/products/simulink.html), [OpenModelica](https://www.openmodelica.org/), [SimulationX](https://www.simulationx.de/), and [Wolfram System Modeler](http://www.wolfram.com/system-modeler/).

ros_fmu provides a library with convenience functions based on common ROS types to load an FMU during runtime, retrieve the input, output, and parameter names,  set timestamped input values, run the FMU's numeric solver, and query the resulting output.

In detail, this repository contains two ROS packages:

*   [ros_fmu](ros_fmu/) provides a generic library and ROS node for loading and running FMUs in ROS-based applications.
*   [ros_fmu_examples](ros_fmu_examples/) provides a tiny example for the use of ros_fmu.

Technical information on the interfaces and use of these packages is given in the README.md files in the corresponding subfolders.


## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).


## Requirements, how to build, test, install, use, etc.

Clone the repository into a ROS workspace and build it using [catkin](http://wiki.ros.org/catkin).


## License

ros_fmu is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in ros_fmu, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Quality assurance

*   Coding style:
    *   Google's C++ coding style is used, with some minor modifications. The conformance is checked using [clang-format](https://clang.llvm.org/docs/ClangFormat.html). The style definition can be found in [.clang-format](.clang-format).
    *   [run_clang-format_in-place.bash](run_clang-format_in-place.bash) reformats all C++ files of the local repository in place according to the style definition.
*   Linters:
    *   The [cpplint](https://github.com/google/styleguide/tree/gh-pages/cpplint) tool is used to detect common flaws and problems in C++ code. The rule configuration is contained in [CPPLINT.cfg](CPPLINT.cfg).
    *   The CMakeLists.txt and package description files are checked with [catkin_lint](http://wiki.ros.org/catkin_lint).
*   Unit tests: We already have a set of unit tests using rostest and a simple FMU. We plan to release them in the next weeks.

The [pre-commit.hook](pre-commit.hook) file provides a client-side commit hook to check the conformance with the coding style and the linters during every commit. In case of a finding by clang-format or one of the linters, the commit will be rejected. To set this client-side commit hook up, please run `ln -s ../../pre-commit.hook .git/hooks/pre-commit` from the local repository root.


## Known issues/limitations

Please notice the following issues/limitations:

*   ros_fmu only supports FMUs according to the FMI 2.0 standard.
*   ros_fmu treats all inputs, outputs and parameters of a given FMU as floating-point values (ROS message std_msgs/Float64, C++ type double, FMI type fmi2fmi2_real_t).
