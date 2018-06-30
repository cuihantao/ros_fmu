General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.


# The ros_fmu_examples package

This package provides a small example for the use of the ros_fmu package. It includes a model of a damped pendulum in the [Modelica language](https://www.modelica.org/). In the following, we explain how to create an FMU from this model and how to simulate this model using the ros_fmu node.


## Creating DampedPendulum.fmu

There are several modeling tools that support the Modelica language and provide FMU export. Examples are [Dymola](http://www.3ds.com/products-services/catia/products/dymola), [JModelica](https://jmodelica.org/), and [OpenModelica](https://www.openmodelica.org/).

In the following, we explain the process by the example of OpenModelica, which has been also used to create the model of the damped pendulum at [share/DampedPendulum.mo](share/DampedPendulum.mo).

*   Download and install OpenModelica for Linux as described in [https://openmodelica.org/download/download-linux](https://openmodelica.org/download/download-linux).
*   Launch `OMEdit` and load the [share/DampedPendulum.mo](share/DampedPendulum.mo) model file.
*   Click on the DampedPendulum model in the project tree on the left.

![Screenshot of the DampedPendulum model in OMEdit V1.12.0](doc/damped_pendulum_in_OMEdit.png)

*   Navigate to Tools -> Options -> FMI and ensure that Version=2.0 and Type=Co-Simulation is selected.
*   Then click FMI -> Export FMU.
*   The path of the resulting FMU file is shown in the message browser at the bottom of the window, typically `/tmp/OpenModelica_[user]/OMEdit/DampedPendulum.fmu`


## Simulating DampedPendulum.fmu using the ros_fmu node

Once the DampedPendulum.fmu file as been created successfully, clone this repository and build it using `catkin build ros_fmu_examples`. Then, launch the ros_fmu node and rqt_plot using the provided [damped_pendulum.launch](launch/damped_pendulum.launch) file by:

`roslaunch ros_fmu_examples damped_pendulum.launch fmu_path:=/[PathTo]/DampedPendulum.fmu`

The ros_fmu node will load the FMU file and simulate it in real-time. The angle of the pendulum is plotted with rqt_plot:

![DampedPendulum.fmu simulation results in rqt_plot](doc/damped_pendulum_in_rqt_plot.png).

Please see the [README.md of the ros_fmu package](../ros_fmu/README.md) for how to load and run an FMU inside an application-specific ROS node or library.
