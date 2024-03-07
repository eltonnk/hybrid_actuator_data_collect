# Quanser Haptics 2 DOF Pantograph

This repository contains C code used to drive the now discontinued
Quanser Haptics 2 DOF Pantograph with the [Quanser AMPAQ-L2 Linear Current Amplifier](https://www.made-for-science.com/de/quanser/?df=made-for-science-quanser-ampaq-l2-amplifier-usermanual.pdf), 
[Quanser QPID Terminal Board](https://www.made-for-science.com/de/quanser/?df=made-for-science-quanser-qpide-data-acquisition-device-usermanual.pdf) and [Quanser HIL SDK](https://github.com/quanser/hil_sdk_win64).

A simple haptics scenario consisting of a virtual spring-damper wall is implemented
to demonstrate the capabilities of the 2 DOF Pantograph as a haptic device when
used with more recent Quanser driver hardware.

## Reasoning

The 2 DOF Pantograph was originally sold with the QPA-L2-E Multi-Channel Linear Amplifier, which is also deprecated, and which was attached to a data acquisiton card that would probably not be compatible with most modern PC motherboards.

The QPID, which is used in this project as a DAC, is also deprecated, but Quanser
sells other very imilar DACs which would be compatible with this project, with just a few minor changes in `panto_demo.c`.

## Requirements

This software has only been tested with the Windows version of the Quanser
HIL SDK, though a
[Linux version](https://github.com/quanser/hil_sdk_linux_x86_64) does exist that
should be compatible, with some changes in `panto_demo.c` to replace 
dependencies to `Windows.h`.

First, install the [Quanser HIL SDK](https://github.com/quanser/hil_sdk_win64)
and ensure that the environment variable `%HIL_DIR%` is set to the appropriate
location. This should be `C:\Program Files\Quanser\HIL SDK`.

Next, install
[Microsoft Visual Studio Community](https://visualstudio.microsoft.com/vs/community/),
as the HIL SDK only supports the `msvc` compiler.

Finally, install [CMake](https://cmake.org/), which will be used to build the
project.

<!-- TODO: reintroduce unit tests in the future? -->
<!-- Optionally, to run the unit tests, install
[Catch2](https://github.com/catchorg/Catch2) with CMake integration via `vcpkg`.
To do so, follow
[these instructions](https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md#installing-catch2-from-vcpkg)
in the `x86 Native Tools` terminal. However, be sure to install the 64-bit
version of Catch2 instead, by running
`vcpkg.exe install catch2:x64-windows`. -->

To run the python scripts in the `design_scripts/` folder, install the [latest version
of Python](https://www.python.org/downloads/). Make sure to select the "Add to PATH" option when going through the installation menus.

Then, run the following commands
from within `design_scripts/` to setup a virtual environment:

```
> py -m venv env
> env\Scripts\activate.bat
> (env) pip install -r requirements.txt
```

## Usage

To compile the project, first create a directory called `build/` inside the
root of the repository, navigate to it, and run CMake to generate the required
Visual Studio projects:
```
> mkdir build
> cd build
> cmake ..
```

Then run
```
cmake --build .
```
from within `build/` to compile the project.

The main executable is `build/src/Debug/PantograpDemo.exe`.


If the Pantograph is connected to the AMPAQ and the QPID, as shown in [this schematic](schematics/Schematic_2_DOF_Pantograph_Harness_2024-02-23.pdf), it will wait 5 seconds for the user to place the pantograph end effector in the homing position before running the controller. After execution, the program will save a CSV file containing the results in a `data` folder that will be created in this repo.

It will have a name like `panto_<TIMESTAMP>.csv`

To run any of the python scripts, run the following commands

```
> env\Scripts\activate.bat
> cd ..
> (env) py -m <insert_script_name_here>.py
```
from within `design_scripts/`.

These scripts plot useful graphs from data produced by `PantograpDemo.exe`.


## Outputs

The CSV output has the following columns

| Column | Description |
| --- | --- |
| `t` | Timestamp (s) |
| `target_current_0` | Saturated target current for motor 0 (A) |
| `target_current_1` | Saturated target current for motor 1 (A) |
| `current_0` | Measured motor 0 current (A) |
| `current_1` | Measured motor 1 current (A) |
| `filt_current_0` | Filtered motor 0 current (A) |
| `filt_current_1` | Filtered motor 1 current (A) |
| `axis_0_phi_1` | Angle between x-axis and linkage 1 (rad), driven by motor 0 |
| `axis_0_phi_3` | Angle between x-axis and linkage 3 (rad), driven by motor 1 |
| `pos_x` | End effector cartesian coordinate on x-axis (m) |
| `pos_y` | End effector cartesian coordinate on y-axis (m) |
| `force_x` | Target force to produce on end effector in x-axis direction (m) |
| `force_y` | Target force to produce on end effector in y-axis direction (m) |
| `axis_0_vel_phi_1` | Angular velocity (rad/s) of `axis_0_phi_1` |
| `axis_0_vel_phi_3` | Angular velocity (rad/s) of `axis_0_phi_3` |
| `vel_x` | End effector velocity in x-axis direction (m/s) |
| `vel_y` | End effector velocity in y-axis direction (m/s) |


## Acknowledgments

Thanks go to Steven Dahdah, who's code in the [quanser_qube](https://github.com/decargroup/quanser_qube) Github repo was for the most part copied directly into this repo, and in the main C file was somewhat modified to fit this specific application. 

