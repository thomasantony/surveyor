# Surveyor spacecraft addon for Orbiter


An [Orbiter](http://orbit.medphys.ucl.ac.uk/) addon that demonstrates the Surveyor lunar probe with a functioning landing autopilot

## Building and running

1. Download/Install Orbiter from [http://orbit.medphys.ucl.ac.uk/](http://orbit.medphys.ucl.ac.uk/)
2. Install Visual Studio 2019 
3. Install Rust using `rustup` (https://rustup.rs)
4. Install the win32 target by running `rustup target add i686-pc-windows-msvc`
5. Build the project by running `cargo build` in the project root
6. Copy the `target/i686-pc-windows-msvc/debug/Surveyor.dll` to the `<OrbiterPath>/Modules`
7. Copy the files under the `Scenarios`, `Meshes` and `Textures` folders into the corresponding folders in the Orbiter install directory
8. Open Orbiter and run the "SurveyorTerminalDescent" scenario

## Descent Guidance

The descent guidance in this addon is implemented based on the descriptions in the following papers/reports:

- [Surveyor Mission Report - Part 1, Page 60](https://ntrs.nasa.gov/api/citations/19690003886/downloads/19690003886.pdf)
- [Surveyor III Mission Report - Part 1, Page 40](https://www.hq.nasa.gov/alsj/a12/Surveyor-III-MIssionRpt1967028267.pdf)
- [Surveyor Spacecraft Automatic Landing System](https://trs.jpl.nasa.gov/bitstream/handle/2014/38026/04-0406.pdf?sequence=1) by Sam W, Thurman
- [Design Considerations for Surveyor Guidance](https://ntrs.nasa.gov/api/citations/19670003873/downloads/19670003873.pdf) by R.K. Cheng, C.M. Meredith, and D.A. Conrad
- [Surveyor Terminal Guidance](https://www.sciencedirect.com/science/article/pii/S1474667017691250) by R.K. Cheng

The "descent contour" was approximated from the plots in the above reports.


## Acknowledgements

Big thanks to [Harish Saranathan](https://github.com/harishsaranathan) for clearing up some of my questions about attitude control. He has made his own version of the Surveyor descent guidance algorithm in C++ which can be found here: [https://github.com/harishsaranathan/SurveyorAutopilotForOrbiterSpaceFlightSimulator](https://github.com/harishsaranathan/SurveyorAutopilotForOrbiterSpaceFlightSimulator). 
