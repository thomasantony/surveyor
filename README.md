# surveyor addon for Orbiter


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
