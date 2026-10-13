# Real vehicles in auvsim

Five vehicles built from published numbers (`src/real_models.hpp`, meshes in `src/real_models_mesh.hpp`, tests in `src/real_models_test.cpp`).
Only numbers were taken from the sources; no source code, mesh or texture was copied. Where a value is not published it is estimated with the methods
of the design engine (`auv_design.hpp`) and listed below. In the Vehicles window: **Real vehicles (open-source data)** > pick a vehicle > *Deploy real vehicle*.
Test option: `./auvsim --real N` (0 REMUS, 1 Tethys, 2 LAUV, 3 ECA A9, 4 BlueROV2).

Vehicle and company names (REMUS, MBARI Tethys, LAUV / OceanScan, ECA Group A9, BlueROV2 / Blue Robotics) are used only to say which vehicle a set of numbers describes.

| # | Vehicle | Data source | Licence |
|---|---------|-------------|---------|
| 0 | REMUS 100, Fossen reference | https://github.com/cybergalactic/PythonVehicleSimulator (`remus100.py`); part of the MSS family https://github.com/cybergalactic/MSS | MIT (Thor I. Fossen) |
| 1 | MBARI Tethys LRAUV | https://github.com/osrf/lrauv (`lrauv_description/models/tethys_equipped/model.sdf`, `scripts/description_generator.py`); MBARI fork https://github.com/mbari-org/gz_lrauv | Apache-2.0 (both) |
| 2 | LAUV | https://github.com/uuvsimulator/lauv_gazebo (`lauv_description/urdf/base.xacro`, `gazebo.xacro`, `snippets.xacro`, `actuators.xacro`) | Apache-2.0 |
| 3 | ECA A9 | https://github.com/uuvsimulator/eca_a9 (`eca_a9_description/urdf/eca_a9_base.xacro`, `eca_a9_snippets.xacro`, `eca_a9_control/.../model_params.yaml`) | Apache-2.0 |
| 4 | BlueROV2 | https://github.com/FletcherFT/bluerov2 and https://github.com/UUVControl/bluerov2 (`bluerov2_description/urdf/*.xacro`, `bluerov2_control/config/TAM.yaml`, `thrusters/T200-...csv`) | Apache-2.0 (both) |

Licences were read from the repositories' LICENSE files (GitHub licence API) on 6 Oct 2026. Not in the repositories, taken from public data sheets and papers:
ECA Group A9 (70 kg, up to 5 kn, 200 m: Naval Technology / ECA Group), OceanScan LAUV (18 kg+, 15 cm, up to 5 kn, 100 m, 8 h at 3 kn),
MBARI Tethys (2.3 m, 0.5 to 1.0 m/s in service, MBARI AUV 2010 and 2012 papers), Blue Robotics BlueROV2 (about 1.5 m/s, 100 m).

## What is published and what is estimated

### 0  REMUS 100 (Fossen)
Published (remus100.py): L 1.6 m, D 0.19 m, rho 1026, mass = spheroid 31.03 kg (docstring says 31.9 kg), CG 0.02 m below the CB, Lamb added mass (A = 0.839, 29.44, 29.44 kg;
0.0336, 3.426, 3.426 kg m2, with A44 = 0.3 Ix), linear damping from the time constants (20 s, 20 s, 20 s surge/sway/heave, 1 s yaw, zeta 0.3 roll, 0.8 pitch), Cd 0.42 on the frontal area
(Xuu 6.11), fins 0.00665 m2 each, rudder CL 0.5 and stern plane CL 0.7, +-15 deg, x -0.8 m, propeller 0.14 m, 1525 rpm, KT0 0.4566, KTmax 0.1798 at Ja 0.6632, thrust deduction 0.1,
KQ 0.0700 / 0.0312, top speed 2.5 m/s. Thrust 104.6 N at standstill and 41.2 N at 2.5 m/s follow from those coefficients. These are used as a reference next to the Step 7A preset, which is unchanged.
Estimated: battery 1100 Wh, hotel 25 W, 100 m depth, power chain zeta 0.45, reverse thrust 0.5, cross-flow quadratic drag in sway/heave/yaw/pitch, fin stall angle, fin servo speed.

### 1  Tethys
Published: total mass 147.5671 kg (neutral at 1025 kg/m3), CB 0.007 m above the CG, inertia (my sum of the published base, battery and part inertias: Ix 3.23, Iy = Iz 56.4 kg m2),
added mass 4.876 / 126.32 / 126.32 kg and 33.46 kg m2, quadratic damping only (6.2282, 601.27, 601.27, 0.1916, 632.70, 632.70), buoyancy engine 80 to 955 cc (500 cc neutral) at 3 cc/s,
31.73 kg battery moving +-0.03 m, propeller 0.2 m with Ct 0.0043123, fins 0.0244 m2 per set (cla 4.13, stall 0.17 rad, +-15 deg, x -1.05 m), propeller x -1.43 m, battery 400 Ah at 14.4 V, hotel 28.8 W.
Modelled with the simulator's buoyancy-engine mode: R/F = engine, T/G = moving battery.
Estimated: length 2.3 m / diameter 0.31 m for the outline, 300 m depth, **maximum propeller speed (50 rad/s assumed: the files give none)**, power chain, reverse thrust, fin servo speed, roll added inertia (source 0).

### 2  LAUV
Published: mass 18 kg, volume 0.017551 m3, L 1.08 m, width 0.15 m, CB 0.01 m above the CG, inertia from the cylinder formulas, added mass diag (1, 16, 16, 0.005, 1.3, 1.3),
linear damping diag (2.4, 23, 23, 0.3, 9.7, 9.7), quadratic diag (2.4, 80, 80, 0.0006, 9.1, 9.1), thruster at x -0.54 m with limit +-50 N and time constant 0.05 s, four fins 0.0064 m2, lift coefficient 3.0, at x -0.4 m.
Estimated: battery 300 Wh, hotel 10 W, 100 m depth, fin travel 30 deg (the +-90 deg in the source is a joint limit), fin stall, power chain. Off-diagonal damping terms were dropped.

### 3  ECA A9
Published: mass 69.7 kg, volume 0.068 m3 (urdf; `model_params.yaml` says 0.0823 m3 and "radius" 0.115: **the two files disagree, the urdf value was used**), L 1.98 m, D 0.23 m, CB 0.06 m above the CG, inertia 0.6 / 30 / 35, added mass diag (4, 95, 75, 0.4, 27, 32),
damping proportional to forward speed (diag 8, 162, 108, 13, 20, 32; evaluated at 1.5 m/s), four X-tail fins at x -0.708 m, area 0.04155 m2, cla 13.98 (above the 2 pi of a thin foil, used as published), stall 0.1745 rad, thruster x -0.9676 m.
Estimated: propeller thrust and top speed (design engine, the source only gives a rotor constant), battery 1500 Wh, hotel 20 W, 200 m depth, fin travel 30 deg (source +-80 deg), cross-flow quadratic drag.

### 4  BlueROV2
Published: mass 11.2 kg (10 kg in the second repo), volume 0.011054 m3, box 0.457 x 0.338 x 0.254 m, inertia 0.2 m (a2 + b2), added mass 1.7182 (surge), 5.468 (heave), 1.2481 (pitch), 0.4006 (yaw),
linear damping (11.74, 20, 31.87, 25, 44.91, 5), thruster positions and the thrust allocation matrix (four horizontal at 45 deg, two vertical), T200 data (51.5 N forward, 39.9 N reverse, 3533 rpm, 381 W at 16 V), thruster time constant 0.05 s.
Estimated: **quadratic damping (published as all zeros)**, sway and roll added mass (published as zero), CG 0.03 m below the CB (the source's 0.30 / 0.3872 m is an origin mix-up), battery 266 Wh, hotel 20 W, 100 m depth, thruster speed loss, rotational quadratic drag.
Two deliberate deviations, both explained in the code: (1) the horizontal thrusters are at z = 0 instead of the published -0.0725 m. This was re-tested with per-strip hull drag (drag acting at the frame's reference height): the published height still does not fly, because full surge (145.7 N) makes a 10.6 N m pitching couple against a 3.3 N m maximum righting moment (BG 0.03 m, estimated; BG 0.10 m only holds 60 % thrust), so the vehicle loops in pitch. A real frame has its centre of drag at about thruster height; the thrust line and the drag strips are both at z = 0 here;
(2) the rear thrusters' axes are flipped from `TAM.yaml`'s sign so that surge is the strong (forward) direction of all four T200 (145.7 N total, as published).

## Simulator changes made for these vehicles
`vehicle.hpp`: thruster reaction torque, actuator-disc power, per-thruster thrust loss; fin stall angle, servo force limit, mixed (X-tail) fins, servo speed; and the thrust allocator now scales a saturating command by the limit of the direction each thruster is used in (reverse thrust is weaker), which removed a surge leak in the BlueROV2 sway command.

## Validation (`./real_models_test`)
Mass, CG below CB, near-neutral trim, thruster allocation (surge, sway, heave, yaw; roll and pitch noted), 30 s full-surge speed against the published number, and the Step 7A REMUS preset against Fossen's numbers.
Flags above 25 % are printed with causes; nothing was tuned to hide them. See the test output and the notes in the summary.

## Licence notes
* All five sources permit use of the numbers. The Apache-2.0 repositories require a notice only when their files are redistributed; no file or code is copied here.
* The MBARI Tethys, LAUV, ECA A9 and BlueROV2 meshes in those repositories were **not** used; the meshes here are generated from code.
* The T200 performance CSV is Blue Robotics' public data distributed inside the BlueROV2 repository; only four numbers from it are used.
* The ECA A9 repository describes a generic "ECA A9" simulation model from UUV Simulator authors (Bosch), not ECA Group data; treat its values as simulation parameters.
