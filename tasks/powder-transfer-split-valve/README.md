# DN100 split-valve dry commissioning fixture

Align a supported 8 kg upper assembly, lock three lugs, retain a measured net
100 N separation load with the vertical position drive disabled, cycle the
mechanically coupled discs, reseat both halves and separate them safely.
The custom fixture tests mechanical readiness before battery-ingredient powder
transfer. It has no powder and does not test filled-container handling or certify
containment. Native feasibility and a genuine model run are still pending.

## Run and score

Use the pinned launchable environment: Python 3.12, Isaac Sim 6.1,
Isaac Lab-Arena, Inspect Robots and `yam-arena==0.1.0`, with an NVIDIA GPU.
The external `uv` executable must be on PATH.

```sh
uv pip install -e tasks/powder-transfer-split-valve --no-deps
python tasks/powder-transfer-split-valve/run.py demo --output /tmp/valve-demo
python tasks/powder-transfer-split-valve/run.py model --bridge /tmp/valve-decisions --output /tmp/valve-model
python tasks/powder-transfer-split-valve/run.py smoke --variant unlocked-proof --max-steps 300 --output /tmp/valve-negative
python -m unittest discover -s tasks/powder-transfer-split-valve -p 'test_*.py'
```

Use fresh output directories and seed0. Inspect Robots uses the registered
custom embodiment and `task_score`; the passive rotor is never commandable.
The run limit is 30 LLM calls and 1,600 actions at20 Hz. Each actual model request
contains two native RGB frames, practical encoder/force readings and operating
notes. Write `response-NNN.json` with `rationale` and a `sequence` of explicit
`{action:[x,y,z,collar_deg,active_deg,proof],steps:N}` waypoints. Each response
may supply at most400 actions. There is no scripted model fallback. The
separately named demonstration uses only public encoders and contact gauges.

Wall time, actual scores and decision count will be reported from completed
current-source recordings. Offline checks alone do not establish feasibility.

## Mechanics and measurements

`task.json` specifies the fixed layout, dimensions, materials and complete rubric.
`assets.py` regenerates the supplied original `machine.usda`; `native.py` builds
the Arena articulation. GPU TGS uses960 Hz physics,64 position/1 velocity
iteration and articulation contacts last. Control and video remain20 Hz.

The 8 kg payload comprises7.500 kg body,0.300 kg passive rotor and eight25 g seat
units. Each rotor has a60x12x12 mm steel dog atY=-135 mm, outside the casing and
collar. The lower and upper faces meet at the interface; opposite ends transmit
opening/closing torque through native contact. Lifting disengages them. The
passive rotor has only a preloaded return spring,0.5 Nm/rad and0.1 Nms/rad,
fixed rest-0.6 rad,0.8 Nm cap. It has no following motor, attachment or commanded
pose. Docked relative angle over2 degrees latches zero as coupling failure.

The active drive is limited to10 Nm and30 deg/s; collar5 Nm and20 deg/s. Normal
Z positioning has1 kN cap,100 kN/m stiffness and1 kNs/m damping, plus gravity
compensation. A Z=0.9998 m setpoint supplies about20 N seating bias against the
actual Z=1.000 m casing stop. The demo compares measured height with the physical
stop, rather than expecting the biased motor setpoint to be reached. During
proof, every Z position/damping term is removed. Upward effort is exactly
payload weight plus100 N; actual lug/collar contact must retain the payload.

Three native compliance axes per seat approximate a deformable lip: radial
10 kN/m,32 Ns/m,-1..+3 mm; axial1 kN/m,9 Ns/m,+/-8 mm; tangential1 kN/m,
9 Ns/m,+/-2 mm. Each unit is3 g radial guide,2 g tangential guide and20 g shoe.
The eight shoe boundaries align with the rotor bending axis. Each solid
100x2 mm disc has one32-sided convex mesh; each shoe has four matching facets.
No continuous elastomer constitutive law or microscopic leakage is modeled.

Each shoe gauge projects contact with its own disc onto the outward radial
axis. Axial force and contact with unrelated parts cannot credit seating.
The retaining gauge measures only downward collar force on the upper assembly.
The scorer uses actual encoders, those contact readings and applied proof effort.
One-second encoder windows establish stationarity; the load hold is two seconds.
Both discs must open, reseat and remain independently seated after separation.

The `unlocked-proof` control omits collar locking and requests the same force-only
separation load. It must physically separate and cannot establish retention.
Native interlock pins also inhibit opening while unlocked and unlocking while
open. The demo observes both closed angles and all sixteen seated gauges for
one continuous second before unlocking/lifting.

The overview shows the supported rig; the second camera looks down the actual
open bore from(0,-0.065,1.60) m. No colliders are hidden or cut away. Private
traces preserve actual link poses, XYZW quaternions, contact vectors, seat
deflections and actuator diagnostics for contact-integrity review. These private
diagnostics and score flags are not model inputs. `export.py` retains every
recorded frame and adds trace-derived captions.

## Provenance and limits

[GEA BUCK](https://www.gea.com/en/products/pumps-valves-flow-components/powder-valves-airlocks/containment-valves/buck-containment-valve/)
documents independently sealing halves, a locking/actuating ring, coupled
operation and DN100-250 sizes. It establishes the application and scale, not
the internals of this fixture. The8 kg payload, drive dogs, lugs, proof load,
spring values and tolerances are explicit custom benchmark choices.

[Festo DRVS data](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2025/DE/DRVS_DE.PDF)
show that a10 Nm quarter-turn actuator is available. This servo is not a GEA
torque curve or a pneumatic-actuator digital twin. Native load/contact
validation and physical spring/seat calibration remain necessary. Original
assets and code are distributed under the included MIT license.
