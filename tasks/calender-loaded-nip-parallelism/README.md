# Loaded nip commissioning on a full-size laboratory calender

This is an unsubmitted implementation candidate. Native success and model
evidence have not yet been established. `task.json` scores and media remain
explicitly pending until genuine runs are recorded.

The objective is to compensate independently adjusted bearing gaps as a
20 kN service proof load opens and tilts the nip. The full-size 200 mm diameter,
300 mm face laboratory scale is grounded in the
[GK 300 L specification](https://matthews-engineering.com/products-services/specialty-machines/laboratory-machines/laboratory-calender-gk300-l/).
That manufacturer describes independent left/right adjustment under load and
15 tonne-force capacity. The benchmark is an original commissioning fixture,
not a digital twin of that machine. The 8/12 kN load split and 200 +/-10 um
gap recipe are declared service-test choices, not a production recipe.
[Micro-Epsilon](https://www.micro-epsilon.com/applications/industries/battery-production/precise-control-of-the-roll-gap/)
documents physical gap metrology for electrode calenders. No material
densification, porosity, coating quality or production throughput is evaluated.

## Mechanism and metrology

Two native joints allow upper-roll elevation and inclination. The lower roll
is fixed. Rolls are solid steel cylinders; rotation is locked for this service
operation. The upper roll and idealized housings have a combined specified
mass of 73.985 kg; the elevation carriage adds 1 kg. The frame and lower roll
are anchored. Two equal springs at Y=+/-175 mm are represented by equivalent
native generalized drives: vertical stiffness 2k=40 MN/m and tilt stiffness
2ka^2=1.225 MN m/rad, k=20 MN/m, a=175 mm. Damping transforms in the same way.
This reduced support model is valid for the small angles used here; the springs
are fixture definitions, not empirically identified OEM properties.
Two non-colliding telescoping housings show the support path from the crosshead
to the upper bearing blocks. They visualize the native drives and add neither
mass nor another load path. Each fixed housing is diameter 50 mm, length 180 mm,
centre Z=650 mm; each rod is diameter 25 mm, length 140 mm, centre 120 mm above
the upper axis. Both lie at Y=+/-175 mm.

Actions change the spring-rest targets with a finite slew rate. They never
write roll poses, teleport objects or impose a measured-gap value. Ideal service
spreaders apply upward forces 8 kN at the left and 12 kN at the right bearing,
equivalent to +20 kN Z force and +700 N m X torque on the upper roll. The
native integrator resolves displacement against the finite spring drives.
The omitted hydraulic cylinder geometry supplies no additional positional
constraint. With unchanged settings, the declared springs predict about
500 um opening and 571.4 urad additional tilt. A native load sweep must confirm
these predictions before submission.

At five Y stations, the paired probe model measures cylinder-surface separation.
For elevation q and tilt t, `gap(y)=q+y*tan(t)-R*(sec(t)-1)` at X=0.
The physical heads sit at X=25 mm in the wider entry wedge; their paired
distance readings are geometrically referenced to the nip line using the known
radius. The simulated ideal gauge reports to 0.1 um with zero noise/bias. Force
readouts are the actual applied ideal-actuator force, not a Boolean load flag.
The same quantities appear in the observation and in the unedited video's
measurement overlay. The overlay comes from recorded trace samples and is
never fed to the policy as a success oracle.

The scorer checks current geometry and load, latches roll contact/invalid states,
and requires an uninterrupted three-second loaded hold. Partial outcomes cover
an unloaded centre adjustment and then the full unloaded width. No credit is
carried forward after a bad final outcome. See `task.json` for exact limits.

## Run on the pinned Linux Isaac environment

Use the launchable's Isaac Lab-Arena, `yam-arena`, Inspect Robots and Isaac Sim
6.1 environment. Import native scene modules only after AppLauncher starts Kit.
Python 3.12, CUDA and RTX rendering are required. The actual installed versions,
hashes, seed, action count and wall time are recorded by each trial manifest.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --no-deps -e /path/to/tasks/calender-loaded-nip-parallelism
cd /path/to/tasks/calender-loaded-nip-parallelism
python -m calender_nip.assets
python -m pytest -q test_calender.py
python run.py smoke --output results/smoke --max-steps 200
python run.py demo --output results/demo
python run.py model --output results/model --bridge results/model-bridge
```

The demo uses feedback from the exposed gauges, not hidden roll state or the
offline equilibrium calculation. The external model policy blocks for actual
LLM responses to saved RGB/gauge requests; it has no scripted fallback. Its
controller/model and code-aware status must be named with any published result.
One response selects a bounded command and 1..200 repeated steps. The runner
fails if Inspect records an infrastructure error, even if the CLI exits zero.
Evidence directories must be new; they are never overwritten.

Negative controls:

```bash
python run.py demo --output results/unloaded --unloaded-only --max-steps 500
python run.py demo --output results/no-force --proof-scale 0 --max-steps 500
python run.py demo --output results/compliance --calibration-bench --max-steps 300
```

The first two controls may not reach 1.0. The compliance run holds the initial
settings fixed, then ramps the load, allowing comparison of the measured shift
against the declared spring constants. Native gap/load stability, source-matched demo,
genuine model attempt, current board novelty review and new media are required
before opening a PR. All geometry and code are original and released under the
included MIT license. No manufacturer asset is redistributed.
