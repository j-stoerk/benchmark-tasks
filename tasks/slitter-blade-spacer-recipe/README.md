# Assemble and retain an electrode-slitter lower knife cassette

Implementation candidate; native demo/model evidence and submission are pending.
This is full-width offline tooling assembly on a vertical service arbor.
It does not evaluate cutting, burr quality or production throughput.

The blade format is grounded in the manufacturer's
[electrode slitter specification](https://www.tmaxcn.com/sodium-ion-battery-roll-to-roll-slitting-machine-for-electrode-manufacturing_p2869.html):
100 mm outside diameter, 65 mm bore and 2 mm lower-knife thickness.
The 330 mm cassette width and 75/80/75 mm mixed-width recipe define this
laboratory benchmark fixture. The service orientation, arbor fit, clamp load,
handling gantry and friction are declared design choices, not an OEM replica.
No manufacturer mesh or drawing is redistributed.

## Mechanism and observable outcome

Four 2 mm lower knives and five spacers begin on separate raised pedestals. The
handling head has an open-centre annular palm and three geared external jaws.
Actual contact must grip each ring, lift it, align its bore with the arbor and
release it onto the accumulating stack. Every task part is a free rigid body.
No object follows a prescribed pose and no grasp joint attaches it to the tool.
The head uses finite-speed, force-limited native joint drives. The annular
tooling geometry is one watertight 96-sided annular surface per free ring, with
sparse SDF collision (resolution 512, subgrid 6). The bore stays open. The chuck
and platen retain their compound convex sectors. This avoids internal compound
faces throughout the free tooling; actual loaded seating still needs validation.
Source dimensions, densities, masses, transforms, drives and frame structure
are explicit in `spec.py`, `assets.py`, `native.py` and `task.json`.

The spacers are 50,73,78,73,48 mm thick, interleaved with the knives.
Their total length plus four knife thicknesses is 330 mm. All four cutting
faces must independently lie at 50,125,205,280 mm from the shoulder; matching
only the total width is insufficient. Identical blades and equal spacers may
exchange identity. An ideal axial profile instrument reports the installed
blade heights in ascending order at 5 um resolution. It does not supply object
poses or a successful-grasp flag. RGB, native head encoders, three pad normal
forces and platen normal contact complete the policy observations.

During loading, the platen is raised and parked at Y=-0.45 m on a native
3 kg lateral carriage. Clamp commands move it to Y=0 at 0.10 m/s and lower it
only after actual alignment; retract commands raise it before parking. The
carriage uses the same 3 MN/m, 15 kNs/m, 1 kN limited drive as the handling
axes. This measured PLC sequence clears the arbor for incoming parts and
does not handle or seat them. The fully retracted platen centre is Z=1.25 m,
giving 40 mm clearance above the arbor before lateral travel. Its vertical
joint range is -90..+50 mm from the 1.20 m reference; initial joint position
is +50 mm. Allow approximately eleven seconds for clamping.
First park the handling head at XYZ=(0.60,0.40,1.35) m; its upper carriage must
also clear the pressure carriage's lateral route. Raising the tool alone does
not clear those upper rails. The controller must select this parking motion.
After assembly and release, the force-driven annular platen applies 500 N downward.
The proof applies 25 N upward and 0.5 Nm Z torque to each free blade, totaling
100 N and 2 Nm. Its force supplies no weight support during assembly. Three
seconds of retained geometry and limited translation/rotation are required.
The platen's joint cannot directly constrain any blade's translation or yaw.
Its real face contact and friction must transmit the loads through the stack.
The clamp gauge filters the platen contact against the final 48 mm spacer and
measures its upward Z component. Shaft rubbing cannot count as clamp loading.
Guide/drive covers are illustrative non-colliding geometry; existing native
constraints represent their bearings and powered support. This is not a frame,
anchor-strength or blade-damage test. All test conditions are declared fixture
proofs, not claimed production quality thresholds.

## Reproduction

Use the pinned Linux/RTX Isaac Lab-Arena environment, Python 3.12, Isaac Sim
6.1, `yam-arena` and Inspect Robots. Import `native.py` only after AppLauncher
starts Kit. The runner captures actual dependency versions, source hashes,
seed, commands, exit status and logs. Seed is 0; actions and scoring run at 20 Hz,
with 480 Hz discrete physics. The 2 mm lower knives retain their actual thickness.
GPU PhysX does not support CCD; smaller physics steps and bounded axis speeds
are used instead of claiming CCD protection.
TGS resolves articulation contacts last to test whether contact ordering corrects
the observed loaded-stack penetration. That setting requires fresh native
validation; the package currently has no full-score assembly evidence.

NVIDIA documents [dynamic SDF collision](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.1/dev_guide/rigid_bodies_articulations/collision.html)
and its [collision tuning limits](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/108.0/dev_guide/guides/collision_guide.html).
The new closed ring meshes have no internal sector faces. Their planar seating
faces and actual 2 mm thickness are unchanged. Mesh topology and SDF settings
are checked offline; cooked geometry accuracy and loaded contacts require native
evidence. The earlier convex-sector stack failed under load even at 255 position
iterations. That failed diagnostic is retained and is not successful demo evidence.

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --no-deps -e /path/to/tasks/slitter-blade-spacer-recipe
cd /path/to/tasks/slitter-blade-spacer-recipe
python -m slitter_stack.assets
python -m pytest -q test_slitter.py
python run.py smoke --output results/smoke --max-steps 200
python run.py demo --output results/demo
python run.py model --output results/model --bridge results/model-bridge
python run.py demo --variant no-grip --output results/no-grip --max-steps 400
python run.py demo --variant unclamped --output results/unclamped
python run.py smoke --variant contact-bench --output results/contact-bench --max-steps 400
```

The contact bench is a separate diagnostic layout with tooling initially
preassembled. It checks actual face contacts under the same clamp/proof loads;
its results never establish assembly success and cannot be used as the task's
demo or model evidence. The runner permits this variant only in smoke mode.

The scripted demo follows the known fixture layout and uses actual encoders
and pad-force feedback. The genuine external LLM bridge saves each observation,
then waits for an actual LLM response containing a bounded waypoint sequence.
No scripted fallback is available. At most 30 model calls and 2400 actions are
allowed. The controller name and code-aware/non-blind status must accompany
its published result; no independent API checkpoint is claimed.

The full-score demo and model run must match the submitted package. Videos
contain all recorded native RGB frames at 20 fps, with a measurement overlay
derived from the trace. New evidence directories are required for each run.
An infrastructure error is reported as a failed evaluation, never a model score.

Original code and generated geometry: MIT, see LICENSE. Recheck the board and
published benchmark objectives before submission; a generic ring-stacking
scene without the mixed internal recipe and real retention mechanism would
not establish this task's intended distinction.

The current contact diagnostic uses a 20 um contact-generation offset and zero rest offset for tooling, shaft, pedestals and machine colliders. Actual face seating remains subject to the same 0.05 mm rubric. This numerical change needs native load validation.

The current loaded-stack diagnostic requests 64 position iterations for both the machine articulation and free tooling, with one velocity iteration. The 1 mm contact-offset experiment failed under gravity; the 20 um offset is restored. No successful assembly run is currently claimed.

Free rings now have explicitly authored uniform-annulus centre of mass and analytic inertias, preserving their stated masses and dimensions. The first SDF reset was unstable before load; the current source needs a short unstacked gravity test before any clamp proof. These settings do not establish assembly success.
