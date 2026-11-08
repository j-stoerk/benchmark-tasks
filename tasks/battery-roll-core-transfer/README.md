# Transfer an electrode-roll mock through its core

A scaled core-handling task for two YAM arms with parallel grippers. The roll
is supported by physical contact with a separate spindle throughout the carry.
Original hollow USD assets, scoring and a physical scripted controller are included.

## Production step and tested ability

The robot threads a spindle through a roll's core, grasps its two exposed ends,
lifts and carries the roll, then lowers the spindle onto two bearing rests.
The final roll hangs above the table. This isolates core-based material handling
between roll-to-roll electrode operations.

[Fraunhofer ISE](https://www.ise.fraunhofer.de/en/business-areas/electrical-energy-storage/production-technology-for-batteries/wet-and-dry-electrode-manufacturing-and-thin-film-technology.html)
describes electrode production operations. A
[roll-handler manufacturer](https://schlumpf-inc.com/products/heavy-duty-roll-handlers)
describes core handling to avoid contacting valuable rolled material. Applying
that general handling principle to this battery-production mock is our design
inference. This is not a replica of a specific manufacturer's battery machine.

The 150 g roll has a 120 mm outside diameter, 50 mm width and 50 mm bore.
The spindle consists of a 60 g aluminium tube, 18 mm outside diameter, approximately
14.4 mm inside diameter and 240 mm long, plus two rigid polymer grip collars.
Each collar is 24 x 30 x 24 mm, centred at y = +/-100 mm relative to the shaft
midpoint. The combined nominal mass is 85 g. The flat collars prevent rolling
on the source table and provide grasping surfaces; they fit through the bore.
The collision cylinder models the tube outside;
its internal bore and exact annular inertia are omitted. These are scaled training choices.
An industrial web, expandable chuck, tension controller, coating, debris,
solvents and electrochemistry are absent. No production-scale lifting capacity
or material-damage detection is claimed.

## Actual hollow geometry

battery_roll/roll.usda contains 24 convex sector colliders under one rigid body.
Their inner chords leave the bore open. There is no single convex hull covering
the entire roll. The separate spindle is a single compound rigid body with a
cylinder and two cube colliders, axis along y.

The original generator is battery_roll/assets.py. Rebuild with:

```bash
python -m battery_roll.assets
```

Both asset files are included in the package and source archive. Arena registry
setup happens after the shared Kit application starts, before native scene
construction. The plugin changes no contact physics and never teleports parts.
GPU runs confirmed lifecycle/registry compatibility and rendering from all three cameras.

## Score, handling envelope and reset

Full numeric rules are in task.json and task.py. The score is 0, 0.25 for a
threaded spindle, 0.5 for a threaded roll moved to target, or 1 for a stable roll
suspended from the spindle on both rests. Both bore faces, finite spindle length,
polygonal clearance, both support locations and load sag are checked. Full
score holds one second. A spindle on empty rests receives zero.

Any score requires both measured TCPs 30 mm clear of both movable bounding boxes.
Additionally, a TCP entering the roll's expanded oriented bounding box at any
20 Hz sample latches zero until reset. This conservative proxy discourages
direct gripper handling; it does not detect every robot-link contact or events
between samples. The instruction states this exact envelope, rather than
claiming a simulated scratch/damage sensor. Use battery_roll; native yam_arena
alone omits the measured rules. Step info reports handling_violation and
measured object poses,
TCP positions and geometry predicates. Pass -E trace_file=path/to/physics.jsonl
to persist these diagnostics; they are not supplied to the Dream action policy.

Physical mock scoring uses top/front photographs, a ruler, the known bore and
shaft dimensions, and the complete handling video. Return the roll between its
chocks and the spindle to its mark; episode reset clears the handling history.

## Run and score

On the Linux L40S host, from the benchmark-tasks repository root:

```bash
source /data/IsaacLab-Arena/yam.env
python -m pip install inspect-robots==1.0.0 inspect-robots-yam==0.38.0
python -m pip install --no-deps -e tasks/battery-roll-core-transfer
python - <<'PY'
import json, subprocess
from pathlib import Path
task = Path('tasks/battery-roll-core-transfer').resolve()
spec = json.loads((task / 'task.json').read_text())
policy = 'battery_roll_scripted'  # Set to 'dream' for the genuine model attempt.
steps = 2400
policy_args = ['-P', 'hold_s=75'] if policy == 'battery_roll_scripted' else []
subprocess.run([
    'inspect-robots', 'run', '--embodiment', 'battery_roll', '--policy', policy,
    '-E', f'task={task / "task.py"}', '-E', 'object_jitter=0', '-E', 'control_hz=20',
    '-E', 'cam_width=320', '-E', 'cam_height=176', '--instruction', spec['instruction'],
    '--scorer', 'task_score', '--max-steps', str(steps), '--seed', '0',
    '--no-prompt', '--store-frames', '--log-dir', f'evidence/roll-{policy}-01',
] + policy_args, check=True)
PY
```

The physical demo uses battery_roll_scripted: the task waypoints plus bounded
joint-error integral compensation (gain 0.8/s, maximum 0.15 rad) for the native
compliant actuators. It receives the same measured joints as the model, without
object poses or simulator handles. Commands are clamped to joint limits; gripper
commands are unchanged. It never attaches objects or moves simulation state.

The genuine model run needs configured Dream credentials. No fallback controller
is used. Choose a new log directory per trial. The control limit is 120 s. Wall time includes startup, reset and inference
delays; exact measured times accompany the submission evidence.
run_config.json records the fixed trial recipe and expected core runtime pins.

Render genuine logs with inspect-robots video and use results.metrics.task_score
from those logs. The matching initial screenshot and complete videos are uploaded through the
repository media tool. The model input is exactly task.json instruction.

## Measured evidence

Both runs use seed 0, zero object jitter, 20 Hz actions and 320 x 176 RGB
observations from the top and both wrist cameras. The action limit is 2,400.

| Trial | Recorded final score | Actions | Control time | Elapsed wall time |
| --- | --- | --- | --- | --- |
| Dream-1 beta | 0.0 | 2,400 | 120.00 s | 520.36 s |
| Scripted physical demo | 1.0 | 1,093 | 54.65 s | 448.34 s |

The demo terminated with success after 20 consecutive full-score samples.
The model reached the step limit, did not thread the spindle and triggered
the handling exclusion. Both completed without an infrastructure error.
Earlier controller-development trials failed; the submitted demo retains the
original scoring thresholds and gives the parts additional time to settle.
Elapsed wall time includes simulator startup. This is one model attempt and
one successful demo, not a success-rate estimate.

The actual GPU runtime was Ubuntu 22.04 on one NVIDIA L40S, driver 565.57.01,
Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab package metadata 17.0.2,
Arena package metadata 0.3.0, Inspect Robots 1.0.0, YAM adapter 0.38.0,
Dream SDK 1.1.3 and NumPy 2.3.1. The launchable setup targets Arena release/0.3.1.
The launchable's package metadata does not equal its source branch name.
The YAM scene source is pinned to
0059d02a1a91bc6834c1d0846022891df0c78730 in
[kairosiann/yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730).
The Dream adapter revision is b5e936a201cec66c54c297e522c920d98445e339;
its default checkpoint, confirmed by the evaluation log, is dreamlabs/dream-1-beta.

[evaluation.json](evaluation.json) records the measured scores, terminations,
frame counts, runtime source hashes, log hashes and uploaded video hashes.
The videos contain every stored frame in order at 20 fps: top view above both
wrist views. Only compositing, top-view scaling and H.264 compression are used.
The setup image is the model run's unmodified initial top-camera frame.
No generated or schematic images are used as evidence. Media ID: M-FHMLF8.

## Novelty screening

The board refresh at 17:21 UTC on 8 November 2026 contained 1,146 tasks.
The closest spool task is TK-GMW6LS: directly grasp and reorient a filament
spool, then slide it onto a fixed cantilever rod. This task instead requires
inserting a separate movable spindle without grasping the roll, carrying the
roll through contact with that tool, and depositing both tool ends on separate
rests. The difference is the tool-mediated load path and handling constraint,
not the material name or colour. Both involve a bore and shaft; this is a
meaningful novelty risk for reviewers to assess.
Relevant neighbors include TK-2RAAL4 (grinding wheel on a spindle), TK-V8Z7SM
(shaft alignment/runout), and TK-ERQ59J (lifting a cover with keys). The proposed
distinction is threading a separate load-carrying tool through an aperture,
lifting at both exposed ends and depositing the suspended load on two rests.
No organizer acceptance or reservation is claimed.

The 25 task tests cover aperture/collar geometry, finite insertion, empty-tool
placement, sag, measured release, handling history reset, bounded feedback and
both full trajectories at 20 Hz. The demo inserts and lifts at the source,
centres the roll between the rests before advancing, and lowers the spindle
above the bearing cheeks. The optional 75 s final hold stays inside the 120 s
run limit and permits genuine contact dynamics to settle.

## License

All original code, generated USD geometry and its generator: MIT, see LICENSE.
No third-party mesh files are included; dependencies keep their upstream licenses.
