# Full-size powered electrode-roll handoff

This is a separate visibility revision. Fresh native demo/model recordings are
pending; previous video scores are not claimed for this revision. The 200 kg
mechanism, collision geometry, physical transforms, limits and scorer are
unchanged, verified by a structural physics audit and 23 offline checks.
The pending submission is preserved while this revision is validated.

## Production basis and scope

The task isolates the core-grip handoff module of roll-change equipment.
SINOLIFT describes production electrode-roll handling with 370/600 mm mandrels,
rolls up to 900 mm and loads of hundreds of kilograms:
https://sinoliftforklift.com/ten-more-cdd15-electric-roll-handling-stackers-for-lithium-battery-electrode-roll-handling/
LG Energy Solution's published replacement system uses concentrically aligned
transport and machine shafts:
https://data.epo.org/publication-server/rest/v1.2/publication-dates/2025-10-29/patents/EP4640603NWA1/document.pdf

The 600 mm x 370 mm, 200 kg instance is an engineering benchmark within those
published production-format ranges. It is not an exact commercial digital twin.
Its uniform annular inertia approximates the wound composite. The task evaluates
load transfer and grip retention, not coating quality, core-crush resistance,
telescoping, web tension control or cart navigation.

## Mechanism and measured load path

The 200 kg roll has a 1.96 kN gravity load. A purpose-built
powered machine has finite reach, velocity and force limits. Three geared radial
jaws expand against the actual bore; pad-force sensors require real
contact. The donor must withdraw and unload. A 25 Nm torque plus 300 N axial
pull tests the supported roll for two seconds. The scorer cannot substitute
commanded expansion, a pose, or an attached object for this load path.
An ideal internal gear constraint keeps the three jaw extensions equal, as in
a self-centering chuck. The three 4 kN drive channels have 12 kN combined input
capacity; the linkage redistributes their reactions. A 160 mm diameter,
80 mm long drive housing is included in the 35 kg head assembly. The geared
internals are not rendered. PhysX's one-DOF prismatic mimic uses its `rotX`
schema instance, as documented in the engine's single-DOF parser and
https://github.com/isaac-sim/IsaacSim/issues/731.

Both rigid contact and actuator compliance are simulated. Jaw drives command
outward or inward force, limited to 4 kN per channel, with zero position stiffness,
800 Ns/m damping and a 4 mm/s physical joint-speed limit. Axis stiffness is 1.5 MN/m,
damping 30 kNs/m; the lift axis is capped at 6 kN. Friction is 0.35 static and
0.30 dynamic. These are stated fixture parameters, not measured specifications
of a named commercial machine. The force-limit and collapsed-grip controls
are supplied as independent scripted controls; fresh measured results are pending.
Each open support frame has two 80 x 60 x 1200 mm posts at local Y=+/-370 mm,
an 80 x 800 x 80 mm crossbeam and a 320 x 800 x 80 mm foot. It reaches the
floor from its specified 600 mm centre height. Contact gauges report normal
force resultants; the applied-load test independently checks actual retention.

## Run

On the program's Linux/RTX Isaac Lab-Arena environment:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --python /data/IsaacLab-Arena/.venv/bin/python --no-deps -e tasks/industrial-roll-handoff
python tasks/industrial-roll-handoff/run.py demo --output evidence/industrial-demo-01
python tasks/industrial-roll-handoff/run.py model --bridge /tmp/industrial-llm --output evidence/industrial-model-01
python tasks/industrial-roll-handoff/run.py demo --lift-force 1000 --output evidence/industrial-weak-lift-01
python tasks/industrial-roll-handoff/run.py demo --no-expand --output evidence/industrial-no-grip-01
```

The embodiment is a powered transfer machine, with no bimanual YAM arms.
The rules therefore require an LLM run rather than Dream-1. The provided LLM
bridge writes RGB observations, measured encoders and pad gauges, then waits
for real external LLM decisions. It has no scripted fallback. The interactive
Codex controller's identity, prompts, responses, code access and call count
must be reported with the resulting score; it is not a blind benchmark estimate.
The separate demo controller is explicitly scripted.

The demo uses the measured donor gauge to preload the receiver before it
withdraws the donor. The new close-up looks along X into the core at the final
destination. Before the roll arrives there, this view intentionally shows the
empty destination; the overview retains the complete transfer.

`run.py` records source hashes, exact commands, native exit status, trace and
Inspect Robots logs. The maximum is 30 LLM calls / 2,400 control actions.
Successful evaluation processes automatically export both complete camera
streams into one H.264 video, with a display of actual trace readings.
Geometry assets regenerate with `python -m industrial_roll.assets`.

## Measurement and physical reproduction

Use the numeric rubric in task.json. Actual donor and three pad-force gauges,
axis encoders, a ruler and angular witness make every condition checkable.
A rated fixture applies the specified torque and axial pull. Force and pressure
readouts must be calibrated before a physical test. The numerical proof loads
are prescribed benchmark tests, not claimed thresholds for coating damage.
Only rated industrial hardware operated behind guards is appropriate; physical
trials by the team are optional. No physical equipment trial is claimed.
The fixed-world joints represent floor anchoring; this is not a mobile-cart
stability or anchor-strength test. The ideal proof actuator imposes a wrench
without supporting the roll's weight; cable geometry is excluded. Physics runs
at 240 Hz and the rubric evaluates 20 Hz measurements over an uninterrupted
two-second proof request.

## Visibility and validation

Both native RGB views are 1280x720, exported without cuts or acceleration into
one H.264 video with actual measured force readouts. The overview is lower and
shows the frame-to-carriage support path. The axial close-up reveals the core
grip once the roll reaches Y=350 mm, Z=1080 mm. Reduced lighting avoids washed
out geometry. Guide covers and a hollow donor bearing cover make the ideal
native guide constraints legible. Those covers have no collision or mass API;
they carry no additional force and cannot prevent a drop. Exact dimensions and
offsets appear in `task.json` and `visual_guides.py`.

New demo, model and media receipts will be added after successful native trials.
No existing recording is relabelled as evidence for this revision.

## Novelty comparison

The current board snapshot was reviewed on 16 November 2026. Nearby spindle or
chuck tasks include TK-S242XN (a keyed drill bit), TK-U2QMWD (a spring toilet-roll
spindle), and the earlier TK-GMW6LS (a filament spool on a rod). This task adds
the complete production-load support handoff: powered axes, a self-centering
core grip, donor unloading and an independent loaded retention check. Its
distinctive objective is the load path and support transition. The task does
not claim novelty from electrode material or colour alone. The production
references above motivate the equipment format and forces.

## License

Original implementation and generated USD geometry: MIT, see LICENSE.
Dependencies retain their own licenses. No third-party mesh files are included.
