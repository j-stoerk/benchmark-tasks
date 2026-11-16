# Full-size powered electrode-roll handoff

Native implementation and 23 offline checks are complete. GPU and LLM validation
are in progress; no physics or LLM score is claimed yet.
This is a new task with a different embodiment and load path, not a resubmission
of the rejected handheld mock.

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

## What changed after the prior review

The roll now weighs 200 kg and has a 1.96 kN gravity load. A purpose-built
powered machine has finite reach, velocity and force limits. Three independently
moving jaws expand against the actual bore; pad-force sensors require real
contact. The donor must withdraw and unload. A 25 Nm torque plus 300 N axial
pull tests the supported roll for two seconds. The scorer cannot substitute
commanded expansion, a pose, or an attached object for this load path.

Both rigid contact and actuator compliance are simulated. Jaws are 4 kN limited,
with 5 MN/m position stiffness and 5 kNs/m damping. Axis stiffness is 1.5 MN/m,
damping 30 kNs/m; the lift axis is capped at 6 kN. Friction is 0.35 static and
0.30 dynamic. These are stated fixture parameters, not measured specifications
of a named commercial machine. Sensitivity and failure-control trials must
accompany the nominal demo before a feasibility claim.
Each fixed support frame is 120 x 800 x 1200 mm and reaches the floor from
its specified 600 mm centre height.

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

## Evidence gates

Required before submission: full-score native-physics demo; underpowered-lift
and unexpanded-grip failure controls; genuine LLM attempt; matching setup image;
all-frame H.264 videos; runtime/hash receipt and fresh novelty comparison.
The review corpus and nearest task comparisons remain in the workspace until
this task's final README is populated with its own evidence.

## License

Original implementation and generated USD geometry: MIT, see LICENSE.
Dependencies retain their own licenses. No third-party mesh files are included.
