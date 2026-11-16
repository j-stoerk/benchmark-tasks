# Manual laboratory film drawdown

Implementation candidate. Native demo/model scores and media have not yet been measured. Do not interpret offline checks as physical success.

The robot draws a fixed-gap applicator across a wet test card and lifts clear. This is the manual coating stroke used in materials research, including electrode-formulation work. It tests maintaining tool contact and carrying a wet bead across a complete area. Gap selection, industrial-scale roll handling, coating chemistry and electrode performance are outside the claim.

The [manufacturer procedure](https://yasudaseiki.com/product/paint/doctor-blade/) describes pulling an80mm applicator at constant speed and gives the100x30x30mm/0.7kg scale used here. The [MTI catalogue](https://mtixtl.com/collections/applicators) lists the80mm fixed-gap SeKTQ4S family from$238.98. With ordinary glass, washable paint and clamps, the props budget is approximately$280 before shipping/tax; this is separate from the shared robot. No precision gauges or heavy custom proof fixture are needed. Our primitive geometry represents this tool class; it is original code, not manufacturer CAD. The two feet and working aperture are dimensioned in build.py and generated robot.usda.

The end tool is mounted before the trial. This is explicit: the task does not test picking up a coating bar. The robot is a single native Cartesian-gimbal manipulator with six controlled joints,120mm/s translation,40N per slide and2Nm per rotary axis. It is neither a pair of YAM arms nor a claim of joint-accurate commercial-arm simulation. An Inspect Robots Cartesian adapter can command a real six-axis arm carrying the same applicator. The evaluation requires a genuine LLM run for this embodiment, with its non-blind/code-aware status declared.

## Setup and scoring

See task.json for every object and its fixed starting position. The glass is120x300x4mm, topZ0.754m. The80x200mm rectangle is visibly marked; the initial3mL bead is atY-0.105m. Use room-temperature water-based washable paint, keeping it on the glass and mounted applicator. This avoids active materials, powders and organic solvents. Clean and reload the same bead for reset.

Score visible coverage using an overhead photograph and5mm transparent grid. The tiers are0,0.25 at25% coverage,0.5 at60%, and1 at95% with no dry stripe wider than5mm, followed by1s with the bar at least30mm above the plate. Any remaining outcome receives its applicable lower tier. Internal contact/height/velocity checks trigger the material response; they are not precision measurements required of the real-world judge. The coating grid and its gaps are rendered into the native scene so the submitted video displays the scored outcome.

Paint is a finite-volume,5mm raster response to actual native bar motion. Both feet must be seated, with actual contact force, before it deposits. Each cell consumes2.5microlitres at the declared nominal0.1mm wet thickness; the simulated bar also experiences a bounded viscous drag. This approximates material response and does not establish film thickness, rheology, adhesion or electrode quality. No object is teleported and no successful coating is inferred from a commanded path.

## Run

On the program's pinned Linux Isaac Lab-Arena environment, with RTX rendering:

```bash
source /data/IsaacLab-Arena/yam.env
uv pip install --no-deps -e tasks/lab-film-drawdown
python tasks/lab-film-drawdown/run.py smoke --output evidence/film-smoke-01 --max-steps 200
python tasks/lab-film-drawdown/run.py demo --output evidence/film-demo-01 --max-steps 1200
python tasks/lab-film-drawdown/run.py model --output evidence/film-model-01 --bridge evidence/film-bridge --max-steps 2400
```

The model policy waits for actual external LLM decisions through the documented request/response bridge and has no scripted fallback. Maximum control time is 120 s; actual wall time is recorded in each manifest. run.py retains the exact instruction, source hashes, installed versions, native Inspect log and physics trace. export.py exports all recorded frames at 20 Hz without selecting, accelerating or regenerating actions. Setup screenshot and separate model/demo media must match the final package before submission.

Adversarial offline checks cover an airborne pass, proximity without force, disabled paint, missing tool clearance and a visible dry stripe. Native no-material and normal trials must still be run. The current board comparison distinguishes this finite-bead/contact stroke from brush/roller panel painting (TK-7P97ME); battery terminology alone is not the distinction. Refresh the board and established-suite comparison before submitting.

Original source and primitive assets: MIT. Dependencies keep their upstream licences.
