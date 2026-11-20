# File a loose sheet in a three-ring binder

A loose page has to flex around the grippers while its three punched holes register with an open ring line. Close the original lever after the page goes onto the rings. The comparable real procedure uses an Avery 79589 one-touch one-inch binder and standard punched letter paper. [Avery 79589 and its lever directions](https://www.avery.com/products/binders/79589).

## Reproduce the scene

Use the calibrated station. X increases away from the mounts and Y toward the left arm. Set the empty binder flat open, with the spine along Y and the lever toward positive Y. Open all three rings. Use one ordinary 80 g/m² sheet of 216 × 279 mm U.S. letter paper with three 6.3–6.6 mm holes, spaced 108 mm apart. The line of punched holes is about 12.5 mm from the sheet edge nearest the spine.

| Part | Dimensions and placement |
| --- | --- |
| Binder | Red board-and-vinyl covers, each about 233 × 304 mm. Centre the spine at X 399 mm, Y 0. The three receiving tips are spaced along Y at −108, 0 and +108 mm. |
| Loose page | Centre at X 277 mm, Y 0, long dimension along Y. The holes face away from the mounts and lie beside the binder spine. |
| Page starting supports | Smooth wooden bars about 160 × 12 × 18 mm. Place one under the page at each of Y −89 and +89 mm. Keep both short ends overhanging the bars so that one pad can slide beneath each end. |

To reset, pull the lever back open and remove the sheet. Use a fresh sheet after any visible tear or extended hole. Straighten and align the sheet at the marks before each trial. Leave the spine, covers and original lever in their setup positions.

## Score

- **0.0** for a torn sheet or zero threaded holes.
- **0.25** if one ring passes through one hole.
- **0.5** if at least two rings pass through their holes.
- **1.0** if all three rings pass through their holes, the rings close, and the released sheet lies within the open cover.

Check the sheet and ring closure visually. After the trial, flip the loose sheet once to confirm attachment through all holes. The prototype uses three linked panels with breakable bends and physical openings for the ring tips. A joint links the three gate halves. Contact-driven lever motion trips the spring closure, which approximates the binder's shared over-centre mechanism. The simulator uses simplified wire and hole cross-sections and a three-panel approximation of paper flex.

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env` and run from this directory. The fixed setup uses seed 0. The limit is 680 steps at 20 Hz, or 34 seconds of control time. Inspect reports `task_score` in the evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 680 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 461 | 23.05 s | 77 s |
| model | 0.0 | 680 | 34.00 s | 100 s |
