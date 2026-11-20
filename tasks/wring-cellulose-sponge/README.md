# Wring a wet sponge into a basin

Two arms grasp opposite ends of a wet cellulose sponge. They lift it above a basin and apply opposite wrist rotations about its long axis. Wringing reduces the water left in a reusable sponge before it goes back on its rack. Cellulose sponges and sponge cloths are commercially available for this use. [3M cellulose sponge product catalogue](https://multimedia.3m.com/mws/media/2194494O/3m-jansan-cleaning-digital-marketing-tool-kit-cbg-home-care-division.pdf), [3M care instructions for cellulose sponge cloths](https://multimedia.3m.com/mws/media/1661435O/scotch-brite-18-spong-cloth.pdf).

## Bench setup

Use a Scotch-Brite ocelo utility sponge 7243-T, nominally 152 × 91 mm, with an unreinforced cellulose body around 24 mm thick when damp. The three simulated sections use a combined 152 × 88 × 24 mm envelope. The sponge is unmodified and both arms grip its original sides. X points away from the bases and Y points toward the left arm.

| Part | Setup |
| --- | --- |
| Sponge | At X 448 mm, Y 0. Lay its long dimension along Y, with both ends exposed beyond the rack bars. Saturate and drain it once for the first trial. Set its starting mass to its dry mass plus 90 ± 2 g using a scale with 1 g resolution. |
| Rack | Two bars 103 × 9 × 20 mm, with their long dimensions across X. Centre the bars at X 448 mm and Y −30 and +30 mm. Fix the bars to a level bench. |
| Basin | Blue rectangular plastic basin, about 198 mm along X, 284 mm along Y and 44 mm high, centred at X 295 mm and Y 0. Begin empty and tare it. |
| Catch surface | Tare a flat absorbent sheet or shallow tray beneath the bench working area to weigh spilled water. |

Place the sponge flat and untwisted on the bars. Use a fresh wet mass measurement for each trial. Empty and dry the basin and restore or replace the catch surface between runs.

## Score and model

Weigh the basin and surrounding catch surface immediately before and after a trial. Check the released sponge with a photograph.

- **0.0** for more than 5 g spilled or less than 20 g in the basin.
- **0.25** for 20 g to less than 40 g in the basin and no more than 5 g spilled.
- **0.5** for at least 40 g in the basin and no more than 5 g spilled.
- **1.0** if the 0.5 conditions hold and the sponge is released on both bars.

Spring-loaded joints convey resistance between the physically grasped sections. Counter-twist discharges water from a finite reserve. Runoff is allocated across three outlets at the section centres, so runoff at one end can spill while the rest is caught. When the sponge relaxes, a further wring can draw from the reduced reserve. The model omits pore structure and local pressure. The setup, torque and water thresholds are initial design targets pending physical calibration. To compare a real sponge, hold its centre above the basin and measure the water expelled from two deliberate counter-twists on a scale. Use the same charge, sponge size and orientation in each calibration trial.

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env`, then run these commands from the task folder. The start is fixed at seed 0. The 900-action cap corresponds to 45 seconds of control time at 20 Hz. Inspect records `task_score` in its evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 900 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 787 | 39.35 s | 198 s |
| model | 0.0 | 900 | 45.00 s | 133 s |
