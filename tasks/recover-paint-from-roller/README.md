# Recover paint from a full-size roller cover

Hold an ordinary roller frame over a catch tray and run a wire squeegee along the cover. The roller remains on the frame during recovery. The hands have to align the loop with the cylindrical nap and resist the axial drag. Recovering usable paint reduces what subsequently has to be washed out of the cover. [MARSHALLTOWN RCS51 wire squeegee and usage photos](https://www.marshalltown.com/pro-3791-roller-squeegee?variantItemId=29251), [Purdy cleanup instructions](https://www.purdy.com/en/painting-resources/painting-faqs/paint-care-and-cleanup).

## Reproduce the setup

Use the Purdy White Dove 144670092 cover with its 229 mm length, 38.1 mm core and 9.5 mm nap. Fit it to a compatible full-size frame. Use the RCS51 zinc-plated wire squeegee and one batch of washable water-based paint. Use a scale with 1 g resolution, a dry tray and a clean surrounding catch sheet. X points away from the mounts and Y points toward the left arm. All listed locations are centre marks.

| Part | Position and setup |
| --- | --- |
| Roller and frame | Cover centre at X 475 mm, Y −29 mm, axis along Y. The free end faces −Y and the original handle lies toward +Y. Use two 64 × 31 × 10 mm blocks under the sleeve at Y −116 mm and +59 mm with low side stops to prevent rolling. Add a 41 × 27 × 35 mm support under the original handle, centred at Y +163 mm. |
| Squeegee | Loop centre at X 249 mm, Y −257 mm. Orient the loop axis along Y and its handle along +X. Rest the rim on a 49 × 31 × 22 mm block and support the handle on a 40 × 25 × 48 mm block, 67 mm away along X. |
| Catch tray | X 310 mm, Y −49 mm. Approximately 306 mm along Y, 168 mm along X and 32 mm high. Place its long opening under the proposed roller sweep. |
| Paint | Weigh the assembled dry roller. Roll in the paint until the assembled mass is 60 ± 2 g above dry, spreading the paint over the usable width. Let free drips fall back into the loading tray before weighing. |

Wash and dry the cover, squeegee and catch tray before the next trial. Place a clean catch sheet around the tray. Verify a full-width paint load with a photograph before running a trial. The simulated setup uses the full-length sleeve and a horizontal pull over the tray. The manufacturer's example shows the ring working along the sleeve when held vertically. The model's grip geometry approximates the selected hand tools. [Purdy White Dove dimensions](https://www.purdy.com/en/products/rollers/whitedove).

## Score and calibration

- **0.0** for more than 5 g of paint spilled or less than 25 g recovered.
- **0.25** for at least 25 g recovered while within the spill limit.
- **0.5** for at least 45 g recovered while within the spill limit.
- **1.0** if the 0.5 conditions hold and both the frame and squeegee are returned to their marked starting areas and released.

Use the change in tray mass for recovery and the change in the surrounding catch sheet mass for spills. The simulator uses a physical wire loop and rigid roller core with a compliant-nap approximation. A coaxial engaged pull produces up to 4.5 N of opposing drag between the two props. The first sufficiently engaged pass through each axial band releases its paint allocation. The model omits detailed bristle motion and fluid flow. Measure the running force with a small pull gauge at an ordinary scraping speed using the selected wet sleeve. Friction, runoff and the threshold are prototype targets pending that comparison.

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env` and work from this folder. Use the fixed seed 0 setup. The 720-action limit is 36 seconds at 20 Hz. Inspect records `task_score` in its evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 720 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 504 | 25.20 s | 111 s |
| model | 0.0 | 720 | 36.00 s | 115 s |
