# Close an open bread bag with its reusable retaining clip

One hand gathers the loose polyethylene neck while the other slides the slot in a separate clip over the film. The clip should remain captured by the film after both hands let go. Use a clean bread bag and the hand-applied Kwik Lok Series A clip, which is 21.4 × 34.9 mm. [Series A closure](https://www.kwiklok.com/product/series-a/).

## Setup and reset

Place a 120 × 240 × 90 mm foam loaf in a clear polyethylene bread bag with at least 100 mm of loose film beyond the loaf. Centre the loaf at X 367 mm and Y +139 mm on the station, with the opening toward negative Y. Two 9 × 190 × 16 mm side guides at X 298 mm and 436 mm, centred at Y 170 mm, and a 147 × 9 × 16 mm rear stop at Y 268 mm limit loaf movement. Rest the base of the loose film on a smooth 46 × 16 × 61 mm high block centred at X 357 mm and Y +7 mm. Leave the neck open about 20 mm wide and hanging past it, with enough free film beyond the working area to keep the clip engaged after release.

Put the clip in an upright shallow slot at X 335 mm and Y −195 mm, with the open notch facing negative X. A 44 × 25 × 32 mm wooden block and two 3 mm-high strips make the clip rest. The slot holds the starting clip upright for pickup. Reset by removing the clip, reopening the bag and returning both to their marks.

## Score

- **0.0** if the neck is torn or fewer than two film sections reach the interior window.
- **0.25** if at least two film sections fit inside the window.
- **0.5** if all five film sections fit while the hands still support the closure.
- **1.0** if the undamaged full neck is captured inside the window, with at least 8 mm of tail, while the loaf rests on the bench and both fingertips are at least 60 mm clear. The clip and loaf must stay within 1.5 mm of their settled positions for two seconds. The clip can move up to 5° within that window.

Check both sides of a physical trial to confirm the film passes through the aperture. The simulated film is represented by five thin articulated strips with breakable creases. Two small sprung teeth approximate the elastic deflection at the clip entry. Film wrinkles, polymer strain and the precise force of entry remain to be checked with the named physical items.

## Run

Use the RoboCurve launchable and source `yam.env`. From the task directory run the following command for each listed policy.

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
| demo | 1.0 | 376 | 18.80 s | 69 s |
| model | 0.0 | 680 | 34.00 s | 106 s |
