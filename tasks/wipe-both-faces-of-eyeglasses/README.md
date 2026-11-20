# Wipe both faces of both eyeglass lenses

Support the glasses by the frame and use a folded microfiber cloth to remove a set of small marks from both optical surfaces of each lens. The grips must expose all four faces and steady the light frame under wiping pressure. [ZEISS cleaning procedure](https://www.zeiss.com/vision-care/en/need-new-lenses/lens-and-screen-cleaning.html).

## Setup

Use an inexpensive plastic adult frame with approximately 47 mm rounded lenses, 62 mm between lens centres and 130–140 mm temples. Fit clear uncoated practice lenses or use a pair intended for the bench trial. Fold a clean 150 mm square microfiber cloth to approximately 36 × 84 mm. Leave about 52 mm of contact surface below the upper grip so the fingers can clear the frame during wiping. X increases away from the mounts and Y toward the left arm.

| Part | Start |
| --- | --- |
| Glasses | Centre of the face at X 360 mm, Y −8 mm. Stand the lenses upright, with their normals along X and the temples pointing back toward the mounts. Support the temples on two 21 × 12 × 54 mm blocks at X 342 mm, Y −68.5 and +52.5 mm. Place a 51 mm high, 137 mm wide rear rail at X 249 mm. Two 26 × 7 × 60 mm outside guides centred at X 256 mm, Y −77.5 and +61.5 mm limit sideways travel at the rear. The temples lift off freely and both lens faces remain exposed. |
| Cloth | X 351 mm, Y −219 mm. Place the cloth in a shallow 20 mm high rest with the folded edge exposed and its working face vertical. The cloth remains free to lift. |
| Marks | On each of the four faces, put three removable 4–5 mm spots on the practice lenses. Relative to each lens centre, mark locations Y/Z of −8/−7, +8/+2 and 0/+12 mm. Use the same washable test soil and stencil for each trial. |

Make a reference photograph of each lens side against a light background before wiping. Check on a spare practice surface that the marks release cleanly with the selected cloth. Restore the stand, frame and cloth to their marked locations and apply the same set of spots. Use a fresh clean cloth face between trials.

## Scoring

- **0.0** for fewer than four marks removed.
- **0.25** for at least four removed.
- **0.5** for at least eight removed.
- **1.0** for all twelve removed and both props released back at their starting areas.

Use the four reference photographs to count missing spots. The simulation estimates folded-cloth contact from the measured cloth and lens poses. It transfers about 1.1 N of reaction to the frame and removes a spot after cloth travel across the marked area. The model approximates contact pressure, pad compliance and lens curvature. The physical test soil and cloth force are prototype specifications pending calibration.

## Run

Use the RoboCurve launchable and native station with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env` and run from this directory. The fixed layout uses seed 0 and at most 1,000 steps (50 seconds at 20 Hz). Inspect reports `task_score` in the evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 1000 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 927 | 46.35 s | 153 s |
| model | 0.0 | 1000 | 50.00 s | 139 s |
