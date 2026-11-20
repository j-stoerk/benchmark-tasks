# Release a suction phone holder without lifting its dashboard pad

Support the empty mount, lift the locking lever, peel its rubber tab to admit air and set the holder down. Removing the seal before lifting protects the dashboard pad and limits the force suddenly released into the operator's hand. [iOttie removal guidance](https://www.iottie.com/faqs/?mod=list&pageid=11), [Easy One Touch 6 mount](https://www.iottie.com/product/easy-one-touch-6-dash-and-windshield-mount/).

## Setup

Use an iOttie Easy One Touch 6 dash and windshield mount with its dashboard pad on a flat nonporous bench panel. The real mount's cup and lever remain attached, and both use their original grip surfaces. Simulated dimensions are listed below. X increases away from the arm bases. Y increases toward the left arm.

| Part | Specification and pose |
| --- | --- |
| Dashboard pad | Centre at X 360 mm, Y 0. Smooth top approximately 96 mm across and 8 mm above the bench. Keep it adhered to the bench panel with its original backing. |
| Suction mount | The original 74 mm rubber cup and telescoping arm. Extend the arm toward +Y. The support grip is about 106 mm from centre and approximately 70 mm above the pad. Leave the cradle empty. |
| Lever and peel tab | The lever begins fully closed and extends toward −Y. Use the original flexible tab outside the working side of the cup, with its free end accessible. |
| Destination | Clear at least 180 × 160 mm of bench on the +Y side. Mark a line 115 mm from the pad centre. |

To reset, clean the pad and cup per the manufacturer, centre the cup again and press the lever down. Leave the peel tab exposed as supplied. Start with the mount secured and the pad aligned to the marked circle. Before a trial give the arm a gentle pull to confirm a pressure seal. Allow the pad adhesive to settle according to the manufacturer when freshly installed.

## Score

- **0.0** if the pad moves or the bond is forced off while the lever and peel are still engaged, or if the mount is damaged.
- **0.25** if the lever is open while the pad is still down and the cup is seated.
- **0.5** if the tab has been lifted and the pressure seal breaks, with the pad still intact and in place.
- **1.0** if the holder is released, settles on the bench with its cup centre at least 115 mm from the pad centre, and both arms are clear.

Use a photo or video of the lever and tab, a ruler from the original pad centre and a photo of the pad after the trial. The model uses hinge resistance and contacts for the lever and tab. The pressure bond is approximated by a fixed releasable constraint driven by the physical lever and tab poses. The model omits the air pressure field, suction ageing, and deterioration of the dashboard pad adhesive.

## Run

Use the calibrated RoboCurve station with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env`, then run from this directory. Seed 0 fixes the start. The 1,000-step limit is 50 seconds of control time at 20 Hz. Inspect records `task_score` in its evaluation JSON.

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
| demo | 1.0 | 660 | 33.00 s | 105 s |
| model | 0.0 | 1000 | 50.00 s | 142 s |
