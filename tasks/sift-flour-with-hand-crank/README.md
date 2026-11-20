# Sift flour into a mixing bowl with a hand crank

Steady the loaded sifter's handle with one hand and drive the crank with the other. Sifting separates clumps and lightens flour for recipes such as angel food cake. The crank agitates the contents against a mesh. Keep the outlet over the bowl while the crank transmits torque into the sifter body. [Norpro rotary mechanism](https://www.norpro.com/product/s-s-rotary-flour-sifter-3c/), [King Arthur Baking on sifting flour](https://www.kingarthurbaking.com/blog/2024/06/24/why-sift-flour).

## Physical setup

Use a Norpro model 136 stainless 3-cup rotary flour sifter, a shallow 190 mm diameter mixing bowl about 40 mm deep, one dry kitchen scale with 1 g resolution, and 60 ± 1 g of fresh all-purpose flour. Mark the final set-down location at X 372 mm, Y 157 mm. X increases away from the arm bases and Y increases toward the left arm. All locations below are measured at the centre of the base.

| Object | Setup |
| --- | --- |
| Sifter | Centred over the bowl on the supports. The fixed handle points toward +Y. Crank faces −Y, with knob at the top of its travel. Fit a removable 20 mm-wide, 1.5 mm-thick food-safe silicone grip sleeve around the wooden knob. The simulated cylinder is 108 mm wide and 110 mm high, with a handle centred 81 mm from the barrel centre and a 31 mm crank radius. |
| Bowl | X 338 mm, Y −24 mm. Use a blue shallow bowl with a flat base, an inside opening of at least 180 mm and no lid. |
| Bowl supports | Two plain, food-safe wooden bars 177 × 8 × 8 mm. Lay them parallel to Y across the rim, each 48 mm from the bowl centre on opposite sides. Fix two 12 × 12 × 15 mm stops on top of each bar at ±47 mm along Y from the centre of the bowl. The sifter's bottom edge rests on both bars and the stops limit sideways motion under torque. Leave the top open so the sifter lifts off freely. |
| Flour | Add one measured charge loosely to the dry sifter. Do not pack or pre-sift it. |
| Spill surface | Use a clean lightweight paper mat or tare a tray before the trial. Its top is flush with the marked work surface. |

Empty and wipe both vessels, smooth the spill surface and recharge with fresh flour between trials. Set the crank straight up. Keep both arms out of the work area until the trial starts. Wipe the grips if dust builds up. A physical scale can check the bowl gain and the amount spilled independently.

## Score

- **0.0** for more than 3 g outside both vessels or dumped through the upper rim, or for less than 30 g sifted into the bowl.
- **0.25** for 30 g to less than 55 g in the bowl and no more than 3 g spilled.
- **0.5** for at least 55 g in the bowl and no more than 3 g spilled.
- **1.0** if the 0.5 conditions hold and the sifter is also released upright within 35 mm of the bench set-down mark.

Keep the bowl on the bench. The simulator uses a manufactured rim, a free sifter body and a damped crank hinge with contacts. It allocates a finite flour charge by measured crank travel and the projected positions of seven regions of the mesh. Flour leaving the upper rim counts as unsifted. Flow rate and flour behaviour are approximations. The tabletop witness mat is represented by the spill tally.

## Run

Use the RoboCurve launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Source `yam.env` and run from the task folder. The fixed layout uses seed 0. The limit is 1,200 actions at 20 Hz, or 60 s of control time. Inspect records `task_score` in its evaluation JSON.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 1200 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 1103 | 55.15 s | 186 s |
| model | 0.0 | 1200 | 60.00 s | 167 s |
