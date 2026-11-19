# Measure a wet coating with a notch gauge

A wet-film comb lets a coating lab check the thickness of a fresh application before it dries. Hold the comb perpendicular to the coupon so both outer feet reach the substrate, then lift to read the boundary between coated and dry teeth. The [Elcometer procedure](https://www.elcometer.com/en/how-to-measure-wet-film-thickness-using-the-elcometer-wet-film-combs) and [ASTM D4414](https://store.astm.org/d4414-95r20.html?gathStatIcon=true) describe this measurement. A loose witness coupon brings stabilizing it and moving the instrument into the same trial.

## Physical setup and reset

Use the calibrated bimanual YAM station. Coordinates below are measured from the midpoint of the two arm mounts projected onto the table. X points away from the arms and Y points left. All props start level on the table. Comb slots run parallel to X.

| Prop | Material and dimensions | Start position |
| --- | --- | --- |
| Coupon | Flat aluminium, 120 × 190 × 6 mm, about 360 g | Centre at X 405 mm, Y 28 mm |
| Wet patch | Washable blue water-based coating, 88 × 54 mm, 750 µm nominal | Centre at X 405 mm, Y −35 mm on the coupon, with the left-arm end dry |
| Notch comb | Stainless steel, 64 × 43 × 3 mm, about 62 g, with notches at 250, 500, 1000 and 1250 µm and two datum feet. See `task.py` for all tooth dimensions. | Standing in pickup slot, X 365 mm, Y −255 mm, teeth down |
| Two slot stands | Each is an 86 × 40 × 6 mm ABS base with two 86 × 10 × 20 mm rails forming a 10 mm slot. The rail tops are 20 mm above the table. | Pickup slot at X 365 mm, Y −255 mm. Reading slot at X 365 mm, Y −155 mm. Fix the stands to the table. |

For an inexpensive physical trial, the [B112AL12473-3 aluminium comb](https://www.elcometer.com/en/coatings-inspection/industrial-finishing-inspection-products/wet-film-powder-thickness/elcometer-112al-punched-wet-film-combs-aluminium.html) has the same manual measurement geometry. It is 75 × 65 × 1 mm with additional notches. Use the 500 and 1000 µm teeth for the score and keep its unused teeth dry outside the patch as needed. A [retailer lists a 10-pack](https://www.transcat.ca/elcometer-instruments-inc-b112al12473-3-b112al12473-3-311184) at CAD 206.18. Use offcut aluminium and two shop-made blocks for the stands. Use a blunt tool and keep the paint away from the gripper pads.

Make a fresh stripe with a 750 µm drawdown bar or two 0.75 mm shims and a straight edge, then remove the preparation tools. Wipe the comb and coupon clean and repeat for each trial. Put the comb and coupon back at the specified coordinates. A fine line or removable tape outside the wet stripe shows whether the coupon moved more than 10 mm.

## Scoring

Photograph the gauge and the impression under good side lighting. With the comb down square and both feet supported for one second, the feet and the 250 and 500 µm teeth should be wet. The 1000 and 1250 µm teeth should be dry. A readable impression earns 0.5. The released upright comb in the reading stand and a coupon within 10 mm of its original mark earn 1.0. A displacement of more than 3 mm across the patch while the comb is in contact, or an unreadable tooth pattern, earns 0. Use the video to check the hold and withdrawal, and a millimetre ruler next to the mark to check any dragged impression.

The simulation uses real gripper, tool, stand and substrate collisions. The wetting model records where actual tooth tips intersect the fixed paint layer. Colour indicates a wetted tooth. The model omits capillary flow, drying and the transfer back from the comb to another surface. The first operation requires a free tool grasp and independent coupon support. The reading step tests the transferred physical evidence of film thickness.

## Run and score

Use the RoboCurve Brev launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. Run from this task folder on the Linux instance after sourcing its `yam.env`.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 950 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

Seed 0 preserves the fixed starting layout. The action limit is 950 steps (47.5 seconds of control time). The evaluator reports `task_score` in the Inspect JSON. Use `inspect-robots video LOG.json --out streams` to recover the overview and both wrist streams. The full overlay places the overview above the two wrist streams.

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 419 | 20.95 s | 83 s |
| model | 0.0 | 950 | 47.50 s | 134 s |
