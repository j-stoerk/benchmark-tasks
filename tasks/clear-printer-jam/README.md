# Clear a jammed A4 sheet through a printer's rear access

Pull a single trapped page out evenly by both sides, following the rear-access extraction used on home and office printers. Both hands have to find the curled edge and maintain their grip while the sheet bends through the nip. A sheet left behind the rollers, or a fragment snapped off and removed, leaves a jam in the machine. See the [HP rear-access procedure](https://support.hp.com/gb-en/document/c06448951).

## Setup

Set the printer on the calibrated YAM station with the rear facing the robot mounts. Use a cooled, unplugged printer and open the rear access door before the trial. This setup isolates the sheet removal part of the operation. In the simulation X points away from the mounts and Y points left. Heights are above the tabletop.

| Part | Material and dimensions | Location |
| --- | --- | --- |
| Printer shell and lower guides | Unbranded ABS housing, 386 mm wide × 310 mm deep × 166 mm high, with the working region open at the rear. The guide gap at the return path spans 240 mm. | Housing centred at X 695 mm, Y 0. Rear lip centred at X 517 mm, 65 mm above the table. |
| Rubber feed rollers | Pair of 24 mm diameter rollers, 225 mm wide. The upper roller has a passive compression spring. | Axles parallel to Y at X 572 mm. Sheet height 71 mm. |
| Paper | A4 copier paper, 210 × 297 mm, 80 g/m². Start with its free edge curled up in the open access area. The trailing edge extends about 160 mm beyond the nip. | Centre the sheet on Y 0, with the curled near edge around X 465 mm and the trailing edge inside the path. |

The simulation uses a compact rear-access housing based on the documented extraction procedure. A spare desktop printer with an accessible paper return gives an inexpensive physical reproduction. Place it with its rear lip facing the robot, turn it off and unplug it, then manually insert a sheet into the open path. Use the regular rollers as the only restraint. Form an upward bend along the exposed 66 mm using two rounded edges. Set the trailing edge about 160 mm behind the nip. Check the exposed edge is visible and easy for both grippers to approach. A lightly curled jam needs no tear, staples or glue to reset. Use a new sheet after each attempt and supervise the cold, unpowered setup.

To reproduce the simulation rather than a specific printer SKU, assemble the panel and guide geometry specified in `task.py` from ABS panels. A fixed 24 mm rubber lower roller and a spring-loaded upper roller bound the nip at the specified coordinates. `printer_parts.py` models a roughly 1.6 N nip preload at the paper surface. Roller resistance and paper tear forces vary by printer and paper grade. The represented bend and break forces are estimates for an unpowered roller path and do not predict a printer's actual jam damage threshold.

## Score and reset

- **0.0** if any part tears off or remains caught beyond the feed rollers.
- **0.5** for removing the entire intact sheet from the path.
- **1.0** if the entire intact sheet is released on the bench below the rear lip and both arms are clear. The sheet may retain the initial curl.

Use a photo of the released sheet and the open path to verify the result. The simulator checks for separated seams and that the sheet rests below the guide lip, outside the pinch rollers and free of the grippers. The sheet is a chain of nine rigid panels with breakable bending joints. The guides and roller use physics contacts. This approximates an axially stiff sheet that can bend along the paper path. Diagonal buckling, detailed tear propagation, and friction changes due to toner and heat are absent.

Start a fresh trial to restore the sheet and all joints. The initial layout is fixed and the submitted trials use seed 0. Run from the task folder on the Robocurve Brev launchable with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0 after sourcing `yam.env`.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  inspect-robots run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E scene=station -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --max-steps 800 --seed 0 --no-prompt \
    --store-frames --log-dir "evidence/$policy"
done
```

The action limit is 800 steps (40 seconds of control time). Inspect Robots records the scalar `task_score` in its evaluation JSON. Use `inspect-robots video LOG.json --out streams` to export the overview and wrist recordings.

| Trial | Score | Actions | Control time | Measured wall time |
| --- | --- | --- | --- | --- |
| demo | 1.0 | 266 | 13.30 s | 79 s |
| model | 0.0 | 800 | 40.00 s | 141 s |
