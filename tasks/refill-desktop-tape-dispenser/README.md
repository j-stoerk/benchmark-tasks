# Refill a desktop tape dispenser

A desk dispenser is ready when its hub rolls in both slots and the first tape end projects over the cutter. Seal-box tasks start with tape loaded. This task starts with the two journals outside their slots and the leader on the wound roll. The arms seat the hub and work the sticky film across an unsupported gap. [Scotch C38 dispenser specification](https://www.scotchbrand.com/3M/en_US/p/d/v000200552/).

## Setup

Use a black Scotch C38 weighted desktop dispenser, its original removable 25.4 mm hub, and 19 mm Scotch office tape on a compatible core. The base is approximately 163 × 69 mm. Use a tape roll 49–54 mm outside diameter and its existing starter end, with only the end tab available to grasp. Put the tape roll on the hub before setting it down. X extends away from the station mounts. Y extends toward the left arm.

| Part | Location |
| --- | --- |
| Dispenser | Base footprint centred at X 346 mm, Y −51 mm. Put its long axis along Y, with the open hub slots at Y +5 mm and blade near Y −122 mm. The stepped model represents the clearance under the C38 cutting lip. |
| Refill | Place the hub and roll at X 346 mm, Y 228 mm on a 29 mm-high pair of narrow bearing blocks. Align the axle with X. Put the factory starting tab above the axle with its free edge approximately 8 mm clear of the outer wrap. Put short front and rear stops at Y 217.5 and 238.5 mm to prevent the roll walking while it waits for pickup. |
| Reference | Place a ruler near the station for inspection after the trial. The robot handles the dispenser, hub and tape. |

Restore the reel to its mark after each run. Wind the lead back to the original start around the roll. Replace wrinkled or torn tape. The dispenser is empty at the start and its two slots are clear. A fresh factory tab is suitable when it is near the specified leading-edge location.

## Evaluation

- **0.0** for a loose reel, a torn film or an adhesive strip disconnected from the reel.
- **0.25** for a hub seated in both slots with the film intact.
- **0.5** once the intact leader passes over the tear bar.
- **1.0** when the unsupported tape remains ready after both arms withdraw, with the tail projecting beyond the blade by at least 2 mm and holding within 3 mm of its end position for at least one second.

Photograph both seated journals and the tape crossing the cutter. Check the tail with a ruler. Pull gently by hand after scoring to verify that more film unwinds. In the simulation, eleven joined panels wrap the refill, and physical pulls bend them as they unwrap. Joint springs approximate the curved rest shape and resistance during peeling. Near-horizontal contact with the cutter land applies a weak breakable joint that represents tack. The peel resistance and adhesive contact have yet to be measured on the physical setup.

## Run

Use the RoboCurve launchable and stock station with Isaac Lab-Arena 0.3.1, Inspect Robots 1.0.0 and Inspect Robots YAM 0.38.0. After sourcing `yam.env`, run from this directory. This uses seed 0 and a fixed layout, with at most 680 control steps at 20 Hz. Inspect reports `task_score` in its evaluation JSON.

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
| demo | 1.0 | 545 | 27.25 s | 101 s |
| model | 0.0 | 680 | 34.00 s | 110 s |
