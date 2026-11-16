# DN100 powder-transfer split-valve mechanical service fixture

Dock an 8 kg supported container half, lock three real lugs, retain a net 100 N separation load with the vertical positioning drive disabled, open both complementary discs through native face contact, reseat, unlock and undock. This is a custom mechanical fixture for battery-ingredient handling; it does not test powder transfer or certify containment.

GEA BUCK describes two independently sealing halves, a central locking/operating ring and DN100-250 sizes: https://www.gea.com/en/products/pumps-valves-flow-components/powder-valves-airlocks/containment-valves/buck-containment-valve/ . DN100 is representative. The 8 kg payload, lug dimensions, loads, springs and tolerances are explicitly specified fixture parameters, not asserted OEM specifications. Do not extrapolate dry contact success to contamination control.

## Runtime

Use the pinned IsaacLab-Arena/YAM environment (`yam-arena==0.1.0`, Isaac Sim 6.1, Python 3.12, NVIDIA GPU). Install this task editable inside it; the external `uv` executable must be available on PATH.

```sh
uv pip install -e tasks/powder-transfer-split-valve --no-deps
python tasks/powder-transfer-split-valve/run.py demo --output /tmp/valve-demo
python tasks/powder-transfer-split-valve/run.py model --bridge /tmp/valve-decisions --output /tmp/valve-model
python tasks/powder-transfer-split-valve/run.py smoke --variant unlocked-proof --max-steps 300 --output /tmp/valve-negative
python tasks/powder-transfer-split-valve/test_valve.py
```

Every trial uses a fresh output directory and seed zero. `run.py` launches Inspect Robots with the registered custom embodiment and genuine external LLM policy or the separately named demonstration. The model bridge waits for actual JSON decisions; it has no scripted fallback. Each request contains two native RGB frames, encoder/contact readings and the operating contract. Respond with `response-NNN.json` containing a rationale and `sequence` of explicit `{action:[x,y,z,collar_deg,active_deg,proof],steps:N}` entries, totaling at most 400 steps. At most 30 calls and 1600 actions are allowed. The passive disc is never commandable. Trial wall time and model decisions will be reported after native validation.

`task.json` defines the complete fixed setup and rubric. `split_valve/spec.py` evaluates actual measurements, `assets.py` reproducibly authors `machine.usda`, and `native.py` builds the Arena articulation. `adapter.py` exposes only practical readouts, not success flags. `export.py` exports full recorded frames at their native rate with trace-derived instrument captions. During the load proof, vertical effort is exactly upper payload weight plus 100 N; stiffness and damping are zero, so an unlocked upper half can physically separate. The `unlocked-proof` negative demonstration deliberately omits collar locking. The seal sensors filter contacts against their own disc, and the retention sensor filters against the collar.

The sixteen 10 kN/m seal-carrier springs are a reduced seating model; leakage, dust, wear and chemical compatibility are excluded. The discrete 240 Hz solver and 48-sector geometry need native validation against the specified tolerances. A full native demo, real LLM recording and negative control must precede submission.


Each solid disc uses one closed 32-sided convex collider (64 vertices), with a maximum 0.25 mm radial faceting error. Annular housings use 48 sectors; each shoe uses four sectors matching the disc facets. The reduced model excludes microscopic leakage. Seal gauges project actual disc-contact force onto the shoe radial axis; axial forces do not establish radial seating. The retaining gauge measures the actual downward collar force. Full force vectors and seal deflections are retained in the private physics trace for calibration. The 240 Hz step keeps disc-edge substep motion below 0.12 mm at the specified speed. Fresh native contact and load-control validation is required.

The seat model provides three native compliance axes per shoe: radial 10 kN/m with 32 Ns/m damping, and axial 1 kN/m with 9 Ns/m damping over +/-8 mm. This represents compliant lip motion rather than a rigid ring blocking the butterfly disc. Each seat is a 3 g radial guide, 2 g tangential guide and 20 g shoe, so the upper aggregate remains 8 kg. The active-disc position drive uses 600 Nm/rad stiffness with the same 5 Nm effort cap. Stationarity is measured from a full second of encoder positions (0.1 mm /0.5 degree range), followed by a two-second loaded hold with the same baseline excursion limits. The demo allows four seconds of proof for settling; instantaneous solver velocity is recorded for diagnostics. Opening credit and final success require actual proof effort to be removed.

The eight shoe sectors on each half are centred at 22.5+45*i degrees. Their boundaries coincide with the rotor Y axis, allowing opposite sides of the reduced lip model to deflect in opposite axial directions. No rigid shoe bridges that bending axis. Native full-cycle validation remains required.

The current commissioning experiment resolves articulation contacts last in TGS. Private traces include actual native link poses (xyzw quaternion convention, verified against the rotary encoders), processed commands, explicit numeric-order seat joint deflections, total and disc-filtered shoe contacts, and available actuator diagnostic values. Only the documented practical encoders/gauges and RGB reach the policy. The earlier full cycle jammed; no successful demo is claimed.

The detailed native camera looks down the open vessel/neck bore from (0,-0.065,1.600) m toward (0,0,1.000), with 55 mm focal length. This exposes actual disc motion without hiding or cutting away colliders. The overview camera is unchanged. Camera visibility still needs inspection in fresh native recordings.

The third seat axis provides tangential compliance: 1 kN/m, 9 Ns/m, +/-2 mm and 5 N cap. It permits circumferential lip deformation instead of enforcing a rigid tangential constraint between contacting halves. The 112 mm housing bore clears combined radial/tangential strokes; the disc aperture remains DN100. TGS now requests 64 position and one velocity iteration. The previous native contact-order experiment completed at 0.5 and jammed near 9 degrees despite the full 5 Nm active drive; no complete-cycle demo has passed. These fixture spring choices require fresh native and physical calibration; they are not measured elastomer properties.

The active-disc drive is now capped at 10 Nm; the collar remains capped at 5 Nm. This is a fixture actuator choice, within the documented 10 Nm theoretical output of a Festo DRVS-32 quarter-turn drive at 6 bar, not an OEM GEA torque curve or a pneumatic-actuator digital twin. The native position servo remains 600 Nm/rad and 2 Nms/rad, with 30 deg/s speed limit. Native contact penetration and the complete cycle require fresh validation. The fixture uses an empty supported upper vessel for dry mechanical commissioning; filled-container transfer and powder containment are outside the task.

[Festo DRVS manufacturer data](https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2025/DE/DRVS_DE.PDF) establish that a 10 Nm quarter-turn drive is available; they do not validate this custom valve geometry or material model. The detailed camera is shifted toward negative Y to clear the support arm in the native image.

The passive disc has a fixed -0.6 rad torsion-spring rest angle: stiffness 0.5 Nm/rad gives 0.3 Nm closing preload against the zero-degree stop, with 0.1 Nms/rad damping and 0.8 Nm effort cap. This is passive hardware, not a commandable follower. Before unlocking/lifting, the demo verifies both discs within 1 degree closed and every radial contact gauge >=0.5 N continuously for one second. The numeric scorer remains unchanged. Latched physical exposure/workspace failures terminate immediately at zero. The previous 10 Nm trial achieved a full open/close but failed because the unpreloaded passive half opened about 4.19 degrees during undocking.

Outside the proof, the demo commands Z=0.9998 m to provide about 20 N seating bias through the existing 100 kN/m force-limited servo against the actual Z=1.000 m casing stop. This restores contact after the 0.1 mm upward lug clearance is taken up during proof. It does not prescribe or attach the payload pose. Proof still completely disables the Z servo and applies exactly payload weight plus 100 N; actual collar contact must carry the load.

The commissioning fixture uses separable external drive dogs: each steel bar is 60 x 12 x 12 mm at Y=-135 mm. Flat faces meet at the closed interface plane; opposite ends transmit opening and closing torque through native contact. The thick coupling avoids relying on the thin flow discs alone. Each rotor assembly is 0.300 kg; upper body 7.500 kg and seats 0.200 kg preserve the 8 kg payload. This custom coupling is not an OEM reconstruction. Its native coupled cycle has not yet been validated.
