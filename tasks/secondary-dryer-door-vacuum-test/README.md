# Secondary-dryer vacuum-readiness fixture: partial implementation

This folder contains the reduced gas model and original articulated USD geometry
for a full-size door, floating hinge, seal carriers, swing/screw clamps and valves.
It has not been loaded and controlled in Isaac, has no native pressure coupling,
no demonstrated full-score trial and no model evaluation. It is not ready for submission.

Run the offline checks from this folder:

```sh
python -m unittest discover -s . -p 'test_*.py'
python -m dryer_vacuum.assets
```

The intended scope is closure and mechanical vacuum readiness of a full-size
electrode-dryer module. It does not measure electrode dryness or certify an
industrial leak specification. The proposed 20 kPa absolute test and 500 Pa
rise over 10 seconds are benchmark choices requiring physical calibration.

`dryer_vacuum.gas.Chamber` stores air mass and derives absolute pressure from
the ideal-gas law at constant 293.15 K. The caller supplies actual chamber
volume, sixteen measured gasket gaps, actual pump/vent shaft angles and the
physical pump switch. There is no closed-door Boolean or prescribed pressure.

Each seal segment uses compressible laminar parallel-plate leakage,
`mdot = width * gap^3 * (p_high^2 - p_low^2) / (24 * viscosity * length * R * T)`.
The effective gap is its positive measured clearance plus a proposed 2 micrometre
residual channel. Large gaps are capped by finite compressible nozzle flow.
The vent is an 8 mm reduced nozzle. Actual valve angle gives a `sin(angle)^2`
open fraction. Valve conductance, discharge coefficient 0.62, seal path 12 mm,
residual channel and constant effective pump speed all require calibration;
they are not manufacturer performance claims.

Backward-Euler integration solves the mass balance at the new pressure and
volume. Moving boundaries conserve air mass when all paths are closed. The
reported signed load `(ambient - chamber pressure) * door area` must be applied
to the actual native door at its centre, producing force and hinge torque.
For a 0.25 m2 door at 20 kPa absolute this is 20,325 N inward. Merely displaying
this value is insufficient: native mechanical coupling remains to be built.

The checks cover analytical pump-down, mass and pV conservation, pressure-force
sign and magnitude, an individual hidden seal gap, an open door, isolation,
venting, finite sonic flow and time-step convergence. Passing them validates
these mathematical limits, not real seal calibration or task feasibility.

The original geometry adds six offline checks: the actual500x500 mm pressure
opening,32.003 kg full-size solid steel door,0.8 mm hard-stop-limited squeeze of a proposed4 mm
seal, parked-clamp clearance through the entire door sweep, swing-before-draw
clearance and the2.8 mm draw distance to the compressed door. Sixteen contact
shoes approximate one continuous seal. Their internal pair contacts are filtered;
actual door contacts remain enabled. Proposed125 kN/m springs per segment,
1 kN clamps and60 Nm door motor still require calibration and native validation.
The gas core has13 checks, geometry6 and the proposed measurement/scoring
contract11,30 total. Contract checks reject pressure-only claims, missing native
load reaction, door-motor support, pump-compensated leakage, a missing seal,
unsafe opening/unlatching and interrupted holds. No pressure force is yet
applied to this CAD, so it cannot establish a loaded-door success.

Sources and parameter boundaries:

- [TOB BE-ZK-125L family](https://www.tobmachine.com/512l-vacuum-degassing-oven_p576.html):
  500 x 500 x 500 mm chamber, 125 L and 130 kg whole unit. The custom560x560x13 mm
  solid steel door is32.003 kg at7850 kg/m3; its dimensions and latch geometry
  are benchmark choices, not TOB specifications. Fixed-shell collider boxes
  represent hollow/insulated panel envelopes, not solid20 mm steel walls.
- [Leybold TRIVAC B](https://www.leybold.com/en/products/vacuum-pumps/oil-sealed-vacuum-pumps/rotary-vane-pumps-trivac-b):
  D16B pumping speed 16.5 m3/hour at 50 Hz is the reference scale. This model
  uses it as a constant chamber-side speed and omits the real pump curve and
  connecting-line conductance; it is not a pump digital twin.
- [NASA mass-flow choking](https://www.grc.nasa.gov/www/k-12/airplane/mflchk.html):
  ideal-gas compressible nozzle mass flux and its sonic bound.
- [MIT viscous-flow equations](https://web.mit.edu/2.25/www/pdf/viscous_flow_eqn.pdf):
  continuum viscous-flow equations underlying the stated slit-law derivation.
- [BINDER VD115 data sheet](https://www.binder-world.com/fileadmin/shop/1.d/files/6/2/62c0e495_Data-Sheet-Model-VD-115-de.pdf):
  its stated 0.01 bar/hour leakage is much tighter than the proposed gross
  500 Pa/10 s readiness threshold. The latter is not an OEM acceptance test.
