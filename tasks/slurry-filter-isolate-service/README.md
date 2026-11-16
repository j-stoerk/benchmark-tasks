# Electrode-filter service fixture: pressure-model work in progress

This folder is not a complete task or submission. The current implementation
only covers two reduced water/air circuits and twenty-five offline checks. A native
cover/clamp/valve assembly, contact-only cartridge handling, scoring adapter,
successful demo and genuine model trial remain required.

The proposed full-size fixture uses a 100 mm cover bore and a 50 kPa gauge
test pressure, giving a real pressure load of 392.7 N. Both supply and return
can pressurize the pocket, so closing supply alone does not make it safe to
open. `hydraulics.py` conserves water volume against a separate, explicit
50 mL gas accumulator at atmospheric pressure. Native valve angles will set
the metering areas; measured cover/gasket gaps will set leakage; swept cover
volume must change the pressure state. The resulting pressure must apply its
force to the native cover, rather than being a decorative gauge value.

The model assumes incompressible water, a 20 C isothermal gas bladder, a
0.62 fixture discharge coefficient and 2 mm line restrictors. These are declared
bench parameters, not slurry properties or a manufacturer pressure rating.
Small open seal channels use the parallel-plate viscous law; large openings
are limited by inertial discharge. Ideal smooth compressed faces have no
open channel. This approximates gross mechanical containment and excludes
surface roughness, permeation, wetting and certification of production seals.

The underlying inviscid pressure/velocity relation is described by
[NASA](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/bernoullis-equation/).
The small-gap relation is the plane Poiseuille solution in
[MIT's fluid mechanics notes](https://ocw.mit.edu/courses/2-25-advanced-fluid-mechanics-fall-2013/1a114d602956fa0dd328155f9b45f93d_MIT2_25F13_Couet_and_Pois.pdf).
Their assumptions do not independently validate the complete fixture.

```sh
python tasks/slurry-filter-isolate-service/test_hydraulics.py
```

The thirteen reference-circuit checks cover pressure-times-area load, retained pressure after isolation,
vent relief, return backflow, source charging, hovering/local seal gaps,
moving-boundary compliance and time-step convergence. These checks do not
establish native contact success. The full contract and unresolved calibration
gates remain in the project design index.

## Extended service circuit

`filter_service/service_loop.py` tracks explicit water volume and air mass.
Four measured valve angles govern supply, return, top bleed/air vent and bottom
drain. The top port discharges water when submerged and air when exposed; it can
admit air under vacuum. A nominal 50 mL headspace supplies compressibility. Actual
cavity capacity and liquid volume below the bleed port are separate inputs, for
later coupling to cover motion and immersed cartridge displacement.

The custom service circuit uses 6 mm liquid restrictors, a 2 mm top bleed and
10 mm bottom drain. These are declared fixture choices, not OEM dimensions or
slurry process specifications. The 0.62 discharge factor and ideal
`sin²(angle)` metering law also need physical calibration for a real-world claim.
Bottom-port pressure includes hydrostatic head. Water uses signed Bernoulli
flow; air uses compressible subcritical/choked nozzle flow. Seal gaps determine
viscous slit leakage limited by inertial discharge. Backward Euler solves the
coupled mass balances and ideal-gas pressure without discarding water or air.
Vacuum produces an inward cover load instead of being hidden by a zero gauge.

The additional twelve checks cover independent water/air conservation, cover
pressure work, depressurization without emptying, drain stalling without air
admission, vented drainage, refill, return pressurization, an unseated seal,
choked flow and 240/480/960 Hz drainage convergence. All 25 checks currently pass.

```sh
python -m unittest discover -s tasks/slurry-filter-isolate-service -p 'test_*.py'
```

The port phase follows liquid level; free-surface dynamics are reduced to a
volume balance. Gas entrainment and detailed two-phase process-line flow are
excluded. Buoyancy and changing immersed solid volume still need native
coupling. These checks do not establish a successful native service procedure.

[DEYA's electrode-slurry cartridge](https://www.deyafiltration.com/products/New-Energy-Lithium-Battery-Filtration/Lithium-Battery-Slurry-Metal-Filter-Element-1.html)
documents stainless-steel construction and a 10 inch format. It does not supply
the custom fixture's mass, pressure or spring parameters.
[NASA's compressible mass-flow derivation](https://www.grc.nasa.gov/www/k-12/airplane/mflchk.html)
supports the reduced air-flow equation, not the completed fixture's accuracy.
