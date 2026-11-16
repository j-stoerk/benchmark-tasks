"""Mass-conserving isothermal vacuum node with measured segmented seal gaps.

The slit law integrates laminar parallel-plate flow with ideal-gas density.
Large openings are capped by compressible nozzle flow. This reduced model
requires physical calibration; it is not a containment or drying model.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Parameters:
    ambient_pa: float = 101300.0
    temperature_k: float = 293.15
    gas_constant: float = 287.05
    gamma: float = 1.4
    viscosity_pa_s: float = 1.81e-5
    chamber_m3: float = 0.125
    door_area_m2: float = 0.25
    seal_perimeter_m: float = 2.0
    seal_path_m: float = 0.012
    residual_channel_m: float = 2e-6
    segments: int = 16
    pump_m3_s: float = 16.5 / 3600
    vent_diameter_m: float = 0.008
    discharge_coefficient: float = 0.62

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"Invalid parameter: {name}")
        if self.segments < 1 or int(self.segments) != self.segments:
            raise ValueError("Seal segment count must be a positive integer")
        positive = (
            self.ambient_pa,
            self.temperature_k,
            self.gas_constant,
            self.viscosity_pa_s,
            self.chamber_m3,
            self.seal_path_m,
        )
        if min(positive) <= 0 or self.gamma <= 1 or self.discharge_coefficient > 1:
            raise ValueError("Invalid gas, geometry or nozzle parameter")

    @property
    def rt(self):
        return self.gas_constant * self.temperature_k


def aperture(angle_degrees):
    """Reduced ball-valve open-area fraction from its actual shaft encoder."""
    if not math.isfinite(angle_degrees) or not 0 <= angle_degrees <= 90:
        raise ValueError("Actual valve angle must be in 0..90 degrees")
    return math.sin(math.radians(angle_degrees)) ** 2


def nozzle_per_area(upstream, downstream, p):
    if upstream <= 0 or upstream == downstream:
        return 0.0
    ratio = downstream / upstream
    critical = (2 / (p.gamma + 1)) ** (p.gamma / (p.gamma - 1))
    if ratio <= critical:
        factor = math.sqrt(p.gamma / p.rt) * (2 / (p.gamma + 1)) ** ((p.gamma + 1) / (2 * (p.gamma - 1)))
    else:
        factor = math.sqrt(
            max(
                0.0,
                2
                * p.gamma
                / (p.rt * (p.gamma - 1))
                * (ratio ** (2 / p.gamma) - ratio ** ((p.gamma + 1) / p.gamma)),
            )
        )
    return p.discharge_coefficient * upstream * factor


class Chamber:
    """Stores air mass; pressure is derived, never directly set during a step."""

    def __init__(self, parameters=None, initial_pa=None):
        self.parameters = p = parameters or Parameters()
        initial = p.ambient_pa if initial_pa is None else initial_pa
        if not math.isfinite(initial) or initial < 0:
            raise ValueError("Initial absolute pressure must be finite and nonnegative")
        self.volume = p.chamber_m3
        self.mass = initial * self.volume / p.rt

    @property
    def pressure(self):
        return self.mass * self.parameters.rt / self.volume

    @property
    def inward_door_force(self):
        """Signed scalar load for subsequent native application at door centre."""
        p = self.parameters
        return (p.ambient_pa - self.pressure) * p.door_area_m2

    def advance(self, dt, *, volume_m3, gaps_m, pump_angle_deg, vent_angle_deg, pump_on):
        p = self.parameters
        gaps = tuple(float(gap) for gap in gaps_m)
        if (
            not math.isfinite(dt)
            or dt <= 0
            or not math.isfinite(volume_m3)
            or volume_m3 <= 0
            or len(gaps) != p.segments
            or not all(math.isfinite(gap) for gap in gaps)
            or not isinstance(pump_on, bool)
        ):
            raise ValueError("Invalid measured geometry, time step or pump switch")
        speed = p.pump_m3_s * aperture(pump_angle_deg) * int(pump_on)
        vent_area = math.pi * p.vent_diameter_m**2 / 4 * aperture(vent_angle_deg)
        width = p.seal_perimeter_m / p.segments
        heights = [p.residual_channel_m + max(0.0, gap) for gap in gaps]
        slit_coefficients = [width * h**3 / (24 * p.viscosity_pa_s * p.seal_path_m * p.rt) for h in heights]
        areas = [width * h for h in heights]

        def flux(pressure):
            direction = 1 if p.ambient_pa >= pressure else -1
            upper, lower = max(p.ambient_pa, pressure), min(p.ambient_pa, pressure)
            sonic = nozzle_per_area(upper, lower, p)
            squared_difference = upper**2 - lower**2
            seal = direction * sum(
                min(c * squared_difference, area * sonic) for c, area in zip(slit_coefficients, areas)
            )
            vent = direction * vent_area * sonic
            pump = speed * pressure / p.rt
            return seal, vent, pump

        old_mass = self.mass
        # Backward Euler preserves nonnegative mass and stays stable for a
        # suddenly open door/vent. Volume change performs isothermal pV work.
        lower = 0.0
        upper = max(p.ambient_pa, old_mass * p.rt / volume_m3)
        for _ in range(44):
            pressure = (lower + upper) / 2
            seal, vent, pump = flux(pressure)
            residual = pressure * volume_m3 / p.rt - old_mass - dt * (seal + vent - pump)
            if residual > 0:
                upper = pressure
            else:
                lower = pressure
        pressure = (lower + upper) / 2
        seal, vent, pump = flux(pressure)
        self.volume = volume_m3
        self.mass = pressure * volume_m3 / p.rt
        return {
            "absolute_pressure_pa": self.pressure,
            "air_mass_kg": self.mass,
            "volume_m3": self.volume,
            "seal_inflow_kg_s": seal,
            "vent_inflow_kg_s": vent,
            "pump_outflow_kg_s": pump,
            "mass_balance_residual_kg": self.mass - old_mass - dt * (seal + vent - pump),
            "inward_door_force_N": self.inward_door_force,
        }
