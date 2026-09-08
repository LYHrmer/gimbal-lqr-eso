"""Independent, static torque/speed approximation for synthetic sensitivity runs.

This is NOT a manufacturer measured curve, a thermal model, or a drive interface.
All generic APIs require explicit parameters; no hardware defaults are provided.

DM4310_24V_REFERENCE is a named SIMULATION reference based on the DM-J4310-2EC
V1.1 motor manual, document V1.0 (2023-11-16), printed page 13, checked 2026-09-08:
https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf#page=13
The listed 24 V specifications include rated torque 3 Nm at rated speed 120 rpm,
peak torque 7 Nm, and no-load maximum speed 200 rpm. The manual's peak value is
NOT a confirmed stall torque. We ASSUME a zero-speed intercept equal to 7 Nm,
then use two straight segments through (120 rpm, 3 Nm) and (200 rpm, 0 Nm).
Passing through a specified rated point does NOT make this a measured full
torque/speed curve or an identification of back-EMF or winding resistance.

The reference's 3 Nm current-related torque cap is a modeling approximation from
the rated specification. It does not imply unlimited continuous operation at
3 Nm at every speed. A peak sensitivity configuration can explicitly replace
the cap with 7 Nm; this module has no permissible peak-duration or heat model.

For torque and speed in the same direction (motoring), an optional rated point
selects a two-segment envelope; without it the general approximation remains
anchor * max(0, 1 - abs(speed) / no_load_speed). Both are constrained by the
independent current-related torque cap. Opposing torque (braking) uses
only that cap; regenerative bus voltage, supply absorption and braking hardware
constraints are not modeled. Overspeed suppresses further accelerating torque
while allowing current-limited braking. This bounds the requested equilibrium
torque; actuator lag, actual applied torque and plant dynamics belong to the
calling simulation, not this stateless function.
"""
from dataclasses import dataclass
import math
from numbers import Real


def _finite_real(name, value):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite real number")
    try:
        converted = float(value)
    except (OverflowError, ValueError) as error:
        raise ValueError(f"{name} must be a finite real number") from error
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be a finite real number")
    return converted


@dataclass(frozen=True)
class MotorEnvelope:
    """Explicit synthetic envelope parameters, all positive and finite.

    ``peak_torque_anchor_nm`` is an assumed zero-speed line intercept, not a
    manufacturer-confirmed stall torque. ``current_torque_limit_nm`` is a torque
    cap, NOT a current in amperes, and must not exceed the anchor. All speeds and
    torques refer to the same output shaft. Supply-voltage scaling is not inferred.
    The optional rated fields must be supplied together. Their speed must lie
    strictly between zero and no-load speed; torque must be positive and no larger
    than the zero-speed anchor. Neither field changes the braking approximation.
    """
    current_torque_limit_nm: float
    peak_torque_anchor_nm: float
    no_load_velocity_rad_s: float
    rated_velocity_rad_s: float | None = None
    rated_torque_nm: float | None = None

    def __post_init__(self):
        for name in ("current_torque_limit_nm", "peak_torque_anchor_nm",
                     "no_load_velocity_rad_s"):
            value = _finite_real(name, getattr(self, name))
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        if self.current_torque_limit_nm > self.peak_torque_anchor_nm:
            raise ValueError("current torque limit must not exceed peak torque anchor")
        if (self.rated_velocity_rad_s is None) != (self.rated_torque_nm is None):
            raise ValueError("rated velocity and torque must be specified together")
        if self.rated_velocity_rad_s is not None:
            rated_velocity = _finite_real("rated_velocity_rad_s", self.rated_velocity_rad_s)
            rated_torque = _finite_real("rated_torque_nm", self.rated_torque_nm)
            if not 0.0 < rated_velocity < self.no_load_velocity_rad_s:
                raise ValueError("rated velocity must be between zero and no-load velocity")
            if not 0.0 < rated_torque <= self.peak_torque_anchor_nm:
                raise ValueError("rated torque must be positive and no larger than peak anchor")
            object.__setattr__(self, "rated_velocity_rad_s", rated_velocity)
            object.__setattr__(self, "rated_torque_nm", rated_torque)


# Only this explicitly named simulation object contains DM4310 reference values.
DM4310_24V_REFERENCE = MotorEnvelope(
    current_torque_limit_nm=3.0,
    peak_torque_anchor_nm=7.0,
    no_load_velocity_rad_s=200.0 * math.tau / 60.0,
    rated_velocity_rad_s=120.0 * math.tau / 60.0,
    rated_torque_nm=3.0,
)


def torque_bounds(velocity_rad_s, parameters):
    """Return signed ``(negative_limit, positive_limit)`` for this shaft speed.

    At rest both directions use the current torque cap. At or above the no-load
    speed only the accelerating direction is zeroed. Raises ValueError for
    nonfinite inputs or an object other than a validated MotorEnvelope.
    """
    if not isinstance(parameters, MotorEnvelope):
        raise ValueError("parameters must be a MotorEnvelope")
    velocity = _finite_real("velocity_rad_s", velocity_rad_s)
    current_limit = parameters.current_torque_limit_nm
    speed = abs(velocity)
    if speed == 0.0:
        return -current_limit, current_limit
    # Compare before dividing: valid extreme finite speeds cannot overflow a ratio.
    if speed >= parameters.no_load_velocity_rad_s:
        motoring_limit = 0.0
    else:
        if parameters.rated_velocity_rad_s is None:
            envelope_limit = parameters.peak_torque_anchor_nm * (
                1.0 - speed / parameters.no_load_velocity_rad_s)
        elif speed <= parameters.rated_velocity_rad_s:
            fraction = speed / parameters.rated_velocity_rad_s
            # Anchor at the rated endpoint to avoid cancellation there when the
            # two torque scales differ greatly.
            envelope_limit = parameters.rated_torque_nm + (1.0 - fraction) * (
                parameters.peak_torque_anchor_nm - parameters.rated_torque_nm)
        else:
            fraction = ((speed - parameters.rated_velocity_rad_s) /
                        (parameters.no_load_velocity_rad_s - parameters.rated_velocity_rad_s))
            envelope_limit = parameters.rated_torque_nm * (1.0 - fraction)
        motoring_limit = min(current_limit, envelope_limit)
    if velocity > 0.0:
        return -current_limit, motoring_limit
    return -motoring_limit, current_limit


def actuator_limit(request_nm, velocity_rad_s, parameters):
    """Clip a finite signed torque request to the static synthetic envelope.

    This returns a modeled available equilibrium torque in Nm, not a measured
    motor torque. It never increases request magnitude or reverses its sign.
    Invalid inputs raise ValueError; callers should stop a simulation rather than
    silently replace invalid state with zero. No driving/enable action occurs.
    """
    request = _finite_real("request_nm", request_nm)
    lower, upper = torque_bounds(velocity_rad_s, parameters)
    if request == 0.0:
        return 0.0
    return min(max(request, lower), upper)
