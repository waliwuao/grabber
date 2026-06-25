"""Position-error PID used as an outer loop that commands X velocity."""

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class PositionPidConfig:
    kp: float = 0.2
    ki: float = 0.0
    kd: float = 0.02
    maximum_speed_mps: float = 0.1
    minimum_speed_mps: float = 0.02
    integral_limit_m_s: float = 0.05
    deadband_m: float = 0.005
    derivative_filter: float = 0.7


class PositionPid:
    """Convert position error in metres to a bounded velocity in m/s."""

    def __init__(self, config: PositionPidConfig) -> None:
        if config.maximum_speed_mps <= 0.0:
            raise ValueError('maximum_speed_mps must be positive')
        if config.minimum_speed_mps < 0.0:
            raise ValueError('minimum_speed_mps cannot be negative')
        if config.minimum_speed_mps >= config.maximum_speed_mps:
            raise ValueError('minimum_speed_mps must be less than maximum_speed_mps')
        if config.integral_limit_m_s < 0.0:
            raise ValueError('integral_limit_m_s cannot be negative')
        if config.deadband_m < 0.0:
            raise ValueError('deadband_m cannot be negative')
        if not 0.0 <= config.derivative_filter < 1.0:
            raise ValueError('derivative_filter must be in [0, 1)')
        self.config = config
        self.reset()

    def reset(self) -> None:
        self._integral_m_s = 0.0
        self._previous_error_m = None
        self._filtered_derivative_mps = 0.0

    @property
    def integral_m_s(self) -> float:
        return self._integral_m_s

    def update(self, error_m: float, dt_s: float) -> float:
        if abs(error_m) <= self.config.deadband_m:
            self.reset()
            return 0.0

        derivative_mps = 0.0
        if self._previous_error_m is not None and dt_s > 1e-6:
            raw_derivative = (
                error_m - self._previous_error_m
            ) / dt_s
            alpha = self.config.derivative_filter
            derivative_mps = (
                alpha * self._filtered_derivative_mps
                + (1.0 - alpha) * raw_derivative
            )
        self._filtered_derivative_mps = derivative_mps

        candidate_integral = self._integral_m_s
        if dt_s > 1e-6:
            candidate_integral = clamp(
                self._integral_m_s + error_m * dt_s,
                -self.config.integral_limit_m_s,
                self.config.integral_limit_m_s,
            )

        unsaturated = (
            self.config.kp * error_m
            + self.config.ki * candidate_integral
            + self.config.kd * derivative_mps
        )
        output = clamp(
            unsaturated,
            -self.config.maximum_speed_mps,
            self.config.maximum_speed_mps,
        )

        if output != 0.0 and abs(output) < self.config.minimum_speed_mps:
            output = self.config.minimum_speed_mps if output > 0.0 else -self.config.minimum_speed_mps

        # Conditional integration: do not wind up farther into saturation.
        if (
            output == unsaturated
            or output * error_m < 0.0
        ):
            self._integral_m_s = candidate_integral

        self._previous_error_m = error_m
        return output
