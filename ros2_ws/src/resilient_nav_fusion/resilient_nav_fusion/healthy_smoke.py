"""ROS-free contract for the Phase 8 healthy-path smoke observation."""

from dataclasses import dataclass, field

@dataclass(frozen=True)
class HeaderObservation:
    """The frame and ROS-time fields required from one observed message."""

    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    child_frame_id: str = ''

    @property
    def has_valid_stamp(self) -> bool:
        """Return whether the observation has a non-negative ROS timestamp."""
        return self.stamp_sec >= 0 and 0 <= self.stamp_nanosec < 1_000_000_000


@dataclass
class HealthySmokeObserver:
    """Require continuous nominal fusion inputs and adaptive odometry output."""

    required_samples: int = 3
    wheel_samples: list[HeaderObservation] = field(default_factory=list)
    imu_samples: list[HeaderObservation] = field(default_factory=list)
    adaptive_samples: list[HeaderObservation] = field(default_factory=list)
    nominal_status_samples: int = 0
    failures: list[str] = field(default_factory=list)

    def __post_init__(self):
        if self.required_samples < 1:
            raise ValueError('required_samples must be at least one')

    def observe_wheel(self, header: HeaderObservation) -> None:
        """Record an adapter wheel measurement under the frozen odom frame."""
        self._observe(
            self.wheel_samples,
            header,
            expected_frame='odom',
            label='wheel',
        )

    def observe_imu(self, header: HeaderObservation) -> None:
        """Record an adapter IMU measurement with a valid source frame."""
        self._observe(self.imu_samples, header, expected_frame=None, label='imu')

    def observe_adaptive(self, header: HeaderObservation) -> None:
        """Record adaptive odometry under the fixed frame ownership contract."""
        self._observe(
            self.adaptive_samples,
            header,
            expected_frame='odom',
            expected_child_frame='base_footprint',
            label='adaptive',
        )

    def observe_fusion_state(self, state: int) -> None:
        """Count only explicit nominal decisions, never inferred health."""
        if state == 1:  # FusionStatus.NOMINAL, kept ROS-free intentionally.
            self.nominal_status_samples += 1

    @property
    def passed(self) -> bool:
        """Return true only after all streams are continuously nominal and valid."""
        return (
            not self.failures
            and len(self.wheel_samples) >= self.required_samples
            and len(self.imu_samples) >= self.required_samples
            and len(self.adaptive_samples) >= self.required_samples
            and self.nominal_status_samples >= self.required_samples
        )

    def summary(self) -> dict[str, int | list[str]]:
        """Return concise serializable evidence without source/truth metadata."""
        return {
            'wheel_samples': len(self.wheel_samples),
            'imu_samples': len(self.imu_samples),
            'adaptive_samples': len(self.adaptive_samples),
            'nominal_status_samples': self.nominal_status_samples,
            'failures': list(self.failures),
        }

    def _observe(
        self,
        samples: list[HeaderObservation],
        header: HeaderObservation,
        *,
        expected_frame: str | None,
        label: str,
        expected_child_frame: str | None = None,
    ) -> None:
        if not header.has_valid_stamp:
            self.failures.append(f'{label}_invalid_stamp')
            return
        if expected_frame is not None and header.frame_id != expected_frame:
            self.failures.append(f'{label}_frame_mismatch')
            return
        if (
            expected_child_frame is not None
            and header.child_frame_id != expected_child_frame
        ):
            self.failures.append(f'{label}_child_frame_mismatch')
            return
        samples.append(header)
