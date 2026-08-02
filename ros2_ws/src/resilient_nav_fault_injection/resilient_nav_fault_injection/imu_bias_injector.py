from resilient_nav_fault_injection.imu_fault_injector import (
    bias_main,
    ImuFaultInjector,
)


class ImuBiasInjector(ImuFaultInjector):
    """Compatibility node for the original IMU bias injector command."""

    def __init__(self):
        super().__init__(node_name='imu_bias_injector')


def main(args=None):
    bias_main(args=args)
