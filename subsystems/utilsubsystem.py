import commands2
from wpilib import PowerDistribution

#This sets the parameters for when the warnings appear on Smartdashboard
class UtilSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

        self.current_warning_high_drive_motor = 50
        self.current_warning_low_drive_motor = 10
        self.current_warning_high_steer_motor = 30
        self.current_warning_low_steer_motor = 10


    def toggle_channel(self, on: bool) -> None:
        self.pdh.setSwitchableChannel(on)