import commands2
import wpilib.simulation
from commands2 import button
from constants import DriveConstants, OperatorConstants
from subsystems.tuningsubsystem import TuningSubsystem
from subsystems.utilsubsystem import UtilSubsystem
from subsystems.drivesubsystem import DriveSubsystem
from wpilib import SmartDashboard, SendableChooser, DriverStation, DataLogManager, LiveWindow, Timer
from commands.debug_mode import DebugMode
from commands.check_drivetrain import CheckDrive
from commands.switch_channel import SwitchPDHChannel
from helpers.pose_estimator import PoseEstimator
from helpers.custom_hid import CustomHID


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        tuning_setter = False
        self.timer = Timer()
        self.timer.start()
        if wpilib.RobotBase.isReal():
            print("Not a simulation, logging enabled!")
            DataLogManager.start()
            DriverStation.startDataLog(DataLogManager.getLog(), True)
        else:
            print("Simulated, logging disabled.")

        LiveWindow.disableAllTelemetry()

        # Instantiate subsystems using their constructors if tuning mode is disabled.
        if not tuning_setter:
            self.pose_estimator = PoseEstimator()
            self.robot_drive = DriveSubsystem(self.timer, self.pose_estimator)
            self.utilsys = UtilSubsystem()  # Only compatible with REV PDH at this time.

        # Setup driver & operator controllers.
        self.driver_controller_raw = CustomHID(OperatorConstants.kDriverControllerPort, "xbox")
        self.operator_controller_raw = CustomHID(OperatorConstants.kOperatorControllerPort, "xbox")
        DriverStation.silenceJoystickConnectionWarning(True)

        # Perform setup as normal, unless tuning mode is enabled.
        if not tuning_setter:
            self.robot_drive.setDefaultCommand(commands2.cmd.run(
                lambda: self.robot_drive.drive_2ok(
                    self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                    self.driver_controller_raw.get_axis_squared("RX", 0.06) * DriveConstants.kMaxAngularSpeed,
                    True
                ), self.robot_drive
            ))

        # Register commands for PathPlanner.
            self.registerCommands()

        # Setup for all event-trigger commands.
            self.configureTriggersDefault()


            SmartDashboard.putData("Debug Mode On", DebugMode(self.robot_drive, True))
            SmartDashboard.putData("Debug Mode Off", DebugMode(self.robot_drive, False))
            SmartDashboard.putData("Drive Subsystem", self.robot_drive)

        # Perform setup for when tuning mode is enabled.
        else:
            self.tuner = TuningSubsystem(True, True, 50)
            self.configureTuningMode()

    def configureTuningMode(self) -> None:

        button.Trigger(lambda: self.driver_controller_raw.get_button("A")).onTrue(
            commands2.cmd.run(lambda: self.tuner.id_ks_dt(0.0001), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("B")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.id_kv_dt(6), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("X")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.set_all_zero(), self.tuner)
        )
        button.Trigger(lambda: self.driver_controller_raw.get_button("Y")).onTrue(
            commands2.cmd.runOnce(lambda: self.tuner.reset_routines(), self.tuner)
        )

    def configureTriggersDefault(self) -> None:
        """Used to set up any commands that trigger when a measured event occurs."""
        button.Trigger(lambda: DriverStation.isTeleopEnabled()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))
        button.Trigger(lambda: DriverStation.isDSAttached()).onTrue(
            commands2.cmd.runOnce(lambda: self.robot_drive.set_alliance(), self.robot_drive))


        # Hold for Slow Mode, variable based on depth of Trigger.
        button.Trigger(lambda: self.driver_controller_raw.get_trigger("R", 0.05)).whileTrue(
            commands2.cmd.run(lambda: self.robot_drive.drive_slow(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis("RX", 0.06),
                True,
                self.driver_controller_raw.refine_trigger("R", 0.05, 0.8, 0.3)), self.robot_drive))

        # Press any direction on the D-pad to enable PID snap to that equivalent angle based on field orientation
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("N")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                180
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("S")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                0
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("E")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                60
            ), self.robot_drive))
        button.Trigger(lambda: self.driver_controller_raw.get_d_pad_pull("W")).toggleOnTrue(
            commands2.cmd.run(lambda: self.robot_drive.snap_drive(
                self.driver_controller_raw.get_axis_squared("LY", 0.06) * DriveConstants.kMaxSpeed,
                self.driver_controller_raw.get_axis_squared("LX", 0.06) * DriveConstants.kMaxSpeed,
                300
            ), self.robot_drive))
        
        # When the robot is enabled, turn on the Time-On Meter.
        button.Trigger(lambda: DriverStation.isEnabled()).onTrue(SwitchPDHChannel(True, self.utilsys))
        button.Trigger(lambda: DriverStation.isEnabled()).onFalse(SwitchPDHChannel(False, self.utilsys))

#def registerCommands(self):
        #NamedCommands.registerCommand("toggle_odo", ToggleOdo(self.vision_system))
        #NamedCommands.registerCommand("check_drive", CheckDrive(self.robot_drive, self.utilsys, self.timer))
        #NamedCommands.registerCommand("master_caution", MasterCaution(self.leds, self.timer))