import math
from wpimath.geometry import Translation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import feetToMeters


class OperatorConstants:
    kDriverControllerPort = 0
    kOperatorControllerPort = 1

class DriveConstants:
   
   wheel_diameter = 0.1016  # meters
   wheel_circumference = wheel_diameter * math.pi
   #these gear ratios are based on MK4i L2 swerve modules
   drive_gear_ratio = 6.12
   angle_gear_ratio = 21.43

   d_position_conversion_factor = wheel_circumference / drive_gear_ratio
   d_velocity_conversion_factor = d_position_conversion_factor / 60  # Conversion from rot/min to m/s
   kMaxSpeed = 5.00  # Set max speed in m/s 10
   # kMaxSpeedTeleop = kMaxSpeed * 1.75
   kMaxAngularSpeed = 7  # Set max rotation speed rot/s 20
   # kMaxAngularSpeedTeleop = kMaxAngularSpeed * 1.75
   kGyroReversed = False

#Locations of the swerve module measured from the center of the robot maybe?
   m_FL_location = Translation2d(0.52705 / 2, 0.52705 / 2)
   m_FR_location = Translation2d(0.52705 / 2, -0.52705 / 2)
   m_BL_location = Translation2d(-0.52705 / 2, 0.52705 / 2)
   m_BR_location = Translation2d(-0.52705 / 2, -0.52705 / 2)
   m_kinematics =  SwerveDrive4Kinematics(m_FL_location, m_FR_location, m_BL_location, m_BR_location)
   
   snap_controller_PID = [0.051, 0, 0]  # 0.01
   turret_controller_PID = [0.08, 0, 0.0001]
   clt_controller_PID = [0.04, 0, 0]  # 11
   drive_controller_PID = [0.0020645, 0, 0]
   azimuth_controller_PID = [0.01, 0, 1.5]
   drive_controller_FF = [0.18 / 12, 2.35, 0.44]  # n/a, 2.35, 0.44
   
   closed_loop_ramp = 0.5  # was 0.1
   open_loop_ramp = 0.5   # was 0.25
   drive_current_limit = 50  # was 60
   azimuth_current_limit = 20  # Was 30
   
   balance_PID = [0.01, 0, 0]
   
   slew_rate_drive = 130  # 50  # 110
   slew_rate_turn = 0
   
   ob_drive_pid = [0.5, 0, 0, 1 / feetToMeters(16.6)]
   ob_steer_pid = [0, 0, 0, 0] #P was at 1


#This sets CAN IDs for motors, encoder ID, and offsets in degrees
class ModuleConstants: 
    fr_drive_id=12
    fr_turn_id=11
    fr_encoder_id=3
    fr_zero_offset=281.1
 

    fl_drive_id=18
    fl_turn_id=17
    fl_encoder_id=2
    fl_zero_offset=93.8 
   
    br_drive_id=14
    br_turn_id=13
    br_encoder_id=0
    br_zero_offset=236.7
   
    bl_drive_id=16
    bl_turn_id=15
    bl_encoder_id=1
    bl_zero_offset=241.2 

