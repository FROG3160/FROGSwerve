#!/usr/bin/env python3
from ctre import WPI_CANCoder, WPI_TalonFX
from magicbot import feedback
import wpilib
import math
from components.drivetrain import SwerveModule, SwerveChassis
from wpimath.geometry import Translation2d
from components.driverstation import FROGStick
from components.sensors import FROGGyro
from components.common import Rescale


# robot characteristics
# we are specifying inches and dividing by 12 to get feet,
# giving us values that can be used with the fromFeet method
# to get a correct Translation2d object
trackwidth = 27.75 / 12  # feet between wheels side to side
wheelbase = 21.75 / 12  # feet between wheels front to back
kDeadzone = 0.2
joystickAxisDeadband = Rescale((-1, 1), (-1, 1), 0.15)
joystickTwistDeadband = Rescale((-1, 1), (-1, 1), 0.2)


class FROGbot(magicbot.MagicRobot):
    """
    Initialize components here.
    """

    gyro: FROGGyro
    swerveChassis: SwerveChassis

    swerveFrontLeft: SwerveModule
    swerveFrontRight: SwerveModule
    swerveBackLeft: SwerveModule
    swerveBackRight: SwerveModule

    def createObjects(self):
        """Create motors and inputs"""
        # Swerve drive motors
        self.swerveFrontLeft_drive = WPI_TalonFX(11)
        self.swerveFrontRight_drive = WPI_TalonFX(12)
        self.swerveBackLeft_drive = WPI_TalonFX(13)
        self.swerveBackRight_drive = WPI_TalonFX(14)
        # Swerve steer motors
        self.swerveFrontLeft_steer = WPI_TalonFX(21)
        self.swerveFrontRight_steer = WPI_TalonFX(22)
        self.swerveBackLeft_steer = WPI_TalonFX(23)
        self.swerveBackRight_steer = WPI_TalonFX(24)
        # Swerve steer encoders (canifier)
        self.swerveFrontLeft_encoder = WPI_CANCoder(31)
        self.swerveFrontRight_encoder = WPI_CANCoder(32)
        self.swerveBackLeft_encoder = WPI_CANCoder(33)
        self.swerveBackRight_encoder = WPI_CANCoder(34)
        # Swerve module locations
        # TODO: move to swerveChassis?
        self.swerveFrontLeft_location = Translation2d.fromFeet(
            wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveFrontRight_location = Translation2d.fromFeet(
            wheelbase / 2,
            -trackwidth / 2,
        )
        self.swerveBackLeft_location = Translation2d.fromFeet(
            -wheelbase / 2,
            trackwidth / 2,
        )
        self.swerveBackRight_location = Translation2d.fromFeet(
            -wheelbase / 2,
            -trackwidth / 2,
        )

        self.swerveFrontLeft_steerOffset = 13.008
        self.swerveFrontRight_steerOffset = 171.914
        self.swerveBackLeft_steerOffset = 22.764
        self.swerveBackRight_steerOffset = -43.242

        # config for saitek joystick
        # self.driveStick = FROGStick(0, 0, 1, 3, 2)
        # config for Logitech Extreme 3D

        self.field = wpilib.Field2d()
        # simulation already places field data in SmartDashboard
        # so we need to keep this from overwriting that data
        # during simulation
        if not self.isSimulation():
            wpilib.SmartDashboard.putData(self.field)

    def autonomousInit(self):
        pass

    def teleopInit(self):
        """Called when teleop starts; optional"""
        self.swerveChassis.enable()

    def teleopPeriodic(self):
        """Called on each iteration of the control loop"""

        xOrig = joystickAxisDeadband(self.driveStick.getFieldForward())
        yOrig = joystickAxisDeadband(self.driveStick.getFieldLeft())
        tOrig = joystickTwistDeadband(self.driveStick.getFieldRotation())

        # Get driver controls
        vX, vY, vT = (
            math.copysign(xOrig**2, xOrig),
            math.copysign(yOrig**2, yOrig),
            math.copysign(tOrig**2, tOrig),
        )
        if vX or vY or vT:
            self.swerveChassis.field_oriented_drive(vX, vY, vT)

        if self.driveStick.getTrigger():
            self.gyro.resetGyro()
            self.swerveChassis.field_oriented_drive(0, 0, 0)

    def testInit(self):
        """Called when test mode starts; optional"""
        pass

    def testPeriodic(self):
        """Called on each iteration of the control loop"""
        pass


if __name__ == "__main__":
    wpilib.run(FROGbot)
