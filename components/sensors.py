from navx import AHRS
from magicbot import feedback


class FROGGyro:
    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        # self.field_heading = 360-242
        self.gyro.reset()
        # self.gyro.setAngleAdjustment(-self.field_heading)

    @feedback(key="heading")
    def getHeading(self):
        # returns gyro heading +180 to -180 degrees
        return self.gyro.getYaw()

    def resetGyro(self):
        # sets yaw reading to 0
        self.gyro.reset()

    def execute(self):
        pass

    @feedback()
    def getAngle(self):
        return self.gyro.getAngle()

    def setAngle(self, angle):
        self.gyro.setAngleAdjustment(angle)

    @feedback()
    def getRadiansCCW(self):
        return math.radians(self.gyro.getYaw())

    @feedback()
    def getCompass(self):
        return self.gyro.getCompassHeading()

    @feedback()
    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()
