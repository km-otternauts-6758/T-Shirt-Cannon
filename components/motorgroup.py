import rev
import wpilib
import wpimath.units
import math

kWheelDiameter = wpimath.units.inchesToMeters(6)
kWheelCircumference = kWheelDiameter * math.pi
kGearRatio = 1 # TODO: Update gear ratio
kConversionFactor = kWheelCircumference / kGearRatio

class MotorGroup:
    def __init__(self, forwardMotorChannel: int, rearMotorChannel: int) -> None:
        self.forwardMotor = rev.CANSparkMax(forwardMotorChannel, rev.CANSparkMax.MotorType.kBrushless)
        self.rearMotor = rev.CANSparkMax(rearMotorChannel, rev.CANSparkMax.MotorType.kBrushless)

        self.motorControllerGroup = wpilib.MotorControllerGroup(self.forwardMotor, self.rearMotor)

        self.forwardEncoder = self.forwardMotor.getEncoder()
        self.rearEncoder = self.rearMotor.getEncoder()

        self.forwardEncoder.setPositionConversionFactor(kConversionFactor)
        self.rearEncoder.setPositionConversionFactor(kConversionFactor)

        self.forwardEncoder.setVelocityConversionFactor(kConversionFactor / 60)
        self.rearEncoder.setVelocityConversionFactor(kConversionFactor / 60)

        self.forwardMotor.clearFaults()
        self.rearMotor.clearFaults()

    def getMotorControllerGroup(self) -> wpilib.MotorControllerGroup:
        return self.motorControllerGroup
    
    def getPosition(self) -> float:
        return (self.forwardEncoder.getPosition() + self.rearEncoder.getPosition()) / 2
    
    def getVelocity(self) -> float:
        return (self.forwardEncoder.getVelocity() + self.rearEncoder.getVelocity()) / 2
    
    def setInverted(self, isInverted: bool):
        self.motorControllerGroup.setInverted(isInverted)
