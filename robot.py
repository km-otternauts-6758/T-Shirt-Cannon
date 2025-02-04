#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# from msilib.schema import Component
from xml.etree.ElementTree import PI
from ntcore import NetworkTableInstance
import rev
from wpilib.cameraserver import CameraServer
import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
from dataclasses import dataclass, fields
from wpilib import DutyCycle, SmartDashboard
from rev import CANSparkMax
from networktables import NetworkTables
from components.limelight import LimeLight
from components.drivetrain import Drivetrain
from components import motorgroup
import math
from wpimath.controller import PIDController
from wpilib import DoubleSolenoid, Solenoid, PneumaticsModuleType
import phoenix5
from wpimath.controller import PIDController

kLEDBuffer = 60


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""

        # Joystick & Xbox Controller
        self.joystick = wpilib.Joystick(0)
        self.driveStick = wpilib.XboxController(1)

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Hex Encoder and pdp
        self.dutyCycle = wpilib.DutyCycle(wpilib.DigitalInput(1))
        self.pdp = wpilib.PowerDistribution()

        # Networktables and Limelight Init
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getTable("limelight-kmrobot")

        # Drivetrain
        self.robotDrive = Drivetrain()

        # Shooting Solenoid // Single Solenoid
        self.shoot = wpilib.Solenoid(
            moduleType=wpilib.PneumaticsModuleType.CTREPCM, channel=1
        )

        # Barrel Solenoid // Double Solenoid
        self.barrel = DoubleSolenoid(
            moduleType=PneumaticsModuleType.CTREPCM, forwardChannel=0, reverseChannel=3
        )

        # Kick Solenoid // Double Solenoid
        self.kick = DoubleSolenoid(
            moduleType=PneumaticsModuleType.CTREPCM, forwardChannel=4, reverseChannel=7
        )

        # Cannon Arm
        self.arm = phoenix5.WPI_TalonSRX(1)

        # Camera
        wpilib.CameraServer.launch()

        # Limelight
        # self.limelight = LimeLight()
        # self.limelight.robotInit()
        # self.ty = self.limelight.getEntry("ty")
        # self.limelight.putValue("priorityid", 4)

        # LED Lights
        # Must be a PWM
        self.led = wpilib.AddressableLED(9)
        # LED Data
        self.ledData = [wpilib.AddressableLED.LEDData() for _ in range(kLEDBuffer)]
        self.rainbowFirstPixelHue = 0
        self.led.setLength(kLEDBuffer)

        # Set the data
        self.led.setData(self.ledData)
        self.led.start()

    # Motor
    def setMotors(self, forward: float, turn: float):
        self.robotDrive.drive(forward, turn)

    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        # Calculates the output of the PID algorithm based on the sensor reading
        # and sends it to a motor
        pass

    def robotPeriodic(self) -> None:
        # Fill the LED buffer with rainbow
        self.rainbow()

        # Set the LEDs
        self.led.setData(self.ledData)
        pass

    def teleopInit(self) -> None:
        self.barrel.set(DoubleSolenoid.Value.kForward)
        self.kick.set(DoubleSolenoid.Value.kReverse)

    def teleopPeriodic(self) -> None:
        # Shoots The Cannon, Button A
        self.shoot.set(self.driveStick.getRawButton(1))
        # Rumble
        if self.shoot.set(self.driveStick.getRawButton(1)):
            self.driveStick.setRumble(self.driveStick.RumbleType.kBothRumble, 1)
        else:
            self.driveStick.setRumble(self.driveStick.RumbleType.kBothRumble, 0)

        # Controls The Barrel, Front Left Bumper
        if self.driveStick.getRawButtonPressed(5):
            self.barrel.toggle()
        # Ejects Cartridge, Currently X But Can Be Changed
        if self.driveStick.getRawButtonPressed(3):
            self.kick.toggle()

        # Controls The Cannon Arm
        self.arm.set(self.driveStick.getRawAxis(5))

        # Camera Server
        wpilib.CameraServer.launch("vision.py:main")
        CameraServer.is_alive()

        self.robotDrive.drive(
            -self.driveStick.getRawAxis(1), -self.driveStick.getRawAxis(0)
        )

    # Align w/ Apriltag
    # if self.driveStick.getAButton():
    #     if self.limelight.getNumber("tx", 0) < -4.5:
    #         self.setMotors(.1, .4)

    #     elif self.limelight.getNumber("tx", 0) > 4.5:
    #         self.setMotors(.1, -.4)

    #     elif self.limelight.getNumber("tx", 0) >= -1 and self.limelight.getNumber("tx", 0) <= 1:
    #         self.setMotors(0,0)

    # Rainbow Lights
    def rainbow(self):
        # For every pixel
        for i in range(kLEDBuffer):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = (self.rainbowFirstPixelHue + (i * 180 / kLEDBuffer)) % 180

        # Set the value
        self.ledData[i].setHSV(int(hue), 255, 128)

        # Increase by to make the rainbow "move"
        self.rainbowFirstPixelHue += 3

        # Check bounds
        self.rainbowFirstPixelHue %= 180
