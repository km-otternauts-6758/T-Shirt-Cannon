import wpilib
from networktables import NetworkTables
import robotpy_apriltag


class LimeLight:
    def robotInit(self):    
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getTable("limelight-kmrobot")
        print(f"tv: {self.limelight.getEntry("tv")}")


    # def lighting(self):
    #     # print(f"ta: {self.limelight.getEntry("ta")}")
    #     # print(f"tv: {self.limelight.getEntry("tv")}")
    #     # print(self.limelight.getValue("ta", 0))
    #     if self.limelight.getNumber("ta", 0) > 1:
    #         self.limelight.putNumber("ledMode", 1)
    #     else:
    #         self.limelight.putNumber("ledMode", 3)
        
    def visible(self, key:str, defaultValue:float):
        self.limelight.getNumber(key, defaultValue)
