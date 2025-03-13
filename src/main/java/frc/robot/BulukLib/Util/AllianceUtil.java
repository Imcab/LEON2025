package frc.robot.BulukLib.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AllianceUtil{

    public static boolean isBlue(){
        return DriverStation.getAlliance().
        orElse(Alliance.Blue) == Alliance.Blue;
    }
    
}
