package frc.robot.SwerveLib.Tidal;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import frc.robot.SwerveLib.Tidal.TidalUtils.Coordinate;
import frc.robot.SwerveLib.Tidal.TidalUtils.Domain;
import frc.robot.SwerveLib.Tidal.TidalUtils.TidalUtil;

public class TidalConfig {
    
    public static class pathGains{

        public static PathConstraints mdefault = new PathConstraints(3.0, 3.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mSlow = new PathConstraints(2.7, 2.5,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mFast = new PathConstraints(4.5, 4.0,
        Units.degreesToRadians(540),
        Units.degreesToRadians(720));

        public static PathConstraints mCustom(PathConstraints gains){
            return gains;
        }
    }
    public static class feeders {

        public static Domain leftFeederDomainBlue = new Domain(4.3, 8);
        public static Domain leftFeederDomainRed = new Domain(0, 4.2);
        public static Coordinate leftFeederAlign = new Coordinate(1.12, 6.96, 124.88);
        public static Coordinate rightFeederAlign = new Coordinate(1.17, 1.04, -125.75);

        public static Coordinate leftFeederAlignRED = TidalUtil.coordinateFlip(leftFeederAlign);
        public static Coordinate rightFeederAlignRED = TidalUtil.coordinateFlip(rightFeederAlign);
            
    }

}
    
    

    
        


