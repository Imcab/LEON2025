package frc.robot.SwerveLib.Tidal.TidalUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class TidalUtil {
    
    public static Field getField2025(){
        return new Field(Units.inchesToMeters(690.876), Units.inchesToMeters(317));
    }

    public static boolean isBlue() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    }

    public static Coordinate pose2dToCoordinate(Pose2d pose){
        return new Coordinate(pose);
    }

    public static Domain fastDomain(double x,double y){
        return new Domain(x, y);
    }

    public static Coordinate fastCoordinate(double x,double y){
        return new Coordinate(x, y);
    }

    /**
     * Flips a coordinate to a RED Alliance
     * @param coordinate the coordinate to flip
     * @return the coordinate flipped
     */
    public static Coordinate coordinateFlip(Coordinate coordinate){

        Translation2d fixedXY =
         new Translation2d(TidalUtil.getField2025().getLength() - coordinate.inX(), TidalUtil.getField2025().getWidth() - coordinate.inY());
        Rotation2d fixedAngle = coordinate.toPose2d().getRotation().rotateBy(Rotation2d.kPi);

        Coordinate fixedCoordinate = new Coordinate(fixedXY.getX(), fixedXY.getY(), fixedAngle.getDegrees());

        return fixedCoordinate;
    }

    /**
     * Flips the coordinate inside the grid
     * @param grid the grid to flip
     * @return the grid flipped 
     */
    public static Grid gridFlip(Grid grid){
        return new Grid(coordinateFlip(grid.asCoordinate()), grid.getTolerance());
    }

    

    

}
