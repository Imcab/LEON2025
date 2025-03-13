package frc.robot.SwerveLib.Tidal.TidalUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Domain{

    private double min, max;
    /**
     * Represents a range zone from x to y value
     * @param min from min
     * @param max to max
     */
    public Domain(double min , double max){
        this.min = min;
        this.max = max;
    }
    /**
     * Represents a range zone from x to y value
     * @param xy the X and Y (min to max)
     */
    public Domain(Translation2d xy){
        this.min = xy.getX();
        this.max = xy.getY();
    }
    /**
     * Represents a range zone from x to y value
     * @param pose2d the pose in X and Y (min to max)
     */
    public Domain(Pose2d pose2d){
        this.min = pose2d.getX();
        this.max = pose2d.getY();
    }
    /**
     * Represents a range zone from x to y value
     * @param coordinate the range in X and Y (min to max)
     */
    public Domain(Coordinate coordinate){
        this.min = coordinate.inX();
        this.max = coordinate.inY();
    }

    public double minValue(){
        return min;
    }
    public double maxValue(){
        return max;
    }

    public boolean inRange(double value){
        return value >= min && value <= max;
    }
}

