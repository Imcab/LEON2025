package frc.robot.SwerveLib.Tidal.TidalUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Coordinate {

    private double x;
    private double y;
    private double degrees;
    
    //Represents coordinates in blue alliance
    public Coordinate(double x, double y, double degrees){
        this.x = x;
        this.y = y;
        this.degrees = degrees;
    }

    public Coordinate(Pose2d frame){
        this.x = frame.getX();
        this.y = frame.getY();
        this.degrees = frame.getRotation().getDegrees();
    }

    public Coordinate(double x, double y){
        this.x = x;
        this.y = y;
        this.degrees = 0;
    }

    public Pose2d toPose2d(){
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    public double inX(){
        return x;
    }

    public double inY(){
        return y;
    }
    
    public double degrees(){
        return degrees;
    }

    public boolean equals(Coordinate other, double tolerance){

        Grid mGrid = new Grid(other, tolerance);

        return mGrid.atGrid(other);

    }

    private double squareOf(double x){
        return x * x;
    }

    public double distanceToCoordinate(Coordinate other){
        return Math.sqrt(squareOf(x - other.inX()) + squareOf(y - other.inY()));
    }

    public double distanceToX(double pointX){
        return Math.abs(x - pointX);
    }

    public double distanceToY(double pointY){
        return Math.abs(y - pointY);
    }


}
