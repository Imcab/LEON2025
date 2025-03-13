package frc.robot.SwerveLib.Tidal.TidalUtils;

public class Field {
    
    private double length;
    private double width;

    public Field(double length, double width){
        this.length = length;
        this.width = width;
    }

    public double getLength(){
        return length;
    }

    public double getWidth(){
        return width;
    }
}
