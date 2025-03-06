package frc.robot.SwerveLib.Util;

public class ModuleLimits{

    public double maxDriveVelocity;
    public double maxDriveAcceleration;
    public double maxSteeringVelocity;

    public ModuleLimits(double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity){

        this.maxDriveVelocity = maxDriveVelocity;
        this.maxDriveAcceleration = maxDriveAcceleration;
        this.maxSteeringVelocity = maxSteeringVelocity;

    }
}
