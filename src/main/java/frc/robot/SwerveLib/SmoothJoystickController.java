package frc.robot.SwerveLib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SmoothJoystickController {

    private double kF;
    private ChassisSpeeds lastSpeeds = new ChassisSpeeds(0,0,0);

    public SmoothJoystickController(double kF){
        setSmoothingFactor(kF);
    }

    public void setSmoothingFactor(double newkF){
        if (newkF < 0 || newkF > 1) {
            throw new IllegalArgumentException("[SmoothJoystickController]: Smoothing Factor must be between 0 and 1!");
        }
        this.kF = newkF;
    }

    public ChassisSpeeds filter(ChassisSpeeds speeds){

        double vx = speeds.vxMetersPerSecond + (speeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) * (1 - kF);
        double vy = speeds.vyMetersPerSecond  + (speeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) * (1 - kF);
        double omega = speeds.omegaRadiansPerSecond + (speeds.omegaRadiansPerSecond - lastSpeeds.omegaRadiansPerSecond) * (1 - kF);

        lastSpeeds = new ChassisSpeeds(vx, vy, omega);

        return lastSpeeds;

    }
}
