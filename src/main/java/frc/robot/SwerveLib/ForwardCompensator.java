package frc.robot.SwerveLib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ForwardCompensator{

    private double kC;
    private double thetaDeadband;

    public ForwardCompensator(double kC, double thetaDeadband){
        this.thetaDeadband = thetaDeadband;
        setCorrectionFactor(kC);
    }

    public void setCorrectionFactor(double newKc){
        if (newKc < 0 || newKc > 1) {
            throw new IllegalArgumentException("[ForwardCompensator]: Correction Factor must be between 0 and 1!");
        }
        this.kC = newKc;
    }

    public ChassisSpeeds compensate(ChassisSpeeds desiredSpeeds, double thetaJoystickInput) {
        double modifiedOmega = desiredSpeeds.omegaRadiansPerSecond;

        //If moving mostly forward, compensate for unintended rotation
        if (Math.abs(desiredSpeeds.vxMetersPerSecond) > Math.abs(desiredSpeeds.vyMetersPerSecond) && Math.abs(thetaJoystickInput) < thetaDeadband) {
            modifiedOmega *= kC;
        }

        return new ChassisSpeeds(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, modifiedOmega);
    }
}
