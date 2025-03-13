package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BulukLib.Vision.Limelight;

public class Vision {
    public Limelight limelight;
    //private boolean loop_ONCE = false;
    private double distanceFromLimelightToGoalMeters;

    public Vision(){

        limelight = new Limelight();

        SmartDashboard.putData("LimelightBuluk", limelight);

    }

    public void periodic(){

        /*loop_ONCE = false;

        if (!loop_ONCE) {
            loop_ONCE = true;

        }*/

    }

    public double getDistanceToTargetMeters(){
        return distanceFromLimelightToGoalMeters;
    }
    public double aim(){
        return limelight.aimAngular();
    }
    public double range(){
        return limelight.rangeForward();
    }
    public double translation(){
        return limelight.translation();
    }

}
