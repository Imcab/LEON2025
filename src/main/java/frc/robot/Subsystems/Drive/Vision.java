package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BulukLib.Vision.Limelight;

public class Vision {
    public Limelight limelight;
    public boolean Limerequest = false;
    private boolean loop_ONCE = false;

    public Vision(){

        limelight = new Limelight();

        SmartDashboard.putData("LimelightBuluk", limelight);

    }
    public void periodic(){

        loop_ONCE = false;

        if (!loop_ONCE) {
            
            limelight.update();

            loop_ONCE = true;

        }

        

    }
 
    public void limeRequest(boolean toggle){
        Limerequest = toggle;
    }
    public boolean limeIsRequested(){
        return Limerequest;
    }
    public boolean lime_hasResults(){
        return limelight.hasResults();
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
