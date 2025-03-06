package frc.robot.Subsystems.Drive;

import frc.robot.BulukLib.Vision.Limelight;
import frc.robot.BulukLib.Vision.PoseObservation;

public class Vision {
    public Limelight limelight;
    public boolean Limerequest = false;
    private boolean loop_ONCE = false;

    public Vision(){

        limelight = new Limelight();

    }

    public void periodic(){

        loop_ONCE = false;

        if (!loop_ONCE) {
            
            limelight.update();

            loop_ONCE = true;

        }

        

    }
 
    public PoseObservation observationLime(){
        return limelight.getObservation();
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

    public double forwardWithLimit(double limit){
        return limelight.limitForward(limit);
    }

    

}
