package frc.robot.BulukLib.Vision;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.BulukLib.Vision.LimelightHelpers.LimelightResults;
import frc.robot.BulukLib.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.BulukLib.Vision.VisionConfig.limelight;
import edu.wpi.first.wpilibj.RobotController;

public class Limelight implements Sendable{

    public static final String kName = VisionConfig.limelight.name;
    private PoseEstimate mt2 = new PoseEstimate();

    @Override
    public void initSendable(SendableBuilder builder){

        builder.addBooleanProperty("TV", ()-> LimelightHelpers.getTV("limelight-buluk"), null);
        builder.addDoubleProperty("TX", ()-> LimelightHelpers.getTX("limelight-buluk"), null);
        builder.addDoubleProperty("TA", ()-> LimelightHelpers.getTA("limelight-buluk"), null);
        builder.addDoubleProperty("TY", ()-> LimelightHelpers.getTY("limelight-buluk"), null);
        builder.addBooleanProperty("Connected", ()->isConnected(), null);
        builder.addDoubleProperty("ForwardSpeed", ()-> rangeForward(), null);
        builder.addDoubleProperty("AngularSpeed", ()-> aimAngular(), null);
        builder.addDoubleProperty("TranslationSpeed", ()-> translation(), null);

    }

    public Limelight(){
    }

    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink(kName);
    }
    public boolean isConnected(){
        double lastUpdate = RobotController.getFPGATime() - LimelightHelpers.getLatency_Pipeline("limelight.buluk")/1000;
        return lastUpdate < 250;
    }
    public void LedOn(){
        LimelightHelpers.setLEDMode_ForceOn(kName);
    }

    public void LedOff(){
        LimelightHelpers.setLEDMode_ForceOff(kName);
    }

    public double ty(){
        return LimelightHelpers.getTY(kName);
    }

    public double tagPercentage(){
        return LimelightHelpers.getTA(kName);
    }

    public double tx(){
        return LimelightHelpers.getTX(kName);
    }

    public boolean hasTarget(){
        return LimelightHelpers.getTV(kName) && isConnected();
    }

    public int targets(){
        return LimelightHelpers.getTargetCount(kName);
        
    }
 
    public void update(){

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(kName);
        }

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kName);
    }

    public boolean hasResults(){

        return mt2.tagCount > 0 && isConnected();
    }

    public boolean hasID(int ID){
        
        LimelightResults results = LimelightHelpers.getLatestResults(kName);

        return hasTarget() && results.valid && results.pipelineID == ID;
    }

    public PoseObservation getObservation(){
        return new PoseObservation(mt2.pose, mt2.timestampSeconds, limelight.trust);
    }
    
    public double rangeForward(){

        // simple proportional ranging control with Limelight's "ty" value
        // this works best if your Limelight's mount height and target mount height are different.
        // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"

        double method = limelight.useTAforRange ? tagPercentage(): ty();

        double targetingForwardSpeed = limelight.useTAforRange?limelight.forwardKp / method: method * limelight.forwardKp;
    
        targetingForwardSpeed *= limelight.TrackMaxSpeed;
        targetingForwardSpeed *= limelight.forwardCoefficient;

        return -targetingForwardSpeed;
    }

    public double translation(){

        double targetXvel = tx() * limelight.translationKp;

        targetXvel *= limelight.TrackMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetXvel *= limelight.translationCoeficient;

        return targetXvel;
    }



    


    public double aimAngular(){
        
        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = tx() * limelight.angularKp;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= limelight.TrackMaxAngularSpeed;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= limelight.aimCoefficient;

        return targetingAngularVelocity;
    }

}
