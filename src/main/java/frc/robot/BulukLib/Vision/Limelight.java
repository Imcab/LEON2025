package frc.robot.BulukLib.Vision;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BulukLib.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.BulukLib.Vision.VisionConfig.limelight;
import edu.wpi.first.wpilibj.RobotController;

public class Limelight extends SubsystemBase{

    public static final String kName = VisionConfig.limelight.name;
    private PoseEstimate mt2 = new PoseEstimate();
    private double distanceFromLimelightToGoalMeters;

    @Override
    public void initSendable(SendableBuilder builder){

        builder.addBooleanProperty("TV", ()-> LimelightHelpers.getTV("limelight-buluk"), null);
        builder.addDoubleProperty("TX", ()-> LimelightHelpers.getTX("limelight-buluk"), null);
        builder.addDoubleProperty("TA", ()-> LimelightHelpers.getTA("limelight-buluk"), null);
        builder.addDoubleProperty("TY", ()-> LimelightHelpers.getTY("limelight-buluk"), null);
        builder.addDoubleProperty("ID", ()-> currentID(), null);
        builder.addBooleanProperty("Connected", ()->isConnected(), null);
        builder.addDoubleProperty("ForwardSpeed", ()-> rangeForward(), null);
        builder.addDoubleProperty("AngularSpeed", ()-> aimAngular(), null);
        builder.addDoubleProperty("TranslationSpeed", ()-> translation(), null);
        builder.addDoubleProperty("DistanceTarget", ()-> getDistanceToTarget(), null);

    }

    public Limelight(){

    }

    @Override
    public void periodic(){
        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-buluk");

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.1; 

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightCM = 19.0; 

        // distance from the target to the floor
        double goalHeightCM = 33.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        distanceFromLimelightToGoalMeters = ((goalHeightCM - limelightLensHeightCM) / Math.tan(angleToGoalRadians)) / 100;

    }

    public double getDistanceToTarget(){
        return distanceFromLimelightToGoalMeters;
    }


    public void blink(){
        LimelightHelpers.setLEDMode_ForceBlink(kName);
    }
    public boolean isConnected(){
        double lastUpdate = RobotController.getFPGATime() - LimelightHelpers.getLatency_Pipeline("limelight-buluk")/1000;
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
        return LimelightHelpers.getTX("limelight-buluk");
    }

    public boolean hasTarget(){
        return LimelightHelpers.getTV("limelight-buluk") && isConnected();
    }

    public int targets(){
        return LimelightHelpers.getTargetCount("limelight-buluk");
        
    }
 
    public void update(){

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight-buluk");
        }

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-buluk");
    }

    public boolean hasResults(){

        return mt2.tagCount > 0 && isConnected();
    }

    public double currentID(){
        return LimelightHelpers.getFiducialID("limelight-buluk");
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
