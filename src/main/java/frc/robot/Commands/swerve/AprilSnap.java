package frc.robot.Commands.swerve;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BulukLib.Vision.LimelightHelpers;
import frc.robot.Subsystems.Drive.swerve;

public class AprilSnap extends Command{
    private swerve drive;
    private DoubleSupplier y;

    public AprilSnap(swerve drive, DoubleSupplier y){   
        this.drive = drive;
        this.y = y;

        addRequirements(drive);
    }

    @Override
    public void initialize(){

        LimelightHelpers.setLEDMode_ForceOn("limeligh-buluk");
    }

    @Override
    public void execute(){

        drive.centerWithApriltag(y.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        LimelightHelpers.setLEDMode_ForceOff("limelight-buluk");
        drive.stopWithX();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
