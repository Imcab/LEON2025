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

    private DoubleSupplier x;
    private DoubleSupplier y;
 
    public AprilSnap(swerve drive, DoubleSupplier x, DoubleSupplier y){   
        this.drive = drive;
        this.x = x;
        this.y = y;

        addRequirements(drive);
    }

    @Override
    public void initialize(){

        LimelightHelpers.setLEDMode_ForceOn("limeligh-buluk");
    }

    @Override
    public void execute(){

        Translation2d velocity = DriveCommands.getLinearVelocityFromJoysticks(x.getAsDouble(), y.getAsDouble());


        boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

        ChassisSpeeds speeds = new ChassisSpeeds(
            velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
             velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
              drive.getVision().aim());

        drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));

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
