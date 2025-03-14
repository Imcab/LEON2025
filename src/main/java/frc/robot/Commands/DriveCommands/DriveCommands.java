package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.BulukLib.Util.QoLUtil;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;
import frc.robot.SwerveLib.Tidal.TidalUtils.Coordinate;

import java.util.function.DoubleSupplier;

public class DriveCommands {

  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = QoLUtil.square(linearMagnitude);

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

   //Field relative drive command using two joysticks (controlling linear and angular velocities).
   
  public static Command joystickDrive(
      swerve drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          omega = Math.copySign(QoLUtil.square(omega), omega);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
  
  public static Command resetHeading(swerve drive){
        return Commands.runOnce(()-> {
            drive.blinkin.setPattern(PatternType.ShotRed);
            drive.resetHeading();
        }, drive).beforeStarting(()-> drive.offLeds(true)).finallyDo(()-> drive.offLeds(false));
  }

  public static Command uploadPose(swerve drive,Coordinate coordinate){
    return Commands.runOnce(
        ()->{
            drive.blinkin.setPattern(PatternType.LightChaseGray);
            drive.setPose(coordinate.toPose2d());
        },
        drive).beforeStarting(()-> drive.offLeds(true)).finallyDo(()-> drive.offLeds(false));
  }

  public static Command brake(swerve drive){
        return Commands.run(()->{
            
            drive.stopWithX();
            
        }, drive);
  }

  public static Command moveInX(swerve drive, double speed){
        return Commands.run(()-> {
            drive.runVelocity(new ChassisSpeeds(0, speed, 0));
        }, drive);
  }

  public static Command moveInY(swerve drive, double speed){
    return Commands.run(()-> {
        drive.runVelocity(new ChassisSpeeds(speed,0, 0));
    }, drive);
}

  public static Command snapToApril(
      swerve drive,
      DoubleSupplier ySupplier) {

    final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  
    return Commands.run(
        () -> {

        var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND));
            
        drive.snapApriltag(ySpeed);

        },
    
        drive).finallyDo(
        drive::stop
        );
  }

  public static Command rotateToApril(
      swerve drive,
      DoubleSupplier ySupplier,
      DoubleSupplier xSupplier) {

    final SlewRateLimiter limiter = new SlewRateLimiter(3);
  
    return Commands.run(
        () -> {

         var ySpeed =
            -limiter.calculate(MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND));

        drive.rotateToApril(ySpeed, -xSupplier.getAsDouble() * 0.5);

        },
    
        drive).finallyDo(
        drive::stop
        );
  }

}
  
