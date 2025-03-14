package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.BulukLib.Math.DomainUtils;
import frc.robot.BulukLib.Swerve.SwerveConfig;
import frc.robot.BulukLib.Swerve.SwerveConfig.currentLimiting;
import frc.robot.BulukLib.Swerve.SwerveConfig.measures;
import frc.robot.BulukLib.Swerve.SwerveConfig.reductions;
import frc.robot.BulukLib.Vision.LimelightHelpers;
import frc.robot.BulukLib.Vision.VisionConfig;
import frc.robot.Subsystems.Hardware.REVBlinkin;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;
import frc.robot.SwerveLib.Dashboard.AdvantageSwerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;


public class swerve extends SubsystemBase{
  
    private final AHRS navX =  new AHRS(NavXComType.kMXP_SPI);

    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(19.0);
    private static final double TRACK_WIDTH_X = SwerveConfig.measures.TRACK_WIDTH_X; 
    private static final double TRACK_WIDTH_Y = SwerveConfig.measures.TRACK_WIDTH_Y; 
    private static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private ModuleSpark []modules  = new ModuleSpark[4];

    private Vision vision = new Vision();

    public REVBlinkin blinkin;

    private CommandXboxController driveController;

    private AdvantageSwerve logSwerve = new AdvantageSwerve("swervePose");

    
    public swerve(REVBlinkin blinkin, CommandXboxController driveController){

      this.blinkin = blinkin;
      this.driveController = driveController;
     
        modules[0] = new ModuleSpark(0);
        modules[1] = new ModuleSpark(1);
        modules[2] = new ModuleSpark(2);
        modules[3] = new ModuleSpark(3);

        new Thread(() -> {
            try{
                Thread.sleep(1000);
                navX.reset();
            } catch (Exception e){
    
            }
          }).start();

      //for pathplanner
      AutoBuilder.configure(this::getPose,
      this::setPose,
      this::getChassisSpeeds,
      this::runVelocity,
      new PPHolonomicDriveController(
        new PIDConstants(5.5, 0.0,0),
         new PIDConstants(2.93, 0.0, 0.001)),
      getPathPlannerConfiguration(),
      () -> DriverStation.getAlliance().
        orElse(Alliance.Blue) == Alliance.Red,
      this);

    }

    private boolean ledRequested = false;

    public void offLeds(boolean value){
        this.ledRequested = value;
    }

    public void periodic(){

      SmartDashboard.putBoolean("LedsByMech", ledRequested);

      for (var module : modules) {
        module.periodic();
      }
      if (DriverStation.isDisabled()) {
        for (var module : modules) {
          module.stop();
      }}

      //LimelightHelpers.SetRobotOrientation(limelight.name, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      
      SwerveModulePosition[] modulePositions = getModulePositions();
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (navX.isConnected() == true) {
        // Use the real gyro angle
        rawGyroRotation = getnavXRotation();
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      
      poseEstimator.update(rawGyroRotation, modulePositions);

      updateMegaTag2();

      logSwerve.sendSwerve(getChassisSpeeds(), poseEstimator.getEstimatedPosition(), getModuleStates());

      boolean validDistance = DomainUtils.inRange(vision.limelight.ty(), -0.7, 100);
      
      if (!ledRequested) {
        if (validDistance && LimelightHelpers.getTV("limelight-buluk")) {
          blinkin.setPattern(PatternType.Violet);

          if (DriverStation.isTeleopEnabled() && DriverStation.isTeleop()) {
            driveController.getHID().setRumble(RumbleType.kBothRumble, 0.8);
          }

        }else if (LimelightHelpers.getTV("limelight-buluk") && !validDistance) {
  
          blinkin.setPattern(PatternType.RedOrange);

          if (DriverStation.isTeleopEnabled() && DriverStation.isTeleop()) {
            driveController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
          }

        }else {
          blinkin.setPattern(PatternType.DarkGreen);
          driveController.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
      }

    

    }

    public void updateMegaTag2(){

      boolean rejectPose = false;

      LimelightHelpers.SetRobotOrientation(
        "limelight-buluk",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0, 0, 0, 0, 0);

      LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-buluk");

      if (Math.abs(navX.getRate()) > 720) {
        rejectPose = true;
      }
      if (megaTag2.tagCount  == 0) {
        rejectPose = true;
      }

      if (!rejectPose) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 999999999));
        poseEstimator.addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
      }

    }

    public double getAngle(){
      return Math.IEEEremainder(navX.getAngle(), 360);
    }

    public RobotConfig getPathPlannerConfiguration(){

      return new RobotConfig(
        measures.robotMassKg,
        measures.robotMOI,
            new ModuleConfig(
              measures.WHEELRADIUS,
              getMaxLinearSpeedMetersPerSec(),
              1.0,
              DCMotor.getNEO(1).
                withReduction(reductions.DriveReduction),
              currentLimiting.driveCurrentLimit,
              1),
        getModuleTranslations());
    }

    public double getYawVelocityRadPerSec(){
      return Units.degreesToRadians(-navX.getRawGyroZ());
    }

    
    public Rotation2d getnavXRotation(){
      return Rotation2d.fromDegrees(-getAngle());
    }

    public ChassisSpeeds getChassisSpeeds(){
      return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Vision getVision(){
      return vision;
    }

    public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
      };
    }

  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }


  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Double getX() {
    return getPose().getX();
  }

  public Double getY() {
    return getPose().getY();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return 5.2;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public void resetHeading(){
    navX.reset();
  }

  public void addObservationStd(Pose2d pose, double timeStamps, Matrix< N3, N1> std){
      poseEstimator.addVisionMeasurement(pose, timeStamps, std);
  }
  public void addObservation(Pose2d pose, double timeStamps){
    poseEstimator.addVisionMeasurement(pose, timeStamps);
  }

  public void snapApriltag(double forwardSpeed){
    var State = kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        new ChassisSpeeds(
          forwardSpeed,
          vision.translation(),
          vision.aim()),
          0.02));

    SwerveDriveKinematics.desaturateWheelSpeeds(State, VisionConfig.limelight.TrackMaxSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(State[i]);
    }
  }

  public void rotateToApril(double forwardSpeed, double translateSpeed){
    var State = kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        new ChassisSpeeds(
          forwardSpeed,
          translateSpeed,
          vision.aim()),
          0.02));

    SwerveDriveKinematics.desaturateWheelSpeeds(State, VisionConfig.limelight.TrackMaxSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(State[i]);
    }

  }

  public void followToApril(double translateSpeed){
    var State = kinematics.toSwerveModuleStates(
      ChassisSpeeds.discretize(
        new ChassisSpeeds(
          vision.range(),
          translateSpeed,
          vision.aim()),
          0.02));

    SwerveDriveKinematics.desaturateWheelSpeeds(State, VisionConfig.limelight.TrackMaxSpeed);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(State[i]);
    }

  }




}
