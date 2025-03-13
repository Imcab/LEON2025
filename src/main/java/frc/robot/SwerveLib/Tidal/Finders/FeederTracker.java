package frc.robot.SwerveLib.Tidal.Finders;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.SwerveLib.AdvantageUtil.AdvantageScopeBoolean;
import frc.robot.SwerveLib.AdvantageUtil.AdvantageScopeData;
import frc.robot.SwerveLib.Tidal.TidalConfig;
import frc.robot.SwerveLib.Tidal.TidalConfig.feeders;

public class FeederTracker extends TidalFinder{


    private AdvantageScopeBoolean atGoal = new AdvantageScopeBoolean("FeederTrack: AtGoal");
    private AdvantageScopeBoolean atLeftFeeder = new AdvantageScopeBoolean("FeederTracker: AtLeftFeeder");
    private AdvantageScopeData<Pose2d> currentPoseGoal = new AdvantageScopeData<>("FeederTracker: CurrentGoal", Pose2d.struct);

    public FeederTracker(swerve drive){
        linkVehicle(drive);
        setNavigationInaccuracy(15);
        setConstraints(TidalConfig.pathGains.mFast);

    }

    public ConditionalCommand left(){
        return navigateToCoordinate(feeders.leftFeederAlign);
    }

    public ConditionalCommand right(){
        return navigateToCoordinate(feeders.rightFeederAlign);
    }
  
    public ConditionalCommand nearest(){
        return new ConditionalCommand(left(), right(), ()-> atLeftFeeder());
    }

    public boolean atLeftFeeder(){
        return isBlue().getAsBoolean() ? TidalConfig.feeders.leftFeederDomainBlue.inRange(lastCoordinate().inY()) : TidalConfig.feeders.leftFeederDomainRed.inRange(lastCoordinate().inY());
    }

    public void stop(){
        stopVehicle();
    }

    @Override
    public void periodic(){
        
        atGoal.sendBoolean(atGoal());
        currentPoseGoal.sendData(getGoal().toPose2d());
        atLeftFeeder.sendBoolean(atLeftFeeder());
         
    }

}
