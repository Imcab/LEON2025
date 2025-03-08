package frc.robot.SwerveLib.Dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveLib.AdvantageUtil.AdvantageScopeArrayData;
import frc.robot.SwerveLib.AdvantageUtil.AdvantageScopeData;

public class AdvantageSwerve {

    private AdvantageScopeData<ChassisSpeeds> robotSpeeds;
    private AdvantageScopeData<Pose2d> botPose;
    private AdvantageScopeArrayData<SwerveModuleState> moduleStates;

    public AdvantageSwerve(String swerveKey){
        this.robotSpeeds = new AdvantageScopeData<>(swerveKey +"/ ChassisSpeeds", ChassisSpeeds.struct);
        this.botPose = new AdvantageScopeData<>(swerveKey + "/ RobotPose2D", Pose2d.struct);
        this.moduleStates = new AdvantageScopeArrayData<>(swerveKey + "/ ModuleStates", SwerveModuleState.struct);
    }

    public void sendSwerve(ChassisSpeeds currentRobotSpeeds, Pose2d odometryPose, SwerveModuleState[] currentModuleStates){
        robotSpeeds.sendData(currentRobotSpeeds);
        botPose.sendData(odometryPose);
        moduleStates.sendData(currentModuleStates);
    }

}
