package frc.robot.SwerveLib.Dashboard;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveWidget{

    public static void build(
        String keyName,
        ModuleStateLog FrontLeft,
        ModuleStateLog FrontRight,
        ModuleStateLog BackLeft,
        ModuleStateLog BackRight,
        DoubleSupplier RobotRotation){
    
        SmartDashboard.putData(keyName, new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", FrontLeft.Angle, null);
        builder.addDoubleProperty("Front Left Velocity", FrontLeft.Speed, null);

        builder.addDoubleProperty("Front Right Angle", FrontRight.Speed, null);
        builder.addDoubleProperty("Front Right Velocity", FrontRight.Speed, null);

        builder.addDoubleProperty("Back Left Angle", BackLeft.Angle, null);
        builder.addDoubleProperty("Back Left Velocity", FrontRight.Speed, null);

        builder.addDoubleProperty("Back Right Angle", BackRight.Angle, null);
        builder.addDoubleProperty("Back Right Velocity", BackRight.Speed, null);

        builder.addDoubleProperty("Robot Angle", RobotRotation, null);
        }
        });
    }
}
