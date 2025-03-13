package frc.robot.Commands.Reset;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;

public class ResetUtils {

    public static Command resetElevatorEncoders(ElevatorSubsystem elevator){
        return Commands.runOnce(()-> elevator.setEncoderPosition(0), elevator);
    }

    public static Command resetWristEncoder(CoralWrist wrist){
        return Commands.runOnce(()-> wrist.setZero(), wrist);
    }
}
