package frc.robot.Commands.RoutinesCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.REVBlinkin.PatternType;

public class Feed extends Command{

    private ElevatorSubsystem elevator;
    private CoralWrist coral;
    private DoubleSupplier position;
    private double degrees;
    private swerve drive;

    public Feed(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees, swerve drive) {
        this.elevator = elevator;
        this.coral = coral;
        this.position = position;
        this.degrees = degrees;
        this.drive = drive;
    
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        drive.offLeds(true);
    }

    @Override
    public void execute(){
        coral.wheelSpeed(-0.6);
        elevator.setPosition(position);
        coral.requestPositionDown(degrees);

        if (coral.hasPiece()) {
            drive.blinkin.setPattern(PatternType.Yellow);
        }else{
            drive.blinkin.setPattern(PatternType.HeartbeatRed);
        }

    }

    @Override
    public void end (boolean interrupted) {
        drive.offLeds(true);
        coral.wheelSpeed(0);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }


}
