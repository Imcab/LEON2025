package frc.robot.Commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;

public class moveBack extends Command{
    
    private ElevatorSubsystem elevator;
    private DoubleSupplier position;
    private CoralWrist coral;
    private double degrees;
    private swerve drive;

    public moveBack(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees, swerve Drive){
        this.elevator = elevator;
        this.position = position;
        this.drive = Drive;

        this.coral = coral;
        this.degrees = degrees;

        addRequirements(elevator, coral);
    }
    
    @Override
    public void initialize () {
        drive.offLeds(true);
    }

    @Override
    public void execute (){
        coral.requestPositionUp(degrees);
        elevator.setPosition(position);

        if( !elevator.atGoal()){
            drive.blinkin.setPattern(PatternType.FireLarge);
        }
    }

    @Override
    public void end (boolean interrupted) {
        drive.offLeds(false);
        coral.stop();
        elevator.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

