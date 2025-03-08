package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.CoralWrist;

public class movePosition extends Command{
    
    private ElevatorSubsystem elevator;
    private DoubleSupplier position;
    CoralWrist coral;
    double degrees;

    public movePosition(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees){
        this.elevator = elevator;
        this.position = position;

        this.coral = coral;
        this.degrees = degrees;

        addRequirements(elevator, coral);
    }
    
    @Override
    public void initialize () {}

    @Override
    public void execute (){
        coral.requestPositionDown(degrees);
        elevator.setPosition(position);
    }

    @Override
    public void end (boolean interrupted) {
        coral.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
