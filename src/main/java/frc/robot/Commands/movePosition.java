package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class movePosition extends Command{
    
    private ElevatorSubsystem elevator;
    private DoubleSupplier position;

    public movePosition(ElevatorSubsystem elevator, DoubleSupplier position){
        this.elevator = elevator;
        this.position = position;

        addRequirements(elevator);
    }
    
    @Override
    public void initialize () {}

    @Override
    public void execute (){
        elevator.setPosition(position);
    }

    @Override
    public void end (boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return false;
    }
}
