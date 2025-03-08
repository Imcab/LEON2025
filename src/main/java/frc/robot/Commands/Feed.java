package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.CoralWrist;

public class Feed extends Command{

    private ElevatorSubsystem elevator;
    private CoralWrist coral;
    private DoubleSupplier position;
    private double degrees;

    public Feed(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees) {
        this.elevator = elevator;
        this.coral = coral;
        this.position = position;
        this.degrees = degrees;
    
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        coral.wheelSpeed(-0.6);
        elevator.setPosition(position);
        coral.requestPositionDown(degrees);
    }

    @Override
    public void end (boolean interrupted) {
        coral.wheelSpeed(0);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }


}
