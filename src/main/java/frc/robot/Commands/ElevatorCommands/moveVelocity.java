package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class moveVelocity extends Command{
    
    ElevatorSubsystem elevator;
    private double velocity;

    public moveVelocity(ElevatorSubsystem elevator, double velocity){
        this.elevator = elevator;
        this.velocity = velocity;

        addRequirements(elevator);
    }
    
    @Override
    public void initialize () {}

    @Override
    public void execute (){
        
        elevator.setSpeed(velocity);
    }

    @Override
    public void end (boolean interrupted) {
        elevator.setSpeed(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
