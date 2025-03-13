package frc.robot.Commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;

public class retractAuto extends Command{
    private ElevatorSubsystem elevator;
    private CoralWrist coral;
 
    public retractAuto(ElevatorSubsystem elevator, CoralWrist coral){
        this.elevator = elevator;
        this.coral = coral;  
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        
        elevator.setPosition(()-> 0.63);
        coral.requestPositionUp(-2);

    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
        coral.stop();
    }

    @Override
    public boolean isFinished(){     
        return elevator.getMeters() <= 0.65;
    }
}
