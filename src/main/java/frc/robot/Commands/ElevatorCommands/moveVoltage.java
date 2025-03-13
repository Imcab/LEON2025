package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;

public class moveVoltage extends Command{

    ElevatorSubsystem elevator;
    private double voltage;

    public moveVoltage(ElevatorSubsystem elevator, double voltage){
        this.elevator = elevator;
        this.voltage = voltage;

        addRequirements(elevator);
    }
    
    @Override
    public void initialize () {}

    @Override
    public void execute (){

        elevator.setVoltage(voltage);
    }

    @Override
    public void end (boolean interrupted) {
        elevator.setVoltage(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
