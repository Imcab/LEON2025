package frc.robot.Commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;

public class joystickElevator extends Command{
    ElevatorSubsystem elevator;
    DoubleSupplier joyStickSupplier;
    public joystickElevator(ElevatorSubsystem elevator, DoubleSupplier joystickSupplier){
        this.elevator = elevator;
        this.joyStickSupplier = joystickSupplier;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        elevator.setSpeed(joyStickSupplier.getAsDouble() * 0.3);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stop();
    }
}
