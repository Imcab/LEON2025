package frc.robot.Commands.Auto;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BulukLib.Math.Domain;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.ElevatorSubsystem;

public class RiseAuto extends Command{
    private final ElevatorSubsystem elevator;
    private final CoralWrist coral;
    private double angleDegrees;

    public DoubleSupplier height;

    public boolean finished = false;;

    public Domain elevatorRange;
    public Domain wristRange;
    public Debouncer timer;

    public RiseAuto(ElevatorSubsystem elevator, CoralWrist coral, DoubleSupplier height, double angleDegrees){
        this.elevator = elevator;
        this.coral = coral;
        this.height = height;
        this.angleDegrees = angleDegrees;
        this.elevatorRange = new Domain(height.getAsDouble() - 0.03, height.getAsDouble() + 0.02);
        this.wristRange = new Domain(angleDegrees - 0.05, angleDegrees + 0.05);
        timer = new Debouncer(0.9);
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        finished = false;
    }

    @Override
    public void execute(){
        
        elevator.setPosition(height);
        
        coral.requestPositionDown(angleDegrees);

    }


    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished(){     
        return elevator.getMeters() <= height.getAsDouble() - 0.04;
    }
    
}
