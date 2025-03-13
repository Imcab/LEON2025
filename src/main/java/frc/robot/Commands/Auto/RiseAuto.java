package frc.robot.Commands.Auto;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BulukLib.Math.Domain;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;

public class RiseAuto extends Command{
    private final ElevatorSubsystem elevator;
    private final CoralWrist coral;
    private final swerve Drive;
    private double angleDegrees;

    public DoubleSupplier height;

    public boolean finished = false;;

    public Domain elevatorRange;
    public Domain wristRange;
    public Debouncer timer;
    public boolean finishCommand;

    public RiseAuto(ElevatorSubsystem elevator, CoralWrist coral, DoubleSupplier height, double angleDegrees, swerve Drive){
        this.elevator = elevator;
        this.coral = coral;
        this.Drive = Drive;
        this.height = height;
        this.angleDegrees = angleDegrees;
        this.wristRange = new Domain(angleDegrees - 1, angleDegrees + 2);
        this.elevatorRange = new Domain(height.getAsDouble() - 0.02, height.getAsDouble() + 0.02);
        timer = new Debouncer(0.2);
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        Drive.offLeds(true);
        finishCommand = false;
    }

    @Override
    public void execute(){
        
        elevator.setPosition(height);

        if (elevatorRange.inRange(elevator.getMeters())) {
            Drive.blinkin.setPattern(PatternType.Gold);
        }else{
            Drive.blinkin.setPattern(PatternType.HeartbeatWhite);
        }

        coral.requestPositionDown(angleDegrees);

    }


    @Override
    public void end(boolean interrupted) {
        Drive.offLeds(false);
    }

    @Override
    public boolean isFinished(){     
        return timer.calculate(elevatorRange.inRange(elevator.getMeters()));
    }
    
}
