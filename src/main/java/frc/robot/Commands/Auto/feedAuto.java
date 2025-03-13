package frc.robot.Commands.Auto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;

public class feedAuto extends Command{

    private ElevatorSubsystem elevator;
    private CoralWrist coral;
    private DoubleSupplier position;
    private double degrees;
    private swerve drive;
    private Debouncer debouncer;
    private double speed;

    public feedAuto(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees, double speed, swerve drive) {
        this.elevator = elevator;
        this.coral = coral;
        this.position = position;
        this.degrees = degrees;
        this.drive = drive;
        this.speed = speed;
        this.debouncer = new Debouncer(0.3, DebounceType.kRising);
    
        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        drive.offLeds(true);
    }

    @Override
    public void execute(){
        coral.wheelSpeed(speed); //-0.6
        elevator.setPosition(position);
        coral.requestPositionDown(degrees);

        if (coral.hasPiece()) {
            drive.blinkin.setPattern(PatternType.SkyBlue);
        }else{
            drive.blinkin.setPattern(PatternType.BreathRed);
        }

    }

    @Override
    public void end (boolean interrupted) {
        drive.offLeds(true);
        coral.wheelSpeed(0);
    }
    
    @Override
    public boolean isFinished(){
        return debouncer.calculate(coral.hasPiece());
    }


}
