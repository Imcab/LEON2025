package frc.robot.Commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.REVBlinkin.PatternType;

public class movePosition extends Command{
    
    private swerve Drive;
    private ElevatorSubsystem elevator;
    private DoubleSupplier position;
    private CoralWrist coral;
    private double degrees;

    public movePosition(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees, swerve Drive){

        this.elevator = elevator;
        this.position = position;
    
        this.coral = coral;
        this.degrees = degrees;

        this.Drive = Drive;

        addRequirements(elevator, coral);
    }
    
    @Override
    public void initialize () {
        Drive.offLeds(true);
    }

    @Override
    public void execute (){

        coral.requestPositionDown(degrees);
        elevator.setPosition(position);

        if( !elevator.atGoal()){
            Drive.blinkin.setPattern(PatternType.HeartbeatWhite);
        }else{
            Drive.blinkin.setPattern(PatternType.White);
        }
    }

    @Override
    public void end (boolean interrupted) {
        Drive.offLeds(false);
        coral.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
