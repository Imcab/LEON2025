package frc.robot.Commands.RoutinesCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;

public class Feed extends Command{

    private ElevatorSubsystem elevator;
    private CoralWrist coral;
    private DoubleSupplier position;
    private double degrees;
    private swerve drive;
    private CommandXboxController driverController;
    private CommandXboxController operatorController;

    public Feed(ElevatorSubsystem elevator, DoubleSupplier position, CoralWrist coral, double degrees, swerve drive, CommandXboxController driverController, CommandXboxController operatorController){
        this.elevator = elevator;
        this.coral = coral;
        this.position = position;
        this.degrees = degrees;
        this.drive = drive;
        this.driverController = driverController;
        this.operatorController = operatorController;

        addRequirements(elevator, coral);
    }

    @Override
    public void initialize(){
        drive.offLeds(true);
    }

    @Override
    public void execute(){
        coral.wheelSpeed(-0.6);
        elevator.setPosition(position);
        coral.requestPositionDown(degrees);

        if (coral.hasPiece()){

            drive.blinkin.setPattern(PatternType.SkyBlue);

            if (DriverStation.isTeleopEnabled() && coral.hasPiece()) {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
            }else{
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }

        }else{

            drive.blinkin.setPattern(PatternType.BreathRed);

            driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            
        }

    }

    @Override
    public void end (boolean interrupted) {
        drive.offLeds(false);
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        coral.wheelSpeed(0);
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }


}
