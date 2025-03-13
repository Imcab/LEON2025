package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BulukLib.Util.AllianceUtil;
import frc.robot.Subsystems.Components.Climber;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin.PatternType;

public class ClimberCommand extends Command {

    private Climber climber;
    private double speed;
    private swerve Drive;
    private PatternType selectedPatter;
    
    public ClimberCommand(Climber climber, double speed, swerve Drive){
        this.climber = climber;
        this.speed = speed;
        this.Drive = Drive;

        if (AllianceUtil.isBlue()) {
            selectedPatter = PatternType.RaimbowOceanPalette;
        }else{
            selectedPatter = PatternType.RaimbowLavaPalette;
        }

        addRequirements(climber);
    }

    @Override
    public void initialize () {
        Drive.offLeds(true);
    }

    @Override
    public void execute (){

        Drive.blinkin.setPattern(selectedPatter);
        climber.setSpeed(speed);
    }

    @Override
    public void end (boolean interrupted) {
        Drive.offLeds(false);
        climber.stop();
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }

    
    

}
