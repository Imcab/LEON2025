package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.CoralWrist;

public class AngleWrist extends Command {
    CoralWrist coral;
    double degrees;

    public AngleWrist(CoralWrist coral, double degrees){
        this.coral = coral;
        this.degrees = degrees;

        addRequirements(coral);
    }

    @Override
    public void initialize () {}

    @Override
    public void execute (){
        coral.requestPositionDown(degrees);
    }

    @Override
    public void end (boolean interrupted) {
       coral.stop();
    }
    
    @Override
    public boolean isFinished(){
        return false;
    }


}
