package frc.robot.Commands.WristCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Components.CoralWrist;

public class wristSpeed extends Command {

    private final CoralWrist coral;
    private double speed;
  
    public wristSpeed(CoralWrist coral, double speed){
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }

    @Override
    public void initialize(){
      
    }

    @Override
    public void execute(){

        coral.wheelSpeed(speed);

    }

    @Override
    public void end(boolean interrupted) {
        coral.wheelSpeed(0);
    }

    @Override
    public boolean isFinished(){  
        return false;
    }
}
