package frc.robot.Subsystems.Components.Hanger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BulukLib.Util.Mechanism;

public class HangerClimber extends SubsystemBase implements Mechanism{

    private Mechanism hanger;

    public HangerClimber(Mechanism hanger){

        this.hanger = hanger;

        hanger.config();
    
    }

    public void setSpeed(double speed){
        hanger.setSpeed(speed);
    }

    public void stop(){
        hanger.stop();
    }

}
