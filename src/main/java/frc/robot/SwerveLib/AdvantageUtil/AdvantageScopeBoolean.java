package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AdvantageScopeBoolean {
    private BooleanPublisher publisher;

    public AdvantageScopeBoolean(String key){
        this.publisher = NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
    }

    public void sendBoolean(boolean value){
        publisher.set(value);
    }
    
}
