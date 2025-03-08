package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class AdvantageScopeString{
    private StringPublisher publisher;

    public AdvantageScopeString(String key){
        this.publisher = NetworkTableInstance.getDefault().getStringTopic(key).publish();
    }

    public void sendDouble(String value){
        publisher.set(value);
    }
    
}
