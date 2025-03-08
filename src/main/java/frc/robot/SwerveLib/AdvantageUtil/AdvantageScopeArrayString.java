package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;

public class AdvantageScopeArrayString {

    private StringArrayPublisher publisher;

    public AdvantageScopeArrayString(String key){
        this.publisher = NetworkTableInstance.getDefault().getStringArrayTopic(key).publish();
    }

    public void sendString(String[] value){
        publisher.set(value);
    }
}
