package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AdvantageScopeArrayBoolean{
    private BooleanArrayPublisher publisher;

    public AdvantageScopeArrayBoolean(String key){
        this.publisher = NetworkTableInstance.getDefault().getBooleanArrayTopic(key).publish();
    }

    public void sendBoolean(boolean[] value){
        publisher.set(value);
    }

}
