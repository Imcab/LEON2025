package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AdvantageScopeArrayDouble {
    private DoubleArrayPublisher publisher;

    public AdvantageScopeArrayDouble(String key){
        this.publisher = NetworkTableInstance.getDefault().getDoubleArrayTopic(key).publish();
    }

    public void sendDouble(double[] value){
        publisher.set(value);
    }

}
