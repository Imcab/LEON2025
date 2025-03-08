package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AdvantageScopeDouble {

    private DoublePublisher publisher;

    public AdvantageScopeDouble(String key){
        this.publisher = NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
    }

    public void sendDouble(Double value){
        publisher.set(value);
    }

}
