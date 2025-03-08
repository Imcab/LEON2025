package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;

public class AdvantageScopeData<DataType>{

    private StructPublisher<DataType> publisher;
    
    public AdvantageScopeData(String key, Struct<DataType> struct) {
        this.publisher = NetworkTableInstance.getDefault().getStructTopic(key, struct).publish();
    }

    public void sendData(DataType value){
        publisher.set(value);
    }

}
