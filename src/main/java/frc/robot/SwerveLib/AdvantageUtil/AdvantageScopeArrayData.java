package frc.robot.SwerveLib.AdvantageUtil;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.struct.Struct;

public class AdvantageScopeArrayData<DataType>{

    private StructArrayPublisher<DataType> publisher;
    
    public AdvantageScopeArrayData(String key, Struct<DataType> struct) {
        this.publisher = NetworkTableInstance.getDefault().getStructArrayTopic(key, struct).publish();
    }

    public void sendData(DataType[] value){
        publisher.set(value);
    }
}
