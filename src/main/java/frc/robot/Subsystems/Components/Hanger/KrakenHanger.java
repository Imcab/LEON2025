package frc.robot.Subsystems.Components.Hanger;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.BulukLib.Util.Mechanism;
import frc.robot.Constants.ConstantsHanger;

public class KrakenHanger implements Mechanism{

    public final TalonFX rightHanger;
    public final TalonFX leftHanger;
   
    public KrakenHanger(){
        rightHanger = new TalonFX(ConstantsHanger.RightHangerPort);
        leftHanger = new TalonFX(ConstantsHanger.LeftHangerPort);
    }

    @Override
    public void setSpeed(double speed){
        rightHanger.set(-speed);
        leftHanger.set(speed);
    }

    @Override
    public void stop(){
        rightHanger.stopMotor();
        leftHanger.stopMotor();
    }
}
