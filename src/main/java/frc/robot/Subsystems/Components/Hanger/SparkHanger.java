package frc.robot.Subsystems.Components.Hanger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.BulukLib.Util.Mechanism;
import frc.robot.Constants.ConstantsHanger;

public class SparkHanger implements Mechanism{

    private final SparkMax RightHanger, LeftHanger;
    private SparkMaxConfig leftConfig, rightConfig;

    public SparkHanger(){

        RightHanger = new SparkMax(ConstantsHanger.RightHangerPort, MotorType.kBrushless);
        LeftHanger = new SparkMax(ConstantsHanger.LeftHangerPort, MotorType.kBrushless);

        rightConfig = new SparkMaxConfig();
        leftConfig = new SparkMaxConfig();

    }
    @Override
    public void setSpeed(double speed){
        RightHanger.set(speed);
        LeftHanger.set(speed);
    }

    @Override
    public void stop(){
        RightHanger.stopMotor();
        LeftHanger.stopMotor();
    }

    @Override
    public void config(){

        RightHanger.setCANTimeout(250);
        rightConfig.idleMode(IdleMode.kBrake).inverted(ConstantsHanger.r_inverted);
        RightHanger.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightHanger.setCANTimeout(0);

        LeftHanger.setCANTimeout(250);
        leftConfig.idleMode(IdleMode.kBrake).inverted(ConstantsHanger.l_inverted);
        LeftHanger.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftHanger.setCANTimeout(0);

    }
}
