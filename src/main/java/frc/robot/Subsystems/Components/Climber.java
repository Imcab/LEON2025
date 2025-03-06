package frc.robot.Subsystems.Components;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConstantsHanger;

public class Climber extends SubsystemBase{

    private final TalonFX rightHanger;
    private final TalonFX leftHanger;

    private final TalonFXConfiguration LeftConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration RightConfig = new TalonFXConfiguration();

    public Climber(){

        rightHanger = new TalonFX(ConstantsHanger.RightHangerPort);
        leftHanger = new TalonFX(ConstantsHanger.LeftHangerPort);

        rightHanger.setNeutralMode(NeutralModeValue.Brake);
        leftHanger.setNeutralMode(NeutralModeValue.Brake);

        LeftConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        LeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        LeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        RightConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        RightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        RightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightHanger.getConfigurator().apply(RightConfig);
        leftHanger.getConfigurator().apply(LeftConfig);
    
    }

    public void setSpeed(double speed){
        rightHanger.set(speed);
        leftHanger.set(speed);
    }

    public void stop(){
        rightHanger.stopMotor();
        leftHanger.stopMotor();
    }

}
