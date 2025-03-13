package frc.robot.Subsystems.Components;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.ClosedLoopRequest;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.Constants.WristConstants.Coral;

public class CoralWrist extends SubsystemBase{
    
    private SparkMax wrist; 
    private RelativeEncoder wristEncoder;
    private SparkMax wheel;

    private SparkMaxConfig ConfigWrist , ConfigEater;

    private double target;

    private DigitalInput beamBreaker = new DigitalInput(Coral.DIO_PORT_SENSOR); 

    private ClosedLoopControl pidUp;
    private ClosedLoopControl pidDown;

    private ClosedLoopRequest requestUp; 
    private ClosedLoopRequest requestDown; 

    //Declaracion de 
    public CoralWrist(){

        pidUp  = new ClosedLoopControl(Coral.GainsUp, OutputType.kNegative);
        pidDown  = new ClosedLoopControl(Coral.GainsDown, OutputType.kNegative);

        requestUp = pidUp.new ClosedLoopRequest();
        requestDown = pidDown.new ClosedLoopRequest();

        wrist = new SparkMax(Coral.CAN_ID_WRIST, MotorType.kBrushless);

        wristEncoder = wrist.getEncoder();

        wheel = new SparkMax(Coral.CAN_ID_EATER, MotorType.kBrushless);

        ConfigWrist = new SparkMaxConfig();
        ConfigEater = new SparkMaxConfig();

        requestUp.enableOutputClamp(true);
        requestUp.withClamp(1);
        pidUp.setTolerance(0.5);

        requestDown.enableOutputClamp(true);
        requestDown.withClamp(1);
        pidDown.setTolerance(0.5);

        Burnflash();

        setZero();

    }

    private void Burnflash(){

        wrist.setCANTimeout(250);
        wheel.setCANTimeout(250);

        ConfigWrist.
            inverted(Coral.wristMotorInverted).
            idleMode(IdleMode.kBrake).
            smartCurrentLimit(Coral.wristCurrentLimit);
        //Config eater
        ConfigEater.
            inverted(Coral.wheelInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(30);

        wrist.configure(ConfigWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        wheel.configure(ConfigEater, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setZero();
        
        wrist.setCANTimeout(0);
        wheel.setCANTimeout(0);
    }

    @Override
    public void periodic(){

        SmartDashboard.putNumber("CORALWRIST: RawPosition:", getRawPosition());
        SmartDashboard.putBoolean("CORALWRIST: AtGoal:"  , atGoal());
        SmartDashboard.putNumber("CORALWRIST: Target:", target);
        SmartDashboard.putBoolean("PIECE", hasPiece());
    }

    public boolean hasPiece(){
        return beamBreaker.get();
    }

    public double getRawPosition(){
        return ((-wristEncoder.getPosition() * 360)/25);
    }

    public void setZero(){
        wristEncoder.setPosition(0);
    }
    
    public void setSpeed(double speed){
        wrist.set(speed);
    }

    public void requestPositionUp(double degrees){
        this.target = degrees;
        wrist.set(pidUp.runRequest(requestUp.withReference(getRawPosition()).toSetpoint(degrees)));
    }

    public void requestPositionDown(double degrees){
        this.target = degrees;
        wrist.set(pidDown.runRequest(requestDown.withReference(getRawPosition()).toSetpoint(degrees)));
    }

    public void wheelSpeed(double speed){
        wheel.set(speed);
    }
   
    public boolean isWheelSpinning(){
        return wheel.get() != 0;
    }

    public void stop(){
        wrist.stopMotor();
        wheel.stopMotor();
    }

    public boolean atGoal(){
   
        return Math.abs(target - getRawPosition()) <= 1.5;
    }

}
