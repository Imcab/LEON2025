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

    private ClosedLoopControl pid;

    private ClosedLoopRequest request; 

    //Declaracion de 
    public CoralWrist(){

        pid  = new ClosedLoopControl(Coral.Gains, OutputType.kNegative);

        request = pid.new ClosedLoopRequest();

        wrist = new SparkMax(Coral.CAN_ID_WRIST, MotorType.kBrushless);

        wristEncoder = wrist.getEncoder();

        wheel = new SparkMax(Coral.CAN_ID_EATER, MotorType.kBrushless);

        ConfigWrist = new SparkMaxConfig();
        ConfigEater = new SparkMaxConfig();

        request.enableOutputClamp(true);

        request.withClamp(1);

        pid.setTolerance(0.5);

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
        pid.graph("CoralWrist");
        SmartDashboard.putNumber("[CORALWRIST]: RawPosition:", getRawPosition());
        SmartDashboard.putBoolean("[CORALWRIST]: AtGoal:"  , atGoal());
        SmartDashboard.putNumber("[CORALWRIST]: Target:", target);
        SmartDashboard.putNumber("CoralP", getRawPosition());
        SmartDashboard.putBoolean("PIECE", hasPiece());
    }

    public boolean hasPiece(){
        return beamBreaker.get();
    }

    public double getRawPosition(){
        return -wristEncoder.getPosition();
    }

    public void setZero(){
        wristEncoder.setPosition(0);
    }
    
    public void setSpeed(double speed){
        wrist.set(speed);
    }

    public void requestPosition(double rotations){
        this.target = rotations;
        wrist.set(pid.runRequest(request.withReference(getRawPosition()).toSetpoint(rotations)));
    }

    public void autoRequest(double rotations){
        this.target = rotations;
        wrist.set(pid.runRequest(request.withReference(getRawPosition()).toSetpoint(rotations)) * 0.3472);
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
   
        return Math.abs(target - getRawPosition()) <= 0.1;
    }

}
