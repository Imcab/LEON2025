package frc.robot.Subsystems.Components;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.ClosedLoopRequest;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.Constants.WristConstants.Algae;

public class AlgaeWrist extends SubsystemBase{

    private SparkMax rightWheel, leftWheel, wrist;
    private SparkMaxConfig configRightWheel, configLeftWheel, configWrist;
    private ClosedLoopControl pid;
    private ClosedLoopRequest request;
    private SparkAbsoluteEncoder encoder;
    
    public AlgaeWrist(){

        //Declarations of the 3 motors (Both sides and the Angler), the PID to get the right degree and the absolute encoder
        rightWheel = new SparkMax(Algae.CAN_ID_RIGHTWHEEL, MotorType.kBrushless);
        leftWheel = new SparkMax(Algae.CAN_ID_LEFTWHEEL, MotorType.kBrushless);
        wrist = new SparkMax(Algae.CAN_ID_WRIST, MotorType.kBrushless);

        //Declaracion del encoder 
        encoder = wrist.getAbsoluteEncoder();

        pid = new ClosedLoopControl(Algae.GAINS, OutputType.kNegative);

        request = pid.new ClosedLoopRequest();

        pid.setTolerance(5);

        configWrist = new SparkMaxConfig();
        configLeftWheel = new SparkMaxConfig();
        configRightWheel = new SparkMaxConfig();

        burnflash();
    }

    @Override
    public void periodic(){

    }

    private void burnflash(){
        //method to configure the motors

        wrist.setCANTimeout(250);
        leftWheel.setCANTimeout(250);
        rightWheel.setCANTimeout(250);
    
        configWrist.
            inverted(Algae.wristMotorInverted).
            idleMode(IdleMode.kBrake).
            smartCurrentLimit(Algae.wristCurrentLimit);

        configLeftWheel.
            inverted(Algae.LeftInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Algae.WheelsCurrentLimit);

        configRightWheel.
            inverted(Algae.RightInverted).
            idleMode(IdleMode.kCoast).
            smartCurrentLimit(Algae.WheelsCurrentLimit);
        
        wrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftWheel.configure(configLeftWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightWheel.configure(configRightWheel, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wrist.setCANTimeout(0);
        leftWheel.setCANTimeout(0);
        rightWheel.setCANTimeout(0);

    }

    public double getPosition(){
        return encoder.getPosition() * 360;
    }

    public double currentSetpoint(){
        return request.setpoint;
    }

    public void setPosition(double degrees){
        wrist.set(pid.runRequest(request.withReference(getPosition()).toSetpoint(degrees )));
    }


    public boolean atGoal(){
        return Math.abs(getPosition() - currentSetpoint()) <= Algae.tolerance;
    }

    public void runWheels(double velocity) {
        rightWheel.set(velocity);
        leftWheel.set(velocity);
        //Set the motors to the desired velocity
    }

    public boolean areWheelsSpinning(){
        return rightWheel.get() != 0;
    }
    
    public void stop(){
        rightWheel.stopMotor();
        leftWheel.stopMotor();
        wrist.stopMotor();
    }
}
