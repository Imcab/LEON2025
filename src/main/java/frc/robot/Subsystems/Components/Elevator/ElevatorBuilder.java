package frc.robot.Subsystems.Components.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.BulukLib.MotionControllers.Gains.TrapezoidalGains;
import frc.robot.BulukLib.MotionControllers.TrapezoidalControl.Trapezoidal;
import frc.robot.BulukLib.MotionControllers.TrapezoidalControl.TrapezoidalTolerance;
import frc.robot.Subsystems.Components.Elevator.ElevatorConstants.ElevatorType;

public class ElevatorBuilder implements ElevatorControl{
    
    private final SparkMax leader, follower;
    private final SparkMaxConfig leaderConfig, followerConfig;
    private final RelativeEncoder leaderEncoder;
    private final Trapezoidal motion;
    private final ElevatorType type;
    private TrapezoidalGains gains;
    private TrapezoidalTolerance tolerance;
    private double output;

    public ElevatorBuilder(ElevatorType type){

        this.type = type;
        this.output = 0;

        leaderConfig = new SparkMaxConfig();
        followerConfig = new SparkMaxConfig();

        leader = new SparkMax(ElevatorConstants.CAN_ID_LEADER, MotorType.kBrushless);
        follower = new SparkMax(ElevatorConstants.CAN_ID_SLAVE, MotorType.kBrushless);

        leaderEncoder = leader.getEncoder();

        if(type == ElevatorType.kMTY){
            this.gains = ElevatorConstants.DEVGains;
            this.tolerance = ElevatorConstants.DEV_TOLERANCE;
        }else{
            this.gains = ElevatorConstants.COMPGains;
            this.tolerance = ElevatorConstants.COMP_TOLERANCE;
        }
   
        motion = new Trapezoidal(gains, OutputType.kPositive);
   
        motion.setTolerance(tolerance);

    }

    @Override
    public boolean isMTY(){
        if(type == ElevatorType.kMTY){
            return true;
        }
        return false;
    }
 
    @Override
    public TrapezoidalGains currentGains(){
        return gains;
    }

    @Override
    public Trapezoidal getController(){
        return motion;
    }

    @Override
    public TrapezoidalTolerance currentTolerance(){
        return tolerance;
    }

    @Override
    public void configureElevator(){

        leaderConfig.idleMode(IdleMode.kBrake).inverted(ElevatorConstants.leaderInverted);
        leaderConfig.encoder
        .uvwAverageDepth(2)
        .uvwMeasurementPeriod(10);

        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.idleMode(IdleMode.kBrake).follow(ElevatorConstants.CAN_ID_LEADER, ElevatorConstants.slaveInverted);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leaderEncoder.setPosition(0);

        motion.initTuning("ElevatorTuner");

        motion.reset(ElevatorControl.kHome); //Resets at Home position
    }

    @Override
    public void updateState(ElevatorState state){

        //Updates the Trapezoidal Graph
        motion.graph("MotionGraph");
        motion.Tune();

        state.currentGoal = motion.getSetpoint();
        state.positionGoal = state.currentGoal.position;
        state.velocityGoal = state.currentGoal.velocity;
        state.meters = (
                (leaderEncoder.getPosition() * ElevatorConstants.CONVERSION_FACTOR)
                + ElevatorConstants.ELEVATOR_OFFSET_CENTIMETERS
            ) / 100;

        state.velocityMetersPerSec = (leaderEncoder.getVelocity() * (ElevatorConstants.CONVERSION_FACTOR / 3)) / 100;
        state.motionOutput = output;
        state.currentVoltage = leader.getBusVoltage();
        state.motorOutput = leader.get();

    }

    @Override
    public void setEncoderPosition(double position){
        leaderEncoder.setPosition(position);
    }

    @Override
    public void setSpeed(double speed){
        leader.set(speed);
    }

    @Override
    public void setVoltage(double volts){
        leader.setVoltage(volts);
    }

    @Override
    public void stop(){
        leader.stopMotor();
    }
}
