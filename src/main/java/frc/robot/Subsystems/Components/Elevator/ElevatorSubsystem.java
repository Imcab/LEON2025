package frc.robot.Subsystems.Components.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BulukLib.Math.Domain;
import frc.robot.BulukLib.Math.DomainUtils;
import frc.robot.BulukLib.Math.Domain.DomainEdge;

import frc.robot.BulukLib.MotionModel.MotionModel.VelocityGoal;

public class ElevatorSubsystem extends SubsystemBase{

    private static final int limitPort = 0;
    private final DigitalInput limitSwitch;

    private String DashboardKey;

    private final SparkMax leader;
    private final SparkMax follower;
    private final SparkMaxConfig leaderConfig;
    private final SparkMaxConfig followerConfig;
    private final RelativeEncoder leaderEncoder;

    private final double minMeters = 0.62;
    private final double maxMeters = 1.92;

    private final double secondStageMeters = 0.89;
    private final double thirdStageMeters = 1.31;

    private final Domain fullRangeMeters;

    private final Domain firstStage;

    private final Domain secondStage;

    private final Domain maxStage;

    private int elevatorStage = 0;

    private ProfiledPIDController trapezoid;

    private double[] kS = new double[]{1.8,1.2,0.5};
    private double[] kV = new double[]{20.2,17,17};
    private double[] kG = new double[]{0.31,0.31,0.31};
    private double[] kA = new double[]{0.052,0.042,0.04};

    private Supplier<State> trapezoidGoal = State::new;

    private TrapezoidProfile.State setpoint = new State();

    private boolean atGoal = false;

    private boolean newGoal = false;

    private boolean shouldStop = false;

    public ElevatorSubsystem(String key){

        this.DashboardKey = key;

        this.fullRangeMeters = new Domain(minMeters, maxMeters); 

        this.firstStage = new Domain(
            Double.NEGATIVE_INFINITY, DomainEdge.FULL,
            secondStageMeters, DomainEdge.EMPTY);

        this.secondStage = new Domain(
            secondStageMeters, DomainEdge.FULL,
            thirdStageMeters, DomainEdge.EMPTY);

        this.maxStage = new Domain(
            thirdStageMeters, DomainEdge.FULL,
            Double.POSITIVE_INFINITY, DomainEdge.FULL);

        limitSwitch = new DigitalInput(limitPort);

        leaderConfig = new SparkMaxConfig();
        followerConfig = new SparkMaxConfig();

        leader = new SparkMax(ElevatorConstants.CAN_ID_LEADER, MotorType.kBrushless);
        follower = new SparkMax(ElevatorConstants.CAN_ID_SLAVE, MotorType.kBrushless);

        leaderEncoder = leader.getEncoder();

        setEncoderPosition(0); // resets the encoders to 0

        leaderConfig.idleMode(IdleMode.kBrake).inverted(ElevatorConstants.leaderInverted);
        leaderConfig.encoder
            .uvwAverageDepth(2)
            .uvwMeasurementPeriod(10);
        
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        followerConfig.idleMode(IdleMode.kCoast).follow(ElevatorConstants.CAN_ID_LEADER, ElevatorConstants.slaveInverted);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.trapezoid = new ProfiledPIDController(
            90.5,
            0,
            0,
            new Constraints(
                ElevatorConstants.fullMaxVelocity,
                ElevatorConstants.fullMaxAcc),
            0.02);

        trapezoid.setTolerance(0.02, 0.08);

        trapezoid.reset(0.63, 0);

    }

    public boolean atGoal(){
        return atGoal;
    }

    public void setEncoderPosition(double position) {
        leaderEncoder.setPosition(position);
    }

    public double getMeters(){
        return (
            (leaderEncoder.getPosition() *
            ElevatorConstants.CONVERSION_FACTOR) +
            ElevatorConstants.ELEVATOR_OFFSET_CENTIMETERS
            ) / 100;
    }

    public boolean pressed(){
        return !limitSwitch.get();
    }

    public void setVoltage(double volts){
        leader.setVoltage(volts);
    }

    public void setSpeed(double speed){
        leader.set(speed);
    }

    public double fixSetpoint(double setpointMeters){

        if (fullRangeMeters.outOfBounds(setpointMeters)) {
            setpointMeters = MathUtil.clamp(setpointMeters, fullRangeMeters.minValue(), fullRangeMeters.maxValue());
        }

        return setpointMeters;
    }


    public double voltsFed(){
        return leader.getBusVoltage();
    }

    public double appliedSpeed(){
        return leader.getAppliedOutput();
    }

    public void stop(){
        newGoal = false;
        leader.stopMotor();
    }

    public int getStage(){
        return elevatorStage;
    }

    public void setPosition(Supplier<State> trapezoidalGoal){
        newGoal = true;
        atGoal = false;
        this.trapezoidGoal = trapezoidalGoal;
    }

    public void setPosition(DoubleSupplier position){
        setPosition(()-> new State(position.getAsDouble(), 0));
    }

    public boolean handleState(BooleanSupplier eStop){
        this.shouldStop = eStop.getAsBoolean();
        return shouldStop;
    }

    public double feedforwardCalculate(double kS, double kV, double kG, double kA, VelocityGoal CurrentVelocity){

        double sign = Math.signum(CurrentVelocity.velocity);

        double feedforwardOut = sign * kS + kV * CurrentVelocity.velocity + kG + kA * CurrentVelocity.acceleration;

        return feedforwardOut;

       }

    private double previousVel = 0;

    @Override
    public void periodic(){

        if (firstStage.inRange(getMeters()) && secondStage.outOfBounds(getMeters()) && maxStage.outOfBounds(getMeters())){
            elevatorStage = 0;
        }else if (firstStage.outOfBounds(getMeters()) && secondStage.inRange(getMeters() ) && maxStage.outOfBounds(getMeters())) {
            elevatorStage = 1;
        }else{
            elevatorStage = 2;
        }

        if (pressed()) {
            setEncoderPosition(0.0);
        }

        final boolean shouldRunMotion = 
            DriverStation.isEnabled() &&
            newGoal;
    
        if (shouldRunMotion) {

            this.setpoint = trapezoidGoal.get();
            previousVel = setpoint.velocity;
        
            double acceleration = (setpoint.velocity - previousVel) / 0.02;

            VelocityGoal goal = new VelocityGoal(setpoint.velocity, acceleration);

            double output = trapezoid.calculate(getMeters(), this.setpoint)
            + feedforwardCalculate(
                kS[getStage()],
                kV[getStage()],
                kG[getStage()],
                kA[getStage()],
                goal);

            leader.setVoltage(output);
    
            atGoal = DomainUtils.inRange(getMeters(), trapezoidGoal.get().position - 0.01, trapezoidGoal.get().position + 0.01);

            if (atGoal) {
                newGoal = false;
            }

        }else{
            setpoint = new State(getMeters(), 0);
        }

        if (shouldStop) {
            stop();
        }

        SmartDashboard.putBoolean(DashboardKey + ": RUN PROFILE", shouldRunMotion);
        SmartDashboard.putBoolean(DashboardKey + ": ES", shouldStop);
        SmartDashboard.putNumberArray(DashboardKey + ": GOAL", new double[]{trapezoidGoal.get().position, trapezoidGoal.get().velocity});
        SmartDashboard.putNumberArray(DashboardKey + ": CURRENT SETPOINT", new double[]{setpoint.position, setpoint.velocity});
        SmartDashboard.putBoolean(DashboardKey + ": ATGOAL", atGoal());
        SmartDashboard.putBoolean(DashboardKey + ": NEW GOAL", newGoal);
        SmartDashboard.putBoolean(DashboardKey + ": LIMITSWITCH", pressed());
        SmartDashboard.putNumber(DashboardKey + ": STAGE", getStage());
        SmartDashboard.putNumber(DashboardKey +": APPLIED SPEED",  appliedSpeed());
        SmartDashboard.putNumber(DashboardKey + ": METERS", getMeters());
        
    }


}
