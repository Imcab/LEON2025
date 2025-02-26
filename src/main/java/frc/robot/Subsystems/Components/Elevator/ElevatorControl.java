package frc.robot.Subsystems.Components.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.BulukLib.MotionControllers.ClosedLoopControl.ClosedLoopControl.OutputType;
import frc.robot.BulukLib.MotionControllers.Gains.TrapezoidalGains;
import frc.robot.BulukLib.MotionControllers.TrapezoidalControl.Trapezoidal;
import frc.robot.BulukLib.MotionControllers.TrapezoidalControl.TrapezoidalTolerance;

public interface ElevatorControl{

    public static final double kHome = 0.63; //Metros

    public static class ElevatorState{
    
        public State currentGoal = new State();
        public double positionGoal = 0;
        public double velocityGoal = 0;
        public double meters = 0;
        public double velocityMetersPerSec = 0;
        public double motionOutput = 0;
        public double currentVoltage = 0;
        public double motorOutput = 0;
  
    }

    public default TrapezoidalGains currentGains(){
        return new TrapezoidalGains(0,0,0,0,0,0,0);
    }

    public default TrapezoidalTolerance currentTolerance(){
        return new TrapezoidalTolerance(0, 0);
    }

    public default void updateState(ElevatorState state){};

    public default void resetController(){}

    public default void setSpeed(double speed){}

    public default void setEncoderPosition(double position){}

    public default void setVoltage(double volts){}

    public default void stop(){}

    public default void configureElevator(){}

    public default boolean isMTY(){
        return false;
    }

    public default Trapezoidal getController(){
        return new Trapezoidal(currentGains(), OutputType.kPositive);
    }
    
}
