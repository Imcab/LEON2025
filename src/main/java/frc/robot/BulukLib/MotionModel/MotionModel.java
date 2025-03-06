package frc.robot.BulukLib.MotionModel;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BulukLib.MotionControllers.Gains.PIDGains;

public class MotionModel extends ProfiledPIDController{

    public static class Gains{

        public double kP;
        public double kI;
        public double kD;
        public double kAcceleration;
        public double kVelocity;
        public double kS;
        public double kA;
        public double kG;
        public double kV;

        /**
        * @param gains The PID Gains (proportional, integrative and derivative)
        * @param kVelocity Max velocity
        * @param kAcceleration Max acceleration for goal
        * @param kV output to overcome velocity
        * @param kS output to overcome static friction
        * @param kG output to overcome gravity
        * @param kA output that modifies acceleration factor
        */
        public Gains(PIDGains gains, double kAcceleration, double kVelocity,double kV, double kS, double kA, double kG){
            this.kP = gains.kp;
            this.kI = gains.ki;
            this.kD = gains.kd;
            this.kAcceleration = kAcceleration;
            this.kVelocity = kVelocity;
            this.kV = kV;
            this.kS = kS;
            this.kA = kA;
            this.kG = kG;
        }

        public double[] toArray(){
            return new double[]{kP,kI,kD,kAcceleration,kVelocity,kV,kS,kA,kG};
        }

    }

    /**
     * Class for velocity and acceleration goals
     */
    public static class VelocityGoal{

        /*
         * Velocity magnitude
         */
        public double velocity;
        /*
         * Acceleration magnitude
         */
        public double acceleration;

        /**
         * Represents a desired velocity and acceleration
         * @param velocity the current velocity 
         * @param acceleration the current system acceleration
         */
        public VelocityGoal(double velocity, double acceleration){
            this.velocity = velocity;
            this.acceleration = acceleration;
        }


    }

    private Gains gains;

    /**
     * The calculated Profile output
     */
    public double output;
    /**
     * The calculated feedforwardOutput
     */
    public double feedforwardOutput;
    /**
     * The current profile measurement
     */
    public double measurement;

    /**
     * Allocates a MotionModel Profiled Controller that uses a ProfilePIDController output and a calculated FeedForward output
     * @param gains The Motion Model Profile Constraints
     */
    public MotionModel(Gains gains){

        super(
            gains.kP,
            gains.kI,
            gains.kD,
            new TrapezoidProfile.Constraints(
                gains.kVelocity,
                gains.kAcceleration)
            );

        this.gains = gains;
        this.output = 0;
        this.measurement = 0;
        this.feedforwardOutput = 0;

    }

    /**
     * Logs useful data to dashboard. Tolerance, output, ffOutput, Setpoint, Error, atSetpoint, atGoal, Measurement, Gains
     * @param key the key to shown in the dashboard as: [yourKey] Value: 
     */
    public void publishToDashboard(String key){
        SmartDashboard.putNumberArray("["+ key +"]" + " Tolerance:", new double[] {getPositionTolerance(), getVelocityTolerance()});
        SmartDashboard.putNumber("["+ key +"]" + " Output:", output);
        SmartDashboard.putNumber("["+ key +"]", feedforwardOutput);
        SmartDashboard.putNumberArray("["+ key +"]" +"Setpoint:", new double[] {getSetpoint().position, getSetpoint().velocity});
        SmartDashboard.putNumber("["+ key +"]" +" Error:", getPositionError());
        SmartDashboard.putBoolean("["+ key +"]" +" AtSetpoint:", atSetpoint());
        SmartDashboard.putBoolean("["+ key +"]" +" AtGoal:", atGoal());
        SmartDashboard.putNumber("["+ key +"]" +" Measurement:", measurement);
        SmartDashboard.putNumberArray("["+ key +"]" +"Gains:", gains.toArray());
      
    }

    /**
     * Gets the current Gains used for the controller.
     * @return the current Gains
     */
    public Gains getGains(){
        return gains;
    }

    /**
     * Sets the output to overcome gravity
     * @param kG the gravity coefficient
     */
    public void setG(double kG){
        this.gains.kG = kG;
        setGains(gains);
    }

    /**
     * Sets the output to overcome static friction
     * @param kS the coefficient
     */
    public void setS(double kS){
        this.gains.kS = kS;
        setGains(gains);
    }

    /**
     * Sets the output to overcome acceleration
     * @param kA the coefficient
     */
    public void setA(double kA){
        this.gains.kA = kA;
        setGains(gains);
    }

    /**
     * Sets the output to overcome velocity
     * @param kV the coefficient
     */
    public void setV(double kV){
        this.gains.kV = kV;
        setGains(gains);
    }

    /**
     * Sets all the gains parameters. All the coeficients and constraints
     * @param newGains the new gains to use
     */
    public void setGains(MotionModel.Gains newGains){
        this.gains = newGains;
        setPID(gains.kP, gains.kI, gains.kD);
        setConstraints(new TrapezoidProfile.Constraints(gains.kVelocity, gains.kAcceleration));
    }

    /**
     * Returns the next output of the PID controller.
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    @Override
    public double calculate(double measurement){
        this.measurement = measurement;
        this.output = super.calculate(measurement);
        return output;
    }

    /**
     * Returns the next output of the PID controller.
     * @param CurrentVelocity The current velocity of the process variable to use FeedForward.
     * @return The FeedForward ouput.
     */
    public double calculate(VelocityGoal CurrentVelocity){

        double sign = Math.signum(CurrentVelocity.velocity);

        this.feedforwardOutput = 
            sign * gains.kS +
            gains.kV * CurrentVelocity.velocity +
            gains.kG +
            gains.kA * CurrentVelocity.acceleration; 

        return feedforwardOutput;
    }
    
    /**
     * Returns the next output of the PID controller.
     * @param measurement The current measurement of the process variable.
     * @param Goal The new goal of the controller.
     * @param CurrentVelocity The current velocity of the process variable to use FeedForward.
     * @return The controller's next output.
     */
    public double calculate(double measurement, State Goal, VelocityGoal CurrentVelocity){
        return calculate(measurement, Goal) + calculate(CurrentVelocity);
    }

    /**
     * Returns the next output of the PID controller.
     * @param measurement The current measurement of the process variable.
     * @param CurrentVelocity The current velocity of the process variable to use FeedForward.
     * @return The controller's next output.
     */
    public double calculate(double measurement, VelocityGoal CurrentVelocity){
        return calculate(measurement) + calculate(CurrentVelocity);
    }

}
