package frc.robot.BulukLib.MotionModel;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.BulukLib.MotionModel.MotionModel.VelocityGoal;

public class ScurveGenerator {

    private MotionModel.VelocityGoal maximum;
    private double currentVel = 0;
    private double currentAcc = 0;
    private double jerk;
    private double lastTime;
    
    public ScurveGenerator(VelocityGoal maximum, double jerk) {

        this.maximum = maximum;
        this.jerk = jerk;
        this.lastTime = Timer.getFPGATimestamp();

    }

    public MotionModel.VelocityGoal generate() {

        double now = Timer.getFPGATimestamp();
        double elapsedTime = now - lastTime;

        if (elapsedTime < 0) {
            elapsedTime = 0;
        }

        if (currentAcc < maximum.acceleration) {
            currentAcc += jerk * elapsedTime;
            currentAcc = Math.min(currentAcc, maximum.acceleration);
        }

        if (currentVel < maximum.velocity) {
            currentVel += currentAcc * elapsedTime;
            currentVel = Math.min(currentVel, maximum.velocity);
        }

        lastTime += elapsedTime;

        return new MotionModel.VelocityGoal(currentVel, currentAcc);
    }
}
