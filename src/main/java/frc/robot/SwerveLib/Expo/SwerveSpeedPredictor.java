package frc.robot.SwerveLib.Expo;

public class SwerveSpeedPredictor {

    private double kF;
    private double lastSpeed = Double.NaN;

    /**
     * Creates a swerve wheel speed predictor. Lower kF resulting on more reactive responses and lower on more stable
     * @param kF the smoothing factor (must be between 0 and 1).
     */
    public SwerveSpeedPredictor(double kF){
        setSmoothingFactor(kF);
    }

    /**
     * Updates the smoothing factor with a new value.
     * @param newSmoothingFactor the new smoothing factor (must be between 0 and 1).
     * @throws IllegalArgumentException if the value is out of range.
     */
    public void setSmoothingFactor(double newSF){
        if (newSF < 0 || newSF > 1) {
            throw new IllegalArgumentException("[SwerveSpeedPredictor]: Smoothing Factor must be between 0 and 1!");
        }

        this.kF = newSF;
    }

    public double getSmoothingFactor() {
        return kF;
    }
    
    /**
     * Predicts the next swerve wheel speed based on the current speed.
     * Implements exponential smoothing.
     * @param currentSpeed the current wheel speed.
     * @return the predicted next speed.
     */
    public double predictNextSpeed(double currentSpeed) {

        if (Double.isNaN(lastSpeed)) {
            lastSpeed = currentSpeed;
        }
        lastSpeed = kF * currentSpeed + (1 - kF) * lastSpeed;
        return lastSpeed;
    }

     /**
     * Returns the last known speed.
     * @return the last predicted speed.
     */
    public double lastKnownSpeed(){
        return lastSpeed;
    }

    
}
