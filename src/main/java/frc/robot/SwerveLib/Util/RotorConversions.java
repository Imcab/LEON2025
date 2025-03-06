package frc.robot.SwerveLib.Util;

/**
 * Converts rotor values to wheel values (Swerve Utils)
 */
public class RotorConversions{

    private RotorConversions() {}

    public static double rotorRotationsToMeters(double rotations, double reduction, double WheelRadiusInMeters) {
        return rotations * (2 * Math.PI * WheelRadiusInMeters) / reduction;
    }

    public static double rotorRPMtoMetersPerSec(double RPM, double reduction, double WheelRadiusInMeters) {
        return (RPM * (2 * Math.PI * WheelRadiusInMeters)) / (60 * reduction);
    }

    public static double rotorRotationsToWheelRadians(double rotations, double reduction) {
        return (rotations * 2 * Math.PI) / reduction;
    }

    public static double rotorRotationsToWheelRadPerSec(double RPM, double reduction) {
        return (RPM * 2 * Math.PI) / (60 * reduction);
    }

    public static double metersPerSecToWheelRadPerSec(double mps, double WheelRadiusInMeters) {
        return mps / WheelRadiusInMeters;
    }

    public static double metersToWheelRadians(double meters, double WheelRadiusInMeters) {
        return meters / WheelRadiusInMeters;
    }

    public static double metersPerSecToRotorRPM(double mps, double reduction, double WheelRadiusInMeters) {
        return (mps * 60 * reduction) / (2 * Math.PI * WheelRadiusInMeters);
    }

    public static double radiansToWheelRotations(double radians, double reduction) {
        return (radians * reduction) / (2 * Math.PI);
    }

    public static double radiansPerSecToWheelRPM(double radPerSec, double reduction) {
        return (radPerSec * 60 * reduction) / (2 * Math.PI);
    }
}
