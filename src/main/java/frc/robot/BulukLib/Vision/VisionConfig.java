package frc.robot.BulukLib.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.BulukLib.Swerve.SwerveConfig;
import frc.robot.BulukLib.Util.QoLUtil;

public class VisionConfig {
    
    public static class limelight {
        public static final String name = "limelight-buluk";
        public static final int forwardCoefficient = -1; // if the robot never turns in the correct direction, kP should be inverted.
        public static final int aimCoefficient = -1;
        public static final int translationCoeficient = -1;
        public static final boolean useMegatag2 = true;
        public static final double forwardKp = 0.25; //0.2
        public static final double angularKp = 0.009; //0.01158
        public static final double translationKp = 0.01; 
        public static final boolean useTAforRange = true;
        public static final Matrix<N3,N1> trust = VecBuilder.fill(0.7, 0.7, 99999);

        public static final double TrackMaxSpeed = QoLUtil.percentageOf(60, SwerveConfig.speeds.MAX_LINEAR_SPEED);
        public static final double TrackMaxAngularSpeed = Math.PI;
        
    }
}
