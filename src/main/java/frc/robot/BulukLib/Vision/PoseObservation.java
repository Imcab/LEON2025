package frc.robot.BulukLib.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PoseObservation {
    Pose2d pose;
    double timeStamps;
    Matrix <N3,N1> std;
    public PoseObservation(Pose2d pose, double timeStamps){
        this.pose = pose;
        this.timeStamps = timeStamps;
        this.std = VecBuilder.fill(0,0,0);
    }
    public PoseObservation(Pose2d pose, double timeStamps, Matrix<N3,N1> std){
        this.pose = pose;
        this.timeStamps = timeStamps;
        this.std = std;
    }
    public Pose2d observation(){
        return pose;
    }
    public double withTimeStamps(){
        return timeStamps;
    }
    public Matrix<N3,N1> getTrust(){
        return std;
    }
    
}
