package frc.robot.SwerveLib.Dashboard;

import java.util.function.DoubleSupplier;

public class ModuleStateLog{

    public DoubleSupplier Angle;
    public DoubleSupplier Speed;

    public ModuleStateLog(DoubleSupplier moduleAngle, DoubleSupplier moduleSpeed){
        this.Angle = moduleAngle;
        this.Speed = moduleSpeed;
    }

}
