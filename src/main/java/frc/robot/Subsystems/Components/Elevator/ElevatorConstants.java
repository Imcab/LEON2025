package frc.robot.Subsystems.Components.Elevator;

import frc.robot.BulukLib.Math.Domain;
import frc.robot.BulukLib.MotionControllers.Gains.TrapezoidalGains;
import frc.robot.BulukLib.MotionControllers.TrapezoidalControl.TrapezoidalTolerance;

public class ElevatorConstants {

    public static final Domain physicalCapacity = new Domain(0.62, 1.96); //Metros
    public static final Domain ground_l2 = new Domain(0.62, 0.87); //Metros
    public static final Domain l2_l3 = new Domain(0.88, 1.26); //Metros
    public static final Domain l3_l4 = new Domain(1.27, 1.96); //Metros

    public enum ElevatorType{
        kMTY, kLEON
    }

    public static final TrapezoidalTolerance DEV_TOLERANCE = new TrapezoidalTolerance(0.02, 0.08);
    public static final TrapezoidalTolerance COMP_TOLERANCE = new TrapezoidalTolerance(0.015, 0.05);

    //Definir por niveles del elevador
    public static final double[] COMPgravityFactor = new double[]{0,0,0};
    public static final double[] COMPaccFactor = new double[]{0,0,0}; 
    public static final double[] COMPstaticFriction = new double[]{0,0,0}; 
    
    public static final double COMPmaxVelocity = 0.69; //MS (DEFINIR)
    public static final double COMPmaxAcc = 0.8625; //MS_Squared (DEFINIR)
    public static final double DEVmaxVelocity = 0.69; //MS
    public static final double DEVmaxAcc = 0.8625; //MS_Squared

    public static final TrapezoidalGains DEVGains = new TrapezoidalGains(7.5, 0, 0.0, 0, 0, DEVmaxAcc, DEVmaxVelocity);
    public static final TrapezoidalGains COMPGains = new TrapezoidalGains(7.5, 0, 0.0, 0, 0, COMPmaxAcc, COMPmaxVelocity);

    public static final int CAN_ID_LEADER = 15;
    public static final int CAN_ID_SLAVE = 16;

    public static final boolean leaderInverted = false;
    public static final boolean slaveInverted = true;

    public static final double CONVERSION_FACTOR = (120.9 / 20.85) / 3; 
    public static final double ELEVATOR_OFFSET_CENTIMETERS = 63;
}
