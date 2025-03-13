package frc.robot.Subsystems.Components.Elevator;

import frc.robot.BulukLib.Math.Domain;

public class ElevatorConstants {

    public static final Domain physicalCapacity = new Domain(0.62, 1.96); //Metros
 
    public static final double fullMaxVelocity = 1.254;
    public static final double fullMaxAcc = 1.242;

    public static final int CAN_ID_LEADER = 15;
    public static final int CAN_ID_SLAVE = 16;

    public static final boolean leaderInverted = false;
    public static final boolean slaveInverted = true;

    public static final double CONVERSION_FACTOR = (120.9 / 20.85) / 3; 
    public static final double ELEVATOR_OFFSET_CENTIMETERS = 63;
}
