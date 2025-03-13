package frc.robot;

import frc.robot.BulukLib.MotionControllers.Gains.Gains;

public class Constants {
    
    public class ConstantsHanger {

        public static final int RightHangerPort = 17;
        public static final int LeftHangerPort = 18;
    }

  public class WristConstants {
  
    public class Algae {

        public static final double tolerance = 5;
        public static final int CAN_ID_WRIST = 13;
        public static final int CAN_ID_RIGHTWHEEL = 12;
        public static final int CAN_ID_LEFTWHEEL = 14;
        public static final boolean wristMotorInverted = false;
        public static final boolean RightInverted = false;
        public static final boolean LeftInverted = true;
        public static final int wristCurrentLimit = 30;
        public static final int WheelsCurrentLimit = 25;
        public static final Gains GAINS = new Gains(0.005,0,0);

    }

    public class Coral {

        public static final int DIO_PORT_SENSOR = 1;
        public static final int CAN_ID_WRIST = 10;
        public static final int CAN_ID_EATER = 11;
        public static final boolean wristMotorInverted = false;
        public static final boolean wheelInverted = false;
        public static final int wristCurrentLimit = 40;
        public static final int wheelCurrentLimit = 15;
        public static final Gains GainsUp = new Gains(0.004,0.0,0.0003);
        public static final Gains GainsDown = new Gains(0.005,0.0,0.00037);
        public static final double encoderPositionFactor = 360; //degrees
        public static final double wristErrorTolerance = 0.1;  
    }
  }

  public class DriveConstants {

    public static final Gains driveGains = new Gains(0.015, 0, 0, 0.05, 0.06); //0.13
    public static final Gains turnGains = new Gains(5.0, 0, 0);
  
    public static final class frontLeft{

        public static final int DrivePort = 1; 
        public static final int TurnPort = 2; 
        public static final int EncPort = 19;
        public static final double offset = 0.08;                                                                                                                                                                                                                                                                                                                                                                                                                                                                        ; //48     //93  //138      //48 o 138 o 228
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class frontRight{

        public static final int DrivePort = 4; 
        public static final int TurnPort = 3; 
        public static final int EncPort = 20; 
        public static final double offset = -0.47; 
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

    public static final class backLeft{

        public static final int DrivePort = 5; 
        public static final int TurnPort = 6; 
        public static final int EncPort = 21; 
        public static final double offset = 0.49;
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true; 

    }

    public static final class backRight{

        public static final int DrivePort = 7; 
        public static final int TurnPort = 8; 
        public static final int EncPort = 22; 
        public static final double offset = -0.43; 
 
        public static final boolean DrivemotorReversed = true;
        public static final boolean TurnmotorReversed = true;

    }

}

}
