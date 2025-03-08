// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AngleWrist;
import frc.robot.Commands.Feed;
import frc.robot.Commands.moveBack;
import frc.robot.Commands.movePosition;
import frc.robot.Commands.moveVelocity;
import frc.robot.Commands.moveVoltage;
import frc.robot.Commands.swerve.AprilSnap;
import frc.robot.Commands.swerve.DriveCommands;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.Climber;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Drive.swerve;


public class RobotContainer {
  
  //public Climber climber;
  public ElevatorSubsystem elevator = new ElevatorSubsystem("ELEVATOR");
  public CommandXboxController driver = new CommandXboxController(0);
  public swerve drive = new swerve();
  public CoralWrist coralWrist = new CoralWrist();

  public RobotContainer() {
    
    //climber = new Climber();
    configureBindings();
  }

  private void configureBindings() {

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, ()-> driver.getLeftY() * 0.8, ()-> driver.getLeftX() * 0.8, ()-> -driver.getRightX() * 0.7));

    /*driver.a().whileTrue(new moveBack(elevator, ()-> 0.63, coralWrist, -2));
    driver.b().whileTrue(new movePosition(elevator, ()-> 0.91, coralWrist, 63));                                                        
    driver.y().whileTrue(new movePosition(elevator, ()-> 1.87, coralWrist, 47));
    driver.x().whileTrue(new movePosition(elevator, ()-> 1.32,coralWrist,  63));
    driver.povLeft().whileTrue(new movePosition(elevator, ()-> 0.83, coralWrist, 15.3472));
    driver.povDown().whileTrue(new InstantCommand(()-> elevator.stop(), elevator));
    driver.rightBumper().whileTrue(new StartEndCommand(()-> coralWrist.wheelSpeed(0.6), ()-> coralWrist.wheelSpeed(0), coralWrist));
    driver.leftBumper().whileTrue(new Feed(elevator, ()-> 0.83, coralWrist, 15.3472));*/

    driver.a().whileTrue(new moveBack(elevator, ()-> 0.63, coralWrist, -2));
    driver.y().whileTrue(new movePosition(elevator, ()-> 1.87, coralWrist, 47));
    driver.rightBumper().whileTrue(new StartEndCommand(()-> coralWrist.wheelSpeed(0.6), ()-> coralWrist.wheelSpeed(0), coralWrist));
    driver.x().whileTrue(new movePosition(elevator, ()-> 1.28,coralWrist,  63));
    driver.b().whileTrue(new AprilSnap(drive, ()-> driver.getLeftY()));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
