// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.movePosition;
import frc.robot.Commands.moveVelocity;
import frc.robot.Commands.moveVoltage;
import frc.robot.Commands.swerve.DriveCommands;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.Components.Climber;
import frc.robot.Subsystems.Drive.swerve;


public class RobotContainer {
  
  //public Climber climber;
  public ElevatorSubsystem elevator = new ElevatorSubsystem("ELEVATOR");
  public CommandXboxController driver = new CommandXboxController(0);
  public swerve drive = new swerve();

  public RobotContainer() {
    
    //climber = new Climber();
    configureBindings();
  }

  private void configureBindings() {

    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, ()-> driver.getLeftY() * 0.8, ()-> driver.getLeftX() * 0.8, ()-> -driver.getRightX() * 0.7));

    elevator.handleState(()-> driver.back().getAsBoolean());

    driver.a().whileTrue(new movePosition(elevator, ()-> 0.63));
    driver.b().whileTrue(new movePosition(elevator, ()-> 0.89));
    driver.x().whileTrue(new movePosition(elevator, ()-> 1.87));
    driver.y().whileTrue(new movePosition(elevator, ()-> 1.30));
    driver.povLeft().whileTrue(new moveVelocity(elevator, -0.1));
    driver.povDown().whileTrue(new InstantCommand(()-> elevator.stop(), elevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
