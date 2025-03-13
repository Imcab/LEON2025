


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.Components.Climber;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ClimberCommand;
import frc.robot.Commands.AlgaeCommands.AlgaeCommands;
import frc.robot.Commands.Auto.RiseAuto;
import frc.robot.Commands.Auto.feedAuto;
import frc.robot.Commands.Auto.retractAuto;
import frc.robot.Commands.DriveCommands.DriveCommands;
import frc.robot.Commands.ElevatorCommands.joystickElevator;
import frc.robot.Commands.ElevatorCommands.moveBack;
import frc.robot.Commands.ElevatorCommands.movePosition;
import frc.robot.Commands.Reset.ResetUtils;
import frc.robot.Commands.RoutinesCommands.Feed;
import frc.robot.Commands.WristCommands.AngleWrist;
import frc.robot.Commands.WristCommands.ManualWrist;
import frc.robot.Commands.WristCommands.wristSpeed;
import frc.robot.Subsystems.Components.AlgaeWrist;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Components.Elevator.ElevatorSubsystem;
import frc.robot.Subsystems.Drive.swerve;
import frc.robot.Subsystems.Hardware.REVBlinkin;

public class RobotContainer {
  
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  private ElevatorSubsystem elevator = new ElevatorSubsystem("Elevator");
  private REVBlinkin ledStrip = new REVBlinkin(1);
  private swerve drive = new swerve(ledStrip, driver);
  private CoralWrist coralWrist = new CoralWrist();
  private AlgaeWrist algae = new AlgaeWrist();
  private Climber climb = new Climber();
  private PathPlannerAuto doubleL4;

  private SendableChooser<Command> autChooser = new SendableChooser<>();
  
  public RobotContainer() {

    NamedCommands.registerCommand("Confirm", DriveCommands.snapToApril(drive, ()-> 0));
    NamedCommands.registerCommand("out", new wristSpeed(coralWrist, 0.7).withTimeout(0.7));
    NamedCommands.registerCommand("ForwardFast", DriveCommands.moveInY(drive, 0.3).withTimeout(0.35).finallyDo(drive::stop));
    NamedCommands.registerCommand("BackFast", DriveCommands.moveInY(drive, -0.3).withTimeout(0.15).finallyDo(drive::stop));
    NamedCommands.registerCommand("Forward", DriveCommands.moveInY(drive, 0.3).withTimeout(0.5).finallyDo(drive::stop));
    NamedCommands.registerCommand("Retract", new retractAuto(elevator, coralWrist));
    NamedCommands.registerCommand("L4", new RiseAuto(elevator, coralWrist, ()-> 1.87, 45, drive));
    NamedCommands.registerCommand("L3", new RiseAuto(elevator,  coralWrist, ()-> 1.31,  63, drive));
    NamedCommands.registerCommand("feedAuto", new feedAuto(elevator, ()-> 0.83, coralWrist, 15.3472, -0.6, drive));
    NamedCommands.registerCommand("feedPrep", new feedAuto(elevator, ()-> 0.83, coralWrist, 15.3472, -0.05,drive));

    doubleL4 = new PathPlannerAuto("L4Right");

    autChooser.setDefaultOption("2piezasL4", doubleL4);

    SmartDashboard.putData("AutoSelector", autChooser);
    
    configureBindings();
  }

  private void configureBindings() {

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, ()-> -driver.getLeftY() * 0.8, ()-> -driver.getLeftX() * 0.8, ()-> -driver.getRightX() * 0.7));
    driver.leftTrigger(0.5).whileTrue(DriveCommands.joystickDrive(drive,()-> -driver.getLeftY() * 0.4,()-> -driver.getLeftX() * 0.4,()-> -driver.getRightX() * 0.3));
    driver.b().whileTrue(DriveCommands.snapToApril(drive, ()-> driver.getLeftY()));
    driver.x().whileTrue(DriveCommands.rotateToApril(drive, ()-> driver.getLeftX(), ()-> driver.getLeftY()));
    //driver.y().whileTrue(DriveCommands.uploadPose(drive, new Coordinate(0.5, 1.66, -180)));

    driver.start().whileTrue(DriveCommands.resetHeading(drive));
    driver.povLeft().whileTrue(DriveCommands.moveInX(drive, 0.6));
    driver.povRight().whileTrue(DriveCommands.moveInX(drive, -0.6));
    driver.povUp().whileTrue(DriveCommands.moveInY(drive, 0.6));
    driver.povDown().whileTrue(DriveCommands.moveInY(drive,- 0.6));

    driver.rightBumper().whileTrue(new ClimberCommand(climb, 1, drive));
    driver.leftBumper().whileTrue(new ClimberCommand(climb, -1, drive));

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------

    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------

    operator.back().whileTrue(ResetUtils.resetElevatorEncoders(elevator));

    operator.start().whileTrue(ResetUtils.resetWristEncoder(coralWrist));

    operator.rightBumper().whileTrue(new Feed(elevator, ()-> 0.83, coralWrist, 15.3472, drive, driver, operator));
    operator.rightTrigger().whileTrue(new wristSpeed(coralWrist, 0.6));
    operator.leftBumper().whileTrue(AlgaeCommands.noStopAlgae(algae, 62, 0.5));
    operator.leftTrigger().whileTrue(AlgaeCommands.shootAlgae(algae, -0.5));
    operator.b().toggleOnTrue(new movePosition(elevator, ()-> 0.91, coralWrist, 63, drive));    
    operator.x().toggleOnTrue(new movePosition(elevator, ()-> 1.31,coralWrist,  63, drive));                                                    
    operator.y().toggleOnTrue(new movePosition(elevator, ()-> 1.87, coralWrist, 47, drive));
    operator.a().whileTrue(new moveBack(elevator, ()-> 0.63, coralWrist, -2, drive));

    operator.povLeft().toggleOnTrue(new movePosition(elevator, ()->  1.63-0.05, coralWrist, 10, drive));
    operator.povRight().toggleOnTrue(new movePosition(elevator, ()-> 1.20, coralWrist, 10, drive));

    operator.povUp().whileTrue(new AngleWrist(coralWrist, -2));

    operator.leftStick().whileTrue(new joystickElevator(elevator, ()-> -operator.getLeftY() * 0.2));
    operator.rightStick().whileTrue(new ManualWrist(coralWrist, ()-> operator.getRightY() * 0.1));
    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------
    

  }

  public Command getAutonomousCommand() {
    return doubleL4;
  }
}
