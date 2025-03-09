// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.AlgaeCommands.AlgaeCommands;
import frc.robot.Commands.Auto.RiseAuto;
import frc.robot.Commands.Auto.feedAuto;
import frc.robot.Commands.Auto.retractAuto;
import frc.robot.Commands.DriveCommands.DriveCommands;
import frc.robot.Commands.ElevatorCommands.joystickElevator;
import frc.robot.Commands.ElevatorCommands.moveBack;
import frc.robot.Commands.ElevatorCommands.movePosition;
import frc.robot.Commands.RoutinesCommands.Feed;
import frc.robot.Commands.WristCommands.AngleWrist;
import frc.robot.Commands.WristCommands.ManualWrist;
import frc.robot.Commands.WristCommands.wristSpeed;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.REVBlinkin;
import frc.robot.Subsystems.REVBlinkin.PatternType;
import frc.robot.Subsystems.Components.AlgaeWrist;
import frc.robot.Subsystems.Components.CoralWrist;
import frc.robot.Subsystems.Drive.swerve;

public class RobotContainer {
  
  //public Climber climber;
  public ElevatorSubsystem elevator = new ElevatorSubsystem("ELEVATOR");
  public CommandXboxController driver = new CommandXboxController(0);
  public CommandXboxController operator = new CommandXboxController(1);
  public REVBlinkin ledStrip = new REVBlinkin(1);
  public swerve drive = new swerve(ledStrip, driver);
  public CoralWrist coralWrist = new CoralWrist();
  public AlgaeWrist algae = new AlgaeWrist();
  

  public RobotContainer() {
    
    //climber = new Climber();
    configureBindings();
  }

  private void configureBindings() {

    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, ()-> driver.getLeftY() * 0.8, ()-> driver.getLeftX() * 0.8, ()-> -driver.getRightX() * 0.7));
    driver.leftTrigger(0.5).whileTrue(DriveCommands.joystickDrive(drive,()-> driver.getLeftY() * 0.4,()-> driver.getLeftX() * 0.4,()-> driver.getRightX() * 0.3));
    driver.x().toggleOnTrue(DriveCommands.snapToApril(drive, ()-> driver.getLeftY()));
    driver.b().toggleOnTrue(DriveCommands.rotateToApril(drive, ()-> driver.getLeftX(), ()-> driver.getLeftY()));

    driver.start().whileTrue(DriveCommands.resetHeading(drive));
    driver.povLeft().whileTrue(DriveCommands.moveInX(drive, 0.6));
    driver.povRight().whileTrue(DriveCommands.moveInX(drive, -0.6));
    driver.povUp().whileTrue(DriveCommands.moveInY(drive, -0.6));
    driver.povDown().whileTrue(DriveCommands.moveInY(drive, 0.5));
    //---------------------------------------------------------------- DRIVER ----------------------------------------------------------------

    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------
    operator.rightBumper().whileTrue(new Feed(elevator, ()-> 0.83, coralWrist, 15.3472, drive));
    operator.rightTrigger().whileTrue(new StartEndCommand(()-> coralWrist.wheelSpeed(0.6), ()-> coralWrist.wheelSpeed(0), coralWrist));
    operator.leftBumper().whileTrue(AlgaeCommands.noStopAlgae(algae, 62, 0.5));
    operator.leftTrigger().whileTrue(AlgaeCommands.shootAlgae(algae, -0.5));

    operator.a().whileTrue(new moveBack(elevator, ()-> 0.63, coralWrist, -2, drive));
    operator.b().toggleOnTrue(new movePosition(elevator, ()-> 0.91, coralWrist, 63, drive));    
    operator.x().toggleOnTrue(new movePosition(elevator, ()-> 1.28,coralWrist,  63, drive));                                                    
    operator.y().toggleOnTrue(new movePosition(elevator, ()-> 1.87, coralWrist, 47, drive));

    operator.povUp().whileTrue(new AngleWrist(coralWrist, -2));

    operator.leftStick().whileTrue(new joystickElevator(elevator, ()-> -operator.getLeftY() * 0.2));
    operator.rightStick().whileTrue(new ManualWrist(coralWrist, ()-> operator.getRightY() * 0.1));
    //---------------------------------------------------------------- OPERATOR ----------------------------------------------------------------
    
    NamedCommands.registerCommand("out", new wristSpeed(coralWrist, 0.7).withTimeout(0.6));
    NamedCommands.registerCommand("ForwardFast", DriveCommands.moveInY(drive, 0.3).withTimeout(0.3));
    NamedCommands.registerCommand("Retract", new retractAuto(elevator, coralWrist));
    NamedCommands.registerCommand("L4", new RiseAuto(elevator, coralWrist, ()-> 1.87, 45));
    NamedCommands.registerCommand("feedAuto", new feedAuto(elevator, ()-> 0.83, coralWrist, 15.3472, drive));

  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("L4Right");
  }
}
