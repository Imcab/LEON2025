// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Components.Elevator.Elevator;
import frc.robot.Subsystems.Components.Elevator.ElevatorBuilder;
import frc.robot.Subsystems.Components.Elevator.ElevatorConstants.ElevatorType;

public class RobotContainer {
  
  public Elevator elevator;

  public RobotContainer() {

    elevator = new Elevator(new ElevatorBuilder(ElevatorType.kLEON));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
