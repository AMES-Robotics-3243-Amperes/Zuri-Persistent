// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CommandDriveTrainManual;
import frc.robot.subsystems.SubsystemDriveTrain;

public class RobotContainer {

  // #######  PERIPHERALS #######
  XboxController controllerPrimary = new XboxController(0);

  // #######  SUBSYTEMS  #######
  SubsystemDriveTrain subsystemDriveTrain = new SubsystemDriveTrain();


  // #######  COMMANDS  #######
  CommandDriveTrainManual commandDriveTrainManual = new CommandDriveTrainManual(controllerPrimary, subsystemDriveTrain);



  public RobotContainer() {
    configureBindings();

    subsystemDriveTrain.setDefaultCommand(commandDriveTrainManual);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
