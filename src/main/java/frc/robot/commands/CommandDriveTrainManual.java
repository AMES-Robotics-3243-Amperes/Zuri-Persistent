// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemDriveTrain;

public class CommandDriveTrainManual extends CommandBase {

  XboxController m_controller;
  SubsystemDriveTrain m_subsystem;


  /** Creates a new DriveTrainManualCommand. */
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public CommandDriveTrainManual(XboxController controller, SubsystemDriveTrain subsystem) {
    m_controller = controller;
    m_subsystem = subsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setCartesianVelocityRaw(m_controller.getLeftX(), m_controller.getLeftY(), m_controller.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
