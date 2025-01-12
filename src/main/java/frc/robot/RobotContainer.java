// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NeoSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.MapleSimSwerve;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer {
  SwerveDrive m_swerveSubsystem;

  CommandJoystick m_driverJoystick;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      m_swerveSubsystem = new NeoSwerve();
    } else {
      m_swerveSubsystem = new MapleSimSwerve();
    }

    m_driverJoystick = new CommandJoystick(0);

    configureBindings();
  }

  private void configureBindings() {
    Trigger fieldOrientedTrigger = new Trigger(m_driverJoystick.button(17));

    m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.driveCommand(
      () -> -m_driverJoystick.getY() * 3,
      () -> -m_driverJoystick.getX() * 3,
      () -> -m_driverJoystick.getZ() / 4,
      () -> fieldOrientedTrigger.getAsBoolean(),
      0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
