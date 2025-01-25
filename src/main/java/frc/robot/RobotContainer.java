// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NeoSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.MapleSimSwerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer {
  SwerveDrive m_swerveSubsystem;

  CommandJoystick m_driverJoystick;
  CommandXboxController m_xboxController;

  public enum AutoSwitcher { // enum to switch between different auto modes
    OFF_THE_LINE
  }

  public RobotContainer() {
    if (RobotBase.isReal()) {
      m_swerveSubsystem = new NeoSwerve();
    } else {
      m_swerveSubsystem = new MapleSimSwerve();
    }

    m_driverJoystick = new CommandJoystick(0);

    m_xboxController = new CommandXboxController(1); 

    configureBindings();
  }

  private void configureBindings() {
    // Triggers
    Trigger fieldOrientedTrigger = m_driverJoystick.button(2);

    // Axes
    DoubleSupplier driveAxis = () -> m_driverJoystick.getY();
    DoubleSupplier strafeAxis = () -> m_driverJoystick.getX();
    DoubleSupplier turnAxis = () -> m_driverJoystick.getZ();

    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
      m_swerveSubsystem,
      driveAxis,
      strafeAxis,
      turnAxis,
      () -> !fieldOrientedTrigger.getAsBoolean())); // will be robot-centric if held down
  }
  
  public Command getAutonomousCommand(AutoSwitcher autoMode) {
    switch (autoMode) {
      default: case OFF_THE_LINE: {
        return new PathPlannerAuto("Basic off-the-line auto");
      }
    }
  }
}