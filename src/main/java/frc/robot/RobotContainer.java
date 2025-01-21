// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NeoSwerve;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.MapleSimSwerve;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer {
  SwerveDrive m_swerveSubsystem;

  CommandJoystick m_driverJoystick;
  CommandXboxController m_xboxController;

  enum ControlMode{// enum to switch between two player, and one player on the joystick and xbox controller
    TWO_PLAYER,
    XBOX,
    JOYSTICK
  };

  public ControlMode setControlMode = ControlMode.JOYSTICK;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      m_swerveSubsystem = new NeoSwerve();
    } else {
      m_swerveSubsystem = new MapleSimSwerve();
    }

    m_driverJoystick = new CommandJoystick(0);

    m_xboxController = new CommandXboxController(1); 

    switch (setControlMode) {
      case TWO_PLAYER: {
        configureBindingsTwoPlayer();
      }
      case XBOX: {
        configureBindingsXboxController();
      }
      case JOYSTICK: {
        configureBindingsJoystick();
      }
    }
  }

  private void configureBindingsXboxController() {
    Trigger fieldOrientedTrigger = new Trigger(m_xboxController.start());

    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
      m_swerveSubsystem,
      () -> m_xboxController.getLeftY(),
      () -> m_xboxController.getLeftX(),
      () -> m_xboxController.getRightTriggerAxis(),
      () -> !fieldOrientedTrigger.getAsBoolean())); // will be robot-centric if held down
  }
  
  private void configureBindingsJoystick() {
    Trigger fieldOrientedTrigger = new Trigger(m_driverJoystick.button(12));

    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
      m_swerveSubsystem,
      () -> m_driverJoystick.getY(),
      () -> m_driverJoystick.getX(),
      () -> m_driverJoystick.getZ(),
      () -> !fieldOrientedTrigger.getAsBoolean())); // will be robot-centric if held down
  }

  private void configureBindingsTwoPlayer() {
    Trigger fieldOrientedTrigger = new Trigger(m_driverJoystick.button(17));

    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
      m_swerveSubsystem,
      () -> m_driverJoystick.getY(),
      () -> m_driverJoystick.getX(),
      () -> m_driverJoystick.getZ(),
      () -> !fieldOrientedTrigger.getAsBoolean())); // will be robot-centric if held down
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
