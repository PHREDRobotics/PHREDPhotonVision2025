// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.proto.Photon;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveReset;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * Calls the commands and such
 * 🚌
 */
public class RobotContainer {
  PhotonVisionSubsystem m_photonVisionSubsystem;
  SwerveSubsystem m_swerveSubsystem;

  Input m_input;

  boolean altCameraSwitch = false;

  public enum AutoSwitcher { // enum to switch between different auto modes
    OFF_THE_LINE,
    LeftScore1,
    LeftScore2,
    LeftScore3,
    CenterScore1,
    RightScore1,
    RightScore2,
    RightScore3
  }

  public RobotContainer() {
    m_photonVisionSubsystem = new PhotonVisionSubsystem();
    m_swerveSubsystem = new SwerveSubsystem(m_photonVisionSubsystem);

    m_input = new Input();

    registerPathPlannerCommands();
    configureBindings();
  }

  private void configureBindings() {

    // Joystick
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
        m_swerveSubsystem,
        () -> m_input.getDriveY(),
        () -> m_input.getDriveX(),
        () -> m_input.getDriveZ(),
        () -> m_input.getThrottle(),
        () -> !m_input.getFieldOriented().getAsBoolean())); // will be robot-centric if held down

    m_input.getSwerveReset().onTrue(new SwerveReset(m_swerveSubsystem));
  }

  public void registerPathPlannerCommands() {
  }

  public boolean getCameraButton() {
    if (m_input.getCameraSwitch().getAsBoolean()) {
      altCameraSwitch = !altCameraSwitch;
    }
    return altCameraSwitch;
  }

  /**
   * @param autoMode
   * @return Command
   */
  public Command getAutonomousCommand(AutoSwitcher autoMode) {
    switch (autoMode) {
      default:
      case OFF_THE_LINE: {
        return new PathPlannerAuto("Basic off-the-line auto");
      }
      case LeftScore1: {
        return new PathPlannerAuto("Left Score 1");
      }
      case LeftScore2: {
        return new PathPlannerAuto("Left Score 2");
      }
      case LeftScore3: {
        return new PathPlannerAuto("Left Score 3");
      }
      case CenterScore1: {
        return new PathPlannerAuto("Center Score 1");
      }
      case RightScore1: {
        return new PathPlannerAuto("Right Score 1");
      }
      case RightScore2: {
        return new PathPlannerAuto("Right Score 2");
      }
      case RightScore3: {
        return new PathPlannerAuto("Right Score 3");
      }
    }
  }
}
