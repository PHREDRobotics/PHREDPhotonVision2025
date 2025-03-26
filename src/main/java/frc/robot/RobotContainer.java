// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.AutoCoralOuttake;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.ExtendLift;
import frc.robot.commands.GoToTag;
import frc.robot.commands.PullGrenadePin;
import frc.robot.commands.RetractLift;
import frc.robot.commands.SwerveReset;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.vision.DistanceSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Calls the commands and such
 */
public class RobotContainer {
  SwerveSubsystem m_swerveSubsystem;
  ClimbSubsystem m_climbSubsystem;
  CoralSubsystem m_coralSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  VisionSubsystem m_visionSubsystem;
  DistanceSubsystem m_distanceSubsystem;

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
    m_swerveSubsystem = new SwerveSubsystem();
    m_coralSubsystem = new CoralSubsystem();
    m_climbSubsystem = new ClimbSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_distanceSubsystem = new DistanceSubsystem();

    m_input = new Input();

    registerPathPlannerCommands();
    configureBindings();
  }

  private void configureBindings() {
    // Xbox con
    m_input.getIntake().whileTrue(new CoralIntakeCommand(m_coralSubsystem));
    m_input.getOuttake().onTrue(new CoralOuttakeCommand(m_coralSubsystem, m_elevatorSubsystem));
    m_input.getExtendLift().onTrue(new ExtendLift(m_climbSubsystem));
    m_input.getRetractLift().onTrue(new RetractLift(m_climbSubsystem));

    m_input.getL1().onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel1, m_elevatorSubsystem)));
    m_input.getL2().onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kHumanPlayerStationLevel, m_elevatorSubsystem)));
    m_input.getL3().onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel3, m_elevatorSubsystem)));
    m_input.getL4().onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel4, m_elevatorSubsystem)));

    // Joystick
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
        m_swerveSubsystem,
        () -> m_input.getDriveY(),
        () -> m_input.getDriveX(),
        () -> m_input.getDriveZ(),
        () -> m_input.getThrottle(),
        () -> !m_input.getFieldOriented().getAsBoolean())); // will be robot-centric if held down

    m_input.getSwerveReset().onTrue(new SwerveReset(m_swerveSubsystem));
    m_input.getGoToTag().whileTrue(new GoToTag(m_visionSubsystem, m_swerveSubsystem, 12));
  }

  public void registerPathPlannerCommands() {
    NamedCommands.registerCommand("ElevatorResetCommand",
        new ResetElevator(m_elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL1Command",
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel1, m_elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL3Command",
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel3, m_elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorL4Command",
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel4, m_elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorHPCommand",
        new AutoElevatorCommand(Constants.ElevatorConstants.kHumanPlayerStationLevel, m_elevatorSubsystem));
    NamedCommands.registerCommand("CoralOuttakeCommand",
        new AutoCoralOuttake(m_coralSubsystem));
    NamedCommands.registerCommand("CoralIntakeCommand",
        new CoralIntakeCommand(m_coralSubsystem));
    NamedCommands.registerCommand("PullGrenadePin",
        new PullGrenadePin(m_elevatorSubsystem));
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
