// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.AlignLoadingStationCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorManualLift;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.ExtendLift;
import frc.robot.commands.GoToReef;
import frc.robot.commands.GoToTag;
import frc.robot.commands.PullGrenadePin;
import frc.robot.commands.RetractLift;
import frc.robot.commands.SwerveReset;
import frc.robot.controls.LogitechPro;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DistanceSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  SwerveSubsystem m_swerveSubsystem;
  AlgaeSubsystem m_algaeSubsystem;
  ClimbSubsystem m_climbSubsystem;
  CoralSubsystem m_coralSubsystem;
  ElevatorSubsystem m_elevatorSubsystem;
  VisionSubsystem m_visionSubsystem;
  DistanceSubsystem m_distanceSubsystem;

  LogitechPro m_driverJoystick;
  CommandXboxController m_xboxController;

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
    m_algaeSubsystem = new AlgaeSubsystem();
    m_coralSubsystem = new CoralSubsystem();
    m_climbSubsystem = new ClimbSubsystem();
    m_elevatorSubsystem = new ElevatorSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_distanceSubsystem = new DistanceSubsystem();

    m_driverJoystick = new LogitechPro(0);
    m_xboxController = new CommandXboxController(1);

    // ResetElevatorCommand is the same as the trough
    NamedCommands.registerCommand("ResetElevatorCommand",
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
        new CoralOuttakeCommand(m_coralSubsystem, m_elevatorSubsystem));
    NamedCommands.registerCommand("CoralIntakeCommand",
        new CoralIntakeCommand(m_coralSubsystem));
    NamedCommands.registerCommand("PullGrenadePin",
        new PullGrenadePin(m_elevatorSubsystem));

    configureBindings();
  }

  private void configureBindings() {
    // Triggers
    Trigger trigger = new Trigger(() -> m_driverJoystick.getTrigger());
    Trigger maryButton = new Trigger(() -> m_driverJoystick.getRawButton(2));
    Trigger button3 = new Trigger(() -> m_driverJoystick.getRawButton(3));
    Trigger button4 = new Trigger(() -> m_driverJoystick.getRawButton(4));
    // Trigger button5 = new Trigger(() -> m_driverJoystick.getRawButton(5));
    // Trigger button6 = new Trigger(() -> m_driverJoystick.getRawButton(6));
    // Trigger button7 = new Trigger(() -> m_driverJoystick.getRawButton(7));
    // Trigger button8 = new Trigger(() -> m_driverJoystick.getRawButton(8));
    // Trigger button9 = new Trigger(() -> m_driverJoystick.getRawButton(9));
    // Trigger button10 = new Trigger(() -> m_driverJoystick.getRawButton(10));
    Trigger button11 = new Trigger(() -> m_driverJoystick.getRawButton(11));
    Trigger button12 = new Trigger(() -> m_driverJoystick.getRawButton(12));

    Trigger bButton = m_xboxController.b();
    Trigger aButton = m_xboxController.a();
    Trigger yButton = m_xboxController.y();
    Trigger xButton = m_xboxController.x();
    Trigger startButton = m_xboxController.start();
    Trigger backButton = m_xboxController.back();
    // Trigger leftBumper = m_xboxController.leftBumper();
    // Trigger rightBumper = m_xboxController.rightBumper();

    // Axes
    DoubleSupplier driveAxis = () -> m_driverJoystick.getPitch();
    DoubleSupplier strafeAxis = () -> m_driverJoystick.getRoll();
    DoubleSupplier turnAxis = () -> -m_driverJoystick.getYaw();
    DoubleSupplier throttleAxis = () -> m_driverJoystick.getCorrectedThrottle();

    // D-Pad Buttons
    Trigger dPadDown = m_xboxController.povDown();
    Trigger dPadUp = m_xboxController.povUp();
    Trigger dPadLeft = m_xboxController.povLeft();
    Trigger dPadRight = m_xboxController.povRight();

    // Assign Commands

    // Xbox con
    xButton.onTrue(new AlgaeIntakeCommand(m_algaeSubsystem));
    yButton.onTrue(new AlgaeOuttakeCommand(m_algaeSubsystem));
    aButton.whileTrue(new CoralIntakeCommand(m_coralSubsystem));
    bButton.onTrue(new CoralOuttakeCommand(m_coralSubsystem, m_elevatorSubsystem));
    startButton.onTrue(new ExtendLift(m_climbSubsystem));
    backButton.onTrue(new RetractLift(m_climbSubsystem));

    dPadDown.onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel1, m_elevatorSubsystem)));

    dPadLeft.onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel2, m_elevatorSubsystem)));

    dPadRight.onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel3, m_elevatorSubsystem)));

    dPadUp.onTrue(new SequentialCommandGroup(new ResetElevator(m_elevatorSubsystem),
        new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel4, m_elevatorSubsystem)));

    m_elevatorSubsystem
        .setDefaultCommand(new ElevatorManualLift(() -> m_xboxController.getLeftY(), m_elevatorSubsystem));

    // Joystick
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
        m_swerveSubsystem,
        driveAxis,
        strafeAxis,
        turnAxis,
        throttleAxis,
        () -> !trigger.getAsBoolean())); // will be robot-centric if held down

    maryButton.onTrue(new SwerveReset(m_swerveSubsystem));
    button3.whileTrue(new GoToReef(m_visionSubsystem, m_swerveSubsystem, "left"));
    button4.whileTrue(new GoToReef(m_visionSubsystem, m_swerveSubsystem, "right"));
    button11.whileTrue(new AlignLoadingStationCommand(m_visionSubsystem, m_swerveSubsystem));
    button12.whileTrue(new GoToTag(m_visionSubsystem, m_swerveSubsystem, 12));
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
        return new PathPlannerAuto("LeftScore1");
      }
      case LeftScore2: {
        return new PathPlannerAuto("LeftScore2");
      }
      case LeftScore3: {
        return new PathPlannerAuto("LeftScore3");
      }
      case CenterScore1: {
        return new PathPlannerAuto("CenterScore1");
      }
      case RightScore1: {
        return new PathPlannerAuto("BottomCoralAutoRight");
      }
      case RightScore2: {
        return new PathPlannerAuto("RightScore2");
      }
      case RightScore3: {
        return new PathPlannerAuto("RightScore3");
      }
    }
  }
}
