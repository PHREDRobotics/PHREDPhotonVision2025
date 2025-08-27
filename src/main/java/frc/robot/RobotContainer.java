// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  CommandJoystick m_joystick;
  CommandXboxController m_xboxController;
  boolean switchButton = false;

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
    m_visionSubsystem = new VisionSubsystem();

    m_joystick = new CommandJoystick(0);
    m_xboxController = new CommandXboxController(1);

    configureBindings();
  }

  private void configureBindings() {
    // Joystick
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(
        m_swerveSubsystem,
        () -> m_joystick.getY(),
        () -> m_joystick.getX(),
        () -> m_joystick.getZ(),
        () -> m_joystick.getThrottle(),
        () -> m_joystick.button(0).getAsBoolean())); // will be robot-centric if held down

    m_joystick.button(6).onTrue(new AlignCommand(m_swerveSubsystem, m_visionSubsystem));

    new Trigger(() -> true) // always active, sends vision estimates to swerve
        .onTrue(new InstantCommand(() -> {
          m_visionSubsystem.getEstimatedGlobalPose().ifPresent(pose -> {
            m_swerveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp());
          });
        }));
  }

  /**
   * @param autoMode
   * @return Command
   */
  public Command getAutonomousCommand(AutoSwitcher autoMode) {
    return new Command() {
    };
  }
}