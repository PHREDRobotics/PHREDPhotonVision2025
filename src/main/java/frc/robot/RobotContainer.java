// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorManualLift;
import frc.robot.commands.ResetElevator;
import frc.robot.commands.AutoElevatorCommand;
import frc.robot.commands.ExtendLift;
import frc.robot.commands.RetractLift;
import frc.robot.controls.LogitechPro;

import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    SwerveSubsystem m_swerveSubsystem;
    AlgaeSubsystem m_algaeSubsystem;
    ClimbSubsystem m_climbSubsystem;
    CoralSubsystem m_coralSubsystem;
    ElevatorSubsystem m_elevatorSubsystem;

    LogitechPro m_driverJoystick;
    CommandXboxController m_xboxController;

    public enum AutoSwitcher { // enum to switch between different auto modes
        OFF_THE_LINE,
        BottomCoral
    }

    public RobotContainer() {
        m_swerveSubsystem = new SwerveSubsystem();
        m_algaeSubsystem = new AlgaeSubsystem();
        m_coralSubsystem = new CoralSubsystem();
        m_climbSubsystem = new ClimbSubsystem();
        m_elevatorSubsystem = new ElevatorSubsystem();

        m_driverJoystick = new LogitechPro(0);
        m_xboxController = new CommandXboxController(1);

        configureBindings();
    }

    private void configureBindings() {
        // Triggers
        Trigger fieldOrientedTrigger = new Trigger(() -> m_driverJoystick.getTrigger());

        Trigger bButton = m_xboxController.b();
        Trigger aButton = m_xboxController.a();
        Trigger yButton = m_xboxController.y();
        Trigger xButton = m_xboxController.x();
        Trigger startButton = m_xboxController.start();
        Trigger backButton = m_xboxController.back();
        Trigger leftBumper = m_xboxController.leftBumper();
        Trigger rightBumper = m_xboxController.rightBumper();
        Trigger leftStickButton = m_xboxController.leftStick();
        Trigger rightStickButton = m_xboxController.rightStick();
        // Trigger Button3 = m_driverJoystick.buttons[3];
        // Trigger Button2 = m_driverJoystick.buttons[2];
        // Axes
        DoubleSupplier driveAxis = () -> m_driverJoystick.getPitch();
        DoubleSupplier strafeAxis = () -> m_driverJoystick.getRoll();
        DoubleSupplier turnAxis = () -> m_driverJoystick.getYaw();
        DoubleSupplier throttleAxis = () -> m_driverJoystick.getCorrectedThrottle();

        // D-Pad Buttons
        // ...
        Trigger dPadDown = m_xboxController.povDown();
        Trigger dPadUp = m_xboxController.povUp();
        Trigger dPadLeft = m_xboxController.povLeft();
        Trigger dPadRight = m_xboxController.povRight();

        // Assign Commands

        bButton.onTrue(new AlgaeIntakeCommand(m_algaeSubsystem));
        aButton.onTrue(new AlgaeOuttakeCommand(m_algaeSubsystem));
        xButton.onTrue(new CoralIntakeCommand(m_coralSubsystem));
        yButton.onTrue(new CoralOuttakeCommand(m_coralSubsystem));
        startButton.onTrue(new ExtendLift(m_climbSubsystem));
        backButton.onTrue(new RetractLift(m_climbSubsystem));
        leftBumper.whileTrue(new ElevatorManualLift(() -> m_xboxController.getLeftY(), m_elevatorSubsystem));
        leftStickButton.onTrue(new ResetElevator(m_elevatorSubsystem));
        rightStickButton.onTrue(new AutoElevatorCommand(Constants.ElevatorConstants.kCoralLevel4, m_elevatorSubsystem));

        m_swerveSubsystem.setDefaultCommand(new DriveCommand(
                m_swerveSubsystem,
                driveAxis,
                strafeAxis,
                turnAxis,
                throttleAxis,
                () -> !fieldOrientedTrigger.getAsBoolean())); // will be robot-centric if held down

        m_elevatorSubsystem.setDefaultCommand(new ElevatorManualLift(() -> m_xboxController.getLeftY(), m_elevatorSubsystem));
    } 

    public Command getAutonomousCommand(AutoSwitcher autoMode) {
        switch (autoMode) {
            default:
            case OFF_THE_LINE: {
                return new PathPlannerAuto("Basic off-the-line auto");
            }
            case BottomCoral: {
                return new PathPlannerAuto("BottomCoralAuto");
            }
        }
    }
}
