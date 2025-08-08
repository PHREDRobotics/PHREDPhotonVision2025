// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final SendableChooser<RobotContainer.AutoSwitcher> autoChooser = new SendableChooser<>();

  boolean climbCamera;

  UsbCamera camera1;
  UsbCamera camera2;

  VideoSink server;

  MjpegServer switchedCamera;

  String trajectoryJSON = "pathplanner/paths/offtheline.json";
  Trajectory trajectory = new Trajectory();
  StructArrayPublisher<Pose3d> notePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyPoseArray", Pose3d.struct)
      .publish();

  public Robot() {
    m_robotContainer = new RobotContainer();

    if (!Robot.isReal()) {
      SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)));
      SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 3)));
      SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralAlgaeStack(new Translation2d(3, 2)));
    }

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory", ex.getStackTrace());
    }

    // CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotInit() {
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);

    switchedCamera = CameraServer.addSwitchedCamera("switched");

    camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
    camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    camera1.setResolution(240, 120);
    camera2.setResolution(320, 240);

    camera1.setFPS(20);
    camera2.setFPS(20);

    autoChooser.setDefaultOption("Off-the-line", RobotContainer.AutoSwitcher.OFF_THE_LINE);

    autoChooser.addOption("Left Score 1", RobotContainer.AutoSwitcher.LeftScore1);
    autoChooser.addOption("Left Score 2", RobotContainer.AutoSwitcher.LeftScore2);
    autoChooser.addOption("Left Score 3", RobotContainer.AutoSwitcher.LeftScore3);
    autoChooser.addOption("Center Score 1", RobotContainer.AutoSwitcher.CenterScore1);
    autoChooser.addOption("Right Score 1", RobotContainer.AutoSwitcher.RightScore1);
    autoChooser.addOption("Right Score 2", RobotContainer.AutoSwitcher.RightScore2);
    autoChooser.addOption("Right Score 3", RobotContainer.AutoSwitcher.RightScore3);

    SmartDashboard.putData("Auto mode", autoChooser);
    SmartDashboard.putBoolean("Climb Camera", climbCamera);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // climbCamera = SmartDashboard.getBoolean("Climb Camera", climbCamera);
    SmartDashboard.putBoolean("climb toggled", climbCamera);

    if (climbCamera) {
      switchedCamera.setSource(camera1);
    } else {
      switchedCamera.setSource(camera2);
    }
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @SuppressWarnings("unused")
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    Pose3d[] notesPoses = SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Algae");
    notePoses.accept(SimulatedArena.getInstance()
        .getGamePiecesArrayByType("Algae"));
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
