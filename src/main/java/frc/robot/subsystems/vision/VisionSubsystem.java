// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for interfacing with the limelight.
 */
public class VisionSubsystem extends SubsystemBase {
  public boolean m_LimelightHasValidTarget = false;
  public boolean m_IsLimeLightCentered = false;
  NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight_phred");
  double tv = m_table.getEntry("tv").getDouble(0);
  double tx = m_table.getEntry("tx").getDouble(0);
  double ty = m_table.getEntry("ty").getDouble(0);
  double ta = m_table.getEntry("ta").getDouble(0);
  double tid = m_table.getEntry("tid").getDouble(0);

  // double kP = 0.9;
  // double kI = 0.01;
  // double kD = 0.01;

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public Command VisionCommand() {

  // // Inline construction of command goes here.
  // // Subsystem::RunOnce implicitly requires `this` subsystem.
  // return runOnce(
  // () -> {
  // /* one-time action goes here */
  // });
  // }
  // We will need to move accordingly but I think that will go elsewhere

  // public void vision() {
  // m_table = NetworkTableInstance.getDefault().getTable("limelight_phred");
  // tv = m_table.getEntry("tv").getDouble(0);
  // tx = m_table.getEntry("tx").getDouble(0);
  // ty = m_table.getEntry("ty").getDouble(0);
  // ta = m_table.getEntry("ta").getDouble(0);
  // tid = m_table.getEntry("tid").getDouble(0);
  // SmartDashboard.putBoolean("Sees target?", !(tid == -1.0));
  // SmartDashboard.putNumber("Target:", tid);
  // SmartDashboard.putNumber("Limelight a value.", ta);
  // SmartDashboard.putNumber("Limelight y value.", ty);
  // SmartDashboard.putNumber("Limelight x value.", tx);
  // SmartDashboard.putNumber("Limelight v value.", tv);
  // SmartDashboard.putBoolean("Is the target in range?",
  // m_LimelightHasValidTarget);
  // SmartDashboard.putBoolean("Is the target centered", m_IsLimeLightCentered);
  // }

  // public double getTargetDistance() {
  // double angleToGoalDegrees = VisionConstants.kLimelightMountAngleDegrees + ty;
  // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  // double d = (VisionConstants.kAmpOrSourceHeightInches -
  // VisionConstants.kLimelightLensHeightInches)
  // / Math.tan(angleToGoalRadians);
  // return d;
  // // 14.75 against amp
  // // 27.1 = 18.75
  // // 36=32
  // // 47.8~=49.75
  // }

  // public double getRoughTargetDistance() {
  // double angleToGoalDegrees = VisionConstants.kLimelightMountAngleDegrees + ty;
  // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
  // double hyp = VisionConstants.kAreaToCentimeters / ta;
  // double d = hyp * Math.cos(angleToGoalRadians);
  // return d;
  // }

  public double getTargetArea() {
    return ta;
  }

  /**
   * @return double
   */
  public double getTargetXDegrees() {
    return tx;
  }

  public void goToDistance(double distance) {
  }

  public int getTargetID() {
    return (int) m_table.getEntry("tid").getInteger(0);
  }

  public double getTargetX() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public double getTargetY() {
    return m_table.getEntry("ty").getDouble(0);
  }

  public double getTargetA() {
    return m_table.getEntry("ta").getDouble(0);
  }

  @Override
  public void periodic() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight_phred");
    // tv = m_table.getEntry("tv").getDouble(0);
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);
    ta = m_table.getEntry("ta").getDouble(0);
    tid = m_table.getEntry("tid").getDouble(0);
    SmartDashboard.putBoolean("Limelight/Sees target?", !(tid == -1.0));
    SmartDashboard.putNumber("Limelight/Target:", tid);
    SmartDashboard.putNumber("Limelight/Limelight a value.", ta);
    SmartDashboard.putNumber("Limelight/Limelight y value.", ty);
    SmartDashboard.putNumber("Limelight/Limelight x value.", tx);
    // SmartDashboard.putNumber("Estimated Distance", getTargetDistance());
    // SmartDashboard.putNumber("Estimated Rough Distance",
    // getRoughTargetDistance());
    // SmartDashboard.putNumber("Limelight v value.", tv);
    SmartDashboard.putBoolean("Limelight/Is the target in range?", m_LimelightHasValidTarget);
    SmartDashboard.putBoolean("Limelight/Is the target centered", m_IsLimeLightCentered);
  }
}
