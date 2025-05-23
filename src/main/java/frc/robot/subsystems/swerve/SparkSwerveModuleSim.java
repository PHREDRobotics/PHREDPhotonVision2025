// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * A swerve module that uses simulation
 */
public class SparkSwerveModuleSim implements SwerveModule {
  private final SparkMaxSim m_driveMotor;
  private final SparkMaxSim m_turnMotor;

  private final SparkRelativeEncoderSim m_driveEncoder;
  private final SparkAbsoluteEncoderSim m_turnEncoder;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turnMotorChannel  PWM output for the turning motor.
   */
  public SparkSwerveModuleSim(
      int driveMotorChannel,
      int turnMotorChannel) {
    m_driveMotor = new SparkMaxSim(new SparkMax(driveMotorChannel, MotorType.kBrushless), DCMotor.getNEO(1));
    m_turnMotor = new SparkMaxSim(new SparkMax(turnMotorChannel, MotorType.kBrushless), DCMotor.getNEO(1));

    m_driveEncoder = m_driveMotor.getRelativeEncoderSim();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoderSim();

    m_driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getVelocity(), new Rotation2d(m_turnMotor.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getPosition(), new Rotation2d(m_turnMotor.getPosition()));
  }

  /**
   * @return double
   */
  @Override
  public double getDriveTemp() {
    return 0;
  }

  @Override
  public double getTurnTemp() {
    return 0;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(m_turnEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    m_driveMotor.setVelocity(desiredState.speedMetersPerSecond);
    m_turnMotor.setPosition(desiredState.angle.getRadians());
  }
}
