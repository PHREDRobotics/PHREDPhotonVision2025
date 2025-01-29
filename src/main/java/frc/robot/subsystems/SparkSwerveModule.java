// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SparkSwerveModule implements SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final SparkAbsoluteEncoder m_turnEncoder;

  private final SparkClosedLoopController m_drivePIDController;
  private final SparkClosedLoopController m_turnPIDController;

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param driveEncoderChannelB DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SparkSwerveModule(
      int driveMotorChannel,
      int turnMotorChannel,
      SparkMaxConfig driveConfig,
      SparkMaxConfig turnConfig) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnMotorChannel, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder();

    m_drivePIDController = m_driveMotor.getClosedLoopController();
    m_turnPIDController = m_turnMotor.getClosedLoopController();

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
        m_driveEncoder.getVelocity(), new Rotation2d(m_turnEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(m_turnEncoder.getPosition()));
  }

  @Override
  public double getDriveTemp() {
    return m_driveMotor.getMotorTemperature();
  }

  @Override
  public double getTurnTemp() {
    return m_turnMotor.getMotorTemperature();
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

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turnPIDController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
  }
}