// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command for retracting the lift.
 */
public class RetractLift extends Command {
  private final ClimbSubsystem m_subsystem;
  /**
   * Creates a new RetractLift command
   *
   * @param subsystem Climb Subsystem
   */
  public RetractLift(ClimbSubsystem subsystem) {
    m_subsystem = subsystem;
    
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.retractPneumaticClimber();
  }

  
  /** 
   * @return boolean
   */
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}