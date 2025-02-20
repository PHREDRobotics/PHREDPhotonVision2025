// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

/**
 * Subsytem for controlling the climber
 */
public class ClimbSubsystem extends SubsystemBase {
  public static PneumaticHub pH;
  public static DoubleSolenoid doubleSolenoidClimber;

  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem() {
    pH = new PneumaticHub(PneumaticsConstants.kPneumaticsCANId);

    pH.enableCompressorDigital();

    doubleSolenoidClimber = pH.makeDoubleSolenoid(PneumaticsConstants.kSolenoidInput,
        PneumaticsConstants.kSolenoidOutput);
    addChild("Double Solenoid Climber", doubleSolenoidClimber);

    doubleSolenoidClimber.set(DoubleSolenoid.Value.kReverse);
  }

  public void extendPneumaticClimber() {
    doubleSolenoidClimber.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPneumaticClimber() {
    doubleSolenoidClimber.set(DoubleSolenoid.Value.kReverse);
  }
}