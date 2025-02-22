package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
/**
 * Coral intake command that uses the limit switch to stop the intake when the coral is loaded.
 */
public class CoralAutoIntakeCommand extends Command {
  private final CoralSubsystem coralSubsystem;

  public CoralAutoIntakeCommand(CoralSubsystem subsystem) {
    coralSubsystem = subsystem;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    coralSubsystem.startIntake();
  }

  
  /** 
   * @param interrupted
   */
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralSubsystem.isCoralLoaded();
  }
}