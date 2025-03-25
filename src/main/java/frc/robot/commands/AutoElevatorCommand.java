package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command for moving the elevator
 */
public class AutoElevatorCommand extends Command {
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final Double m_elevator_pos;

  /**
   * Creates a new auto elevator command.
   * 
   * @param elevatorPos Default elevator positions can be found in the Constants
   * @param elevator    Elevator subsystem
   */
  public AutoElevatorCommand(Double elevatorPos, ElevatorSubsystem elevator) {
    m_elevator_pos = elevatorPos;
    m_ElevatorSubsystem = elevator;
    addRequirements(m_ElevatorSubsystem);
  }

  @Override
  public void execute() {
    m_ElevatorSubsystem.moveToPosition(m_elevator_pos);
  }

  /**
   * @return boolean
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
