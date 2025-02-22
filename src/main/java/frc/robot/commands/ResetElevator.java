package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command to drive the motor down until the limit switch is triggered and then
 * 0's the encoder
 */
public class ResetElevator extends Command {
    ElevatorSubsystem m_elevatorSubsystem;

    public ResetElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setSpeed(Constants.ElevatorConstants.kElevatorResetSpeed);
    }

    /**
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isLimitSwitchPressed();
    }
}