package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ResetElevator extends Command {
    ElevatorSubsystem m_elevatorSubsystem;

    public ResetElevator(ElevatorSubsystem elevatorSubsystem) {
        m_elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return m_elevatorSubsystem.isLimitSwitchPressed();
    }
}