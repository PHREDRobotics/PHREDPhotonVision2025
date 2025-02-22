package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Command for moving the elevator in order to pull the grenade pin
 */
public class PullGrenadePin extends Command {
    private final ElevatorSubsystem m_ElevatorSubsystem;

    /**
     * 
     * @param elevator The elevator subsystem this command uses.
     */
    public PullGrenadePin(ElevatorSubsystem elevator) {
        m_ElevatorSubsystem = elevator;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_ElevatorSubsystem.moveToPosition(Constants.ElevatorConstants.kCoralLevel2);
    }

    /**
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}