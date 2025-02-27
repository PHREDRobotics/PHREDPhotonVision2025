package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Algae intake command that uses the limit switch to stop the intake when the
 * algae is loaded.
 */
public class AlgaeIntakeCommand extends Command {
    private final AlgaeSubsystem m_AlgaeSubsystem;
    private static final Timer timer = new Timer();

    /**
     * Creates a new algae intake command.
     * 
     * @param subsystem Algae subsystem
     */
    public AlgaeIntakeCommand(AlgaeSubsystem subsystem) {
        m_AlgaeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        m_AlgaeSubsystem.Intake();
    }

    @Override
    public void execute() {
        // Periodically runs; nothing for now.
    }

    /**
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted) {
        m_AlgaeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_AlgaeSubsystem.isAlgaeLoaded();

        // return timer.hasElapsed(Constants.AlgaeConstants.kAlgaeTime);
        /**
         * isFinished() will return isAlgaeLoaded() which checks if the limit switch (or
         * beam break) for the algae is pressed.
         */
    }
}