package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Algae outtake command that uses a timer to stop the outtake after a certain time.
 */
public class AlgaeOuttakeCommand extends Command {
    private final AlgaeSubsystem m_AlgaeSubsystem;

    private static final Timer timer = new Timer();

    /**
     * Creates a new algae outtake command.
     * 
     * @param subsystem Algae subsystem
     */
    public AlgaeOuttakeCommand(AlgaeSubsystem subsystem) {
        m_AlgaeSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
        m_AlgaeSubsystem.Outtake();
    }

    @Override
    public void end(boolean interrupted) {
        m_AlgaeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Constants.AlgaeConstants.kAlgaeTime);
    }
}