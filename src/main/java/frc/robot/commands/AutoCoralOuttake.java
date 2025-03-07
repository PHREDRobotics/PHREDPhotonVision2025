package frc.robot.commands;

import frc.robot.subsystems.CoralSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Coral intake command that uses the limit switch to stop the intake when the
 * coral is loaded.
 */
public class AutoCoralOuttake extends Command {
    private final CoralSubsystem coralSubsystem;

    /**
     * Creates a new CoralIntakeCommand.
     * 
     * @param subsystem Coral subsystem
     */
    public AutoCoralOuttake(CoralSubsystem subsystem) {
        coralSubsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        coralSubsystem.autoStartOuttake();
    }

    @Override
    public void execute() {
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
        // return coralSubsystem.isCoralLoaded();
        return CoralSubsystem.isTimeDone();
    }
}