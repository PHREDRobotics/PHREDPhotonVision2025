package frc.robot.commands;

import frc.robot.subsystems.AlgaeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Algae intake command that uses the limit switch to stop the intake when the algae is loaded.
 */
public class AlgaeIntakeCommand extends Command {
    private final AlgaeSubsystem cmdSubsystem;

    /**
     * Creates a new algae intake command.
     * 
     * @param subsystem Algae subsystem
     */
    public AlgaeIntakeCommand(AlgaeSubsystem subsystem) {
        cmdSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        cmdSubsystem.Intake();
    }

    @Override
    public void execute() {
        // Periodically runs; nothing for now.
    }

    @Override
    public void end(boolean interrupted) {
        cmdSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // return cmdSubsystem.isAlgaeLoaded();

        return false; // Only for whileTrue for RobotContainer
         /**
          * isFinished() will return isAlgaeLoaded() which checks if the limit switch (or beam break) for the algae is pressed.
          */
    }
} 