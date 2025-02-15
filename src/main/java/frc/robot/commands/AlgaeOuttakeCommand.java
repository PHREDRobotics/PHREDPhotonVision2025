package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeOuttakeCommand extends Command {
    private final AlgaeSubsystem cmdSubsystem;
    
  private static final Timer timer = new Timer();

    /**
     * 
     * @param subsystem
     */
    public AlgaeOuttakeCommand(AlgaeSubsystem subsystem) {
        cmdSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
           timer.reset();
        cmdSubsystem.Outtake();
    }

    @Override
    public void execute() {
        // nothing for now, this will run periodically..
    }

    @Override
    public void end(boolean interrupted) {
        cmdSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        
       return timer.hasElapsed(Constants.CoralConstants.kCoralOuttakeTime);
    }
}