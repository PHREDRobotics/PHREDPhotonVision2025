package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * A command that manually controls the elevator.
 */
public class ElevatorManualLift extends Command {
    DoubleSupplier power;
    ElevatorSubsystem elevator_subsystem;

    /**
     * Creates a new ElevatorManualLift.
     * @param power Power for the left lift motor
     * @param elevatorSubsystem the elevator subsystem
     */
    public ElevatorManualLift(DoubleSupplier power, ElevatorSubsystem elevatorSubsystem) {
        this.power = power;
        this.elevator_subsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = power.getAsDouble();
        //Cube it
        speed = speed * Math.abs(speed) * Math.abs(speed);

        if (Math.abs(speed) < Constants.ControllerConstants.kXboxDeadband) {
            speed = 0;
        }

        this.elevator_subsystem.setSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return false; //elevator_subsystem.isLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted){
        this.elevator_subsystem.setRawPower(0);
    }

}