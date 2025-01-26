package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveCommand extends Command {
    private SwerveDrive swerveDrive;
    private DoubleSupplier ySpeed;
    private DoubleSupplier xSpeed;
    private DoubleSupplier rot;
    private BooleanSupplier fieldOriented;

    public DriveCommand(
            SwerveDrive swerveDrive,
            DoubleSupplier ySpeed,
            DoubleSupplier xSpeed,
            DoubleSupplier rot,
            BooleanSupplier fieldOriented) {
        this.swerveDrive = swerveDrive;
        this.ySpeed = ySpeed;
        this.xSpeed = xSpeed;
        this.rot = rot;
        this.fieldOriented = fieldOriented;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        // Deadzone
        double ySpeedAdjusted = MathUtil.applyDeadband(ySpeed.getAsDouble(), Constants.ControllerConstants.kFlightStickYDeadband);
        double xSpeedAdjusted = MathUtil.applyDeadband(xSpeed.getAsDouble(), Constants.ControllerConstants.kFlightStickXDeadband);
        double rotAdjusted = MathUtil.applyDeadband(rot.getAsDouble(), Constants.ControllerConstants.kFlightStickZDeadband);

        swerveDrive.drive(
            () -> ySpeedAdjusted * Constants.PhysicalConstants.kMaxSpeed,
            () -> -xSpeedAdjusted * Constants.PhysicalConstants.kMaxSpeed,
            () -> rotAdjusted * Constants.PhysicalConstants.kMaxAngularSpeed,
            fieldOriented);
    }
}