package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveCommand extends Command {
    private SwerveSubsystem swerveDrive;
    private DoubleSupplier ySpeed;
    private DoubleSupplier xSpeed;
    private DoubleSupplier rot;
    private DoubleSupplier throttle;
    private BooleanSupplier fieldOriented;

    public DriveCommand(
            SwerveSubsystem swerveDrive,
            DoubleSupplier ySpeed,
            DoubleSupplier xSpeed,
            DoubleSupplier rot,
            DoubleSupplier throttle,
            BooleanSupplier fieldOriented) {
        this.swerveDrive = swerveDrive;
        this.ySpeed = ySpeed;
        this.xSpeed = xSpeed;
        this.rot = rot;
        this.throttle = throttle;
        this.fieldOriented = fieldOriented;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double adjustedThrottle = throttle.getAsDouble() * Constants.ControllerConstants.kThrottleMultiplier;

        // Deadzone
        double ySpeedAdjusted = MathUtil.applyDeadband(
                (ySpeed.getAsDouble() * Math.abs(ySpeed.getAsDouble())) * adjustedThrottle,
                Constants.ControllerConstants.kFlightStickYDeadband);
        double xSpeedAdjusted = MathUtil.applyDeadband(
                (xSpeed.getAsDouble() * Math.abs(xSpeed.getAsDouble())) * adjustedThrottle,
                Constants.ControllerConstants.kFlightStickXDeadband);
        double rotAdjusted = MathUtil.applyDeadband(
                (rot.getAsDouble() * Math.abs(rot.getAsDouble())) * adjustedThrottle,
                Constants.ControllerConstants.kFlightStickZDeadband);

        swerveDrive.drive(
                () -> ySpeedAdjusted,
                () -> -xSpeedAdjusted,
                () -> -rotAdjusted,
                fieldOriented);

        SmartDashboard.putNumber("Joystick/Y", ySpeedAdjusted);
        SmartDashboard.putNumber("Joystick/X", xSpeedAdjusted);
        SmartDashboard.putNumber("Joystick/Z", rotAdjusted);
        SmartDashboard.putNumber("Joystick/T", adjustedThrottle);
    }
}