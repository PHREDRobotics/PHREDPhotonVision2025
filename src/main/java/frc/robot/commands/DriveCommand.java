package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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

    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter zLimiter;

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

        xLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAcceleration);
        yLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAcceleration);
        zLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAngularAcceleration);

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double adjustedThrottle = ((-(Constants.ControllerConstants.kMaxThrottle
                - Constants.ControllerConstants.kMinThrottle) / 2) * throttle.getAsDouble())
                + (Constants.ControllerConstants.kMinThrottle + Constants.ControllerConstants.kMaxThrottle) / 2;

        ySpeed = () -> MathUtil.applyDeadband(ySpeed.getAsDouble(),
                Constants.ControllerConstants.kFlightStickYDeadband);
        xSpeed = () -> MathUtil.applyDeadband(-xSpeed.getAsDouble(),
                Constants.ControllerConstants.kFlightStickXDeadband);
        rot = () -> MathUtil.applyDeadband(-rot.getAsDouble(), Constants.ControllerConstants.kFlightStickZDeadband);

        ySpeed = () -> ySpeed.getAsDouble() * Math.abs(ySpeed.getAsDouble());
        xSpeed = () -> xSpeed.getAsDouble() * Math.abs(xSpeed.getAsDouble());
        rot = () -> rot.getAsDouble() * Math.abs(rot.getAsDouble());

        ySpeed = () -> yLimiter.calculate(ySpeed.getAsDouble()) * adjustedThrottle;
        xSpeed = () -> xLimiter.calculate(xSpeed.getAsDouble()) * adjustedThrottle;
        rot = () -> zLimiter.calculate(rot.getAsDouble());

        swerveDrive.drive(
                ySpeed,
                xSpeed,
                rot,
                fieldOriented);

        SmartDashboard.putNumber("Joystick/Y", ySpeed.getAsDouble());
        SmartDashboard.putNumber("Joystick/X", xSpeed.getAsDouble());
        SmartDashboard.putNumber("Joystick/Z", rot.getAsDouble());
        SmartDashboard.putNumber("Joystick/T", adjustedThrottle);
    }
}