package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command that allows the robot to be driven with a joystick.
 */
public class DriveCommand extends Command {
        private SwerveSubsystem swerveDrive;
        private DoubleSupplier ySpeedFunc;
        private DoubleSupplier xSpeedFunc;
        private DoubleSupplier rotFunc;
        private DoubleSupplier throttle;
        private BooleanSupplier fieldOriented;

        private SlewRateLimiter xLimiter;
        private SlewRateLimiter yLimiter;
        private SlewRateLimiter zLimiter;

        private double ySpeed;
        private double xSpeed;
        private double rot;

        /**
         * Creates a new DriveCommand.
         * 
         * @param swerveDrive   The swerve subsystem
         * @param ySpeedFunc    Forward speed
         * @param xSpeedFunc    Sideways speed
         * @param rotFunc       Rotational speed
         * @param throttle      Speed control
         * @param fieldOriented Whether the robot should drive oriented to itself or the
         *                      field
         */
        public DriveCommand(
                        SwerveSubsystem swerveDrive,
                        DoubleSupplier ySpeedFunc,
                        DoubleSupplier xSpeedFunc,
                        DoubleSupplier rotFunc,
                        DoubleSupplier throttle,
                        BooleanSupplier fieldOriented) {
                this.swerveDrive = swerveDrive;
                this.ySpeedFunc = ySpeedFunc;
                this.xSpeedFunc = xSpeedFunc;
                this.rotFunc = rotFunc;
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
                                + (Constants.ControllerConstants.kMinThrottle
                                                + Constants.ControllerConstants.kMaxThrottle) / 2;

                ySpeed = MathUtil.applyDeadband(ySpeedFunc.getAsDouble(),
                                Constants.ControllerConstants.kFlightStickYDeadband);
                xSpeed = MathUtil.applyDeadband(-xSpeedFunc.getAsDouble(),
                                Constants.ControllerConstants.kFlightStickXDeadband);
                rot = MathUtil.applyDeadband(-rotFunc.getAsDouble(),
                                Constants.ControllerConstants.kFlightStickZDeadband);

                ySpeed = ySpeed * Math.abs(ySpeed);
                xSpeed = xSpeed * Math.abs(xSpeed);
                rot = rot * Math.abs(rot);

                ySpeed = yLimiter.calculate(ySpeed) * adjustedThrottle;
                xSpeed = xLimiter.calculate(xSpeed) * adjustedThrottle;
                rot = zLimiter.calculate(rot);

                swerveDrive.drive(
                                () -> ySpeed,
                                () -> xSpeed,
                                () -> rot,
                                fieldOriented);

                SmartDashboard.putNumber("Joystick/Y", ySpeed);
                SmartDashboard.putNumber("Joystick/X", xSpeed);
                SmartDashboard.putNumber("Joystick/Z", rot);
                // SmartDashboard.putNumber("Joystick/T", adjustedThrottle);
        }
}