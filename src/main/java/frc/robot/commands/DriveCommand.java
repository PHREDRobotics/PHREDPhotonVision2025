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
    private BooleanSupplier fieldOriented;

    public DriveCommand(
            SwerveSubsystem swerveDrive,
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
            () -> ySpeedAdjusted,
            () -> -xSpeedAdjusted,
            () -> rotAdjusted,
            fieldOriented);
        
        SmartDashboard.putNumber("Joystick/Y", ySpeedAdjusted);
        SmartDashboard.putNumber("Joystick/X", xSpeedAdjusted);
        SmartDashboard.putNumber("Joystick/Z", rotAdjusted);
    }
}