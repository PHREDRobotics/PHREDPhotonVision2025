package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * A command that allows the robot to be driven with a joystick.
 */
public class TargetCommand extends Command {
  private SwerveSubsystem swerveDrive;
  private PhotonVisionSubsystem photonVision;

  private SlewRateLimiter xLimiter;
  private SlewRateLimiter yLimiter;
  private SlewRateLimiter zLimiter;

  private double ySpeed;
  private double xSpeed;
  private double rot;

  /**
   * Creates a new DriveCommand.
   * 
   * @param swerveDrive  The swerve subsystem
   * @param photonVision The photon vision subsystem
   */
  public TargetCommand(
      SwerveSubsystem swerveDrive,
      PhotonVisionSubsystem photonVision) {
    this.swerveDrive = swerveDrive;
    this.photonVision = photonVision;

    xLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAcceleration);
    yLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAcceleration);
    zLimiter = new SlewRateLimiter(Constants.PhysicalConstants.kMaxAngularAcceleration);

    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {

    xSpeed = photonVision.getXspeed();
    ySpeed = photonVision.getYspeed();
    rot = photonVision.getRotSpeed();

    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    rot = zLimiter.calculate(rot);

    swerveDrive.drive(
        () -> ySpeed,
        () -> xSpeed,
        () -> rot,
        () -> false);

    SmartDashboard.putNumber("PID/Y", ySpeed);
    SmartDashboard.putNumber("PID/X", xSpeed);
    SmartDashboard.putNumber("PID/Z", rot);
  }
}
