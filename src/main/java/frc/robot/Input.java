package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

/**
 * Handles the input for the robot
 */
public class Input {
  CommandJoystick m_driverJoystick = new CommandJoystick(0);
  CommandXboxController m_xboxController = new CommandXboxController(1);
  SlewRateLimiter xLimiter = new SlewRateLimiter(ControllerConstants.kXRateLimit);
  SlewRateLimiter yLimiter = new SlewRateLimiter(ControllerConstants.kYRateLimit);
  SlewRateLimiter zLimiter = new SlewRateLimiter(ControllerConstants.kZRateLimit);

  boolean one_player_mode = false;

  public Input() {
    one_player_mode = SmartDashboard.getBoolean("One Player Mode", false);
    SmartDashboard.putBoolean("One Player Mode", one_player_mode);
  }

  public double getDriveX() {
    return xLimiter.calculate(one_player_mode
        ? Math.signum(m_xboxController.getLeftX()) * Math.pow(m_xboxController.getLeftX(), 2)
        : Math.signum(m_driverJoystick.getX()) * Math.pow(m_driverJoystick.getX(), 2));
  }

  public double getDriveY() {
    return yLimiter.calculate(one_player_mode
        ? Math.signum(m_xboxController.getLeftY()) * Math.pow(m_xboxController.getLeftY(), 2)
        : Math.signum(m_driverJoystick.getY()) * Math.pow(m_driverJoystick.getY(), 2));
  }

  public double getDriveZ() {
    return zLimiter.calculate(one_player_mode
        ? Math.signum(m_xboxController.getRightX()) * Math.pow(m_xboxController.getRightX(), 2)
        : Math.signum(m_driverJoystick.getZ()) * Math.pow(m_driverJoystick.getZ(), 2));
  }

  public double getThrottle() {
    return one_player_mode
        ? 1.0
        : ((1 + (-1 * m_driverJoystick.getThrottle())) / 2);
  }

  public Trigger getFieldOriented() {
    return one_player_mode
        ? m_xboxController.leftBumper()
        : m_driverJoystick.button(0);
  }

  public Trigger getIntake() {
    return m_xboxController.a();
  }

  public Trigger getOuttake() {
    return m_xboxController.b();
  }

  public Trigger getL1() {
    return m_xboxController.povDown();
  }

  public Trigger getL2() {
    return m_xboxController.povLeft();
  }

  public Trigger getL3() {
    return m_xboxController.povRight();
  }

  public Trigger getL4() {
    return m_xboxController.povUp();
  }

  public Trigger getExtendLift() {
    return m_xboxController.start();
  }

  public Trigger getRetractLift() {
    return m_xboxController.back();
  }

  public Trigger getSwerveReset() {
    return one_player_mode
        ? m_xboxController.rightBumper()
        : m_driverJoystick.button(2);
  }

  public Trigger getGoToTag() {
    return one_player_mode
        ? new Trigger(() -> false)
        : m_driverJoystick.button(1);
  }

  public Trigger getCameraSwitch() {
    return one_player_mode
        ? new Trigger(() -> false)
        : m_driverJoystick.button(3);
  }
}
