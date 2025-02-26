package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Subsystem for controlling the coral intake and outtake
 */
public class CoralSubsystem extends SubsystemBase {
  private static final Timer timer = new Timer();
  public SparkMax coralMotorSparkMax = new SparkMax(CoralConstants.kCoralCANId, MotorType.kBrushless);
  public SparkLimitSwitch forwardLimit = coralMotorSparkMax.getForwardLimitSwitch();

  public double intakeSpeed = CoralConstants.kCoralIntakeSpeed;
  public double outtakeSpeedL1 = CoralConstants.kCoralOuttakeSpeedL1;
  public double outtakeSpeedL2 = CoralConstants.kCoralOuttakeSpeedL2;
  public double outtakeSpeedL3 = CoralConstants.kCoralOuttakeSpeedL3;
  public double outtakeSpeedL4 = CoralConstants.kCoralOuttakeSpeedL4;

  /**
   * Creates a new CoralSubsystem.
   */
  public CoralSubsystem() {
    coralMotorSparkMax.configure(Configs.CoralMotor.motorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void startOuttake(int level) {
    timer.reset();
    if (level == 1) {
      coralMotorSparkMax.set(outtakeSpeedL1);
    } else if (level == 2) {
      coralMotorSparkMax.set(outtakeSpeedL2);
    } else if (level == 3) {
      coralMotorSparkMax.set(outtakeSpeedL3);
    } else if (level == 4) {
      coralMotorSparkMax.set(outtakeSpeedL4);
    }
    timer.start();
  }

  /**
   * @param inSpeed
   * @return double
   */
  public double speedConvert(double inSpeed) {
    return inSpeed;
  }

  public void stop() {
    coralMotorSparkMax.set(0);
  }

  public void startIntake() {
    timer.reset();
    coralMotorSparkMax.set(intakeSpeed);
    timer.start();
  }

  public boolean isCoralLoaded() {
    return !forwardLimit.isPressed();
  }

  public static boolean isTimeDone() {
    return timer.hasElapsed(Constants.CoralConstants.kCoralOuttakeTime);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Pressed?", isCoralLoaded());
    // SmartDashboard.putBoolean("Manual Override Press",
    // SmartDashboard.getBoolean("Manual Override Press", false));
    SmartDashboard.putBoolean("Is Coral Limit Switch Triggered?", !forwardLimit.isPressed());

    // Slider things VARIABLES
    // outtakeSpeed = SmartDashboard.getNumber("Outtake Speed", outtakeSpeed);
    // intakeSpeed = SmartDashboard.getNumber("Intake Speed", intakeSpeed);
    // SmartDashboard.putNumber("Outtake Speed", outtakeSpeed);
    // SmartDashboard.putNumber("Intake Speed", intakeSpeed);
  }
}