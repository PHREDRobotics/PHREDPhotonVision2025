package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// *************************************************************** 23.5 INCHES = HP STATION DISTANCE ------------------------------------------------

/**
 * Subsystem for controlling the ultrasonic sensor
 */
public class DistanceSubsystem extends SubsystemBase {
  // Ultrasonic sensors tend to be quite noisy and susceptible to sudden outliers,
  // so measurements are filtered with a 5-sample median filter
  private final MedianFilter m_leftfilter = new MedianFilter(Constants.UltrasonicConstants.kMedianFilterSize);
  private final MedianFilter m_rightfilter = new MedianFilter(Constants.UltrasonicConstants.kMedianFilterSize);
  // Smartdashboard editable variable
  private double m_distanceFromWallLeft = Constants.UltrasonicConstants.kTargetDistanceFromReef;
  private double m_distanceFromWallRight = Constants.UltrasonicConstants.kTargetDistanceFromReef;
  private final Ultrasonic m_leftultrasonic = new Ultrasonic(
      Constants.UltrasonicConstants.kUltrasonicPingPortLeft,
      Constants.UltrasonicConstants.kUltrasonicEchoPortLeft);

  private final Ultrasonic m_rightultrasonic = new Ultrasonic(
      Constants.UltrasonicConstants.kUltrasonicPingPortRight,
      Constants.UltrasonicConstants.kUltrasonicEchoPortRight);

  public DistanceSubsystem() {
    m_leftultrasonic.setEnabled(true);
    m_rightultrasonic.setEnabled(true);
    Ultrasonic.setAutomaticMode(true);
  }

  public void periodic() {
    double leftmeasurement = m_leftultrasonic.getRangeInches();
    double rightmeasurement = m_rightultrasonic.getRangeInches();
    double leftfilteredMeasurement = m_leftfilter.calculate(leftmeasurement);
    double rightfilteredMeasurement = m_rightfilter.calculate(rightmeasurement);

    m_distanceFromWallLeft = SmartDashboard.getNumber("Ultrasound/Left/Distance From Wall Target Left",
        m_distanceFromWallLeft);
    m_distanceFromWallRight = SmartDashboard.getNumber("Ultrasound/Right/Distance From Wall Target Right",
        m_distanceFromWallRight);
    SmartDashboard.putNumber("Ultrasound/Left/Distance From Wall Target Left", m_distanceFromWallLeft);
    SmartDashboard.putNumber("Ultrasound/Left/Distance From Wall Left", leftfilteredMeasurement);
    SmartDashboard.putNumber("Ultrasound/Left/Error Left",
        m_distanceFromWallLeft - leftfilteredMeasurement);
    SmartDashboard.putNumber("Ultrasound/Right/Distance From Wall Target Right", m_distanceFromWallRight);
    SmartDashboard.putNumber("Ultrasound/Right/Distance From Wall Right", rightfilteredMeasurement);
    SmartDashboard.putNumber("Ultrasound/Right/Error Right",
        m_distanceFromWallLeft - rightfilteredMeasurement);
    SmartDashboard.putNumber("Ultrasound/Left/Distance From Wall Left Raw", leftmeasurement);
    SmartDashboard.putNumber("Ultrasound/Right/Distance From Wall Right Raw", rightmeasurement);
  }
}