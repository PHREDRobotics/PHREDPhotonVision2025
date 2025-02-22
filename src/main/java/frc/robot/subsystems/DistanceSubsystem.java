package frc.robot.subsystems;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    private final Ultrasonic m_leftultrasonic = new Ultrasonic(Constants.UltrasonicConstants.kUltrasonicPingPortLeft,
            Constants.UltrasonicConstants.kUltrasonicEchoPortLeft);
    private final Ultrasonic m_rightultrasonic = new Ultrasonic(Constants.UltrasonicConstants.kUltrasonicPingPortRight,
            Constants.UltrasonicConstants.kUltrasonicEchoPortLeft);

    public void periodic() {
        double leftmeasurement = m_leftultrasonic.getRangeMM();
        double rightmeasurement = m_rightultrasonic.getRangeMM();
        double leftfilteredMeasurement = m_leftfilter.calculate(leftmeasurement);
        double rightfilteredMeasurement = m_rightfilter.calculate(rightmeasurement);

        m_distanceFromWallLeft = SmartDashboard.getNumber("Distance From Wall Target Left", m_distanceFromWallLeft);
        m_distanceFromWallRight = SmartDashboard.getNumber("Distance From Wall Target Right", m_distanceFromWallRight);
        SmartDashboard.putNumber("Distance From Wall Target Left", m_distanceFromWallLeft);
        SmartDashboard.putNumber("Distance From Wall Left", leftfilteredMeasurement);
        SmartDashboard.putNumber("Error Left", m_distanceFromWallLeft - leftfilteredMeasurement);
        SmartDashboard.putNumber("Distance From Wall Target Right", m_distanceFromWallRight);
        SmartDashboard.putNumber("Distance From Wall Right", rightfilteredMeasurement);
        SmartDashboard.putNumber("Error Right", m_distanceFromWallLeft - rightfilteredMeasurement);
    }
}