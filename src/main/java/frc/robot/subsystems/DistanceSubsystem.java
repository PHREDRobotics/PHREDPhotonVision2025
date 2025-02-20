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
    private final MedianFilter m_filter = new MedianFilter(Constants.UltrasonicConstants.kMedianFilterSize);
    // Smartdashboard editable variable
    private double m_distanceFromWall = Constants.UltrasonicConstants.kTargetDistanceFromReef;
    private final Ultrasonic m_ultrasonic = new Ultrasonic(Constants.UltrasonicConstants.kUltrasonicPingPort,
            Constants.UltrasonicConstants.kUltrasonicEchoPort);

    public void periodic() {
        double measurement = m_ultrasonic.getRangeMM();
        double filteredMeasurement = m_filter.calculate(measurement);

        m_distanceFromWall = SmartDashboard.getNumber("Distance From Wall Target", m_distanceFromWall);
        SmartDashboard.putNumber("Distance From Wall Target", m_distanceFromWall);
        SmartDashboard.putNumber("Distance From Wall", filteredMeasurement);
        SmartDashboard.putNumber("Error", m_distanceFromWall - filteredMeasurement);
    }
}