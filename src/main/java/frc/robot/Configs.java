package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI / Constants.SwerveConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;

            double drivingVelocityFeedForward = 1 / 6;

            drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
            drivingConfig.encoder
                .positionConversionFactor(drivingFactor)
                .velocityConversionFactor(drivingFactor / 60);
            drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                .inverted(false)
                .positionConversionFactor(turningFactor) // radiansd
                .velocityConversionFactor(turningFactor / 60);
            turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
        }
    }
}
