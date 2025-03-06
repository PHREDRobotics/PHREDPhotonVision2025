package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Configs for the spark maxes
 */
public final class Configs {
        public static final class FrontLeftConfig {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
                                        / Constants.SwerveConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;

                        double drivingVelocityFeedForward = 1 / 6;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(true);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor)
                                        .velocityConversionFactor(drivingFactor / 60);
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.4, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                        turningConfig.absoluteEncoder
                                        .inverted(true)
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

        public static final class FrontRightConfig {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
                                        / Constants.SwerveConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;

                        double drivingVelocityFeedForward = 1 / 6;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(true);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor)
                                        .velocityConversionFactor(drivingFactor / 60);
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.4, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                        turningConfig.absoluteEncoder
                                        .inverted(true)
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

        public static final class BackLeftConfig {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
                                        / Constants.SwerveConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;

                        double drivingVelocityFeedForward = 1 / 6;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(false);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor)
                                        .velocityConversionFactor(drivingFactor / 60);
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.4, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                        turningConfig.absoluteEncoder
                                        .inverted(true)
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

        public static final class BackRightConfig {
                public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
                public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

                static {
                        double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
                                        / Constants.SwerveConstants.kDrivingMotorReduction;
                        double turningFactor = 2 * Math.PI;

                        double drivingVelocityFeedForward = 1 / 6;

                        drivingConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(false);
                        drivingConfig.encoder
                                        .positionConversionFactor(drivingFactor)
                                        .velocityConversionFactor(drivingFactor / 60);
                        drivingConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        .pid(0.4, 0, 0)
                                        .velocityFF(drivingVelocityFeedForward)
                                        .outputRange(-1, 1);

                        turningConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(20)
                                        .inverted(true);
                        turningConfig.absoluteEncoder
                                        .inverted(true)
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

        public static final class ElevatorMotor {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(true);
                        motorConfig.limitSwitch
                                        .reverseLimitSwitchEnabled(true)
                                        .forwardLimitSwitchEnabled(false)
                                        .reverseLimitSwitchType(Type.kNormallyClosed);
                        motorConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                        // These are example gains you may need to them for your own robot!
                                        .pid(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI,
                                                        Constants.ElevatorConstants.kD)
                                        .outputRange(-1, 1)
                                        .velocityFF(Constants.ElevatorConstants.kVelocityFF);
                        motorConfig.closedLoop.maxMotion
                                        .maxAcceleration(Constants.ElevatorConstants.kMaxAcceleration)
                                        .maxVelocity(Constants.ElevatorConstants.kMaxMotorSpeed)
                                        .allowedClosedLoopError(Constants.ElevatorConstants.kClosedLoopError);
                }
        }

        public static final class CoralMotor {
                public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

                static {
                        motorConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(50)
                                        .inverted(false);
                        motorConfig.limitSwitch
                                        .reverseLimitSwitchEnabled(false)
                                        .forwardLimitSwitchEnabled(true)
                                        .forwardLimitSwitchType(Type.kNormallyClosed);
                }
        }
}
