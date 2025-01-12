/*package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeoSwerve extends SubsystemBase {
    private final Translation2d m_frontLeftLocation = Constants.SwerveConstants.kFrontLeftLocation;
    private final Translation2d m_frontRightLocation = Constants.SwerveConstants.kFrontRightLocation;
    private final Translation2d m_backLeftLocation = Constants.SwerveConstants.kBackLeftLocation;
    private final Translation2d m_backRightLocation = Constants.SwerveConstants.kBackRightLocation;
    
    private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
    private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
    private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
    private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

    private final AHRS m_gyro = new AHRS(Constants.kComType);

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

    private final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
    
    public NeoSwerve() {
        m_gyro.reset();
    }

    public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, BooleanSupplier fieldOriented, double periodSeconds) {
        return runOnce(
            () -> {
                var swerveModuleStates =
                    m_kinematics.toSwerveModuleStates(
                        ChassisSpeeds.discretize(
                            fieldOriented.getAsBoolean()
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), m_gyro.getRotation2d())
                                : new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble()),
                            periodSeconds));
                
                SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.kMaxSpeed);
                m_frontLeft.setDesiredState(swerveModuleStates[0]);
                m_frontRight.setDesiredState(swerveModuleStates[1]);
                m_backLeft.setDesiredState(swerveModuleStates[2]);
                m_backRight.setDesiredState(swerveModuleStates[3]);
            }
        );
    }

    public void updateOdometry() {
        m_odometry.update(
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            });
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
        updateOdometry();
    }
} */