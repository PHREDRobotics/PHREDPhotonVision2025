package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for implementation of simulation. This lets you switch between it and the real robot.
 */
public interface SwerveDrive extends Subsystem {
    void drive(DoubleSupplier xSpeed, 
                        DoubleSupplier ySpeed, 
                        DoubleSupplier rot, 
                        BooleanSupplier fieldOriented);
    void drive(ChassisSpeeds speeds, BooleanSupplier fieldOriented);
    Pose2d getPose();
    void updateOdometry();
    void resetOdometry(Pose2d pose);
}