package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Interface for implementation of simulation. This lets you switch between it and the real robot.
 */
public interface SwerveDrive extends Subsystem {
    void drive(DoubleSupplier xSpeed, 
                        DoubleSupplier ySpeed, 
                        DoubleSupplier rot, 
                        BooleanSupplier fieldOriented);
}

