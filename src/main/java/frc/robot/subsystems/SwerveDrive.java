package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveDrive extends Subsystem {
    Command driveCommand(DoubleSupplier xSpeed, 
                        DoubleSupplier ySpeed, 
                        DoubleSupplier rot, 
                        BooleanSupplier fieldOriented, 
                        double periodSeconds);
}

