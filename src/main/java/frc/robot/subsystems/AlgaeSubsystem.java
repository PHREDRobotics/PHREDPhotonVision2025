package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem for controlling algae intake and outtake.
 */
public class AlgaeSubsystem extends SubsystemBase {
    private static final Timer timer = new Timer();
    public SparkMax leftAlgaeSparkMax = new SparkMax(AlgaeConstants.kLeftAlgaeCANId, MotorType.kBrushless);
    public SparkMax rightAlgaeSparkMax = new SparkMax(AlgaeConstants.kRightAlgaeCANId, MotorType.kBrushless);
    public SparkLimitSwitch forwardLimit = leftAlgaeSparkMax.getForwardLimitSwitch();

    public double algaeSpeed = AlgaeConstants.kAlgaeSpeed;

    public void Intake() {
        if (!forwardLimit.isPressed()) {
            timer.reset();
            leftAlgaeSparkMax.set(speedConvert(algaeSpeed)); // In last years code this was similar to pickUpNote() in
                                                             // IntakeSubsystem.
            rightAlgaeSparkMax.set(speedConvert(-algaeSpeed));
            timer.start();
        }
        // timer.reset();
        // leftAlgaeSparkMax.set(speedConvert(algaeSpeed)); // In last years code this
        // was similar to pickUpNote() in
        // // IntakeSubsystem.
        // rightAlgaeSparkMax.set(speedConvert(-algaeSpeed));
        // timer.start();
    }

    public void Outtake() {
        timer.reset();
        leftAlgaeSparkMax.set(speedConvert(-algaeSpeed));
        rightAlgaeSparkMax.set(speedConvert(algaeSpeed));
        timer.start();
    }

    /**
     * @param inSpeed
     * @return double
     */
    public double speedConvert(double inSpeed) {
        if (inSpeed < 0.2 && inSpeed > -0.2) {
            return 0.0;
        }
        return inSpeed;
    }

    public static boolean isTimeDone() {
        return timer.hasElapsed(Constants.AlgaeConstants.kAlgaeTime);
    }

    public void stop() {
        leftAlgaeSparkMax.set(0);
        rightAlgaeSparkMax.set(0);
    }

    public boolean isAlgaeLoaded() {
        return forwardLimit.isPressed() || SmartDashboard.getBoolean("Manual override press", false);
    }

    public static boolean algaeIsTimeDone() {
        return timer.hasElapsed(AlgaeConstants.kAlgaeTime);
    }

    @Override
    public void periodic() {
        // Slider variables
        algaeSpeed = SmartDashboard.getNumber("Algae/Algae Speed", algaeSpeed);
        SmartDashboard.putNumber("Algae/Algae Speed", algaeSpeed);
        SmartDashboard.putBoolean("Algae/Algae Limit Switch Pressed: ", forwardLimit.isPressed());

        // This will be called once per scheduler run
    }
}