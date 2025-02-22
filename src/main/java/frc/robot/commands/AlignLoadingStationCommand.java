package frc.robot.commands;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command to align to the loading station
 */
public class AlignLoadingStationCommand extends Command {
    private VisionSubsystem m_vision_subsystem;
    private SwerveSubsystem m_swerve_subsystem;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;

    private int targetTag;

    private boolean isFinished;

    // private static final int TAG_TO_CHASE = 5;
    // private static final Transform3d TAG_TO_GOAL = new Transform3d(
    // new Translation3d(1.0, 0.0, 0.0),
    // new Rotation3d(0.0, 0.0, Math.PI));

    /**
     * Create new AlignLoadingStationCommand
     * 
     * @param vision_subsystem The vision subsystem used for this command.
     * @param swerve_subsystem The swerve subsystem used for this command.
     */
    public AlignLoadingStationCommand(VisionSubsystem vision_subsystem, SwerveSubsystem swerve_subsystem) {
        m_vision_subsystem = vision_subsystem;
        m_swerve_subsystem = swerve_subsystem;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        targetTag = vision_subsystem.getTargetID();

        if (targetTag == -1) {
            isFinished = true;
        }

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_vision_subsystem);
        addRequirements(swerve_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // lastTarget(or whatever we call it)=null

    }

    /*
     * The fact that this is the execute code seems a little overkill
     * Seems like there could be a better way to do this.
     */
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // 1. get current pos
        Pose2d currentPose2d = m_swerve_subsystem.getPose();
        // 2. get target pos off of current targeted apriltag
        Pose2d targetPose2d = new Pose2d(); // just in case of errors

        targetPose2d = getTargetPose(VisionConstants.kAprilTags[targetTag - 1]);

        TrapezoidProfile.State targetxState = new TrapezoidProfile.State(targetPose2d.getX(), 0);
        TrapezoidProfile.State targetyState = new TrapezoidProfile.State(targetPose2d.getY(), 0);
        TrapezoidProfile.State targetomegaState = new TrapezoidProfile.State(targetPose2d.getRotation().getRadians(),
                0);
        // 3. set speeds using pid
        xSpeed = xController.calculate(currentPose2d.getX(), targetxState, X_CONSTRAINTS);
        ySpeed = yController.calculate(currentPose2d.getY(), targetyState, Y_CONSTRAINTS);
        xSpeed = omegaController.calculate(currentPose2d.getRotation().getRadians(), targetomegaState,
                OMEGA_CONSTRAINTS);
        // 4. make chassis speeds
        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, m_swerve_subsystem.getPose().getRotation());
        // 5. Output each module states to wheels
        m_swerve_subsystem.drive(chassisSpeeds, () -> false);

        // Smart dash varibs. The string ones proably work
        SmartDashboard.putNumber("X-Speed", xSpeed);
        SmartDashboard.putNumber("Y-Speed", ySpeed);
        SmartDashboard.putNumber("Turn-Speed", turningSpeed);
        SmartDashboard.putString("Target-Pose", targetPose2d.toString());
        SmartDashboard.putString("Current-Pose", currentPose2d.toString());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }

    private Pose2d getTargetPose(Pose2d tag) {
        Pose2d targetPose = new Pose2d(
                tag.getX()
                        + VisionConstants.kMetersFromLoadingStation * Math.cos(tag.getRotation().getRadians()),
                tag.getY()
                        + VisionConstants.kMetersFromLoadingStation * Math.sin(tag.getRotation().getRadians()),
                tag.getRotation().rotateBy(new Rotation2d(Math.PI)));

        return targetPose;
    }
}