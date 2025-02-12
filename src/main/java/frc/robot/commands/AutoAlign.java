// /**
//  * Writen by Armando Mac Beath
//  * 
//  * {@MÆTH}
//  */

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.cameraserver.CameraServer;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// public class AutoAlign extends Command {
//     private final SlewRateLimiter xLimiter;
//     private final SlewRateLimiter yLimiter;
//     private final SlewRateLimiter giroLimiter;

//     private final PIDController drivePID;
//     private final PIDController strafePID;
//     private final PIDController rotationPID;

//     private final double driveOffset;
//     private final double strafeOffset;
//     private final double rotationOffset;

//     private final SwerveSubsystem swerve;
//     private final VisionSubsystem vision;

//     public AutoAlign() {
//         this.swerve = SwerveSubsystem.getInstance();
//         this.vision = VisionSubsystem.getInstance();

//         this.xLimiter = new SlewRateLimiter(SwerveConstants.kDriveAccelerationLimiter);
//         this.yLimiter = new SlewRateLimiter(SwerveConstants.kDriveAccelerationLimiter);
//         this.giroLimiter = new SlewRateLimiter(SwerveConstants.kRotationAccelerationLimiter);

//         /**
//          * PID Controllers for the align
//          */
//         this.drivePID = new PIDController(
//                 VisionConstants.kPdrive,
//                 VisionConstants.kIdrive,
//                 VisionConstants.kDdrive);

//         this.strafePID = new PIDController(
//                 VisionConstants.kPstrafe,
//                 VisionConstants.kIstrafe,
//                 VisionConstants.kDstrafe);

//         this.rotationPID = new PIDController(
//                 VisionConstants.kProtation,
//                 VisionConstants.kIrotation,
//                 VisionConstants.kDrotation);

//         /**
//          * Offsets for the limelight
//          */
//         this.driveOffset = 2.1;
//         this.strafeOffset = -0.2;
//         this.rotationOffset = 10.2;

//         addRequirements(swerve);
//         addRequirements(vision);
//     }

//     @Override
//     public void initialize() {
//         /**
//          * Start a camera server so we can visualize the limelight on the Shuffleboard
//          */
//         CameraServer.startAutomaticCapture();
//     }

//     @Override
//     public void execute() {
//         double velForward = 0;
//         double velStrafe = 0;
//         double velGiro = 0;

//         if (VisionSubsystem.hasValidTarget()) {
//             velForward = drivePID.calculate(limelight.getArea(), driveOffset);
//             velStrafe = strafePID.calculate(limelight.getXDistance(), strafeOffset);
//             velGiro = -rotationPID.calculate(limelight.getYaw(), rotationOffset);
//         } else if (limelight.hasValueTargets() == false) {
//             velForward = 0;
//             velStrafe = 0;
//             velGiro = 0.4;
//         } else {
//             velForward = 0;
//             velStrafe = 0;
//             velGiro = 0;
//         }

//         // 2. Apply deadband
//         velForward = Math.abs(velForward) > OIConstants.kDeadband ? velForward : 0.0;
//         velStrafe = Math.abs(velStrafe) > OIConstants.kDeadband ? velStrafe : 0.0;
//         velGiro = Math.abs(velGiro) > OIConstants.kDeadband ? velGiro : 0.0;

//         // 3. Make the driving smoother
//         velForward = xLimiter.calculate(velForward) * 3;
//         velStrafe = yLimiter.calculate(velStrafe) * 3;
//         velGiro = giroLimiter.calculate(velGiro) * 5;

//         // 4. Construct desired chassis speeds
//         ChassisSpeeds chassisSpeeds;

//         // Relative to robot
//         chassisSpeeds = new ChassisSpeeds(velForward, velStrafe, velGiro);

//         // 5. Convert chassis speeds to individual module states
//         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

//         // 6. Output each module states to wheels
//         swerve.setModuleStates(moduleStates);

//     }

// }