package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.drive.DriveConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import org.littletonrobotics.junction.Logger;
import static frc.robot.subsystems.drive.DriveConstants.ppConfig;

import java.util.Arrays;
import java.util.List;

public class DockingController {

    private final VisionIO vision;
    private final Drive drivetrain;
    private boolean dockingActive = false;

    public DockingController(VisionIO vision, Drive drivetrain) {
        this.vision = vision;
        this.drivetrain = drivetrain;
    }

    public void initiateDocking() {
        // Try to get the latest pose estimate from LimelightHelpers
        Pose2d estimatedPose2d = LimelightHelpers.getBotPose2d_wpiBlue("limelight-sim");

        if (estimatedPose2d != null && estimatedPose2d.getTranslation().getNorm() > 0) {
            System.out.println("✅ Valid pose estimate from Limelight: " + estimatedPose2d);

            // Reset odometry to align with the estimated pose
            drivetrain.resetOdometry(estimatedPose2d);
        } else {
            System.out.println("❌ No valid pose estimate from Limelight, using odometry.");
        }

        var robotPose = drivetrain.getPose();
        System.out.println("Starting robot pose: " + robotPose);

        Pose3d robotPose3d = new Pose3d(
            robotPose.getTranslation().getX(),
            robotPose.getTranslation().getY(),
            0.0, // Z-coordinate is zero for 2D to 3D conversion
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())
        );

        // Find tags within range
        var tagsInRange = vision.getTagsInRange(robotPose3d, 3.0);
        if (!tagsInRange.isEmpty()) {
            // Sort tags by distance to robot
            tagsInRange.sort((tag1, tag2) -> Double.compare(
                tag1.pose.getTranslation().getDistance(robotPose3d.getTranslation()),
                tag2.pose.getTranslation().getDistance(robotPose3d.getTranslation())
            ));

            // Offset the target pose to stop 1 inch away from the tag
            double offsetDistance = Units.inchesToMeters(1.0);
            Transform3d offsetTransform = new Transform3d(
                new Pose3d(),
                new Pose3d(offsetDistance, 0.0, 0.0, new Rotation3d())
            );
            Pose3d adjustedTargetPose3d = tagsInRange.get(0).pose.transformBy(offsetTransform);
            Pose2d targetPose = adjustedTargetPose3d.toPose2d();

            dockingActive = true;

            // Debugging for target pose
            System.out.println("AprilTag Pose: " + tagsInRange.get(0).pose);
            System.out.println("Transformed Target Pose: " + adjustedTargetPose3d);

            Logger.recordOutput("Docking/TargetPose", new double[] {
                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()
            });

            // Create docking path
            createAndFollowPath(robotPose, targetPose);
        } else {
            System.out.println("No visible AprilTags to dock to!");
            dockingActive = false;
        }
    }

    private void createAndFollowPath(Pose2d startPose, Pose2d targetPose) {
        // Define waypoints
        List<Pose2d> poses = List.of(
            startPose,
            new Pose2d(
                (startPose.getX() + targetPose.getX()) / 2, // Midpoint X
                (startPose.getY() + targetPose.getY()) / 2, // Midpoint Y
                startPose.getRotation()
            ),
            targetPose
        );

        // Debugging for waypoints
        System.out.println("Waypoints: " + poses);

        // Convert poses to waypoints
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // Define constraints
        PathConstraints constraints = new PathConstraints(
            Units.feetToMeters(0.5), // Max linear velocity (M/S)
            Units.feetToMeters(0.25), // Max linear acceleration (M/S^2)
            Math.toRadians(45),      // Max angular velocity (Rad/S)
            Math.toRadians(30),      // Max angular acceleration (Rad/S^2)
            12.0,        // Nominal voltage (Volts)
            false                    // Should constraints be unlimited
        );

        // Create path
        PathPlannerPath dockingPath = new PathPlannerPath(
            waypoints,
            constraints,
            null, // IdealStartingState (optional)
            new GoalEndState(0.0, targetPose.getRotation()), // Stop and align to the target
            false // Forward path
        );

        // Provide initial chassis speeds based on the robot's current state
        ChassisSpeeds initialChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, // Initial velocity in X (m/s)
            0.0, // Initial velocity in Y (m/s)
            0.0, // Initial angular velocity (rad/s)
            startPose.getRotation() // Current robot rotation
        );

        // Generate trajectory
        RobotConfig robotConfig = ppConfig; // Use drivetrain-specific config
        PathPlannerTrajectory trajectory = dockingPath.generateTrajectory(
            initialChassisSpeeds, // Initial speeds
            startPose.getRotation(),
            robotConfig
        );

        // Debugging for trajectory
        System.out.println("Generated Trajectory: " + trajectory);

        // Follow trajectory
        AutoBuilder.followPath(dockingPath).schedule();

        // Test drivetrain response (manual test motion)
        drivetrain.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
            0.1, // Test forward motion
            0.0, // No strafe
            0.0, // No rotation
            startPose.getRotation()
        ));
    }

    public boolean isDockingActive() {
        return dockingActive;
    }

    public void completeDocking() {
        dockingActive = false;
        System.out.println("Docking complete.");
    }
}
