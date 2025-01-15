package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
        var robotPose = drivetrain.getPose();
        Pose3d robotPose3d = new Pose3d(
            robotPose.getTranslation().getX(),
            robotPose.getTranslation().getY(),
            0.0, // Z-coordinate is zero for 2D to 3D conversion
            new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians())
        );

        // Find tags within range
        var tagsInRange = vision.getTagsInRange(robotPose3d, 3.0);
        if (!tagsInRange.isEmpty()) {
            Pose2d targetPose = tagsInRange.get(0).pose.toPose2d();
            dockingActive = true;

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

        // Convert poses to waypoints
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        // Define constraints
        PathConstraints constraints = new PathConstraints(
            Units.feetToMeters(4.0), // Max linear velocity (M/S)
            Units.feetToMeters(2.0), // Max linear acceleration (M/S^2)
            Math.toRadians(180),     // Max angular velocity (Rad/S)
            Math.toRadians(90),      // Max angular acceleration (Rad/S^2)
            12.0,                    // Nominal voltage (Volts)
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

        // Generate trajectory
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
        chassisSpeeds.vxMetersPerSecond = DriveConstants.maxSpeedMetersPerSec;
        chassisSpeeds.vyMetersPerSecond = DriveConstants.maxSpeedMetersPerSec;
        RobotConfig robotConfig = ppConfig; // Use drivetrain-specific config
        PathPlannerTrajectory trajectory = dockingPath.generateTrajectory(
            chassisSpeeds,
            startPose.getRotation(),
            robotConfig
        );

        // Follow trajectory
        AutoBuilder.followPath(dockingPath).schedule();
    }

    public boolean isDockingActive() {
        return dockingActive;
    }
}