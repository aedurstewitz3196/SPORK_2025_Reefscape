package frc.robot.util;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import java.util.List;
import java.util.Optional;

/**
 * DockingController is responsible for handling docking operations using vision data.
 */
public class DockingController {
    private final Drive drive;

    public DockingController(Drive drive) {
        this.drive = drive;
    }

    /**
     * Determines the closest AprilTag for docking and initiates path planning.
     */
    public void driveToClosestAprilTag() {
        Pose2d currentPose = drive.getPose();
        if (currentPose == null) {
            System.out.println("DockingController - No pose data available yet.");
            return;
        }

        Optional<AprilTag> closestTag = VisionConstants.aprilTagLayout.getTags().stream()
            .min((tag1, tag2) -> {
                double dist1 = currentPose.getTranslation().getDistance(tag1.pose.toPose2d().getTranslation());
                double dist2 = currentPose.getTranslation().getDistance(tag2.pose.toPose2d().getTranslation());
                return Double.compare(dist1, dist2);
            });

        if (closestTag.isPresent()) {
            AprilTag tag = closestTag.get();
            System.out.println("Closest AprilTag ID: " + tag.ID + " at " + tag.pose);

            // Approach distance to give the robot room to align before docking
            double approachDistance = 0.5;  // 0.5 meters (about 20 inches)
            double offsetDistance = 0.0762; // 3 inches in meters

            // Get the tag's pose and rotation
            Pose2d tagPose = tag.pose.toPose2d();
            Translation2d tagTranslation = tagPose.getTranslation();
            Rotation2d tagRotation = tagPose.getRotation();

            // Calculate the docking position by moving backward along the AprilTag's orientation
            Translation2d dockingTranslation = tagTranslation.minus(
                new Translation2d(offsetDistance * Math.cos(tagRotation.getRadians()),
                                  offsetDistance * Math.sin(tagRotation.getRadians()))
            );

            // Calculate the approach position, further back from the docking position
            Translation2d approachTranslation = tagTranslation.minus(
                new Translation2d((offsetDistance + approachDistance) * Math.cos(tagRotation.getRadians()),
                                  (offsetDistance + approachDistance) * Math.sin(tagRotation.getRadians()))
            );

            // Create Pose2d objects for docking and approach
            Pose2d dockingPose = new Pose2d(dockingTranslation, tagRotation);
            Pose2d approachPose = new Pose2d(approachTranslation, tagRotation);

            // Create waypoints from current position to the offset docking position
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, approachPose, dockingPose);
            
            // Define path constraints (adjust parameters as needed)
            PathConstraints constraints = new PathConstraints(.0254, .0254, 0.05, 0.05, 12.0, false);

            // Ensure the robot aligns perfectly with the AprilTag orientation at the end
            GoalEndState goalEndState = new GoalEndState(0.0, tagRotation);  // Ensures alignment with the tag's angle
            
            // Create the PathPlannerPath
            PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                null, // IdealStartingState can be null if unknown
                goalEndState
            );
            
            // Log the generated path waypoints for debugging
            System.out.println("Generated Path Waypoints:");
            waypoints.forEach(waypoint -> System.out.println(waypoint.toString()));

            // Create a FollowPathCommand to execute the path
            Command pathCommand = AutoBuilder.followPath(path);
            
            // Execute the path
            pathCommand.schedule();
        } else {
            System.out.println("No AprilTags found in the layout.");
        }
    }
}
