package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralOutput;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** Utility class for executing predefined robot actions. */
public class RobotActions {

    public static void executeDockToClosestAprilTag(Drive drive) {
        new frc.robot.util.DockingController(drive).driveToClosestAprilTag();
    }
    public static void ShootCoral(XboxController commandXboxController, PWMSparkMax shooterMotor) {
                    CoralOutput CoralOutput = new CoralOutput(shooterMotor, commandXboxController);
        }
    }




