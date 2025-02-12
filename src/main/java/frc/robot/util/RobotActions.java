package frc.robot.util;

import frc.robot.commands.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralOutput;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.util.ControllerProfiles;
import frc.robot.util.ControllerProfiles.ControllerProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ElevatorConstants;


/** Utility class for executing predefined robot actions. */
public class RobotActions {
    //static ElevatorCommands elevator;

    public static void executeDockToClosestAprilTag(Drive drive) {
        new frc.robot.util.DockingController(drive).driveToClosestAprilTag();
    }
    // The functions below are for moving the elevator up and down to the level you want
    public static void movetoL1() {
        //elevator.set_height(ElevatorConstants.L1);
    }
    public static void movetoL2() {
        //elevator.set_height(ElevatorConstants.L2);
    }
    public static void movetoL3() {
        //elevator.set_height(ElevatorConstants.L3);
    }
    public static void movetoL4() {
        //elevator.set_height(ElevatorConstants.L4);
    }
    public static void ShootCoral(XboxController commandXboxController, PWMSparkMax shooterMotor) {
                    CoralOutput CoralOutput = new CoralOutput(shooterMotor, commandXboxController);
    }
}