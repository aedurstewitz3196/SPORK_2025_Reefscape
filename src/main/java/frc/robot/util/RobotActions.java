package frc.robot.util;

import frc.robot.commands.CoralOutput;
import frc.robot.subsystems.drive.Drive;


/** Utility class for executing predefined robot actions. */
public class RobotActions {
    //static ElevatorCommands elevator;
    static CoralOutput shooter;

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
    public static void ShootCoral(double power) {
        if (power >= 0.5 && power <= 0.8) {
            shooter.shoot(0.5);
        }
        else if (power > 0.8) {
            shooter.shoot(0.8);
        }
    }
}