package frc.robot.util;

import frc.robot.commands.CoralOutput;
import frc.robot.subsystems.drive.Drive;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ElevatorConstants;

/** Utility class for executing predefined robot actions. */
public class RobotActions {
    static ElevatorCommands elevator;
    static CoralOutput shooter;
    static boolean stopper;

    public RobotActions() {
        elevator = new ElevatorCommands();
        shooter = new CoralOutput();
        stopper = shooter.elevatorstopper();
    }

    public static void executeDockToClosestAprilTag(Drive drive) {
        new frc.robot.util.DockingController(drive).driveToClosestAprilTag();
    }

    // The functions below are for moving the elevator up and down to the level you want
    public void movetoL1() {
        System.out.println("Elevattor to L1");
        elevator.set_height(ElevatorConstants.L1, stopper);
    }
    public void movetoL2() {
        elevator.set_height(ElevatorConstants.L2, stopper);
    }
    public void movetoL3() {
        elevator.set_height(ElevatorConstants.L3, stopper);
    }
    public void movetoL4() {
        System.out.println("Elevator to L4");
        elevator.set_height(ElevatorConstants.L4, stopper);
    }
    public void ShootCoral(double power) {
        if (power >= 0.5 && power <= 0.8) {
            shooter.shoot(0.5);
        }
        else if (power > 0.8) {
            shooter.shoot(0.8);
        }
    }
}