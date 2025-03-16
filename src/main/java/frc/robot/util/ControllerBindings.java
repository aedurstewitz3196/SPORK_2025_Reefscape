package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralOutputSubsystem;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorSubsystem;
import frc.robot.commands.PrimeShooterCommand;
import frc.robot.commands.SetElevatorHeightCommand;
import frc.robot.commands.ShootCoralCommand;;

public class ControllerBindings {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController; 
    private final Drive drive;
    private final ControllerProfiles.ControllerProfile activeProfile;
    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final CoralOutputSubsystem coralOutput = new CoralOutputSubsystem();

    private int lastPOV = -1; // Tracks the previous POV state

    public ControllerBindings(CommandXboxController driverController,CommandXboxController operatorController, Drive driveSubsystem) {
        this.driverController = driverController;
        this.operatorController = operatorController;
        this.drive = driveSubsystem;

        // Detect and set the active controller profile
        this.activeProfile = ControllerProfiles.detectControllerProfile();
    }
    public void configure(){
        configureButtonBindings();
        configureTriggerBindings();
    }
    /*
    Configures button bindings dynamically based on the detected controller profile.
    This is where we map our functional classes to controller buttons
    */
    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive,
            () -> {
                double y = -driverController.getRawAxis(activeProfile.leftYAxis);
                return Math.abs(y) > 0.1 ? y : 0; // Deadband 0.1
            },
            () -> {
                double x = -driverController.getRawAxis(activeProfile.leftXAxis);
                return Math.abs(x) > 0.1 ? x : 0; // Deadband 0.1
            },
            () -> {
                double rotation = -driverController.getRawAxis(activeProfile.rightXAxis);
                return Math.abs(rotation) > 0.1 ? rotation :0;
            }
        ));

            //Primary Controller face buttons
        driverController.button(activeProfile.buttonA)
            .whileTrue(Commands.run(() -> System.out.println("Driver A Button Pushed")));
 
        driverController.button(activeProfile.buttonB)
            .whileTrue(Commands.run(() -> System.out.println("Driver B Button Pushed")));

        driverController.button(activeProfile.buttonX)
            .whileTrue(Commands.run(() -> System.out.println("Driver X Button Pushed")));
       
            // Secondary Controller face buttons
        operatorController.button(activeProfile.buttonA)
            .onTrue(new SetElevatorHeightCommand(elevator, 49, false));

        operatorController.button(activeProfile.buttonB)
            .onTrue(new SetElevatorHeightCommand(elevator, 28.5, false));
        
        operatorController.button(activeProfile.buttonY)
            .onTrue(new SetElevatorHeightCommand(elevator, 14, false));

        operatorController.button(activeProfile.buttonX)
            .onTrue(new SetElevatorHeightCommand(elevator, 3.3, false));
            }
    //Cannot press same DPad button twice, must press another DPad button before pressing again.
    //Doesn't need fix because you are pressing one button at a time anyways.
    private void configureTriggerBindings() {
        // Driver Controller Triggers (Primary)
        //new Trigger(() -> driverController.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
        //    .whileTrue(Commands.run(() -> RobotActions.ShootCoral(driverController.getRawAxis(activeProfile.rightTriggerAxis))));
    
        // Operator Controller Triggers (Secondary)
        new Trigger(() -> operatorController.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
            .onTrue(new ShootCoralCommand(coralOutput,0.6)
            .andThen(new SetElevatorHeightCommand(elevator, 3.3, false)));
        
            // Primary Controller DPad Setttings
        new Trigger(() -> {
            int currentPOV = driverController.getHID().getPOV();
            boolean pressed = (currentPOV != -1 && currentPOV != lastPOV);
            if (pressed) {
                lastPOV = currentPOV; // Update the lastPOV state
            }
            return pressed; // True only on rising edge
        }).onTrue(new InstantCommand(() -> {
            // Execute specific actions based on the POV direction
            switch (driverController.getHID().getPOV()) {
                case 0: // Up
                    executePOVCommand("Up", new InstantCommand(() -> {
                        System.out.println("POV Up pressed!");
                        // Add logic for Up direction
                    }));
                    break;
                case 90: // Right
                    executePOVCommand("Right", new InstantCommand(() -> {
                        System.out.println("POV Right pressed!");
                        // Add logic for Right direction
                    }));
                    break;
                case 180: // Down
                    executePOVCommand("Down", new InstantCommand(() -> {
                        System.out.println("POV Down pressed!");
                        //RobotActions.executeDockToClosestAprilTag(drive);
                    }));
                    break;
                case 270: // Left
                    executePOVCommand("Left", new InstantCommand(() -> {
                        System.out.println("POV Left pressed!");
                        // Add logic for Left direction
                    }));
                    break;
                default:
                    // Handle other or undefined POV angles (if necessary)
                    System.out.println("POV direction not defined: " + driverController.getHID().getPOV());
                    break;
            }
        })); 
        // Secondary Controller DPad Settings
        new Trigger(() -> {
            int currentPOV = driverController.getHID().getPOV();
            boolean pressed = (currentPOV != -1 && currentPOV != lastPOV);
            if (pressed) {
                lastPOV = currentPOV; // Update the lastPOV state
            }
            return pressed; // True only on rising edge
        }).onTrue(new InstantCommand(() -> {
            // Execute specific actions based on the POV direction
            switch (operatorController.getHID().getPOV()) {
                case 0: // Up
                    executePOVCommand("Up", new InstantCommand(() -> {
                        System.out.println("POV Up pressed!");
                        // Add logic for Up direction
                    }));
                    break;
                case 90: // Right
                    executePOVCommand("Right", new InstantCommand(() -> {
                        System.out.println("POV Right pressed!");
                        // Add logic for Right direction
                    }));
                    break;
                case 180: // Down
                    executePOVCommand("Down", new InstantCommand(() -> {
                        System.out.println("POV Down pressed!");
                        // Add logic for Down direction
                    }));
                    break;
                case 270: // Left
                    executePOVCommand("Left", new InstantCommand(() -> {
                        System.out.println("POV Left pressed!");
                        // Add logic for Left direction
                    }));
                    break;
                default:
                    // Handle other or undefined POV angles (if necessary)
                    System.out.println("POV direction not defined: " + driverController.getHID().getPOV());
                    break;
            }
        }));
    }
    private void executePOVCommand(String direction, Command command) {
        System.out.println("Executing POV " + direction + " command.");
        command.schedule();
        lastPOV = -1;
    }
}