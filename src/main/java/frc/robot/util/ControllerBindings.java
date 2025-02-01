package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

public class ControllerBindings {
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController; 
    private final Drive drive;
    private final VisionIO vision;
    private final ControllerProfiles.ControllerProfile activeProfile;

    public ControllerBindings(CommandXboxController controller, Drive driveSubsystem, VisionIO vision) {
        this.controller = controller;
        this.drive = driveSubsystem;
        this.vision = vision;
        this.elevator = elevator;

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
                // Pay attention to the fact that some of these are inverted
                () -> controller.getRawAxis(activeProfile.leftYAxis), // Forward/backward
                () -> controller.getRawAxis(activeProfile.leftXAxis), // Strafe
                () -> controller.getRawAxis(activeProfile.rightXAxis) // Rotation
            ));

        // Example: Button A - Drive forward at half speed
        //controller.button(activeProfile.buttonA)
        //    .onTrue(null);

        // Example: Button B - Stop the drive subsystem
        //controller.button(activeProfile.buttonB)
        //    .onTrue(null);

        operatorController.button(activeProfile.buttonB)
            .whileTrue(Commands.run(() -> System.out.println("Operator B Button Pushed")));

        operatorController.button(activeProfile.buttonX)
            .whileTrue(Commands.run(() -> System.out.println("Operator X Button Pushed")));
            }
    //Cannot press same DPad button twice, must press another DPad button before pressing again.
    //Doesn't need fix because you are pressing one button at a time anyways.
    private void configureTriggerBindings() {
        // Driver Controller Triggers (Primary)
        new Trigger(() -> driverController.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Driver Right Trigger Pushed")));
    
        // Operator Controller Triggers (Secondary)
        new Trigger(() -> operatorController.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Operator Right Trigger Pushed - Shoot")));
        
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
                        RobotActions.executeDockToClosestAprilTag(drive, vision);
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