package frc.robot.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

public class ControllerBindings {
    private final CommandXboxController controller;
    private final Drive drive;
    private final VisionIO vision;
    private final ControllerProfiles.ControllerProfile activeProfile;
    private int lastPOV = -1; // Tracks the previous POV state

    public ControllerBindings(CommandXboxController controller, Drive driveSubsystem, VisionIO vision) {
        this.controller = controller;
        this.drive = driveSubsystem;
        this.vision = vision;

        // Detect and set the active controller profile
        this.activeProfile = ControllerProfiles.detectControllerProfile();
    }
    public void configure(){
        configureButtonBindings();
        configureTriggerBindings();
    }
    /**
     * Configures button bindings dynamically based on the detected controller profile.
     * This is where we map our functional classes to controller buttons
     */
    private void configureButtonBindings() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                // Pay attention to the fact that some of these are inverted
                () -> controller.getRawAxis(activeProfile.leftYAxis), // Forward/backward
                () -> controller.getRawAxis(activeProfile.leftXAxis), // Strafe
                () -> controller.getRawAxis(activeProfile.rightXAxis) // Rotation
            ));

        
            // Face Button Testing   
        controller.button(activeProfile.buttonA)
            .whileTrue(Commands.run(() -> System.out.println("A Button Pushed"))); // A Button Testing

        controller.button(activeProfile.buttonB)
            .whileTrue(Commands.run(() -> System.out.println("B Button Pushed"))); // B Button Testing

        controller.button(activeProfile.buttonX)
            .whileTrue(Commands.run(() -> System.out.println("X Button Pushed"))); // X Button Testing

        

             /* Trigger Testing
        new Trigger(() -> controller.getRawAxis(activeProfile.leftTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Left Trigger Pushed"))); // Left Trigger Testing

        new Trigger(() -> controller.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Right Trigger Pushed"))); //Right Trigger Testing*/


            // Bumper Testing
        controller.button(activeProfile.rightBumper)
            .whileTrue(Commands.run(() -> System.out.println("Right Bumper Pushed"))); // Right Bumper Testing
    }
    //Cannot press same DPad button twice, must press another DPad button before pressing again.
    //Doesn't need fix because you are pressing one button at 
    private void configureTriggerBindings() {
        // Trigger Testing
        new Trigger(() -> controller.getRawAxis(activeProfile.leftTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Left Trigger Pushed"))); // Left Trigger Testing

        new Trigger(() -> controller.getRawAxis(activeProfile.rightTriggerAxis) > 0.5)
            .whileTrue(Commands.run(() -> System.out.println("Right Trigger Pushed"))); //Right Trigger Testing
        
        // Define a Trigger that monitors POV changes
        new Trigger(() -> {
            int currentPOV = controller.getHID().getPOV();
            boolean pressed = (currentPOV != -1 && currentPOV != lastPOV);
            if (pressed) {
                lastPOV = currentPOV; // Update the lastPOV state
            }
            return pressed; // True only on rising edge
        }).onTrue(new InstantCommand(() -> {
            // Execute specific actions based on the POV direction
            switch (controller.getHID().getPOV()) {
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
                    System.out.println("POV direction not defined: " + controller.getHID().getPOV());
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