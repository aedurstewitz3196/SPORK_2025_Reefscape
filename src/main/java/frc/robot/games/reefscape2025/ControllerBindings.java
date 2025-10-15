    package frc.robot.games.reefscape2025;

    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.WaitCommand;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.common.commands.DriveCommands;
import frc.robot.common.subsystems.drive.Drive;
import frc.robot.games.reefscape2025.commands.SetElevatorHeightCommand;
import frc.robot.games.reefscape2025.commands.SetElevatorHeightCommandWithEncoder;
import frc.robot.games.reefscape2025.commands.ShootCoralCommand;
import edu.wpi.first.wpilibj.XboxController;;

    public class ControllerBindings {
        private final XboxController driverController;
        private final XboxController operatorController; 
        private final Drive drive;
        private final ElevatorSubsystemWithEncoder elevator;
        private final CoralOutputSubsystem coralOutput;

        // Store buttons and triggers as fields to ensure they persist
        private final JoystickButton aButton;
        private final JoystickButton bButton;
        private final JoystickButton yButton;
        private final JoystickButton xButton;
        private final Trigger rightTrigger;
        private final Trigger leftTrigger;

        public ControllerBindings(XboxController driverController,XboxController operatorController, Drive driveSubsystem, ElevatorSubsystemWithEncoder elevator, CoralOutputSubsystem coralOutput) {
            this.driverController = driverController;
            this.operatorController = operatorController;
            
            this.drive = driveSubsystem;
            this.elevator = elevator;
            this.coralOutput = coralOutput;

            aButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
            bButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
            yButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
            xButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
            rightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.5);
            leftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.5);

            drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));
        
            // Secondary Controller face buttons
            aButton.onTrue(new SetElevatorHeightCommandWithEncoder(elevator, 20.5, false));
            bButton.onTrue(new SetElevatorHeightCommandWithEncoder(elevator, 15, false));
            yButton.onTrue(new SetElevatorHeightCommandWithEncoder(elevator, 7, false));
            xButton.onTrue(new SetElevatorHeightCommandWithEncoder(elevator, 3.3, false));

            // Trigger bindings with debug
            rightTrigger.onTrue(
                new InstantCommand(() -> System.out.println("Right Trigger Activated"))
                    .andThen(new ShootCoralCommand(coralOutput, 0.6, false))
                    .andThen(new WaitCommand(1.5))
                    .andThen(new SetElevatorHeightCommandWithEncoder(elevator, .5, false))
            );
            leftTrigger.onTrue(
                new InstantCommand(() -> System.out.println("Left Trigger Activated"))
                    .andThen(new ShootCoralCommand(coralOutput, -0.6, true))
            );
        }
    }