    package frc.robot.util;

    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import edu.wpi.first.wpilibj2.command.button.Trigger;
    import frc.robot.commands.CoralOutputSubsystem;
    import frc.robot.commands.DriveCommands;
    import frc.robot.subsystems.drive.Drive;
    import edu.wpi.first.wpilibj.XboxController;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.commands.ElevatorSubsystem;
    import frc.robot.commands.PrimeShooterCommand;
    import frc.robot.commands.SetElevatorHeightCommand;
    import frc.robot.commands.ShootCoralCommand;;

    public class ControllerBindings {
        private final XboxController driverController;
        private final XboxController operatorController; 
        private final Drive drive;
        private final ElevatorSubsystem elevator;
        private final CoralOutputSubsystem coralOutput;

        // Store buttons and triggers as fields to ensure they persist
        private final JoystickButton aButton;
        private final JoystickButton bButton;
        private final JoystickButton yButton;
        private final JoystickButton xButton;
        private final Trigger rightTrigger;
        private final Trigger leftTrigger;

        public ControllerBindings(XboxController driverController,XboxController operatorController, Drive driveSubsystem, ElevatorSubsystem elevator, CoralOutputSubsystem coralOutput) {
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
            aButton.onTrue(new SetElevatorHeightCommand(elevator, 49, false));
            bButton.onTrue(new SetElevatorHeightCommand(elevator, 28.5, false));
            yButton.onTrue(new SetElevatorHeightCommand(elevator, 14, false));
            xButton.onTrue(new SetElevatorHeightCommand(elevator, 3.3, false));

            // Trigger bindings with debug
            rightTrigger.onTrue(
                new InstantCommand(() -> System.out.println("Right Trigger Activated"))
                    .andThen(new ShootCoralCommand(coralOutput, 0.6, false))
                    .andThen(new WaitCommand(1.5))
                    .andThen(new SetElevatorHeightCommand(elevator, 3.3, false))
            );
            leftTrigger.onTrue(
                new InstantCommand(() -> System.out.println("Left Trigger Activated"))
                    .andThen(new ShootCoralCommand(coralOutput, -0.6, true))
            );
        }
    }