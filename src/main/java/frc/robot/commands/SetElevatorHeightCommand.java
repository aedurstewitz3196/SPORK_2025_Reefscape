package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class SetElevatorHeightCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final double wantedHeight;
    private final boolean stopper;
    private final PIDController pidController;

    private static final double KP = 0.25;
    private static final double KI = 0.01;
    private static final double KD = 0.0;

    public SetElevatorHeightCommand(ElevatorSubsystem elevator, double wantedHeight, boolean stopper) {
        this.elevator = elevator;
        this.wantedHeight = wantedHeight;
        this.stopper = stopper;

        this.pidController = new PIDController(KP, KI, KD);
        pidController.setSetpoint(wantedHeight);
        pidController.setTolerance(elevator.getError());

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        pidController.reset();
        System.out.println("Moving elevator to height: " + wantedHeight);
    }

    @Override
    public void execute() {
        double currentHeight = elevator.getHeight();
        double output = pidController.calculate(currentHeight);
        output = Math.max(-elevator.getElevatorSpeed(), Math.min(elevator.getElevatorSpeed(), output));
        elevator.setElevatorSpeed(output);

        if (stopper) {
            elevator.setElevatorSpeed(0);
            pidController.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return stopper;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.setElevatorSpeed(0);
        } else {
            elevator.setElevatorSpeed(pidController.calculate(elevator.getHeight()));
        }
        System.out.println("Elevator stopped at height: " + elevator.getHeight());
    }

    public Command SetElevatorHeightCommand(ElevatorSubsystem elevator2, double d, boolean b) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'SetElevatorHeightCommand'");
    }
}