package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class ShootCoralCommand extends Command {
    private final CoralOutputSubsystem coralOutput;
    private final double shootPower;
    private final Timer timer;
    private final boolean runLong;
    private double SHOOT_DURATION = 0.5; // Seconds to run motor (adjust as needed)

    public ShootCoralCommand(CoralOutputSubsystem coralouter, double shootPower, boolean runLong) {
        this.coralOutput = coralouter;
        this.shootPower = shootPower;
        this.timer = new Timer();
        this.runLong = runLong;
        addRequirements(coralouter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (runLong) {
            SHOOT_DURATION = SHOOT_DURATION + 3;
        }

        coralOutput.shoot(shootPower); // Start shooting
        System.out.println("Firing coral at power: " + shootPower);
    }

    @Override
    public void execute() {
        // Keep shooting until duration is reached
    }

    @Override
    public boolean isFinished() {
        // End after SHOOT_DURATION seconds
        return timer.get() >= SHOOT_DURATION;
    }

    @Override
    public void end(boolean interrupted) {
        coralOutput.shoot(0); // Stop the motor (toggle off)
        timer.stop();
        System.out.println("Coral firing " + (interrupted ? "interrupted" : "completed"));
    }
}