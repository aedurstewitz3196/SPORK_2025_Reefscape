package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class PrimeShooterCommand extends Command {
    private final CoralOutputSubsystem coralOutput;
    private static final double PULL_SPEED = 0.7;

    public PrimeShooterCommand(CoralOutputSubsystem coralOutput) {
        this.coralOutput = coralOutput;
        addRequirements(coralOutput);
    }

    @Override
    public void initialize() {
        // Start pulling if ShooterLazer is blocked
        if (coralOutput.isShooterLazerBlocked()) {
            coralOutput.pull(PULL_SPEED);
        }
        System.out.println("Priming shooter started");
    }

    @Override
    public void execute() {
        // Continue pulling if still blocked; do nothing if already unblocked
        if (coralOutput.isShooterLazerBlocked() && !coralOutput.isPulling()) {
            coralOutput.pull(PULL_SPEED);
        }
    }

    @Override
    public boolean isFinished() {
        // Stop when ShooterLazer is no longer blocked
        return !coralOutput.isShooterLazerBlocked();
    }

    @Override
    public void end(boolean interrupted) {
        coralOutput.stopPulling();
        System.out.println("Priming shooter " + (interrupted ? "interrupted" : "completed"));
    }
}