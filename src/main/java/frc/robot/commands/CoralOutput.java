
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class CoralOutput extends CommandBase {
    private final PWMSparkMax shooterMotor;
    private final XboxController commandXboxController;

    public CoralOutput(PWMSparkMax shooterMotor, XboxController commandXboxController) {
        this.shooterMotor = shooterMotor;
        this.commandXboxController = commandXboxController;
    }

    public void initialize() {}

    public void execute() {
        if (commandXboxController.getRightTriggerAxis() > 0.8) {
            shooterMotor.set(1.0);
        } else {
            shooterMotor.set(0.0);
        }
    }

    public void end(boolean interrupted) {
        shooterMotor.set(0.0);
    }

    public boolean isFinished() {
        return false;
    }
}
