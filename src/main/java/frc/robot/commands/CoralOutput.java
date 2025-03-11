package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
public class CoralOutput extends Command {
    private final Spark shooterMotor;
    private final DigitalInput FunnelLazer;
    private final DigitalInput ShooterLazer;

    public CoralOutput() {
        shooterMotor = new Spark(ElevatorConstants.shooter_spark_channel);
        FunnelLazer = new DigitalInput(1);
        ShooterLazer = new DigitalInput(2);
    }

    public void pull() {
        while (!FunnelLazer.get() && ShooterLazer.get()) {
            shooterMotor.set(.5);
        }
    }
    public void shoot(double power) {
            shooterMotor.set(power);
    }
    public boolean elevatorstopper() {
        if (ShooterLazer.get() && ShooterLazer.get()){
            return false;
        }
        return false;
    }

}
