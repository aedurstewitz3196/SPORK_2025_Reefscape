package frc.robot.commands;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOutput extends Command {
    private final Spark shooterMotor;
    public CoralOutput() {
        shooterMotor = new Spark(ElevatorConstants.shooter_spark_channel);
    }

    public void shoot(double power) {
            shooterMotor.set(power);
    }

}
