package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ElevatorConstants;
import frc.robot.util.ControllerProfiles;
import frc.robot.util.ControllerProfiles.ControllerProfile;

public class ControllerBindingsElevator {
    ElevatorCommands elevator;
    ControllerProfile activeProfile;

    public ControllerBindingsElevator() {
        elevator = new ElevatorCommands();
        this.activeProfile = ControllerProfiles.detectControllerProfile();

    if (activeProfile.buttonX == 1){
        elevator.set_height(ElevatorConstants.L1);
    }

    /*
    if (activeProfile.buttonY == 1){
        elevator.set_height(ElevatorConstants.L2);
    }
    */

    if (activeProfile.buttonB == 1){
        elevator.set_height(ElevatorConstants.L3);
    }

    if (activeProfile.buttonA == 1){
        elevator.set_height(ElevatorConstants.L4);
    }
}
}