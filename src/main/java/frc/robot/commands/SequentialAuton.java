package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SequentialAuton extends SequentialCommandGroup {
    public SequentialAuton(Drive drive, Arm arm) {
        /*addCommands(
            () -> {drive.shiftDown();},
            new ArmSetPos(0.720, arm),
            new ShiftDown(drive)
            
        );*/
    }
}
