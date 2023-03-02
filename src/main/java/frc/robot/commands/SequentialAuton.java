package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SequentialAuton extends SequentialCommandGroup {
    public SequentialAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new ShiftDown(drive),
            new AutoArmSetPos(0.998, arm),
            new AutoStack1(drive).getRamseteCommand(),
            new ManipulatorOutAuton(manipulator, 2000),
            new AutoStack2(drive).getRamseteCommand(),
            new AutoArmSetPos(0.998, arm)
        );
    }
}
