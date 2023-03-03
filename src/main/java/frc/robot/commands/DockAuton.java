package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DockAuton extends SequentialCommandGroup {
    public DockAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoShiftDown(drive),
            new AutoArmSetPos(0.998, arm, true),
            new ManipulatorOutAuton(manipulator, 2000),
            new BackwardAutoConeScore(drive).getRamseteCommand()
        );
    }
}
