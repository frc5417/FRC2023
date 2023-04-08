package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DockAuton extends SequentialCommandGroup {
    public DockAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(0.80, arm, true),
            new ManipulatorOutAutonCube(manipulator, 375),
            new BackwardDockAutoConeScore(drive).getRamseteCommand()
        );
    }
}
