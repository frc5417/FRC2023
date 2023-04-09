package frc.robot.commands.Autonomous.AutonomousGroups;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAutonCube;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardDockAutoConeScore;
import frc.robot.commands.Teleop.AutoArmSetPos;
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
