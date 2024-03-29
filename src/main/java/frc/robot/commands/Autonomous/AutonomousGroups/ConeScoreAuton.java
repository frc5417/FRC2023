package frc.robot.commands.Autonomous.AutonomousGroups;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Autonomous.NonpathUtilityCommands.ManipulatorOutAuton;
import frc.robot.commands.Autonomous.RameseteCommands.BackwardAutoConeScore;
import frc.robot.commands.Autonomous.RameseteCommands.ForwardAutoConeScore;
import frc.robot.commands.Teleop.AutoArmSetPos;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreAuton extends SequentialCommandGroup {
    public ConeScoreAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ForwardAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAuton(manipulator, 1500)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardAutoConeScore(drive).getRamseteCommand()
            ),
            new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, true)
        );
    }
}
