package frc.robot.commands;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ConeScoreMoveAuton extends SequentialCommandGroup {
    public ConeScoreMoveAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ForwardAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAuton(manipulator, 500)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardAutoConeScore2(drive).getRamseteCommand()
            ),
            new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, true),
            new BackwardAutoConeScore3(drive).getRamseteCommand(),
            new ManipulatorOutAuton(manipulator, 500)
        );
    }
}
