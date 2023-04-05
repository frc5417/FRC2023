package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class EngageScoreAuton extends SequentialCommandGroup {
    public EngageScoreAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ForwardEngageAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAutonCube(manipulator, 500)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardEngageAutoConeScore2(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, false),
                new BackwardEngageAutoConeScore3(drive).getRamseteCommand()
            ),
            new AutoBalance(drive)
        );
    }
}