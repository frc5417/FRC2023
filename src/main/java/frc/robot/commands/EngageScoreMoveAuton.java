package frc.robot.commands;
import frc.robot.subsystems.*;
import frc.robot.Constants.ManipulatorConstants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

public class EngageScoreMoveAuton extends SequentialCommandGroup {
    public EngageScoreMoveAuton(Drive drive, Arm arm, Manipulator manipulator) {
        addCommands(
            new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, true),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                //TODO - increase how much it goes forward before competition
                new ForwardEngageAutoConeScore(drive).getRamseteCommand()
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new ManipulatorOutAuton(manipulator, 500)
            ),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armThirdScorePoint, arm, false),
                new BackwardEngageAutoConeScore2(drive).getRamseteCommand()
            ),
            new DriveBreakAuton(drive),
            new ParallelRaceGroup(
                new AutoArmSetPos(ManipulatorConstants.armIntakePoint, arm, false),
                new BackwardEngageAutoConeScore4(drive).getRamseteCommand()
            ),
            new DriveCoastAuton(drive),
            new ForwardEngageAutoConeScore2(drive).getRamseteCommand(),
            new AutoBalance(drive)
        );
    }
}