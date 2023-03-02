package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand1 extends CommandBase {
    public TestCommand1() { 

}
  @Override
  public void initialize() {
    System.out.println("testing 1");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
