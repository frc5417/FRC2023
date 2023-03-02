package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestCommand2 extends CommandBase {
    public TestCommand2() { 

}
  @Override
  public void initialize() {
    System.out.println("testing 2");
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
