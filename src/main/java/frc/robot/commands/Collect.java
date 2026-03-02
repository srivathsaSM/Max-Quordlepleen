package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class Collect extends Command {
  private final Collector collector;
  
  public Collect(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    collector.putCollectorOut();
    collector.setRollerSpeed(CollectorConstants.collectingRollerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    collector.putCollectorIn();
    collector.stopEverything();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
