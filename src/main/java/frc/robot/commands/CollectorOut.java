package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

public class CollectorOut extends Command {
  private final Collector collector;

  public CollectorOut(Collector collector) {
    this.collector = collector;
    addRequirements(collector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    collector.putOut();
  }

  @Override
  public void end(boolean interrupted) {
    collector.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
