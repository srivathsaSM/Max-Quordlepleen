package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;


public class CollectorToTilt extends Command {
  private final Collector collector;
  private double tilt;

  public CollectorToTilt(Collector collector, double tilt) {
    this.collector = collector;
    addRequirements(collector);
  }
  
  @Override
  public void initialize() {}
  
  @Override
  public void execute() {
    collector.setTilt(tilt);
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
