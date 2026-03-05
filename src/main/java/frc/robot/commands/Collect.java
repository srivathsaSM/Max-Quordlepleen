package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    SmartDashboard.putBoolean("Running Collector", true);
    if (!collector.isNotePresent()) {
      collector.putCollectorOut();
      collector.setRollerSpeed(CollectorConstants.collectingRollerSpeed);
    } else {
      collector.putCollectorIn();
      collector.setRollerSpeed(0.0);
    }
    // collector.putCollectorOut();
    // collector.setRollerSpeed(CollectorConstants.collectingRollerSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    collector.putCollectorIn();
    collector.setRollerSpeed(0.0);
    SmartDashboard.putBoolean("Running Collector", false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
