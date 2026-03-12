package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Shooter;

public class Load extends Command {
  private final Collector collector;
  private final Shooter shooter;

  public Load(Shooter shooter, Collector collector) {
    this.shooter = shooter;
    this.collector = collector;
    addRequirements(shooter, collector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      if (!shooter.isNotePresent()) {
        collector.setRollerSpeed(CollectorConstants.collectorLoadingSpeed);
        shooter.runIndexer(ShooterConstants.indexerSpeed);
      } else {
        collector.setRollerSpeed(0.0);
        shooter.runIndexer(0.0);
      }
  }

  @Override
  public void end(boolean interrupted) {
    collector.setRollerSpeed(0.0);
    shooter.runIndexer(0.0);
  }

  @Override
  public boolean isFinished() {
    if (shooter.isNotePresent()) {
      return true;
    }
    return false;
  }
}
