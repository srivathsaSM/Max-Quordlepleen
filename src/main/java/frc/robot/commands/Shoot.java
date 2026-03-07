package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
  private final Shooter shooter;
  public Shoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooter.runShooter(ShooterConstants.shooterRPM);
    shooter.runIndexer(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
