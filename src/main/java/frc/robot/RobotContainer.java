package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.CollectorIn;
import frc.robot.commands.CollectorOut;
import frc.robot.commands.CollectorToTilt;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.ZeroHeading;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Collector collector = new Collector();
  private final SendableChooser <Command> autoChooser;

  //note: pushing joystick forward is negative y (on the joystick)
  private final Joystick joystick = new Joystick(Constants.kDriverControllerPort);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //in WPILib, positive x = forward and positive y = left
    //to move forward, you need to push the joystick forward, so the xspeed has to be positive when the Y of the joystick is negative (forward)
    //to move left, you need to push the joystick to the left, so the yspeed has to be positive when the x of the joystick is negative (left)
    //twist is just twist
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(
      swerveSubsystem,
      () -> joystick.getRawAxis(SwerveConstants.driverYAxis),
      () -> -joystick.getRawAxis(SwerveConstants.driverXAxis),
      () -> -joystick.getRawAxis(SwerveConstants.driverRotAxis),
      () -> joystick.getRawButton(SwerveConstants.driverFieldOrientedButtonIndex)));
      
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(joystick, SwerveConstants.zeroHeadingButtonIndex).whileTrue(new ZeroHeading(swerveSubsystem));
    new JoystickButton(joystick, CollectorConstants.collectorInButtonIndex).whileTrue(new CollectorIn(collector));
    new JoystickButton(joystick, CollectorConstants.collectorOutButtonIndex).whileTrue(new CollectorOut(collector));
    new JoystickButton(joystick, CollectorConstants.tiltToPosButtonIndex).whileTrue(new CollectorToTilt(collector, 
    (joystick.getRawAxis(CollectorConstants.tiltToPosAxisIndex)+1.0)/2.0)); //axis is from -1 to 1, this scales it to be from 0 to 1
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
