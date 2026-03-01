package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //note: pushing joystick forward is negative y (on the joystick)
  private final Joystick joystick = new Joystick(Constants.kJoystickPort);
  private final XboxController controller = new XboxController(Constants.kControllerPort);

  public RobotContainer() {
    //in WPILib, positive x = forward and positive y = left
    //to move forward, you need to push the joystick forward, so the xspeed has to be positive when the Y of the joystick is negative (forward)
    //to move left, you need to push the joystick to the left, so the yspeed has to be positive when the x of the joystick is negative (left)
    //twist is just twist
    
    //joystick default command
    // swerveSubsystem.setDefaultCommand(new SwerveJoystick(
    //   swerveSubsystem,
    //   () -> joystick.getRawAxis(SwerveConstants.driverYAxis),
    //   () -> -joystick.getRawAxis(SwerveConstants.driverXAxis),
    //   () -> -joystick.getRawAxis(SwerveConstants.driverRotAxis),
    //   () -> joystick.getRawAxis(SwerveConstants.sliderAxis),
    //   () -> !joystick.getRawButton(SwerveConstants.strafeOnlyButtonIndex),
    //   () -> !joystick.getRawButton(SwerveConstants.invertedButtonIndex)));

    //controller
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(
      swerveSubsystem,
      () -> controller.getRawAxis(1), //left joystick y
      () -> -controller.getRawAxis(0), //left joystick x 
      () -> -controller.getRawAxis(4), //right joystick x
      () -> controller.getRawAxis(3), //right trigger (for slider functonality)
      () -> controller.getRawButton(6), //right bumper (strafe only)
      () -> controller.getRawButton(5))); //left bumper (invert)
      
    configureBindings();

    SmartDashboard.putBoolean("Field Oriented", swerveSubsystem.isFieldOriented);
  }

  private void configureBindings() {
    new JoystickButton(joystick, SwerveConstants.zeroHeadingButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()).ignoringDisable(true));
    new JoystickButton(joystick, SwerveConstants.straightenButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.straightenAll()).ignoringDisable(true));
    new JoystickButton(joystick, SwerveConstants.driverFieldOrientedButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.toggleFieldOriented()).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
