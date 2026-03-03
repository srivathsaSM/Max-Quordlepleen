package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.Collect;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Collector collector = new Collector();

  //note: pushing joystick forward is negative y (on t he joystick)
  private final Joystick joystick = new Joystick(Constants.kJoystickPort);
  private final CommandXboxController controller = new CommandXboxController(Constants.kControllerPort);

  public RobotContainer() {
    //in WPILib, positive x = forward and positive y = left
    //to move forward, you need to push the joystick forward, so the xspeed has to be positive when the Y of the joystick is negative (forward)
    //to move left, you need to push the joystick to the left, so the yspeed has to be positive when the x of the joystick is negative (left)
    //twist is just twist
    
    if (!Constants.xboxControllerMode) {
      //joystick default command
      swerveSubsystem.setDefaultCommand(new SwerveJoystick(
        swerveSubsystem,
        () -> joystick.getRawAxis(SwerveConstants.driverYAxis),
        () -> -joystick.getRawAxis(SwerveConstants.driverXAxis),
        () -> -joystick.getRawAxis(SwerveConstants.driverRotAxis),
        () -> joystick.getRawAxis(SwerveConstants.sliderAxis),
        () -> joystick.getRawButton(SwerveConstants.strafeOnlyButtonIndex),
        () -> joystick.getRawButton(SwerveConstants.invertedButtonIndex)));
    } else {
      //controller -> getHID gets the underlying xbox controller object in the commandxboxcontroller object
      swerveSubsystem.setDefaultCommand(new SwerveJoystick(
        swerveSubsystem,
        () -> controller.getHID().getRawAxis(1), //left joystick y
        () -> -controller.getHID().getRawAxis(0), //left joystick x 
        () -> -controller.getHID().getRawAxis(4), //right joystick x
        () -> controller.getHID().getRawAxis(3), //right trigger (for slider functonality)
        () -> controller.getHID().getRawButton(6), //right bumper (strafe only)
        () -> controller.getHID().getRawButton(5))); //left bumper (invert)
    }
      
    configureBindings();

    SmartDashboard.putBoolean("Field Oriented", swerveSubsystem.isFieldOriented);
  }

  private void configureBindings() {
    if (!Constants.xboxControllerMode) { //Joystick Bindings (if not in xbox controller mode)
      //swerve bindings
      
      //NOTE: try false with the ignore disable for the zeroheading and straightenall functions (make changes to the xbox controller ones too if applicable)
      new JoystickButton(joystick, SwerveConstants.zeroHeadingButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()).ignoringDisable(true));
      new JoystickButton(joystick, SwerveConstants.straightenButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.straightenAll()).ignoringDisable(true));
      new JoystickButton(joystick, SwerveConstants.driverFieldOrientedButtonIndex).whileTrue(Commands.runOnce(() -> swerveSubsystem.toggleFieldOriented()).ignoringDisable(false));

      //collector bindings
      new JoystickButton(joystick, CollectorConstants.collectButtonIndex).whileTrue(new Collect(collector));
    } else { //Xbox Controller Bindings (if in xbox controller mode)
      //d-pad controls: up = 0 degrees, right = 90 degrees, down = 180 degrees, left = 270 degrees, unpressed = -1 degrees

      //swerve bindings
      new JoystickButton(controller.getHID(), 4).whileTrue(Commands.runOnce(() -> swerveSubsystem.zeroHeading()).ignoringDisable(true)); //Y button
      new JoystickButton(controller.getHID(), 3).whileTrue(Commands.runOnce(() -> swerveSubsystem.straightenAll()).ignoringDisable(true)); //X button
      new JoystickButton(controller.getHID(), 1).whileTrue(Commands.runOnce(() -> swerveSubsystem.toggleFieldOriented()).ignoringDisable(false)); //A button

      //collector bindings
      controller.povUp().whileTrue(new Collect(collector)); //dpad up for the collect command
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
