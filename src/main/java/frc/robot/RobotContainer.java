/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveAutoDrive;
import frc.robot.commands.swerveControls.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveDrive swerveDrive = new SwerveDrive();
  
  private final XboxController controller = new XboxController(3);
  private final Command swerveAuto;


  public RobotContainer() {
    // Configure the button bindings
    //swerveAuto = new SwerveToPoint(swerveDrive, -1, -0.5, 0);
    swerveAuto = new SwerveAutoDrive(swerveDrive);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    swerveDrive.setDefaultCommand(new SwerveDriveCommand(swerveDrive,
      () -> controller.getX(Hand.kLeft) * -1,
      () -> controller.getX(Hand.kRight) * -1,
      () -> controller.getY(Hand.kRight) * -1,
      () -> controller.getTriggerAxis(Hand.kLeft),
      () -> controller.getTriggerAxis(Hand.kRight),
            controller::getPOV)
    );
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return swerveAuto;
  }
}
