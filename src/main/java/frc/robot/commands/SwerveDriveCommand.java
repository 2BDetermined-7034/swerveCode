/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier leftJoystickX;
  private final DoubleSupplier rightJoystickX;
  private final DoubleSupplier rightJoystickY;

  public SwerveDriveCommand(SwerveDrive swerveDrive, DoubleSupplier leftJoystickX, DoubleSupplier rightJoystickX, DoubleSupplier rightJoystickY) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrive = swerveDrive;
    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.rightJoystickY = rightJoystickY;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.printf("%s, %s, %s\n", leftJoystickX.getAsDouble(), rightJoystickX.getAsDouble(), rightJoystickY.getAsDouble());
    //swerveDrive.setChassisSpeeds(new ChassisSpeeds(rightJoystickY.getAsDouble(), rightJoystickX.getAsDouble(), leftJoystickX.getAsDouble() * Math.PI));
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 1, 0));
  };

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerveDrive.setDriveSpeed(0);
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
