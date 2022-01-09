/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier leftJoystickX;
  private final DoubleSupplier rightJoystickX;
  private final DoubleSupplier rightJoystickY;
  private final DoubleSupplier leftJoystickTrigger;
  private final DoubleSupplier rightJoystickTrigger;
  private final DoubleSupplier snap;
  private final edu.wpi.first.wpilibj.controller.PIDController headingPID;

  public SwerveDriveCommand(SwerveDrive swerveDrive, DoubleSupplier leftJoystickX, DoubleSupplier rightJoystickX, DoubleSupplier rightJoystickY, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger, DoubleSupplier snap) {

    this.swerveDrive = swerveDrive;

    this.leftJoystickX = leftJoystickX;
    this.rightJoystickX = rightJoystickX;
    this.rightJoystickY = rightJoystickY;
    this.leftJoystickTrigger = leftTrigger;
    this.rightJoystickTrigger = rightTrigger;
    this.snap = snap;
    this.headingPID = new edu.wpi.first.wpilibj.controller.PIDController(3.2, 2.0, 0.01);
    headingPID.enableContinuousInput(-0.5, 0.5);
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {      
    double rightY = rightJoystickY.getAsDouble();
    double rightX = rightJoystickX.getAsDouble();
    double leftX = leftJoystickX.getAsDouble();
    double leftTrig = leftJoystickTrigger.getAsDouble();
    double rightTrig = rightJoystickTrigger.getAsDouble();
    double snapDir = snap.getAsDouble();


    if(Math.abs(rightY) <= 0.1 && Math.abs(rightX) <= 0.1){
      rightY = 0;
      rightX = 0;
    }

    if(Math.abs(leftX) <= 0.1) leftX = 0;

    leftX += 0.2 * leftTrig - 0.2 * rightTrig;
    leftX = MathUtil.clamp(leftX, -1, 1);


    //Manage snap heading control
    if (snapDir != -1){

      if (snapDir > 180) snapDir -= 360;
      
      SmartDashboard.putNumber("Snap", snapDir);
      snapDir *= -1;

      leftX =  MathUtil.clamp(headingPID.calculate(swerveDrive.getCurrentAngle() / 360, snapDir / 360), -1, 1);

    }
    SmartDashboard.putNumber("leftx", leftX);
    swerveDrive.debugNavX();

    swerveDrive.setChassisSpeeds(new ChassisSpeeds(rightY, rightX, leftX));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
