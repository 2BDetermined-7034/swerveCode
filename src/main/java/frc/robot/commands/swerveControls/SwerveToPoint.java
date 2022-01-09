// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerveControls;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj.controller.PIDController;

public class SwerveToPoint extends CommandBase {

  private final SwerveDrive swerveDrive;
  private final PIDController headingPID;
  private final PIDController translationPID;

  private final double targetX;
  private final double targetY;
  private final double targetHeading;

  private final double confidence;
  private boolean atPoint;

  /**
   * Command to drive a swerve drive to a point
   * @param swerveDrive the drivetrain you wish to control
   * @param setX the target x (meters away across field)
   * @param setY the target y (meters away along field)
   * @param setHeading the target heading (in degrees of the range 0 to -180/180)
   */
  public SwerveToPoint(SwerveDrive swerveDrive, double setX, double setY, double setHeading, double confidence) {
    this.swerveDrive = swerveDrive;

    this.targetX = setX;
    this.targetY = setY;
    this.targetHeading = setHeading;

    this.headingPID = new PIDController(0.5, 0,1);
    this.translationPID = new PIDController(0.5, 0 ,1);

    this.confidence = confidence;
    atPoint = false;

    headingPID.enableContinuousInput(-0.5, 0.5);
    addRequirements(swerveDrive);
  }

  public SwerveToPoint(SwerveDrive swerveDrive, double setX, double setY, double setHeading){
    this(swerveDrive, setX, setY, setHeading, 0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  /**
   * Use a PID controller to get to a setpoint
   */
  @Override
  public void execute() {
    double rightX = MathUtil.clamp(translationPID.calculate(swerveDrive.getPosAcross(), targetX), -1, 1);
    double rightY = MathUtil.clamp(translationPID.calculate(swerveDrive.getPosAlong(), targetY), -1, 1);
    double leftX = MathUtil.clamp(headingPID.calculate(swerveDrive.getCurrentAngle() / 360, targetHeading / 360), -1, 1);
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(rightY, rightX, leftX));
    if (rightX <= confidence && rightY <= confidence && leftX <= confidence) atPoint = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPoint;
  }
}
