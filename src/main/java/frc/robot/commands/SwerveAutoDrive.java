/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerveControls.SwerveToPoint;
import frc.robot.subsystems.SwerveDrive;


public class SwerveAutoDrive extends SequentialCommandGroup {

  public SwerveAutoDrive(SwerveDrive swerveDrive) {
    addCommands(
            new SwerveToPoint(swerveDrive, 0, 0 ,0, 0.01),
            new WaitCommand(0.5),
            new SwerveToPoint(swerveDrive, 2, 0, 0, 0.01),
            new WaitCommand(0.5),
            new SwerveToPoint(swerveDrive, 2, 2, 0, 0.01),
            new WaitCommand(0.5),
            new SwerveToPoint(swerveDrive, 0, 2, 0, 0.01),
            new WaitCommand(0.5),
            new SwerveToPoint(swerveDrive, 0, 0, 0, 0.01)

    );
  }
}
