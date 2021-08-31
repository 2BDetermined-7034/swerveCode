/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Add your docs here.
 */
public class SwerveModule {

    private final Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax spinMotor;

    private final CANAnalog spinAnalogEncoder;
    private final edu.wpi.first.wpilibj.controller.PIDController spinPIDController;


    public SwerveModule(int driveMotorId, int spinMotorId, Translation2d location, double offset_) {
        this.location = location;
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);


        spinAnalogEncoder = spinMotor.getAnalog(AnalogMode.kAbsolute);
        spinAnalogEncoder.setPositionConversionFactor(1 / 3.3);
        

        // TODO Velocity Conversion Factor

        //NOTE: currently REVs PID controller doesn't support continous input, when they do, switch it to use the CANPIDcontroller

        //spinPIDController = spinMotor.getPIDController();
        //spinPIDController.setFeedbackDevice(spinAnalogEncoder);  
        spinPIDController = new edu.wpi.first.wpilibj.controller.PIDController(0.02, 0, 0, 0.02);
        spinPIDController.enableContinuousInput(-0.5, 0.5);  
        spinPIDController.setIntegratorRange(-0.005, 0.005);     


    }

    public Translation2d getLocation() {
        return location;
    }

    public CANAnalog getSpinAnlogEncoder() {
        return spinAnalogEncoder;
    }

    public edu.wpi.first.wpilibj.controller.PIDController getSpinPIDController() {
        return spinPIDController;
    }

    public CANSparkMax getDriveMotor() {
        return driveMotor;
    }

    public CANSparkMax getSpinMotor() {
        return spinMotor;
    }

    public void setSpinEncoderInverted(boolean inverted) {
        spinAnalogEncoder.setInverted(inverted);
    }

    public void setDriveMotorInverted(boolean inverted) {
        driveMotor.setInverted(inverted);
    }

    public void setSpinPIDConstants(double kP, double kI, double kD) {
        spinPIDController.setP(kP);
        spinPIDController.setI(kI);
        spinPIDController.setD(kD);
    }

    // DON'T USE THIS YET. IT ISN'T TESTED
    // WE ALSO NEED TO ADD THE ACTUAL DRIVE WHEEL MOTORS
    public void setModuleState(SwerveModuleState state) {
        if(state.angle.getDegrees() <= 0 && state.speedMetersPerSecond == 0 && spinPIDController.atSetpoint()) {
            driveMotor.set(0);
            spinMotor.set(0);
            return;
        }

        double curROT = spinAnalogEncoder.getPosition();
        double ROT = state.angle.getDegrees() / 360;
        
        SmartDashboard.putNumber(" Encoder", spinAnalogEncoder.getPosition());
        double move = MathUtil.clamp(spinPIDController.calculate(curROT, ROT), -1, 1);

        SmartDashboard.putNumber("move", move);
        spinMotor.set(move);


        /*
            SPED CONTROL
                TODO: Should be done with PID and velocity instead
        
        */

        double driveSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed; ;
        driveMotor.set(driveSpeed);
    }

    public double correct(double cons) {
        double start = (spinAnalogEncoder.getVoltage() / 3.3);
        return (start - cons) % 1;
    }

}

