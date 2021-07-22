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
    private final CANPIDController spinPIDController;

    private final double offset;
    
    public SwerveModule(int driveMotorId, int spinMotorId, Translation2d location, double offset_) {
        this.location = location;
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);

        offset = offset_;

        spinAnalogEncoder = spinMotor.getAnalog(AnalogMode.kAbsolute);
        spinAnalogEncoder.setPositionConversionFactor(1 / 3.3);
        

        // TODO Velocity Conversion Factor
        spinPIDController = spinMotor.getPIDController();
        spinPIDController.setFeedbackDevice(spinAnalogEncoder);
        
        

    }

    public Translation2d getLocation() {
        return location;
    }

    public CANAnalog getSpinAnlogEncoder() {
        return spinAnalogEncoder;
    }

    public CANPIDController getSpinPIDController() {
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

    // DON'T USE THIS YET. IT ISN'T TESTED.
    // WE ALSO NEED TO ADD THE ACTUAL DRIVE WHEEL MOTORS
    public void setModuleState(SwerveModuleState curState, SwerveModuleState state) {
        if(state.angle.getDegrees() == 0 && state.speedMetersPerSecond == 0) {
            driveMotor.set(0);
            return;
        }
        //if(driveMotor.getDeviceId() != 7) return;
        // Desired angle (in degrees)
        double newAngle = state.angle.getDegrees();


        if(newAngle < 0) newAngle += 360;
        //angle = MathUtil.clamp(angle, 30, 330);
        double rot = newAngle / 360; 

        SmartDashboard.putNumber(" rot: ", rot);

        
        
        if(rot < 0.8) {
            rot += 0.5;
        }
        if(rot > 0.92) {
            rot -= 0.5;
        }

        spinPIDController.setReference(rot, ControlType.kPosition);

        boolean invertDrive = false; 
        invertDrive = rot < 0.9;


        //SmartDashboard.putNumber("SPINNY SPINNY!", state.speedMetersPerSecond);

        // TODO Should be done with PID and velocity instead
        double driveSpeed = state.speedMetersPerSecond;
        driveSpeed /= 4;
        if(invertDrive) driveSpeed *= -1;

        // TODO Temp
        //driveMotor.set(state.speedMetersPerSecond / 6.0);
        driveMotor.set(driveSpeed);
        //spinPIDController.setReference(0.30, ControlType.kPosition);
    }

    public double optimize(double currAngle, double newAngle){

        


        if(newAngle < 0) newAngle += 360;
        //angle = MathUtil.clamp(angle, 30, 330);
        double rot = newAngle / 360; 

        SmartDashboard.putNumber(" rot: ", rot);

        
        
        if(rot < 0.8) {
            rot += 0.5;
        }
        if(rot > 0.92) {
            rot -= 0.5;
        }

        return newAngle;
    }

    public double correct(double cons) {
        double start = (spinAnalogEncoder.getVoltage() / 3.3);
        return (start - cons) % 1;
    }
}

