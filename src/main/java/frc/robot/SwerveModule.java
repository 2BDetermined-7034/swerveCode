/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANAnalog;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * Add your docs here.
 */
public class SwerveModule {

    private final Translation2d location;

    private final CANSparkMax driveMotor;
    private final CANSparkMax spinMotor;

    private final CANAnalog spinAnalogEncoder;
    private final CANPIDController spinPIDController;
    
    public SwerveModule(int driveMotorId, int spinMotorId, Translation2d location) {
        this.location = location;
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        spinMotor = new CANSparkMax(spinMotorId, MotorType.kBrushless);

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

    public void setSpinPIDConstants(double kP, double kI, double kD) {
        spinPIDController.setP(kP);
        spinPIDController.setI(kI);
        spinPIDController.setD(kD);
    }

    // DON'T USE THIS YET. IT ISN'T TESTED.
    // WE ALSO NEED TO ADD THE ACTUAL DRIVE WHEEL MOTORS
    public void setModuleState(SwerveModuleState state) {
        // Desired angle (in degrees)
        double angle = state.angle.getDegrees();
        spinPIDController.setReference(angle / 360, ControlType.kPosition);

        // TODO Temp
        driveMotor.set(state.speedMetersPerSecond / 6.0);
    }



    public double correct(double cons){
        double start = (spinAnalogEncoder.getVoltage() / 3.3);



        return (start - cons) % 1;
    }
}

