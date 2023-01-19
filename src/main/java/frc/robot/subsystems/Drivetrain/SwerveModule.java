// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class SwerveModule {
    private int tmp;
    private Spark turningMotor;
    private CANSparkMax driveMotor;

    // private Encoder turningEncoder;
    private Encoder turningEncoder;
    private RelativeEncoder driveEncoder;

    private PIDController turningPidController;

    public PIDController GetPIDTurning()
    {
        return turningPidController;
    }

    public SwerveModule(Drivetrain d, int driveChannel, int turnChannel, int channel1, int channel2,
            boolean driveMotorReversed, boolean turningMotorReversed, double EncoderResolution) {
        tmp = 0;
        turningMotor = new Spark(turnChannel);
        driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);

        turningMotor.setInverted(turningMotorReversed);
        driveMotor.setInverted(driveMotorReversed);

        
        resetMotors();

        turningEncoder = new Encoder(channel2, channel1);
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(
                Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
        // driveEncoder.setVelocityConversionFactor();
        // turningEncoder.setDistancePerPulse(360 / Constants.kEncoderResolution);

        turningEncoder.setDistancePerPulse(Math.PI * 2.0 / EncoderResolution);

        if (driveChannel == 0)
        {
            turningPidController = new PIDController(100,0,0);
        }else{
            turningPidController = new PIDController(5, 0, 0);
        }
        // SASHA'S NOTES: encoder distance will be in radians
        turningPidController.setTolerance(Math.PI * 1.0 / 180);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningPosition() {
        double pos = turningEncoder.getDistance();
        // if(pos < -1 * Math.PI)
        // {
        //     while(pos < -1 * Math.PI)
        //     {
        //         pos += Math.PI;
        //     }
        // }
        // else if(pos > Math.PI)
        // {
        //     while(pos > Math.PI)
        //     {
        //         pos -= Math.PI;
        //     }
        // }
        return pos;//Math.IEEERemainder(turningEncoder.getDistance(), Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        tmp = (tmp + 1) % 100;
        double speed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
        if(tmp % 100 == 0)
        {
            System.out.println("SPEED: " + state.speedMetersPerSecond);
            System.out.println("CURR: " + getTurningPosition());
            System.out.println("Target: " + state.angle.getRadians());
            System.out.println("CALC_speed: " + speed);
            System.out.println("ember: " + turningEncoder.getDistance());
        }
        driveMotor.set(state.speedMetersPerSecond);
        turningMotor.set(speed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public Rotation2d getAngle()
    {
        return getState().angle;
    }

    public void resetEncoders() {
        turningEncoder.reset();
        driveEncoder.setPosition(0);
    }

    private void resetMotors() {
        driveMotor.restoreFactoryDefaults();
        turningMotor.setSafetyEnabled(false);
    }

    public Encoder GetEncoder()
    {
        return turningEncoder;
    }
}
