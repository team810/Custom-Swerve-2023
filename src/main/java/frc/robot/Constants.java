// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static  final double MaxSpeed = .40;
    public static final double MaxTurnSpeed = .75; // deggres per second
    public final class Auto
    {
        public static final String dir_path = "";
        public static final double MaxSpeed = 4;
        public static final double TurningSpeed = 20; // How long in seconds it takes to make one turn in place

        public final double FieldLength = 15.6;
        public static final double K_XController = 1;
        public static final double K_YController = 1;
        public static final double K_RController = 1;

    }

    /*
     * Vendor deps:
     * Rev: https://software-metadata.revrobotics.com/REVLib.json
     * NavX: https://www.kauailabs.com/dist/frc/2022/navx_frc.json
     */
    // Robot stats
    public static final double kEncoderResolution = 414.1666667;// 596.4
    //    public static final double kEncoderResolution = 415.0;
    public static final double GEAR_RATIO = 6.67;
    public static final double CIRCUMFERENCE = 4.0 * Math.PI;
    public static final double MAX_VELOCITY = 5; // units: m/s

    // CAN ID
    public static final int FRONT_LEFT_CAN = 1;
    public static final int FRONT_RIGHT_CAN = 2;
    public static final int BACK_RIGHT_CAN = 4;
    public static final int BACK_LEFT_CAN = 3;

    // PWM
    public static final int FRONT_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
    public static final int BACK_RIGHT = 3;
    public static final int BACK_LEFT = 4;

    // ENCODERS
    public static final int FRONT_LEFT_PORT_1 = 0;
    public static final int FRONT_LEFT_PORT_2 = 1;

    public static final int FRONT_RIGHT_PORT_1 = 2;
    public static final int FRONT_RIGHT_PORT_2 = 3;

    public static final int BACK_RIGHT_PORT_1 = 8;
    public static final int BACK_RIGHT_PORT_2 = 9;

    public static final int BACK_LEFT_PORT_1 = 6;
    public static final int BACK_LEFT_PORT_2 = 7;

    public static final double kMaxSpeedMetersPerSecond = .60;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.41;
    // Distance between front and back wheels on robot

    public static final double kWheelBase = 0.66;

    //    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
//            new Translation2d(kTrackWidth / 2, kWheelBase / 2), // FL
//            new Translation2d(-kTrackWidth / 2, kWheelBase / 2), // FR
//            new Translation2d(-kTrackWidth / 2, -kWheelBase / 2), // BL
//            new Translation2d(kTrackWidth / 2, -kWheelBase / 2)); // BR
    public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL // differnet in the video that I found.
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    public static final double kpDriveVel = 2.7066;


    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }



}
