// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//TODO: Add actual values for almost all of these constants

public final class Constants {
    
    public static final class ShooterConstants {
    
        public static final int SHOOTER_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum ShooterState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private ShooterState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class IntakeConstants {
    
        public static final int INTAKE_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum IntakeState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private IntakeState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class HoodConstants {
        
        public static final int HOOD_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum HoodState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private HoodState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class TurretConstants {
        
        public static final int TURRET_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum TurretState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private TurretState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public class SwerveConstants {
        
        public static final double STEER_P = 0;
        public static final double STEER_I = 0;
        public static final double STEER_D = 0;

        public static final double STEER_S = 0;
        public static final double STEER_V = 0;
        public static final double STEER_A = 0;

        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(STEER_P).withKI(STEER_I).withKD(STEER_D)
            .withKS(STEER_S).withKV(STEER_V).withKA(STEER_A)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

        public static final double DRIVE_P = 0;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;

        public static final double DRIVE_S = 0;
        public static final double DRIVE_V = 0;

        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(DRIVE_P).withKI(DRIVE_I).withKD(DRIVE_D)
            .withKS(DRIVE_S).withKV(DRIVE_V);
            
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement driveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement steerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

        private static final Current SLIP_CURRENT = Amps.of(120.0);

        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(60.0))
                    .withStatorCurrentLimitEnable(true)
            );

        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        public static final CANBus CANBus = new CANBus("canivore", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity SPEED_AT_12VOLTS = MetersPerSecond.of(0);

        // Every 1 rotation of the azimuth results in coupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 0;

        private static final double DRIVE_GEAR_RATIO = 0;
        private static final double STEER_GEAR_RATIO = 0;
        private static final Distance WHEEL_RADIUS = Inches.of(0);

        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = true;

        private static final int PIGEON_ID = 1;

        // These are only used for simulation
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // Simulated voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(CANBus.getName())
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSlipCurrent(SLIP_CURRENT)
                .withSpeedAt12Volts(SPEED_AT_12VOLTS)
                .withDriveMotorType(driveMotorType)
                .withSteerMotorType(steerMotorType)
                .withFeedbackSource(steerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


        // Front Left
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 0;
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 0;
        private static final int FRONT_LEFT_ENCODER_ID = 1;
        private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0);
        private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
        private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;

        private static final Distance FRONT_LEFT_X_POSITION = Inches.of(0);
        private static final Distance FRONT_LEFT_Y_POSITION = Inches.of(0);

        // Front Right
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 0;
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 0;
        private static final int FRONT_RIGHT_ENCODER_ID = 1;
        private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(0);
        private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
        private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;

        private static final Distance FRONT_RIGHT_X_POSITION = Inches.of(0);
        private static final Distance FRONT_RIGHT_Y_POSITION = Inches.of(0);

        // Back Left
        private static final int BACK_LEFT_STEER_MOTOR_ID = 0;
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 0;
        private static final int BACK_LEFT_ENCODER_ID = 1;
        private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0);
        private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
        private static final boolean BACK_LEFT_ENCODER_INVERTED = false;

        private static final Distance BACK_LEFT_X_POSITION = Inches.of(0);
        private static final Distance BACK_LEFT_Y_POSITION = Inches.of(0);

        // Back Right
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 0;
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 0;
        private static final int BACK_RIGHT_ENCODER_ID = 1;
        private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(0);
        private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
        private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;

        private static final Distance BACK_RIGHT_X_POSITION = Inches.of(0);
        private static final Distance BACK_RIGHT_Y_POSITION = Inches.of(0);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeft = 
            ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID, 
                FRONT_LEFT_DRIVE_MOTOR_ID, 
                FRONT_LEFT_ENCODER_ID, 
                FRONT_LEFT_ENCODER_OFFSET,
                FRONT_LEFT_X_POSITION, 
                FRONT_LEFT_Y_POSITION, 
                INVERT_LEFT_SIDE, 
                FRONT_LEFT_STEER_MOTOR_INVERTED, 
                FRONT_LEFT_ENCODER_INVERTED
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRight = 
            ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID, 
                FRONT_RIGHT_DRIVE_MOTOR_ID, 
                FRONT_RIGHT_ENCODER_ID, 
                FRONT_RIGHT_ENCODER_OFFSET,
                FRONT_RIGHT_X_POSITION, 
                FRONT_RIGHT_Y_POSITION, 
                INVERT_RIGHT_SIDE, 
                FRONT_RIGHT_STEER_MOTOR_INVERTED, 
                FRONT_RIGHT_ENCODER_INVERTED
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeft = 
            ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID, 
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID, 
                BACK_LEFT_ENCODER_OFFSET,
                BACK_LEFT_X_POSITION, 
                BACK_LEFT_Y_POSITION, 
                INVERT_LEFT_SIDE, 
                BACK_LEFT_STEER_MOTOR_INVERTED, 
                BACK_LEFT_ENCODER_INVERTED
            );

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRight =
            ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID, 
                BACK_RIGHT_DRIVE_MOTOR_ID, 
                BACK_RIGHT_ENCODER_ID, 
                BACK_RIGHT_ENCODER_OFFSET,
                BACK_RIGHT_X_POSITION, 
                BACK_RIGHT_Y_POSITION, 
                INVERT_RIGHT_SIDE, 
                BACK_RIGHT_STEER_MOTOR_INVERTED, 
                BACK_RIGHT_ENCODER_INVERTED
            );
    }
}   
