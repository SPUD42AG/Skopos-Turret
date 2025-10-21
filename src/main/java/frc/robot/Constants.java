// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
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
}
