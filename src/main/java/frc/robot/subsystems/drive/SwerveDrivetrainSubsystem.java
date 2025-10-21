package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;;

public class SwerveDrivetrainSubsystem extends com.ctre.phoenix6.swerve.SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    public SwerveDrivetrainSubsystem(
        SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, 
        Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, 
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(
            TalonFX::new,
            TalonFX::new, 
            CANcoder::new, 
            drivetrainConstants, 
            odometryUpdateFrequency, 
            odometryStandardDeviation, 
            visionStandardDeviation, 
            modules
        );
    }
}
