package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface{
    
    private TalonFX motor;

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

    public double setpoint = 0.0;

    public ShooterSubsystem() {
        initializeMotors();
    }

    //#region Initzalization

    private void initializeMotors() {
        motor = new TalonFX(SHOOTER_MOTOR_ID);
            motor.setNeutralMode(NeutralModeValue.Brake);

        var mConfigurator = motor.getConfigurator();
            mConfigurator.apply(PIDConfigs);
            mConfigurator.apply(currentLimits);
            mConfigurator.apply(feedbackConfig);
    }

    //#endregion
    //#region Measurment

    public AngularVelocity getSpeed(){
        return RPM.of(motor.getVelocity().getValue().in(RPM));
    }

    //#endregion
    //#region Movement

    private void setSpeed(double speed) {
        velocityVoltage.Velocity = speed;
        motor.setControl(velocityVoltage);
        setpoint = speed;
    }

    public void setPresetSpeed (ShooterState speed) {
        setSpeed(speed.speed);
    }

    //#endregion
    //#region Commands

    public Command setSpeedCommand(double newSpeed){
        return Commands.runOnce(() -> setSpeed(newSpeed));
    }

    @Override
    public void onModeSwitch() {
        setPresetSpeed(ShooterState.PLACE_HOLDER);
    }
}
