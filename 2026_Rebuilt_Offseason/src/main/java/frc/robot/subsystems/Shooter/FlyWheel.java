package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class FlyWheel extends SubsystemBase {
    private SparkFlex leftFlyWheelMotor = new SparkFlex(2, null);
    private SparkFlex rightFlyWheelMotor = new SparkFlex(3, null);

    public FlyWheel() {
        final SparkFlexConfig leftFlyWheelMotorConfig = new SparkFlexConfig();
        final SparkFlexConfig rightFlyWheelMotorConfig = new SparkFlexConfig();

        rightFlyWheelMotorConfig.idleMode(IdleMode.kCoast);

        leftFlyWheelMotorConfig.follow(rightFlyWheelMotor, true);

        leftFlyWheelMotor.configure(leftFlyWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        rightFlyWheelMotor.configure(rightFlyWheelMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command runCommand() {
        return this.run(() -> {
            rightFlyWheelMotor.set(0.1);
        }).finallyDo(() -> {
            rightFlyWheelMotor.stopMotor();
        });
    }
}
