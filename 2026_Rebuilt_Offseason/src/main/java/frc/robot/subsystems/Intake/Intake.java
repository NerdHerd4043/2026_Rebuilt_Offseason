package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

public class Intake extends SubsystemBase {
    private SparkMax intakingMotor = new SparkMax(IntakeConstants.intakingMotorID, MotorType.kBrushless);

    public Intake() {
        final SparkMaxConfig intakingMotorConfig = new SparkMaxConfig();

        intakingMotorConfig.idleMode(IdleMode.kBrake);

        intakingMotor.configure(intakingMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public Command runIntake() {
        return this.run(() -> {
            intakingMotor.set(IntakeConstants.intakeSpeed);
        }).finallyDo(() -> {
            intakingMotor.stopMotor();
        });
    }
}
