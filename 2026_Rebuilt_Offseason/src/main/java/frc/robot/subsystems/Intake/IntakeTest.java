package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeTest extends SubsystemBase {
    private SparkFlex intakingMotor = new SparkFlex(IntakeConstants.intakingMotorID, MotorType.kBrushless);

    public IntakeTest() {
        final SparkFlexConfig intakingMotorConfig = new SparkFlexConfig();

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
