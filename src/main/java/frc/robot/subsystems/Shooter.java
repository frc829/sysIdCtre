package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
        private final SparkMax motor = new SparkMax(17, SparkLowLevel.MotorType.kBrushless);

        private final MutVoltage appliedVoltage = Volts.mutable(0);
        private final MutAngle position = Rotations.mutable(0);
        private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);

        private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                        Volts.per(Seconds).of(1.0),
                                        Volts.of(4),
                                        Seconds.of(3.0) // Use default timeout (10 s)

                        ),
                        new SysIdRoutine.Mechanism(
                                        volts -> {
                                                motor.setVoltage(volts.baseUnitMagnitude());
                                                SignalLogger.writeDouble("Voltage", volts.baseUnitMagnitude());
                                        },
                                        log -> {
                                                // Record a frame for the shooter motor.
                                                log.motor("motor")
                                                                .voltage(appliedVoltage
                                                                                .mut_setMagnitude(motor.getBusVoltage()
                                                                                                * motor.getAppliedOutput()))
                                                                .angularPosition(position
                                                                                .mut_setMagnitude(motor.getEncoder()
                                                                                                .getPosition()))
                                                                .angularVelocity(
                                                                                velocity.mut_setMagnitude(motor
                                                                                                .getEncoder()
                                                                                                .getVelocity()));
                                        },
                                        this));

        /**
         * Creates a new Shooter subsystem.
         */
        public Shooter() {
                double factor = 1.0 / (4.0 * 3.0 * 5.0 * 32.0 / 24.0);
                EncoderConfig encoderConfig = new EncoderConfig()
                                .uvwMeasurementPeriod(16)
                                .uvwAverageDepth(2)
                                .positionConversionFactor(factor)
                                .velocityConversionFactor(factor / 60.0);
                SparkBaseConfig config = new SparkMaxConfig()
                                .apply(encoderConfig);
                config.idleMode(SparkBaseConfig.IdleMode.kBrake);
                config.inverted(true);
                motor.configure(
                                config,
                                SparkBase.ResetMode.kResetSafeParameters,
                                SparkBase.PersistMode.kPersistParameters);
        }

        public void stop() {
                motor.stopMotor();
        }

        public Command createStop() {
                return runOnce(this::stop);
        }

        public Command sysIdQuasistaticForward() {
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        }

        public Command sysIdQuasistaticReverse() {
                return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        }

        public Command sysIdDynamicForward() {
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        }

        public Command sysIdDynamicReverse() {
                return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
        }
}
