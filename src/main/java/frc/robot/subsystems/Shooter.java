package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.*;


public class Shooter extends SubsystemBase {
    // The motor on the shooter wheel .
    private final TalonFX motor = new TalonFX(4);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(false);
    private final VoltageOut voltageOutFOC = new VoltageOut(0.0).withEnableFOC(true);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    // The shooter wheel encoder

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle angle = Radians.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity velocity = RadiansPerSecond.mutable(0);

    // Create a new SysId routine for characterizing the shooter.
    private final SysIdRoutine sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            null,         // Use default ramp rate (1 V/s)
                            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                            null,          // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                            state -> SignalLogger.writeString("state", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            volts -> motor.setControl(voltageOut.withOutput(volts)),
                            null,
                            this
                    )
            );

    /**
     * Creates a new Shooter subsystem.
     */
    public Shooter() {
        // AudioConfigs audioConfigs = new AudioConfigs()
        //         .withBeepOnBoot(true)
        //         .withAllowMusicDurDisable(true)
        //         .withBeepOnConfig(true);
        // CustomParamsConfigs customParamsConfigs = new CustomParamsConfigs()
        //         .withCustomParam0(0)
        //         .withCustomParam1(0);
        // CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
        //         .withStatorCurrentLimit(Amps.of(120))
        //         .withStatorCurrentLimitEnable(true)
        //         .withSupplyCurrentLimit(Amps.of(70))
        //         .withSupplyCurrentLimitEnable(true)
        //         .withSupplyCurrentLowerLimit(Amps.of(40))
        //         .withSupplyCurrentLowerTime(Seconds.of(1.0));
        // ClosedLoopRampsConfigs closedLoopRampsConfigs = new ClosedLoopRampsConfigs()
        //         .withDutyCycleClosedLoopRampPeriod(Seconds.of(0.0))
        //         .withVoltageClosedLoopRampPeriod(Seconds.of(0.0))
        //         .withTorqueClosedLoopRampPeriod(Seconds.of(0.0));
        // ClosedLoopGeneralConfigs closedLoopGeneralConfigs = new ClosedLoopGeneralConfigs();
        // closedLoopGeneralConfigs.ContinuousWrap = false;
        // // DifferentialConstantsConfigs differentialConstantsConfigs = new DifferentialConstantsConfigs()
        // //         .withPeakDifferentialDutyCycle(2.0)
        // //         .withPeakDifferentialVoltage(Volts.of(32))
        // //         .withPeakDifferentialTorqueCurrent(Amps.of(1600));
        // // DifferentialSensorsConfigs differentialSensorsConfigs = new DifferentialSensorsConfigs()
        // //         .withDifferentialRemoteSensorID(0)
        // //         .withDifferentialSensorSource(null)
        // //         .withDifferentialTalonFXSensorID(0);
        // FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        //         .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
        //         .withFeedbackRemoteSensorID(0)
        //         .withFeedbackRotorOffset(Rotations.of(0.0))
        //         .withVelocityFilterTimeConstant(Seconds.of(0))
        //         .withRotorToSensorRatio(1.0)
        //         .withSensorToMechanismRatio(1.0);
        // MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
        //         .withControlTimesyncFreqHz(Hertz.of(0))
        //         .withInverted(InvertedValue.CounterClockwise_Positive)
        //         .withNeutralMode(NeutralModeValue.Coast)
        //         .withDutyCycleNeutralDeadband(0)
        //         .withPeakForwardDutyCycle(1.0)
        //         .withPeakReverseDutyCycle(-1.0);
        // MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
        //         .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(0.))
        //         .withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.0))
        //         .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0.0))
        //         .withMotionMagicExpo_kV(0.0)
        //         .withMotionMagicExpo_kA(0.0);
        // OpenLoopRampsConfigs openLoopRampsConfigs = new OpenLoopRampsConfigs()
        //         .withDutyCycleOpenLoopRampPeriod(Seconds.of(0.0))
        //         .withTorqueOpenLoopRampPeriod(Seconds.of(0.0))
        //         .withVoltageOpenLoopRampPeriod(Seconds.of(0.0));
        // Slot0Configs slot0Configs = new Slot0Configs()
        //         .withGravityType(GravityTypeValue.Elevator_Static)
        //         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        //         .withKP(0.0)
        //         .withKI(0.0)
        //         .withKD(0.0)
        //         .withKS(0.0)
        //         .withKV(0.0)
        //         .withKA(0.0)
        //         .withKG(0.0);
        // TorqueCurrentConfigs torqueCurrentConfigs = new TorqueCurrentConfigs()
        //         .withPeakForwardTorqueCurrent(Amps.of(120)) // maybe use 120 so as not to piss off people.  Myself included
        //         .withPeakReverseTorqueCurrent(Amps.of(-120))
        //         .withTorqueNeutralDeadband(Amps.of(0.0));
        // VoltageConfigs voltageConfigs = new VoltageConfigs()
        //         .withPeakForwardVoltage(Volts.of(12.0))  // TODO: probably best not to use 16 and use 12 instead.  See previous comment
        //         .withPeakReverseVoltage(Volts.of(-12.0))
        //         .withSupplyVoltageTimeConstant(Seconds.of(0.0));


        // TalonFXConfiguration config = new TalonFXConfiguration()
        //         .withAudio(audioConfigs)
        //         .withCustomParams(customParamsConfigs)
        //         .withCurrentLimits(currentLimitsConfigs)
        //         .withClosedLoopRamps(closedLoopRampsConfigs)
        //         .withClosedLoopGeneral(null)
        //         // .withDifferentialConstants(null)
        //         // .withDifferentialSensors(null)
        //         .withFeedback(feedbackConfigs)
        //         // .withHardwareLimitSwitch(null)
        //         .withMotorOutput(motorOutputConfigs)
        //         .withMotionMagic(motionMagicConfigs)
        //         .withOpenLoopRamps(openLoopRampsConfigs)
        //         .withSlot0(slot0Configs)
        //         .withSlot1(new Slot1Configs())
        //         .withSlot2(new Slot2Configs())
        //         // .withSoftwareLimitSwitch(null)
        //         .withTorqueCurrent(torqueCurrentConfigs)
        //         .withVoltage(voltageConfigs);
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().apply(config);
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                motor.getPosition(),
                motor.getVelocity(),
                motor.getMotorVoltage());
        motor.optimizeBusUtilization();

        SignalLogger.start();

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
