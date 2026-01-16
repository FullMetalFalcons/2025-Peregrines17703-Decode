package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;
public class PeregrineShooter {

    private DcMotorEx launcher, intake, belt;
    private Servo pusher;

    public PeregrineShooter(HardwareMap hardwareMap, Telemetry telemetry)
    {
        launcher = (DcMotorEx) hardwareMap.dcMotor.get("launch_ball");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        belt = (DcMotorEx) hardwareMap.dcMotor.get("belt");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pusher = (Servo) hardwareMap.servo.get("pusher");
    }

    public class IntakeToPower implements Action
    {
        private double intakePower;
        private double intakeTime;
        private boolean intakeInitialized = false;
        private long startInNS;
        private double intakeDelay;

        public IntakeToPower(double IntakePower, double IntakeTime, double delay)
        {
            super();
            intakePower = IntakePower;
            intakeTime = IntakeTime;
            intakeDelay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!intakeInitialized)
            {
                startInNS = System.nanoTime();
                intakeInitialized = true;
            }

            if (System.nanoTime() < startInNS + TimeUnit.MILLISECONDS.toNanos((long) ((intakeTime) * 1000)))
            {
                intake.setPower(intakePower);
                return true;
            }
            else
            {
                intake.setPower(0);
                return false;
            }
        }
    }

    public IntakeToPower intakeToPower(double intakePower, double intakeTime, double delay)
    {
        return new IntakeToPower(intakePower, intakeTime, delay);
    }

    public class BeltToPower implements Action
    {
        private double beltPower;
        private double beltTime;
        private boolean beltInitialized = false;
        private long startInNS;
        private double beltDelay;

        public BeltToPower(double BeltPower, double BeltTime, double delay)
        {
            super();
            beltPower = BeltPower;
            beltTime = BeltTime;
            beltDelay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!beltInitialized)
            {
                startInNS = System.nanoTime();
                beltInitialized = true;
            }

            if (System.nanoTime() < startInNS + TimeUnit.MILLISECONDS.toNanos((long) ((beltTime) * 1000)))
            {
                belt.setPower(beltPower);
                return true;
            }
            else
            {
                belt.setPower(0);
                return false;
            }
        }
    }

    public BeltToPower beltToPower(double beltPower, double beltTime, double delay)
    {
        return new BeltToPower(beltPower, beltTime, delay);
    }


    public class LauncherToPower implements Action
    {
        private double launcherPower;
        private double launcherTime;
        private boolean launcherInitialized = false;
        private long startInNS;
        private double launcherDelay;

        public LauncherToPower(double LauncherPower, double LauncherTime, double delay)
        {
            super();
            launcherPower = LauncherPower;
            launcherTime = LauncherTime;
            launcherDelay = delay;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!launcherInitialized)
            {
                startInNS = System.nanoTime();
                launcherInitialized = true;
            }

            if (System.nanoTime() < startInNS + TimeUnit.MILLISECONDS.toNanos((long) ((launcherTime) * 1000)))
            {
                launcher.setPower(launcherPower);
                return true;
            }
            else
            {
                launcher.setPower(0);
                return false;
            }
        }
    }

    public LauncherToPower launcherToPower(double launcherPower, double launcherTime, double delay)
    {
        return new LauncherToPower(launcherPower, launcherTime, delay);
    }

    public class PusherToPosition implements Action {
        // Use constructor parameter to set target position
        private double targetPosition;

        public PusherToPosition(double rotatorPos) {
            super();
            targetPosition = rotatorPos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            pusher.setPosition(targetPosition);
            return false;
        }
    }

    public PusherToPosition pusherToPosition(double rotatorPos) {
        return new PusherToPosition(rotatorPos);
    }
}
