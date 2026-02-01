package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@Autonomous
public class CloseRed6 extends LinearOpMode {

    VoltageSensor voltageSensor;

    public void runOpMode()
    {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        PeregrineShooter shooter = new PeregrineShooter(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(47, 55, Math.toRadians(45+180)));

        TrajectoryActionBuilder preloaded, moveToCloseStrip, collectCloseStrip, preloadedClose, leave, moveToMiddleStrip, collectMiddleStrip,
                preloadedMiddle, moveToFarStrip, collectFarStrip;

        preloaded = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(24.03);
        //.turn(Math.toRadians(-5))

        collectCloseStrip = preloaded.endTrajectory().fresh()
                .strafeTo(new Vector2d(20, 14))
                .turn(Math.toRadians(135));
        //.strafeTo(new Vector2d(-16.2, 10))

        moveToCloseStrip = collectCloseStrip.endTrajectory().fresh() //-31, 39
                .strafeTo(new Vector2d(65, 16));
        //.waitSeconds(1)
        preloadedClose = moveToCloseStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(16.2, 23.8))
                .turn(Math.toRadians(-135));
        moveToMiddleStrip = preloadedClose.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -8))
                .turn(Math.toRadians(135));
        leave = preloadedClose.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, 0));


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //collectCloseStrip
                new SequentialAction(

                )
        );
    }

}
