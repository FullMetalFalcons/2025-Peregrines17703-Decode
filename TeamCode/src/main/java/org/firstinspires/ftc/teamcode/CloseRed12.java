package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

// Regular FTC Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Config
@Autonomous
public class CloseRed12 extends LinearOpMode {

    VoltageSensor voltageSensor;

    public void runOpMode()
    {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        PeregrineShooter shooter = new PeregrineShooter(hardwareMap, telemetry);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(47, 55, Math.toRadians(45+180)));

        TrajectoryActionBuilder preloaded, moveToCloseStrip, collectCloseStrip, preloadedClose, leave, moveToMiddleStrip, collectMiddleStrip,
                preloadedMiddle, moveToFarStrip, collectFarStrip, preloadedFar;

        preloaded = drive.actionBuilder(drive.localizer.getPose())
                .lineToY(24.03);
        //.turn(Math.toRadians(-5))

        moveToCloseStrip = preloaded.endTrajectory().fresh()
                .strafeTo(new Vector2d(20, 14))
                .turn(Math.toRadians(135));
        //.strafeTo(new Vector2d(-16.2, 10))

        collectCloseStrip = moveToCloseStrip.endTrajectory().fresh() //-31, 39
                .strafeTo(new Vector2d(65, 16));
        //.waitSeconds(1)
        preloadedClose = moveToCloseStrip.endTrajectory().fresh()
                /*.strafeTo(new Vector2d(16.2, 23.8))
                .turn(Math.toRadians(-135));*/
                .strafeToLinearHeading(new Vector2d(16.2, 23.8), Math.toRadians(-215));
        moveToMiddleStrip = preloadedClose.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, 8))
                .turn(Math.toRadians(135));
        collectMiddleStrip = moveToMiddleStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, -8), new TranslationalVelConstraint(15));
        preloadedMiddle = collectMiddleStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -8))
                .turn(-135)
                .strafeTo(new Vector2d(16.2, 23.8));
        moveToFarStrip = preloadedMiddle.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -32))
                .turn(Math.toRadians(135));
        collectFarStrip = moveToFarStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(65, -32), new TranslationalVelConstraint(15));
        preloadedFar = collectFarStrip.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -32))
                .turn(Math.toRadians(-135))
                .strafeTo(new Vector2d(16.2, 23.8));
        leave = preloadedFar.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, 0));


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                //collectCloseStrip
                new SequentialAction(
                        //move to preloaded spot while winding up launcher
                        new ParallelAction(
                                preloaded.build(),
                                shooter.launcherToPower(1400, 2, 0)
                        ),
                        //push first ball out
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset servo
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        //launch other 2 balls
                        new ParallelAction(
                                shooter.launcherToPower(1400, 1, 0),
                                shooter.beltToPower(1, 1, 0)
                        ),
                        //use servo as fail safe
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        //move to close strip
                        new ParallelAction(
                                moveToCloseStrip.build(),
                                shooter.launcherToPower(-600, 3, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //collect close strip
                        new ParallelAction(
                                collectCloseStrip.build(),
                                shooter.intakeToPower(1, 2.5, 0),
                                shooter.beltToPower(1, 2.5, 0)
                        ),
                        //move to shooting position while winding up launcher
                        new ParallelAction(
                                preloadedClose.build(),
                                shooter.launcherToPower(1400, 3, 0)
                        ),
                        //push first ball out
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset servo
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        //launch other 2 balls
                        new ParallelAction(
                                shooter.launcherToPower(1400, 1, 0),
                                shooter.beltToPower(1, 1, 0)
                        ),
                        //use servo as fail safe
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),


                        //move to middle strip
                        new ParallelAction(
                                moveToMiddleStrip.build(),
                                shooter.launcherToPower(-600, 3, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //collect middle strip
                        new ParallelAction(
                                collectMiddleStrip.build(),
                                shooter.intakeToPower(1, 2.5, 0),
                                shooter.beltToPower(1, 2.5, 0)
                        ),
                        //move to shooting position while winding up launcher
                        new ParallelAction(
                                preloadedMiddle.build(),
                                shooter.launcherToPower(1400, 3, 0)
                        ),
                        //push first ball out
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset servo
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        //launch other 2 balls
                        new ParallelAction(
                                shooter.launcherToPower(1400, 1, 0),
                                shooter.beltToPower(1, 1, 0)
                        ),
                        //use servo as fail safe
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),

                        //move to far strip
                        new ParallelAction(
                                moveToFarStrip.build(),
                                shooter.launcherToPower(-600, 3, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //collect far strip
                        new ParallelAction(
                                collectFarStrip.build(),
                                shooter.intakeToPower(1, 2.5, 0),
                                shooter.beltToPower(1, 2.5, 0)
                        ),
                        //move to shooting position while winding up launcher
                        new ParallelAction(
                                preloadedFar.build(),
                                shooter.launcherToPower(1400, 3, 0)
                        ),
                        //push first ball out
                        new ParallelAction(
                                shooter.launcherToPower(1400, .5, 0),
                                shooter.pusherToPosition(0)
                        ),
                        //reset servo
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        //launch other 2 balls
                        new ParallelAction(
                                shooter.launcherToPower(1400, 1, 0),
                                shooter.beltToPower(1, 1, 0)
                        ),
                        //use servo as fail safe
                        new ParallelAction(
                                shooter.launcherToPower(1400, .25, 0),
                                shooter.pusherToPosition(.75)
                        ),
                        leave.build()

                )
        );
    }

}
