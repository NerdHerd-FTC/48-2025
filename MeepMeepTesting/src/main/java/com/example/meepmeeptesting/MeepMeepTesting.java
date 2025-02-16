package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 50, Math.toRadians(180), Math.toRadians(180), 16)
                .build();

        Pose2d initPose = new Pose2d(-12,-64,Math.toRadians(90));

        Pose2d scorePose = new Pose2d(-50, -50, Math.toRadians(45));

        Pose2d spikePose = new Pose2d(-49.00, -40.00, Math.toRadians(-90.00));
        Pose2d spikeForwardPose = new Pose2d(spikePose.position.x, spikePose.position.y + 0.50, spikePose.heading.toDouble());
        double forwardMoveY = spikeForwardPose.position.y;

        Action startToBasket = myBot.getDrive().actionBuilder(initPose)
                .splineToLinearHeading(scorePose, Math.toRadians(-135.00))
                .build();

        Action basketToFirstSpike = myBot.getDrive().actionBuilder(scorePose)
                .splineToLinearHeading(spikePose, Math.toRadians(90.00))
                .build();

        Action forwardMove = myBot.getDrive().actionBuilder(spikePose)
                .setTangent(Math.toRadians(-90))
                .lineToY(forwardMoveY)
                .build();

        Action firstSpikeToBasket = myBot.getDrive().actionBuilder(spikeForwardPose)
                .splineToLinearHeading(scorePose, Math.toRadians(-135.00))
                .build();

        Action park = myBot.getDrive().actionBuilder(scorePose)
                .splineToLinearHeading(new Pose2d(36,-60,Math.toRadians(90)),Math.toRadians(0))
                .build();

        myBot.runAction(new SequentialAction(
                // go to basket
                new ParallelAction(
                        startToBasket
                ),
                // score
                new SleepAction(1),
                new SleepAction(1),
                new SleepAction(0.5),
                // go to spike mark
                new ParallelAction(
                        basketToFirstSpike
                ),
                // pick up
                new SleepAction(1),
                forwardMove,
                new SleepAction(0.5),
                new SleepAction(1),
                new ParallelAction(
                        firstSpikeToBasket
                ),
                new SleepAction(1),
                new SleepAction(1),
                new SleepAction(0.5),
                park
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}