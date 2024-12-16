package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-40, -62, Math.PI / 2))
                .strafeTo(new Vector2d(50, -61))
                .build());
                /*
                // score preloaded specimen
                .lineToY(-32)
                // push samples into observation zone
                .setTangent(Math.toRadians(310))
                .splineToConstantHeading(new Vector2d(36, -36), Math.PI / 2)
                .lineToY(-12)
                .setTangent(Math.toRadians(40))
                .splineToConstantHeading(new Vector2d(46, -50), Math.PI / 2)
                .lineToY(-14)
                .setTangent(Math.toRadians(45))
                .splineToConstantHeading(new Vector2d(60, -50), Math.PI /2)
                // turn robot around, pick up specimen
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(38, -46, Math.toRadians(-90)), Math.PI)
                .setTangent(3*Math.PI/2)
                .lineToY(-60)
                // move to bar and score specimen
                .setTangent(Math.toRadians(130))
                .splineToLinearHeading(new Pose2d(8, -40, Math.toRadians(-270)), Math.toRadians(120))
                .build());
                 */

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}