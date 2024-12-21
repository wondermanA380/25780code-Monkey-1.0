package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NonNls;

import jdk.tools.jlink.internal.plugins.VendorBugURLPlugin;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // Declare our first bot
        RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(16.5,16.5)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.2)
                .build();

        myFirstBot.runAction(myFirstBot.getDrive().actionBuilder(new Pose2d(10, -62, Math.toRadians(90)))
                .lineToY(-52)
                .strafeTo(new Vector2d(31.5, -52))
                .setTangent(Math.toRadians(90))
                .lineToY(-58)

                .lineToY(-40)
                .strafeToLinearHeading(new Vector2d(0,-40),80)

                .strafeTo(new Vector2d(0,-34))

                .strafeTo(new Vector2d(0,-52))
                .strafeToLinearHeading(new Vector2d(31.5,-52),0)
                        .strafeTo(new Vector2d(31.5,-58))
                .strafeToLinearHeading(new Vector2d(0,-40),80)
                        .strafeTo(new Vector2d(0,-34))
                                .build());
//                moveForward(10,0.6);
//        strafeRight(53,0.6);
//        moveBackward(5.6, 0.6);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myFirstBot)
                .start();

//                .addEntity(mySecondBot)

    }
}