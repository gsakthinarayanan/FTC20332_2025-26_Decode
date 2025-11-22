package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Near Side Autonomous", group = "Decode2025")
public class RedNearAutonomousOp extends DecodeAutonomousOp {
    Pose startPose = new Pose(124, 124, Math.toRadians(135)); // Start Pose of our robot.
    Pose motifPose = new Pose(80, 80,  Math.toRadians(90));
    Pose scorePose = new Pose(80, 96,  Math.toRadians(43));

    Pose straffLeftPose = new Pose(80, 115,  Math.toRadians(43));
    Pose navigateToPickup1Pose = new Pose(84, 80,  Math.toRadians(0));
    Pose grabPickup1Pose = new Pose(100, 80,  Math.toRadians(0));
    Pose grabPickup2Pose = new Pose(105, 80,  Math.toRadians(0));
    Pose grabPickup3Pose = new Pose(110, 80,  Math.toRadians(0));

    private Path navigateToMotif, scorePreloaded, straffLeft;
    private PathChain scorePreloadedArtifacts, navigateToPickup1, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        navigateToMotif = new Path(new BezierLine(startPose, motifPose));
        navigateToMotif.setLinearHeadingInterpolation(startPose.getHeading(), motifPose.getHeading());

        //Here is an example for Linear Interpolation
        //navigateToMotif.setLinearHeadingInterpolation(startPose.getHeading(), motifPose.getHeading());

        //Here is an example for Constant Interpolation
        //scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        scorePreloaded = new Path(new BezierLine(motifPose, scorePose));
        scorePreloaded.setLinearHeadingInterpolation(motifPose.getHeading(), scorePose.getHeading());

        scorePreloadedArtifacts = follower.pathBuilder()
                .addPath(new BezierLine(motifPose, scorePose))
                .setLinearHeadingInterpolation(motifPose.getHeading(), scorePose.getHeading())
                .build();

        navigateToPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, navigateToPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), navigateToPickup1Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(navigateToPickup1Pose, grabPickup1Pose))
                .setLinearHeadingInterpolation(navigateToPickup1Pose.getHeading(), grabPickup1Pose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(grabPickup1Pose, grabPickup2Pose))
                .setLinearHeadingInterpolation(grabPickup1Pose.getHeading(), grabPickup2Pose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(grabPickup2Pose, grabPickup3Pose))
                .setLinearHeadingInterpolation(grabPickup2Pose.getHeading(), grabPickup3Pose.getHeading())
                .build();

        straffLeft = new Path(new BezierLine(scorePose, straffLeftPose));
        straffLeft.setLinearHeadingInterpolation(scorePose.getHeading(), straffLeftPose.getHeading());
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                follower.followPath(navigateToMotif);
                setPathState(1);
                break;
            case 1:
                 /*You could check for
                - Follower State: "if(!follower.isBusy()) {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"*/

                // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position
                if(!follower.isBusy()) {
                    aprilTagID = identifyAprilTagID();
                    prePositionCarouselForDelivery(aprilTagID);
                    follower.followPath(scorePreloadedArtifacts, true);
                    if (follower.getPose().getHeading() < Math.toRadians(50)) {
                        scoreArtifacts(aprilTagID, true);
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(navigateToPickup1,false);
                    deactivateDeliveryRollers();
                    activateIntakeSystem();
                    carousselIndexPurpleLeftDown();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1,0.5, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    carousselIndexPurpleRightDown();
                    follower.followPath(grabPickup2,0.2, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    carouselIndexGreenDown();
                    follower.followPath(grabPickup3,0.2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    deactivateIntakeSystem();
                    prePositionCarouselForDelivery(aprilTagID);
                    follower.followPath(scorePreloadedArtifacts, true);
                    if (follower.getPose().getHeading() > Math.toRadians(38)) {
                        scoreArtifacts(aprilTagID, true);
                        setPathState(7);
                    }
                    deactivateDeliveryRollers();
                }
                break;
            case 7:
                follower.followPath(straffLeft);
                setPathState(8);
                break;
        }
    }

    private void prePositionCarouselForDelivery(int aprilTagID) {
        if (aprilTagID == 21)
            carousselIndexGreenUp();
        else
            carousselIndexPurpleLeftUp();

        activateDeliveryRollers();
    }

    private void scoreArtifacts(int aprilTagID, boolean reducedVelocity) throws InterruptedException {
        actionTimer.resetTimer();
        if (reducedVelocity)
            shooterWheelsVelocity = 630;
        else
            shooterWheelsVelocity = 700;

        synchronized (actionTimer) {
            switch (aprilTagID) {
                case 21:
                    activateDeliveryRollers();
                    activateCarouselOuttake();
                    actionTimer.wait(2000);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 720;
                    activateDeliveryRollers();
                    carousselIndexPurpleLeftUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 690;
                    activateDeliveryRollers();
                    carousselIndexPurpleRightUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    actionTimer.wait(1500);
                    break;
                case 22:
                    activateDeliveryRollers();
                    activateCarouselOuttake();
                    actionTimer.wait(2000);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 720;
                    activateDeliveryRollers();
                    carousselIndexGreenUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 690;
                    activateDeliveryRollers();
                    carousselIndexPurpleRightUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    actionTimer.wait(1500);
                    break;
                case 23:
                    activateDeliveryRollers();
                    activateCarouselOuttake();
                    actionTimer.wait(2000);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 720;
                    activateDeliveryRollers();
                    carousselIndexPurpleRightUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    shooterWheelsVelocity = 690;
                    activateDeliveryRollers();
                    carousselIndexGreenUp();
                    actionTimer.wait(800);
                    activateCarouselOuttake();
                    actionTimer.wait(1500);
                    deactivateCarouselOuttake();
                    actionTimer.wait(1500);
                    break;
            }
        }
    }

    @Override
    void setStartingPose() {
        follower.setStartingPose(startPose);
    }
}
