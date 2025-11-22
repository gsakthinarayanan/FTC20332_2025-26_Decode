package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Blue Near Side Autonomous", group = "Decode2025")
public class BlueNearAutonomousOp extends DecodeAutonomousOp {
    Pose startPose = new Pose(20, 124, Math.toRadians(45)); // Start Pose of our robot.
    Pose motifPose = new Pose(62, 84,  Math.toRadians(90));
    Pose scorePose = new Pose(64, 108,  Math.toRadians(134));
    Pose straffRightPose = new Pose(64, 115,  Math.toRadians(134));
    Pose navigateToPickup1Pose = new Pose(60, 94,  Math.toRadians(180));
    Pose grabPickup1Pose = new Pose(42, 94,  Math.toRadians(180));
    Pose grabPickup2Pose = new Pose(38, 94,  Math.toRadians(180));
    Pose grabPickup3Pose = new Pose(32, 94,  Math.toRadians(180));


    private Path navigateToMotif, scorePreloaded, straffRight;
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

        straffRight = new Path(new BezierLine(scorePose, straffRightPose));
        straffRight.setLinearHeadingInterpolation(scorePose.getHeading(), straffRightPose.getHeading());
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
                    if (follower.getPose().getHeading() > Math.toRadians(130)) {
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
                    follower.followPath(grabPickup1,0.4, true);
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
                    follower.followPath(scorePreloadedArtifacts, true);
                    prePositionCarouselForDelivery(aprilTagID);
                    if (follower.getPose().getHeading() < Math.toRadians(138)) {
                        scoreArtifacts(aprilTagID, false);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                follower.followPath(straffRight);
                setPathState(8);
                break;
        }
    }

    private void prePositionCarouselForDelivery(int aprilTagID) throws InterruptedException {
        actionTimer.resetTimer();
        synchronized (actionTimer) {
            actionTimer.wait(500);
        }

        if (aprilTagID == 21)
            carousselIndexGreenUp();
        else
            carousselIndexPurpleLeftUp();

        activateDeliveryRollers();
    }

    private void scoreArtifacts(int aprilTagID, boolean reducedVelocity) throws InterruptedException {
        actionTimer.resetTimer();
        if (reducedVelocity)
            shooterWheelsVelocity = 650;
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
