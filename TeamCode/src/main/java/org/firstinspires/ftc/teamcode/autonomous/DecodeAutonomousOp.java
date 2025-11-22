package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Decode Autonomous Advanced", group = "Decode2025")
public class DecodeAutonomousOp extends OpMode {

    Follower follower;
    Timer pathTimer, actionTimer, opmodeTimer;
    int pathState;

    Limelight3A camera;

    DcMotor intakeFeedRoller, artifactShooterLeft, artifactShooterRight;

    Servo indexingCaroussel, deliveryShooterTilt, groundIntakeRoller, artifactOuttakeSpinner, artifactOuttakeTilt;

    ColorSensor artifactColorDetector;

    BNO055IMU imu;

    Pose startPose = new Pose(60, 8, Math.toRadians(90)); // Start Pose of our robot.

    Servo deliveryPushLeft;
    DistanceSensor intakeArtifactSensor;
    DistanceSensor leftCarouselSensor;
    DistanceSensor rightCarouselSensor;
    DistanceSensor deliveryArtifactSensor;
    Servo deliveryPushRight;

    int purpleLeftArtifact;
    int purpleRightArtifact;
    boolean intakeArtifactDetected;
    boolean intakeDistanceSensorActive;
    int shooterWheelsVelocity;
    boolean deliveryDistanceSensorActive;
    boolean intakeSpinEnabled;
    String colorIdentified;
    double purpleLeftBinPos;
    int greenArtifact;
    int operationMode;
    boolean colorSensorActive;
    double greenBinPos;
    double purpleRightBinPos;
    int aprilTagID;

    void buildPaths() {
    }

    void autonomousPathUpdate() throws InterruptedException {
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("April Tag", aprilTagID);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        camera = hardwareMap.get(Limelight3A.class, "limelight");
        intakeFeedRoller = hardwareMap.get(DcMotor.class, "intakeFeedRoller");
        artifactShooterLeft = hardwareMap.get(DcMotor.class, "artifactShooterLeft");
        artifactShooterRight = hardwareMap.get(DcMotor.class, "artifactShooterRight");
        indexingCaroussel = hardwareMap.get(Servo.class, "indexingCaroussel");
        deliveryShooterTilt = hardwareMap.get(Servo.class, "deliveryShooterTilt");
        groundIntakeRoller = hardwareMap.get(Servo.class, "groundIntakeRoller");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        artifactColorDetector = hardwareMap.get(ColorSensor.class, "artifactColorDetector");
        artifactOuttakeSpinner = hardwareMap.get(Servo.class, "artifactOuttakeSpinner");
        artifactOuttakeTilt = hardwareMap.get(Servo.class, "artifactOuttakeTilt");

        deliveryPushLeft = hardwareMap.get(Servo.class, "deliveryPushLeft");
        deliveryPushRight = hardwareMap.get(Servo.class, "deliveryPushRight");
        intakeArtifactSensor = hardwareMap.get(DistanceSensor.class, "intakeArtifactSensor");
        leftCarouselSensor = hardwareMap.get(DistanceSensor.class, "leftCarouselSensor");
        rightCarouselSensor = hardwareMap.get(DistanceSensor.class, "rightCarouselSensor");
        deliveryArtifactSensor = hardwareMap.get(DistanceSensor.class, "deliveryArtifactSensor");

        initIMU();
        initActuators();
        initCam();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        setStartingPose();
    }

    void setStartingPose() {
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        initPosition();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        camera.stop();
    }

    Pose getRobotPoseFromCamera() {
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }

    int identifyAprilTagID() throws InterruptedException {
        synchronized (actionTimer) {
            LLResult result;
            List<LLResultTypes.FiducialResult> fiducialResults;
            LLResultTypes.FiducialResult fiducialResult;
            int fiducialResultID;

            actionTimer.resetTimer();
            actionTimer.wait(500);

            int motifID = 0;

            result = camera.getLatestResult();
            if (result != null) {
                // Access general information.
                // Access fiducial results.
                fiducialResults = result.getFiducialResults();
                telemetry.addData("fiducialResults", fiducialResults.size());
                telemetry.update();
                for (LLResultTypes.FiducialResult fiducialResult_item : fiducialResults) {
                    fiducialResult = fiducialResult_item;
                    fiducialResultID = fiducialResult.getFiducialId();
                    telemetry.addData("fiducialResultID", fiducialResultID);
                    telemetry.update();
                    if (fiducialResultID == 21 || fiducialResultID == 22 || fiducialResultID == 23) {
                        motifID = fiducialResultID;
                    }
                }
                telemetry.addData("Motif ID identified", motifID);
                camera.stop();
            } else {
                telemetry.addData("Limelight", "No data available");
            }
            telemetry.update();
            return motifID;
        }
    }

    void initActuators() {
        intakeFeedRoller.setDirection(DcMotor.Direction.REVERSE);
        intakeFeedRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeFeedRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        artifactShooterLeft.setDirection(DcMotor.Direction.FORWARD);
        artifactShooterRight.setDirection(DcMotor.Direction.REVERSE);
        artifactShooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        artifactShooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        artifactShooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        artifactShooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexingCaroussel.scaleRange(0.1, 1);
        deliveryShooterTilt.scaleRange(0, 0.9);
        groundIntakeRoller.setDirection(Servo.Direction.REVERSE);
        deliveryPushLeft.setDirection(Servo.Direction.REVERSE);
    }

    private void initCam() {
        telemetry.setMsTransmissionInterval(11);
        camera.pipelineSwitch(0);
        // Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
        camera.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    private void initIMU() {
        BNO055IMU.Parameters imuParameters;

        // Create a new IMU parameters object
        imuParameters = new BNO055IMU.Parameters();
        // Set the IMU mode to IMU so it automatically calibrates itself
        imuParameters.mode = BNO055IMU.SensorMode.IMU;
        // Use degrees as our angle unit
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Use meters per second as unit of acceleration
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Warn driver that this may take several seconds
        telemetry.addData("Status", "Initializing IMU.. Please wait..");
        telemetry.update();
        // Initialize IMU with these parameters
        imu.initialize(imuParameters);
        // Tell driver that IMU init is done
        telemetry.addData("Status", "IMU initialized!");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void initPosition() {
        artifactOuttakeTilt.setPosition(0.55);
        greenArtifact = 0;
        purpleLeftArtifact = 0;
        purpleRightArtifact = 0;
        carousselIndexGreenUp();
        operationMode = 1;
    }

    /**
     * Describe this function...
     */
    void activateIntakeSystem() {
        groundIntakeRoller.setPosition(1);
        ((DcMotorEx) intakeFeedRoller).setVelocity(1000);
        greenArtifact = 0;
        purpleLeftArtifact = 0;
        purpleRightArtifact = 0;
        spinCarouselForNextIntake();
        colorSensorActive = true;
    }

    /**
     * Describe this function...
     */
    void deactivateIntakeSystem() {
        ((DcMotorEx) intakeFeedRoller).setVelocity(0);
        groundIntakeRoller.setPosition(0.5);
        colorSensorActive = false;
        intakeDistanceSensorActive = false;
    }

    /**
     * Describe this function...
     */
    void revertIntake() {
        groundIntakeRoller.setPosition(0);
        ((DcMotorEx) intakeFeedRoller).setVelocity(-1000);
    }

    /**
     * Describe this function...
     */
    void tiltDeliveryShooterToMiddle() {
        deliveryShooterTilt.setPosition(0.5);
    }

    /**
     * Describe this function...
     */
    void activateCarouselOuttake() {
        intakeDistanceSensorActive = false;
        artifactOuttakeSpinner.setPosition(0);
        artifactOuttakeTilt.setPosition(0.46);
    }

    /**
     * Describe this function...
     */
    void activateDeliveryRollers() {
        ((DcMotorEx) artifactShooterLeft).setVelocity(shooterWheelsVelocity);
        ((DcMotorEx) artifactShooterRight).setVelocity(shooterWheelsVelocity);
        deliveryPushLeft.setPosition(1);
        deliveryPushRight.setPosition(1);
    }

    /**
     * Describe this function...
     */
    void deactivateCarouselOuttake() {
        deliveryDistanceSensorActive = false;
        artifactOuttakeSpinner.setPosition(0.5);
        artifactOuttakeTilt.setPosition(0.55);
    }

    /**
     * Describe this function...
     */
    void initConstants() {
        boolean enableAutomatedControls;
        boolean robotDistanceSensorsEnabled;
        enableAutomatedControls = true;
        robotDistanceSensorsEnabled = false;
    }

    /**
     * Describe this function...
     */
    void carousselIndexGreenUp() {
        intakeSpinEnabled = false;
        indexingCaroussel.setPosition(0.505);
        greenBinPos = 1;
        purpleLeftBinPos = 2.5;
        purpleRightBinPos = 2;
    }

    /**
     * Describe this function...
     */
    void deactivateDeliveryRollers() {
        ((DcMotorEx) artifactShooterLeft).setVelocity(0);
        ((DcMotorEx) artifactShooterRight).setVelocity(0);
        deliveryPushLeft.setPosition(0.5);
        deliveryPushRight.setPosition(0.5);
    }

    /**
     * Describe this function...
     */
    void carousselIndexPurpleLeftUp() {
        intakeSpinEnabled = false;
        indexingCaroussel.setPosition(0.37);
        purpleLeftBinPos = 1;
        greenBinPos = 2;
        purpleRightBinPos = 2.5;
    }

    /**
     * Describe this function...
     */
    void carousselIndexPurpleRightUp() {
        intakeSpinEnabled = false;
        indexingCaroussel.setPosition(0.62);
        purpleRightBinPos = 1;
        greenBinPos = 2.5;
        purpleLeftBinPos = 2;
    }

    /**
     * Describe this function...
     */
    void carouselIndexGreenDown() {
        intakeSpinEnabled = false;
        greenArtifact = 0;
        intakeDistanceSensorActive = false;
        intakeArtifactDetected = false;
        indexingCaroussel.setPosition(0.31);
        greenBinPos = 0;
        purpleLeftBinPos = 0.5;
        purpleRightBinPos = 1.5;
    }

    /**
     * Describe this function...
     */
    void carousselIndexPurpleLeftDown() {
        intakeSpinEnabled = false;
        purpleLeftArtifact = 0;
        intakeDistanceSensorActive = false;
        intakeArtifactDetected = false;
        indexingCaroussel.setPosition(0.19);
        purpleLeftBinPos = 0;
        greenBinPos = 1.5;
        purpleRightBinPos = 0.5;
    }

    /**
     * Describe this function...
     */
    void carousselIndexPurpleRightDown() {
        intakeSpinEnabled = false;
        purpleRightArtifact = 0;
        intakeDistanceSensorActive = false;
        intakeArtifactDetected = false;
        indexingCaroussel.setPosition(0.44);
        purpleRightBinPos = 0;
        greenBinPos = 0.5;
        purpleLeftBinPos = 1.5;
    }

    /**
     * Describe this function...
     */
    String detectArtifactColor() {
        int alphaValue;
        int redValue;
        int greenValue;
        int blueValue;
        String colorIdentified = "Unknown";

        alphaValue = artifactColorDetector.alpha();
        redValue = artifactColorDetector.red() / alphaValue;
        greenValue = artifactColorDetector.green() / alphaValue;
        blueValue = artifactColorDetector.blue() / alphaValue;
        if (redValue >= 0.75 && redValue <= 0.8 && greenValue >= 1.25 && greenValue <= 1.35 && blueValue >= 0.9 && blueValue <= 1) {
            colorIdentified = "Unknown";
        } else if (redValue >= 0.79 && redValue <= 0.81 && greenValue >= 1.05 && greenValue <= 1.25 && blueValue >= 0.95 && blueValue <= 1.1) {
            colorIdentified = "Purple";
        } else if (redValue >= 0.6 && redValue <= 0.74 && greenValue >= 1.2 && greenValue <= 1.4 && blueValue >= 0.9 && blueValue <= 1.1) {
            colorIdentified = "Green";
        }
        return colorIdentified;
    }

    /**
     * Describe this function...
     */
    void spinCarouselForNextDelivery() {
        if (greenArtifact == 1) {
            carousselIndexGreenUp();
        } else if (purpleLeftArtifact == 1) {
            carousselIndexPurpleLeftUp();
        } else if (purpleRightArtifact == 1) {
            carousselIndexPurpleRightUp();
        } else {
            operationMode = 1;
            deactivateDeliveryRollers();
            activateIntakeSystem();
            carouselIndexGreenDown();
        }
    }

    /**
     * Describe this function...
     */
    void spinCarouselForNextIntake() {
        intakeDistanceSensorActive = false;
        if (greenArtifact == 0) {
            carouselIndexGreenDown();
        } else if (purpleLeftArtifact == 0) {
            carousselIndexPurpleLeftDown();
        } else if (purpleRightArtifact == 0) {
            carousselIndexPurpleRightDown();
        }
    }

    /**
     * Describe this function...
     */
    void carousselIndexGreenDown() {
        intakeSpinEnabled = false;
        greenArtifact = 0;
        intakeDistanceSensorActive = false;
        intakeArtifactDetected = false;
        indexingCaroussel.setPosition(0.31);
        greenBinPos = 0;
        purpleLeftBinPos = 0.5;
        purpleRightBinPos = 1.5;
    }
}
