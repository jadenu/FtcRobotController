package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.internal.OptimizedController;
import org.firstinspires.ftc.internal.OptimizedRobot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.examples.ExampleControllerMapping;
import org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="Odo/Vuforia combo")
public class TeleopOdoPlusVuforiaLocation extends OpMode {

    OptimizedRobot robot;
    OptimizedController controller1, controller2;
    Telemetry.Log log;

    private static final String VUFORIA_KEY = "AS2jf2L/////AAABmXgZ8WffLklTjSGx4BHLMEyIUzz/MDvofYi6H+Wn9qY4aJN5vPfPNmDqPOZkFax2dve7+6w6JCk+CoHc0hAq8UueKVMgQ4ma5xOB0mrS8RdkV330C2uom99XO9gX6Pbvy/CZHU3Is0BXnZJZ0ZhG8vqrJ+c0KFl1uiFpLLE5KeT82u4FA9cO5ZSR9iWyVGHkFYlXpd/nuKdWVf25KGhElSSBDIhBTQV6ykNZqySS8sqdWg+aG7uqdbEYV40ac+iL4Fm6OyyotS8CwdY9TvYsemRlMHsK1OY3ngIWZ9rpQ9dx/oPDDtezc0WhpfpBsoxtta78pJNLRofHhulw3sIAGgN5bwAwNrWUzMzEYhw15Y7R";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;

    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    org.firstinspires.ftc.teamcode.internal.roadrunner.drive.SampleMecanumDrive drive;

    private boolean targetVisible       = false;

    List<VuforiaTrackable> allTrackables;

    @Override
    public void init() {
        controller1 = new OptimizedController(gamepad1);
        controller2 = new OptimizedController(gamepad2);

        log = telemetry.log();

        robot = new OptimizedRobot(controller1, controller2, telemetry, hardwareMap, new ExampleControllerMapping());

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targets);

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 6.5f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 7.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 6.3f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);

        targets.activate();
    }

    @Override
    public void loop() {

        drive.update();

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null)
                    lastLocation = robotLocationTransform;
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            drive.setPoseEstimate(new Pose2d(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, Math.toRadians(rotation.thirdAngle)));
            drive.updatePoseEstimate();
            telemetry.addData("Estimate", "updated");
        }
        else
            telemetry.addData("Visible Target", "none");
        telemetry.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();
/*
// Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
        */
        robot.updateDrive(controller1, controller2, true, false, 1d, OptimizedRobot.RobotDirection.BACK, OptimizedRobot.RobotDirection.BACK, false);

    }

    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }
}
