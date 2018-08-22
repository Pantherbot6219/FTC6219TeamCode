package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;

public class VuMark {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    LinearOpMode opMode;

    public VuMark(LinearOpMode opMode) {
        this.opMode = opMode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
    }

    public void activate (){
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AUgbT0j/////AAAAGTj0axLoDUvmo1PWo4YoBxkeKIG99PQSTrgFoQVymuq7TMc3OGQozLZo6d+dLt0EiQwR87XKW4aXt/7BZr95khAoylxKP02Vh5PmZbp18YDa5g9gWiYUGUvHqLUbNgMKotFhGgE9noRvWbiP2RxgOCy+HoT39NaFXiLiH69cLGbCzpz1tzuvRPce/EVkWBBomcS2yC/hl1hTlBjvzTHN1lKMv59s9gYhC69DNODQeNg2JMOv3ggMlRDTjOpbNZUZAQkHfqS/2w0W8d0+krzVBD129juhL2r6u4mWVhXFq2FOZoUTezbkaFTxKCRadl3v5ot5aPmuEU4mSFtmrw15J6R9XtRAg/U8/I1k7zpRpVTg";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vuMark = RelicRecoveryVuMark.UNKNOWN;
        //VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //VuforiaTrackable relicTemplate = relicTrackables.get(0);
        //relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        //relicTrackables.activate();
    }

    public void decodePictograph() throws InterruptedException {
        int count = 0;
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && count < 4000000) {
            count++;
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN){
                break;
            }
            else{

            }
            //RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
    }
    public RelicRecoveryVuMark getCryptoboxKey() {
        return(vuMark);
    }

    public void deactivate(){
        relicTrackables.deactivate();
    }
}