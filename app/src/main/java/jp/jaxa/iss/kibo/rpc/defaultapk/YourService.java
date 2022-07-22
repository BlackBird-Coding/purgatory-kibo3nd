package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        api.startMission();
        moveToWrapper(10.71000, -7.70000, 4.53000,0, 0.707, 0, 0.707);
        api.reportPoint1Arrival();

        moveToTarget1(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);

        moveToWrapper(11.1,-9.5,4.55,0,0,-0.707, 0.707);
        moveToWrapper(11.27460, -9.7, 5.29881,0, 0, -0.707, 0.707);

        //x-0.07 z+0.17
        moveToTarget2(11.27460, -9.92284, 5.29881,0, 0, -0.707, 0.707);

        moveToWrapper(11.27460,-9.5000,4.55,0,0,-0.707, 0.707);
        moveToWrapper(11.27460,-8.3000,4.55,0,0,-0.707, 0.707);

        moveToWrapper(11.27460, -7.89178, 4.96538,0, 0, -0.707, 0.707);

        api.reportMissionCompletion();
    }

    private void moveToWrapper(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                               double qua_w) {
        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        int i = 0;

        Result result;
        do {
            result = api.moveTo(point, quaternion, true);
            ++i;
        } while (!result.hasSucceeded() && i < LOOP_MAX);
    }

    private  void  moveToTarget1(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                                 double qua_w){
        moveToWrapper(pos_x,pos_y, pos_z, qua_x,qua_y,qua_z, qua_w);
        api.flashlightControlFront(0.025f);
        api.relativeMoveTo(new Point(0, -0.09, 0),new Quaternion(0, (float)0.707, 0, (float)0.707),true);
        wait(1000);
        api.laserControl(true);
        api.flashlightControlFront(0f);
        api.takeTarget1Snapshot();
        api.laserControl(false);
    }

    private  void  moveToTarget2(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                             double qua_w){
        double[] result;
        moveToWrapper(pos_x,pos_y, pos_z, qua_x,qua_y,qua_z, qua_w);
        wait(1000);
        api.flashlightControlFront(0.025f);
        result = DetectAR2(api.getMatNavCam());
        moveToWrapper(pos_x + result[0], pos_y, pos_z + result[1], qua_x, qua_y, qua_z, qua_w);
        wait(1000);
        api.flashlightControlFront(0f);
        api.laserControl(true);
        api.takeTarget2Snapshot();
        api.laserControl(false);
    }


    private double[] DetectAR2(Mat matImage){
        Mat image = new Mat();
        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Imgproc.undistort(matImage,image,camMat,distCoeffs);

        Imgproc.medianBlur(image, image,3);

        Imgproc.adaptiveThreshold(image, image, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 23, 80);
        Mat circles = new Mat();
        Imgproc.HoughCircles(image, circles, Imgproc.HOUGH_GRADIENT, 1.0,100.0, 100.0, 30.0, 30, 55);

        org.opencv.core.Point center = new org.opencv.core.Point();

        double[] c = circles.get(0, 0);
        center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));
        double newX = -((735 - center.x)/950);
        double newZ = -((460 - center.y)/775);
        return new double[]{newX,newZ};
    }

    private void wait(int ms){
        try
        {
            Thread.sleep(ms);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }
}

