package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import gov.nasa.arc.astrobee.Kinematics;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import purgatory.kibo.Calculate;

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        api.startMission();
        moveToWrapper(10.71000, -7.70000, 4.53000,0, 0.707, 0, 0.707);
        api.reportPoint1Arrival();

        moveToTarget1(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);

        moveToWrapper(11.3,-9.5000,4.4,0,0,-0.707, 0.707);
        moveToWrapper(11.27460, -9.7, 5.29881,0, 0, -0.707, 0.707);

        //x-0.07 z+0.17
        moveToTarget2(11.27460, -9.92284, 5.29881,0, 0, -0.707, 0.707);

        moveToWrapper(11.3,-9.5000,4.4,0,0,-0.707, 0.707);
        moveToWrapper(11.3,-8.0000,4.4,0,0,-0.707, 0.707);

        moveToWrapper(11.27460, -7.89178, 4.96538,0, 0, -0.707, 0.707);

        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
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
        double[] result;
        moveToWrapper(pos_x,pos_y, pos_z, qua_x,qua_y,qua_z, qua_w);
        wait(1000);
        api.flashlightControlFront(0.025f);
        result = DetectAR1(api.getMatNavCam());
        moveToWrapper(pos_x+result[0],pos_y+result[1], pos_z, qua_x,qua_y,qua_z, qua_w);
        wait(1000);
        api.laserControl(true);
        api.flashlightControlFront(0f);
        api.takeTarget1Snapshot();
        //api.saveMatImage(api.getMatNavCam(),"target1.png");
        api.laserControl(false);
        return;
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
        api.saveMatImage(api.getMatNavCam(),"target2.png");
        api.laserControl(false);
        return;
    }


    private double[] DetectAR1(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat _ids = new Mat();
        List<Mat> rejectedCondinates = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_adaptiveThreshWinSizeMin(5);
        parameters.set_adaptiveThreshWinSizeMax(10);
        parameters.set_minMarkerPerimeterRate(0.05d);
        parameters.set_maxMarkerPerimeterRate(5.0d);
        parameters.set_errorCorrectionRate(0.03d);

        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        Mat image = new Mat();

        Imgproc.undistort(matImage,image,camMat,distCoeffs);


        try {
            Aruco.detectMarkers(image, dictionary, corners, _ids, parameters);
            List<Integer> ids = new Calculate().flatten(_ids);

            int centerX = 0;
            int centerY = 0;

            for (int i = 0; i < 4; i++) {
                for(int j=0; j<4; j++) {
                    centerX += corners.get(i).get(0, j)[0];
                    centerY += corners.get(i).get(0, j)[1];
                    Imgproc.circle(image ,new org.opencv.core.Point(corners.get(i).get(0, j)[0],corners.get(i).get(0, j)[1]), 1, new Scalar(200, 100, 100), 3, 8, 0);
                }
            }

            org.opencv.core.Point center = new org.opencv.core.Point(centerX /16,centerY/16);
            Imgproc.circle(image ,center, 1, new Scalar(0, 100, 100), 3, 8, 0);

            double width = image.size().width;
            double height = image.size().height;
            Imgproc.circle(image , new org.opencv.core.Point(width/2,height/2), 2, new Scalar(0, 100, 100), 3, 8, 0);

            //api.saveMatImage(image,"debug1.png");

            double newY = ((width/2 - centerX/16)/800)-0.031;
            double newX = ((height/2 - centerY/16)/800)-0.05;

           /*  Log.i("cX",String.valueOf(centerX/16));
            Log.i("cY",String.valueOf(centerY/16));
            Log.i("newX", String.valueOf(newX));
            Log.i("newY", String.valueOf(newY));*/

            Log.i("AR[status]:", " end");
            return new double[]{newX,newY};
        } catch (Exception e) {
            Log.i("AR[status]:", " Not detected");
        }
        return null;
    }


    private double[] DetectAR2O(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat _ids = new Mat();
        List<Mat> rejectedCondinates = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_adaptiveThreshWinSizeMin(5);
        parameters.set_adaptiveThreshWinSizeMax(10);
        parameters.set_minMarkerPerimeterRate(0.05d);
        parameters.set_maxMarkerPerimeterRate(5.0d);
        parameters.set_errorCorrectionRate(0.03d);

        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        Mat image = new Mat();

        Imgproc.undistort(matImage,image,camMat,distCoeffs);


        try {
            Aruco.detectMarkers(image, dictionary, corners, _ids, parameters);
            List<Integer> ids = new Calculate().flatten(_ids);

            int centerX = 0;
            int centerY = 0;

            for (int i = 0; i < 4; i++) {
                for(int j=0; j<4; j++) {
                    centerX += corners.get(i).get(0, j)[0];
                    centerY += corners.get(i).get(0, j)[1];
                    Imgproc.circle(image ,new org.opencv.core.Point(corners.get(i).get(0, j)[0],corners.get(i).get(0, j)[1]), 1, new Scalar(200, 100, 100), 3, 8, 0);
                }
            }

            org.opencv.core.Point center = new org.opencv.core.Point(centerX /16,centerY/16);
            Imgproc.circle(image ,center, 1, new Scalar(0, 100, 100), 3, 8, 0);

            double width = image.size().width;
            double height = image.size().height;
            Imgproc.circle(image , new org.opencv.core.Point(width/2,height/2), 2, new Scalar(0, 100, 100), 3, 8, 0);

            api.saveMatImage(image,"debug2.png");

            double newX = ((width/2 - centerX/16)/1000)-0.055;
            double newZ = (-(height/2 - centerY/16)/1000)+0.045;

            Log.i("cX",String.valueOf(centerX/16));
            Log.i("cY",String.valueOf(centerY/16));
            Log.i("newX", String.valueOf(newX));
            Log.i("newZ", String.valueOf(newZ));

            Log.i("AR[status]:", " end");
            return new double[]{newX,newZ};
        } catch (Exception e) {
            Log.i("AR[status]:", " Not detected");
        }
        return null;
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
        //api.saveMatImage(image,"th.png");
        Mat circles = new Mat();
        Imgproc.HoughCircles(image, circles, Imgproc.HOUGH_GRADIENT, 1.0,100.0, 100.0, 30.0, 30, 55);
        Log.i("Circle col", String.valueOf(circles.cols()));

        org.opencv.core.Point center = new org.opencv.core.Point();

            double[] c = circles.get(0, 0);
            center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(image, center, 1, new Scalar(0, 200, 100), 3, 8, 0);
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(image, center, radius, new Scalar(80, 0, 255), 3, 8, 0);
            Log.i("Circle x", String.valueOf(center.x));
            Log.i("Circle y", String.valueOf(center.y));
        api.saveMatImage(image,"debug2.png");
        double newX = -((735 - center.x)/950);
        double newZ = -((460 - center.y)/775);
        Log.i("new X",String.valueOf(newX));
        Log.i("new Z",String.valueOf(newZ));
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

