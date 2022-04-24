package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.core.Mat;
import org.opencv.aruco.Dictionary;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.List;
import  java.util.Arrays;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        moveToWrapper(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
        moveToAr(10.71000, -7.70000, 4.48000,0, 0.707, 0, 0.707);
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

    private void moveToAr(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        int i = 0;

        float[] eu = new float[]{};

        do {
            Log.i("AR[count]:", String.valueOf(i));
            api.moveTo(point, quaternion, true);
            eu = DetectAR(api.getMatNavCam());
            ++i;
        } while (i < 5 && eu == null);

        api.laserControl(true);
        api.laserControl(false);
        api.takeTarget1Snapshot();
        api.saveMatImage(api.getMatNavCam(),"target.png");
        return;
    }



    float[] DetectAR(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat ids = new Mat();
        List<Mat> rejectedCondinates = new ArrayList<>();
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        DetectorParameters parameter = DetectorParameters.create();
        double[][] camIntrinsics = api.getNavCamIntrinsics();
        Mat camMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        camMat.put(0,0,camIntrinsics[0]);
        distCoeffs.put(0,0,camIntrinsics[1]);

        Mat rvecs = new Mat();
        Mat tvecs = new Mat();

        try {
            Aruco.detectMarkers(matImage, dictionary, corners, ids, parameter);
            List<Integer> values = flatten(ids);
            //for(int i = 0;i<values.size();i++) {
            Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMat, distCoeffs, rvecs, tvecs);
            //}

            List<float[]> arucos = new ArrayList<>();


            for(int i = 0; i < ids.size(); i++)
            {
                Mat rotationMatrix = new Mat();
                Mat r = new Mat();
                Mat q = new Mat();
                Calib3d.Rodrigues(rvecs.row(i), rotationMatrix);
                double[] eulerAngles = Calib3d.RQDecomp3x3(rotationMatrix, r, q);

                float pitch = (float) eulerAngles[0];
                float yaw = (float) eulerAngles[1];
                float roll = (float) eulerAngles[2];

                arucos.add(new float[]{values.get(i),yaw,pitch,roll});
            }

            float[] eu = new float[]{};

            /*
            for(int i=0;i<arucos.size();i++){
                float[] aruco = arucos.get(i);
                Log.i("AR[status]:",Arrays.toString(aruco));
                if(aruco[0] == 4){
                    eu = new float[]{aruco[1],aruco[2],aruco[3]};
                }
            }*/


            return eu;
        } catch (Exception e) {
           Log.i("AR[status]:", " Not detected");
        }
        Log.i("AR[status]:", " end");
        return null;
    }

    List<Integer> flatten(Mat src) {
        List<Integer> values = new ArrayList<>();
        for (int i = 0; i < src.rows(); i++) {
            for (int j = 0; j < src.cols(); j++) {
                values.add((int)src.get(i,j)[0]);
            }
        }
        return values;
    }

    public void moveToEuler(double pos_x, double pos_y, double pos_z, double eu_x, double eu_y, double eu_z) {
        double yaw=Math.toRadians(eu_x);
        double pitch=Math.toRadians(eu_y);
        double roll=Math.toRadians(eu_z);

        double cosYaw = Math.cos(yaw / 2);
        double cosPitch = Math.cos(pitch / 2);
        double cosRoll = Math.cos(roll/2);

        double sinYaw = Math.sin(yaw / 2);
        double sinPitch = Math.sin(pitch / 2);
        double sinRoll = Math.sin(roll/2);


        double qua_x sinYaw * sinPitch * cosRoll - cosYaw * cosPitch * sinRoll;
        double qua_y = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;
        double qua_z = cosYaw * sinPitch * cosRoll - cosYaw * sinPitch * sinRoll;
        double qua_w = cosYaw * cosPitch * cosRoll - sinYaw * sinPitch * sinRoll;

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

}

