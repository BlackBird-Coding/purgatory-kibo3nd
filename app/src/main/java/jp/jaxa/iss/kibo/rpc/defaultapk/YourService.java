package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.os.SystemClock;
import android.util.Log;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import purgatory.kibo.Calculate;

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        Log.i("Bla_ckB","start");
        moveToEuler(10.71000, -7.70000, 4.48000,0,90,0);
        api.saveMatImage(api.getMatNavCam(),"up.png");
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

        double[] eu = new double[]{};

        do {
            Log.i("AR[count]:", String.valueOf(i));
            api.moveTo(point, quaternion, true);
            eu = DetectAR(api.getMatNavCam());
            moveToEuler(pos_x,pos_y,pos_z,eu[3],eu[2]-90,eu[1]);
            ++i;
        } while (i < 5 && eu == null);

        api.laserControl(true);
        api.laserControl(false);
        api.takeTarget1Snapshot();
        api.saveMatImage(api.getMatNavCam(),"target.png");
        return;
    }



    private double[] DetectAR(Mat matImage) {
        Log.i("AR[status]:", " start");
        long startTime = SystemClock.elapsedRealtime();

        Mat _ids = new Mat();
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
            Aruco.detectMarkers(matImage, dictionary, corners, _ids, parameter);
            List<Integer> ids = new Calculate().flatten(_ids);
            //for(int i = 0;i<values.size();i++) {
            Aruco.estimatePoseSingleMarkers(corners, 0.05f, camMat, distCoeffs, rvecs, tvecs);
            //}

            List<double[]> arucos = new ArrayList<>();


            for(int i = 0; i < ids.size(); i++)
            {
                Mat rotationMatrix = new Mat();
                Mat r = new Mat();
                Mat q = new Mat();
                Calib3d.Rodrigues(rvecs.row(i), rotationMatrix);
                double[] eulerAngles = Calib3d.RQDecomp3x3(rotationMatrix, r, q);

                //Log.i("AR[EU RAW]:", "X"+eulerAngles[0]+"Y"+(-eulerAngles[1])+"Z"+(-eulerAngles[2]));

                arucos.add(new double[]{ids.get(i),eulerAngles[0],-eulerAngles[1],-eulerAngles[2]});
            }

            double[] eu = new double[]{};

            /*
            eu[1] = (arucos.get(0)[1] + arucos.get(1)[1]+ arucos.get(2)[1] +arucos.get(3)[1]) / 4;
            eu[2] = (arucos.get(0)[2] + arucos.get(1)[2]+ arucos.get(2)[2] +arucos.get(3)[2]) / 4;
            eu[3] = (arucos.get(0)[3] + arucos.get(1)[3]+ arucos.get(2)[3] +arucos.get(3)[3]) / 4;
            */
            eu = arucos.get(0);

            return eu;
        } catch (Exception e) {
           Log.i("AR[status]:", " Not detected");
        }
        Log.i("AR[status]:", " end");
        return null;
    }

    private void moveToEuler(double pos_x, double pos_y, double pos_z, double eu_x, double eu_y, double eu_z) {
        double roll=Math.toRadians(eu_x);
        double pitch=Math.toRadians(eu_y);
        double yaw=Math.toRadians(eu_z);

        double cosYaw = Math.cos(yaw / 2);
        double cosPitch = Math.cos(pitch / 2);
        double cosRoll = Math.cos(roll/2);

        double sinYaw = Math.sin(yaw / 2);
        double sinPitch = Math.sin(pitch / 2);
        double sinRoll = Math.sin(roll/2);

        /*
        double qua_x = sinYaw * sinPitch * cosRoll + cosYaw * cosPitch * sinRoll;
        double qua_y = sinYaw * cosPitch * cosRoll + cosYaw * sinPitch * sinRoll;
        double qua_z = cosYaw * sinPitch * cosRoll - sinYaw * cosPitch * sinRoll;
        double qua_w = cosYaw * cosPitch * cosRoll - sinYaw * sinPitch * sinRoll;
        */
        double qua_x = sinRoll * cosPitch * cosYaw + cosRoll * sinPitch * sinYaw;
        double qua_y = cosRoll * sinPitch * cosYaw - sinRoll * cosPitch * sinYaw;
        double qua_z = cosRoll * cosPitch * sinYaw + sinRoll * sinPitch * cosYaw;
        double qua_w = cosRoll * cosPitch * cosYaw - sinRoll * sinPitch * sinYaw;

        Log.i("AR[QUA]:", "X"+String.valueOf(eu_x)+"Y"+String.valueOf(eu_y)+"Z"+String.valueOf(eu_z));
        Log.i("AR[QUA]:", "X"+String.valueOf(qua_x)+"Y"+String.valueOf(qua_y)+"Z"+String.valueOf(qua_z)+"W"+String.valueOf(qua_w));

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

