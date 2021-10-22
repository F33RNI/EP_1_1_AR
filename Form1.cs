/*
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 *  * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * For more information, please refer to <https://unlicense.org>
 */

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace EP_1_1_AR
{
    public partial class Form1 : Form
    {
        // Plase your camera calibration here
        private static readonly float[] distCoeffsArr = { 0.25767844915390015F, -1.3006402254104614F, -0.004285777453333139F, -0.002507657278329134F, 2.307018518447876F };
        private static readonly float[,] cameraMatrixArr = { { 1019.0990600585938F, 0.0F, 655.7727661132812F }, { 0.0F, 1011.92724609375F, 381.6077880859375F }, { 0.0F, 0.0F, 1.0F } };

        // OpenCV variables
        private volatile VideoCapture capture;
        private Mat frame, frameCopy, rvecs, tvecs, cameraMatrix, distCoeffs;
        private Dictionary dictionary;
        private DetectorParameters detectorParameters;
        private volatile bool opencvRunning = false;

        // List of points (from CSV file)
        private List<double[]> pointsToDraw;


        public Form1()
        {
            InitializeComponent();
        }


        /// <summary>
        /// Opens VideoCapture camera object by pressing the Start button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            // Create cumera object
            capture = new VideoCapture(Convert.ToInt32(this.numericUpDown1.Value));

            // Start cumera
            capture.Start();

            // Show error message if cumera is not opened
            if (!capture.IsOpened)
            {
                MessageBox.Show("Error opening VideoCapture!");
                return;
            }

            // Initilize objects
            frame = new Mat();
            frameCopy = new Mat();
            rvecs = new Mat();
            tvecs = new Mat();
            dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_50);
            detectorParameters = DetectorParameters.GetDefault();

            // Load cumera matrix and distrotion from arrays
            loadCameraCalibration();

            // Set OpenCV loop flag
            opencvRunning = true;

            // Start OpenCV loop
            Thread opencvThread = new Thread(opencvLoop);
            opencvThread.Name = "OpenCV loop";
            opencvThread.Priority = ThreadPriority.AboveNormal;
            opencvThread.Start();

            // Disable Start button and enable Stop button
            this.button1.Enabled = false;
            this.button2.Enabled = true;
        }


        /// <summary>
        /// Closes VideoCapture camera object by pressing the Stop button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button2_Click(object sender, EventArgs e)
        {
            // Clear OpenCV loop flag
            opencvRunning = false;

            // Do nothing if capture is null
            if (capture == null)
                return;

            // Disable Stop button and enable Start button
            this.button1.Enabled = true;
            this.button2.Enabled = false;
        }

        /// <summary>
        /// Reads CSV file by pressing the button
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button3_Click(object sender, EventArgs e)
        {
            try
            {
                // Parse CSV file into List object
                pointsToDraw = parseCSV(this.textBox1.Text);

                // Check if there is parsed data
                if (pointsToDraw.Count > 0)
                    Console.WriteLine("CSV File has been read");
                else
                {
                    pointsToDraw = null;
                    MessageBox.Show("Error! Empty CSV File!");
                }

            }
            catch (Exception ex)
            {
                pointsToDraw = null;
                MessageBox.Show("Error loading CSV File! \n" + ex.Message);
            }
        }

        /// <summary>
        /// Does all the work on a background thread
        /// </summary>
        private void opencvLoop()
        {
            // Unused variabes (for DecomposeProjectionMatrix function)
            Mat _cameraMatrix = new Mat(3, 3, DepthType.Cv32F, 1);
            Mat _rotationMatrix = new Mat(3, 3, DepthType.Cv32F, 1);
            Mat _transVect = new Mat(4, 1, DepthType.Cv32F, 1);
            Mat _rotationMatrixX = new Mat(3, 3, DepthType.Cv32F, 1);
            Mat _rotationMatrixY = new Mat(3, 3, DepthType.Cv32F, 1);
            Mat _rotationMatrixZ = new Mat(3, 3, DepthType.Cv32F, 1);

            // Spin in a loop while the opencvRunning flag is set
            while (opencvRunning)
            {
                // Read one frame from cumera
                capture.Retrieve(frame, 0);

                // Check if frame is not empty
                if (!frame.IsEmpty)
                {
                    // Copy frame to another Mat (to avoid errors)
                    frame.CopyTo(frameCopy);

                    using (VectorOfInt ids = new VectorOfInt())
                    using (VectorOfVectorOfPointF corners = new VectorOfVectorOfPointF())
                    using (VectorOfVectorOfPointF rejected = new VectorOfVectorOfPointF())
                    {
                        // Detect markers on frame
                        ArucoInvoke.DetectMarkers(frameCopy, dictionary, corners, ids, detectorParameters, rejected);

                        // Check if at least one marker is detected
                        if (corners.Length > 0 && ids.Length > 0)
                        {
                            // Circle the found marker in green
                            ArucoInvoke.DrawDetectedMarkers(frameCopy, corners, ids, new MCvScalar(0, 255, 0));

                            // Estimate pose of detected markers
                            ArucoInvoke.EstimatePoseSingleMarkers(corners, 1.0f, cameraMatrix, distCoeffs, rvecs, tvecs);

                            // Find marker ID
                            int markerId = -1;
                            for (int i = 0; i < ids.Size; i++)
                            {
                                if (ids[i] == Convert.ToInt32(this.numericUpDown2.Value))
                                    markerId = i;
                            }

                            // Check if ID found
                            if (markerId >= 0)
                            {
                                using (Mat rvecMat = rvecs.Row(markerId))
                                using (Mat tvecMat = tvecs.Row(markerId))
                                using (VectorOfDouble rvec = new VectorOfDouble())
                                using (VectorOfDouble tvec = new VectorOfDouble())
                                {
                                    // Convert rvec and tvec from Mat to VectorOfDouble
                                    double[] values = new double[3];
                                    tvecMat.CopyTo(values);
                                    tvec.Push(values);
                                    rvecMat.CopyTo(values);
                                    rvec.Push(values);

                                    // Draw axes on target marker
                                    ArucoInvoke.DrawAxis(frameCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.5f);

                                    // Check if there is at least one point to draw
                                    if (pointsToDraw != null && pointsToDraw.Count > 0)
                                    {
                                        // TODO: Find better way to calculate euler angles

                                        // Get 3x3 rotation matrix (rmat) from rotation vector (rvec)
                                        Mat rmat = new Mat(3, 3, DepthType.Cv32F, 1);
                                        CvInvoke.Rodrigues(rvec, rmat);

                                        // Create 3x4 full rotation matrix (with translation vector (tvec))
                                        Mat rotationFull = new Mat(3, 4, DepthType.Cv32F, 1);
                                        for (int row = 0; row < 3; row++)
                                            for (int col = 0; col < 3; col++)
                                                rotationFull.SetValue(row, col, (float)rmat.GetValue(row, col));

                                        // Copy tvec values to the last column
                                        rotationFull.SetValue(0, 3, (float)tvec[0]);
                                        rotationFull.SetValue(1, 3, (float)tvec[1]);
                                        rotationFull.SetValue(2, 3, (float)tvec[2]);

                                        // Create array of euler angles (roll, pitch, yaw)
                                        VectorOfDouble eulerAngles = new VectorOfDouble();

                                        // Find euler angles (roll, pitch, yaw)
                                        CvInvoke.DecomposeProjectionMatrix(rotationFull, _cameraMatrix, _rotationMatrix, _transVect,
                                            _rotationMatrixX, _rotationMatrixY, _rotationMatrixZ, eulerAngles);

                                        // Calculate average maerker size
                                        double markerSize = CvInvoke.ArcLength(corners[0], true) / 4.0;

                                        // Create array of calculated AR points
                                        double[,] pointsAR = new double[pointsToDraw.Count, 2];

                                        // Calculate AR position of the points
                                        for (int pointIndex = 0; pointIndex < pointsToDraw.Count; pointIndex++)
                                        {
                                            // Calculate X, Y, Z of point relatively to markerSize
                                            double pointsToDrawX = pointsToDraw[pointIndex][0] * markerSize;
                                            double pointsToDrawY = pointsToDraw[pointIndex][1] * markerSize;
                                            double pointsToDrawZ = pointsToDraw[pointIndex][2] * markerSize;

                                            // Combine them to one 1D array
                                            double[] pointToDraw = { pointsToDrawX, pointsToDrawY, pointsToDrawZ };

                                            // Rotate point
                                            double[] rotatedPoint = rotate3D(ref pointToDraw,
                                                -(Math.PI / 180) * eulerAngles[0],
                                                -(Math.PI / 180) * eulerAngles[1],
                                                -(Math.PI / 180) * eulerAngles[2]);

                                            // Final coordinates = rotatedPoint + Top Left angle of ArUco marker
                                            pointsAR[pointIndex, 0] = rotatedPoint[0] + (double)corners[0][0].X;
                                            pointsAR[pointIndex, 1] = rotatedPoint[1] + (double)corners[0][0].Y;
                                        }

                                        // Draw calculated AR points
                                        for (int pointIndex = 0; pointIndex < pointsToDraw.Count; pointIndex++)
                                        {
                                            // Create Point object of current coordinates
                                            Point point = new Point((int)pointsAR[pointIndex, 0], (int)pointsAR[pointIndex, 1]);

                                            // Draw line to next point
                                            if (pointIndex < pointsToDraw.Count - 1)
                                                CvInvoke.Line(frameCopy, point,
                                                    new Point((int)pointsAR[pointIndex + 1, 0], (int)pointsAR[pointIndex + 1, 1]),
                                                    new MCvScalar(0, 127, 255), 1);

                                            // Draw point as circle
                                            CvInvoke.Circle(frameCopy, point, 5, new MCvScalar(255, 127, 0), -1);

                                            // Draw point ID as text
                                            CvInvoke.PutText(frameCopy, pointIndex.ToString(),
                                                point, FontFace.HersheySimplex, 1, new MCvScalar(255, 0, 255));
                                        }
                                    }

                                    // If the array of points is empty, display the message
                                    else
                                        CvInvoke.PutText(frameCopy, "No points! Please load CSV file!", 
                                            new Point(50, 50), FontFace.HersheyPlain, 1, new MCvScalar(0, 0, 255));
                                }
                            }
                        }
                    }

                    // Draw current frame on pictore box
                    try
                    {
                        // Try to remove old image
                        if (this.pictureBox1.Image != null)
                            this.pictureBox1.Image.Dispose();

                        // Resize frame to pictureBox
                        CvInvoke.Resize(frameCopy, frameCopy, 
                            new Size(this.pictureBox1.Width, this.pictureBox1.Height), 0, 0, Inter.Linear);

                        // Draw new image
                        this.pictureBox1.Image = frameCopy.ToBitmap(true);
                    }
                    catch (Exception) { }
                }
            }

            // Close the cumera if the flag was cleared
            try
            {
                capture.Stop();
            }
            catch (Exception) { }
            Console.WriteLine("OpenCV loop finished");
        }

        /// <summary>
        /// Rotates 3D point around X Y Z axes (roll, pitch, yaw angles)
        /// </summary>
        /// <param name="point">1D array of 3 points of type double</param>
        /// <param name="rollRadians">roll angle in radians</param>
        /// <param name="pitchRadians">pitch angle in radians</param>
        /// <param name="yawRadians">yaw angle in radians</param>
        /// <returns>1D array of 3 points of type double</returns>
        private double[] rotate3D(ref double[] point, double rollRadians, double pitchRadians, double yawRadians)
        {
            // Calculate sin and cos values for each angle
            double rollSin = Math.Sin(rollRadians);
            double rollCos = Math.Cos(rollRadians);
            double pitchSin = Math.Sin(pitchRadians);
            double pitchCos = Math.Cos(pitchRadians);
            double yawSin = Math.Sin(yawRadians);
            double yawCos = Math.Cos(yawRadians);

            // Calculate rotation matrixes
            double[,] rollMat = { { 1, 0, 0 }, { 0, rollCos, -rollSin }, { 0, rollSin, rollCos } };
            double[,] pitchMat = { { pitchCos, 0, pitchSin }, { 0, 1, 0 }, { -pitchSin, 0, pitchCos } };
            double[,] yawMat = { { yawCos, -yawSin, 0 }, { yawSin, yawCos, 0 }, { 0, 0, 1 } };

            // Apply roll rotation matrix
            double xRoll = point[0] * rollMat[0, 0] + point[1] * rollMat[1, 0] + point[2] * rollMat[2, 0];
            double yRoll = point[0] * rollMat[0, 1] + point[1] * rollMat[1, 1] + point[2] * rollMat[2, 1];
            double zRoll = point[0] * rollMat[0, 2] + point[1] * rollMat[1, 2] + point[2] * rollMat[2, 2];

            // Apply pitch rotation matrix
            double xPitch = xRoll * pitchMat[0, 0] + yRoll * pitchMat[1, 0] + zRoll * pitchMat[2, 0];
            double yPitch = xRoll * pitchMat[0, 1] + yRoll * pitchMat[1, 1] + zRoll * pitchMat[2, 1];
            double zPitch = xRoll * pitchMat[0, 2] + yRoll * pitchMat[1, 2] + zRoll * pitchMat[2, 2];

            // Apply yaw rotation matrix
            double x = xPitch * yawMat[0, 0] + yPitch * yawMat[1, 0] + zPitch * yawMat[2, 0];
            double y = xPitch * yawMat[0, 1] + yPitch * yawMat[1, 1] + zPitch * yawMat[2, 1];
            double z = xPitch * yawMat[0, 2] + yPitch * yawMat[1, 2] + zPitch * yawMat[2, 2];

            // Return rotated coordinates
            return new double[] { x, y, z };
        }

        /// <summary>
        /// Loads cumera matrix and distorions from 2D array into Mat object
        /// </summary>
        private void loadCameraCalibration()
        {
            // Load cumera distortions
            distCoeffs = new Mat(1, distCoeffsArr.Length, DepthType.Cv32F, 1);
            for (int i = 0; i < distCoeffsArr.Length; i++)
                distCoeffs.SetValue(0, i, distCoeffsArr[i]);

            // Load cumera matrix
            cameraMatrix = new Mat(3, 3, DepthType.Cv32F, 1);
            for (int row = 0; row < 3; row++)
                for (int col = 0; col < 3; col++)
                    cameraMatrix.SetValue(row, col, cameraMatrixArr[row, col]);
        }

        /// <summary>
        /// Parses CSV file (with textBox2 delimiter) into List of type double[3]
        /// </summary>
        /// <param name="filePath"></param>
        /// <returns>List on double array</returns>
        private List<double[]> parseCSV(string filePath)
        {
            StreamReader streamReader = new StreamReader(filePath);
            var data = new List<double[]>();
            while (!streamReader.EndOfStream) {
                string[] lines = streamReader.ReadLine().Split(this.textBox2.Text[0]);
                if (lines != null && lines.Length > 0)
                {
                    // Stop parsing if number of coordinates != 3
                    if (lines.Length != 3)
                    {
                        MessageBox.Show("Error! Invalid CSV file! \nThe file must contain triples of numbers!");
                        break;
                    }
                    
                    // Convert strings to double
                    double[] linesDouble = new double[lines.Length];
                    for (int i = 0; i < lines.Length; i++)
                        linesDouble[i] = Convert.ToDouble(lines[i]);
                    data.Add(linesDouble);
                }
            }
            return data;
        }
    }
}
