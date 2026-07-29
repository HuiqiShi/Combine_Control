using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Thorlabs.MotionControl.XA;
using Thorlabs.MotionControl.XA.Products;
using FUTEK.Devices;
using Basler.Pylon;

namespace Combine_Control
{
    class Program
    {
        // ===== 可调参数 =====
        private static string _kstDeviceId = "26007392";
        private static string _usbSamplingRateHz = "20";
        private static double _startPositionMm = 48.0;
        private static double _targetPositionMm = 10.0;

        private static double _fastAccel = 3.0;
        private static double _fastVel = 3.0;
        private static double _slowAccel = 0.1;
        private static double _slowVel = 0.1;

        private static double _waitSec = 10.0;

        // ===== 力触发参数 =====
        private static double _firstTriggerForce = 0.06;
        private static double _forceStep = 0.03;
        private static double _forceStopThreshold = 0.35;

        // ===== 相机序列号 =====
        private const string LEFT_SERIAL = "40241344";
        private const string RIGHT_SERIAL = "40241347";

        // ===== 线程间共享 =====
        private static volatile bool _motionRunning = true;
        private static BlockingCollection<int> _cameraTriggerQueue = new BlockingCollection<int>();
        private static List<(IGrabResult Left, IGrabResult Right, double ShutterTime)> _capturedPairs
            = new List<(IGrabResult, IGrabResult, double)>();
        private static Stopwatch _sharedStopwatch = new Stopwatch();

        static void Main(string[] args)
        {
            string saveDir = Path.Combine(Environment.CurrentDirectory, "Captures");
            Directory.CreateDirectory(saveDir);

            // ========== 初始化 KST201 ==========
            SystemManager systemManager;
            try
            {
                systemManager = SystemManager.Create();
                systemManager.Startup();
            }
            catch (Exception ex)
            {
                Console.WriteLine("XA Startup Exception: {0}", ex.Message);
                return;
            }

            Kst201 kstDevice;
            if (!systemManager.TryOpenDevice(_kstDeviceId, "", OperatingModes.Default, out kstDevice))
            {
                Console.WriteLine("Failed to open KST201 device {0}", _kstDeviceId);
                systemManager.Shutdown();
                return;
            }
            Console.WriteLine("KST201 opened successfully.");

            // ========== 初始化 USB225 ==========
            DeviceRepository repo = new DeviceRepository();
            List<FUTEK.Devices.Device> usbDevices = repo.DetectDevices().ToList();
            if (usbDevices.Count == 0)
            {
                Console.WriteLine("No FUTEK device found.");
                kstDevice.Close();
                systemManager.Shutdown();
                return;
            }
            DeviceUSB225 usb225 = usbDevices.First() as DeviceUSB225;
            if (usb225 == null)
            {
                Console.WriteLine("The connected FUTEK device is not a USB225.");
                kstDevice.Close();
                systemManager.Shutdown();
                return;
            }
            Console.WriteLine("USB225 found and connected.");

            List<String> usbRates = usb225.GetChannelXSamplingRatePossibleValues(0);
            if (!usbRates.Contains(_usbSamplingRateHz))
            {
                Console.WriteLine("USB sampling rate {0} not supported. Available: {1}",
                    _usbSamplingRateHz, string.Join(", ", usbRates));
                kstDevice.Close();
                systemManager.Shutdown();
                return;
            }
            usb225.SetChannelXSamplingRate(0, usbRates.First(x => x == _usbSamplingRateHz));
            Console.WriteLine("USB225 sampling rate set to {0} Hz.", _usbSamplingRateHz);

            // ========== 初始化 双目相机 ==========
            Camera leftCamera = null;
            Camera rightCamera = null;
            try
            {
                leftCamera = new Camera(LEFT_SERIAL);
                leftCamera.CameraOpened += Configuration.AcquireContinuous;
                leftCamera.Open();
                rightCamera = new Camera(RIGHT_SERIAL);
                rightCamera.CameraOpened += Configuration.AcquireContinuous;
                rightCamera.Open();
                leftCamera.StreamGrabber.Start();
                rightCamera.StreamGrabber.Start();
                Console.WriteLine("Both cameras opened and streaming.");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Camera init failed: {0}", ex.Message);
                if (leftCamera != null) { leftCamera.Close(); leftCamera.Dispose(); }
                if (rightCamera != null) { rightCamera.Close(); rightCamera.Dispose(); }
                kstDevice.Close();
                systemManager.Shutdown();
                return;
            }

            var kstLog = new List<(double TimeSec, double PositionMm)>();
            var usbLog = new List<(double TimeSec, double Force)>();

            try
            {
                // ========== 使能、回零 ==========
                kstDevice.SetEnableState(EnableState.Enabled, TimeSpan.FromSeconds(1));
                ConnectedProductInfo productInfo = kstDevice.GetConnectedProductInfo();
                Unit deviceUnit = productInfo.UnitType;

                Console.WriteLine("Homing KST201...");
                kstDevice.Home(TimeSpan.FromSeconds(60));
                Console.WriteLine("Homing completed.");

                Console.WriteLine("Waiting {0} s...", _waitSec);
                Thread.Sleep((int)(_waitSec * 1000));

                // ---- 第一段：快速去48mm ----
                SetVelocity(kstDevice, deviceUnit, _fastAccel, _fastVel);
                long startInDeviceUnits = kstDevice.FromPhysicalToDeviceUnit(ScaleType.Distance, deviceUnit, _startPositionMm);
                Console.WriteLine("Moving to start position {0} mm (fast)...", _startPositionMm);
                kstDevice.Move(MoveMode.Absolute, (int)startInDeviceUnits, TimeSpan.FromSeconds(120));
                Console.WriteLine("Reached start position.");

                Console.WriteLine("Waiting {0} s before main motion...", _waitSec);
                Thread.Sleep((int)(_waitSec * 1000));

                // ---- 第二段：慢速参数 ----
                SetVelocity(kstDevice, deviceUnit, _slowAccel, _slowVel);
                Console.WriteLine("Velocity set (SLOW): Acc={0}, MaxVel={1}", _slowAccel, _slowVel);

                // ========== 启动相机触发处理线程 ==========
                Task cameraTask = Task.Run(() => CameraTriggerWorker(leftCamera, rightCamera));

                // ========== 启动 USB 流式采集 ==========
                usb225.PreStreamingOperations();

                long targetInDeviceUnits = kstDevice.FromPhysicalToDeviceUnit(ScaleType.Distance, deviceUnit, _targetPositionMm);

                // ========== 秒表与移动同步 ==========
                Task moveTask = Task.Run(() =>
                {
                    _sharedStopwatch.Start();
                    kstDevice.Move(MoveMode.Absolute, (int)targetInDeviceUnits, TimeSpan.FromSeconds(800));
                });

                double nextTriggerForce = _firstTriggerForce;
                double sampleInterval = 1.0 / double.Parse(_usbSamplingRateHz);

                // ========== USB采集线程（方案A：批内推算时间）==========
                Task usbTask = Task.Run(() =>
                {
                    while (!_sharedStopwatch.IsRunning) { }

                    while (_motionRunning)
                    {
                        StreamDataPoint[] points = usb225.GetStreamingDataConverted();
                        double batchTime = _sharedStopwatch.Elapsed.TotalSeconds;
                        for (int j = 0; j < points.Length; j++)
                        {
                            if (double.TryParse(points[j].ConvertedValue, out double forceVal))
                            {
                                double pointTime = batchTime + j * sampleInterval;
                                usbLog.Add((pointTime, forceVal));

                                if (forceVal >= nextTriggerForce)
                                {
                                    _cameraTriggerQueue.Add(1);
                                    while (forceVal >= nextTriggerForce)
                                        nextTriggerForce += _forceStep;
                                }

                                if (forceVal >= _forceStopThreshold)
                                {
                                    _motionRunning = false;
                                }
                            }
                        }
                    }
                });

                // ========== 主线程：KST 位置采集 ==========
                while (_motionRunning)
                {
                    Int32 posInDeviceUnits = kstDevice.GetPositionCounter(TimeSpan.FromMilliseconds(200));
                    UnitConversionResult posPhysical = kstDevice.FromDeviceUnitToPhysical(ScaleType.Distance, posInDeviceUnits);
                    kstLog.Add((_sharedStopwatch.Elapsed.TotalSeconds, posPhysical.Value));

                    if (moveTask.IsCompleted)
                        _motionRunning = false;
                }

                // ========== 停止 ==========
                try { kstDevice.Stop(StopMode.Immediate, TimeSpan.FromSeconds(5)); } catch { }

                usb225.PostStreamingOperations();

                _cameraTriggerQueue.CompleteAdding();
                cameraTask.Wait();

                try { moveTask.Wait(2000); } catch { }
                usbTask.Wait();
                _sharedStopwatch.Stop();

                Console.WriteLine("Acquisition complete.");
                Console.WriteLine("KST samples: {0}, USB samples: {1}, Camera pairs: {2}",
                    kstLog.Count, usbLog.Count, _capturedPairs.Count);

                // ========== 保存相机图片（快门时刻插值力和位置命名）==========
                Console.WriteLine("Saving camera images...");
                foreach (var pair in _capturedPairs)
                {
                    double t = pair.ShutterTime;
                    double posAtShutter = InterpolateValue(kstLog, t);
                    double forceAtShutter = InterpolateValue(usbLog, t);

                    string leftName = string.Format("left_{0:F3}s_{1:F3}lb_{2:F2}mm.png", t, forceAtShutter, posAtShutter);
                    string rightName = string.Format("right_{0:F3}s_{1:F3}lb_{2:F2}mm.png", t, forceAtShutter, posAtShutter);
                    SaveResult(pair.Left, Path.Combine(saveDir, leftName));
                    SaveResult(pair.Right, Path.Combine(saveDir, rightName));
                    Console.WriteLine("  Shutter @ {0:F3}s -> {1:F3}lb, {2:F2}mm", t, forceAtShutter, posAtShutter);
                }

                // ========== 输出合并 TXT（Time / Force / Position，以力点为基准插值位置）==========
                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                string mergedPath = Path.Combine(Environment.CurrentDirectory,
                    string.Format("Merged_ForceVsPosition_{0}.txt", ts));
                var mergedTxt = new StringBuilder();
                mergedTxt.AppendLine("Time(s)\tForce(lb)\tPosition(mm)");
                int mergedCount = 0;
                foreach (var fp in usbLog)
                {
                    double interpPos = InterpolateValue(kstLog, fp.TimeSec);
                    if (double.IsNaN(interpPos)) continue;
                    mergedTxt.AppendLine(string.Format("{0:F4}\t{1:F6}\t{2:F4}",
                        fp.TimeSec, fp.Force, interpPos));
                    mergedCount++;
                }
                File.WriteAllText(mergedPath, mergedTxt.ToString());
                Console.WriteLine("Merged data saved: {0} ({1} points)", mergedPath, mergedCount);

                // 各存一份原始数据
                SaveRawTxt(kstLog, "KST_Position", ts, "Time(s)\tPosition(mm)");
                SaveRawTxt(usbLog, "USB_Force", ts, "Time(s)\tForce(lb)");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Exception: {0}", ex.Message);
            }
            finally
            {
                _motionRunning = false;
                if (!_cameraTriggerQueue.IsAddingCompleted)
                    _cameraTriggerQueue.CompleteAdding();

                try { usb225.PostStreamingOperations(); } catch { }

                foreach (var pair in _capturedPairs)
                {
                    pair.Left?.Dispose();
                    pair.Right?.Dispose();
                }

                if (leftCamera != null) { try { leftCamera.StreamGrabber.Stop(); } catch { } leftCamera.Close(); leftCamera.Dispose(); }
                if (rightCamera != null) { try { rightCamera.StreamGrabber.Stop(); } catch { } rightCamera.Close(); rightCamera.Dispose(); }

                kstDevice.Disconnect();
                kstDevice.Close();
                systemManager.Shutdown();
            }

            Console.WriteLine("Press any key to exit.");
            Console.ReadKey();
        }

        static void SetVelocity(Kst201 kstDevice, Unit deviceUnit, double accel, double vel)
        {
            VelocityParams vp = kstDevice.GetVelocityParams(TimeSpan.FromSeconds(2));
            vp.Acceleration = (int)kstDevice.FromPhysicalToDeviceUnit(ScaleType.Acceleration, deviceUnit, accel);
            vp.MaxVelocity = (int)kstDevice.FromPhysicalToDeviceUnit(ScaleType.Velocity, deviceUnit, vel);
            kstDevice.SetVelocityParams(vp);
        }

        static void CameraTriggerWorker(Camera leftCamera, Camera rightCamera)
        {
            foreach (var trigger in _cameraTriggerQueue.GetConsumingEnumerable())
            {
                IGrabResult leftClone = null;
                IGrabResult rightClone = null;

                using (Barrier barrier = new Barrier(2))
                {
                    Task lt = Task.Run(() =>
                    {
                        barrier.SignalAndWait();
                        using (IGrabResult r = leftCamera.StreamGrabber.RetrieveResult(5000, TimeoutHandling.ThrowException))
                            leftClone = r.Clone();
                    });
                    Task rt = Task.Run(() =>
                    {
                        barrier.SignalAndWait();
                        using (IGrabResult r = rightCamera.StreamGrabber.RetrieveResult(5000, TimeoutHandling.ThrowException))
                            rightClone = r.Clone();
                    });
                    Task.WaitAll(lt, rt);
                }

                double shutterTime = _sharedStopwatch.Elapsed.TotalSeconds;

                lock (_capturedPairs)
                {
                    _capturedPairs.Add((leftClone, rightClone, shutterTime));
                }
            }
        }

        static void SaveResult(IGrabResult grabResult, string filePath)
        {
            if (grabResult != null && grabResult.GrabSucceeded)
                ImagePersistence.Save(ImageFileFormat.Png, filePath, grabResult);
        }

        static void SaveRawTxt(List<(double TimeSec, double Val)> log, string prefix, string ts, string header)
        {
            string path = Path.Combine(Environment.CurrentDirectory, string.Format("{0}_{1}.txt", prefix, ts));
            var sb = new StringBuilder();
            sb.AppendLine(header);
            foreach (var e in log)
                sb.AppendLine(string.Format("{0:F4}\t{1:F6}", e.TimeSec, e.Val));
            File.WriteAllText(path, sb.ToString());
        }

        static double InterpolateValue(List<(double TimeSec, double Val)> log, double t)
        {
            if (log.Count == 0) return double.NaN;
            if (t < log[0].TimeSec || t > log[log.Count - 1].TimeSec) return double.NaN;
            for (int i = 0; i < log.Count - 1; i++)
            {
                double t1 = log[i].TimeSec, t2 = log[i + 1].TimeSec;
                if (t >= t1 && t <= t2)
                {
                    double v1 = log[i].Val, v2 = log[i + 1].Val;
                    if (t2 - t1 < 1e-9) return v1;
                    return v1 + (t - t1) / (t2 - t1) * (v2 - v1);
                }
            }
            return log[log.Count - 1].Val;
        }
    }
}