# pragma once

# include <opencv2/opencv.hpp>
# include <thread>
# include <stdio.h>
# include <windows.h>

# ifdef _DEBUG
# define CAMERA_EXT "d.lib"
# else
# define CAMERA_EXT ".lib"
# endif

# define STR_EXP(__A) #__A
# define STR(__A) STR_EXP(__A)
# define CV_VER STR(CV_VERSION_MAJOR) STR(CV_VERSION_MINOR) STR(CV_VERSION_REVISION)
# pragma comment(lib, "opencv_world410d.lib")


# include <HSC/baslerClass.hpp>
# pragma comment(lib, "BaslerLib" CAMERA_EXT)

# define USE_PROJECTOR 0
# define AREA_THRESHOLD_MIN 2 //---- 輪郭抽出の最小値 1.5
# define AREA_THRESHOLD_MAX 100 //---- 輪郭抽出の最大値 30
# define CAPTURE_BUFFER_NUM 50
# define TRACKING_ROI_SIZE 25 //---- トラッキング時のROIサイズ
# define TRACKING_ROI_SIZE_HALF ((TRACKING_ROI_SIZE-1)/2)



int main() {
# if USE_PROJECTOR
	//---- HighSpeedProjector接続

# endif


	//    カメラ起動
	basler cam;

	int width = 720;
	int height = 540;
	float  fps = 500.0f;
	float gain = 1.0f;

	cam.connect(0);
	cam.setParam(paramTypeCamera::paramInt::WIDTH, width);
	cam.setParam(paramTypeCamera::paramInt::HEIGHT, height);
	cam.setParam(paramTypeCamera::paramFloat::FPS, fps);
	cam.setParam(paramTypeCamera::paramFloat::GAIN, gain);
	cam.setParam(paramTypeBasler::Param::ExposureTime, 1000.0f);
	cam.setParam(paramTypeBasler::AcquisitionMode::EnableAcquisitionFrameRate);
	//cam.setParam(paramTypeBasler::FastMode::SensorReadoutModeFast);
	cam.setParam(paramTypeBasler::GrabStrategy::OneByOne);
	cam.setParam(paramTypeBasler::CaptureType::MonocroGrab);
	//cam.setParam(paramTypeBasler::CaptureType::ColorBGRGrab);
	cam.parameter_all_print();


	std::vector<cv::Mat> frameGrayBuffer;        //画像データを管理するリングバッファ
	cv::Mat frameGray_bin = cv::Mat::zeros(height, width, CV_8UC1);    //二値化画像
	cv::Mat frameGray = cv::Mat::zeros(height, width, CV_8UC1);
	int latest_buffer_idx = 0;        // 最新の画像のバッファ位置
	int latest_proc_buffer_idx = 0;        //直前に読み取ったバッファの位置

	cv::Point centroid_arr;        //重心座標


	//    frameGrayBuffer初期化
	for (int buffer_idx = 0; buffer_idx < CAPTURE_BUFFER_NUM; buffer_idx++) {
		frameGrayBuffer.push_back(cv::Mat::zeros(height, width, CV_8UC1));
	}

	cam.start();

	cv::namedWindow("contours", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
	//重心位置初期化
	while (1) {
		//画像取得
		cam.captureFrame(frameGray.data);
		//    2値化
		cv::adaptiveThreshold(frameGray, frameGray_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8);
		//    輪郭抽出
		std::vector<std::vector<cv::Point>> contour_;
		cv::findContours(frameGray_bin, contour_, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		//    面積の小さいor大きい輪郭を削除
		auto contour_it = std::remove_if(contour_.begin(), contour_.end(), [](const std::vector<cv::Point> & p) {return cv::contourArea(p) < AREA_THRESHOLD_MIN || cv::contourArea(p) > AREA_THRESHOLD_MAX; });
		contour_.erase(contour_it, contour_.end());
		//if (contour_.size() != 1) return -1;
		//    重心を計算
		for (auto& contour : contour_) {
			cv::Moments moment = cv::moments(contour, true);
			centroid_arr = cv::Point(moment.m10 / moment.m00, moment.m01 / moment.m00);
		}
		cv::circle(frameGray, centroid_arr, 3, cv::Scalar(255), 1);  
		cv::imshow("contours", frameGray);
		if (cv::waitKey(1) >= 0) {
			break;
		}
	}
	cv::destroyAllWindows();

	//    トラッキング開始
	bool flag = true;

	//    画像取得
	std::thread thrA([&] {  // frameGrayBufferに撮像した画像を保存するスレッド 
		while (flag) {  //  スレッドAでは(データを書き込む)
			if ((latest_buffer_idx + 1) < frameGrayBuffer.size()) {  
				cam.captureFrame(frameGrayBuffer[latest_buffer_idx + 1].data);  // カメラから画像を取得しlatest_buffer_idx + 1の位置に保存
				latest_buffer_idx++;  //  latest_buffer_idx++とし最新の画像の位置を更新しています
			}
			else {  // latest_buffer_idx++ = buf_sizeの時、latest_buffer_idx = 0として値を更新
				cam.captureFrame(frameGrayBuffer[0].data);
				latest_buffer_idx = 0;
			}
		}
	});

	std::thread thrB([&] { 
		while (flag) {  

			if (latest_proc_buffer_idx != latest_buffer_idx) {

				cv::Point2f roipt1(centroid_arr.x - TRACKING_ROI_SIZE_HALF, centroid_arr.y - TRACKING_ROI_SIZE_HALF);
				cv::Point2f roipt2(centroid_arr.x + TRACKING_ROI_SIZE_HALF + 1, centroid_arr.y + TRACKING_ROI_SIZE_HALF + 1);

				if (0 <= roipt1.x && 0 <= roipt1.y && roipt2.x <= width && roipt2.y <= height) { // ROIが画面内にある
					cv::Rect roi(roipt1, roipt2); //Rectは pt1 <= x,y < pt2
					cv::Mat grayroiimg = frameGrayBuffer[latest_buffer_idx](roi);
					latest_proc_buffer_idx = latest_buffer_idx;

					cv::Mat binroiimg;
					cv::adaptiveThreshold(grayroiimg, binroiimg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 7, 8); // Method=MEAN_C, Block=5*5, C=8

					cv::Moments moment = cv::moments(binroiimg, true);
					cv::Point2f roicentroid(moment.m10 / moment.m00, moment.m01 / moment.m00); // ROIが真白だった場合はmomentに変な値が入る模様

					if (roicentroid.x >= 0 && roicentroid.y >= 0) { // トラッキング成功
						centroid_arr = roipt1 + roicentroid;
					}//トラッキング失敗時更新しない        
				}
			}
		}
	});
	//画面表示
	while (1) {
		//frameGray = frameGrayBuffer[latest_buffer_idx];
		cv::Mat frameDisp(height, width, CV_8UC3);
		cv::cvtColor(frameGray, frameDisp, cv::COLOR_GRAY2BGR);
		cv::circle(frameDisp, centroid_arr, 3, cv::Scalar(255, 0, 0), 1);
		cv::imshow("frameGray_tmp", frameDisp);
		int key = cv::waitKey(1);
		if (key == 'q')break;
	}

	flag = false;
	thrA.join();
	thrB.join();

	cam.stop();
	cam.disconnect();
}