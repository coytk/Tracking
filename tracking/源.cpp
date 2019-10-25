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
# define AREA_THRESHOLD_MIN 2 //---- �֊s���o�̍ŏ��l 1.5
# define AREA_THRESHOLD_MAX 100 //---- �֊s���o�̍ő�l 30
# define CAPTURE_BUFFER_NUM 50
# define TRACKING_ROI_SIZE 25 //---- �g���b�L���O����ROI�T�C�Y
# define TRACKING_ROI_SIZE_HALF ((TRACKING_ROI_SIZE-1)/2)



int main() {
# if USE_PROJECTOR
	//---- HighSpeedProjector�ڑ�

# endif


	//    �J�����N��
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


	std::vector<cv::Mat> frameGrayBuffer;        //�摜�f�[�^���Ǘ����郊���O�o�b�t�@
	cv::Mat frameGray_bin = cv::Mat::zeros(height, width, CV_8UC1);    //��l���摜
	cv::Mat frameGray = cv::Mat::zeros(height, width, CV_8UC1);
	int latest_buffer_idx = 0;        // �ŐV�̉摜�̃o�b�t�@�ʒu
	int latest_proc_buffer_idx = 0;        //���O�ɓǂݎ�����o�b�t�@�̈ʒu

	cv::Point centroid_arr;        //�d�S���W


	//    frameGrayBuffer������
	for (int buffer_idx = 0; buffer_idx < CAPTURE_BUFFER_NUM; buffer_idx++) {
		frameGrayBuffer.push_back(cv::Mat::zeros(height, width, CV_8UC1));
	}

	cam.start();

	cv::namedWindow("contours", cv::WINDOW_AUTOSIZE | cv::WINDOW_FREERATIO);
	//�d�S�ʒu������
	while (1) {
		//�摜�擾
		cam.captureFrame(frameGray.data);
		//    2�l��
		cv::adaptiveThreshold(frameGray, frameGray_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8);
		//    �֊s���o
		std::vector<std::vector<cv::Point>> contour_;
		cv::findContours(frameGray_bin, contour_, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
		//    �ʐς̏�����or�傫���֊s���폜
		auto contour_it = std::remove_if(contour_.begin(), contour_.end(), [](const std::vector<cv::Point> & p) {return cv::contourArea(p) < AREA_THRESHOLD_MIN || cv::contourArea(p) > AREA_THRESHOLD_MAX; });
		contour_.erase(contour_it, contour_.end());
		//if (contour_.size() != 1) return -1;
		//    �d�S���v�Z
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

	//    �g���b�L���O�J�n
	bool flag = true;

	//    �摜�擾
	std::thread thrA([&] {  // frameGrayBuffer�ɎB�������摜��ۑ�����X���b�h 
		while (flag) {  //  �X���b�hA�ł�(�f�[�^����������)
			if ((latest_buffer_idx + 1) < frameGrayBuffer.size()) {  
				cam.captureFrame(frameGrayBuffer[latest_buffer_idx + 1].data);  // �J��������摜���擾��latest_buffer_idx + 1�̈ʒu�ɕۑ�
				latest_buffer_idx++;  //  latest_buffer_idx++�Ƃ��ŐV�̉摜�̈ʒu���X�V���Ă��܂�
			}
			else {  // latest_buffer_idx++ = buf_size�̎��Alatest_buffer_idx = 0�Ƃ��Ēl���X�V
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

				if (0 <= roipt1.x && 0 <= roipt1.y && roipt2.x <= width && roipt2.y <= height) { // ROI����ʓ��ɂ���
					cv::Rect roi(roipt1, roipt2); //Rect�� pt1 <= x,y < pt2
					cv::Mat grayroiimg = frameGrayBuffer[latest_buffer_idx](roi);
					latest_proc_buffer_idx = latest_buffer_idx;

					cv::Mat binroiimg;
					cv::adaptiveThreshold(grayroiimg, binroiimg, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 7, 8); // Method=MEAN_C, Block=5*5, C=8

					cv::Moments moment = cv::moments(binroiimg, true);
					cv::Point2f roicentroid(moment.m10 / moment.m00, moment.m01 / moment.m00); // ROI���^���������ꍇ��moment�ɕςȒl������͗l

					if (roicentroid.x >= 0 && roicentroid.y >= 0) { // �g���b�L���O����
						centroid_arr = roipt1 + roicentroid;
					}//�g���b�L���O���s���X�V���Ȃ�        
				}
			}
		}
	});
	//��ʕ\��
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