#include "stdafx.h"
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "detector.h"

#define KEY_ESCAPE 27      // ESC ¼ü
#define KEY_SPACE 32       // ¿Õ¸ñ¼ü
#define KEY_RETURN 13      // »Ø³µ¼ü
#define CV_EDIT_VIEW	"¾µÆ¬"

int refreshUI(cv::Mat frame, cv::Mat background)
{
	bool refresh = false;
	int medianBlurKSize = -1, morphKSize = -1;
	int medianBlurTrack = 0, morphKTrack = 9;

	int margin = 50, padding = 15, settingWidth = 480, settingHeight = 270;
	int settingX = background.cols - settingWidth - margin, settingY = background.rows - settingHeight - margin;

	Detector detector = Detector();

	while (cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
		if ( (medianBlurKSize != medianBlurTrack || morphKSize != morphKTrack) && !cvui::mouse(cvui::LEFT_BUTTON, cvui::IS_DOWN) && !cvui::mouse(cvui::RIGHT_BUTTON, cvui::IS_DOWN)
			|| refresh) {
			medianBlurKSize = medianBlurTrack;
			morphKSize = morphKTrack;
			refresh = false;

			detector.detect(frame.clone(), (medianBlurKSize << 1) + 1, morphKSize, background);
		}

		cvui::window(background, settingX, settingY, settingWidth, settingHeight, "Setting");

		cvui::text(background, settingX + padding, settingY + margin, "Edge Curl");
		cvui::trackbar(background, settingX + padding * 2, settingY + margin + padding, settingWidth - padding * 3, &medianBlurTrack, 0, 9, 0, "%.0Lf");

		cvui::text(background, settingX + padding, settingY + (margin + padding) * 2, "Morph Kernel");
		cvui::trackbar(background, settingX + padding * 2, settingY + (margin + padding) * 2 + padding, settingWidth - padding * 3, &morphKTrack, 1, 50, 1, "%.0Lf");

		if (cvui::button(background, settingX + settingWidth - margin * 4, settingY + settingHeight - (margin + padding), "FindNext")) {
			detector.findNext();
			refresh = true;
		}

		if (cvui::button(background, settingX + settingWidth - margin * 2, settingY + settingHeight - (margin + padding), "Save")) {
			detector.saveToDxf("eyeglass.dxf");
		}
		cvui::update();
		cv::imshow(CV_EDIT_VIEW, background);

		int key = cv::waitKey(20);
		switch (key) {
		case KEY_ESCAPE:
		case KEY_RETURN:
		case KEY_SPACE:
			return key;
		}
	}
}

int main(int argc, char* argv[]) {
	cv::VideoCapture capture;
	if (argc <= 1)
		capture.open(cv::CAP_ANY);
	else
		capture.open(argv[1]);
	if (!capture.isOpened())
		return -1;

	cv::VideoWriter writer;

	cv::Mat frame;
	capture >> frame;

	cvui::init(CV_EDIT_VIEW);

	cv::namedWindow(CV_EDIT_VIEW, cv::WINDOW_NORMAL);
	cv::setWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	cv::Size screenSize = cv::Size(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN)-GetSystemMetrics(SM_CYCAPTION)-GetSystemMetrics(SM_CYMENU));

	if (!frame.empty()) {
		double fps = 1;// capture.get(cv::CAP_PROP_FPS);
		int fourcc = (int)capture.get(cv::CAP_PROP_FOURCC);
#if _DEBUG
		char fourcc_name[] = {
			(char)fourcc, // First character is lowest bits
			(char)(fourcc >> 8), // Next character is bits 8-15
			(char)(fourcc >> 16), // Next character is bits 16-23
			(char)(fourcc >> 24), // Last character is bits 24-31
			'\0' // and don't forget to terminate
		};
		printf("fourcc: %s\r\n", fourcc_name);
#endif
		fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
		writer.open("demo.mp4", fourcc, fps, screenSize);
	}

	int n = 1;
	cv::Mat background(screenSize, CV_8UC3, cv::Scalar(0));
	while (!frame.empty() || ++n < argc) {
		if (frame.empty() && capture.open(argv[n])) {
			capture >> frame;
			if (frame.empty())
				break;
		}

		double scale = frame.rows / 960.0;
		cv::resize(frame, frame, cv::Size(cvRound(frame.cols / scale), cvRound(frame.rows / scale)), 0, 0, cv::INTER_LINEAR);

		if (refreshUI(frame, background) == KEY_ESCAPE)
			break;

		if (writer.isOpened()) {
			writer.write(background);
		}

		background.setTo(cv::Scalar(0));//Çå¿Õ±³¾°
		capture >> frame;
	}
	writer.release();
	capture.release();

	cv::destroyAllWindows();
	return 0;
}
