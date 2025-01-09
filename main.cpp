#include "stdafx.h"
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "detector.h"

#define KEY_ESCAPE	27			// ESC 键
#define KEY_SPACE	32			// 空格键
#define KEY_RETURN	13			// 回车键
#define KEY_LEFT	0x250000	// 向左
#define KEY_TOP		0x260000	// 向上
#define KEY_RIGHT	0x270000	// 向右
#define KEY_BOTTOM	0x280000	// 向下
#define KEY_DEL		0x2E0000	// 删除
#define CV_EDIT_VIEW	"镜片"

//打印日志到Output窗口
void trace(char* fmt, ...) {
	char out[1024];
	va_list body;
	va_start(body, fmt);
	vsprintf(out, fmt, body);
	va_end(body);
	OutputDebugStringA(out);
}

cv::Rect selectRect;
bool isEditSelectArea = false;
void mouseCallback(int event, int x, int y, int flags, void* userdata) {
	cv::Rect* editArea = static_cast<cv::Rect*>(userdata);
	if (!editArea->contains(cv::Point(x, y))) {
		// 将鼠标事件传递给 cvui 的事件管理系统
		cvui::handleMouse(event, x, y, flags, &cvui::internal::getContext(CV_EDIT_VIEW));
		return;
	}

	if (event == cv::EVENT_LBUTTONDOWN) {
		isEditSelectArea = true;
		selectRect = cv::Rect(x, y, 0, 0);
	}
	else if (event == cv::EVENT_MOUSEMOVE) {
		if (isEditSelectArea) {
			int rectX = std::min(selectRect.x, x);
			int rectY = std::min(selectRect.y, y);
			int rectWidth = std::abs(x - selectRect.x);
			int rectHeight = std::abs(y - selectRect.y);

			selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
		}
	}
	else if (event == cv::EVENT_LBUTTONUP) {
		isEditSelectArea = false;
		int rectX = std::min(selectRect.x, x);
		int rectY = std::min(selectRect.y, y);
		int rectWidth = std::abs(x - selectRect.x);
		int rectHeight = std::abs(y - selectRect.y);

		selectRect = cv::Rect(rectX, rectY, rectWidth, rectHeight);
	}
}

int refreshUI(cv::Mat frame, cv::Mat background, cv::VideoWriter writer)
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
		else {
			detector.drawFrame(frame.clone(), background);
		}

		cv::Mat tmp = background.clone();
		cvui::window(tmp, settingX, settingY, settingWidth, settingHeight, "Setting");

		cvui::text(tmp, settingX + padding, settingY + margin, "Edge Curl");
		cvui::trackbar(tmp, settingX + padding * 2, settingY + margin + padding, settingWidth - padding * 3, &medianBlurTrack, 0, 9, 0, "%.0Lf");

		cvui::text(tmp, settingX + padding, settingY + (margin + padding) * 2, "Morph Kernel");
		cvui::trackbar(tmp, settingX + padding * 2, settingY + (margin + padding) * 2 + padding, settingWidth - padding * 3, &morphKTrack, 1, 50, 1, "%.0Lf");

		if (cvui::button(tmp, settingX + settingWidth - margin * 4, settingY + settingHeight - (margin + padding), "FindNext")) {
			detector.findNext();
			refresh = true;
		}

		if (cvui::button(tmp, settingX + settingWidth - margin * 2, settingY + settingHeight - (margin + padding), "Save")) {
			detector.saveToDxf("eyeglass.dxf");
		}
		cvui::update();

		cv::rectangle(tmp, selectRect, cv::Scalar(255, 0, 255), 2);
		cv::imshow(CV_EDIT_VIEW, tmp);

		if (writer.isOpened()) {
			writer.write(tmp);
		}

		int key = cv::waitKeyEx(20);
		trace("key: %d\r\n", key);
		switch (key) {
		case KEY_ESCAPE:
		case KEY_RETURN:
		case KEY_SPACE:
			return key;

		case KEY_LEFT:
			detector.moveContour(selectRect, -1, 0);
			break;

		case KEY_RIGHT:
			detector.moveContour(selectRect, 1, 0);
			break;

		case KEY_TOP:
			detector.moveContour(selectRect, 0, -1);
			break;

		case KEY_BOTTOM:
			detector.moveContour(selectRect, 0, 1);
			break;

		case KEY_DEL:
			detector.deleteContour(selectRect);
			break;
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
		double fps = capture.get(cv::CAP_PROP_FPS);
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

		if(cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
			selectRect = cv::Rect();
			cv::Rect editArea = { 0,0,frame.cols,frame.rows };
			cv::setMouseCallback(CV_EDIT_VIEW, mouseCallback, &editArea);
		}

		if (refreshUI(frame, background, writer) == KEY_ESCAPE)
			break;

		background.setTo(cv::Scalar(0));//清空背景
		capture >> frame;
	}
	writer.release();
	capture.release();

	cv::destroyAllWindows();
	return 0;
}
