#include "stdafx.h"
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "detector.h"
#include <codecvt>

#define KEY_UNPRESS -1			// 无按键
#define KEY_ESCAPE	27			// ESC 键
#define KEY_SPACE	32			// 空格键
#define KEY_RETURN	13			// 回车键
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

bool isRegisted(const char *code) {
	DWORD dwCode = 0;
	DWORD dwSerial = 0;
	DWORD dwIDESerial = 0;
	//GetVolumeInformation("\\", NULL, NULL, &dwIDESerial, NULL, NULL, NULL, NULL);
	int cpuInfo[4] = { 0 };
	__cpuid(cpuInfo, 1); // EAX=1 查询 Processor ID
	dwIDESerial = (cpuInfo[0] & 0xFFFFFFFFl + cpuInfo[3] & 0xFFFFFFFFl) & 0xFFFFFFFFl;//cpuInfo[1]可能每次不一样

	sscanf(code, "%x", &dwCode);
	while (dwCode)
	{
		dwSerial = dwSerial * 10 + dwCode % 10;
		dwCode /= 10;
	}

	return dwSerial == dwIDESerial;
}

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
	Detector* detector = static_cast<Detector*>(userdata);

	cv::Rect editArea = detector->getEditArea();
	if (!editArea.contains(cv::Point(x, y))) {
		// 将鼠标事件传递给 cvui 的事件管理系统
		cvui::handleMouse(event, x, y, flags, &cvui::internal::getContext(CV_EDIT_VIEW));
		return;
	}

	detector->onMouse(event, x, y);
}

int refreshUI(cv::Mat frame, cv::Mat background, cv::VideoWriter writer, bool& isEdit)
{
	bool refresh = false;
	int medianBlurKSize = -1, morphKSize = -1;
	int medianBlurTrack = 0, morphKTrack = 7;

	int margin = 50, padding = 15, settingWidth = 240, settingHeight = 270;
	int settingX = background.cols - settingWidth - margin, settingY = background.rows - settingHeight - margin;

	Detector detector = Detector(cv::Rect(0, 0, frame.cols, frame.rows));
	if (cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
		cv::setMouseCallback(CV_EDIT_VIEW, mouseCallback, &detector);
	}

	std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
	std::string code = converter.to_bytes(L"QQ:515311445");
	bool registed = isRegisted(code.c_str());

	while (cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
		if ( (medianBlurKSize != medianBlurTrack || morphKSize != morphKTrack) && !cvui::mouse(cvui::LEFT_BUTTON, cvui::IS_DOWN) && !cvui::mouse(cvui::RIGHT_BUTTON, cvui::IS_DOWN)
			|| refresh) {
			medianBlurKSize = medianBlurTrack;
			morphKSize = morphKTrack;
			refresh = false;

			detector.detect(frame.clone(), (medianBlurKSize << 1) + 1, morphKSize + 1, background);
		}
		else {
			detector.drawFrame(frame.clone(), background);
		}

		cvui::window(background, settingX, settingY, settingWidth, settingHeight, "Setting");

		cvui::text(background, settingX + padding, settingY + margin, "Edge Curl");
		cvui::trackbar(background, settingX + padding * 2, settingY + margin + padding, settingWidth - padding * 3, &medianBlurTrack, 0, 9, 0, "%.0Lf");

		cvui::text(background, settingX + padding, settingY + (margin + padding) * 2, "Morph Kernel");
		cvui::trackbar(background, settingX + padding * 2, settingY + (margin + padding) * 2 + padding, settingWidth - padding * 3, &morphKTrack, 0, 9, 1, "%.0Lf");

		if(!registed && code.find("QQ")==0) cvui::text(background, frame.cols/2- padding, frame.rows/2, code, 1.0, 0xFF0000);
		if (cvui::button(background, settingX + settingWidth - margin * 4, settingY + settingHeight - (margin + padding), "FindNext")) {
			detector.findNext();
			refresh = true;
			isEdit = true;
		}

		if (cvui::button(background, settingX + settingWidth - margin * 2, settingY + settingHeight - (margin + padding), "Save")) {
			if (registed) {
				detector.saveToDxf("eyeglass.dxf");
			}
			else {
				STARTUPINFO si;
				ZeroMemory(&si, sizeof(si));
				si.cb = sizeof(si);

				PROCESS_INFORMATION pi;
				ZeroMemory(&pi, sizeof(pi));

				CreateProcess("Crack.exe", NULL, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi);

				CloseHandle(pi.hProcess);
				CloseHandle(pi.hThread);
				return KEY_ESCAPE;
			}
			isEdit = true;
		}
		cvui::update();

		cv::imshow(CV_EDIT_VIEW, background);

		if (writer.isOpened()) {
			writer.write(background);
		}

		int key = cv::waitKeyEx(20);
		trace("key: %d\r\n", key);
		switch (key) {
		case KEY_UNPRESS:
			if (!isEdit) return key;
			break;

		case KEY_ESCAPE:
			return key;

		case KEY_RETURN:
		case KEY_SPACE:
			isEdit = !isEdit;
			break;

		default:
			detector.onKey(key);
			break;
		}
	}
	return KEY_ESCAPE;
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

	double frameCount = capture.get(cv::CAP_PROP_FRAME_COUNT);
	trace("frameCount: %d", frameCount);

	int n = 1;
	bool isEdit = frameCount<=1;
	cv::Mat background(screenSize, CV_8UC3, cv::Scalar(0));
	while (!frame.empty() || ++n < argc) {
		if (frame.empty() && capture.open(argv[n])) {
			capture >> frame;
			if (frame.empty())
				break;

			isEdit = true;
		}

		double scale = frame.rows / std::min(960.0, screenSize.height*1.0);
		cv::resize(frame, frame, cv::Size(cvRound(frame.cols / scale), cvRound(frame.rows / scale)), 0, 0, cv::INTER_LINEAR);

		if (refreshUI(frame, background, writer, isEdit) == KEY_ESCAPE)
			break;

		background.setTo(cv::Scalar(0));//清空背景
		capture >> frame;
	}
	writer.release();
	capture.release();

	cv::destroyAllWindows();
	return 0;
}
