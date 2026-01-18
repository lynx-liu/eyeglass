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

cv::VideoCapture capture;

namespace ui {
	static inline cv::Scalar bgr(unsigned int rgb) {
		return cv::Scalar(rgb & 0xFF, (rgb >> 8) & 0xFF, (rgb >> 16) & 0xFF);
	}

	struct Theme {
		cv::Scalar panelBg = bgr(0x1F232A);
		cv::Scalar panelBorder = bgr(0x3A404A);
		cv::Scalar accent = bgr(0x4AA3FF);
	};

	static const Theme kTheme{};

	static constexpr int kHeaderH = 56;

	static inline void filledRoundedRect(cv::Mat& img, const cv::Rect& r, const cv::Scalar& color, int radius) {
		const int rad = std::max(0, std::min({ radius, r.width / 2, r.height / 2 }));
		if (rad == 0) {
			cv::rectangle(img, r, color, cv::FILLED, cv::LINE_AA);
			return;
		}

		cv::rectangle(img, cv::Rect(r.x + rad, r.y, r.width - 2 * rad, r.height), color, cv::FILLED, cv::LINE_AA);
		cv::rectangle(img, cv::Rect(r.x, r.y + rad, r.width, r.height - 2 * rad), color, cv::FILLED, cv::LINE_AA);
		cv::circle(img, cv::Point(r.x + rad, r.y + rad), rad, color, cv::FILLED, cv::LINE_AA);
		cv::circle(img, cv::Point(r.x + r.width - rad, r.y + rad), rad, color, cv::FILLED, cv::LINE_AA);
		cv::circle(img, cv::Point(r.x + rad, r.y + r.height - rad), rad, color, cv::FILLED, cv::LINE_AA);
		cv::circle(img, cv::Point(r.x + r.width - rad, r.y + r.height - rad), rad, color, cv::FILLED, cv::LINE_AA);
	}

	static inline void roundedRect(cv::Mat& img, const cv::Rect& r, const cv::Scalar& color, int thickness, int radius) {
		const int rad = std::max(0, std::min({ radius, r.width / 2, r.height / 2 }));
		if (rad == 0) {
			cv::rectangle(img, r, color, thickness, cv::LINE_AA);
			return;
		}

		cv::line(img, cv::Point(r.x + rad, r.y), cv::Point(r.x + r.width - rad, r.y), color, thickness, cv::LINE_AA);
		cv::line(img, cv::Point(r.x + rad, r.y + r.height), cv::Point(r.x + r.width - rad, r.y + r.height), color, thickness, cv::LINE_AA);
		cv::line(img, cv::Point(r.x, r.y + rad), cv::Point(r.x, r.y + r.height - rad), color, thickness, cv::LINE_AA);
		cv::line(img, cv::Point(r.x + r.width, r.y + rad), cv::Point(r.x + r.width, r.y + r.height - rad), color, thickness, cv::LINE_AA);

		cv::ellipse(img, cv::Point(r.x + rad, r.y + rad), cv::Size(rad, rad), 180, 0, 90, color, thickness, cv::LINE_AA);
		cv::ellipse(img, cv::Point(r.x + r.width - rad, r.y + rad), cv::Size(rad, rad), 270, 0, 90, color, thickness, cv::LINE_AA);
		cv::ellipse(img, cv::Point(r.x + r.width - rad, r.y + r.height - rad), cv::Size(rad, rad), 0, 0, 90, color, thickness, cv::LINE_AA);
		cv::ellipse(img, cv::Point(r.x + rad, r.y + r.height - rad), cv::Size(rad, rad), 90, 0, 90, color, thickness, cv::LINE_AA);
	}

	static inline void panel(cv::Mat& img, const cv::Rect& r, const char* title) {
		filledRoundedRect(img, r, kTheme.panelBg, 12);
		roundedRect(img, r, kTheme.panelBorder, 1, 12);

		cvui::text(img, r.x + 14, r.y + 18, title, 0.55, 0xE6E6E6);
		cv::line(img, cv::Point(r.x + 12, r.y + kHeaderH), cv::Point(r.x + r.width - 12, r.y + kHeaderH), kTheme.panelBorder, 1, cv::LINE_AA);
	}

	static inline void sectionTitle(cv::Mat& img, int x, int y, const char* title) {
		cvui::text(img, x, y, title, 0.45, 0xA9B0BA);
		cv::line(img, cv::Point(x, y + 16), cv::Point(x + 90, y + 16), kTheme.accent, 1, cv::LINE_AA);
	}
}

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

void getNextFrame(cv::Mat dest) {
    cv::Mat frame;
    capture >> frame;
    if(!frame.empty()) cv::resize(frame, dest, dest.size(), 0, 0, cv::INTER_LINEAR);
}

int refreshUI(cv::Mat frame, cv::Mat background, cv::VideoWriter writer, bool& isEdit, double pxToMm, void* userdata)
{
    Detector* detector = static_cast<Detector*>(userdata);

    double angle = 0.0;
    bool refresh = false;
    bool isPreview = false, prePreview = isPreview;
    int clipLimitValue = -1, medianBlurKSize = -1, morphKSize = -1;
    int clipLimitTrack = 0, medianBlurTrack = 0, morphKTrack = 7;

    int margin = 40, padding = 16, paddingH = 10, settingWidth = 240, settingHeight = 720;
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
            if(isPreview) getNextFrame(frame);
            detector->drawFrame(detector->rotate(frame.clone(), angle), background, isEdit);
        }

        ui::panel(background, settingsRect, "Settings");

        const int innerX = settingX + padding;
        const int innerY = settingY + ui::kHeaderH + 14;
        const int innerW = settingWidth - padding * 2;
        const int rowH = 38;

        if (onlyContour) {
            ui::sectionTitle(background, innerX, innerY - 26, "Edit contour");

            if (cvui::button(background, innerX, innerY + 0 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "+")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x, editArea.y + editArea.height / 2, +1);
            }
            cvui::text(background, innerX + 6, innerY + 0 * (rowH + paddingH) + 12, "Left", 0.45, 0xA9B0BA);
            if (cvui::button(background, innerX + innerW / 2, innerY + 0 * (rowH + paddingH), innerW / 2, rowH, "-")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x, editArea.y + editArea.height / 2, -1);
            }

            if (cvui::button(background, innerX, innerY + 1 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "+")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width, editArea.y + editArea.height / 2, +1);
            }
            cvui::text(background, innerX + 6, innerY + 1 * (rowH + paddingH) + 12, "Right", 0.45, 0xA9B0BA);
            if (cvui::button(background, innerX + innerW / 2, innerY + 1 * (rowH + paddingH), innerW / 2, rowH, "-")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width, editArea.y + editArea.height / 2, -1);
            }

            if (cvui::button(background, innerX, innerY + 2 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "+")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width / 2, editArea.y, +1);
            }
            cvui::text(background, innerX + 6, innerY + 2 * (rowH + paddingH) + 12, "Top", 0.45, 0xA9B0BA);
            if (cvui::button(background, innerX + innerW / 2, innerY + 2 * (rowH + paddingH), innerW / 2, rowH, "-")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width / 2, editArea.y, -1);
            }

            if (cvui::button(background, innerX, innerY + 3 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "+")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width / 2, editArea.y + editArea.height, +1);
            }
            cvui::text(background, innerX + 6, innerY + 3 * (rowH + paddingH) + 12, "Bottom", 0.45, 0xA9B0BA);
            if (cvui::button(background, innerX + innerW / 2, innerY + 3 * (rowH + paddingH), innerW / 2, rowH, "-")) {
                cv::Rect editArea = detector->getEditArea();
                detector->onMouse(cv::EVENT_MOUSEWHEEL, editArea.x + editArea.width / 2, editArea.y + editArea.height, -1);
            }

            double pupilHeight = detector->getPupilHeight(), pupilWidth = detector->getPupilWidth();
            double lastPupiHeight = pupilHeight, lastPupiWidth = pupilWidth;
            ui::sectionTitle(background, innerX, innerY + 4 * (rowH + paddingH) - 20, "Pupil");
            cvui::counter(background, innerX, innerY + 4 * (rowH + paddingH), &pupilWidth, 0.1, "Width: %.1f", innerW, rowH);
            if (lastPupiWidth != pupilWidth) detector->setPupilWidth(pupilWidth);

            cvui::counter(background, innerX, innerY + 5 * (rowH + paddingH), &pupilHeight, 0.1, "Height: %.1f", innerW, rowH);
            if (lastPupiHeight != pupilHeight) detector->setPupilHeight(pupilHeight);

            ui::sectionTitle(background, innerX, innerY + 6 * (rowH + paddingH) - 20, "View");
            cvui::checkbox(background, innerX, innerY + 6 * (rowH + paddingH), "Live preview", &isPreview, 0xCECECE, rowH / 2);
            if (prePreview != isPreview) {
                prePreview = isPreview;
                detector->setPreview(isPreview);
            }
        }
        else {
            ui::sectionTitle(background, innerX, innerY - 26, "Detection");

            cvui::text(background, innerX, innerY, "Edge Curl", 0.45, 0xE6E6E6);
            if (cvui::trackbar(background, innerX + padding, innerY + 16, innerW - padding, &medianBlurTrack, 0, 9, 0, "%.0Lf"))
                isEdit = true;

            cvui::text(background, innerX, innerY + 1 * (rowH + paddingH) + 8, "Morph Kernel", 0.45, 0xE6E6E6);
            if (cvui::trackbar(background, innerX + padding, innerY + 1 * (rowH + paddingH) + 24, innerW - padding, &morphKTrack, 0, 9, 1, "%.0Lf"))
                isEdit = true;

            cvui::text(background, innerX, innerY + 2 * (rowH + paddingH) + 16, "CLAHE clipLimit", 0.45, 0xE6E6E6);
            if (cvui::trackbar(background, innerX + padding, innerY + 2 * (rowH + paddingH) + 32, innerW - padding, &clipLimitTrack, 0, 9, 1, "%.0Lf"))
                isEdit = true;
        }

        const int footerY = settingY + settingHeight - padding - rowH * 5 - paddingH * 4;
        ui::sectionTitle(background, innerX, footerY - 26, "Actions");

        if (cvui::button(background, innerX, footerY + 0 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "Rotate -")) {
            if (detector->onKey('R')) {
                isEdit = true;
                refresh = true;
                angle+=0.05;
            }
        }

        if (cvui::button(background, innerX + innerW / 2, footerY + 0 * (rowH + paddingH), innerW / 2, rowH, "Rotate +")) {
            if (detector->onKey('R')) {
                isEdit = true;
                refresh = true;
                angle-=0.05;
            }
        }

        if (cvui::button(background, innerX, footerY + 1 * (rowH + paddingH), innerW / 2 - paddingH, rowH, "Scale +")) {
            detector->scaleCurrentContour(+1);
            isEdit = true;
        }

        if (cvui::button(background, innerX + innerW / 2, footerY + 1 * (rowH + paddingH), innerW / 2, rowH, "Scale -")) {
            detector->scaleCurrentContour(-1);
            isEdit = true;
        }

        cvui::checkbox(background, innerX, footerY + 2 * (rowH + paddingH), "Contour edit mode", &onlyContour, 0xCECECE, rowH / 2);
        if (preOnlyContour != onlyContour) {
            preOnlyContour = onlyContour;
            detector->setOnlyContour(onlyContour);
            isEdit = true;
            isPreview = false;
        }

        if (cvui::button(background, innerX, footerY + 3 * (rowH + paddingH), innerW, rowH, "Find next")) {
            if (detector->findNext()) {
                refresh = true;
                isEdit = true;
            }
        }

        if (cvui::button(background, innerX, footerY + 4 * (rowH + paddingH), innerW, rowH, "Export DXF")) {
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
