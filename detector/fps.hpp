#include <deque>
#include <algorithm>
#include <opencv2/opencv.hpp>

class FPS
{
#define NUM_TO_MEAN 12
public:
    FPS::FPS(int num_average = 50)
    {
        _num_average = num_average;
        memset(szFps, 0, sizeof(szFps));
    }

    FPS::~FPS()
    {
    }

    void tic()
    {
        _startTick = cv::getTickCount();
    }

    int64_t mean_diff() {
        int64_t sum = 0;
        for (size_t i = _diffs.size() <= NUM_TO_MEAN ? std::min(size_t(NUM_TO_MEAN), _diffs.size()) : NUM_TO_MEAN; i < _diffs.size(); i++) {
            sum += _diffs[i];
        }
        return sum / _diffs.size();
    }

    void toc()
    {
        _endTick = cv::getTickCount();
        _diff = (_endTick - _startTick);
        _diffs.push_back(_diff);

        if (_diffs.size() > _num_average) {
            _diffs.pop_front();
        }
    }

    // elapsed time in seconds
    double elapsed()
    {
        return mean_diff() / cv::getTickFrequency();
    }

    double fps()
    {
        int64_t d = mean_diff();
        if (d == 0)
            return 0;
        return cv::getTickFrequency() / d;
    }

    cv::String toString()
    {
        sprintf_s(szFps, sizeof(szFps), "%dms : %3.2ffps", cvRound(1000 * elapsed()), fps());
        return szFps;
    }

private:
    int64_t _startTick;
    int64_t _endTick;
    int64_t _diff;
    std::deque<int64_t> _diffs;
    int _num_average;
    char szFps[_MAX_PATH];
};
