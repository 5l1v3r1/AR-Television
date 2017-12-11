////////////////////////////////////////////////////////////
/// AR Television
/// Copyright(c) 2017 Carnegie Mellon University
/// Licensed under The MIT License[see LICENSE for details]
/// Written by Kai Yu, Zhongxu Wang, Ruoyuan Zhao, Qiqi Xiao
////////////////////////////////////////////////////////////
#pragma once

#ifndef CVUTILS_H
#define CVUTILS_H

#include <utility>
#include <vector>
#include <string>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>

#include <common/ErrorCodes.h>

#ifdef _WIN32
#ifdef COMMON_EXPORTS
#define COMMON_API __declspec(dllexport)
#else
#define COMMON_API __declspec(dllimport)
#endif
#else
#define COMMON_API
#endif

namespace ar {
    /// The interface FrameStream specifies the routine of frame stream classes.
    ///	Subclasses should retrieve video content from various sources, and return
    ///	a required frame on call of the nextFrame method.
    class COMMON_API FrameStream {
    public:
        virtual ERROR_CODE NextFrame(cv::Mat &outputBuf) = 0;
    };

    class COMMON_API NaiveFrameStream : public FrameStream {
        cv::VideoCapture cap_;
    public:
        inline ERROR_CODE Open(const char *videoPath) { cap_.open(videoPath); return AR_SUCCESS; }

        inline ERROR_CODE NextFrame(cv::Mat &outputBuf) override { cap_ >> outputBuf; return AR_SUCCESS; }
    };

    class COMMON_API RealtimeLocalVideoStream : public FrameStream {
        cv::VideoCapture cap_;
        double fps_{};
        std::chrono::steady_clock::time_point start_time_;
        int frame_cnt_{};
    public:
        inline RealtimeLocalVideoStream() { Restart(); }

        void Restart();

        ERROR_CODE Open(const char *videoPath);

        ERROR_CODE NextFrame(cv::Mat &outputBuf) override;
    };

    class COMMON_API InterestPointsTracker {
    public:

        InterestPointsTracker(cv::Ptr<cv::Feature2D> detector,
                              cv::Ptr<cv::DescriptorMatcher> matcher);

        void GenKeypointsDesc(const cv::Mat &frame,
                              std::vector<cv::KeyPoint> &keypoints,
                              cv::Mat &descriptors);

        std::vector<std::pair<int, int>> MatchKeypoints(const cv::Mat &descriptors1,
                                                        const cv::Mat &descriptors2);

    protected:
        /// Nearest-neighbour matching ratio.
        const double NN_MATCH_RATIO = 0.8f;
        cv::Ptr<cv::Feature2D> detector_;
        cv::Ptr<cv::DescriptorMatcher> matcher_;
    };
}

#endif //CVUTILS_H
