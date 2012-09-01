/* Copyright (C) 2011-2012 Brandon L. Reiss
   brandon@brandonreiss.com
   
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   
   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.
   
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/
#ifndef ADAPTIVE_HISTOGRAM_CAMSHIFT
#define ADAPTIVE_HISTOGRAM_CAMSHIFT

#include "cv.h"
#include <string>
#include <vector>

///<summary>A color tracker using OpenCV camshift and an update scheme to
///track a patch in the input image. The supplied parameters vMin, vMax,
///and sMin control the behavior of the color matching while the parameters
///of sBox and the histogram update control how new color data are integrated
///into the currently tracked histogram.</summary>
class AdaptiveHistogramCamshift
{
public:
  ///<summary>Initialization parameters.</summary>
  struct InitParams
  {
    ///<summary>Set default parameter values.</summary>
    InitParams();

    int histDims;
    int vMin;
    int vMax;
    int sMin;
    int sBox;
    bool showBackproject;
    bool showControlsGUI;
    bool showHistogram;
    float histRanges[2];
  };

  AdaptiveHistogramCamshift();
  ~AdaptiveHistogramCamshift();

  void Init(const CvSize& frameSize, const InitParams& initParams);
  void Deinit();
  void InitTrackWindow(const IplImage* img, const CvRect &selRect);
  void ProcessFrame(const IplImage* img, IplImage** out);

  void SetSubdivBoxSize(int boxSize);
  void ShowBackproject();
  void HideBackproject();
  void ShowControlsGUI();
  void HideControlsGUI();
  void ShowHistogram();
  void HideHistogram();
  bool ToggleShowBackproject();
  bool ToggleShowHistogram();

  bool IsTracking() const;
  void StopTracking();

  const CvRect& TrackWindow() const;
  const CvBox2D& TrackBox() const;
  const CvHistogram& TrackHistogram() const;
  int TrackHistogramDims() const;
  const IplImage* Hue() const;
  const IplImage* BackProjection() const;
  const std::string& Name() const;

  ///<summary>Get the velocity.</summary>
  ///<remarks><para>Normalizes velocity by the size of the image so that the
  ///implementation is safe from changes in image size. Recommended to use an
  ///aging mechanism on this value since it is very noisy. Something like
  ///velocity = (v_prev * .75) + (v_now * 0.25) should be
  ///okay.</para></remarks>
  CvScalar ComputeNormalizedVelocity() const;

private:
  bool ComputeCamshift(const IplImage* hue, const IplImage* mask);
  void AdaptHistogram(IplImage* hue, IplImage* mask, IplImage* out);
  void SubdivideSumTrackWnd(int sBox,
                            int* numRows, int* numCols,
                            std::vector<float>* subdivs);
  void PresentOutput(IplImage *img);
  static void OnMouse(int event, int x, int y, int flags, void* param);

  int m_id;
  CvSize m_frameSize;

  IplImage* m_imgHue;
  IplImage* m_imgMask;
  IplImage* m_imgHSV;

  CvHistogram* m_hist;
  CvHistogram* m_histSubdiv;
  CvHistogram* m_histTrackWnd;
  int m_histDims;
  int m_binWidth;
  float m_histRanges[2];
  IplImage* m_histImg;

  ///<summary>Controls the amount of the new track window histogram to weight
  ///into the current tracking histogram.</summary>
  int m_ageRatio;

  IplImage* m_imgBackproject;

  bool m_showBackproject;
  bool m_showControlsGUI;
  bool m_showHistogram;

  std::string m_controlsGUIWndName;
  std::string m_backprojectWndName;
  std::string m_histogramWndName;

  bool m_tracking;
  bool m_initialized;

  CvRect m_trackWindow;
  CvBox2D m_trackBox;
  CvRect m_trackCompRect;
  CvPoint m_trackPosTwoFramesBack;
  float m_trackAreaTwoFramesBack;
  CvScalar m_velocity;

  bool m_reaquire;

  // Camshift parameters.
  int m_vMin;
  int m_vMax;
  int m_sMin;
  int m_sBox;

  static CvRect g_selRect;
  static int g_selId;
  static bool g_selectObject;
  static bool g_initTracking;
  static CvPoint g_selOrigin;
};

#endif  //ADAPTIVE_HISTOGRAM_CAMSHIFT
