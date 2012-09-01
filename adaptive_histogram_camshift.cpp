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
#include "adaptive_histogram_camshift.h"
#include "colors.h"
#include "cv.h"
#include "highgui.h"
#include <cstdio>
#include <sstream>
#include <csignal>
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif //_USE_MATH_DEFINES
#include <math.h>

/* VELOCITY_WEIGHT controls how much to apply the velocity of the tracked
   object toward stretching the track window. */
#define VELOCITY_WEIGHT 1.0f
/* HIST_SCALE controls the max value of any histogram bin for the tracking
   histogram and all histograms weighted into it. */
#define HIST_SCALE 255.0
/* SMALLEST_AVG_HIST_BIN controls the smalles allowed average bin value for
   the tracking histogram and is used to prevent the zero histogram case.*/
#define SMALLEST_AVG_HIST_BIN 5.0
/* REACQUIRE_TIMEOUT_FRAMES controls how many frames to wait before searching
   the full image again when the track box size drops. */
#define REACQUIRE_TIMEOUT_FRAMES 15

///<summary>Flag indicating that application should run..</summary>
static bool s_shouldRun = true;

///<summary>Signal handler to terminate application on CTRL-C.</summary>
void OnSigInt(int)
{
  s_shouldRun = false;
}


enum { WindowName_ControlsGui, WindowName_BackProjection, WindowName_Histogram, };
static const char WindowNames[][256] = {"Adaptive Histogram Camshift Controls",
                                        "Camshift BackProjection",
                                        "Histogram",
};

enum { ControlName_VMin, ControlName_VMax, ControlName_SMin,
       ControlName_SBox, ControlName_AgingFactor, };
static const char ControlNames[][256] = {"vMin",
                                         "vMax",
                                         "sMin",
                                         "Sampling Box Size",
                                         "Histogram Aging Factor",
};

AdaptiveHistogramCamshift::InitParams::InitParams()
: histDims(96),
  vMin(32),
  vMax(256),
  sMin(16),
  sBox(8),
  showBackproject(true),
  showControlsGUI(true),
  showHistogram(true),
  histRanges()
{
  histRanges[0] = 0;
  histRanges[1] = 360.0f;
}

////////////////////////////////////////////////////////////////////////////////
// Static variables
bool AdaptiveHistogramCamshift::g_selectObject = false;
bool AdaptiveHistogramCamshift::g_initTracking = false;
int AdaptiveHistogramCamshift::g_selId = -1;
CvRect AdaptiveHistogramCamshift::g_selRect;
CvPoint AdaptiveHistogramCamshift::g_selOrigin;

namespace
{
inline
int NextTrackerId()
{
  static int s_id = 0;
  return ++s_id;
}

inline
std::string AppendWindowId(const char* baseName, int id)
{
  std::stringstream ss;
  ss << baseName << " - " << id;
  return ss.str();
}

inline
CvScalar hsv2rgb(float hue)
{
    static const int sector_data[][3]= {{0,2,1}, {1,2,0}, {1,0,2},
                                        {2,0,1}, {2,1,0}, {0,1,2}};
    // H' = H / 60
    hue *= 0.01666666666666666666666666666666667f;
    const int sector = cvFloor(hue);
    int p = cvRound(255*(hue - sector)) ^ (sector & 1 ? 255 : 0);
    int rgb[3];
    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;
    return cvScalar(rgb[2], rgb[1], rgb[0], 0);
}
} // end anonymous ns

AdaptiveHistogramCamshift::AdaptiveHistogramCamshift()
: m_id(NextTrackerId()),
  m_frameSize(),
  m_imgHue(NULL),
  m_imgMask(NULL),
  m_imgHSV(NULL),
  m_hist(NULL),
  m_histSubdiv(NULL),
  m_histTrackWnd(NULL),
  m_histDims(0),
  m_binWidth(0),
  m_histRanges(),
  m_histImg(NULL),
  m_ageRatio(15),
  m_imgBackproject(NULL),
  m_showBackproject(false),
  m_showControlsGUI(false),
  m_showHistogram(false),
  m_controlsGUIWndName(AppendWindowId(WindowNames[WindowName_ControlsGui], m_id)),
  m_backprojectWndName(AppendWindowId(WindowNames[WindowName_BackProjection], m_id)),
  m_histogramWndName(AppendWindowId(WindowNames[WindowName_Histogram], m_id)),
  m_tracking(false),
  m_initialized(false),
  m_trackWindow(),
  m_trackBox(),
  m_trackCompRect(),
  m_trackPosTwoFramesBack(cvPoint(0, 0)),
  m_trackAreaTwoFramesBack(0),
  m_velocity(cvScalarAll(0)),
  m_vMin(),
  m_vMax(),
  m_sMin(),
  m_sBox()
{}

AdaptiveHistogramCamshift::~AdaptiveHistogramCamshift()
{
  // Safe free buffers.
  Deinit();
  // Destroy windows.
  cvDestroyWindow(m_controlsGUIWndName.c_str());
  cvDestroyWindow(m_backprojectWndName.c_str());
  cvDestroyWindow(m_histogramWndName.c_str());
}

void AdaptiveHistogramCamshift::OnMouse(int event, int x, int y, int /*flags*/, void* param)
{
  // Update rect while mouse is down.
  if (g_selectObject)
  {
    g_selRect.x = MIN(x,g_selOrigin.x);
    g_selRect.y = MIN(y,g_selOrigin.y);
    g_selRect.width = g_selRect.x + CV_IABS(x - g_selOrigin.x);
    g_selRect.height = g_selRect.y + CV_IABS(y - g_selOrigin.y);

    g_selRect.x = MAX( g_selRect.x, 0 );
    g_selRect.y = MAX( g_selRect.y, 0 );
    g_selRect.width -= g_selRect.x;
    g_selRect.height -= g_selRect.y;
  }

  // Store or update rect.
  switch (event)
  {
    case CV_EVENT_LBUTTONDOWN:
      // Set which window is doing the selection.
      g_selId = *static_cast<int*>(param);
      // Init selection rect.
      g_selRect = cvRect(x, y, 0, 0);
      g_selOrigin = cvPoint(x, y);
      g_selectObject = true;
      break;
    case CV_EVENT_LBUTTONUP:
      // Set flag for initialization.
      g_selectObject = false;
      if ((g_selRect.width > 0) && (g_selRect.height > 0))
      {
        g_initTracking = true;
      }
      break;
  }
}

void AdaptiveHistogramCamshift::InitTrackWindow(const IplImage* img, const CvRect &selRect)
{
  // Do nothing for empty selection or when not initialized
  if ((0 == (selRect.height * selRect.width)) || !m_initialized)
  {
    return;
  }

  // Set origins
  m_imgHue->origin = m_imgMask->origin = m_imgHSV->origin = m_imgBackproject->origin = img->origin;

  // Get hue and mask
  cvCvtColor(img, m_imgHSV, CV_BGR2HSV);
  int _sMin = m_sMin, _vMax = m_vMax, _vMin = m_vMin;
  cvInRangeS(m_imgHSV,
             cvScalar(m_histRanges[0], _sMin, MIN(_vMin, _vMax), 0),
             cvScalar(m_histRanges[1], 255, MAX(_vMin, _vMax), 0), m_imgMask);
  cvSplit(m_imgHSV, m_imgHue, 0, 0, 0);

  // Initalize track histogram and window to selection
  float maxVal = 0.0f;
  cvSetImageROI(m_imgHue, selRect);
  cvSetImageROI(m_imgMask, selRect);
  cvCalcHist(&m_imgHue, m_hist, 0, m_imgMask);
  cvGetMinMaxHistValue(m_hist, 0, &maxVal, 0, 0);
  cvConvertScale(m_hist->bins, m_hist->bins, maxVal ? HIST_SCALE / maxVal : 0., 0);
  cvResetImageROI(m_imgHue);
  cvResetImageROI(m_imgMask);
  m_trackWindow = selRect;
  m_trackCompRect = selRect;

  // Create histogram image
  m_binWidth = m_histImg->width / m_histDims;
  for( int i = 0; i < m_histDims; i++ )
  {
    const double raw = cvGetReal1D(m_hist->bins,i) * (m_histImg->height / HIST_SCALE);
    int val = cvRound(raw);
    CvScalar color = hsv2rgb((i * m_histRanges[1] * 2.0f) / m_histDims);
    cvRectangle(m_histImg, cvPoint(i * m_binWidth, m_histImg->height),
                cvPoint( (i + 1) * m_binWidth, m_histImg->height - val),
                color, -1, 8, 0);
  }

  // Show histogram
  if (m_showHistogram)
  {
    cvShowImage(m_histogramWndName.c_str(), m_histImg);
  }

  // Init trackBox center for velocity calc
  m_trackBox.center.x = selRect.x + (selRect.width / 2.0f);
  m_trackBox.center.y = selRect.y + (selRect.height / 2.0f);
  m_trackBox.size.width = static_cast<float>(selRect.width);
  m_trackBox.size.height = static_cast<float>(selRect.height);
  m_trackBox.angle = 0.0f;

  // Compute camshift and adapt histogram
  const bool camShiftRes = ComputeCamshift(m_imgHue, m_imgMask);
  if (camShiftRes)
  {
    AdaptHistogram(m_imgHue, m_imgMask, NULL);
    // Set tracking flag
    m_tracking = true;
  }
}


void AdaptiveHistogramCamshift::Init(const CvSize& frameSize,
                                     const InitParams& initParams)
{
  // Get frame size
  m_frameSize = frameSize;

  // Load init params.
  m_histDims = initParams.histDims;
  m_vMin = initParams.vMin;
  m_vMax = initParams.vMax;
  m_sMin = initParams.sMin;
  m_sBox = initParams.sBox;
  m_histRanges[0] = initParams.histRanges[0] * 0.5f;
  m_histRanges[1] = initParams.histRanges[1] * 0.5f;

  // Create histograms
  float* histRanges[2] = { &m_histRanges[0], &m_histRanges[1] };
  m_hist = cvCreateHist(1, &m_histDims, CV_HIST_ARRAY, histRanges, 1);
  m_histSubdiv = cvCreateHist(1, &m_histDims, CV_HIST_ARRAY, histRanges, 1);
  m_histTrackWnd = cvCreateHist(1, &m_histDims, CV_HIST_ARRAY, histRanges, 1);

  // Create images
  m_imgHue = cvCreateImage(m_frameSize, 8, 1);
  m_imgMask = cvCreateImage(m_frameSize, 8, 1);
  m_imgHSV = cvCreateImage(m_frameSize, 8, 3);
  m_imgBackproject = cvCreateImage(m_frameSize, 8, 1);
  cvZero(m_imgBackproject);

  // Create histogram image
  m_histImg = cvCreateImage( m_frameSize, 8, 3 );
  cvZero( m_histImg );

  // Show controlsGUI, backproject, histogram
  if (initParams.showControlsGUI)
  {
    ShowControlsGUI();
  }
  if (initParams.showBackproject)
  {
    ShowBackproject();
  }
  if (initParams.showHistogram)
  {
    ShowHistogram();
  }

  // Set initialized flag
  m_initialized = true;
}

void AdaptiveHistogramCamshift::Deinit()
{
  // Free buffers that enable tracking.
  StopTracking();

  cvReleaseImage(&m_imgHue);
  cvReleaseImage(&m_imgMask);
  cvReleaseImage(&m_imgHSV);
  cvReleaseImage(&m_imgBackproject);
  cvReleaseImage(&m_histImg);

  cvReleaseHist(&m_hist);
  cvReleaseHist(&m_histSubdiv);
  cvReleaseHist(&m_histTrackWnd);
  m_initialized = false;
}

void AdaptiveHistogramCamshift::PresentOutput(IplImage *img)
{
  // Draw track box
  if (m_tracking)
  {
    const bool trackBoxHasArea = (m_trackBox.size.width * m_trackBox.size.height) > 0;
    if (trackBoxHasArea)
    {
      cvEllipseBox(img, m_trackBox, colors[CYAN], 3, CV_AA, 0);
    }
  }
  // Show output
  cvShowImage(m_controlsGUIWndName.c_str(), img);
  // Show backproject
  if (m_showBackproject)
  {
    ShowBackproject();
  }

}

void AdaptiveHistogramCamshift::ShowControlsGUI()
{
  cvNamedWindow(m_controlsGUIWndName.c_str(), 1);
  cvNamedWindow("Trackbars", 1);
  if (!m_showControlsGUI)
  {
    cvMoveWindow(m_controlsGUIWndName.c_str(), m_frameSize.width + 10, 0);
    cvCreateTrackbar(ControlNames[ControlName_VMin], "Trackbars", &m_vMin, 256, 0);
    cvCreateTrackbar(ControlNames[ControlName_VMax], "Trackbars", &m_vMax, 256, 0);
    cvCreateTrackbar(ControlNames[ControlName_SMin], "Trackbars", &m_sMin, 256, 0);
    cvCreateTrackbar(ControlNames[ControlName_SBox], "Trackbars", &m_sBox, 64, 0);
    cvCreateTrackbar(ControlNames[ControlName_AgingFactor], "Trackbars", &m_ageRatio, 100, 0);
    cvSetMouseCallback(m_controlsGUIWndName.c_str(), &AdaptiveHistogramCamshift::OnMouse, &m_id);
  }
  m_showControlsGUI = true;
}

void AdaptiveHistogramCamshift::HideControlsGUI()
{
  cvDestroyWindow(m_controlsGUIWndName.c_str());
  m_showControlsGUI = false;
}

void AdaptiveHistogramCamshift::ShowBackproject()
{
  cvNamedWindow(m_backprojectWndName.c_str(), 1);
  if (!m_showBackproject)
  {
    cvMoveWindow(m_backprojectWndName.c_str(), (2 * m_frameSize.width) + 20, 0);
  }
  cvShowImage(m_backprojectWndName.c_str(), m_imgBackproject);
  m_showBackproject = true;
}

void AdaptiveHistogramCamshift::HideBackproject()
{
  cvDestroyWindow(m_backprojectWndName.c_str());
  m_showBackproject = false;
}

void AdaptiveHistogramCamshift::ShowHistogram()
{
  cvNamedWindow(m_histogramWndName.c_str(), 1);    
  if (!m_showHistogram)
  {
    cvMoveWindow(m_histogramWndName.c_str(), (2 * m_frameSize.width) + 20, m_frameSize.height + 55);
  }
  cvShowImage(m_histogramWndName.c_str(), m_histImg);
  m_showHistogram = true;
}

void AdaptiveHistogramCamshift::HideHistogram()
{
  cvDestroyWindow(m_histogramWndName.c_str());
  m_showHistogram = false;
}

bool AdaptiveHistogramCamshift::ToggleShowBackproject()
{
  if (!m_showBackproject)
  {
    ShowBackproject();
  }
  else
  {
    HideBackproject();
  }
  return m_showBackproject;
}

bool AdaptiveHistogramCamshift::ToggleShowHistogram()
{
  if (!m_showHistogram)
  {
    ShowHistogram();
  }
  else
  {
    HideHistogram();
  }
  return m_showHistogram;
}

void AdaptiveHistogramCamshift::ProcessFrame(const IplImage* img, IplImage** out)
{
  // Check image size, type matches.
  const bool outMatches = (NULL != *out) &&
                          (img->width == (*out)->width) &&
                          (img->height == (*out)->height) &&
                          (img->depth == (*out)->depth) &&
                          (img->nChannels == (*out)->nChannels);
  if (!outMatches)
  {
    cvReleaseImage(out);
    *out = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
  }
  cvCopy(img, *out);

  // Check for selection
  if (g_selId == m_id)
  {
    // DEBUG selection
    //printf("AdaptiveHistCamshift %d is g_selId.\n", m_id);

    // Init already done, so m_frameSize should be set
    g_selRect.width = std::min(g_selRect.width, m_frameSize.width);
    g_selRect.height = std::min(g_selRect.height, m_frameSize.height);

    // Check if selecting
    if (g_selectObject)
    {
      // DEBUG selection
      //printf("AdaptiveHistCamshift %d detects in progress selection.\n", m_id);

      // Draw selection box
      if ((g_selRect.width > 0) && (g_selRect.height > 0))
      {
        cvSetImageROI(*out, g_selRect);
        cvXorS(*out, cvScalarAll(255), *out, 0);
        cvResetImageROI(*out);
      }
    }
    // Check if time to init
    else if (g_initTracking)
    {
      // DEBUG selection
      //printf("AdaptiveHistCamshift %d detects time to init.\n", m_id);

      InitTrackWindow(img, g_selRect);
      g_initTracking = false;
      g_selId = -1;
    }
  }

  // Check if not initialized
  if (!m_tracking)
  {
    // Show input image and return
    PresentOutput(*out);
  }
  else
  {
    // Get hue and mask
    cvCvtColor(img, m_imgHSV, CV_BGR2HSV);
    cvInRangeS(m_imgHSV,
               cvScalar(m_histRanges[0], m_sMin, std::min(m_vMin, m_vMax), 0),
               cvScalar(m_histRanges[1], 255, std::max(m_vMin, m_vMax), 0), m_imgMask);
    cvSplit(m_imgHSV, m_imgHue, 0, 0, 0);

    m_trackWindow = m_trackCompRect;
//    // Draw initial search window
//    cvRectangle( img,
//    cvPoint(m_trackWindow.x, m_trackWindow.y),
//    cvPoint(m_trackWindow.x + m_trackWindow.width, m_trackWindow.y + m_trackWindow.height),
//    colors[ORANGE], 1 );

    // Grow track window in direction of velocity
    // Compute velocity (last frame minus two frames back)
    const float trackBoxArea = m_trackBox.size.width * m_trackBox.size.height;
    m_velocity = cvScalar(m_trackBox.center.x - m_trackPosTwoFramesBack.x,
                          m_trackBox.center.y - m_trackPosTwoFramesBack.y,
                          trackBoxArea - m_trackAreaTwoFramesBack);

    // DEBUG velocity
    //printf("wnd vcty: (%f, %f, %f)\n", m_velocity.val[0], m_velocity.val[1], m_velocity.val[2]);

    // Draw velocity
    CvPoint vPt1 = cvPoint(static_cast<int>(m_trackBox.center.x),
                           static_cast<int>(m_trackBox.center.y));
    CvPoint vPt2 = cvPoint(static_cast<int>(vPt1.x + m_velocity.val[0]),
                           static_cast<int>(vPt1.y + m_velocity.val[1]));
    cvLine(*out, vPt1, vPt2, colors[PURPLE], 4);
    cvCircle(*out, vPt2, 3, colors[RED], CV_FILLED); 
    {
      const int dx = static_cast<int>(m_velocity.val[0] * VELOCITY_WEIGHT);
      const int l = dx > 0 ? m_trackWindow.x : m_trackWindow.x + dx;
      const int r = l + m_trackWindow.width + std::abs(dx);
      m_trackWindow.x = std::max(l, 0);
      m_trackWindow.width = std::min(r - l, m_frameSize.width - m_trackWindow.x);
    }
    {
      const int dy = static_cast<int>(m_velocity.val[1] * VELOCITY_WEIGHT);
      const int t = dy > 0 ? m_trackWindow.y : m_trackWindow.y + dy;
      const int b = t + m_trackWindow.height + std::abs(dy);
      m_trackWindow.y = std::max(t, 0);
      m_trackWindow.height = std::min(b - t, m_frameSize.height - m_trackWindow.y);
    }

    // DEBUG enhanced window point
    //printf("wnd init: (%d, %d, %d, %d)\n",
    //m_trackWindow.x, m_trackWindow.y,
    //m_trackWindow.width, m_trackWindow.height);

    // Draw enhanced search window
    cvRectangle(*out,
                cvPoint(m_trackWindow.x, m_trackWindow.y),
                cvPoint(m_trackWindow.x + m_trackWindow.width,
                        m_trackWindow.y + m_trackWindow.height),
                colors[PURPLE], 3);

    // Now compute camshift and adapt histogram
    const bool camShiftRes = ComputeCamshift(m_imgHue, m_imgMask);
    if (camShiftRes)
    {
      AdaptHistogram(m_imgHue, m_imgMask, *out);
    }
    else
    {
      m_tracking = false;
    }
    // Show output
    PresentOutput(*out);
  }
}

bool AdaptiveHistogramCamshift::ComputeCamshift(const IplImage* hue, const IplImage* mask)
{
  // Compute backproject
  cvCalcBackProject(&hue, m_imgBackproject, m_hist);
  cvAnd(m_imgBackproject, mask, m_imgBackproject, 0);

  // Init velocity
  m_trackPosTwoFramesBack = cvPoint(static_cast<int>(m_trackBox.center.x),
                                    static_cast<int>(m_trackBox.center.y));
  m_trackAreaTwoFramesBack = m_trackBox.size.width * m_trackBox.size.height;

  // DEBUG track window area
  //printf("track wnd area: %f\n", m_trackBox.size.width * m_trackBox.size.height);

  // Compute camshift this frame
  CvConnectedComp trackComp;
  assert((m_trackWindow.height > 0) && (m_trackWindow.width > 0));
  CvBox2D trackBox;
  const int camShiftRes = cvCamShift(m_imgBackproject,
                                     m_trackWindow,
                                     cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1),
                                     &trackComp,
                                     &trackBox);
  if (camShiftRes >= 0)
  {
    m_trackBox = trackBox;
    m_trackCompRect = trackComp.rect;
    return true;
  }
  else
  {
    return false;
  }
}

void AdaptiveHistogramCamshift::AdaptHistogram(IplImage* hue, IplImage* mask, IplImage* out)
{
  // Prepare to analyze new track window
  const CvRect& updateHistRect = m_trackCompRect;

  // Now subdivide the track window and sum all pixels in each square
  // subdivision of size m_sBox using pixel values given by the
  // current hustogram.  When we are done, we will normalize the sum
  // from each subdivision and then be able to tell roughly how
  // similar each subregion is to the track histogram.

  // Make sure box size is greater than or equal to 2, and protect from
  // changes made in GUI
  const int sBox = std::max(m_sBox, 2);
  if (sBox != m_sBox)
  {
    m_sBox = sBox;
    cvSetTrackbarPos(ControlNames[ControlName_SBox], m_controlsGUIWndName.c_str(), sBox);
  }
  std::vector<float> subdivs;
  int numRows, numCols;
  SubdivideSumTrackWnd(sBox, &numRows, &numCols, &subdivs);

  // Check window is less than sBox in size.
  if (subdivs.empty())
  {
    return;
  }

  // Histogram for image subdivisions
  for (int i = 0; i < m_histDims; ++i)
  {
    float *bin = cvGetHistValue_1D(m_histTrackWnd, i);
    *bin = 0;
  }

  // Find the max vlaue of the subdivisions
  float maxVal = *std::max_element(subdivs.begin(), subdivs.end());;

  // DEBUG maxval
  //printf("maxVal: %f\n", maxVal);

  // Step through all subdivisions and weight them into a new histogram
  // if they are greater than minVal.  The weight function is r^2 where
  // r is the ratio of the subdivision value to the max subdivision
  // value.
  if (maxVal > 0)
  {
    float minVal = maxVal * 0.125f;
    float* subdivsCur = &subdivs[0];
    for (int i = 0; i < numRows; ++i)
    {
      for (int j = 0; j < numCols; ++j, ++subdivsCur)
      {
        // Create a box around this area
        CvPoint roiP1 = cvPoint(updateHistRect.x + sBox * j, updateHistRect.y + sBox * i);
        CvPoint roiP2 = cvPoint(roiP1.x + sBox, roiP1.y + sBox);

        if (*subdivsCur < minVal)
        {
          if(*subdivsCur > (minVal * 0.0625))
          {
            // Get ratio to max subdivision
            float ratioMaxSubdiv = *subdivsCur / maxVal;
            // Get color of surrounding box
            CvScalar boxColor = colors[GREEN];
            for (int colorInd = 0; colorInd < 3; ++colorInd)
            {
              boxColor.val[colorInd] *= ratioMaxSubdiv;
            }
            // Draw the box (darker green means less weight)
            if (out)
            {
              cvRectangle(out, roiP1, roiP2, boxColor, 1);
            }
          }
          else
          {
            // Draw a red box around this subdivision since it is not used
            if (out)
            {
              cvRectangle(out, roiP1, roiP2, colors[RED], 1 );
            }
          }
        }
        else
        {
          // Get ratio to max subdivision
          float ratioMaxSubdiv = *subdivsCur / maxVal;
          // Get weight into histogram of track window
          float weightVal = ratioMaxSubdiv * ratioMaxSubdiv;

          // DEBUG weights
          //printf("w %d: %f\t", j, weightVal);

          // Get color of surrounding box
          CvScalar boxColor = colors[GREEN];
          for (int colorInd = 0; colorInd < 3; ++colorInd)
          {
            boxColor.val[colorInd] *= ratioMaxSubdiv;
          }
          // Draw the box (darker green means less weight)
          if (out)
          {
            cvRectangle(out, roiP1, roiP2, boxColor, 1);
          }

          // Weight this subdivision into the histogram for the track window
          CvRect thisSubdivRect = cvRect(roiP1.x, roiP1.y, sBox, sBox);
          cvSetImageROI(hue, thisSubdivRect);
          cvSetImageROI(mask, thisSubdivRect);
          cvCalcHist(&hue, m_histSubdiv, 0, mask);
          cvResetImageROI(hue);
          cvResetImageROI(mask);

          // Weight this into the track window histogram
          for (int binNum = 0; binNum < m_histDims; ++binNum)
          {
            float* thisBin = cvGetHistValue_1D(m_histTrackWnd, binNum);
            *thisBin *= (1.0f - weightVal);
            *thisBin += static_cast<float>(cvGetReal1D(m_histSubdiv->bins, binNum)) * weightVal;
          }
        }
      }
      // DEBUG weights
      //printf("\n");
    }
    // DEBUG weights
    //printf("\n");
  }

  // DEBUG histograms
  //printf("Wnd BEFORE WT\n");
  //for( int i = 0; i < m_histDims; i++ ) {
  //  float *bin = cvGetHistValue_1D( m_histTrackWnd, i );
  //  printf("%2d: %3.1f  ", i, *bin);
  //  if( 0 == (i+1) % 8 ) {
  //    printf("\n");
  //  }
  //}


  // Now scale track window histogram to tracking histogram scale
  float trackWndHistMaxVal;
  cvGetMinMaxHistValue(m_histTrackWnd, 0, &trackWndHistMaxVal, 0, 0);
  cvConvertScale(m_histTrackWnd->bins, m_histTrackWnd->bins,
                 trackWndHistMaxVal ? HIST_SCALE / trackWndHistMaxVal : 0., 0);

  // Use aging to weight track window histogram into tracking histogram
  float averageBin = 0;
  for (int binNum = 0; binNum < m_histDims; ++binNum)
  {
    float* thisBin = cvGetHistValue_1D(m_hist, binNum);
    *thisBin *= (1.0f - (m_ageRatio / 100.0f));
    *thisBin += static_cast<float>(cvGetReal1D(m_histTrackWnd->bins, binNum)) *
                (m_ageRatio / 100.0f);
    averageBin += *thisBin;
  }
  averageBin /= m_histDims;

  // DEBUG average bin
  //printf("Avg bin: %f.\n", averageBin);

  // See if this histogram is dying
  //if( averageBin < SMALLEST_AVG_HIST_BIN ) {
  //    cvConvertScale( m_hist->bins, m_hist->bins, SMALLEST_AVG_HIST_BIN / averageBin, 0 );

  // DEBUG averageBin
  //printf("Hist saved for average bin: %f\n", averageBin);
  //}

  // DEBUG track hist
  //printf("Track\n");
  //for( int i = 0; i < m_histDims; i++ ) {
  //    float *bin = cvGetHistValue_1D( m_hist, i );
  //    printf("%2d: %3.1f  ", i, *bin);
  //    if( 0 == (i+1) % 8 ) {
  //        printf("\n");
  //    }
  //}
  //printf("\n");

  // Now compute histogram image
  cvZero(m_histImg);
  for (int i = 0; i < m_histDims; ++i)
  {
    const double raw = cvGetReal1D(m_hist->bins, i) * (m_histImg->height / HIST_SCALE);
    const int val = cvRound(raw);
    CvScalar color = hsv2rgb((i * m_histRanges[1]) / m_histDims);
    cvRectangle(m_histImg,
                cvPoint(i * m_binWidth, m_histImg->height),
                cvPoint((i + 1) * m_binWidth, m_histImg->height - val),
                color, -1, 8, 0);
  }

  // Show histogram
  if (m_showHistogram)
  {
    ShowHistogram();
  }
}

void AdaptiveHistogramCamshift::SubdivideSumTrackWnd(int sBox,
                                                     int* numRows, int* numCols,
                                                     std::vector<float>* subdivs)
{
  assert(numCols && subdivs);
  // NOTE: This is designed for GRAYSCALE images.

  // Subdivide found window
  *numRows = static_cast<int>(m_trackCompRect.height / sBox);
  *numCols = static_cast<int>(m_trackCompRect.width / sBox);

  // Check if we can subdivide
  const int numBoxes = *numRows * *numCols;
  if (0 == numBoxes)
  {
    subdivs->clear();
    return;
  }

  // Create window subdivisions
  subdivs->assign(numBoxes, 0);

  // The following are used to debug by painting a gradient over the
  // region that is sampled within the track window.  Its direction
  // from dark-to-light displays the direction in which the window
  // is subdivided and sampled.
  // **NOTE: only use one at a time.
//#define DEBUG_COLOR_ROW
//#define DEBUG_COLOR_COL
#ifdef DEBUG_COLOR_ROW
  unsigned int debugColor = 0;
#endif
#ifdef DEBUG_COLOR_COL
  unsigned int debugColor = 0;
#endif

  // Sum mask values in windows. Only update in the ellipse box.
  const float angleRad = (m_trackBox.angle * static_cast<float>(M_PI)) / 180.0f;
  const float cosThetaTb = cos(angleRad);
  const float sinThetaTb = sin(angleRad);
  const int rowOffset = m_imgBackproject->widthStep * m_trackCompRect.y;
  uchar* rowPtr = reinterpret_cast<uchar*>(m_imgBackproject->imageData + rowOffset) +
                  m_trackCompRect.x;
  float* subdiv = &subdivs->at(0);
  int y = m_trackCompRect.y;
  for (int r = 0; r < *numRows; ++r)
  {
    // Subdivision box rows
    for (int rPx = 0; rPx < sBox; ++rPx, ++y)
    {
      int x = m_trackCompRect.x;
      uchar* rowCur = rowPtr;
      float* subdivCur = subdiv;
  #ifdef DEBUG_COLOR_COL
      debugColor = 0;
  #endif
      // Step through row and put pixels in their bins
      for (int c = 0; c < *numCols; ++c, ++subdivCur)
      {
        for (int cPx = 0; cPx < sBox; ++cPx, ++rowCur, ++x)
        {
          const float vx =  x - m_trackBox.center.x;
          const float vy = -y + m_trackBox.center.y;
          const float xTrans = (vx * cosThetaTb) + (vy * -sinThetaTb);
          const float yTrans = (vx * sinThetaTb) + (vy *  cosThetaTb);
          const float xT_div_a = xTrans / (0.5f * m_trackBox.size.width);
          const float yT_div_b = yTrans / (0.5f * m_trackBox.size.height);
          const float f_xy = (xT_div_a * xT_div_a) + (yT_div_b * yT_div_b);
          const bool inEllipse = f_xy <= 1.0f;
          if (inEllipse)
          {
            *subdivCur += static_cast<int>(*rowCur);
          }
  #ifdef DEBUG_COLOR_COL
          *rowCur = debugColor++;
          debugColor %= 255;
  #elif DEBUG_COLOR_ROW
          *rowCur = debugColor;
  #endif
        }
      }
      // Jump to next scanline
      rowPtr += m_imgBackproject->widthStep;
  #ifdef DEBUG_COLOR_ROW
      debugColor++;
      debugColor %= 255;
  #endif
    }
    // Jump to next set of bins
    subdiv += *numCols;
  }
}

const CvRect& AdaptiveHistogramCamshift::TrackWindow() const
{
  return m_trackWindow;
}

const CvBox2D& AdaptiveHistogramCamshift::TrackBox() const
{
  return m_trackBox;
}

const CvHistogram& AdaptiveHistogramCamshift::TrackHistogram() const
{
  return *m_hist;
}

int AdaptiveHistogramCamshift::TrackHistogramDims() const
{
  return m_histDims;
}

const std::string& AdaptiveHistogramCamshift::Name() const
{
  return m_controlsGUIWndName;
}

const IplImage* AdaptiveHistogramCamshift::Hue() const
{
  return m_imgHue;
}

void AdaptiveHistogramCamshift::StopTracking()
{
  m_tracking = false;
  m_trackWindow = cvRect(0, 0, 0, 0);
}

bool AdaptiveHistogramCamshift::IsTracking() const
{
  return m_tracking;
}

CvScalar AdaptiveHistogramCamshift::ComputeNormalizedVelocity() const
{
  // Return normalized velocity
  return cvScalar(m_velocity.val[0] / m_frameSize.width,
                  m_velocity.val[1] / m_frameSize.height,
                  m_velocity.val[2] / (m_frameSize.width * m_frameSize.height),
                  0 );
}

int main(int /*argc*/, char** /*argv*/)
{
  enum { Width = 320, };
  enum { Height = 240, };
  std::signal(SIGINT, &OnSigInt);
  try
  {
    // Create image grab loop, pass to tracker. There is a BUG in OpenCV that
    // leads to a crash when no camera device is available. The crash is caused
    // by an access violation in highgui.
    cv::VideoCapture capture;
    capture.open(0);
    if (!capture.isOpened())
    {
      return 1;
    }
    cv::namedWindow("Raw");
    capture.set(CV_CAP_PROP_FRAME_WIDTH, Width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, Height);
    AdaptiveHistogramCamshift tracker;
    tracker.Init(cvSize(static_cast<int>(capture.get(CV_CAP_PROP_FRAME_WIDTH)),
                        static_cast<int>(capture.get(CV_CAP_PROP_FRAME_HEIGHT))),
                 AdaptiveHistogramCamshift::InitParams());
    IplImage* out = NULL;
    cvNamedWindow("Out");
    while (s_shouldRun)
    {
      const bool grabbed = capture.grab();
      if (!grabbed)
      {
        break;
      }
      cv::Mat frame;
      capture.retrieve(frame);
      cv::imshow("Raw", frame);
      IplImage frameIpl = frame;
      tracker.ProcessFrame(&frameIpl, &out);
      if (out)
      {
        cvShowImage("Out", out);
      }
      cv::waitKey(1);
    }
    if (out)
    {
      cvReleaseImage(&out);
    }
    capture.release();
    return 0;
  }
  catch (const cv::Exception& e)
  {
    std::cerr << e.what();
    return 1;
  }
}
