// FILE:          MMDevice.h
 // PROJECT:       Micro-Manager
 // SUBSYSTEM:     MMDevice - Device adapter kit
 //-----------------------------------------------------------------------------
 // DESCRIPTION:   The interface to the Micro-Manager devices. Defines the
 //                plugin API for all devices.
 //
 // AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 06/08/2005
 //
 // COPYRIGHT:     University of California, San Francisco, 2006-2014
 //                100X Imaging Inc, 2008
 //
 // LICENSE:       This file is distributed under the BSD license.
 //                License text is included with the source distribution.
 //
 //                This file is distributed in the hope that it will be useful,
 //                but WITHOUT ANY WARRANTY; without even the implied warranty
 //                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 //
 //                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 //                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 //                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.
 
 
 // Header version
 // If any of the class definitions changes, the interface version
 // must be incremented
 #define DEVICE_INTERFACE_VERSION 70
 
 
 // N.B.
 //
 // Never add parameters or return values that are not POD
 // (http://stackoverflow.com/a/146454) to any method of class Device and its
 // derived classes defined in this file. For example, a std::string parameter
 // is not acceptable (use const char*). This is to prevent inter-DLL
 // incompatibilities.
 
 
 #pragma once
 #ifndef MMMMDEVICE_H
 #define MMMMDEVICE_H
 
 #include "MMDeviceConstants.h"
 #include "DeviceUtils.h"
 #include "ImageMetadata.h"
 #include "DeviceThreads.h"
 #include <string>
 #include <cstring>
 #include <climits>
 #include <cstdlib>
 #include <vector>
 #include <sstream>
 
 
 #ifdef MODULE_EXPORTS
 #   ifdef _MSC_VER
 #      define MM_DEPRECATED(prototype) __declspec(deprecated) prototype
 #   elif defined(__GNUC__)
 #      define MM_DEPRECATED(prototype) prototype __attribute__((deprecated))
 #   else
 #      define MM_DEPRECATED(prototype) prototype
 #   endif
 #else
 #   define MM_DEPRECATED(prototype) prototype
 #endif
 
 
 #ifdef WIN32
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>
 
    typedef HMODULE HDEVMODULE;
 #else
    typedef void* HDEVMODULE;
 #endif
 
 #include "FixSnprintf.h"
 
 
 class ImgBuffer;
 
 
 
 namespace MM {
 
    // forward declaration for the MMCore callback class
    class Core;
 
    class MMTime
    {
       public:
          MMTime(double uSecTotal = 0.0)
          {
             sec_ = (long) (uSecTotal / 1.0e6);
             uSec_ = (long) (uSecTotal - sec_ * 1.0e6);
          }
 
          MMTime(long sec, long uSec) : sec_(sec), uSec_(uSec)
          {
             Normalize();
          }
 
          ~MMTime() {}
 
          MMTime(std::string serialized) {
             std::stringstream is(serialized);
             is >> sec_ >> uSec_;
             Normalize();
          }
 
          std::string serialize() {
             std::ostringstream os;
             os << sec_ << " " << uSec_;
             return os.str().c_str();
          }
 
          long sec_;
          long uSec_;
 
          MMTime operator+(const MMTime &other) const
          {
             MMTime res(sec_ + other.sec_, uSec_ + other.uSec_);
             return res;
          }
 
          MMTime operator-(const MMTime &other) const
          {
             MMTime res(sec_ - other.sec_, uSec_ - other.uSec_);
             return res;
          }
 
          bool operator>(const MMTime &other) const
          {
             if (sec_ > other.sec_)
                return true;
             else if (sec_ < other.sec_)
                return false;
 
             if (uSec_ > other.uSec_)
                return true;
             else
                return false;
          }
 
          bool operator<(const MMTime &other) const
          {
             if (*this == other)
                return false;
 
             return ! (*this > other);
          }
 
          bool operator==(const MMTime &other) const
          {
             if (sec_ == other.sec_ && uSec_ == other.uSec_)
                return true;
             else
                return false;
          }
 
          double getMsec() const
          {
             return sec_ * 1000.0 + uSec_ / 1000.0;
          }
 
          double getUsec() const
          {
             return sec_ * 1.0e6 + uSec_;
          }
 
       private:
          void Normalize()
          {
             if (sec_ < 0)
             {
                sec_ = 0L;
                uSec_ = 0L;
                return;
             }
 
             if (uSec_ < 0)
             {
                sec_--;
                uSec_ = 1000000L + uSec_;
             }
 
             long overflow = uSec_ / 1000000L;
             if (overflow > 0)
             {
                sec_ += overflow;
                uSec_ -= overflow * 1000000L;
             }
          }
    };
 
 
    class TimeoutMs
    {
    public:
       // arguments:  MMTime start time, millisecond interval time
       TimeoutMs(const MMTime startTime, const unsigned long intervalMs) :
          startTime_(startTime),
          interval_(0, 1000*intervalMs)
       {
       }
       TimeoutMs(const MMTime startTime, const MMTime interval) :
          startTime_(startTime),
          interval_(interval)
       {
       }
       ~TimeoutMs()
       {
       }
       bool expired(const MMTime tnow)
       {
          MMTime elapsed = tnow - startTime_;
          return ( interval_ < elapsed );
       }
    private:
       TimeoutMs(const MM::TimeoutMs&) {}
       const TimeoutMs& operator=(const MM::TimeoutMs&) {return *this;}
       MMTime startTime_; // start time
       MMTime interval_; // interval in milliseconds
    };
 
 
    class Device {
    public:
       Device() {}
       virtual ~Device() {}
 
       virtual unsigned GetNumberOfProperties() const = 0;
       virtual int GetProperty(const char* name, char* value) const = 0;
       virtual int SetProperty(const char* name, const char* value) = 0;
       virtual bool HasProperty(const char* name) const = 0;
       virtual bool GetPropertyName(unsigned idx, char* name) const = 0;
       virtual int GetPropertyReadOnly(const char* name, bool& readOnly) const = 0;
       virtual int GetPropertyInitStatus(const char* name, bool& preInit) const = 0;
       virtual int HasPropertyLimits(const char* name, bool& hasLimits) const = 0;
       virtual int GetPropertyLowerLimit(const char* name, double& lowLimit) const = 0;
       virtual int GetPropertyUpperLimit(const char* name, double& hiLimit) const = 0;
       virtual int GetPropertyType(const char* name, MM::PropertyType& pt) const = 0;
       virtual unsigned GetNumberOfPropertyValues(const char* propertyName) const = 0;
       virtual bool GetPropertyValueAt(const char* propertyName, unsigned index, char* value) const = 0;
       virtual int IsPropertySequenceable(const char* name, bool& isSequenceable) const = 0;
       virtual int GetPropertySequenceMaxLength(const char* propertyName, long& nrEvents) const = 0;
       virtual int StartPropertySequence(const char* propertyName) = 0;
       virtual int StopPropertySequence(const char* propertyName) = 0;
       virtual int ClearPropertySequence(const char* propertyName) = 0;
       virtual int AddToPropertySequence(const char* propertyName, const char* value) = 0;
       virtual int SendPropertySequence(const char* propertyName) = 0;
 
       virtual bool GetErrorText(int errorCode, char* errMessage) const = 0;
       virtual bool Busy() = 0;
       virtual double GetDelayMs() const = 0;
       virtual void SetDelayMs(double delay) = 0;
       virtual bool UsesDelay() = 0;
 
       // TODO Get/SetModuleHandle() is no longer used; can remove at a
       // convenient time.
       virtual HDEVMODULE GetModuleHandle() const = 0;
       virtual void SetModuleHandle(HDEVMODULE hLibraryHandle) = 0;
       virtual void SetLabel(const char* label) = 0;
       virtual void GetLabel(char* name) const = 0;
       virtual void SetModuleName(const char* moduleName) = 0;
       virtual void GetModuleName(char* moduleName) const = 0;
       virtual void SetDescription(const char* description) = 0;
       virtual void GetDescription(char* description) const = 0;
 
       virtual int Initialize() = 0;
       virtual int Shutdown() = 0;
 
       virtual DeviceType GetType() const = 0;
       virtual void GetName(char* name) const = 0;
       virtual void SetCallback(Core* callback) = 0;
 
       //device discovery API
       virtual bool SupportsDeviceDetection(void) = 0;
       virtual MM::DeviceDetectionStatus DetectDevice(void) = 0;
 
       // hub-peripheral relationship
       virtual void SetParentID(const char* parentId) = 0;
       virtual void GetParentID(char* parentID) const = 0;
       // virtual void SetID(const char* id) = 0;
       // virtual void GetID(char* id) const = 0;
    };
 
    class Generic : public Device
    {
    public:
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
    };
 
    class Camera : public Device {
    public:
       Camera() {}
       virtual ~Camera() {}
 
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // Camera API
       virtual int SnapImage() = 0;
       virtual const unsigned char* GetImageBuffer() = 0;
       virtual const unsigned char* GetImageBuffer(unsigned channelNr) = 0;
       virtual const unsigned int* GetImageBufferAsRGB32() = 0;
       virtual unsigned GetNumberOfComponents() const = 0;
       virtual int GetComponentName(unsigned component, char* name) = 0;
       virtual int unsigned GetNumberOfChannels() const = 0;
       virtual int GetChannelName(unsigned channel, char* name) = 0;
       virtual long GetImageBufferSize() const = 0;
       virtual unsigned GetImageWidth() const = 0;
       virtual unsigned GetImageHeight() const = 0;
       virtual unsigned GetImageBytesPerPixel() const = 0;
       virtual unsigned GetBitDepth() const = 0;
       virtual double GetPixelSizeUm() const = 0;
       virtual int GetBinning() const = 0;
       virtual int SetBinning(int binSize) = 0;
       virtual void SetExposure(double exp_ms) = 0;
       virtual double GetExposure() const = 0;
       virtual int SetROI(unsigned x, unsigned y, unsigned xSize, unsigned ySize) = 0;
       virtual int GetROI(unsigned& x, unsigned& y, unsigned& xSize, unsigned& ySize) = 0;
       virtual int ClearROI() = 0;
       virtual bool SupportsMultiROI() = 0;
       virtual bool IsMultiROISet() = 0;
       virtual int GetMultiROICount(unsigned& count) = 0;
       virtual int SetMultiROI(const unsigned* xs, const unsigned* ys,
               const unsigned* widths, const unsigned* heights,
               unsigned numROIs) = 0;
       virtual int GetMultiROI(unsigned* xs, unsigned* ys, unsigned* widths,
               unsigned* heights, unsigned* length) = 0;
       virtual int StartSequenceAcquisition(long numImages, double interval_ms, bool stopOnOverflow) = 0;
       virtual int StartSequenceAcquisition(double interval_ms) = 0;
       virtual int StopSequenceAcquisition() = 0;
       virtual int PrepareSequenceAcqusition() = 0;
       virtual bool IsCapturing() = 0;
 
       virtual void GetTags(char* serializedMetadata) = 0;
 
       virtual void AddTag(const char* key, const char* deviceLabel, const char* value) = 0;
 
       virtual void RemoveTag(const char* key) = 0;
 
       virtual int IsExposureSequenceable(bool& isSequenceable) const = 0;
 
       // Sequence functions
       // Sequences can be used for fast acquisitions, synchronized by TTLs rather than
       // computer commands.
       // Sequences of exposures can be uploaded to the camera.  The camera will cycle through
       // the uploaded list of exposures (triggered by either an internal or
       // external trigger).  If the device is capable (and ready) to do so isSequenceable will
       // be true. If your device can not execute this (true for most cameras)
       // simply set IsExposureSequenceable to false
       virtual int GetExposureSequenceMaxLength(long& nrEvents) const = 0;
       virtual int StartExposureSequence() = 0;
       virtual int StopExposureSequence() = 0;
       // Remove all values in the sequence
       virtual int ClearExposureSequence() = 0;
       // Add one value to the sequence
       virtual int AddToExposureSequence(double exposureTime_ms) = 0;
       // Signal that we are done sending sequence values so that the adapter can send the whole sequence to the device
       virtual int SendExposureSequence() const = 0;
    };
 
    class Shutter : public Device
    {
    public:
       Shutter() {}
       virtual ~Shutter() {}
 
       // Device API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // Shutter API
       virtual int SetOpen(bool open = true) = 0;
       virtual int GetOpen(bool& open) = 0;
       virtual int Fire(double deltaT) = 0;
    };
 
    class Stage : public Device
    {
    public:
       Stage() {}
       virtual ~Stage() {}
 
       // Device API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // Stage API
       virtual int SetPositionUm(double pos) = 0;
       virtual int SetRelativePositionUm(double d) = 0;
       virtual int Move(double velocity) = 0;
       virtual int Stop() = 0;
       virtual int Home() = 0;
       virtual int SetAdapterOriginUm(double d) = 0;
       virtual int GetPositionUm(double& pos) = 0;
       virtual int SetPositionSteps(long steps) = 0;
       virtual int GetPositionSteps(long& steps) = 0;
       virtual int SetOrigin() = 0;
       virtual int GetLimits(double& lower, double& upper) = 0;
 
       virtual int GetFocusDirection(FocusDirection& direction) = 0;
 
       virtual int IsStageSequenceable(bool& isSequenceable) const = 0;
 
       virtual int IsStageLinearSequenceable(bool& isSequenceable) const = 0;
 
       // Check if a stage has continuous focusing capability (positions can be set while continuous focus runs).
       virtual bool IsContinuousFocusDrive() const = 0;
 
       // Sequence functions
       // Sequences can be used for fast acquisitions, synchronized by TTLs rather than
       // computer commands.
       // Sequences of positions can be uploaded to the stage.  The device will cycle through
       // the uploaded list of states (triggered by an external trigger - most often coming
       // from the camera).  If the device is capable (and ready) to do so isSequenceable will
       // be true. If your device can not execute this (true for most stages)
       // simply set isSequenceable to false
       virtual int GetStageSequenceMaxLength(long& nrEvents) const = 0;
       virtual int StartStageSequence() = 0;
       virtual int StopStageSequence() = 0;
       virtual int ClearStageSequence() = 0;
       virtual int AddToStageSequence(double position) = 0;
       virtual int SendStageSequence() = 0;
 
       virtual int SetStageLinearSequence(double dZ_um, long nSlices) = 0;
    };
 
    class XYStage : public Device
    {
    public:
       XYStage() {}
       virtual ~XYStage() {}
 
       // Device API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // XYStage API
       // it is recommended that device adapters implement the  "Steps" methods
       // taking long integers but leave the default implementations (in
       // DeviceBase.h) for the "Um" methods taking doubles. The latter utilize
       // directionality and origin settings set by user and operate via the
       // "Steps" methods. The step size is the inherent minimum distance/step
       // and should be defined by the adapter.
       virtual int SetPositionUm(double x, double y) = 0;
       virtual int SetRelativePositionUm(double dx, double dy) = 0;
       virtual int SetAdapterOriginUm(double x, double y) = 0;
       virtual int GetPositionUm(double& x, double& y) = 0;
       virtual int GetLimitsUm(double& xMin, double& xMax, double& yMin, double& yMax) = 0;
       virtual int Move(double vx, double vy) = 0;
 
       virtual int SetPositionSteps(long x, long y) = 0;
       virtual int GetPositionSteps(long& x, long& y) = 0;
       virtual int SetRelativePositionSteps(long x, long y) = 0;
       virtual int Home() = 0;
       virtual int Stop() = 0;
 
       virtual int SetOrigin() = 0;
 
       virtual int SetXOrigin() = 0;
 
       virtual int SetYOrigin() = 0;
 
       virtual int GetStepLimits(long& xMin, long& xMax, long& yMin, long& yMax) = 0;
       virtual double GetStepSizeXUm() = 0;
       virtual double GetStepSizeYUm() = 0;
       virtual int IsXYStageSequenceable(bool& isSequenceable) const = 0;
       // Sequence functions
       // Sequences can be used for fast acquisitions, synchronized by TTLs rather than
       // computer commands.
       // Sequences of positions can be uploaded to the XY stage.  The device will cycle through
       // the uploaded list of states (triggered by an external trigger - most often coming
       // from the camera).  If the device is capable (and ready) to do so isSequenceable will
       // be true. If your device can not execute this (true for most XY stages
       // simply set isSequenceable to false
       virtual int GetXYStageSequenceMaxLength(long& nrEvents) const = 0;
       virtual int StartXYStageSequence() = 0;
       virtual int StopXYStageSequence() = 0;
       virtual int ClearXYStageSequence() = 0;
       virtual int AddToXYStageSequence(double positionX, double positionY) = 0;
       virtual int SendXYStageSequence() = 0;
 
    };
 
    class State : public Device
    {
    public:
       State() {}
       virtual ~State() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // MMStateDevice API
       virtual int SetPosition(long pos) = 0;
       virtual int SetPosition(const char* label) = 0;
       virtual int GetPosition(long& pos) const = 0;
       virtual int GetPosition(char* label) const = 0;
       virtual int GetPositionLabel(long pos, char* label) const = 0;
       virtual int GetLabelPosition(const char* label, long& pos) const = 0;
       virtual int SetPositionLabel(long pos, const char* label) = 0;
       virtual unsigned long GetNumberOfPositions() const = 0;
       virtual int SetGateOpen(bool open = true) = 0;
       virtual int GetGateOpen(bool& open) = 0;
    };
 
    class Serial : public Device
    {
    public:
       Serial() {}
       virtual ~Serial() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // Serial API
       virtual PortType GetPortType() const = 0;
       virtual int SetCommand(const char* command, const char* term) = 0;
       virtual int GetAnswer(char* txt, unsigned maxChars, const char* term) = 0;
       virtual int Write(const unsigned char* buf, unsigned long bufLen) = 0;
       virtual int Read(unsigned char* buf, unsigned long bufLen, unsigned long& charsRead) = 0;
       virtual int Purge() = 0;
    };
 
    class AutoFocus : public Device
    {
    public:
       AutoFocus() {}
       virtual ~AutoFocus() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // AutoFocus API
       virtual int SetContinuousFocusing(bool state) = 0;
       virtual int GetContinuousFocusing(bool& state) = 0;
       virtual bool IsContinuousFocusLocked() = 0;
       virtual int FullFocus() = 0;
       virtual int IncrementalFocus() = 0;
       virtual int GetLastFocusScore(double& score) = 0;
       virtual int GetCurrentFocusScore(double& score) = 0;
       virtual int AutoSetParameters() = 0;
       virtual int GetOffset(double &offset) = 0;
       virtual int SetOffset(double offset) = 0;
    };
 
    class ImageProcessor : public Device
    {
       public:
          ImageProcessor() {}
          virtual ~ImageProcessor() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // image processor API
       virtual int Process(unsigned char* buffer, unsigned width, unsigned height, unsigned byteDepth) = 0;
 
 
    };
 
    class SignalIO : public Device
    {
    public:
       SignalIO() {}
       virtual ~SignalIO() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // signal io API
       virtual int SetGateOpen(bool open = true) = 0;
       virtual int GetGateOpen(bool& open) = 0;
       virtual int SetSignal(double volts) = 0;
       virtual int GetSignal(double& volts) = 0;
       virtual int GetLimits(double& minVolts, double& maxVolts) = 0;
 
       virtual int IsDASequenceable(bool& isSequenceable) const = 0;
 
       // Sequence functions
       // Sequences can be used for fast acquisitions, synchronized by TTLs rather than
       // computer commands.
       // Sequences of voltages can be uploaded to the DA.  The device will cycle through
       // the uploaded list of voltages (triggered by an external trigger - most often coming
       // from the camera).  If the device is capable (and ready) to do so isSequenceable will
       // be true. If your device can not execute this simply set isSequenceable to false
       virtual int GetDASequenceMaxLength(long& nrEvents) const = 0;
       virtual int StartDASequence() = 0;
       virtual int StopDASequence() = 0;
       virtual int ClearDASequence() = 0;
 
       virtual int AddToDASequence(double voltage) = 0;
       virtual int SendDASequence() = 0;
 
    };
 
    class Magnifier : public Device
    {
    public:
       Magnifier() {}
       virtual ~Magnifier() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       virtual double GetMagnification() = 0;
    };
 
 
    class SLM : public Device
    {
    public:
       SLM() {}
       virtual ~SLM() {}
 
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       // SLM API
       virtual int SetImage(unsigned char * pixels) = 0;
 
       virtual int SetImage(unsigned int * pixels) = 0;
 
       virtual int DisplayImage() = 0;
 
       virtual int SetPixelsTo(unsigned char intensity) = 0;
 
       virtual int SetPixelsTo(unsigned char red, unsigned char green, unsigned char blue) = 0;
 
       virtual int SetExposure(double interval_ms) = 0;
 
       virtual double GetExposure() = 0;
 
       virtual unsigned GetWidth() = 0;
 
       virtual unsigned GetHeight() = 0;
 
       virtual unsigned GetNumberOfComponents() = 0;
 
       virtual unsigned GetBytesPerPixel() = 0;
 
       // SLM Sequence functions
       // Sequences can be used for fast acquisitions, synchronized by TTLs rather than
       // computer commands.
       // Sequences of images can be uploaded to the SLM.  The SLM will cycle through
       // the uploaded list of images (perhaps triggered by an external trigger or by
       // an internal clock.
       // If the device is capable (and ready) to do so IsSLMSequenceable will return
       // be true. If your device can not execute sequences, IsSLMSequenceable returns false.
 
       virtual int IsSLMSequenceable(bool& isSequenceable) const = 0;
 
       virtual int GetSLMSequenceMaxLength(long& nrEvents) const = 0;
 
       virtual int StartSLMSequence() = 0;
 
       virtual int StopSLMSequence() = 0;
 
       virtual int ClearSLMSequence() = 0;
 
       virtual int AddToSLMSequence(const unsigned char * const pixels) = 0;
 
       virtual int AddToSLMSequence(const unsigned int * const pixels) = 0;
 
       virtual int SendSLMSequence() = 0;
 
    };
 
    class Galvo : public Device
    {
    public:
       Galvo() {}
       virtual ~Galvo() {}
 
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
    //Galvo API:
 
       virtual int PointAndFire(double x, double y, double time_us) = 0;
       virtual int SetSpotInterval(double pulseInterval_us) = 0;
       virtual int SetPosition(double x, double y) = 0;
       virtual int GetPosition(double& x, double& y) = 0;
       virtual int SetIlluminationState(bool on) = 0;
       virtual double GetXRange() = 0;
       virtual double GetXMinimum() = 0;
       virtual double GetYRange() = 0;
       virtual double GetYMinimum() = 0;
       virtual int AddPolygonVertex(int polygonIndex, double x, double y) = 0;
       virtual int DeletePolygons() = 0;
       virtual int RunSequence() = 0;
       virtual int LoadPolygons() = 0;
       virtual int SetPolygonRepetitions(int repetitions) = 0;
       virtual int RunPolygons() = 0;
       virtual int StopSequence() = 0;
       virtual int GetChannel(char* channelName) = 0;
    };
 
    class Hub : public Device
    {
    public:
       Hub() {}
       virtual ~Hub() {}
 
       // MMDevice API
       virtual DeviceType GetType() const { return Type; }
       static const DeviceType Type;
 
       virtual int DetectInstalledDevices() = 0;
 
       virtual void ClearInstalledDevices() = 0;
 
       virtual unsigned GetNumberOfInstalledDevices() = 0;
 
       virtual Device* GetInstalledDevice(int devIdx) = 0;
    };
 
    class Core
    {
    public:
       Core() {}
       virtual ~Core() {}
 
       virtual int LogMessage(const Device* caller, const char* msg, bool debugOnly) const = 0;
       virtual Device* GetDevice(const Device* caller, const char* label) = 0;
       virtual int GetDeviceProperty(const char* deviceName, const char* propName, char* value) = 0;
       virtual int SetDeviceProperty(const char* deviceName, const char* propName, const char* value) = 0;
 
 
       virtual void GetLoadedDeviceOfType(const Device* caller, MM::DeviceType devType, char* pDeviceName, const unsigned int deviceIterator) = 0;
 
       virtual int SetSerialProperties(const char* portName,
                                       const char* answerTimeout,
                                       const char* baudRate,
                                       const char* delayBetweenCharsMs,
                                       const char* handshaking,
                                       const char* parity,
                                       const char* stopBits) = 0;
       virtual int SetSerialCommand(const Device* caller, const char* portName, const char* command, const char* term) = 0;
       virtual int GetSerialAnswer(const Device* caller, const char* portName, unsigned long ansLength, char* answer, const char* term) = 0;
       virtual int WriteToSerial(const Device* caller, const char* port, const unsigned char* buf, unsigned long length) = 0;
       virtual int ReadFromSerial(const Device* caller, const char* port, unsigned char* buf, unsigned long length, unsigned long& read) = 0;
       virtual int PurgeSerial(const Device* caller, const char* portName) = 0;
       virtual MM::PortType GetSerialPortType(const char* portName) const = 0;
 
       virtual int OnPropertiesChanged(const Device* caller) = 0;
       virtual int OnPropertyChanged(const Device* caller, const char* propName, const char* propValue) = 0;
       virtual int OnStagePositionChanged(const Device* caller, double pos) = 0;
       virtual int OnXYStagePositionChanged(const Device* caller, double xPos, double yPos) = 0;
       virtual int OnExposureChanged(const Device* caller, double newExposure) = 0;
       virtual int OnSLMExposureChanged(const Device* caller, double newExposure) = 0;
       virtual int OnMagnifierChanged(const Device* caller) = 0;
 
       virtual unsigned long GetClockTicksUs(const Device* caller) = 0;
       virtual MM::MMTime GetCurrentMMTime() = 0;
 
       // sequence acquisition
       virtual int AcqFinished(const Device* caller, int statusCode) = 0;
       virtual int PrepareForAcq(const Device* caller) = 0;
       virtual int InsertImage(const Device* caller, const ImgBuffer& buf) = 0;
       virtual int InsertImage(const Device* caller, const unsigned char* buf, unsigned width, unsigned height, unsigned byteDepth, unsigned nComponents, const char* serializedMetadata, const bool doProcess = true) = 0;
       virtual int InsertImage(const Device* caller, const unsigned char* buf, unsigned width, unsigned height, unsigned byteDepth, const Metadata* md = 0, const bool doProcess = true) = 0;
       virtual int InsertImage(const Device* caller, const unsigned char* buf, unsigned width, unsigned height, unsigned byteDepth, const char* serializedMetadata, const bool doProcess = true) = 0;
       virtual void ClearImageBuffer(const Device* caller) = 0;
       virtual bool InitializeImageBuffer(unsigned channels, unsigned slices, unsigned int w, unsigned int h, unsigned int pixDepth) = 0;
       virtual int InsertMultiChannel(const Device* caller, const unsigned char* buf, unsigned numChannels, unsigned width, unsigned height, unsigned byteDepth, Metadata* md = 0) = 0;
 
       // autofocus
       // TODO This interface needs improvement: the caller pointer should be
       // passed, and it should be clarified whether the use of these methods is
       // to be limited to autofocus or not. - Mark T.
       virtual const char* GetImage() = 0;
       virtual int GetImageDimensions(int& width, int& height, int& depth) = 0;
       virtual int GetFocusPosition(double& pos) = 0;
       virtual int SetFocusPosition(double pos) = 0;
       virtual int MoveFocus(double velocity) = 0;
       virtual int SetXYPosition(double x, double y) = 0;
       virtual int GetXYPosition(double& x, double& y) = 0;
       virtual int MoveXYStage(double vX, double vY) = 0;
       virtual int SetExposure(double expMs) = 0;
       virtual int GetExposure(double& expMs) = 0;
       virtual int SetConfig(const char* group, const char* name) = 0;
       virtual int GetCurrentConfig(const char* group, int bufLen, char* name) = 0;
       virtual int GetChannelConfig(char* channelConfigName, const unsigned int channelConfigIterator) = 0;
 
       // direct access to specific device types
       // TODO With the exception of GetParentHub(), these should be removed in
       // favor of methods providing indirect access to the required
       // functionality. Eventually we should completely avoid access to raw
       // pointers to devices of other device adapters (because we loose
       // information on errors, because direct access ignores any
       // synchronization implemented in the Core, and because it would be bad
       // if device adapters stored the returned pointer). - Mark T.
       virtual MM::ImageProcessor* GetImageProcessor(const MM::Device* caller) = 0; // Use not recommended
       virtual MM::AutoFocus* GetAutoFocus(const MM::Device* caller) = 0; // Use not recommended
 
       virtual MM::Hub* GetParentHub(const MM::Device* caller) const = 0;
 
       virtual MM::State* GetStateDevice(const MM::Device* caller, const char* deviceName) = 0; // Use not recommended
       virtual MM::SignalIO* GetSignalIODevice(const MM::Device* caller, const char* deviceName) = 0; // Use not recommended
 
       // asynchronous error handling
       // TODO We do need a framework for handling asynchronous errors, but this
       // interface is poorly thought through. I'm working on a better design.
       // - Mark T.
       MM_DEPRECATED(virtual void NextPostedError(int& /*errorCode*/, char* /*pMessage*/, int /*maxlen*/, int& /*messageLength*/)) = 0;
       MM_DEPRECATED(virtual void PostError(const int, const char*)) = 0;
       MM_DEPRECATED(virtual void ClearPostedErrors(void)) = 0;
    };
 
 } // namespace MM
 
 #endif //MMMMDEVICE_H