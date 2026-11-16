// Native Windows backend.
//
// Replaces v1's Python dependencies one-for-one:
//   pycaw                       -> Core Audio IAudioEndpointVolume
//   screen_brightness_control   -> WMI WmiMonitorBrightnessMethods (laptop panels)
//                                  + DDC/CI via dxva2 (external monitors)
//   pyautogui                   -> SendInput / SetCursorPos
//   psutil                      -> Toolhelp32 process snapshot (cached)
//   win32gui (topmost window)   -> SetWindowPos
//
// COM is initialised lazily on whichever thread first needs it, because the
// AsyncSystemControl decorator calls this class from its worker thread.
#include <windows.h>

#include <endpointvolume.h>
#include <highlevelmonitorconfigurationapi.h>
#include <mmdeviceapi.h>
#include <physicalmonitorenumerationapi.h>
#include <shellapi.h>
#include <tlhelp32.h>
#include <wbemidl.h>

#include <algorithm>
#include <chrono>
#include <cwctype>
#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "handtracker/core/system_control.hpp"

namespace ht {
namespace {

// GUIDs defined locally so we don't depend on uuid.lib/wbemuuid.lib quirks
// across MSVC and MinGW toolchains.
const GUID kClsidMMDeviceEnumerator = {
    0xBCDE0395, 0xE52F, 0x467C, {0x8E, 0x3D, 0xC4, 0x57, 0x92, 0x91, 0x69, 0x2E}};
const GUID kIidIMMDeviceEnumerator = {
    0xA95664D2, 0x9614, 0x4F35, {0xA7, 0x46, 0xDE, 0x8D, 0xB6, 0x36, 0x17, 0xE6}};
const GUID kIidIAudioEndpointVolume = {
    0x5CDF2C82, 0x841E, 0x4546, {0x97, 0x22, 0x0C, 0xF7, 0x40, 0x78, 0x22, 0x9A}};
const GUID kClsidWbemLocator = {
    0x4590F811, 0x1D3A, 0x11D0, {0x89, 0x1F, 0x00, 0xAA, 0x00, 0x4B, 0x2E, 0x24}};
const GUID kIidIWbemLocator = {
    0xDC12A687, 0x737F, 0x11CF, {0x88, 0x4D, 0x00, 0xAA, 0x00, 0x4B, 0x2E, 0x24}};

/// Minimal intrusive COM pointer (avoids depending on WRL/ATL).
template <typename T>
class ComPtr {
 public:
  ComPtr() = default;
  ~ComPtr() { reset(); }
  ComPtr(const ComPtr&) = delete;
  ComPtr& operator=(const ComPtr&) = delete;
  ComPtr(ComPtr&& o) noexcept : p_(o.p_) { o.p_ = nullptr; }
  ComPtr& operator=(ComPtr&& o) noexcept {
    if (this != &o) {
      reset();
      p_ = o.p_;
      o.p_ = nullptr;
    }
    return *this;
  }

  T* get() const { return p_; }
  T* operator->() const { return p_; }
  explicit operator bool() const { return p_ != nullptr; }
  T** put() {
    reset();
    return &p_;
  }
  void** putVoid() { return reinterpret_cast<void**>(put()); }
  void reset() {
    if (p_) p_->Release();
    p_ = nullptr;
  }

 private:
  T* p_ = nullptr;
};

/// RAII BSTR.
class Bstr {
 public:
  explicit Bstr(const wchar_t* s) : b_(SysAllocString(s)) {}
  ~Bstr() { SysFreeString(b_); }
  Bstr(const Bstr&) = delete;
  Bstr& operator=(const Bstr&) = delete;
  BSTR get() const { return b_; }

 private:
  BSTR b_;
};

std::wstring widen(const std::string& s) {
  if (s.empty()) return {};
  const int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
  std::wstring w(static_cast<std::size_t>(n), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
  return w;
}

std::wstring lowerW(std::wstring s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](wchar_t c) { return static_cast<wchar_t>(std::towlower(c)); });
  return s;
}

/// Initialises COM once per calling thread.
///
/// Deliberately never calls CoUninitialize: the COM objects cached in
/// Win32SystemControl may outlive the worker thread that created them, and
/// releasing them after the apartment is torn down crashes. The OS reclaims
/// everything at process exit.
void ensureCom() {
  thread_local bool initialised = [] {
    // RPC_E_CHANGED_MODE means someone else initialised COM here; still usable.
    CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    // Needed for WMI. Fails harmlessly with RPC_E_TOO_LATE if already set.
    CoInitializeSecurity(nullptr, -1, nullptr, nullptr, RPC_C_AUTHN_LEVEL_DEFAULT,
                         RPC_C_IMP_LEVEL_IMPERSONATE, nullptr, EOAC_NONE, nullptr);
    return true;
  }();
  (void)initialised;
}

struct KeyInfo {
  WORD vk;
  bool extended;
};

bool lookupKey(const std::string& name, KeyInfo& out) {
  if (name.size() == 1) {
    const char c = name[0];
    if (c >= 'a' && c <= 'z') return out = {static_cast<WORD>('A' + (c - 'a')), false}, true;
    if (c >= '0' && c <= '9') return out = {static_cast<WORD>(c), false}, true;
    return false;
  }
  if (name[0] == 'f' && name.size() <= 3) {
    const int n = std::atoi(name.c_str() + 1);
    if (n >= 1 && n <= 24) return out = {static_cast<WORD>(VK_F1 + n - 1), false}, true;
  }
  static const std::unordered_map<std::string, KeyInfo> kKeys = {
      {"ctrl", {VK_CONTROL, false}},
      {"alt", {VK_MENU, false}},
      {"shift", {VK_SHIFT, false}},
      {"win", {VK_LWIN, true}},
      {"tab", {VK_TAB, false}},
      {"enter", {VK_RETURN, false}},
      {"esc", {VK_ESCAPE, false}},
      {"space", {VK_SPACE, false}},
      {"backspace", {VK_BACK, false}},
      {"delete", {VK_DELETE, true}},
      {"insert", {VK_INSERT, true}},
      {"home", {VK_HOME, true}},
      {"end", {VK_END, true}},
      {"pageup", {VK_PRIOR, true}},
      {"pagedown", {VK_NEXT, true}},
      {"left", {VK_LEFT, true}},
      {"right", {VK_RIGHT, true}},
      {"up", {VK_UP, true}},
      {"down", {VK_DOWN, true}},
      {"printscreen", {VK_SNAPSHOT, true}},
      {"media_next", {VK_MEDIA_NEXT_TRACK, true}},
      {"media_prev", {VK_MEDIA_PREV_TRACK, true}},
      {"media_playpause", {VK_MEDIA_PLAY_PAUSE, true}},
      {"media_stop", {VK_MEDIA_STOP, true}},
      {"volume_mute", {VK_VOLUME_MUTE, true}},
      {"volume_up", {VK_VOLUME_UP, true}},
      {"volume_down", {VK_VOLUME_DOWN, true}},
      {"browser_back", {VK_BROWSER_BACK, true}},
      {"browser_forward", {VK_BROWSER_FORWARD, true}},
      {"browser_refresh", {VK_BROWSER_REFRESH, true}},
  };
  const auto it = kKeys.find(name);
  if (it == kKeys.end()) return false;
  out = it->second;
  return true;
}

INPUT keyInput(const KeyInfo& k, bool down) {
  INPUT in{};
  in.type = INPUT_KEYBOARD;
  in.ki.wVk = k.vk;
  in.ki.wScan = static_cast<WORD>(MapVirtualKeyW(k.vk, MAPVK_VK_TO_VSC));
  in.ki.dwFlags = (k.extended ? KEYEVENTF_EXTENDEDKEY : 0) | (down ? 0 : KEYEVENTF_KEYUP);
  return in;
}

BOOL CALLBACK collectMonitor(HMONITOR monitor, HDC, LPRECT, LPARAM data) {
  reinterpret_cast<std::vector<HMONITOR>*>(data)->push_back(monitor);
  return TRUE;
}

class Win32SystemControl final : public SystemControl {
 public:
  explicit Win32SystemControl(std::ostream& log) : log_(log) {}

  bool setVolume(float level) override {
    ensureCom();
    if (!volume_ && !acquireVolume()) return false;
    const HRESULT hr =
        volume_->SetMasterVolumeLevelScalar(std::clamp(level, 0.f, 1.f), nullptr);
    if (FAILED(hr)) {
      volume_.reset();  // Default device may have changed; re-acquire next time.
      return false;
    }
    return true;
  }

  bool setBrightness(float level) override {
    ensureCom();
    const int percent = static_cast<int>(std::clamp(level, 0.f, 1.f) * 100.f + 0.5f);
    const bool viaWmi = setBrightnessWmi(percent);
    const bool viaDdc = setBrightnessDdc(level);
    if (!viaWmi && !viaDdc && !warnedBrightness_) {
      log_ << "warning: no display accepted a brightness change (WMI and DDC/CI both failed)\n";
      warnedBrightness_ = true;
    }
    return viaWmi || viaDdc;
  }

  bool sendKeys(const Hotkey& hk) override {
    std::vector<KeyInfo> keys;
    for (const auto& name : hk.keys) {
      KeyInfo k{};
      if (!lookupKey(name, k)) {
        log_ << "warning: unsupported key '" << name << "'\n";
        return false;
      }
      keys.push_back(k);
    }
    std::vector<INPUT> inputs;
    for (const auto& k : keys) inputs.push_back(keyInput(k, true));
    for (auto it = keys.rbegin(); it != keys.rend(); ++it) inputs.push_back(keyInput(*it, false));
    const UINT sent = SendInput(static_cast<UINT>(inputs.size()), inputs.data(), sizeof(INPUT));
    return sent == inputs.size();
  }

  bool launch(const std::string& target) override {
    ensureCom();
    const std::wstring w = widen(target);
    const auto result = reinterpret_cast<INT_PTR>(
        ShellExecuteW(nullptr, L"open", w.c_str(), nullptr, nullptr, SW_SHOWNORMAL));
    if (result <= 32) {
      log_ << "warning: failed to launch '" << target << "'\n";
      return false;
    }
    return true;
  }

  bool moveCursor(float x, float y) override {
    const int w = GetSystemMetrics(SM_CXSCREEN);
    const int h = GetSystemMetrics(SM_CYSCREEN);
    const int px = static_cast<int>(std::clamp(x, 0.f, 1.f) * static_cast<float>(w - 1));
    const int py = static_cast<int>(std::clamp(y, 0.f, 1.f) * static_cast<float>(h - 1));
    return SetCursorPos(px, py) != 0;
  }

  bool setMouseButton(bool pressed) override {
    INPUT in{};
    in.type = INPUT_MOUSE;
    in.mi.dwFlags = pressed ? MOUSEEVENTF_LEFTDOWN : MOUSEEVENTF_LEFTUP;
    return SendInput(1, &in, sizeof(INPUT)) == 1;
  }

  bool isProcessRunning(const std::string& exeName) override {
    using Clock = std::chrono::steady_clock;
    const std::wstring target = lowerW(widen(exeName));
    const auto now = Clock::now();
    const auto cached = cache_.find(target);
    if (cached != cache_.end() && now - cached->second.when < std::chrono::seconds(2)) {
      return cached->second.running;
    }

    bool running = false;
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap != INVALID_HANDLE_VALUE) {
      PROCESSENTRY32W entry{};
      entry.dwSize = sizeof(entry);
      for (BOOL ok = Process32FirstW(snap, &entry); ok; ok = Process32NextW(snap, &entry)) {
        if (lowerW(entry.szExeFile) == target) {
          running = true;
          break;
        }
      }
      CloseHandle(snap);
    }
    cache_[target] = {now, running};
    return running;
  }

  std::string name() const override { return "win32"; }

 private:
  struct CacheEntry {
    std::chrono::steady_clock::time_point when;
    bool running;
  };

  bool acquireVolume() {
    ComPtr<IMMDeviceEnumerator> enumerator;
    if (FAILED(CoCreateInstance(kClsidMMDeviceEnumerator, nullptr, CLSCTX_ALL,
                                kIidIMMDeviceEnumerator, enumerator.putVoid()))) {
      return false;
    }
    ComPtr<IMMDevice> device;
    if (FAILED(enumerator->GetDefaultAudioEndpoint(eRender, eConsole, device.put()))) return false;
    return SUCCEEDED(
        device->Activate(kIidIAudioEndpointVolume, CLSCTX_ALL, nullptr, volume_.putVoid()));
  }

  bool connectWmi() {
    if (wmi_) return true;
    if (wmiUnavailable_) return false;
    ComPtr<IWbemLocator> locator;
    if (FAILED(CoCreateInstance(kClsidWbemLocator, nullptr, CLSCTX_INPROC_SERVER,
                                kIidIWbemLocator, locator.putVoid()))) {
      wmiUnavailable_ = true;
      return false;
    }
    Bstr ns(L"ROOT\\WMI");
    if (FAILED(locator->ConnectServer(ns.get(), nullptr, nullptr, nullptr, 0, nullptr, nullptr,
                                      wmi_.put()))) {
      wmiUnavailable_ = true;
      return false;
    }
    CoSetProxyBlanket(wmi_.get(), RPC_C_AUTHN_WINNT, RPC_C_AUTHZ_NONE, nullptr,
                      RPC_C_AUTHN_LEVEL_CALL, RPC_C_IMP_LEVEL_IMPERSONATE, nullptr, EOAC_NONE);
    return true;
  }

  /// Internal laptop panels expose brightness through WMI.
  bool setBrightnessWmi(int percent) {
    if (!connectWmi()) return false;
    Bstr wql(L"WQL");
    Bstr query(L"SELECT * FROM WmiMonitorBrightnessMethods");
    ComPtr<IEnumWbemClassObject> results;
    if (FAILED(wmi_->ExecQuery(wql.get(), query.get(),
                               WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY, nullptr,
                               results.put()))) {
      return false;
    }

    Bstr className(L"WmiMonitorBrightnessMethods");
    Bstr methodName(L"WmiSetBrightness");
    bool any = false;
    for (;;) {
      ComPtr<IWbemClassObject> instance;
      ULONG returned = 0;
      if (results->Next(WBEM_INFINITE, 1, instance.put(), &returned) != WBEM_S_NO_ERROR ||
          returned == 0) {
        break;
      }
      VARIANT path;
      VariantInit(&path);
      if (FAILED(instance->Get(L"__PATH", 0, &path, nullptr, nullptr)) || path.vt != VT_BSTR) {
        VariantClear(&path);
        continue;
      }

      ComPtr<IWbemClassObject> cls;
      ComPtr<IWbemClassObject> inDef;
      ComPtr<IWbemClassObject> args;
      if (SUCCEEDED(wmi_->GetObject(className.get(), 0, nullptr, cls.put(), nullptr)) &&
          SUCCEEDED(cls->GetMethod(L"WmiSetBrightness", 0, inDef.put(), nullptr)) &&
          SUCCEEDED(inDef->SpawnInstance(0, args.put()))) {
        VARIANT v;
        VariantInit(&v);
        v.vt = VT_I4;
        v.lVal = 0;  // Timeout (seconds)
        args->Put(L"Timeout", 0, &v, 0);
        v.vt = VT_UI1;
        v.bVal = static_cast<BYTE>(percent);
        args->Put(L"Brightness", 0, &v, 0);
        if (SUCCEEDED(wmi_->ExecMethod(path.bstrVal, methodName.get(), 0, nullptr, args.get(),
                                       nullptr, nullptr))) {
          any = true;
        }
      }
      VariantClear(&path);
    }
    return any;
  }

  /// External monitors via DDC/CI (only works if the monitor supports it).
  bool setBrightnessDdc(float level) {
    std::vector<HMONITOR> monitors;
    EnumDisplayMonitors(nullptr, nullptr, collectMonitor, reinterpret_cast<LPARAM>(&monitors));
    bool any = false;
    for (HMONITOR m : monitors) {
      DWORD count = 0;
      if (!GetNumberOfPhysicalMonitorsFromHMONITOR(m, &count) || count == 0) continue;
      std::vector<PHYSICAL_MONITOR> physical(count);
      if (!GetPhysicalMonitorsFromHMONITOR(m, count, physical.data())) continue;
      for (const auto& pm : physical) {
        DWORD lo = 0, cur = 0, hi = 0;
        if (GetMonitorBrightness(pm.hPhysicalMonitor, &lo, &cur, &hi) && hi > lo) {
          const auto value = static_cast<DWORD>(static_cast<float>(lo) +
                                                level * static_cast<float>(hi - lo) + 0.5f);
          if (SetMonitorBrightness(pm.hPhysicalMonitor, value)) any = true;
        }
      }
      DestroyPhysicalMonitors(count, physical.data());
    }
    return any;
  }

  std::ostream& log_;
  ComPtr<IAudioEndpointVolume> volume_;
  ComPtr<IWbemServices> wmi_;
  bool wmiUnavailable_ = false;
  bool warnedBrightness_ = false;
  std::unordered_map<std::wstring, CacheEntry> cache_;
};

}  // namespace

std::unique_ptr<SystemControl> createPlatformSystemControl(std::ostream& log) {
  return std::make_unique<Win32SystemControl>(log);
}

void platformInit() {
  // Without this, cursor coordinates are scaled on high-DPI displays and the
  // mouse can't reach the screen edges.
  SetProcessDPIAware();
}

bool makeWindowTopmost(const std::string& windowTitle) {
  HWND hwnd = FindWindowW(nullptr, widen(windowTitle).c_str());
  if (!hwnd) return false;
  return SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE) != 0;
}

}  // namespace ht
