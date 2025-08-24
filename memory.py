import sys, os

def alloc_mb(mb):
    b = bytearray(mb * 1024 * 1024)
    for i in range(0, len(b), 4096):  # touch each page
        b[i] = 1
    return len(b)

        

def get_mem_kb():
    """Return (rss_now_kb, peak_kb). Peak is lifetime peak if available."""
    if sys.platform.startswith("linux"):
        rss = peak = None
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1])         # KB
                elif line.startswith("VmHWM:"):
                    peak = int(line.split()[1])        # KB (lifetime peak)
        return rss, peak
    elif sys.platform.startswith("win"):
        import ctypes, ctypes.wintypes as wt
        psapi = ctypes.WinDLL('Psapi.dll')
        kernel32 = ctypes.WinDLL('Kernel32.dll')
        class PMC(ctypes.Structure):
            _fields_ = [
                ('cb', wt.DWORD), ('PageFaultCount', wt.DWORD),
                ('PeakWorkingSetSize', ctypes.c_size_t),
                ('WorkingSetSize', ctypes.c_size_t),
                ('QuotaPeakPagedPoolUsage', ctypes.c_size_t),
                ('QuotaPagedPoolUsage', ctypes.c_size_t),
                ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t),
                ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                ('PagefileUsage', ctypes.c_size_t),
                ('PeakPagefileUsage', ctypes.c_size_t),
            ]
        h = kernel32.GetCurrentProcess()
        pmc = PMC(); pmc.cb = ctypes.sizeof(PMC)
        psapi.GetProcessMemoryInfo(h, ctypes.byref(pmc), pmc.cb)
        return pmc.WorkingSetSize/1024.0, pmc.PeakWorkingSetSize/1024.0
    else:  # macOS or fallback
        try:
            import resource, psutil
            rss = psutil.Process(os.getpid()).memory_info().rss/1024.0
            peak = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0  # bytes→KB
            return rss, peak
        except Exception:
            return None, None
import time

rss0, peak0 = get_mem_kb()

t0 = time.perf_counter()
result = alloc_mb(2)
dt_ms = (time.perf_counter() - t0) * 1000

rss1, peak1 = get_mem_kb()

print(f"time={dt_ms:.2f} ms")
print(f"RSS now: {rss1:.1f} KB  ΔRSS: {(rss1 - rss0):.1f} KB")
print(f"Peak during call (approx): {(peak1 - peak0):.1f} KB")

