
# RT-PhysicsCore — Telemetry & Profiling

Design decisions, data structures, and pipeline: scoped profiling instrumentation, lock-free memory tracking, Chrome Tracing serialization, multi-mode in-engine visualization, dual-window OpenGL context sharing, and the standalone offline trace viewer. Engine loop and ECS live in `engine_core.md`; physics dynamics live in `physics_design.md`.

## Architecture & Overview

The telemetry pipeline operates across four decoupled layers: zero-overhead instrumentation, singleton serialization, in-engine presentation, and standalone offline replay.

```
┌────────────────────────────────────────────────────────────────────────┐
│                        Instrumentation Layer                           │
│  RT_PROFILE_SCOPE(name) ──► TelemetryTimer (RAII) ──► ProfileResult    │
│  Global new / delete    ──► MemoryTracker (Lock-free atomics)          │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                       Data & Serialization Layer                       │
│  TelemetryManager (Singleton)                                          │
│   ├── outputStream ──────► telemetry_logs/run_YYYY-MM-DD_HH-MM-SS.json │
│   ├── liveResults  ──────► In-memory session buffer                    │
│   └── frameHistory ──────► 120-frame rolling summary queue             │
└──────────────┬───────────────────────────────────────────┬─────────────┘
               │                                           │
               ▼                                           ▼
┌──────────────────────────────┐           ┌─────────────────────────────┐
│    In-Engine Presentation    │           │    Standalone Viewer App    │
│  Mode 0: Off                 │           │  RT-ProfilerViewer.exe      │
│  Mode 1: Minimal HUD Overlay │           │   ├── SessionLoader (JSON)  │
│  Mode 2: Full Profiler Win   │           │   ├── SessionManager (Disk) │
│  (F3 Key Cycle, Context-Safe)│           │   └── ProfilerUI (Flame/RAM)│
└──────────────────────────────┘           └─────────────────────────────┘
```

## Instrumentation (`TelemetryMacros.h`, `TelemetryTimer.h/.cpp`)

- `RT_ENABLE_PROFILING`: Active in `_DEBUG` builds or when explicitly forced via the `RT_FORCE_PROFILING` CMake option (`-DRT_FORCE_PROFILING=ON`). In release builds without this flag, all instrumentation macros compile to empty no-ops with zero runtime branch or function call overhead.
- `RT_PROFILE_SCOPE(name)`: Instantiates a stack-bound `TelemetryTimer timer##__LINE__(name)`. RAII measures scope lifetime from constructor to destructor.
- `RT_PROFILE_FUNCTION()`: Convenience wrapper passing compiler `__FUNCTION__` metadata to `RT_PROFILE_SCOPE`.
- Scope duration measurement: Uses `std::chrono::high_resolution_clock`. Start and end timestamps are cast to microseconds since epoch (`std::chrono::time_point_cast<std::chrono::microseconds>`).
- Thread identification: Queries `std::this_thread::get_id()` and hashes it to a 32-bit integer (`uint32_t`), ensuring multi-threaded scopes are partitioned cleanly in visualization timelines.
- Frame-level memory snapshotting: Inside `TelemetryTimer::Stop()`, if `name == "Frame"`, current heap bytes, peak bytes, and active allocation counts are sampled directly from `MemoryTracker` and attached to the `ProfileResult`. Child function scopes do not sample memory to minimize cache churn and timer overhead.

## Memory Tracking (`MemoryTracker.h/.cpp`)

- Global allocation interception: Overloads global `operator new`, `operator delete`, `operator new[]`, and `operator delete[]` when profiling is enabled.
- Lock-free atomic accounting:
  - `currentBytes`: Tracks outstanding heap bytes via `std::atomic<std::size_t>`.
  - `peakBytes`: High-water mark updated via a lock-free compare-and-swap loop (`compare_exchange_weak`) using `std::memory_order_relaxed`.
  - `allocationCount`: Number of currently outstanding heap allocations.
- Sized deallocation: Leverages C++14 sized delete (`operator delete(void* ptr, std::size_t size) noexcept`). Subtracts the known block size directly from `currentBytes`, eliminating the need for hidden allocation headers or tracking hash tables that would pollute cache lines.

## Serialization & Trace Format (`TelemetryManager.h/.cpp`)

- Format compliance: Serializes to the standard Google Chrome Tracing Event Format (`chrome://tracing` and [Perfetto UI](https://ui.perfetto.dev)).
- Event schema: Complete Phase events (`"ph": "X"`), requiring start timestamp (`ts`) and duration (`dur`) in microseconds:
  ```json
  {"cat":"function","dur":1245,"name":"PhysicsStep","ph":"X","pid":0,"tid":28491024,"ts":1727827200100}
  ```
- Memory metadata injection: On `"Frame"` scopes, memory metrics are serialized as flat top-level keys to remain parser-friendly and avoid nested object allocations:
  ```json
  {"cat":"function","dur":16650,"name":"Frame","ph":"X","pid":0,"tid":0,"ts":1727827200000,"memory_current":2458900,"memory_peak":3145728,"memory_allocs":142}
  ```
- Output directory & naming: Traces are saved to `telemetry_logs/run_YYYY-MM-DD_HH-MM-SS.json`. Directory existence is verified and created on startup via `std::filesystem::create_directories`.
- Thread safety: Ingestion into the stream and live memory buffers is guarded by a `std::mutex` in `WriteProfile()`.
- Abrupt exit protection: Each event write flushes the stream buffer (`outputStream.flush()`). `WriteHeader()` outputs `{"traceEvents":[` on startup, and `Shutdown()` terminates the array with `]}`.

## In-Engine Presentation Modes (`TelemetryOverlay.h/.cpp`, `ProfilerWindow.h/.cpp`)

The engine maintains a three-state visual mode toggle cycled sequentially via the **F3** key:

| Mode | Name | Presentation Mechanism | Performance Impact |
|---|---|---|---|
| `0` | **Off** | No UI rendered; trace serialization continues silently in background. | Baseline |
| `1` | **Minimal** | Dear ImGui heads-up display overlay rendered directly on the simulation window. | Negligible (<0.1 ms) |
| `2` | **Full** | Dedicated secondary GLFW window running live flame graphs and scrubbing. | Minor (1 extra buffer swap) |

- **Mode 1 (HUD Overlay)**:
  - Rendered by `TelemetryOverlay::Render()`.
  - Positioned at fixed screen coordinates `(10, 10)` with background alpha `0.65f` and window flags `NoDecoration | AlwaysAutoResize | NoSavedSettings | NoFocusOnAppearing | NoNav`.
  - Computes a 1-second rolling average across the 120-element `FrameSummary` queue in `TelemetryManager`: FPS, Total Frame Time (ms), Physics Step (ms), Update (ms), and Render (ms).
  - Displays live heap metrics in MB: `currentBytes / (1024 * 1024)` and peak bytes.
- **Mode 2 (Dual-Monitor Window)**:
  - Managed by `ProfilerWindow`. Creates a 960×640 GLFW window sharing the primary window's OpenGL context (`glfwCreateWindow(..., mainWindow)`).
  - Embeds `ProfilerUI` across the entire secondary window client area.

## Multi-Window OpenGL & ImGui Management

Creating and tearing down secondary debug windows at runtime introduces severe graphics driver and context hazards. The engine enforces the following invariants:

- **Context Sharing**: `ProfilerWindow::Open(mainWindow)` passes `mainWindow` as the `share` parameter to `glfwCreateWindow`. This allows the secondary window to share texture objects and state while maintaining independent default framebuffers and swap chains.
- **Dual ImGui Contexts**: Dear ImGui contexts are per-window. The profiler allocates its own `ImGuiContext*` via `ImGui::CreateContext()`. Before any ImGui calls on either window, the engine explicitly binds the target context (`ImGui::SetCurrentContext`).
- **Context Restoration Invariant**: When `ProfilerWindow::Render()` or `ProfilerWindow::Close()` binds the secondary window's OpenGL context via `glfwMakeContextCurrent(window)`, it **must** restore the primary window context (`glfwMakeContextCurrent(mainWindow)`) and primary ImGui context before returning.
- **Teardown Crash Prevention**: Destroying a GLFW window while its context is active leaves the calling thread with a `NULL` context. Any subsequent OpenGL call (such as `glClear` in the next frame's `BeginFrame`) causes an immediate access violation at address `0x0000000000000000` inside function-pointer loaders. `ProfilerWindow::Close(mainWindow)` explicitly restores `mainWindow` before destruction.
- **Frame Lifecycle Guard (`ScopedRenderFrame`)**:
  `main.cpp` enforces a strict frame lifecycle wrapper:
  ```cpp
  engine.SetRenderCallback([&](double alpha)
  {
      ScopedRenderFrame frame(renderer); // Calls renderer.BeginFrame()
      scene.RenderUpdateSystems();
      overlay.Render();
      // Destructor calls renderer.EndFrame() and swaps buffers
      profilerWindow.Render(renderer.GetWindow());
  });
  ```
  This guarantees `ImGui::NewFrame()` and `ImGui::Render()` stay strictly paired even if early returns or exceptions occur, preventing the `IM_ASSERT(g.WithinFrameScope)` assertion.

## Interactive Flame Graph & Scrubbing (`ProfilerUI.h/.cpp`)

`ProfilerUI` is a reusable UI component shared between the live Mode 2 window and the standalone viewer:

- **Frame Timeline**: Renders per-frame durations using `ImGui::PlotLines`. Evaluates peak frame times to auto-scale the graph ceiling (`maxVal * 1.2f`).
- **Frame Scrubbing**:
  - Gathers all scopes named `"Frame"` from the results vector into an indexed lookup table.
  - A `SliderInt("Scrub Frame")` and `"Go to Latest"` button allow scrubbed inspection across thousands of recorded frames.
  - Memory panel reads directly from the selected frame's embedded `memoryCurrentBytes`, `memoryPeakBytes`, and `memoryAllocations`, accurately reconstructing historical memory without external state.
- **Nesting Depth Calculation**:
  - Iterates scopes falling within the selected frame's `[startTimestamp, endTimestamp]`.
  - Maintains a `depthEndStack` of end timestamps. When a new scope begins after the timestamp at the top of the stack, scopes are popped until a parent scope is found. The stack size directly dictates the vertical render depth (`entry.depth`).
- **Canvas Rendering**:
  - Rendered inside an `ImGui::BeginChild` canvas with horizontal scrolling.
  - Box dimensions: `x0 = canvasPos.x + normalizedStart * canvasWidth`, `y0 = canvasPos.y + depth * (barHeight + padding)`.
  - Scope coloring: Generates an FNV-1a-style hash from the scope name string, mapping the value to a phase-shifted HSV cosine palette for deterministic, consistent color coding across runs.
  - Hover tooltips: Invisible buttons overlaid on each bar expose an interactive tooltip displaying the scope label, microsecond/millisecond duration, and thread ID.

## Standalone Session Viewer (`RT-ProfilerViewer.exe`)

The standalone tool (`tools/profiler/ProfilerApplication.cpp`) provides post-mortem trace analysis, run comparisons, and disk management without running the physics simulation:

- **Session Discovery (`SessionLoader::DiscoverSessions`)**: Scans `telemetry_logs/` for `run_*.json` files. Parses timestamps from filenames, extracts file sizes, detects starred tags, and sorts the list newest-first.
- **Streaming Parser (`SessionLoader::Load`)**:
  - Custom zero-dependency JSON streaming tokenizer optimized for Chrome Tracing files up to hundreds of megabytes.
  - String Pointer Safety: `ProfileResult` stores `const char* name`. To prevent pointer invalidation when parsing unknown numbers of events, parsed string labels are stored in a `static std::deque<std::string>`. Unlike `std::vector`, `std::deque` guarantees stable references and pointers across dynamic growth.
- **Session Manager UI (`SessionManager.h/.cpp`)**:
  - Split two-panel viewport: session catalog on the left (320px width), active profiler inspection panel on the right.
  - Star / Pinning: Toggling the star status (`[*]`) renames the file on disk (`run_..._STARRED.json`).
  - Deletion Protection: Starred sessions cannot be deleted from the UI.
  - Bulk Cleanup: Includes a "Delete All Unstarred" button with a modal confirmation popup (`ConfirmDeleteAll`) to purge non-pinned runs while preserving benchmark baselines.
- **Metric Reconstruction (`SessionLoader::ReconstructFrameSummaries`)**: Scans raw traces offline to reconstruct high-level subsystem metrics (`totalFrameMs`, `physicsMs`, `updateMs`, `renderMs`) by matching `"Frame"` intervals with child `"FixedUpdate"`, `"Update"`, and `"Render"` events.
