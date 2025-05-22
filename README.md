# Measurement Manager

A minimal, generic C++ framework for state propagation and measurement application in recursive filters with delayed measurements support. Perfect for Kalman filters, particle filters, and other estimation algorithms that need to handle out-of-order measurements.

## Overview

Measurement manager solves a common problem in real-time filtering: handling measurements that arrive out of chronological order or with significant delays. Instead of discarding late measurements or applying them incorrectly, this framework maintains a buffer of historical states and replays the filter from the appropriate point in time.

### Key Concepts

- **State Buffer**: Maintains a history of filter states at different timestamps
- **Measurement Buffer**: Queues measurements with their timestamps for chronological processing
- **Replay Mechanism**: When new measurements arrive, rewinds to the appropriate state and replays forward
- **Automatic Pruning**: Keeps memory usage bounded by removing old data beyond the configured window

## Features

- ✅ Type-safe timestamps using `std::chrono::duration`
- ✅ Generic state types (default-constructible, copy-assignable type)
- ✅ Deterministic buffer management with configurable time window
- ✅ Callback-based interface
- ✅ Memory efficient
- ✅ Header-only


## Algorithm:
1. Identify earliest unprocessed measurement
2. Locate the most recent state before that measurement
3. Reset filter to that state
4. Predict and apply measurements chronologically up to target time
5. Final prediction to exact requested timestamp
6. Prune states/measurements older than specified max buffer length


## Build & Run
Check the example to see minimal usage.

```bash
git clone https://github.com/brow1633/measurement_manager.git
mkdir -p measurement_manager/build && cd measurement_manager/build
cmake .. && make
./example
``` 

## License

MIT License © 2025 Ethan Brown

---