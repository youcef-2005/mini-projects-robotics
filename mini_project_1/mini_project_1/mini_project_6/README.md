# Mini-Project 6: ROS 2 Multi-Threading Performance Test

## Overview
This project demonstrates advanced concurrency management in ROS 2. It proves how to prevent a slow, resource-intensive callback from blocking a high-frequency, time-critical task within the same Node.

## Key Technical Concepts
* **Default Behavior (Single Threaded):** By default, ROS 2 processes callbacks one by one. A slow callback (e.g., image processing) will delay all others, causing critical failures (like watchdog timeouts on motors).
* **MultiThreadedExecutor:** This executor uses multiple CPU threads (4 threads in this case) to allow different callbacks to run at the same time.
* **ReentrantCallbackGroup:** This group tells ROS 2 that callbacks from this group can be started even if another callback is already running.

## Scenario
The node has two timer callbacks:
1.  **Fast Callback:** Runs at 20 Hz (every 50ms) to simulate a vital system tick (like a control loop).
2.  **Slow Callback:** Runs at 2 Hz (every 500ms) and includes a `time.sleep(0.4)` to simulate a heavy workload (400ms).

With this implementation, the **"Fast tick"** logs are published every 50ms consistently, completely unaffected by the 400ms delays of the slow callback.
