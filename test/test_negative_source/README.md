# ros2_tf_stresser

This stresser program tests throughput on ROS2 tf messages. This is a minimal program to reproduce DDS traffic capacity problems I was having.
In my case, I had two computers, A and B, connected over wifi.

1. On computer A, execute `ros2 run tf2_ros tf2_monitor`. This sets up a reliable listener, which will cause DDS backpressure on the publisher.
2. On computer A, networked over wifi, execute `ros2 topic hz /tf --window 4000` - this sets up a best effort subscriber and reports the rate of messages.
3. On computer B, execute `ros2 run tf_stresser stresser`. This publishes the messages at a rate that should be higher than the DDS implementation can handle.
4. On computer A, when the window is full (window: 4000), read off the average rate. This is the score. Higher is better.

Note the scores are pretty noisy. Depending on WiFi interference and network traffic, they can vary pretty wildly, so make sure to run consecutive tests as near as possible to each other.