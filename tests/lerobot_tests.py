import time
# lerobot 0.5.x: SO-101 lives in lerobot.robots.so_follower
# (older versions used lerobot.common.robots.so101_follower)
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower

# 1. Define the hardware configuration for your arm '1/u
config = SO101FollowerConfig(
    port="COM3",
    id="jlo",
    use_degrees=True  # Returns angles in degrees; set to False for radians
)

# 2. Instantiate your robot objectq
robot = SO101Follower(config)

print(f"Connecting to {config.id} on {config.port}...")

# 3. Use the context manager to open/close the port automatically
with robot:
    # connect() leaves torque ENABLED (motors stiff). Disable it so the arm
    # can be held/posed freely by hand while it readsq back its position.
    robot.bus.disable_torque()

    print("Connected! Torque disabled - hold the arm in its homeq pose.")
    print("Reading observations (Press Ctrl+C to stop)...")

    try:
        while True:
            # 4. Call get_observation() to fetch the dictionaryq of positions
            obs = robot.get_observation()
            
            print("\n--- Current State ---")
            print(obs)
            
            # Read at roughly 10Hz to avoid flooding the console
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping observation stream.")

print("Robot cleanly disconnected.")