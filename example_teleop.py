"""
Example script showing cross-platform robot control.
Works in both simulation and physical environments!
"""
from stretch_toolkit import controller, teleop, BACKEND_NAME
import time

print(f"\n=== Running on {BACKEND_NAME} backend ===\n")

def teleop_demo():
    """Run teleoperation loop."""
    print("Teleop demo started. Use gamepad/keyboard to control.")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            # Get normalized velocities from input devices
            velocities = teleop.get_normalized_velocities()
            
            # Send to robot (physical or simulated)
            controller.set_velocities(velocities)
            
            time.sleep(1/30)  # 30 Hz update rate
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        # Stop all motion
        controller.set_velocities({})
        controller.stop()
        print("Demo complete!")

if __name__ == "__main__":
    teleop_demo()
