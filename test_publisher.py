"""
Test NetworkTables Publisher
Publishes simulated robot pose data for testing the localization display.
"""

from networktables import NetworkTables
import time
import math

def main():
    """Publish test pose data to NetworkTables."""
    print("Initializing NetworkTables client connection to localhost...")
    
    # Initialize as a client connecting to localhost
    # Note: You need to have a NetworkTables server running, OR
    # we can start in server mode here
    import logging
    logging.basicConfig(level=logging.INFO)
    
    # Start NetworkTables in server mode
    NetworkTables.initialize()
    NetworkTables.setServer("localhost")
    
    # Wait for connection
    print("Starting NetworkTables server mode...")
    time.sleep(0.5)
    
    pose_table = NetworkTables.getTable('Pose')
    
    print("\n" + "=" * 60)
    print("NetworkTables Test Publisher - Running in SERVER mode")
    print("=" * 60)
    print("Publishing test pose data to 'Pose' table")
    print("The robot will follow a circular path on the field")
    print("Field dimensions: 16.46m x 8.23m")
    print("Press Ctrl+C to stop")
    print("=" * 60 + "\n")
    
    # Wait a moment for the server to stabilize
    time.sleep(1.0)
    
    t = 0
    dt = 0.02  # time step
    angular_velocity = 0.05  # radians per time step
    
    try:
        while True:
            # Generate circular motion centered on field
            # Field is 16.46m x 8.23m, so center is at (8.23, 4.115)
            center_x = 8.23
            center_y = 4.115
            radius_x = 3.0
            radius_y = 2.0
            
            # Position: x(t) = center_x + radius_x * cos(t)
            #           y(t) = center_y + radius_y * sin(t)
            x = center_x + radius_x * math.cos(t)
            y = center_y + radius_y * math.sin(t)
            theta = t + math.pi / 2  # Tangent to the path
            
            # Velocity is the derivative of position with respect to time
            # dx/dt = -radius_x * sin(t) * dt/dt = -radius_x * sin(t) * angular_velocity
            # dy/dt = radius_y * cos(t) * dt/dt = radius_y * cos(t) * angular_velocity
            vx = -radius_x * math.sin(t) * angular_velocity / dt
            vy = radius_y * math.cos(t) * angular_velocity / dt
            velocity_magnitude = math.sqrt(vx**2 + vy**2)
            
            # Publish to NetworkTables
            pose_table.putNumber('X', x)
            pose_table.putNumber('Y', y)
            pose_table.putNumber('Theta', theta)
            pose_table.putNumber('VX', vx)
            pose_table.putNumber('VY', vy)
            
            # Force update
            NetworkTables.flush()
            
            # Print current values
            theta_deg = math.degrees(theta) % 360
            print(f"Publishing: X={x:6.3f}m | Y={y:6.3f}m | Theta={theta_deg:6.2f}° | V={velocity_magnitude:5.3f}m/s", end='\r')
            
            # Update at 50 Hz
            time.sleep(dt)
            t += angular_velocity
            
    except KeyboardInterrupt:
        print("\n\nStopped by user")
        print("NetworkTables server shutting down...")

if __name__ == "__main__":
    main()
