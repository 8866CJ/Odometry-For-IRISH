"""Real-time robot position visualization using NetworkTables and pygame"""

import pygame
import math
import sys
import os
from networktables import NetworkTables

# Screen setup
SCREEN_WIDTH = 1340
SCREEN_HEIGHT = 670
FIELD_WIDTH_M = 16.46
PIXELS_PER_METER = 81.41

# Robot dimensions (AndyMark West Coast)
ROBOT_WIDTH_M = 0.61
ROBOT_LENGTH_M = 0.81
ROBOT_WIDTH_PX = ROBOT_WIDTH_M * PIXELS_PER_METER
ROBOT_LENGTH_PX = ROBOT_LENGTH_M * PIXELS_PER_METER

# Colors
ROBOT_COLOR = (0, 120, 255)
ROBOT_OUTLINE = (255, 255, 255)
ARROW_COLOR = (255, 0, 0)
VELOCITY_ARROW_COLOR = (0, 255, 0)
ROTATION_INDICATOR_COLOR = (200, 0, 255)
TEXT_COLOR = (0, 0, 0)

TARGET_FPS = 60
FIELD_IMAGE = "IrishField.png"


class RobotLocalizationDisplay:
    
    def __init__(self):
        # Connect to NetworkTables
        print("Initializing NetworkTables connection to 127.0.0.1...")
        NetworkTables.initialize(server='127.0.0.1')
        self.pose_table = NetworkTables.getTable('Pose')
        
        # Setup pygame window (resizable)
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption('Robot Localization Display - IRISH Field')
        self.clock = pygame.time.Clock()
        
        self.window_width = SCREEN_WIDTH
        self.window_height = SCREEN_HEIGHT
        self.aspect_ratio = SCREEN_WIDTH / SCREEN_HEIGHT
        
        # Load field image
        self.load_field_image()
        self.scaled_field_image = self.field_image
        
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        
        # Robot position and orientation
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        # Smoothed values for display
        self.smoothed_x = 0.0
        self.smoothed_y = 0.0
        self.smoothed_theta = 0.0
        
        # For velocity calculations
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.last_update_time = pygame.time.get_ticks() / 1000.0
        
        # Velocities
        self.robot_vx = 0.0
        self.robot_vy = 0.0
        self.robot_omega = 0.0
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_omega = 0.0
        
        self.prev_theta = 0.0
        self.show_telemetry = True
        self.running = True
        
        print(f"Display initialized: {SCREEN_WIDTH}x{SCREEN_HEIGHT} pixels")
        print(f"Scaling: {PIXELS_PER_METER:.2f} pixels/meter")
        print(f"Robot dimensions: {ROBOT_WIDTH_M}m x {ROBOT_LENGTH_M}m")
        print(f"Robot pixels: {ROBOT_WIDTH_PX:.1f}px x {ROBOT_LENGTH_PX:.1f}px")
    
    def load_field_image(self):
        # Try to load the field image, create placeholder if not found
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            image_path = os.path.join(script_dir, FIELD_IMAGE)
            
            if not os.path.exists(image_path):
                print(f"Warning: {FIELD_IMAGE} not found at {image_path}")
                print("Creating a placeholder field background...")
                self.field_image = self.create_placeholder_field()
            else:
                print(f"Loading field image: {image_path}")
                self.field_image = pygame.image.load(image_path)
                
                if self.field_image.get_size() != (SCREEN_WIDTH, SCREEN_HEIGHT):
                    print(f"Warning: Image size {self.field_image.get_size()} doesn't match expected {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
                    print("Scaling image to fit...")
                    self.field_image = pygame.transform.scale(self.field_image, (SCREEN_WIDTH, SCREEN_HEIGHT))
                else:
                    print(f"Field image loaded successfully: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")
                    
        except Exception as e:
            print(f"Error loading field image: {e}")
            print("Using placeholder field background...")
            self.field_image = self.create_placeholder_field()
    
    def create_placeholder_field(self):
        # Make a green field with basic markings
        surface = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
        surface.fill((34, 139, 34))
        pygame.draw.rect(surface, (255, 255, 255), (0, 0, SCREEN_WIDTH, SCREEN_HEIGHT), 3)
        center_y = SCREEN_HEIGHT // 2
        pygame.draw.line(surface, (255, 255, 255), (0, center_y), (SCREEN_WIDTH, center_y), 2)
        marker_size = 20
        for x, y in [(0, 0), (SCREEN_WIDTH, 0), (0, SCREEN_HEIGHT), (SCREEN_WIDTH, SCREEN_HEIGHT)]:
            pygame.draw.circle(surface, (255, 255, 0), (x, y), marker_size, 2)
        return surface
    
    def get_scale_factor(self):
        return self.window_width / SCREEN_WIDTH
    
    def meters_to_pixels(self, x_meters, y_meters):
        # Convert field coordinates (meters) to screen pixels
        scale = self.get_scale_factor()
        pixel_x = x_meters * PIXELS_PER_METER * scale
        pixel_y = self.window_height - (y_meters * PIXELS_PER_METER * scale)
        return pixel_x, pixel_y
    
    def clamp_to_field_bounds(self, x, y):
        # Keep robot within field boundaries
        field_min_x = 0.0
        field_max_x = FIELD_WIDTH_M
        field_min_y = 0.0
        field_max_y = FIELD_WIDTH_M * (SCREEN_HEIGHT / SCREEN_WIDTH)
        
        half_diagonal = math.sqrt(ROBOT_LENGTH_M**2 + ROBOT_WIDTH_M**2) / 2
        
        clamped_x = max(field_min_x + half_diagonal, min(field_max_x - half_diagonal, x))
        clamped_y = max(field_min_y + half_diagonal, min(field_max_y - half_diagonal, y))
        
        return clamped_x, clamped_y
    
    def update_pose_data(self):
        # Get latest data from NetworkTables
        current_time = pygame.time.get_ticks() / 1000.0
        dt = current_time - self.last_update_time
        
        raw_x = self.pose_table.getNumber('X', 0.0)
        raw_y = self.pose_table.getNumber('Y', 0.0)
        new_theta = self.pose_table.getNumber('Theta', 0.0)
        
        new_x, new_y = self.clamp_to_field_bounds(raw_x, raw_y)
        
        # Calculate velocities from position changes
        if dt > 0.001:
            self.robot_vx = (new_x - self.robot_x) / dt
            self.robot_vy = (new_y - self.robot_y) / dt
            
            # Calculate angular velocity (handle wraparound)
            delta_theta = new_theta - self.prev_theta
            while delta_theta > math.pi:
                delta_theta -= 2 * math.pi
            while delta_theta < -math.pi:
                delta_theta += 2 * math.pi
            self.robot_omega = delta_theta / dt
        
        self.robot_x = new_x
        self.robot_y = new_y
        self.robot_theta = new_theta
        self.prev_theta = new_theta
        self.last_update_time = current_time
        
        # Override with NetworkTables velocities if available
        nt_vx = self.pose_table.getNumber('VX', None)
        nt_vy = self.pose_table.getNumber('VY', None)
        nt_omega = self.pose_table.getNumber('Omega', None)
        if nt_vx is not None and nt_vy is not None:
            self.robot_vx = nt_vx
            self.robot_vy = nt_vy
        if nt_omega is not None:
            self.robot_omega = nt_omega
        
        # Apply smoothing to prevent jitter
        smoothing_factor = 0.3
        self.smoothed_x = (smoothing_factor * self.robot_x + (1 - smoothing_factor) * self.smoothed_x)
        self.smoothed_y = (smoothing_factor * self.robot_y + (1 - smoothing_factor) * self.smoothed_y)
        
        # Smooth theta with wraparound
        theta_diff = new_theta - self.smoothed_theta
        while theta_diff > math.pi:
            theta_diff -= 2 * math.pi
        while theta_diff < -math.pi:
            theta_diff += 2 * math.pi
        self.smoothed_theta = self.smoothed_theta + smoothing_factor * theta_diff
        
        self.smoothed_vx = (smoothing_factor * self.robot_vx + (1 - smoothing_factor) * self.smoothed_vx)
        self.smoothed_vy = (smoothing_factor * self.robot_vy + (1 - smoothing_factor) * self.smoothed_vy)
        self.smoothed_omega = (smoothing_factor * self.robot_omega + (1 - smoothing_factor) * self.smoothed_omega)
    
    def draw_field(self):
        self.screen.blit(self.scaled_field_image, (0, 0))
    
    def draw_robot(self):
        # Draw robot rectangle at current position
        pixel_x, pixel_y = self.meters_to_pixels(self.smoothed_x, self.smoothed_y)
        
        # Clamp to screen bounds for visibility (with margin)
        scale = self.get_scale_factor()
        margin = max(ROBOT_WIDTH_PX, ROBOT_LENGTH_PX) * scale
        pixel_x = max(-margin, min(self.window_width + margin, pixel_x))
        pixel_y = max(-margin, min(self.window_height + margin, pixel_y))
        
        # Create robot rectangle (centered at origin before rotation)
        # Length is along the "forward" direction (local X-axis of robot)
        # Width is perpendicular (local Y-axis of robot)
        half_length = ROBOT_LENGTH_PX * scale / 2
        half_width = ROBOT_WIDTH_PX * scale / 2
        
        # Define rectangle corners in robot's local frame
        # Front of robot is in +X direction (along length)
        local_corners = [
            (-half_length, -half_width),  # Back-left
            (half_length, -half_width),   # Front-left
            (half_length, half_width),    # Front-right
            (-half_length, half_width)    # Back-right
        ]
        
        # Rotate corners by theta
        # Note: Screen Y is inverted, so we negate theta for proper rotation
        # theta = 0 → robot faces right (+X)
        # theta = π/2 → robot faces up (+Y on field, -Y on screen)
        rotated_corners = []
        cos_theta = math.cos(-self.smoothed_theta)
        sin_theta = math.sin(-self.smoothed_theta)
        
        for local_x, local_y in local_corners:
            # Apply rotation matrix
            rotated_x = local_x * cos_theta - local_y * sin_theta
            rotated_y = local_x * sin_theta + local_y * cos_theta
            
            # Translate to screen position
            screen_x = pixel_x + rotated_x
            screen_y = pixel_y + rotated_y
            rotated_corners.append((screen_x, screen_y))
        
        # Draw robot body
        pygame.draw.polygon(self.screen, ROBOT_COLOR, rotated_corners)
        pygame.draw.polygon(self.screen, ROBOT_OUTLINE, rotated_corners, 2)
        
        # Draw heading arrow from center to front
        scale = self.get_scale_factor()
        arrow_length = ROBOT_LENGTH_PX * 0.7 * scale
        arrow_end_x = pixel_x + arrow_length * math.cos(-self.smoothed_theta)
        arrow_end_y = pixel_y + arrow_length * math.sin(-self.smoothed_theta)
        
        # Draw arrow shaft
        line_width = max(1, int(4 * scale))
        pygame.draw.line(self.screen, ARROW_COLOR, 
                        (pixel_x, pixel_y), (arrow_end_x, arrow_end_y), line_width)
        
        # Draw arrowhead
        arrow_head_size = 12 * scale
        arrow_angle1 = -self.smoothed_theta + math.pi * 0.75
        arrow_angle2 = -self.smoothed_theta - math.pi * 0.75
        
        head_point1_x = arrow_end_x + arrow_head_size * math.cos(arrow_angle1)
        head_point1_y = arrow_end_y + arrow_head_size * math.sin(arrow_angle1)
        head_point2_x = arrow_end_x + arrow_head_size * math.cos(arrow_angle2)
        head_point2_y = arrow_end_y + arrow_head_size * math.sin(arrow_angle2)
        
        pygame.draw.line(self.screen, ARROW_COLOR, 
                        (arrow_end_x, arrow_end_y), (head_point1_x, head_point1_y), line_width)
        pygame.draw.line(self.screen, ARROW_COLOR, 
                        (arrow_end_x, arrow_end_y), (head_point2_x, head_point2_y), line_width)
        
        # Draw velocity arrow
        self.draw_velocity_arrow(pixel_x, pixel_y)
        
        # Draw rotation indicator
        self.draw_rotation_indicator(pixel_x, pixel_y)
        
        # Draw center point for reference
        center_radius = max(1, int(5 * scale))
        pygame.draw.circle(self.screen, (255, 255, 0), (int(pixel_x), int(pixel_y)), center_radius)
    
    def draw_velocity_arrow(self, pixel_x, pixel_y):
        """
        Draw velocity arrow that scales with speed.
        
        Args:
            pixel_x: Robot center X position in pixels
            pixel_y: Robot center Y position in pixels
        """
        # Calculate velocity magnitude in m/s using smoothed values
        velocity_magnitude = math.sqrt(self.smoothed_vx**2 + self.smoothed_vy**2)
        
        # Only draw if there's significant velocity (> 0.01 m/s)
        if velocity_magnitude < 0.01:
            return
        
        # Calculate velocity direction
        velocity_angle = math.atan2(self.smoothed_vy, self.smoothed_vx)
        
        # Scale arrow length based on velocity (pixels per m/s)
        # Make it visible: 100 pixels per 1 m/s, capped at 300 pixels
        scale = self.get_scale_factor()
        arrow_scale = 100 * scale  # pixels per m/s
        max_arrow_length = 300 * scale  # maximum arrow length in pixels
        velocity_arrow_length = min(velocity_magnitude * arrow_scale, max_arrow_length)
        
        # Calculate arrow end point (negate angle for screen coordinates)
        vel_end_x = pixel_x + velocity_arrow_length * math.cos(-velocity_angle)
        vel_end_y = pixel_y + velocity_arrow_length * math.sin(-velocity_angle)
        
        # Draw velocity arrow shaft
        line_width = max(1, int(5 * scale))
        pygame.draw.line(self.screen, VELOCITY_ARROW_COLOR, 
                        (pixel_x, pixel_y), (vel_end_x, vel_end_y), line_width)
        
        # Draw velocity arrowhead
        vel_head_size = 15 * scale
        vel_angle1 = -velocity_angle + math.pi * 0.75
        vel_angle2 = -velocity_angle - math.pi * 0.75
        
        vel_head1_x = vel_end_x + vel_head_size * math.cos(vel_angle1)
        vel_head1_y = vel_end_y + vel_head_size * math.sin(vel_angle1)
        vel_head2_x = vel_end_x + vel_head_size * math.cos(vel_angle2)
        vel_head2_y = vel_end_y + vel_head_size * math.sin(vel_angle2)
        
        pygame.draw.line(self.screen, VELOCITY_ARROW_COLOR, 
                        (vel_end_x, vel_end_y), (vel_head1_x, vel_head1_y), line_width)
        pygame.draw.line(self.screen, VELOCITY_ARROW_COLOR, 
                        (vel_end_x, vel_end_y), (vel_head2_x, vel_head2_y), line_width)
        
        # Draw velocity magnitude label near the arrow
        if velocity_magnitude > 0.1:  # Only show text for significant velocities
            vel_text = self.small_font.render(f"{velocity_magnitude:.2f} m/s", True, VELOCITY_ARROW_COLOR)
            text_offset_x = 10
            text_offset_y = -20
            self.screen.blit(vel_text, (int(vel_end_x + text_offset_x), int(vel_end_y + text_offset_y)))
    
    def draw_rotation_indicator(self, pixel_x, pixel_y):
        """
        Draw arrow to indicate rotation/angular velocity.
        Purple arrow perpendicular to heading shows direction and speed of rotation.
        
        Args:
            pixel_x: Robot center X position in pixels
            pixel_y: Robot center Y position in pixels
        """
        # Only draw if there's significant rotation (> 0.1 rad/s ≈ 6 deg/s)
        if abs(self.smoothed_omega) < 0.1:
            return
        
        # Arrow points perpendicular to robot's heading
        # Positive omega (CCW) = arrow points left (90° from heading)
        # Negative omega (CW) = arrow points right (-90° from heading)
        perp_angle = self.smoothed_theta + (math.pi / 2 if self.smoothed_omega > 0 else -math.pi / 2)
        
        # Scale arrow length based on rotation speed (reduced for relative display)
        # Much smaller scale for gimmick/relative indication
        scale = self.get_scale_factor()
        arrow_scale = 30 * scale  # pixels per rad/s (was 80)
        max_arrow_length = 120 * scale  # maximum arrow length (was 250)
        rotation_arrow_length = min(abs(self.smoothed_omega) * arrow_scale, max_arrow_length)
        
        # Calculate arrow end point (negate angle for screen coordinates)
        rot_end_x = pixel_x + rotation_arrow_length * math.cos(-perp_angle)
        rot_end_y = pixel_y + rotation_arrow_length * math.sin(-perp_angle)
        
        # Draw rotation arrow shaft (purple)
        line_width = max(1, int(5 * scale))
        pygame.draw.line(self.screen, ROTATION_INDICATOR_COLOR, 
                        (pixel_x, pixel_y), (rot_end_x, rot_end_y), line_width)
        
        # Draw arrowhead
        rot_head_size = 15 * scale
        rot_angle1 = -perp_angle + math.pi * 0.75
        rot_angle2 = -perp_angle - math.pi * 0.75
        
        rot_head1_x = rot_end_x + rot_head_size * math.cos(rot_angle1)
        rot_head1_y = rot_end_y + rot_head_size * math.sin(rot_angle1)
        rot_head2_x = rot_end_x + rot_head_size * math.cos(rot_angle2)
        rot_head2_y = rot_end_y + rot_head_size * math.sin(rot_angle2)
        
        pygame.draw.line(self.screen, ROTATION_INDICATOR_COLOR, 
                        (rot_end_x, rot_end_y), (rot_head1_x, rot_head1_y), line_width)
        pygame.draw.line(self.screen, ROTATION_INDICATOR_COLOR, 
                        (rot_end_x, rot_end_y), (rot_head2_x, rot_head2_y), line_width)
        
        # Draw rotation speed label (optional - only for higher speeds)
        omega_deg_per_sec = abs(math.degrees(self.smoothed_omega))
        if omega_deg_per_sec > 15:  # Only show for significant rotation
            direction = "CCW" if self.smoothed_omega > 0 else "CW"
            rot_text = self.small_font.render(f"{omega_deg_per_sec:.0f}°/s {direction}", True, ROTATION_INDICATOR_COLOR)
            text_offset_x = 10
            text_offset_y = -20
            self.screen.blit(rot_text, (int(rot_end_x + text_offset_x), int(rot_end_y + text_offset_y)))
    
    def draw_telemetry(self):
        """Draw telemetry text overlay showing current pose values."""
        if not self.show_telemetry:
            # Still show the toggle hint even when hidden
            hint_text = self.small_font.render("Press 'H' to show telemetry", True, (150, 150, 150))
            hint_bg = pygame.Surface((hint_text.get_width() + 20, hint_text.get_height() + 10))
            hint_bg.set_alpha(150)
            self.screen.blit(hint_bg, (10, 10))
            self.screen.blit(hint_text, (20, 15))
            return
        
        # Convert theta to degrees (use actual value for telemetry)
        theta_degrees = math.degrees(self.robot_theta) % 360
        
        # Calculate velocity magnitude using smoothed values
        velocity_magnitude = math.sqrt(self.smoothed_vx**2 + self.smoothed_vy**2)
        
        # Create telemetry text
        telemetry_lines = [
            f"X: {self.robot_x:.3f} m",
            f"Y: {self.robot_y:.3f} m",
            f"Theta: {theta_degrees:.2f}°",
            f"Velocity: {velocity_magnitude:.3f} m/s",
            f"Rotational Velocity: {math.degrees(self.robot_omega):.2f} °/s"
        ]
        
        
        # Draw text lines
        y_offset = 20
        for line in telemetry_lines:
            text_surface = self.font.render(line, True, TEXT_COLOR)
            self.screen.blit(text_surface, (20, y_offset))
            y_offset += 35
        
        # Draw toggle hint
        hint_text = self.small_font.render("Press 'H' to hide", True, (180, 180, 180))
        self.screen.blit(hint_text, (20, y_offset))
        
        # Draw FPS counter
        fps = self.clock.get_fps()
        fps_text = self.small_font.render(f"FPS: {fps:.1f}", True, TEXT_COLOR)
        self.screen.blit(fps_text, (SCREEN_WIDTH - 100, 20))
        
        # Draw scaling info
        scale_info = self.small_font.render(
            f"Scale: {PIXELS_PER_METER:.2f} px/m", 
            True, (200, 200, 200)
        )
        self.screen.blit(scale_info, (SCREEN_WIDTH - 180, SCREEN_HEIGHT - 30))
    
    def handle_events(self):
        """Process pygame events."""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_h:
                    # Toggle telemetry overlay
                    self.show_telemetry = not self.show_telemetry
                    status = "shown" if self.show_telemetry else "hidden"
                    print(f"Telemetry overlay {status}")
            elif event.type == pygame.VIDEORESIZE:
                # Handle window resize while maintaining aspect ratio
                new_width = event.w
                new_height = event.h
                
                # Calculate dimensions that maintain aspect ratio
                # Fit to width
                fit_width_height = int(new_width / self.aspect_ratio)
                # Fit to height
                fit_height_width = int(new_height * self.aspect_ratio)
                
                # Choose the fit that keeps both dimensions within the new window
                if fit_width_height <= new_height:
                    # Width-constrained
                    self.window_width = new_width
                    self.window_height = fit_width_height
                else:
                    # Height-constrained
                    self.window_width = fit_height_width
                    self.window_height = new_height
                
                # Recreate the display with new size
                self.screen = pygame.display.set_mode((self.window_width, self.window_height), pygame.RESIZABLE)
                
                # Scale the field image to new size
                self.scaled_field_image = pygame.transform.scale(self.field_image, 
                                                                  (self.window_width, self.window_height))
    
    def run(self):
        """Main application loop."""
        print("\n" + "=" * 60)
        print("Robot Localization Display - IRISH Field")
        print("=" * 60)
        print(f"NetworkTables: 127.0.0.1 → Table 'Pose' (X, Y, Theta)")
        print(f"Field: {FIELD_WIDTH_M}m → {SCREEN_WIDTH}px ({PIXELS_PER_METER:.2f} px/m)")
        print(f"Origin: (0,0) meters → bottom-left of image")
        print(f"Controls: ESC or close window to quit")
        print("=" * 60 + "\n")
        
        while self.running:
            # Handle events
            self.handle_events()
            
            # Update robot pose from NetworkTables
            self.update_pose_data()
            
            # Draw everything
            self.draw_field()
            self.draw_robot()
            self.draw_telemetry()
            
            # Update display
            pygame.display.flip()
            
            # Maintain target FPS
            self.clock.tick(TARGET_FPS)
        
        # Cleanup
        pygame.quit()
        print("\nApplication closed")


def main():
    """Entry point for the application."""
    try:
        app = RobotLocalizationDisplay()
        app.run()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        pygame.quit()
        sys.exit(0)
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        pygame.quit()
        sys.exit(1)


if __name__ == "__main__":
    main()
