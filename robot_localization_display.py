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

# Limelight Funky Offsets (Simulating Limelight view and orientation)
VISION_FOV_H = 62.5  # degrees
VISION_FOV_V = 48.9  # degrees
VISION_MAX_DISTANCE_M = 1.85  # 5 feet in meters (realistic for Limelight 2)
TURRET_SMOOTHING = 0.15  # Smoothing factor for turret rotation


# Colors
ROBOT_COLOR = (0, 120, 255)
ROBOT_OUTLINE = (255, 255, 255)
LIMELIGHT_CONE = (50, 205, 50, 100)  # Lime green with transparency
TURRET_ARROW_COLOR = (245, 135, 66)
ARROW_COLOR = (255, 0, 0)
VELOCITY_ARROW_COLOR = (0, 255, 0)
ROTATION_INDICATOR_COLOR = (200, 0, 255)
TEXT_COLOR = (0, 0, 0)
MENU_BG = (40, 40, 40)
MENU_TEXT = (255, 255, 255)
BUTTON_BG = (70, 70, 70)
BUTTON_HOVER = (100, 100, 100)
BUTTON_SELECTED = (0, 150, 255)
APRILTAG_COLOR = (255, 255, 0)
APRILTAG_BORDER = (0, 0, 0)

TARGET_FPS = 60
FIELD_IMAGE = "IrishField.png"

# AprilTag configuration (simulated targets in corners)
APRILTAG_SIZE_M = 0.2  # 20cm AprilTags
APRILTAG_MARGIN_M = 0.3  # Distance from corner


class RobotLocalizationDisplay:
    
    def __init__(self):
        # Connect to NetworkTables
        print("Initializing NetworkTables connection to 127.0.0.1...")
        NetworkTables.initialize(server='127.0.0.1')
        self.pose_table = NetworkTables.getTable('Pose')
        self.vision_table = NetworkTables.getTable('Vision')
        self.smart_dashboard = NetworkTables.getTable('SmartDashboard')
        
        # Setup pygame window (fixed size)
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption('Robot Localization Display - IRISH Field')
        self.clock = pygame.time.Clock()
        
        self.window_width = SCREEN_WIDTH
        self.window_height = SCREEN_HEIGHT
        self.aspect_ratio = SCREEN_WIDTH / SCREEN_HEIGHT
        
        # Load field image
        self.load_field_image()
        self.scaled_field_image_red = self.field_image_red
        self.scaled_field_image_blue = self.field_image_blue
        
        # Track field image position for letterboxing
        self.field_offset_x = 0
        self.field_offset_y = 0
        self.field_draw_width = SCREEN_WIDTH
        self.field_draw_height = SCREEN_HEIGHT
        
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

        # Turret and Vision tracking
        self.turret_angle = 0.0
        self.smoothed_turret_angle = 0.0
        self.has_target = False
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.target_area = 0.0
        self.target_id = 0.0
        self.detected_target = None  # Store detected target for drawing
        
        self.prev_theta = 0.0
        self.show_telemetry = True
        self.running = True
        
        # Menu and setup state
        self.menu_active = True
        self.alliance = None  # 'red' or 'blue'
        self.simulation_active = False
        
        # AprilTag positions (will be initialized when alliance is selected)
        self.apriltag_positions = None
        
        # Field image variants
        self.field_image_red = None
        self.field_image_blue = None
        
        print(f"Display initialized: {SCREEN_WIDTH}x{SCREEN_HEIGHT} pixels")
        print(f"Scaling: {PIXELS_PER_METER:.2f} pixels/meter")
        print(f"Robot dimensions: {ROBOT_WIDTH_M}m x {ROBOT_LENGTH_M}m")
        print(f"Robot pixels: {ROBOT_WIDTH_PX:.1f}px x {ROBOT_LENGTH_PX:.1f}px")
        print("\nPlease select your alliance in the menu to begin...")
    
    def initialize_apriltags(self):
        """Initialize AprilTag positions in field coordinates (meters)."""
        field_height_m = FIELD_WIDTH_M * (SCREEN_HEIGHT / SCREEN_WIDTH)

        # Red alliance: Tag 1 on left, Tag 2 on right
        # Blue alliance: Tag 2 on left, Tag 1 on right (IDs swap)
        if self.alliance == 'blue':
            tags = {
                'bottom_left': {
                    'id': 2,
                    'x': APRILTAG_MARGIN_M,
                    'y': APRILTAG_MARGIN_M,
                    'size': APRILTAG_SIZE_M
                },
                'bottom_right': {
                    'id': 1,
                    'x': FIELD_WIDTH_M - APRILTAG_MARGIN_M,
                    'y': APRILTAG_MARGIN_M,
                    'size': APRILTAG_SIZE_M
                }
            }
        else:
            tags = {
                'bottom_left': {
                    'id': 1,
                    'x': APRILTAG_MARGIN_M,
                    'y': APRILTAG_MARGIN_M,
                    'size': APRILTAG_SIZE_M
                },
                'bottom_right': {
                    'id': 2,
                    'x': FIELD_WIDTH_M - APRILTAG_MARGIN_M,
                    'y': APRILTAG_MARGIN_M,
                    'size': APRILTAG_SIZE_M
                }
            }
        
        return tags
    
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
            
            # Create red alliance version (original)
            self.field_image_red = self.field_image.copy()
            # Create blue alliance version (horizontally flipped)
            self.field_image_blue = pygame.transform.flip(self.field_image, True, False)
            print(f"Field variants created: red={self.field_image_red is not None}, blue={self.field_image_blue is not None}")
                    
        except Exception as e:
            print(f"Error loading field image: {e}")
            print("Using placeholder field background...")
            self.field_image = self.create_placeholder_field()
            self.field_image_red = self.field_image.copy()
            self.field_image_blue = pygame.transform.flip(self.field_image, True, False)
    
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
        return self.field_draw_width / SCREEN_WIDTH
    
    def meters_to_pixels(self, x_meters, y_meters):
        # Convert field coordinates (meters) to screen pixels
        scale = self.get_scale_factor()
        pixel_x = (x_meters * PIXELS_PER_METER * scale) + self.field_offset_x
        pixel_y = (self.field_draw_height - (y_meters * PIXELS_PER_METER * scale)) + self.field_offset_y
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
        
        # Only update from NetworkTables if simulation is active
        if self.simulation_active:
            # Get robot code's coordinates directly (no offset)
            raw_x = self.pose_table.getNumber('X', 0.5)
            raw_y = self.pose_table.getNumber('Y', 0.5)
            new_theta = self.pose_table.getNumber('Theta', 0.0)
        else:
            # Keep current position when simulation is not active
            raw_x = self.robot_x
            raw_y = self.robot_y
            new_theta = self.robot_theta
        
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
        
        # Update turret data from SmartDashboard (published by robot code)
        new_turret_angle = self.smart_dashboard.getNumber('Turret Angle', 0.0)
        
        # Smooth turret angle to prevent snappy movement
        turret_diff = new_turret_angle - self.smoothed_turret_angle
        # Handle wraparound for angles  
        while turret_diff > 180:
            turret_diff -= 360
        while turret_diff < -180:
            turret_diff += 360
        self.smoothed_turret_angle = self.smoothed_turret_angle + TURRET_SMOOTHING * turret_diff
        self.turret_angle = new_turret_angle  # Keep raw value for logic
        
        # Read vision data that robot code published (when it had a target)
        self.has_target = self.vision_table.getBoolean('HasTarget', False)
        self.target_yaw = self.vision_table.getNumber('Target_Yaw', 0.0)
        self.target_pitch = self.vision_table.getNumber('Target_Pitch', 0.0)
        self.target_area = self.vision_table.getNumber('Target_Area', 0.0)
        self.target_id = self.vision_table.getNumber('Target_ID', -1)
    
    def draw_field(self):
        # Fill background (for letterboxing)
        self.screen.fill((0, 0, 0))
        
        # Use appropriate field image based on alliance, positioned with offset
        if self.alliance == 'blue':
            self.screen.blit(self.scaled_field_image_blue, (self.field_offset_x, self.field_offset_y))
        else:
            self.screen.blit(self.scaled_field_image_red, (self.field_offset_x, self.field_offset_y))
        
        # Draw AprilTags
        self.draw_apriltags()
    
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
        
        # Draw turret angle indicator
        self.draw_turret_indicator(pixel_x, pixel_y)
    
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
    
    def draw_turret_indicator(self, pixel_x, pixel_y):
        """
        Draw turret angle indicator showing turret orientation relative to robot.
        Orange arrow shows turret direction relative to robot heading.
        
        Args:
            pixel_x: Robot center X position in pixels
            pixel_y: Robot center Y position in pixels
        """
        # Calculate absolute turret angle in radians using SMOOTHED turret angle
        # turret_angle is relative to robot's heading
        # We need to add robot's theta to get world frame angle
        turret_world_angle = self.smoothed_theta + math.radians(self.smoothed_turret_angle)
        
        # Scale arrow length
        scale = self.get_scale_factor()
        turret_arrow_length = ROBOT_LENGTH_PX * 0.6 * scale  # Slightly shorter than heading arrow
        
        # Calculate arrow end point (negate angle for screen coordinates)
        turret_end_x = pixel_x + turret_arrow_length * math.cos(-turret_world_angle)
        turret_end_y = pixel_y + turret_arrow_length * math.sin(-turret_world_angle)
        
        # Draw turret arrow shaft (orange)
        line_width = max(1, int(3 * scale))
        pygame.draw.line(self.screen, TURRET_ARROW_COLOR, 
                        (pixel_x, pixel_y), (turret_end_x, turret_end_y), line_width)
        
        # Draw arrowhead
        turret_head_size = 10 * scale
        turret_angle1 = -turret_world_angle + math.pi * 0.75
        turret_angle2 = -turret_world_angle - math.pi * 0.75
        
        turret_head1_x = turret_end_x + turret_head_size * math.cos(turret_angle1)
        turret_head1_y = turret_end_y + turret_head_size * math.sin(turret_angle1)
        turret_head2_x = turret_end_x + turret_head_size * math.cos(turret_angle2)
        turret_head2_y = turret_end_y + turret_head_size * math.sin(turret_angle2)
        
        pygame.draw.line(self.screen, TURRET_ARROW_COLOR, 
                        (turret_end_x, turret_end_y), (turret_head1_x, turret_head1_y), line_width)
        pygame.draw.line(self.screen, TURRET_ARROW_COLOR, 
                        (turret_end_x, turret_end_y), (turret_head2_x, turret_head2_y), line_width)
        
        # Draw line to detected target if present
        if self.detected_target:
            self.draw_target_line(pixel_x, pixel_y)
        
        # Draw FOV cone attached to turret
        self.draw_fov_cone(pixel_x, pixel_y, turret_world_angle)
    
    def draw_target_line(self, pixel_x, pixel_y):
        """
        Draw a line from turret to the detected AprilTag showing distance and angle.
        
        Args:
            pixel_x: Robot center X position in pixels
            pixel_y: Robot center Y position in pixels
        """
        if not self.detected_target:
            return
        
        scale = self.get_scale_factor()
        target = self.detected_target
        
        # Find the actual AprilTag position from apriltag_positions
        tag_data = None
        for tag_key, data in self.apriltag_positions.items():
            if data['id'] == target['id']:
                tag_data = data
                break
        
        if not tag_data:
            return
        
        # Get target position in pixels
        target_pixel_x, target_pixel_y = self.meters_to_pixels(tag_data['x'], tag_data['y'])
        
        # Draw line from robot to target (bright cyan)
        line_width = max(2, int(3 * scale))
        pygame.draw.line(self.screen, (0, 255, 255), 
                        (int(pixel_x), int(pixel_y)), 
                        (int(target_pixel_x), int(target_pixel_y)), 
                        line_width)
        
        # Draw distance and angle text at midpoint
        mid_x = (pixel_x + target_pixel_x) / 2
        mid_y = (pixel_y + target_pixel_y) / 2
        
        distance_text = self.small_font.render(
            f"{target['distance']:.2f}m | {target['yaw']:.1f}°",
            True, (255, 255, 0)
        )
        # Background for text
        text_bg = pygame.Surface((distance_text.get_width() + 10, distance_text.get_height() + 4))
        text_bg.fill((0, 0, 0))
        text_bg.set_alpha(200)
        self.screen.blit(text_bg, (int(mid_x - distance_text.get_width()/2 - 5), int(mid_y - distance_text.get_height()/2 - 2)))
        self.screen.blit(distance_text, (int(mid_x - distance_text.get_width()/2), int(mid_y - distance_text.get_height()/2)))
        
        # Draw small circle at target
        pygame.draw.circle(self.screen, (0, 255, 255), 
                         (int(target_pixel_x), int(target_pixel_y)), 
                         max(4, int(8 * scale)), max(2, int(3 * scale)))
    
    def draw_fov_cone(self, pixel_x, pixel_y, turret_world_angle):

        scale = self.get_scale_factor()
        
        # FOV cone visual parameters - match actual detection range
        fov_visual_length = VISION_MAX_DISTANCE_M * PIXELS_PER_METER * scale  
        fov_angle_rad = math.radians(VISION_FOV_H / 2)  # Half of horizontal FOV in radians
        
        # Calculate the two edges of the FOV cone
        # Left edge
        left_angle = turret_world_angle + fov_angle_rad
        left_end_x = pixel_x + fov_visual_length * math.cos(-left_angle)
        left_end_y = pixel_y + fov_visual_length * math.sin(-left_angle)
        
        # Right edge
        right_angle = turret_world_angle - fov_angle_rad
        right_end_x = pixel_x + fov_visual_length * math.cos(-right_angle)
        right_end_y = pixel_y + fov_visual_length * math.sin(-right_angle)
        
        # Create points for the triangle (FOV cone)
        points = [
            (pixel_x, pixel_y),
            (left_end_x, left_end_y),
            (right_end_x, right_end_y)
        ]
        
        # Create a surface with per-pixel alpha for transparency
        fov_surface = pygame.Surface((self.window_width, self.window_height), pygame.SRCALPHA)
        
        # Draw the semi-transparent cone
        pygame.draw.polygon(fov_surface, LIMELIGHT_CONE, points)
        
        # Draw the cone edges (more visible)
        pygame.draw.line(fov_surface, (50, 255, 50, 180), points[0], points[1], max(1, int(2 * scale)))
        pygame.draw.line(fov_surface, (50, 255, 50, 180), points[0], points[2], max(1, int(2 * scale)))
        
        # Blit the FOV surface onto the main screen
        self.screen.blit(fov_surface, (0, 0))
    
    def draw_apriltags(self):
        """Draw simulated AprilTags in the bottom corners of the field."""
        if self.apriltag_positions is None:
            return
            
        scale = self.get_scale_factor()
        
        for tag_key, tag_data in self.apriltag_positions.items():
            # Convert tag position to pixels
            tag_x, tag_y = self.meters_to_pixels(tag_data['x'], tag_data['y'])
            tag_size_px = tag_data['size'] * PIXELS_PER_METER * scale
            
            # Draw tag as a yellow square with black border
            tag_rect = pygame.Rect(
                tag_x - tag_size_px / 2,
                tag_y - tag_size_px / 2,
                tag_size_px,
                tag_size_px
            )
            pygame.draw.rect(self.screen, APRILTAG_COLOR, tag_rect)
            pygame.draw.rect(self.screen, APRILTAG_BORDER, tag_rect, max(1, int(3 * scale)))
            
            # Draw tag ID
            id_text = self.small_font.render(str(tag_data['id']), True, APRILTAG_BORDER)
            id_rect = id_text.get_rect(center=(tag_x, tag_y))
            self.screen.blit(id_text, id_rect)
    
    def check_apriltag_in_fov(self):
        """
        Check if any AprilTags are visible in the limelight's FOV.
        Updates vision table with target information if tag is visible.
        Publishes simulated vision data back to NetworkTables for robot code.
        """
        if not self.simulation_active:
            # Clear vision data when not active
            self.vision_table.putBoolean('HasTarget', False)
            return
        
        # Calculate turret's absolute angle in world frame (use raw angle for detection)
        turret_world_angle = self.smoothed_theta + math.radians(self.turret_angle)
        
        # Get robot position
        robot_x = self.smoothed_x
        robot_y = self.smoothed_y
        
        closest_tag = None
        closest_distance = float('inf')
        
        for tag_key, tag_data in self.apriltag_positions.items():
            tag_x = tag_data['x']
            tag_y = tag_data['y']
            
            # Calculate vector from robot to tag
            dx = tag_x - robot_x
            dy = tag_y - robot_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Check if tag is within max detection distance
            if distance > VISION_MAX_DISTANCE_M:
                continue
            
            # Calculate angle to tag from robot
            angle_to_tag = math.atan2(dy, dx)
            
            # Calculate relative angle to turret direction
            angle_diff = angle_to_tag - turret_world_angle
            
            # Normalize angle difference to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            # Convert to degrees
            angle_diff_deg = math.degrees(angle_diff)
            
            # Check if tag is within horizontal FOV
            if abs(angle_diff_deg) <= VISION_FOV_H / 2:
                # Tag is in FOV - check if it's the closest
                if distance < closest_distance:
                    closest_distance = distance
                    closest_tag = {
                        'id': tag_data['id'],
                        'yaw': angle_diff_deg,
                        'pitch': 0.0,  # Simplified - assume same height
                        'distance': distance,
                        'area': max(0.1, min(100.0, (tag_data['size'] / distance) * 100)),  # Simple area calc
                        'tag_x': tag_x,
                        'tag_y': tag_y
                    }
        
        # Update vision table with simulated PhotonVision/AprilTag data
        # Only publish if we detect a target - robot code will read this
        if closest_tag:
            # Store for drawing
            self.detected_target = closest_tag
            
            # Update internal state
            self.has_target = True
            self.target_yaw = closest_tag['yaw']
            self.target_pitch = closest_tag['pitch']
            self.target_area = closest_tag['area']
            self.target_id = closest_tag['id']
            
            # Publish target detection data to Vision table
            # Robot code expects this format from PhotonVision
            self.vision_table.putBoolean('HasTarget', True)
            self.vision_table.putNumber('Target_Yaw', closest_tag['yaw'])
            self.vision_table.putNumber('Target_Pitch', closest_tag['pitch'])
            self.vision_table.putNumber('Target_Area', closest_tag['area'])
            self.vision_table.putNumber('Target_ID', closest_tag['id'])
            self.vision_table.putNumber('Target_Distance', closest_tag['distance'])
        else:
            # No target in view - CLEAR the vision table so robot knows
            self.detected_target = None
            self.has_target = False
            self.target_id = -1
            
            # CRITICAL: Publish HasTarget = False so robot stops tracking
            self.vision_table.putBoolean('HasTarget', False)
            self.vision_table.putNumber('Target_Yaw', 0.0)
            self.vision_table.putNumber('Target_Pitch', 0.0)
            self.vision_table.putNumber('Target_Area', 0.0)
            self.vision_table.putNumber('Target_ID', -1)
            self.vision_table.putNumber('Target_Distance', 0.0)
    
    def draw_menu(self):
        """Draw the alliance selection menu."""
        # Semi-transparent overlay
        overlay = pygame.Surface((self.window_width, self.window_height))
        overlay.set_alpha(200)
        overlay.fill(MENU_BG)
        self.screen.blit(overlay, (0, 0))
        
        # Title
        title_font = pygame.font.Font(None, 72)
        title_text = title_font.render("IRISH Robot Localization", True, MENU_TEXT)
        title_rect = title_text.get_rect(center=(self.window_width // 2, self.window_height // 4))
        self.screen.blit(title_text, title_rect)
        
        # Subtitle
        subtitle_text = self.font.render("Select Your Alliance", True, MENU_TEXT)
        subtitle_rect = subtitle_text.get_rect(center=(self.window_width // 2, self.window_height // 4 + 80))
        self.screen.blit(subtitle_text, subtitle_rect)
        
        # Red Alliance Button
        button_width = 300
        button_height = 80
        red_button_rect = pygame.Rect(
            self.window_width // 2 - button_width - 20,
            self.window_height // 2 - button_height // 2,
            button_width,
            button_height
        )
        
        # Blue Alliance Button
        blue_button_rect = pygame.Rect(
            self.window_width // 2 + 20,
            self.window_height // 2 - button_height // 2,
            button_width,
            button_height
        )
        
        # Check mouse hover
        mouse_pos = pygame.mouse.get_pos()
        red_hover = red_button_rect.collidepoint(mouse_pos)
        blue_hover = blue_button_rect.collidepoint(mouse_pos)
        
        # Draw buttons
        red_color = BUTTON_HOVER if red_hover else BUTTON_BG
        blue_color = BUTTON_HOVER if blue_hover else BUTTON_BG
        
        pygame.draw.rect(self.screen, red_color, red_button_rect, border_radius=10)
        pygame.draw.rect(self.screen, (200, 0, 0), red_button_rect, 4, border_radius=10)
        
        pygame.draw.rect(self.screen, blue_color, blue_button_rect, border_radius=10)
        pygame.draw.rect(self.screen, (0, 0, 200), blue_button_rect, 4, border_radius=10)
        
        # Button text
        red_text = self.font.render("RED ALLIANCE", True, (255, 100, 100))
        red_text_rect = red_text.get_rect(center=red_button_rect.center)
        self.screen.blit(red_text, red_text_rect)
        
        blue_text = self.font.render("BLUE ALLIANCE", True, (100, 100, 255))
        blue_text_rect = blue_text.get_rect(center=blue_button_rect.center)
        self.screen.blit(blue_text, blue_text_rect)
        
        # Instructions
        instructions = [
            "After selecting alliance:",
            "- Simulation starts immediately",
            "- Robot starts at bottom-left corner (0.5, 0.5m)",
            "- Robot moves based on NetworkTables data",
            "- Press ESC to return to menu"
        ]
        
        y_offset = self.window_height // 2 + 100
        for instruction in instructions:
            inst_text = self.small_font.render(instruction, True, MENU_TEXT)
            inst_rect = inst_text.get_rect(center=(self.window_width // 2, y_offset))
            self.screen.blit(inst_text, inst_rect)
            y_offset += 30
        
        return red_button_rect, blue_button_rect
    

    
    def get_menu_buttons(self):
        """Get the menu button rectangles."""
        button_width = 300
        button_height = 80
        red_button_rect = pygame.Rect(
            self.window_width // 2 - button_width - 20,
            self.window_height // 2 - button_height // 2,
            button_width,
            button_height
        )
        blue_button_rect = pygame.Rect(
            self.window_width // 2 + 20,
            self.window_height // 2 - button_height // 2,
            button_width,
            button_height
        )
        return red_button_rect, blue_button_rect
    
    def setup_robot_for_alliance(self, alliance):
        """Set up robot initial position based on alliance."""
        # Initialize AprilTags based on alliance
        self.apriltag_positions = self.initialize_apriltags()
        
        # Both alliances start at bottom-left corner (matching robot code default)
        self.robot_x = 0.5
        self.robot_y = 0.5
        self.robot_theta = 0.0  # Facing right
        
        # Initialize smoothed values
        self.smoothed_x = self.robot_x
        self.smoothed_y = self.robot_y
        self.smoothed_theta = self.robot_theta
        self.prev_theta = self.robot_theta
        self.smoothed_turret_angle = 0.0
        
        # Start simulation immediately
        self.simulation_active = True
        self.menu_active = False
        
        theta_degrees = math.degrees(self.robot_theta)
        print(f"\nSimulation started for {alliance.upper()} alliance")
        print(f"Robot starting at bottom-left: X={self.robot_x:.3f}m, Y={self.robot_y:.3f}m, Theta={theta_degrees:.2f}°")
        print(f"Robot will move based on NetworkTables odometry data\n")
    
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
            f"Rotational Velocity: {math.degrees(self.robot_omega):.2f} °/s",
            "",  # Separator
            f"Turret Angle: {self.turret_angle:.2f}°",
            f"Has Target: {'Yes' if self.has_target else 'No'}"
        ]
        
        # Add vision target data if target is detected
        if self.has_target:
            telemetry_lines.extend([
                f"Target Yaw: {self.target_yaw:.2f}°",
                f"Target Pitch: {self.target_pitch:.2f}°",
                f"Target Area: {self.target_area:.2f}%",
                f"Target ID: {int(self.target_id)}"
            ])
        
        
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
                    if self.menu_active:
                        self.running = False
                    else:
                        # Return to menu
                        self.menu_active = True
                        self.odometry_active = False
                        self.alliance = None
                        print("Returned to menu")
                elif event.key == pygame.K_h:
                    # Toggle telemetry overlay
                    self.show_telemetry = not self.show_telemetry
                    status = "shown" if self.show_telemetry else "hidden"
                    print(f"Telemetry overlay {status}")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mouse_pos = pygame.mouse.get_pos()
                    
                    if self.menu_active and self.alliance is None:
                        # Check menu button clicks
                        red_button_rect, blue_button_rect = self.get_menu_buttons()
                        if red_button_rect.collidepoint(mouse_pos):
                            self.alliance = 'red'
                            self.setup_robot_for_alliance('red')
                        elif blue_button_rect.collidepoint(mouse_pos):
                            self.alliance = 'blue'
                            self.setup_robot_for_alliance('blue')
            elif event.type == pygame.VIDEORESIZE:
                # Force window to maintain aspect ratio
                new_width = event.w
                new_height = event.h
                
                # Calculate which dimension changed more
                width_ratio = new_width / self.window_width
                height_ratio = new_height / self.window_height
                
                # Use the dimension that changed more
                if abs(width_ratio - 1) > abs(height_ratio - 1):
                    # Width changed more - adjust height to match aspect ratio
                    self.window_width = new_width
                    self.window_height = int(new_width / self.aspect_ratio)
                else:
                    # Height changed more - adjust width to match aspect ratio
                    self.window_height = new_height
                    self.window_width = int(new_height * self.aspect_ratio)
                
                # Resize window to forced dimensions
                self.screen = pygame.display.set_mode((self.window_width, self.window_height), pygame.RESIZABLE)
                
                # No offset needed - image fills entire window
                self.field_offset_x = 0
                self.field_offset_y = 0
                self.field_draw_width = self.window_width
                self.field_draw_height = self.window_height
                
                # Scale both field image variants to new size (if they exist)
                if hasattr(self, 'field_image_red') and self.field_image_red is not None:
                    self.scaled_field_image_red = pygame.transform.scale(self.field_image_red, 
                                                                      (self.field_draw_width, self.field_draw_height))
                if hasattr(self, 'field_image_blue') and self.field_image_blue is not None:
                    self.scaled_field_image_blue = pygame.transform.scale(self.field_image_blue, 
                                                                      (self.field_draw_width, self.field_draw_height))
    
    def run(self):
        """Main application loop."""
        print("\n" + "=" * 60)
        print("Robot Localization Simulation - IRISH Field")
        print("=" * 60)
        print(f"NetworkTables: 127.0.0.1 → Table 'Pose' (X, Y, Theta)")
        print(f"Field: {FIELD_WIDTH_M}m → {SCREEN_WIDTH}px ({PIXELS_PER_METER:.2f} px/m)")
        print(f"Origin: (0,0) meters → bottom-left of image")
        print(f"Controls:")
        print(f"  - Select alliance (Red/Blue)")
        print(f"  - Robot starts at (0.5, 0.5m)")
        print(f"  - H to toggle telemetry")
        print(f"  - ESC to return to menu / quit")
        print("=" * 60 + "\n")
        
        while self.running:
            # Handle events
            self.handle_events()
            
            # Update robot pose from NetworkTables (only when simulation active)
            self.update_pose_data()
            
            # Check AprilTag visibility in FOV
            self.check_apriltag_in_fov()
            
            # Draw everything
            self.draw_field()
            
            if self.menu_active:
                # Draw menu overlay
                self.draw_menu()
            else:
                # Draw robot and telemetry
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
