import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import os
from datetime import datetime
from rack_scanner.occupancy_detector import OccupancyDetector

class RackImageProcessor(Node):
    def __init__(self):
        super().__init__('rack_image_processor')
        
        self.subscription = self.create_subscription(
            Image,
            '/raw_image',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        
        # --- TUNING PARAMETERS ---
        self.vertical_ignore_percent = 5
        self.min_line_width_ratio = 0.50  # Minimum line width as ratio of image width
        self.beam_merge_distance = 35     # Pixels to merge nearby beams
        self.min_edge_strength = 25       # For edge detection
        self.min_beam_spacing_ratio = 0.48  # Minimum spacing between beams as ratio of image height
        self.min_beam_thickness = 8       # Minimum beam thickness in pixels
        self.min_votes_required = 2       # Minimum detection votes to accept a beam
        
        # Initialize occupancy detector
        self.occupancy_detector = OccupancyDetector(occupancy_threshold=0.10)
        
        # Setup output directories
        self.output_dir = os.path.expanduser('~/rack_scanner_output')
        self.images_dir = os.path.join(self.output_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=True)
        
        # CSV file setup
        self.csv_file = os.path.join(self.output_dir, 'rack_scan_data.csv')
        self.init_csv()
        
        # Frame counter for saving
        self.frame_count = 0
        self.save_interval = 30  # Save every 30 frames (~1 sec at 30fps)
        
        self.get_logger().info('Rack Scanner V8: Robust beam filtering with occupancy')
        self.get_logger().info(f'Output directory: {self.output_dir}')

    def init_csv(self):
        """Initialize CSV file with headers"""
        file_exists = os.path.isfile(self.csv_file)
        if not file_exists:
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'frame_number', 'image_filename', 
                               'num_beams', 'occupancy_percent', 'shelf_status'])

    def detect_edges_horizontal(self, image):
        """Detect horizontal edges using Sobel operator"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE for better contrast in varying lighting
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        
        # Sobel for horizontal edges (detecting vertical changes)
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        abs_sobel = np.absolute(sobel_y)
        sobel_8u = np.uint8(255 * abs_sobel / np.max(abs_sobel)) if np.max(abs_sobel) > 0 else np.zeros_like(gray)
        
        return sobel_8u

    def detect_lines_hough(self, edge_image, width):
        """Use Hough Line Transform to detect horizontal lines"""
        lines = cv2.HoughLinesP(
            edge_image,
            rho=1,
            theta=np.pi/180,
            threshold=50,
            minLineLength=int(width * self.min_line_width_ratio),
            maxLineGap=30
        )
        
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Check if line is roughly horizontal (within 10 degrees)
                angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                if angle < 10 or angle > 170:
                    y_avg = (y1 + y2) // 2
                    line_length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    horizontal_lines.append((y_avg, line_length, x1, x2))
        
        return horizontal_lines

    def detect_dark_horizontal_bands(self, image):
        """Detect dark horizontal structures (for black/dark racks)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Multiple thresholds for different darkness levels
        masks = []
        for v_max in [80, 120, 160]:
            mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, v_max]))
            masks.append(mask)
        
        return masks

    def detect_metallic_horizontal_bands(self, image):
        """Detect metallic/chrome shelves (silver, grey)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Low saturation, medium-high value = metallic/grey
        metallic_mask = cv2.inRange(hsv, np.array([0, 0, 100]), np.array([180, 50, 220]))
        
        return metallic_mask

    def find_horizontal_structures(self, mask, width, min_width_ratio=0.3):
        """Find horizontal structures in a binary mask"""
        kernel_width = max(int(width * min_width_ratio), 50)
        horizontal_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, 3))
        
        # Morphological opening to keep only horizontal structures
        horizontal = cv2.morphologyEx(mask, cv2.MORPH_OPEN, horizontal_kernel)
        
        # Get row-wise intensity
        row_sums = np.sum(horizontal, axis=1)
        threshold = width * 255 * 0.2  # At least 20% of width should be detected
        
        beam_rows = np.where(row_sums > threshold)[0]
        return beam_rows

    def cluster_beam_rows(self, beam_rows, merge_distance=20):
        """Cluster nearby row indices into beam centers"""
        if len(beam_rows) == 0:
            return []
        
        beams = []
        current_group = [beam_rows[0]]
        
        for i in range(1, len(beam_rows)):
            if beam_rows[i] - beam_rows[i-1] < merge_distance:
                current_group.append(beam_rows[i])
            else:
                beams.append(int(np.mean(current_group)))
                current_group = [beam_rows[i]]
        
        beams.append(int(np.mean(current_group)))
        return beams

    def validate_beam_consistency(self, beams, image, rx, rw):
        """Validate beams by checking for consistent horizontal structure"""
        validated = []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        for beam_y in beams:
            # Check a strip around the beam
            y_start = max(0, beam_y - 5)
            y_end = min(image.shape[0], beam_y + 5)
            strip = gray[y_start:y_end, rx:rx+rw]
            
            if strip.size == 0:
                continue
            
            # Calculate horizontal variance (low variance = consistent horizontal line)
            row_means = np.mean(strip, axis=1)
            col_std = np.std(strip, axis=0)
            
            # A real shelf beam should have relatively consistent intensity across width
            avg_col_std = np.mean(col_std)
            
            # Also check using gradient - real beams have strong horizontal edges
            sobel_y = cv2.Sobel(strip, cv2.CV_64F, 0, 1, ksize=3)
            edge_strength = np.mean(np.abs(sobel_y))
            
            # Accept if either low variance OR strong edge
            if avg_col_std < 80 or edge_strength > 20:
                validated.append(beam_y)
        
        return validated

    def is_thick_beam(self, beam_y, image, width):
        """
        Check if the detected line is a thick structural beam (not thin wire mesh).
        Real shelf beams are typically 5-20 pixels thick and darker than surroundings.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        height = gray.shape[0]
        
        # Check region around beam
        check_radius = 15
        y_start = max(0, beam_y - check_radius)
        y_end = min(height, beam_y + check_radius)
        
        strip = gray[y_start:y_end, :]
        if strip.size == 0:
            return False
        
        # Get vertical profile (average intensity at each row)
        vertical_profile = np.mean(strip, axis=1)
        
        # Find the darkest region (beam center)
        min_idx = np.argmin(vertical_profile)
        min_val = vertical_profile[min_idx]
        
        # Check if there's a significant dip (beam is darker than surroundings)
        edge_avg = (np.mean(vertical_profile[:3]) + np.mean(vertical_profile[-3:])) / 2
        darkness_diff = edge_avg - min_val
        
        # Count consecutive dark pixels (beam thickness)
        threshold = min_val + (darkness_diff * 0.5)
        dark_pixels = vertical_profile < threshold
        
        # Find the longest consecutive run of dark pixels
        max_run = 0
        current_run = 0
        for is_dark in dark_pixels:
            if is_dark:
                current_run += 1
                max_run = max(max_run, current_run)
            else:
                current_run = 0
        
        # Beam must be thick enough and have significant darkness difference
        is_thick = max_run >= self.min_beam_thickness
        is_dark_enough = darkness_diff > 8  # Relaxed from 15
        
        return is_thick or is_dark_enough  # Accept if EITHER condition met

    def detect_vertical_posts(self, image):
        """Detect vertical rack posts to establish rack boundaries"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Detect vertical lines
        height, width = gray.shape
        vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, int(height * 0.3)))
        vertical_lines = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, vertical_kernel)
        
        # Find columns with strong vertical presence
        col_sums = np.sum(vertical_lines, axis=0)
        threshold = height * 255 * 0.2
        
        strong_cols = np.where(col_sums > threshold)[0]
        
        if len(strong_cols) > 0:
            left_post = strong_cols[0]
            right_post = strong_cols[-1]
            return left_post, right_post
        
        return 0, width

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            display_img = cv_image.copy()

            # --- 1. CALCULATE IGNORE ZONES ---
            margin_px = int(height * (self.vertical_ignore_percent / 100.0))
            valid_y_min = margin_px
            valid_y_max = height - margin_px

            # --- 2. MULTI-METHOD BEAM DETECTION ---
            all_beam_candidates = []

            # Method 1: Edge-based detection (works for any color rack)
            edge_image = self.detect_edges_horizontal(cv_image)
            _, edge_binary = cv2.threshold(edge_image, self.min_edge_strength, 255, cv2.THRESH_BINARY)
            
            hough_lines = self.detect_lines_hough(edge_binary, width)
            for y_avg, length, x1, x2 in hough_lines:
                if valid_y_min < y_avg < valid_y_max and length > width * 0.3:
                    all_beam_candidates.append(('hough', y_avg, length))

            # Method 2: Dark band detection (black racks)
            dark_masks = self.detect_dark_horizontal_bands(cv_image)
            for i, mask in enumerate(dark_masks):
                beam_rows = self.find_horizontal_structures(mask, width)
                beams = self.cluster_beam_rows(beam_rows, self.beam_merge_distance)
                for b in beams:
                    if valid_y_min < b < valid_y_max:
                        all_beam_candidates.append(('dark', b, width))

            # Method 3: Metallic detection (chrome/wire racks)
            metallic_mask = self.detect_metallic_horizontal_bands(cv_image)
            beam_rows = self.find_horizontal_structures(metallic_mask, width, min_width_ratio=0.2)
            beams = self.cluster_beam_rows(beam_rows, self.beam_merge_distance)
            for b in beams:
                if valid_y_min < b < valid_y_max:
                    all_beam_candidates.append(('metallic', b, width))

            # Method 4: Gradient-based row analysis
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            row_gradients = np.abs(np.diff(gray.astype(float), axis=0))
            row_gradient_sums = np.sum(row_gradients, axis=1)
            
            # Find peaks in gradient (edges of shelves)
            gradient_threshold = np.mean(row_gradient_sums) + np.std(row_gradient_sums)
            gradient_peaks = np.where(row_gradient_sums > gradient_threshold)[0]
            gradient_beams = self.cluster_beam_rows(gradient_peaks, self.beam_merge_distance)
            for b in gradient_beams:
                if valid_y_min < b < valid_y_max:
                    all_beam_candidates.append(('gradient', b, width))

            # --- 3. MERGE AND VOTE ON CANDIDATES ---
            # Group candidates that are close together
            if all_beam_candidates:
                candidate_ys = [c[1] for c in all_beam_candidates]
                candidate_ys.sort()
                
                # Merge candidates within merge_distance
                merged_beams = []
                current_group = [candidate_ys[0]]
                
                for i in range(1, len(candidate_ys)):
                    if candidate_ys[i] - candidate_ys[i-1] < self.beam_merge_distance:
                        current_group.append(candidate_ys[i])
                    else:
                        # Weight by number of methods that detected it
                        merged_beams.append((int(np.mean(current_group)), len(current_group)))
                        current_group = [candidate_ys[i]]
                
                merged_beams.append((int(np.mean(current_group)), len(current_group)))
                
                # Filter: keep beams with enough votes
                voted_beams = []
                for beam_y, vote_count in merged_beams:
                    if vote_count >= self.min_votes_required:
                        voted_beams.append((beam_y, vote_count))
                
                # Sort by vote count (strongest first)
                voted_beams.sort(key=lambda x: -x[1])
                
                # --- 4. ENFORCE MINIMUM SPACING ---
                # Real shelf beams are spaced apart, not close together
                min_spacing = int(height * self.min_beam_spacing_ratio)
                final_beams = []
                
                for beam_y, vote_count in voted_beams:
                    # Check if this beam is far enough from already accepted beams
                    is_valid = True
                    for accepted_y in final_beams:
                        if abs(beam_y - accepted_y) < min_spacing:
                            is_valid = False
                            break
                    
                    if is_valid:
                        # Additional validation: check beam thickness/darkness
                        if self.is_thick_beam(beam_y, cv_image, width):
                            final_beams.append(beam_y)
                
                # Sort final beams by Y position
                final_beams.sort()
            else:
                final_beams = []

            # --- 5. FIND RACK BOUNDARIES ---
            left_post, right_post = self.detect_vertical_posts(cv_image)
            rx = max(0, left_post - 10)
            rw = min(width, right_post + 10) - rx

            # --- 5. VISUALIZATION ---
            center_y = height // 2
            line_above = None
            line_below = None
            
            for b in final_beams:
                cv2.line(display_img, (rx, b), (rx+rw, b), (255, 0, 0), 2)

            above_candidates = [b for b in final_beams if b < center_y]
            if above_candidates: line_above = max(above_candidates)

            below_candidates = [b for b in final_beams if b > center_y]
            if below_candidates: line_below = min(below_candidates)

            overlay = display_img.copy()
            
            # DRAW THE IGNORE ZONES (Grey overlay)
            cv2.rectangle(overlay, (0, 0), (width, margin_px), (100, 100, 100), -1)
            cv2.rectangle(overlay, (0, valid_y_max), (width, height), (100, 100, 100), -1)
            
            # Shelf Highlight and Occupancy Detection
            if line_above and line_below:
                cv2.rectangle(overlay, (rx, line_above), (rx+rw, line_below), (0, 255, 0), -1)
                
                # Detect occupancy for this shelf
                shelf_info = self.occupancy_detector.analyze_shelf(
                    cv_image, line_above, line_below, rx, rw
                )
                self.occupancy_detector.draw_occupancy(display_img, shelf_info, rx, rw)
                
                status_text = f"SHELF: {shelf_info['occupancy_percent']:.0f}% filled"
                cv2.putText(display_img, status_text, (rx+10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            elif line_above:
                cv2.rectangle(overlay, (rx, line_above), (rx+rw, height-margin_px), (0, 255, 0), -1)
            elif line_below:
                cv2.rectangle(overlay, (rx, margin_px), (rx+rw, line_below), (0, 255, 0), -1)
            
            cv2.addWeighted(overlay, 0.3, display_img, 0.7, 0, display_img)
            cv2.line(display_img, (0, center_y), (width, center_y), (0, 255, 255), 1)

            # Debug info
            cv2.putText(display_img, f"Beams: {len(final_beams)}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Save image and data periodically
            self.frame_count += 1
            if self.frame_count % self.save_interval == 0:
                self.save_frame_data(display_img, final_beams, shelf_info if (line_above and line_below) else None)

            # Comment out display to avoid Qt/Wayland issues
            # cv2.imshow("Rack Scanner V7 (Robust)", display_img)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'CV Error: {e}')
    
    def save_frame_data(self, display_img, beams, shelf_info):
        """Save annotated image and log data to CSV"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            image_filename = f'rack_scan_{timestamp}_frame{self.frame_count}.jpg'
            image_path = os.path.join(self.images_dir, image_filename)
            
            # Save image
            cv2.imwrite(image_path, display_img)
            
            # Prepare CSV data
            occupancy = shelf_info['occupancy_percent'] if shelf_info else 0.0
            status = 'DETECTED' if shelf_info else 'NO_SHELF'
            
            # Write to CSV
            with open(self.csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    self.frame_count,
                    image_filename,
                    len(beams),
                    f'{occupancy:.1f}',
                    status
                ])
            
            self.get_logger().info(f'Saved: {image_filename}, Beams: {len(beams)}, Occupancy: {occupancy:.1f}%')
        
        except Exception as e:
            self.get_logger().error(f'Save error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RackImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()