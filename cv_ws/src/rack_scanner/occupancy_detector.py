import cv2
import numpy as np

class OccupancyDetector:
    """
    Detects shelf occupancy by finding the top horizontal line where objects exist.
    A row is considered occupied if at least a threshold percentage has objects.
    """
    
    def __init__(self, occupancy_threshold=0.10, edge_margin=10):
        """
        Args:
            occupancy_threshold: Fraction of row width that must have objects (default 10%)
            edge_margin: Pixels to ignore at left/right edges
        """
        self.occupancy_threshold = occupancy_threshold
        self.edge_margin = edge_margin
    
    def detect_objects_mask(self, image, shelf_top, shelf_bottom, rx, rw):
        """
        Create a mask of objects within the shelf region.
        Uses edge detection and background subtraction approach.
        """
        height, width = image.shape[:2]
        
        # Extract shelf region
        y_start = max(0, shelf_top)
        y_end = min(height, shelf_bottom)
        x_start = max(0, rx + self.edge_margin)
        x_end = min(width, rx + rw - self.edge_margin)
        
        if y_end <= y_start or x_end <= x_start:
            return None, None
        
        shelf_roi = image[y_start:y_end, x_start:x_end]
        
        # Convert to grayscale
        gray = cv2.cvtColor(shelf_roi, cv2.COLOR_BGR2GRAY)
        
        # Apply CLAHE for better contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Edge detection to find object boundaries
        edges = cv2.Canny(enhanced, 30, 100)
        
        # Dilate edges to connect nearby edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        dilated = cv2.dilate(edges, kernel, iterations=2)
        
        # Also detect non-background regions using color variance
        hsv = cv2.cvtColor(shelf_roi, cv2.COLOR_BGR2HSV)
        
        # Background is usually uniform (low saturation, consistent value)
        # Objects have more color/texture variation
        sat_channel = hsv[:, :, 1]
        
        # Threshold for objects: either high saturation OR significant edges
        sat_mask = sat_channel > 25
        
        # Combine edge-based and saturation-based detection
        object_mask = np.logical_or(dilated > 0, sat_mask).astype(np.uint8) * 255
        
        # Clean up with morphological operations
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        object_mask = cv2.morphologyEx(object_mask, cv2.MORPH_CLOSE, kernel_close)
        
        return object_mask, (y_start, y_end, x_start, x_end)
    
    def find_occupancy_line(self, image, shelf_top, shelf_bottom, rx, rw):
        """
        Find the top-most horizontal line where objects are detected.
        
        Args:
            image: BGR image
            shelf_top: Y coordinate of shelf top beam
            shelf_bottom: Y coordinate of shelf bottom beam
            rx: X start of rack region
            rw: Width of rack region
        
        Returns:
            occupancy_y: Y coordinate of top object line (None if empty)
            occupancy_percent: Percentage of shelf height occupied
        """
        object_mask, roi_coords = self.detect_objects_mask(
            image, shelf_top, shelf_bottom, rx, rw
        )
        
        if object_mask is None:
            return None, 0.0
        
        y_start, y_end, x_start, x_end = roi_coords
        roi_width = x_end - x_start
        shelf_height = y_end - y_start
        
        # Ignore top margin to avoid detecting the rack beam as object
        beam_margin = 15  # pixels to skip from top/bottom to avoid beam detection
        
        # Scan row occupancy
        row_occupancy = np.sum(object_mask > 0, axis=1) / roi_width
        
        # Scan from BOTTOM to TOP to find the top of actual objects
        # This avoids catching the top rack beam as an object
        top_occupied_row = None
        
        for row_idx in range(len(row_occupancy) - beam_margin - 1, beam_margin, -1):
            if row_occupancy[row_idx] >= self.occupancy_threshold:
                # Found an occupied row, now scan upward to find the top of this object
                top_occupied_row = row_idx
                # Continue scanning up to find the actual top
                for upper_row in range(row_idx - 1, beam_margin, -1):
                    if row_occupancy[upper_row] >= self.occupancy_threshold:
                        top_occupied_row = upper_row
                    else:
                        # Found a gap - this is the top of the object
                        break
                break
        
        if top_occupied_row is None:
            return None, 0.0
        
        # Convert back to image coordinates
        occupancy_y = y_start + top_occupied_row
        
        # Calculate occupancy percentage (how much of shelf height is filled)
        occupied_height = shelf_height - top_occupied_row
        occupancy_percent = (occupied_height / shelf_height) * 100 if shelf_height > 0 else 0
        
        return occupancy_y, occupancy_percent
    
    def analyze_shelf(self, image, shelf_top, shelf_bottom, rx, rw):
        """
        Complete shelf analysis returning occupancy info.
        
        Returns:
            dict with:
                - occupancy_line_y: Y coordinate of top object line
                - occupancy_percent: Percentage of shelf filled
                - is_occupied: Boolean if shelf has any objects
        """
        occupancy_y, occupancy_percent = self.find_occupancy_line(
            image, shelf_top, shelf_bottom, rx, rw
        )
        
        return {
            'occupancy_line_y': occupancy_y,
            'occupancy_percent': round(occupancy_percent, 1),
            'is_occupied': occupancy_y is not None,
            'shelf_top': shelf_top,
            'shelf_bottom': shelf_bottom
        }
    
    def draw_occupancy(self, image, shelf_info, rx, rw, color=(0, 165, 255)):
        """
        Draw occupancy line on image.
        
        Args:
            image: Image to draw on (modified in place)
            shelf_info: Dict from analyze_shelf()
            rx: X start of rack
            rw: Width of rack
            color: BGR color for occupancy line (default orange)
        """
        if shelf_info['is_occupied']:
            y = shelf_info['occupancy_line_y']
            # Draw occupancy line
            cv2.line(image, (rx, y), (rx + rw, y), color, 2)
            
            # Draw fill indicator
            cv2.putText(
                image, 
                f"{shelf_info['occupancy_percent']:.0f}%",
                (rx + rw + 5, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )
        
        return image