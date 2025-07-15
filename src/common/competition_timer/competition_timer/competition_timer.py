import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32, Int32
from race_msgs.msg import VehicleFlag
import yaml
import os
from datetime import datetime
import numpy as np
import csv
from ament_index_python.packages import get_package_share_directory
import pymap3d
class CompetitionTimerNode(Node):
    def __init__(self):
        super().__init__('competition_timer')
        ################## parameters initialization ##################
        # Declare parameters
        self.declare_parameter('results_dir', os.path.join(os.getcwd(), 'competition_results'))
        self.results_dir = self.get_parameter('results_dir').value

        # Declare config file path parameter
        default_config_path = os.path.join(
            get_package_share_directory('competition_timer'),
            'config',
            'LVMS.yaml'
        )
        self.declare_parameter('config_file_path', default_config_path)
        self.config_file_path = self.get_parameter('config_file_path').value

        # Declare target laps parameter
        self.declare_parameter('target_laps', 10)  # Default to 10 laps
        self.target_laps = self.get_parameter('target_laps').value
        self.get_logger().info(f'Target laps set to: {self.target_laps}')

        # Declare vehicle flag parameter (now accepts string)
        self.declare_parameter('vehicle_flag', 'red')
        self._flag_map = {
            'green': VehicleFlag.GREEN,
            'red': VehicleFlag.RED,
            'black': VehicleFlag.BLACK,
            'g10': VehicleFlag.G10,
            'g20': VehicleFlag.G20,
            'g40': VehicleFlag.G40,
            'g60': VehicleFlag.G60,
            'g80': VehicleFlag.G80,
            'g100': VehicleFlag.G100
        }
        # Reverse mapping for logging
        self._flag_to_string = {
            VehicleFlag.GREEN: 'GREEN',
            VehicleFlag.RED: 'RED',
            VehicleFlag.BLACK: 'BLACK',
            VehicleFlag.G10: 'G10',
            VehicleFlag.G20: 'G20',
            VehicleFlag.G40: 'G40',
            VehicleFlag.G60: 'G60',
            VehicleFlag.G80: 'G80',
            VehicleFlag.G100: 'G100'
        }
        self.vehicle_flag = self._flag_map.get(self.get_parameter('vehicle_flag').value.lower(), VehicleFlag.RED)
        self.get_logger().info(f'Vehicle flag set to: {self._flag_to_string[self.vehicle_flag]}')

        
        self.get_logger().info(f'Results will be saved to: {self.results_dir}')
        # Load finish line configuration
        self.load_config()
        # self.get_logger().info(f'Finish line configuration loaded')
        
        ################## subscriptions ##################
        # Subscribe to required topics
        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/gps_top/fix',
            self.gps_callback,
            10
        )
        # self.get_logger().info('GPS subscription created')
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        # self.get_logger().info('Clock subscription created')
        
        ################## publishers ##################
        # Publisher for vehicle flag
        self.vehicle_flag_publisher = self.create_publisher(
            VehicleFlag,
            '/vehicle_flag',
            10
        )
        # self.get_logger().info('Vehicle flag publisher created')
        
        # Create timer for publishing vehicle flag at 50Hz
        self.flag_timer = self.create_timer(0.02, self.publish_vehicle_flag)
        
        # Publisher for lap times
        self.lap_time_publisher = self.create_publisher(
            Float32,
            '/lap_time',
            10
        )
        # self.get_logger().info('Lap time publisher created')
        
        ################## state variables ##################
        # Initialize state variables
        self.competition_active = False
        self.start_time = None
        self.end_time = None
        self.current_time = None
        self.last_crossing_time = None
        self.lap_times = []
        
        # Ensure results directory exists
        os.makedirs(self.results_dir, exist_ok=True)
        
        self.get_logger().info('Competition Timer Node initialized')
        # self.get_logger().info(f'Results will be saved to: {self.results_dir}')

    def load_config(self):
        """Load finish line configuration from YAML file"""
        try:
            with open(self.config_file_path, 'r') as f:
                config = yaml.safe_load(f)
                
            self.origin = (config['origin']['X'], 
                          config['origin']['Y'], 
                          config['origin']['Z'])
            finish_line = config['finish_line']
            self.point1 = np.array((finish_line['point1']['X'], 
                          finish_line['point1']['Y']))
            self.point2 = np.array((finish_line['point2']['X'], 
                          finish_line['point2']['Y']))
            self.crossing_tolerance = finish_line['crossing_tolerance']
            self.min_crossing_interval = finish_line['min_crossing_interval']
            
            self.get_logger().info(f'Loaded config file: {self.config_file_path}')
            self.get_logger().info(f'Origin: {self.origin}')
            self.get_logger().info(f'Point1: {self.point1}')
            self.get_logger().info(f'Point2: {self.point2}')
            # self.get_logger().info(f'Crossing tolerance: {self.crossing_tolerance}')
            # self.get_logger().info(f'Min crossing interval: {self.min_crossing_interval}')
            # self.get_logger().info('Loaded finish line configuration')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            raise

    def publish_vehicle_flag(self):
        """Publish the current vehicle flag state"""

        # self.get_logger().info(f'Publishing vehicle flag: {self._flag_to_string[self.vehicle_flag]}')
        msg = VehicleFlag()
        param_value = self.get_parameter('vehicle_flag').value.lower()
        self.vehicle_flag = self._flag_map.get(param_value, VehicleFlag.RED)
        msg.flag = self.vehicle_flag
        self.vehicle_flag_publisher.publish(msg)
        
        # Handle competition state changes based on vehicle flag
        is_competition_active = (self.vehicle_flag == VehicleFlag.GREEN or 
                               self.vehicle_flag == VehicleFlag.G10 or
                               self.vehicle_flag == VehicleFlag.G20 or
                               self.vehicle_flag == VehicleFlag.G40 or
                               self.vehicle_flag == VehicleFlag.G60 or
                               self.vehicle_flag == VehicleFlag.G80 or
                               self.vehicle_flag == VehicleFlag.G100)

        if is_competition_active and not self.competition_active:
            # Competition just started
            self.competition_active = True
            self.start_time = self.current_time
            self.last_crossing_time = None
            self.lap_times = []
            self.current_lap = 0
            self.get_logger().info(f'Competition started at {self.start_time}')
        elif not is_competition_active and self.competition_active:
            self.get_logger().info(f'Flag set to {self._flag_to_string[self.vehicle_flag]}')
            # Competition just ended
            if self.vehicle_flag == VehicleFlag.BLACK:
                self.competition_active = False
                self.get_logger().info(f'Competition ended! ')
                if self.current_lap <= self.target_laps:
                    self.get_logger().info(f'Competition ended at lap {self.current_lap}. competition not completed')
                else:
                    self.get_logger().info(f"{self.target_laps} completed laps. Competition completed. Saving results")
                    self.save_results()
            else:
                self.get_logger().info(f'Competition paused with {self._flag_to_string[self.vehicle_flag]} flag')

    def clock_callback(self, msg):
        """Update current time from ROS clock"""
        self.current_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def gps_callback(self, msg):
        """Process GPS updates and detect finish line crossings"""
        if not self.competition_active or self.current_time is None:
            return
            
        self.latest_gps = msg
        # Convert GPS to local coordinates
        local_x, local_y, _ = pymap3d.geodetic2enu(
            msg.latitude, 
            msg.longitude, 
            msg.altitude if msg.altitude else 0.0,
            self.origin[0],
            self.origin[1],
            self.origin[2]
        )
        
        # Check if we've crossed the finish line
        if self.check_finish_line_crossing(local_x, local_y):
            crossing_time = self.current_time
            
            if self.current_lap > 0:
                self.get_logger().info(f'Finish line crossed at {crossing_time}')
            
            # Calculate and publish lap time
            if self.last_crossing_time is not None:
                if self.current_lap == 1:
                    lap_time = crossing_time - self.start_time
                else:
                    lap_time = crossing_time - self.last_crossing_time
                
                # Publish lap time
                time_msg = Float32()
                time_msg.data = lap_time
                self.lap_time_publisher.publish(time_msg)
                
                # Record lap time
                self.lap_times.append(lap_time)
                self.get_logger().info(f'Lap {self.current_lap} completed in {lap_time:.3f} seconds')
                
                # Check if target laps reached
                if self.current_lap >= self.target_laps:
                    self.end_time = crossing_time
                    self.get_logger().info(f'Target laps ({self.target_laps}) reached! Ending race. at {self.end_time}');
                    # Set vehicle flag to black to end the race
                    self.set_parameters([rclpy.parameter.Parameter('vehicle_flag', rclpy.Parameter.Type.STRING, 'black')])
                    self.vehicle_flag = VehicleFlag.BLACK
                    self.get_logger().info(f'Vehicle flag set to: {self._flag_to_string[self.vehicle_flag]}')
            
            self.current_lap += 1
            self.last_crossing_time = crossing_time

    def check_finish_line_crossing(self, x, y):
        """
        Check if the given local position crosses the finish line
        Uses a simple point-to-line distance calculation
        """
        if self.last_crossing_time is not None:
            # Check minimum interval between crossings
            if self.current_time - self.last_crossing_time < self.min_crossing_interval:
                return False
        
        # Current position as numpy array
        p = np.array([x, y])
        
        # Calculate distance to line segment
        # Vector from point1 to point2
        ab = self.point2 - self.point1
        # Vector from point1 to current position
        ap = p - self.point1
        
        # Project ap onto ab
        projection = np.dot(ap, ab) / np.dot(ab, ab)
        
        # Check if projection lies outside line segment
        if projection < 0:
            distance = np.linalg.norm(ap)
        elif projection > 1:
            distance = np.linalg.norm(p - self.point2)
        else:
            # Calculate perpendicular distance
            distance = np.linalg.norm(ap - projection * ab)
        
        return distance < self.crossing_tolerance

    def save_results(self):
        """Save lap times and competition results to CSV"""

        if not self.start_time:
            self.get_logger().warn('Competition not started')
            return
        else:
            self.get_logger().info(f'Competition started at {self.start_time}')
            self.get_logger().info(f'Competition ended at {self.end_time}')
            self.get_logger().info(f'Total time: {self.end_time - self.start_time:.3f} seconds')

        if not self.lap_times:
            self.get_logger().warn('No lap times to save')
            return
            
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.results_dir, f'lap_times_{timestamp}.csv')
        
        try:
            with open(filepath, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                # Write header
                writer.writerow(['Lap', 'Time (seconds)'])
                # Write lap times
                for lap, time in enumerate(self.lap_times, 1):
                    writer.writerow([lap, f'{time:.3f}'])
                # Write summary
                writer.writerow([])
                writer.writerow(['Total Laps', len(self.lap_times)])
                writer.writerow(['Total Time', f'{sum(self.lap_times):.3f}'])
                writer.writerow(['Average Lap Time', f'{sum(self.lap_times)/len(self.lap_times):.3f}'])
                writer.writerow(['Best Lap Time', f'{min(self.lap_times):.3f}'])
                
            self.get_logger().info(f'Results saved to: {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CompetitionTimerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.competition_active:
            node.save_results()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 