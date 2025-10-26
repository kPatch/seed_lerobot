#!/usr/bin/env python3
"""
LeRobot Policy Controller Node

This ROS2 node manages the execution of LeRobot policy recording commands.
It provides services to start, stop, and query the status of the policy process.

Services:
    /ledog/start_policy (std_srvs/srv/Trigger):
        Start the LeRobot policy recording process
        
    /ledog/stop_policy (std_srvs/srv/Trigger):
        Stop the currently running policy process
        
    /ledog/get_policy_status (std_srvs/srv/Trigger):
        Get the current status of the policy process

Author: kpatch
License: MIT
"""

import subprocess
import threading
import time
import os
import signal
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class PolicyControllerNode(Node):
    """
    ROS2 node for managing LeRobot policy execution.
    
    This node handles the lifecycle of the lerobot-record CLI command,
    which does not exit automatically. It runs the command in a separate
    thread and provides services to start, stop, and monitor the process.
    """
    
    def __init__(self):
        super().__init__('policy_controller_node')
        
        # Declare parameters
        self.declare_parameter('conda_env_name', 'lerobot')
        self.declare_parameter('conda_base_path', '~/miniconda3')
        self.declare_parameter('shutdown_timeout', 5.0)
        self.declare_parameter('robot_id', 'follower_1')
        self.declare_parameter('robot_port', '/dev/ttyACM0')
        self.declare_parameter('robot_type', 'so100_follower')
        
        # Get parameters
        self.conda_env = self.get_parameter('conda_env_name').value
        self.conda_base = os.path.expanduser(
            self.get_parameter('conda_base_path').value
        )
        self.shutdown_timeout = self.get_parameter('shutdown_timeout').value
        
        # Process management
        self.process: Optional[subprocess.Popen] = None
        self.process_thread: Optional[threading.Thread] = None
        self.process_lock = threading.Lock()
        
        # Create services
        self.start_service = self.create_service(
            Trigger,
            '/ledog/start_policy',
            self.start_policy_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            '/ledog/stop_policy',
            self.stop_policy_callback
        )
        
        self.status_service = self.create_service(
            Trigger,
            '/ledog/get_policy_status',
            self.get_policy_status_callback
        )
        
        self.get_logger().info('LeRobot Policy Controller Node initialized')
        self.get_logger().info(f'Services available:')
        self.get_logger().info('  - /ledog/start_policy')
        self.get_logger().info('  - /ledog/stop_policy')
        self.get_logger().info('  - /ledog/get_policy_status')
    
    def start_policy_callback(self, request, response):
        """
        Service callback to start the LeRobot policy process.
        
        Args:
            request: Empty Trigger request
            response: Trigger response with success status and message
            
        Returns:
            Trigger.Response with success=True if started, False if already running
        """
        with self.process_lock:
            # Check if process is already running
            if self.process is not None and self.process.poll() is None:
                response.success = False
                response.message = f'Policy already running (PID: {self.process.pid})'
                self.get_logger().warn(response.message)
                return response
            
            # Start the policy in a separate thread
            try:
                self.process_thread = threading.Thread(
                    target=self._run_policy_thread,
                    daemon=True
                )
                self.process_thread.start()
                
                # Give the process a moment to start
                time.sleep(0.2)
                
                # Verify it started
                if self.process is not None and self.process.poll() is None:
                    response.success = True
                    response.message = f'LeRobot policy started successfully (PID: {self.process.pid})'
                    self.get_logger().info(response.message)
                else:
                    response.success = False
                    response.message = 'Policy process started but exited immediately (check logs and conda environment)'
                    self.get_logger().error(response.message)
                    
            except Exception as e:
                response.success = False
                response.message = f'Failed to start policy: {str(e)}'
                self.get_logger().error(response.message)
        
        return response
    
    def stop_policy_callback(self, request, response):
        """
        Service callback to stop the running LeRobot policy process.
        
        Args:
            request: Empty Trigger request
            response: Trigger response with success status and message
            
        Returns:
            Trigger.Response with success=True if stopped, False if not running
        """
        with self.process_lock:
            # Check if process exists
            if self.process is None:
                response.success = False
                response.message = 'No policy process has been started'
                self.get_logger().warn(response.message)
                return response
            
            # Check if process is still running
            if self.process.poll() is not None:
                exit_code = self.process.poll()
                response.success = False
                response.message = f'Policy already stopped (exit code: {exit_code})'
                self.get_logger().warn(response.message)
                return response
            
            # Stop the process and all its children
            try:
                pid = self.process.pid
                pgid = os.getpgid(pid)
                self.get_logger().info(f'Stopping policy process group (PGID: {pgid}, leader PID: {pid})...')
                
                # Try graceful termination first - kill entire process group
                try:
                    os.killpg(pgid, signal.SIGTERM)
                    self.get_logger().info(f'Sent SIGTERM to process group {pgid}')
                except ProcessLookupError:
                    # Process group already terminated
                    response.success = False
                    response.message = f'Process group {pgid} not found (already terminated)'
                    self.get_logger().warn(response.message)
                    return response
                
                try:
                    # Wait for graceful shutdown
                    self.process.wait(timeout=self.shutdown_timeout)
                    response.success = True
                    response.message = f'Policy stopped gracefully (PID: {pid}, exit code: {self.process.returncode})'
                    self.get_logger().info(response.message)
                    
                except subprocess.TimeoutExpired:
                    # Force kill entire process group if timeout expires
                    self.get_logger().warn(f'Graceful shutdown timeout, force killing process group {pgid}')
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                        self.get_logger().info(f'Sent SIGKILL to process group {pgid}')
                        self.process.wait()
                        response.success = True
                        response.message = f'Policy force-killed after timeout (PID: {pid})'
                        self.get_logger().info(response.message)
                    except ProcessLookupError:
                        # Process group already gone
                        response.success = True
                        response.message = f'Process group {pgid} terminated (no longer found)'
                        self.get_logger().info(response.message)
                    
            except Exception as e:
                response.success = False
                response.message = f'Error stopping policy: {str(e)}'
                self.get_logger().error(response.message)
        
        return response
    
    def get_policy_status_callback(self, request, response):
        """
        Service callback to get the current status of the policy process.
        
        Args:
            request: Empty Trigger request
            response: Trigger response with success status and message
            
        Returns:
            Trigger.Response with current process status
        """
        with self.process_lock:
            if self.process is None:
                response.success = True
                response.message = 'idle (no policy has been started)'
                return response
            
            exit_code = self.process.poll()
            
            if exit_code is None:
                # Process is still running
                response.success = True
                response.message = f'running (PID: {self.process.pid})'
            elif exit_code == 0:
                # Process finished successfully
                response.success = True
                response.message = f'finished successfully (exit code: 0)'
            else:
                # Process finished with error
                response.success = False
                response.message = f'finished with error (exit code: {exit_code})'
        
        return response
    
    def _run_policy_thread(self):
        """
        Run the LeRobot policy command in a separate thread.
        This method blocks until the process completes or is terminated.
        """
        # Build the lerobot-record command
        # Note: This is the hardcoded command as specified
        cmd = f"""bash -c 'source {self.conda_base}/etc/profile.d/conda.sh && \
conda activate {self.conda_env} && \
lerobot-record \
    --robot.id=follower_1 \
    --robot.port=/dev/ttyACM0 \
    --robot.type=so100_follower \
    --dataset.repo_id=zhoumiaosen/eval_policy_act1 \
    --dataset.num_episodes=2 \
    --dataset.single_task="Scoop the poop" \
    --display_data=true \
    --robot.cameras="{{ top: {{\\"type\\": \\"opencv\\", \\"index_or_path\\": 2, \\"width\\": 640, \\"height\\": 480, \\"fps\\": 30}}, left: {{\\"type\\": \\"opencv\\", \\"index_or_path\\": 0, \\"width\\": 640, \\"height\\": 480, \\"fps\\": 30}} }}" \
    --policy.device=cuda \
    --policy.path=ishandotsh/poopascoopa_act \
    --dataset.fps=30 \
    --dataset.episode_time_s=120'
"""
        
        self.get_logger().info('Starting LeRobot policy process...')
        self.get_logger().info(f'Conda environment: {self.conda_env}')
        self.get_logger().info(f'Conda base path: {self.conda_base}')
        
        try:
            # Start the process with new session to allow process group killing
            self.process = subprocess.Popen(
                cmd,
                shell=True,
                start_new_session=True,  # Create new process group for proper cleanup
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # Line buffered
                universal_newlines=True
            )
            
            pgid = os.getpgid(self.process.pid)
            self.get_logger().info(f'LeRobot policy started with PID: {self.process.pid}, PGID: {pgid}')
            
            # Stream output to ROS logs
            # Note: This will block until process completes
            for line in iter(self.process.stdout.readline, ''):
                if line:
                    self.get_logger().info(f'[LeRobot] {line.rstrip()}')
            
            # Wait for process to complete
            self.process.wait()
            
            # Log completion
            exit_code = self.process.returncode
            if exit_code == 0:
                self.get_logger().info(f'LeRobot policy finished successfully')
            elif exit_code == -15:  # SIGTERM
                self.get_logger().info(f'LeRobot policy terminated by user')
            elif exit_code == -9:  # SIGKILL
                self.get_logger().warn(f'LeRobot policy was force-killed')
            else:
                self.get_logger().error(f'LeRobot policy exited with code {exit_code}')
                
                # Read and log stderr if there was an error
                if self.process.stderr:
                    stderr_output = self.process.stderr.read()
                    if stderr_output:
                        self.get_logger().error(f'Error output: {stderr_output[:1000]}')
                        
        except Exception as e:
            self.get_logger().error(f'Exception running LeRobot policy: {str(e)}')
    
    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        Ensures any running policy process is stopped.
        """
        self.get_logger().info('Shutting down Policy Controller Node...')
        
        # Stop any running process and its children
        with self.process_lock:
            if self.process is not None and self.process.poll() is None:
                try:
                    pid = self.process.pid
                    pgid = os.getpgid(pid)
                    self.get_logger().info(f'Stopping running policy process group (PGID: {pgid})...')
                    
                    # Try graceful termination of entire process group
                    try:
                        os.killpg(pgid, signal.SIGTERM)
                        self.get_logger().info(f'Sent SIGTERM to process group {pgid}')
                    except ProcessLookupError:
                        self.get_logger().info('Process group already terminated')
                    
                    try:
                        self.process.wait(timeout=self.shutdown_timeout)
                    except subprocess.TimeoutExpired:
                        self.get_logger().warn(f'Force killing process group {pgid}')
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                            self.get_logger().info(f'Sent SIGKILL to process group {pgid}')
                            self.process.wait()
                        except ProcessLookupError:
                            self.get_logger().info('Process group already terminated')
                except Exception as e:
                    self.get_logger().error(f'Error during cleanup: {str(e)}')
        
        super().destroy_node()


def main(args=None):
    """
    Main entry point for the policy controller node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    try:
        node = PolicyControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in policy controller node: {e}')
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
