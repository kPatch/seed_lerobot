#!/usr/bin/env python3
"""
Motor Interface Module

This module provides a simplified interface to control Feetech motors
using the lerobot FeetechMotorsBus.
"""

from typing import Optional
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class FeetechMotorInterface:
    """Simple interface for controlling individual Feetech STS3215 motors."""
    
    # Control table addresses for STS3215
    ADDR_ID = 5
    ADDR_BAUD_RATE = 6
    ADDR_RETURN_DELAY_TIME = 7
    ADDR_RESPONSE_STATUS_LEVEL = 8
    ADDR_MIN_ANGLE_LIMIT = 9
    ADDR_MAX_ANGLE_LIMIT = 11
    ADDR_MAX_TEMPERATURE_LIMIT = 13
    ADDR_MAX_VOLTAGE_LIMIT = 14
    ADDR_MIN_VOLTAGE_LIMIT = 15
    ADDR_MAX_TORQUE = 16
    ADDR_UNLOADING_CONDITION = 18
    ADDR_LED_ALARM_CONDITION = 19
    ADDR_P_COEFFICIENT = 21
    ADDR_D_COEFFICIENT = 22
    ADDR_I_COEFFICIENT = 23
    ADDR_MINIMUM_STARTUP_FORCE = 24
    ADDR_CW_DEAD_ZONE = 26
    ADDR_CCW_DEAD_ZONE = 27
    ADDR_PROTECTION_CURRENT = 28
    ADDR_ANGULAR_RESOLUTION = 30
    ADDR_OFFSET = 31
    ADDR_OPERATION_MODE = 33
    ADDR_PROTECTION_TORQUE = 34
    ADDR_PROTECTION_TIME = 36
    ADDR_OVERLOAD_TORQUE = 37
    ADDR_SPEED_CLOSED_LOOP_P_PROPORTIONAL_COEF = 38
    ADDR_OVERCURRENT_PROTECTION_TIME = 39
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACCELERATION = 41
    ADDR_GOAL_POSITION = 42
    ADDR_RUNNING_TIME = 44
    ADDR_RUNNING_SPEED = 46
    ADDR_TORQUE_LIMIT = 48
    ADDR_LOCK = 55
    ADDR_PRESENT_POSITION = 56
    ADDR_PRESENT_SPEED = 58
    ADDR_PRESENT_LOAD = 60
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMPERATURE = 63
    ADDR_ASYNC_WRITE_FLAG = 64
    ADDR_SERVO_STATUS = 65
    ADDR_MOVING_STATUS = 66
    ADDR_PRESENT_CURRENT = 69
    
    # Protocol and communication settings
    PROTOCOL_VERSION = 0
    DEFAULT_BAUDRATE = 1000000
    
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        """
        Initialize motor interface.
        
        Args:
            port: Serial port (e.g., /dev/ttyACM0)
            baudrate: Communication baudrate (default: 1000000)
        """
        self.port = port
        self.baudrate = baudrate
        
        # Initialize SDK handlers
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)
        
        self.connected = False
    
    def connect(self) -> bool:
        """
        Open serial port and establish connection.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            if self.port_handler.openPort():
                if self.port_handler.setBaudRate(self.baudrate):
                    self.connected = True
                    return True
            return False
        except Exception as e:
            print(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Close serial port connection."""
        if self.connected:
            self.port_handler.closePort()
            self.connected = False
    
    def ping(self, motor_id: int) -> bool:
        """
        Ping a motor to check if it's responding.
        
        Args:
            motor_id: Motor ID (1-253)
            
        Returns:
            True if motor responds, False otherwise
        """
        if not self.connected:
            return False
        
        model_number, result, error = self.packet_handler.ping(self.port_handler, motor_id)
        return result == COMM_SUCCESS
    
    def unlock(self, motor_id: int) -> bool:
        """
        Unlock EEPROM area for writing (allows changing configuration).
        
        Args:
            motor_id: Motor ID
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_LOCK, 0
        )
        return result == COMM_SUCCESS
    
    def set_operation_mode(self, motor_id: int, mode: int = 0) -> bool:
        """
        Set motor operation mode.
        
        Args:
            motor_id: Motor ID
            mode: 0 = position control (servo mode), 1 = continuous rotation
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        # Unlock first
        self.unlock(motor_id)
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_OPERATION_MODE, mode
        )
        return result == COMM_SUCCESS
    
    def enable_torque(self, motor_id: int) -> bool:
        """
        Enable motor torque (makes motor hold position and respond to commands).
        
        Args:
            motor_id: Motor ID
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1
        )
        return result == COMM_SUCCESS
    
    def disable_torque(self, motor_id: int) -> bool:
        """
        Disable motor torque (makes motor limp/free to move).
        
        Args:
            motor_id: Motor ID
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0
        )
        return result == COMM_SUCCESS
    
    def read_position(self, motor_id: int) -> Optional[int]:
        """
        Read current motor position.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Position value (0-4095) or None if read fails
        """
        if not self.connected:
            return None
        
        position, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_POSITION
        )
        
        if result == COMM_SUCCESS:
            return position
        return None
    
    def write_position(self, motor_id: int, position: int) -> bool:
        """
        Write goal position to motor.
        
        Args:
            motor_id: Motor ID
            position: Target position (0-4095)
            
        Returns:
            True if successful
        """
        if not self.connected:
            return False
        
        # Clamp position to valid range
        position = max(0, min(4095, position))
        
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_GOAL_POSITION, position
        )
        return result == COMM_SUCCESS
    
    def read_temperature(self, motor_id: int) -> Optional[int]:
        """Read motor temperature in Celsius."""
        if not self.connected:
            return None
        
        temp, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_TEMPERATURE
        )
        
        if result == COMM_SUCCESS:
            return temp
        return None
    
    def read_voltage(self, motor_id: int) -> Optional[float]:
        """Read motor voltage in Volts."""
        if not self.connected:
            return None
        
        voltage_raw, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_VOLTAGE
        )
        
        if result == COMM_SUCCESS:
            return voltage_raw / 10.0  # Convert to actual voltage
        return None
    
    def read_current(self, motor_id: int) -> Optional[int]:
        """Read motor current in mA."""
        if not self.connected:
            return None
        
        current, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_CURRENT
        )
        
        if result == COMM_SUCCESS:
            return current
        return None
    
    def read_load(self, motor_id: int) -> Optional[int]:
        """Read motor load percentage."""
        if not self.connected:
            return None
        
        load_raw, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_LOAD
        )
        
        if result == COMM_SUCCESS:
            # Convert to percentage (interpretation may vary)
            load_percent = (load_raw & 0x3FF) / 10.0  # Lower 10 bits
            return int(load_percent)
        return None
    
    def read_speed(self, motor_id: int) -> Optional[int]:
        """Read motor speed."""
        if not self.connected:
            return None
        
        speed, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_SPEED
        )
        
        if result == COMM_SUCCESS:
            return speed
        return None
    
    def read_all_data(self, motor_id: int) -> Optional[dict]:
        """
        Read all available motor data.
        
        Args:
            motor_id: Motor ID
            
        Returns:
            Dictionary with motor state or None if connection fails
        """
        if not self.connected:
            return None
        
        return {
            "position": self.read_position(motor_id) or 0,
            "temperature": self.read_temperature(motor_id) or 0,
            "voltage": self.read_voltage(motor_id) or 0.0,
            "current": self.read_current(motor_id) or 0,
            "load": self.read_load(motor_id) or 0,
            "speed": self.read_speed(motor_id) or 0
        }
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()

