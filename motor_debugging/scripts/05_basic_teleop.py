#!/usr/bin/env python3
"""
Basic Teleoperation for LeKiwi Mobile Base

Control the mobile base using keyboard:
- W/S: Forward/Backward
- A/D: Rotate Left/Right
- Q/E: Strafe Left/Right
- SPACE: Stop
- ESC: Exit
"""

import argparse
import sys
import time
from rich.console import Console
from rich.panel import Panel
from rich.live import Live
from rich.table import Table

console = Console()


class LeKiwiTeleop:
    """Teleoperation controller for LeKiwi mobile base."""
    
    def __init__(self, port, motor_ids=[7, 8, 9]):
        self.port = port
        self.motor_ids = motor_ids
        self.running = False
        self.current_command = "STOP"
        
    def connect(self):
        """Connect to motors."""
        console.print("[yellow]Connecting to motors...[/yellow]")
        # Placeholder for actual motor connection
        time.sleep(1)
        console.print("[green]✓ Connected to motors[/green]")
        return True
    
    def stop(self):
        """Stop all motors."""
        self.current_command = "STOP"
        # Placeholder for actual motor stop command
        # In real implementation:
        # for motor_id in self.motor_ids:
        #     motor.set_velocity(motor_id, 0)
    
    def move_forward(self, speed=100):
        """Move forward."""
        self.current_command = f"FORWARD (speed: {speed})"
        # Placeholder for actual motor commands
    
    def move_backward(self, speed=100):
        """Move backward."""
        self.current_command = f"BACKWARD (speed: {speed})"
        # Placeholder for actual motor commands
    
    def rotate_left(self, speed=100):
        """Rotate left."""
        self.current_command = f"ROTATE LEFT (speed: {speed})"
        # Placeholder for actual motor commands
    
    def rotate_right(self, speed=100):
        """Rotate right."""
        self.current_command = f"ROTATE RIGHT (speed: {speed})"
        # Placeholder for actual motor commands
    
    def strafe_left(self, speed=100):
        """Strafe left (omnidirectional movement)."""
        self.current_command = f"STRAFE LEFT (speed: {speed})"
        # Placeholder for actual motor commands
    
    def strafe_right(self, speed=100):
        """Strafe right (omnidirectional movement)."""
        self.current_command = f"STRAFE RIGHT (speed: {speed})"
        # Placeholder for actual motor commands
    
    def disconnect(self):
        """Disconnect from motors."""
        self.stop()
        console.print("[yellow]Disconnecting...[/yellow]")
        time.sleep(0.5)
        console.print("[green]✓ Disconnected[/green]")


def create_status_display(teleop):
    """Create status display table."""
    table = Table(show_header=False, box=None, padding=(0, 2))
    table.add_column("Key", style="cyan", width=20)
    table.add_column("Value", style="green")
    
    table.add_row("Current Command:", teleop.current_command)
    table.add_row("Port:", teleop.port)
    table.add_row("Motors:", str(teleop.motor_ids))
    table.add_row("Status:", "[green]RUNNING[/green]" if teleop.running else "[yellow]STOPPED[/yellow]")
    
    return Panel(table, title="[bold cyan]LeKiwi Teleoperation[/bold cyan]", border_style="cyan")


def print_controls():
    """Print control instructions."""
    controls = Table(title="Keyboard Controls", show_header=True, header_style="bold magenta")
    controls.add_column("Key", style="cyan", width=15)
    controls.add_column("Action", style="green", width=30)
    
    controls.add_row("W", "Move Forward")
    controls.add_row("S", "Move Backward")
    controls.add_row("A", "Rotate Left")
    controls.add_row("D", "Rotate Right")
    controls.add_row("Q", "Strafe Left")
    controls.add_row("E", "Strafe Right")
    controls.add_row("SPACE", "Stop")
    controls.add_row("ESC / Ctrl+C", "Exit")
    
    console.print(controls)


def main():
    parser = argparse.ArgumentParser(description="LeKiwi basic teleoperation")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--motor_ids", nargs='+', type=int, default=[7, 8, 9],
                       help="Motor IDs (default: 7 8 9)")
    parser.add_argument("--speed", type=int, default=100, help="Default speed (0-1000)")
    args = parser.parse_args()
    
    console.print(Panel.fit(
        "[bold cyan]LeKiwi Basic Teleoperation[/bold cyan]",
        subtitle="Keyboard Control"
    ))
    
    print_controls()
    
    console.print("\n[bold yellow]⚠ Safety Notice:[/bold yellow]")
    console.print("  • Ensure the robot has clear space to move")
    console.print("  • Keep emergency stop accessible")
    console.print("  • Monitor battery level")
    console.print("  • Stop immediately if you hear unusual sounds")
    
    response = input("\n[bold]Ready to start teleoperation? (yes/no):[/bold] ").strip().lower()
    if response not in ['yes', 'y']:
        console.print("[yellow]Teleoperation cancelled[/yellow]")
        return
    
    # Initialize teleop
    teleop = LeKiwiTeleop(args.port, args.motor_ids)
    
    if not teleop.connect():
        console.print("[bold red]✗ Failed to connect to motors[/bold red]")
        sys.exit(1)
    
    teleop.running = True
    
    console.print("\n[green]✓ Teleoperation active![/green]")
    console.print("[yellow]Press keys to control. ESC or Ctrl+C to exit.[/yellow]\n")
    
    try:
        # Try to use pynput for keyboard control
        from pynput import keyboard
        
        def on_press(key):
            """Handle key press events."""
            try:
                if hasattr(key, 'char'):
                    if key.char == 'w':
                        teleop.move_forward(args.speed)
                    elif key.char == 's':
                        teleop.move_backward(args.speed)
                    elif key.char == 'a':
                        teleop.rotate_left(args.speed)
                    elif key.char == 'd':
                        teleop.rotate_right(args.speed)
                    elif key.char == 'q':
                        teleop.strafe_left(args.speed)
                    elif key.char == 'e':
                        teleop.strafe_right(args.speed)
                elif key == keyboard.Key.space:
                    teleop.stop()
                elif key == keyboard.Key.esc:
                    teleop.running = False
                    return False
            except AttributeError:
                pass
        
        def on_release(key):
            """Handle key release events."""
            # Auto-stop on key release for safety
            teleop.stop()
        
        # Start keyboard listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        
        # Status display loop
        with Live(create_status_display(teleop), refresh_per_second=10, console=console) as live:
            while teleop.running:
                live.update(create_status_display(teleop))
                time.sleep(0.1)
        
        listener.stop()
        
    except ImportError:
        console.print("[yellow]pynput not available, using simple input mode[/yellow]")
        console.print("Enter commands: w/s/a/d/q/e/space/exit\n")
        
        # Simple input-based control
        while teleop.running:
            console.print(create_status_display(teleop))
            cmd = input("\nCommand: ").strip().lower()
            
            if cmd == 'w':
                teleop.move_forward(args.speed)
            elif cmd == 's':
                teleop.move_backward(args.speed)
            elif cmd == 'a':
                teleop.rotate_left(args.speed)
            elif cmd == 'd':
                teleop.rotate_right(args.speed)
            elif cmd == 'q':
                teleop.strafe_left(args.speed)
            elif cmd == 'e':
                teleop.strafe_right(args.speed)
            elif cmd == 'space' or cmd == '':
                teleop.stop()
            elif cmd in ['exit', 'quit', 'esc']:
                break
            else:
                console.print("[yellow]Unknown command[/yellow]")
    
    finally:
        teleop.disconnect()
        console.print("\n[bold green]✓ Teleoperation ended[/bold green]")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n\n[yellow]Teleoperation interrupted[/yellow]")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[bold red]Error: {e}[/bold red]")
        import traceback
        console.print(traceback.format_exc())
        sys.exit(1)

