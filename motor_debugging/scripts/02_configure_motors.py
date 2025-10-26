#!/usr/bin/env python3
"""
Configure Motor IDs for LeKiwi Mobile Base

This script helps configure motor IDs for the Feetech STS3215 servos.
Connect one motor at a time and assign unique IDs.
"""

import argparse
import sys
import yaml
from pathlib import Path
from rich.console import Console
from rich.panel import Panel
from rich.prompt import Confirm, IntPrompt

console = Console()


def load_config():
    """Load configuration from YAML file."""
    config_path = Path(__file__).parent.parent / "config" / "lekiwi_config.yaml"
    if config_path.exists():
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    return None


def configure_motor(port, motor_id, brand="feetech", model="sts3215", baudrate=1000000):
    """Configure a single motor with specified ID."""
    try:
        # Import lerobot configuration script functionality
        # Note: This assumes lerobot is installed
        import subprocess
        
        cmd = [
            "python", "-m", "lerobot.scripts.configure_motor",
            "--port", port,
            "--brand", brand,
            "--model", model,
            "--baudrate", str(baudrate),
            "--ID", str(motor_id)
        ]
        
        console.print(f"\n[yellow]Configuring motor with ID {motor_id}...[/yellow]")
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode == 0:
            console.print(f"[bold green]✓ Motor {motor_id} configured successfully![/bold green]")
            return True
        else:
            console.print(f"[bold red]✗ Failed to configure motor {motor_id}[/bold red]")
            console.print(f"Error: {result.stderr}")
            return False
            
    except ImportError:
        console.print("[bold red]Error: lerobot library not found[/bold red]")
        console.print("Please install it with: pip install 'lerobot[feetech]'")
        return False
    except Exception as e:
        console.print(f"[bold red]Error: {e}[/bold red]")
        return False


def main():
    parser = argparse.ArgumentParser(description="Configure LeKiwi motor IDs")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--auto", action="store_true", help="Auto-configure all wheel motors (7, 8, 9)")
    parser.add_argument("--motor_id", type=int, help="Specific motor ID to configure")
    args = parser.parse_args()
    
    # Load configuration
    config = load_config()
    
    console.print(Panel.fit(
        "[bold cyan]LeKiwi Motor ID Configuration[/bold cyan]",
        subtitle="Step 2: Set Motor IDs"
    ))
    
    console.print(f"\n[bold]Port:[/bold] {args.port}")
    console.print(f"[bold]Motor Model:[/bold] Feetech STS3215")
    console.print(f"[bold]Baudrate:[/bold] 1,000,000")
    
    if args.auto:
        # Auto-configure wheel motors
        console.print("\n[bold yellow]⚠ Auto-configuration mode[/bold yellow]")
        console.print("This will configure motors with IDs: 7, 8, 9")
        console.print("\n[bold red]IMPORTANT:[/bold red]")
        console.print("  1. Connect ONLY ONE motor at a time to the bus")
        console.print("  2. You will be prompted to connect each motor")
        console.print("  3. Wait for confirmation before connecting the next motor")
        
        if not Confirm.ask("\nReady to start?"):
            console.print("[yellow]Configuration cancelled[/yellow]")
            return
        
        wheel_ids = [7, 8, 9]
        for i, motor_id in enumerate(wheel_ids, 1):
            console.print(f"\n{'='*60}")
            console.print(f"[bold]Motor {i} of 3[/bold]")
            console.print(f"{'='*60}")
            console.print(f"\n1. Connect motor for ID {motor_id} to the motor bus")
            console.print("2. Ensure ONLY this motor is connected")
            
            if not Confirm.ask(f"\nMotor connected and ready to configure as ID {motor_id}?"):
                console.print("[yellow]Skipping this motor[/yellow]")
                continue
            
            success = configure_motor(args.port, motor_id)
            
            if success:
                console.print(f"\n[green]✓ Motor {motor_id} configured[/green]")
                if i < len(wheel_ids):
                    console.print("\n[bold]Next steps:[/bold]")
                    console.print(f"  1. Disconnect motor {motor_id}")
                    console.print(f"  2. Connect motor for ID {wheel_ids[i]}")
                    input("\nPress Enter when ready to continue...")
            else:
                console.print(f"\n[red]✗ Failed to configure motor {motor_id}[/red]")
                if not Confirm.ask("Continue with next motor?"):
                    break
        
        console.print("\n[bold green]Configuration complete![/bold green]")
        console.print("\n[bold]Next step:[/bold]")
        console.print(f"  python scripts/03_test_single_motor.py --port {args.port} --motor_id 7")
        
    elif args.motor_id:
        # Configure single motor
        console.print(f"\n[bold]Configuring single motor with ID: {args.motor_id}[/bold]")
        console.print("\nEnsure ONLY ONE motor is connected to the bus!")
        
        if Confirm.ask("\nMotor connected and ready?"):
            success = configure_motor(args.port, args.motor_id)
            if success:
                console.print(f"\n[bold]Next step:[/bold]")
                console.print(f"  python scripts/03_test_single_motor.py --port {args.port} --motor_id {args.motor_id}")
    else:
        # Interactive mode
        console.print("\n[bold]Interactive Configuration Mode[/bold]")
        console.print("\nRecommended motor IDs for LeKiwi wheels: 7, 8, 9")
        
        if Confirm.ask("\nConfigure motor now?"):
            motor_id = IntPrompt.ask("Enter motor ID (1-253)", default=7)
            console.print(f"\n[yellow]Connect ONLY the motor you want to set as ID {motor_id}[/yellow]")
            
            if Confirm.ask("Motor connected and ready?"):
                success = configure_motor(args.port, motor_id)
                if success and Confirm.ask("\nConfigure another motor?"):
                    # Recursive call for additional motors
                    console.print("\nRestart the script to configure another motor")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n\n[yellow]Configuration interrupted[/yellow]")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[bold red]Error: {e}[/bold red]")
        import traceback
        console.print(traceback.format_exc())
        sys.exit(1)

